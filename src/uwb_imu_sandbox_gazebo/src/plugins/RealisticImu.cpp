// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// RealisticImu: IMU on a link, driven by the exact Gazebo motion of that link
// and the measured error model (sealed imu_model.bin).
//
//   <plugin filename="RealisticImu" name="uwb_imu_sandbox::RealisticImu">
//     <link_name>imu_link</link_name>
//     <topic>imu/raw</topic>
//     <frame_id>base_link</frame_id>
//     <seed>-1</seed>                       -1: new random bias every start
//     <bias_reseed_period>600</bias_reseed_period>   0: keep the turn-on bias
//   </plugin>
//
// It publishes gz.msgs.IMU (bridge it to sensor_msgs/Imu) and, on
// <truth_topic>, the exact values without any error, for evaluation.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/transport/Node.hh>

#include "common.hpp"
#include "uwb_imu_sandbox_gazebo/imu_model.hpp"

namespace uwb_imu_sandbox
{

class RealisticImu : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "[RealisticImu] must be attached to a model\n";
      return;
    }
    const std::string link_name = sdfGet<std::string>(sdf, "link_name", "");
    link_ = gz::sim::Link(
      link_name.empty() ? model.CanonicalLink(ecm) : model.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      gzerr << "[RealisticImu] link '" << link_name << "' not found\n";
      return;
    }
    link_.EnableVelocityChecks(ecm, true);
    offset_ = sdfGet<gz::math::Pose3d>(sdf, "imu_offset", gz::math::Pose3d::Zero);

    ImuOptions opt;
    opt.enable_noise = sdfGet<bool>(sdf, "enable_noise", true);
    opt.enable_bias = sdfGet<bool>(sdf, "enable_bias", true);
    opt.enable_quantisation = sdfGet<bool>(sdf, "enable_quantisation", true);
    opt.enable_sample_hold = sdfGet<bool>(sdf, "enable_sample_hold", true);
    opt.bias_reseed_period = sdfGet<double>(sdf, "bias_reseed_period", 600.0);
    opt.bias_transition_time = sdfGet<double>(sdf, "bias_transition_time", 5.0);
    opt.bias_scale = sdfGet<double>(sdf, "bias_scale", 1.0);
    opt.noise_scale = sdfGet<double>(sdf, "noise_scale", 1.0);
    opt.publish_period = 1.0 / std::max(1.0, sdfGet<double>(sdf, "update_rate", 100.0));

    const std::string spec = sdfGet<std::string>(sdf, "model_file",
        "model://uwb_imu_sandbox_data/imu_model.bin");
    const std::string path = resolveModelFile(spec, sdf);
    if (path.empty()) {
      gzerr << "[RealisticImu] cannot find IMU model file '" << spec
            << "' (check GZ_SIM_RESOURCE_PATH)\n";
      return;
    }
    try {
      model_ = std::make_unique<ImuErrorModel>(path, opt, seedFrom(sdfGet<int>(sdf, "seed", -1)));
    } catch (const std::exception & e) {
      gzerr << "[RealisticImu] " << e.what() << "\n";
      return;
    }
    frame_id_ = sdfGet<std::string>(sdf, "frame_id", "base_link");
    topic_ = sdfGet<std::string>(sdf, "topic", "imu/raw");
    truth_topic_ = sdfGet<std::string>(sdf, "truth_topic", "imu/truth");
    pub_ = node_.Advertise<gz::msgs::IMU>(topic_);
    pub_truth_ = node_.Advertise<gz::msgs::IMU>(truth_topic_);

    const auto world = gz::sim::worldEntity(ecm);
    if (auto g = ecm.Component<gz::sim::components::Gravity>(world)) {
      gravity_ = toVec(g->Data());
    }
    gzmsg << "[RealisticImu] on link " << link_name << " -> " << topic_
          << " (model " << path << ")\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm)
  override
  {
    if (!model_ || info.paused) {return;}
    const double t = std::chrono::duration<double>(info.simTime).count();
    const auto pose = link_.WorldPose(ecm);
    const auto v = link_.WorldLinearVelocity(ecm);
    const auto w = link_.WorldAngularVelocity(ecm);
    if (!pose || !v || !w) {return;}
    const gz::math::Pose3d imu_pose = *pose * offset_;
    // velocity of the IMU point (the offset matters when the link turns)
    const gz::math::Vector3d r = pose->Rot().RotateVector(offset_.Pos());
    const gz::math::Vector3d v_imu = *v + w->Cross(r);

    ImuTruth tr;
    tr.world_lin_vel = toVec(v_imu);
    tr.orientation = toQuat(imu_pose.Rot());
    tr.body_ang_vel = toVec(imu_pose.Rot().RotateVectorReverse(*w));
    tr.gravity = gravity_;

    ImuSample s;
    if (!model_->update(t, tr, s)) {return;}

    gz::msgs::IMU msg;
    fill(msg, t, s.orientation, s.gyro, s.accel);
    pub_.Publish(msg);

    // exact values at the same instant (no bias, noise, hold or resolution)
    if (have_prev_ && t - t_prev_ > 1e-6) {
      const Vec3 a_world = (tr.world_lin_vel - v_prev_) * (1.0 / (t - t_prev_));
      const Vec3 f = tr.orientation.inverseRotate(a_world - tr.gravity);
      gz::msgs::IMU truth;
      fill(truth, t, tr.orientation, tr.body_ang_vel, f);
      pub_truth_.Publish(truth);
    }
    v_prev_ = tr.world_lin_vel;
    t_prev_ = t;
    have_prev_ = true;
  }

private:
  void fill(gz::msgs::IMU & m, double t, const Quat & q, const Vec3 & g, const Vec3 & a) const
  {
    const int sec = static_cast<int>(t);
    m.mutable_header()->mutable_stamp()->set_sec(sec);
    m.mutable_header()->mutable_stamp()->set_nsec(static_cast<int>((t - sec) * 1e9));
    auto * d = m.mutable_header()->add_data();
    d->set_key("frame_id");
    d->add_value(frame_id_);
    m.set_entity_name(frame_id_);
    m.mutable_orientation()->set_w(q.w);
    m.mutable_orientation()->set_x(q.x);
    m.mutable_orientation()->set_y(q.y);
    m.mutable_orientation()->set_z(q.z);
    m.mutable_angular_velocity()->set_x(g.x);
    m.mutable_angular_velocity()->set_y(g.y);
    m.mutable_angular_velocity()->set_z(g.z);
    m.mutable_linear_acceleration()->set_x(a.x);
    m.mutable_linear_acceleration()->set_y(a.y);
    m.mutable_linear_acceleration()->set_z(a.z);
  }

  gz::sim::Link link_;
  gz::math::Pose3d offset_;
  std::unique_ptr<ImuErrorModel> model_;
  Vec3 gravity_{0, 0, -9.80665};
  std::string frame_id_, topic_, truth_topic_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_, pub_truth_;
  bool have_prev_{false};
  double t_prev_{0};
  Vec3 v_prev_;
};

}  // namespace uwb_imu_sandbox

GZ_ADD_PLUGIN(
  uwb_imu_sandbox::RealisticImu, gz::sim::System,
  uwb_imu_sandbox::RealisticImu::ISystemConfigure,
  uwb_imu_sandbox::RealisticImu::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(uwb_imu_sandbox::RealisticImu, "uwb_imu_sandbox::RealisticImu")
