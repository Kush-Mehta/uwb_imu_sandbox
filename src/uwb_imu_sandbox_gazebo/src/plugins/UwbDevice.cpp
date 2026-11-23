// Copyright 2026 IITGN Robotics
// SPDX-License-Identifier: Apache-2.0
//
// UwbDevice: one Gazebo system plugin for both UWB roles.
//
//   <plugin filename="UwbDevice" name="uwb_imu_sandbox::UwbDevice">
//     <role>anchor</role>        anchor | tag
//     <id>1</id>
//     ...
//   </plugin>
//
// Anchors only announce where they are.  A tag ranges to every anchor in its
// network one after the other, with the slot timing of the real tag, and
// publishes each result as the same text line the real tag prints on its
// serial port:
//
//   [network_count,anchor_id,range_m,rx,fp,quality,delta]
//
// range is in metres; rx, fp, quality and delta are on a unitless 0..100
// scale (normalised signal fields).  For each slot it also publishes a JSON
// ground-truth line (true distance, LOS/NLOS, obstacle depth, outcome).

#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

#include "common.hpp"
#include "uwb_imu_sandbox_gazebo/uwb_model.hpp"

namespace uwb_imu_sandbox
{

// ---------------------------------------------------------------------------
// anchors known to this simulation (shared by all UwbDevice instances)
// ---------------------------------------------------------------------------
struct AnchorEntry
{
  int id{0};
  gz::sim::Entity model{gz::sim::kNullEntity};
  gz::sim::Entity link{gz::sim::kNullEntity};
  gz::math::Pose3d offset;
};

class AnchorRegistry
{
public:
  static AnchorRegistry & instance()
  {
    static AnchorRegistry r;
    return r;
  }
  bool add(const AnchorEntry & e)
  {
    std::lock_guard<std::mutex> l(m_);
    for (const auto & [k, v] : map_) {
      if (v.id == e.id && k != e.model) {return false;}
    }
    map_[e.model] = e;
    return true;
  }
  void remove(gz::sim::Entity model)
  {
    std::lock_guard<std::mutex> l(m_);
    map_.erase(model);
  }
  std::vector<AnchorEntry> list()
  {
    std::lock_guard<std::mutex> l(m_);
    std::vector<AnchorEntry> v;
    for (const auto & kv : map_) {v.push_back(kv.second);}
    std::sort(v.begin(), v.end(), [](const auto & a, const auto & b) {return a.id < b.id;});
    return v;
  }

private:
  std::mutex m_;
  std::map<gz::sim::Entity, AnchorEntry> map_;
};

// ---------------------------------------------------------------------------
class UwbDevice : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  ~UwbDevice() override
  {
    if (role_ == Role::kAnchor) {AnchorRegistry::instance().remove(model_);}
  }

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = entity;
    gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "[UwbDevice] must be attached to a model\n";
      return;
    }
    const std::string role = sdfGet<std::string>(sdf, "role", "tag");
    id_ = sdfGet<int>(sdf, "id", 1);
    const std::string link_name = sdfGet<std::string>(sdf, "link_name", "");
    link_ = link_name.empty() ? model.CanonicalLink(ecm) : model.LinkByName(ecm, link_name);
    if (link_ == gz::sim::kNullEntity) {
      gzerr << "[UwbDevice] link '" << link_name << "' not found in model "
            << model.Name(ecm) << "\n";
      return;
    }
    offset_ = sdfGet<gz::math::Pose3d>(sdf, "antenna_offset", gz::math::Pose3d::Zero);

    if (role == "anchor") {
      role_ = Role::kAnchor;
      if (!AnchorRegistry::instance().add({id_, model_, link_, offset_})) {
        gzerr << "[UwbDevice] anchor id " << id_ << " is used twice; ignoring "
              << model.Name(ecm) << "\n";
      } else {
        gzmsg << "[UwbDevice] anchor " << id_ << " on " << model.Name(ecm) << "\n";
      }
      return;
    }
    if (role != "tag") {
      gzerr << "[UwbDevice] <role> must be 'anchor' or 'tag', got '" << role << "'\n";
      return;
    }
    role_ = Role::kTag;

    const std::string spec = sdfGet<std::string>(sdf, "model_file",
        "model://uwb_imu_sandbox_data/uwb_model.bin");
    const std::string path = resolveModelFile(spec, sdf);
    if (path.empty()) {
      gzerr << "[UwbDevice] cannot find UWB model file '" << spec
            << "' (check GZ_SIM_RESOURCE_PATH)\n";
      role_ = Role::kNone;
      return;
    }
    try {
      model_data_ = std::make_unique<UwbChannelModel>(path);
    } catch (const std::exception & e) {
      gzerr << "[UwbDevice] " << e.what() << "\n";
      role_ = Role::kNone;
      return;
    }
    model_data_->setMaxRange(sdfGet<double>(sdf, "max_range", 12.0));
    model_data_->setMaxMaterial(sdfGet<double>(sdf, "max_material", 1.0));
    model_data_->setSpatialCell(sdfGet<double>(sdf, "nlos_cell_size", 0.25));
    model_data_->setDelaySpread(sdfGet<double>(sdf, "antenna_delay_spread", 0.03));
    model_data_->setDelayOffset(sdfGet<double>(sdf, "antenna_delay_offset", 0.0));
    model_data_->setMultipathSeed(
      0x9E3779B97F4A7C15ULL ^ static_cast<uint64_t>(sdfGet<int>(sdf, "multipath_seed", 0)));
    join_delay_ = sdfGet<double>(sdf, "join_delay", 0.5);
    leave_delay_ = sdfGet<double>(sdf, "leave_delay", 1.0);
    rng_.seed(seedFrom(sdfGet<int>(sdf, "seed", -1)));

    const std::string prefix = "uwb/tag" + std::to_string(id_);
    topic_raw_ = sdfGet<std::string>(sdf, "topic", prefix + "/serial");
    topic_truth_ = sdfGet<std::string>(sdf, "truth_topic", prefix + "/truth");
    pub_raw_ = node_.Advertise<gz::msgs::StringMsg>(topic_raw_);
    pub_truth_ = node_.Advertise<gz::msgs::StringMsg>(topic_truth_);
    gzmsg << "[UwbDevice] tag " << id_ << " on " << model.Name(ecm) << " -> " << topic_raw_
          << " (model " << path << ", max range " << model_data_->maxRange() << " m)\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm)
  override
  {
    if (role_ != Role::kTag || info.paused) {return;}
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (!started_) {
      started_ = true;
      // power-up phase of the ranging loop
      next_slot_ = t + std::uniform_real_distribution<double>(0.0, 0.1)(rng_);
      refreshObstacles(ecm);
      last_refresh_ = t;
    }
    if (t - last_refresh_ > 1.0) {
      refreshObstacles(ecm);
      last_refresh_ = t;
    }
    int guard = 0;
    while (next_slot_ <= t && guard++ < 64) {
      runSlot(next_slot_, ecm);
    }
    if (guard >= 64) {
      // the simulation jumped far ahead (e.g. a reset); restart the loop
      next_slot_ = t + 0.05;
    }
  }

private:
  enum class Role { kNone, kAnchor, kTag };

  struct Cached
  {
    gz::sim::Entity collision;
    gz::sim::Entity owner;
    Obstacle shape;
  };

  struct Member
  {
    bool in_network{false};
    double seen_since{-1};
    double lost_since{-1};
    UwbLinkState link;
  };

  // all collision shapes in the world we can intersect
  void refreshObstacles(const gz::sim::EntityComponentManager & ecm)
  {
    cache_.clear();
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity & e, const gz::sim::components::Collision *,
      const gz::sim::components::Geometry * geom) -> bool
      {
        const sdf::Geometry & g = geom->Data();
        Obstacle o;
        switch (g.Type()) {
          case sdf::GeometryType::BOX:
            o.type = ShapeType::kBox;
            o.size = toVec(g.BoxShape()->Size());
            break;
          case sdf::GeometryType::CYLINDER:
            o.type = ShapeType::kCylinder;
            o.radius = g.CylinderShape()->Radius();
            o.length = g.CylinderShape()->Length();
            break;
          case sdf::GeometryType::SPHERE:
            o.type = ShapeType::kSphere;
            o.radius = g.SphereShape()->Radius();
            break;
          case sdf::GeometryType::CAPSULE:
            o.type = ShapeType::kCylinder;   // conservative: cylinder over the whole capsule
            o.radius = g.CapsuleShape()->Radius();
            o.length = g.CapsuleShape()->Length() + 2 * o.radius;
            break;
          case sdf::GeometryType::ELLIPSOID: {
              o.type = ShapeType::kSphere;
              const auto r = g.EllipsoidShape()->Radii();
              o.radius = std::max({r.X(), r.Y(), r.Z()});
              break;
            }
          case sdf::GeometryType::PLANE:
          case sdf::GeometryType::EMPTY:
            return true;
          default:
            if (warned_.insert(e).second) {
              gzwarn << "[UwbDevice] collision '" << gz::sim::scopedName(e, ecm)
                     << "' has a shape UWB cannot ray-test (use box/cylinder/sphere); "
                     << "it will not block UWB\n";
            }
            return true;
        }
        cache_.push_back({e, gz::sim::topLevelModel(e, ecm), o});
        return true;
      });
  }

  std::optional<gz::math::Pose3d> antennaPose(
    gz::sim::Entity link, const gz::math::Pose3d & off,
    const gz::sim::EntityComponentManager & ecm) const
  {
    if (link == gz::sim::kNullEntity) {return std::nullopt;}
    return gz::sim::worldPose(link, ecm) * off;
  }

  void runSlot(double t_slot, const gz::sim::EntityComponentManager & ecm)
  {
    const auto anchors = AnchorRegistry::instance().list();
    const auto tag_pose = antennaPose(link_, offset_, ecm);
    if (!tag_pose) {next_slot_ = t_slot + 0.1; return;}
    const Vec3 tag = toVec(tag_pose->Pos());

    // at the start of each cycle: update who is in the network
    if (slot_index_ == 0) {
      network_.clear();
      for (const auto & a : anchors) {
        auto & m = members_[a.id];
        const Vec3 ap = toVec((gz::sim::worldPose(a.link, ecm) * a.offset).Pos());
        const bool in_range = (ap - tag).norm() <= model_data_->maxRange();
        if (in_range) {
          m.lost_since = -1;
          if (m.seen_since < 0) {m.seen_since = t_slot;}
          if (!m.in_network && t_slot - m.seen_since >= join_delay_) {m.in_network = true;}
        } else {
          m.seen_since = -1;
          if (m.lost_since < 0) {m.lost_since = t_slot;}
          if (m.in_network && t_slot - m.lost_since >= leave_delay_) {m.in_network = false;}
        }
        if (m.in_network) {network_.push_back(a);}
      }
      cycle_size_ = static_cast<int>(network_.size());
      if (cycle_size_ == 0) {
        // nothing to range to: try again shortly
        next_slot_ = t_slot + model_data_->slotGap(1, 0);
        return;
      }
    }

    const AnchorEntry & a = network_[slot_index_];
    const Vec3 ap = toVec((gz::sim::worldPose(a.link, ecm) * a.offset).Pos());

    std::vector<Obstacle> obstacles;
    obstacles.reserve(cache_.size());
    for (const auto & c : cache_) {
      if (c.owner == model_ || c.owner == a.model) {continue;}
      Obstacle o = c.shape;
      o.pose = toPose(gz::sim::worldPose(c.collision, ecm));
      obstacles.push_back(o);
    }
    const LinkGeometry g = analyseLink(ap, tag, obstacles, model_data_->wavelength());
    UwbMeasurement m;
    UwbLinkState & link_state = members_[a.id].link;
    link_state.anchor_id = a.id;
    const UwbOutcome outcome = model_data_->sample(g, link_state, rng_, m);

    const int sec = static_cast<int>(t_slot);
    const int nsec = static_cast<int>((t_slot - sec) * 1e9);
    char buf[256];
    if (outcome == UwbOutcome::kOk) {
      std::snprintf(buf, sizeof(buf), "[%d,%d,%.5f,%.4f,%.4f,%.4f,%.4f]",
        cycle_size_, a.id, m.range, m.rx, m.fp, m.quality, m.delta);
      gz::msgs::StringMsg msg;
      msg.mutable_header()->mutable_stamp()->set_sec(sec);
      msg.mutable_header()->mutable_stamp()->set_nsec(nsec);
      msg.set_data(buf);
      pub_raw_.Publish(msg);
    }
    static const char * names[] = {"ok", "dropped", "out_of_range", "too_much_material"};
    char tbuf[512];
    std::snprintf(tbuf, sizeof(tbuf),
      "{\"t\":%.4f,\"tag\":%d,\"anchor\":%d,\"true_range\":%.5f,\"los\":%s,"
      "\"nlos_weight\":%.3f,\"obstacles\":%d,\"clearance\":%.4f,\"nu\":%.3f,"
      "\"outcome\":\"%s\",\"tag_pos\":[%.4f,%.4f,%.4f],\"anchor_pos\":[%.4f,%.4f,%.4f]}",
      t_slot, id_, a.id, g.distance, g.blocked ? "false" : "true",
      model_data_->nlosWeight(g), g.n_obstacles, g.clearance, g.nu,
      names[static_cast<int>(outcome)], tag.x, tag.y, tag.z, ap.x, ap.y, ap.z);
    gz::msgs::StringMsg tmsg;
    tmsg.mutable_header()->mutable_stamp()->set_sec(sec);
    tmsg.mutable_header()->mutable_stamp()->set_nsec(nsec);
    tmsg.set_data(tbuf);
    pub_truth_.Publish(tmsg);

    std::normal_distribution<double> jit(0.0, model_data_->timingJitter());
    const double gap = model_data_->slotGap(cycle_size_, slot_index_);
    next_slot_ = t_slot + std::max(0.002, gap + jit(rng_));
    slot_index_ = (slot_index_ + 1) % cycle_size_;
  }

  Role role_{Role::kNone};
  int id_{0};
  gz::sim::Entity model_{gz::sim::kNullEntity};
  gz::sim::Entity link_{gz::sim::kNullEntity};
  gz::math::Pose3d offset_;

  std::unique_ptr<UwbChannelModel> model_data_;
  std::mt19937_64 rng_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_raw_, pub_truth_;
  std::string topic_raw_, topic_truth_;

  bool started_{false};
  double next_slot_{0};
  double last_refresh_{0};
  double join_delay_{0.5}, leave_delay_{1.0};
  int slot_index_{0};
  int cycle_size_{0};
  std::vector<AnchorEntry> network_;
  std::map<int, Member> members_;
  std::vector<Cached> cache_;
  std::set<gz::sim::Entity> warned_;
};

}  // namespace uwb_imu_sandbox

GZ_ADD_PLUGIN(
  uwb_imu_sandbox::UwbDevice, gz::sim::System,
  uwb_imu_sandbox::UwbDevice::ISystemConfigure,
  uwb_imu_sandbox::UwbDevice::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(uwb_imu_sandbox::UwbDevice, "uwb_imu_sandbox::UwbDevice")
