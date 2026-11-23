# Ground truth for one ranging slot (simulation only; use it to score
# filters, LOS/NLOS classifiers and localization, never as an input).

std_msgs/Header header
uint16 tag_id
uint16 anchor_id
float64 true_range           # exact antenna-to-antenna distance [m]
bool line_of_sight           # false if an obstacle cuts the straight path
float32 nlos_weight          # 0 = pure LOS behaviour .. 1 = fully NLOS behaviour
uint8 obstacles              # number of obstacles cutting the straight path
float32 clearance            # depth behind the obstacle edge [m] (>0 blocked, <0 clear by)
float32 fresnel_nu           # diffraction parameter of the worst obstacle
string outcome               # ok | dropped | out_of_range | too_much_material
geometry_msgs/Point tag_position
geometry_msgs/Point anchor_position
