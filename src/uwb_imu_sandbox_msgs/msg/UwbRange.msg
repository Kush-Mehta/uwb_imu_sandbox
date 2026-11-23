# One UWB ranging result from a tag to one anchor.
#
# The tag ranges to the anchors in its network one after the other, so these
# arrive one anchor at a time (about 6 Hz per anchor with 4 anchors).
#
# The four signal fields are normalised, unitless values on a 0..100 scale.
# They keep the order of the physical quantity (higher rx_level = stronger
# signal) and its behaviour (they drop behind obstacles, they separate LOS
# from NLOS), but they are not in dBm.

std_msgs/Header header   # stamp: time of the ranging slot; frame_id: tag frame
uint16 tag_id
uint16 anchor_id
uint8 network_count      # anchors currently in the tag's network
float64 range            # measured distance [m]
float32 rx_level         # received power level (normalised)
float32 fp_level         # first-path power level (normalised)
float32 quality          # first-path quality index (normalised)
float32 delta_level      # received minus first-path power (normalised)
