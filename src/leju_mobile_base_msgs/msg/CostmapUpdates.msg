# Message containing multiple rectangular incremental updates to a costmap
# This allows publishing all rectangular updates to the previously published full map in a single message
# instead of sending multiple separate update messages

std_msgs/Header header
map_msgs/OccupancyGridUpdate[] updates
