#!/bin/bash

# Surge Only
rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0]'

rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[8.0, 0.0, 0.0]' '[0.0, 0.0, 0]'

rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0]'

# Yaw Only
rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.2]'

rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -0.2]'

# Surge and Yaw
rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0]'

rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.2]'

rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -0.2]'

# Stop
rostopic pub -1 /cora/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


