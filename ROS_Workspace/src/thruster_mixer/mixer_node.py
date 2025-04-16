#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# import your mixing functions
from thruster_mixer.planar_mixer import forward, reverse, turn_right, turn_left

def cmd_cb(msg: Twist):
    # Decide which maneuver based on msg, for example:
    thrust = abs(msg.linear.x)
    if msg.linear.x > 0:
        forces = forward(thrust)
    else:
        forces = reverse(thrust)
    # …or for yaw:
    # forces = turn_right(thrust) if msg.angular.z < 0 else turn_left(thrust)
    out = Float32MultiArray(data=forces.tolist())
    pub.publish(out)

if __name__ == '__main__':
    rospy.init_node('thruster_mixer')
    pub = rospy.Publisher('/thruster_cmds', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/cmd_vel', Twist, cmd_cb)
    rospy.spin()
