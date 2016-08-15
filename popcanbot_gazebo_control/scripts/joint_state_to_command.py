#!/usr/bin/env python
# Lucas Walter
# make a joint exactly what the command wants it to be- this only works
# for position control.

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointStateToCommand:
    def __init__(self):
        self.joint_name = rospy.get_param("~joint_name")
        self.command_pub = rospy.Publisher("command", Float64, queue_size=1)
        self.joint_sub = rospy.Subscriber("joint_state", JointState,
                                          self.joint_state_callback, queue_size=1)

    def joint_state_callback(self, msg):
        ind = None
        for i in range(len(msg.name)):
            if msg.name[i] == self.joint_name:
                ind = i
                break
        if ind is None:
            return
        self.command_pub.publish(msg.position[ind])

if __name__ == '__main__':
    rospy.init_node('joint_state_to_command')
    joint_state_to_command = JointStateToCommand()
    rospy.spin()
