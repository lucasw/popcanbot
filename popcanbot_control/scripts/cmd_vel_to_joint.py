#!/usr/bin/env python
# Lucas Walter
# June 2016
#
# Subscribe to a cmd_vel Twist message and interpret the linear
# and angular components into a joint state for the wheels.

import math

import rospy
import tf2_py as tf2
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from tf import transformations


class CmdVelToJoint():
    def __init__(self):
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        # angular mode maps angular z directly to steering angle
        # (adjusted appropriately)
        # non-angular mode is somewhat suspect, but it turns
        # a linear y into a command to turn just so that the
        # achieved linear x and y match the desired, though
        # the vehicle has to turn to do so.
        # Non-angular mode is not yet supported.
        self.angular_mode = rospy.get_param("~angular_mode", True)

        # Not used yet
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf = tf2_ros.TransformListener(self.tf_buffer)
        # broadcast odometry
        self.br = tf2_ros.TransformBroadcaster()
        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = "base_link"
        self.ts.transform.rotation.w = 1.0
        self.angle = 0

        # The cmd_vel is assumed to be in the base_link frame centered
        # on the middle of the two driven wheels
        # This is half the distance between the two drive wheels
        self.base_radius = rospy.get_param("~base_radius", 0.06)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.04)

        self.left_wheel_joint = rospy.get_param("~left_wheel_joint", "wheel_front_left_joint")
        self.right_wheel_joint = rospy.get_param("~right_wheel_joint", "wheel_front_right_joint")

        self.point_pub = rospy.Publisher("cmd_vel_spin_center", PointStamped, queue_size=1)
        self.joint_pub = {}
        self.joint_pub['left'] = rospy.Publisher("front_left/joint_state",
                                                 JointState, queue_size=1)
        self.joint_pub['right'] = rospy.Publisher("front_right/joint_state",
                                                  JointState, queue_size=1)
        # TODO(lucasw) is there a way to get TwistStamped out of standard
        # move_base publishers?
        # TODO(lucasw) make this more generic, read in a list of any number of wheels
        # the requirement is that that all have to be aligned, and also need
        # a set spin center.
        self.ind = {}
        self.joint_state = {}
        self.joint_state['left'] = JointState()
        self.joint_state['left'].name.append(self.left_wheel_joint)
        self.joint_state['left'].position.append(0.0)
        self.joint_state['left'].velocity.append(0.0)
        self.joint_state['right'] = JointState()
        self.joint_state['right'].name.append(self.right_wheel_joint)
        self.joint_state['right'].position.append(0.0)
        self.joint_state['right'].velocity.append(0.0)

        self.cmd_vel = Twist()
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def update(self, event):

        # if the cmd_vel is pure linear x
        linear_x = self.cmd_vel.linear.x
        # linear_y = self.cmd_vel.linear.y
        angular_z = self.cmd_vel.angular.z
        wheel_angular_velocity = 0
        if True:  # self.angular_mode or linear_y == 0.0:
            linear_x_offset = angular_z * self.base_radius
            # TODO(lucas) handle different arrangements of wheels and different
            # wheel radiuses generically.
            left_wheel_angular_vel = (linear_x - linear_x_offset) / self.wheel_radius
            self.joint_state['left'].velocity[0] = left_wheel_angular_vel
            # TODO(lucasw) assuming fixed period for now, could
            # measure actual dt with event parameter.
            self.joint_state['left'].position[0] += left_wheel_angular_vel * self.period

            # TODO(lucasw) if velocities exceed maximums then scale them back here?

            right_wheel_angular_vel = (linear_x + linear_x_offset) / self.wheel_radius
            self.joint_state['right'].velocity[0] = right_wheel_angular_vel
            self.joint_state['right'].position[0] += right_wheel_angular_vel * self.period

            # TODO(lucasw) compute where the spin center is for cmd_vel with
            # combined linear x and angular z and publish an rviz Marker for it
            if False:  # self.angular_mode and angular_z != 0.0:
                base_turn_radius = abs(linear_x) / abs(angular_z)
                # base_turn_radius * sin(fixed_to_base_angle) = fixed_to_base.transform.translation.x
                spin_center_y = base_turn_radius * math.cos(fixed_to_base_angle)
                steer_angle = math.atan2(fixed_to_steer.transform.translation.x,
                                         spin_center_y)
                if angular_z > 0.0:
                    spin_center_y = -spin_center_y
                    steer_angle = -steer_angle
                if linear_x < 0.0:
                    spin_center_y = -spin_center_y
                    steer_angle = -steer_angle
                self.joint_state.position[0] = steer_angle
            # TODO(lucasw) compute what the wheels have to do to accomplish
            # a linear_y (and a linear_x is also needed for any linear_y)
            # Look at carbot code for an example

        for key in self.joint_state.keys():
            # TODO(lucasw) add an offset into the future?
            self.joint_state[key].header.stamp = event.current_real
            self.joint_pub[key].publish(self.joint_state[key])

        # update odometry
        self.angle += angular_z * self.period
        distance = linear_x * self.period
        # TODO(lucasw) actually compute the arc traveled rather than assuming the travel
        # was a straight line.
        self.ts.transform.translation.x += distance * math.cos(-self.angle)
        self.ts.transform.translation.y -= distance * math.sin(-self.angle)

        quat = transformations.quaternion_from_euler(0, 0, self.angle)
        self.ts.transform.rotation.x = quat[0]
        self.ts.transform.rotation.y = quat[1]
        self.ts.transform.rotation.z = quat[2]
        self.ts.transform.rotation.w = quat[3]

        self.br.sendTransform(self.ts)


if __name__ == '__main__':
    rospy.init_node("cmd_vel_to_joint")
    cmd_vel_to_joint = CmdVelToJoint()
    rospy.spin()
