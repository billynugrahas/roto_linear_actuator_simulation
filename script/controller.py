#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

class NeedleController:
    def __init__(self):
        self.linear_state_pos = 0.0
        self.linear_state_vel = 0.0
        self.linear_state_error = 0.0
        self.prismatic_pub = rospy.Publisher("/needle_bot/prismatic_position_controller/command", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher("/needle_bot/servo_position_controller/command", Float64, queue_size=1)
        self.prismatic_sub = rospy.Subscriber("/needle_bot/prismatic_position_controller/state", JointControllerState, self.prismatic_cb)

    def set_init_pos(self):
        target_pos = Float64()
        target_pos.data = 0
        self.servo_pub.publish(target_pos)
        self.prismatic_pub.publish(target_pos)

    def prismatic_cb(self, msg):
        self.linear_state_pos = msg.process_value
        self.linear_state_vel = msg.process_value_dot
        self.linear_state_error = msg.error

    def linear_control(self, target_pos):
        msg = Float64()
        msg.data = target_pos #meter
        self.prismatic_pub.publish(msg)

    def rotation_control(self, target_pos):
        msg = Float64()
        msg.data = target_pos #meter
        self.servo_pub.publish(msg)

    def mpc_control(self, event=None):
        print("[linear_state] feedback: pos: %01.03f - vel: %01.03f - pos: %01.03f" % (self.linear_state_pos, self.linear_state_vel, self.linear_state_error))
        #do the calculation stuff here then publish it using linear_control(target_pos)
        self.linear_control(0.5)

if __name__ == '__main__':
    rospy.init_node('needle_controller')
    controller_rate = 10.0 # Hz
    needle_controller = NeedleController()
    rospy.Timer(rospy.Duration(1.0/controller_rate), needle_controller.mpc_control)
    rospy.spin()
    