#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from armms_msgs.srv import SetTorqueMean
from armms_msgs.srv import SetFloat
from armms_msgs.msg import WebInterface

class NodeTest:
    def __init__(self):
        self.alpha_slow_ema = rospy.get_param("~alpha_slow_ema")
        self.alpha_fast_ema = rospy.get_param("~alpha_fast_ema")
        self.threshold_high = rospy.get_param("~threshold_high")
        self.threshold_low = rospy.get_param("~threshold_low")
        self.user_mean_torque = rospy.get_param("~user_mean_torque")


        self.ema_init = False
        self.slow_ema = 0
        self.fast_ema = 0
        self.delta_ema = 0
        self.computed_cmd = 0
        self.position = 0
        self.torque = 0

        self.joint_states_sub = rospy.Subscriber(
            '/joint_states', JointState, self.callback_joint_states)
    
        self.mean_torque_pub = rospy.Publisher('~mean_torque', Float32, queue_size=1)
        self.threshold_hi_pub = rospy.Publisher('~threshold_hi', Float32, queue_size=1)
        self.threshold_lo_pub = rospy.Publisher('~threshold_lo', Float32, queue_size=1)
        self.fast_ema_pub = rospy.Publisher('~fast_ema', Float32, queue_size=1)
        self.slow_ema_pub = rospy.Publisher('~slow_ema', Float32, queue_size=1)
        self.delta_ema_pub = rospy.Publisher('~delta_ema', Float32, queue_size=1)
        self.cmd_pub = rospy.Publisher('~cmd_pub', Int32, queue_size=1)
        self.web_pub = rospy.Publisher('/armms_web/web_interface', WebInterface, queue_size=1)

        self.set_torque_mean_service = rospy.Service('~set_torque_mean', SetTorqueMean, self.set_torque_mean)
        self.set_thresh_lo_service = rospy.Service('~set_thresh_lo', SetFloat, self.set_thresh_lo)
        self.set_thresh_hi_service = rospy.Service('~set_thresh_hi', SetFloat, self.set_thresh_hi)

        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_cmd)

    def ema(self, alpha, data, current_value):
        return (alpha*data) + ((1-alpha) * current_value)

    def publish_cmd(self, event):
        rospy.loginfo('publish_cmd')
        if(self.ema_init == False):
            return
        # Compute EMA torque filtering 
        self.slow_ema = self.ema(self.alpha_slow_ema, self.torque, self.slow_ema)
        self.fast_ema = self.ema(self.alpha_fast_ema, self.torque, self.fast_ema)        
        self.delta_ema = self.fast_ema - self.slow_ema

        # Direct torque control using threshold + hysteresis detection
        if(self.computed_cmd == -1):
            if(self.fast_ema < (self.user_mean_torque+self.threshold_low)):
                self.computed_cmd = 0
        elif(self.computed_cmd == +1):
            if(self.fast_ema > (self.user_mean_torque-self.threshold_low)):
                self.computed_cmd = 0    
        if(self.fast_ema > (self.user_mean_torque+self.threshold_high)):
            self.computed_cmd = -1
        elif(self.fast_ema < (self.user_mean_torque-self.threshold_high)):
            self.computed_cmd = +1

        cmd_msg = Int32()
        cmd_msg.data = self.computed_cmd
        web_msg = WebInterface()
        if self.computed_cmd == 0:
            web_msg.user_button_up = False
            web_msg.user_button_down = False
        elif self.computed_cmd == -1:
            web_msg.user_button_up = True
            web_msg.user_button_down = False
        elif self.computed_cmd == 1:
            web_msg.user_button_up = False
            web_msg.user_button_down = True

        # Publish computed command (0 or +1 or -1)
        self.cmd_pub.publish(cmd_msg)

        # Publish actuator control command (Mock the web gui interface)
        self.web_pub.publish(web_msg)

        msg = Float32()
        # Publish fast EMA torque filtering 
        msg.data = self.fast_ema
        self.fast_ema_pub.publish(msg)

        # Publish slow EMA torque filtering 
        msg.data = self.slow_ema
        self.slow_ema_pub.publish(msg)

        # Publish delta between fast and slow EMA
        msg.data = self.delta_ema
        self.delta_ema_pub.publish(msg)
        
        # Publish mean torque value
        msg.data = self.user_mean_torque
        self.mean_torque_pub.publish(msg)

        # Publish threshold high value
        msg.data = self.threshold_high
        self.threshold_hi_pub.publish(msg)

        # Publish threshold low value
        msg.data = self.threshold_low
        self.threshold_lo_pub.publish(msg)

    # @staticmethod
    def callback_joint_states(self, msg):
        self.position = msg.position[0]
        self.torque = msg.effort[0]
        if(self.ema_init == False):
            self.fast_ema = self.position
            self.slow_ema = self.position
            self.ema_init = True

    def set_torque_mean(self, req):
        if(req.current_torque):
            self.user_mean_torque = self.fast_ema
        else:
            self.user_mean_torque = req.value
        return []

    def set_thresh_lo(self, req):
        self.threshold_low = req.value
        return []
    
    def set_thresh_hi(self, req):
        self.threshold_high = req.value
        return []
        
        
if __name__ == '__main__':
    rospy.init_node('armms_torque_test')
    NodeTest()
    rospy.spin()
