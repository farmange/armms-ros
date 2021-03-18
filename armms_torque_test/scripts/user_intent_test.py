#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from armms_msgs.msg import WebInterface

class NodeTest:
    def __init__(self):
        self.alpha_slow_ema = rospy.get_param("~alpha_slow_ema")
        self.alpha_fast_ema = rospy.get_param("~alpha_fast_ema")
        self.threshold_high = rospy.get_param("~threshold_high")
        self.threshold_low = rospy.get_param("~threshold_low")
        self.user_mean_torque = rospy.get_param("~user_mean_torque")
        self.detection_delay = rospy.get_param("~detection_delay")

        table_size = (self.detection_delay*100)
        self.window = [0] * int(table_size)
        self.window_index = 0
        self.windows_init = False
        self.fast_ema_prev = 0
        
        self.mass = (2.065+0.063)
        self.length = 0.295
        self.slow_ema = 0
        self.fast_ema = 0
        self.delta_ema = 0
        self.threshold_detection = 0
        self.computed_cmd = 0
        self.position = 0
        self.torque = 0
        self.torque_zero = 0
        # self.table = [0] * 80
        self.tableToUp = [6.600011882778809, 6.601024759302092, 6.580408350313492, 6.564738568808329, 6.538910831772383, 6.5193833993424395, 6.501966130707494, 6.483398741059086, 6.463322242794318, 6.442331938115678, 6.435170668001825, 6.42414166969339, 6.4281627468718145, 6.42692077589712, 6.4322734541119315, 6.448459458343881, 6.460494920460119, 6.4822408890925125, 6.502146065915139, 6.535554404952226, 6.569278654587903, 6.609102231044998, 6.650763111806787, 6.703650953345411, 6.741523097489362, 6.788804118952617, 6.838963156121731, 6.88409892156092, 6.938772192917497, 6.981540867507514, 7.0328400959271065, 7.0777616414168705, 7.121263750382462, 7.160213140644862, 7.1901263474242345, 7.218286626361821, 7.240158296641059, 7.267097868829483, 7.280858729852826, 7.299485093505558, 7.303213471088059, 7.299611491004228, 7.296414903120684, 7.284680299373268, 7.273261284767829, 7.259028561264891, 7.2412633421062065, 7.217267674700817, 7.1885286853537735, 7.152312295089959, 7.1267299409090645, 7.101081258325133, 7.066171541705321, 7.032561164834945, 7.0034423834343515, 6.986717856552493, 6.9564180026496345, 6.934513782886623, 6.906965853455473, 6.88690085962127, 6.877981526204663, 6.86606634472675, 6.860762168503816, 6.853775932522915, 6.858487345878415, 6.854337283840388, 6.864721311310329, 6.864536371314944, 6.875388416824743, 6.890808091758408, 6.901128895877014, 6.9194299928295, 6.939220725799163, 6.972986535993462, 7.057770615334718, 7.057770615334718, 7.057770615334718, 7.057770615334718, 7.057770615334718, 7.057770615334718]
        # self.tableToDown = [6.6402123124877575, 6.585303026085459, 6.5510726689119645, 6.52679949923771, 6.528180280286987, 6.5387989302683645, 6.556733984645057, 6.56557460232928, 6.585850009722086, 6.605339985200259, 6.637015454801117, 6.66752155098666, 6.700571130090555, 6.752793284658492, 6.805776042119316, 6.84894983940079, 6.9018933782429155, 6.92776212163235, 6.985676710448781, 7.033540398014332, 7.079835329659904, 7.122610402685561, 7.161122488117872, 7.211378836005204, 7.23956087128541, 7.281024831715536, 7.303981127557204, 7.341037366538422, 7.354012071371603, 7.378856840419012, 7.3888558928104375, 7.399224609282657, 7.401446628875878, 7.393981437397937, 7.391837041851501, 7.375404888680429, 7.368075808470977, 7.339654984574925, 7.322773988566398, 7.295379369418376, 7.269255785296858, 7.244870064633438, 7.2036201171924334, 7.17898852303954, 7.139302144080335, 7.112828946453064, 7.080398710016617, 7.054771881642463, 7.015027378832787, 6.993949237582341, 6.9788284908210745, 6.9652643098733495, 6.946653922185452, 6.936406699522271, 6.928052938343996, 6.9147858141831495, 6.9187109672212905, 6.92253750553083, 6.923308126500759, 6.928998194896831, 6.939867077366801, 6.954068623570316, 6.950981607728022, 6.9608256423069115, 6.974514336794591, 6.992013520692585, 6.996899685459437, 7.006996070841023, 7.016410097255089, 7.014549057167572, 7.0183457750594656, 7.010406195912323, 7.024692863992884, 7.017485319556163, 7.014132266111148, 6.994485570185495, 6.994485570185495, 6.994485570185495, 6.994485570185495, 6.994485570185495]
        self.table = self.tableToUp
        self.joint_states_sub = rospy.Subscriber(
            '/joint_states', JointState, self.callback_joint_states)
    
        self.fast_ema_pub = rospy.Publisher('fast_ema', Float32, queue_size=1)
        self.slow_ema_pub = rospy.Publisher('slow_ema', Float32, queue_size=1)
        self.delta_ema_pub = rospy.Publisher('delta_ema', Float32, queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_pub', Int32, queue_size=1)
        self.web_pub = rospy.Publisher('/armms_web/web_interface', WebInterface, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_cmd)

    def ema(self, alpha, data, current_value):
        return (alpha*data) + ((1-alpha) * current_value)

    def threshold(self, threshold, data):
        detected_value =0
        if data > threshold:
            detected_value = +1
        elif data < -threshold:
            detected_value = -1
        return detected_value

    def convert_cmd(self, current_input, prev_input, prev_cmd):
        converted_value = prev_cmd
        # detect rizing edge
        if current_input > 0 and prev_input == 0:
            if converted_value < 0:
                # if previous value where opposite direction then cmd = 0
                converted_value = 0
            elif converted_value == 0:
                # if previous value where 0 then go in this direction
                converted_value = +1
        # detect falling edge
        elif current_input < 0 and prev_input == 0:
            if converted_value > 0:
                # if previous value where opposite direction then cmd = 0
                converted_value = 0
            elif converted_value == 0:
                # if previous value where 0 then go in this direction
                converted_value = -1           
        return converted_value

    def publish_cmd(self, event):
        rospy.loginfo('publish_cmd')
        self.fast_ema_prev = self.fast_ema
        self.slow_ema = self.ema(self.alpha_slow_ema, self.torque, self.slow_ema)
        self.fast_ema = self.ema(self.alpha_fast_ema, self.torque, self.fast_ema)        
        self.delta_ema = self.fast_ema - self.slow_ema
        # Test with hard coded model 
        # pos_to_index = int(round(self.position))-155
        # self.slow_ema = 0
        # if(pos_to_index >=0 and pos_to_index < 800):
        #     # self.table[pos_to_index] = self.fast_ema
        #     # rospy.loginfo(self.table)
        #     # self.slow_ema = self.table[pos_to_index]
        #     self.slow_ema = (self.mass*9.81*self.length*np.cos((self.position+5.0)*np.pi/180.)) + self.table[pos_to_index]
        # self.delta_ema = self.fast_ema - self.slow_ema

        # Simple delta between EMA and threshold detection
        # threshold_detection_prev = self.threshold_detection
        # self.threshold_detection = self.threshold(self.threshold_value, self.delta_ema)
        # computed_cmd_prev = self.computed_cmd
        # self.computed_cmd = self.convert_cmd(self.threshold_detection, threshold_detection_prev, computed_cmd_prev)
        
        # Derivative
        # if(self.windows_init == False):
        #     self.fast_ema_prev =  self.fast_ema
        #     self.windows_init = True
        # derivative = (self.fast_ema - self.fast_ema_prev) / 100.0
        # self.delta_ema = derivative
        # self.slow_ema = self.ema(self.alpha_slow_ema, self.delta_ema, self.slow_ema)

        # Direct torque control with hysteresis threshold
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


        # Threshold detection within a specified delay
        # self.window[self.window_index] = self.fast_ema
        # self.window_index = self.window_index + 1
        # if(self.window_index >= len(self.window)):
        #     self.window_index = 0
        #     self.windows_init = True
        # if(self.windows_init):
        #     min_val = min(self.window)
        #     max_val = max(self.window)
        #     if((max_val-min_val) > self.threshold_value):
        #         if(self.computed_cmd == 0):
        #             if(self.fast_ema > (min_val + (self.threshold_value/2.0))):
        #                 self.computed_cmd = -1
        #             else:
        #                 self.computed_cmd = +1
        #     else:
        #         self.computed_cmd = 0


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
        self.cmd_pub.publish(cmd_msg)
        self.web_pub.publish(web_msg)
        msg = Float32()
        msg.data = self.fast_ema
        self.fast_ema_pub.publish(msg)
        msg.data = self.slow_ema
        self.slow_ema_pub.publish(msg)
        msg.data = self.delta_ema
        self.delta_ema_pub.publish(msg)

    # @staticmethod
    def callback_joint_states(self, msg):
        self.position = msg.position[0]
        self.torque = msg.effort[0]
        if(self.torque_zero == 0):
            self.torque_zero == self.torque
        
if __name__ == '__main__':
    rospy.init_node('armms_torque_test')
    NodeTest()
    rospy.spin()
