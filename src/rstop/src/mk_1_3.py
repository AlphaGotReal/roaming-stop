#! /usr/bin/python3.9

import rospy
import time
import signal
import sys
import can
import glob
import threading
import tinymovr

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tinymovr.tee import init_tee
from tinymovr.config import create_device, get_bus_config

AXLE_LENGTH = 0.76  # distance between the left and right wheels
WHEEL_RADIUS = 0.19  # radius of each wheel

class TinyM:
    def find_serial_devices(self, pattern):
        serial_devices = glob.glob(pattern)
        return serial_devices

    def __init__(self):
        rospy.init_node('bldc_node', anonymous=True)

        pattern = '/dev/ttyACM*'
        matching_devices = self.find_serial_devices(pattern)

        if matching_devices:
            channel = matching_devices[0]
        else:
            channel = "/dev/ttyACM0"

        params = get_bus_config()
        params["interface"] = "slcan"
        params["bitrate"] = 1000000
        params["channel"] = channel
        init_tee(can.Bus(**params))

        self.tm3 = create_device(node_id=3)
        self.tm2 = create_device(node_id=2)

        self.tm3.reset()
        self.tm2.reset()

        self.tm3.encoder.type = 1
        self.tm3.motor.pole_pairs = 4
        self.tm3.controller.velocity.p_gain = 0.0105 #0.0105 initially was 0.007
        self.tm3.controller.velocity.i_gain = 0.005 #try, initially was 0.001
        self.tm3.save_config()
        self.tm3.reset()
        time.sleep(2)

        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 4
        self.tm2.controller.velocity.p_gain = 0.0105
        self.tm2.controller.velocity.i_gain = 0.005
        self.tm2.save_config()
        self.tm2.reset()
        time.sleep(2)

        self.rate = rospy.Rate(10)

    def engage(self):
        self.tm3.controller.velocity_mode()
        self.tm3.controller.velocity_setpoint = 0
        time.sleep(1)
        self.tm2.controller.velocity_mode()
        self.tm2.controller.velocity_setpoint = 0
        time.sleep(1)
        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_vel_clbk(self, msg):
        try:
            left_w_vel = msg.linear.x - (msg.angular.z * AXLE_LENGTH)
            right_w_vel = msg.linear.x + (msg.angular.z * AXLE_LENGTH)

            left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
            right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

            self.tm3.controller.velocity.setpoint = (right_w_rpm / 60) * 24 * 60
            self.tm2.controller.velocity.setpoint = (left_w_rpm / 60) * 24 * 60
        
            print("TM2: " + str(self.tm2.controller))
            print("                                                     ")
            print("-----------------------------------------------------")
            print("TM3: " + str(self.tm3.controller))
            print("                                                     ")
            print("-----------------------------------------------------")
        except tinymovr.channel.ResponseError as e:
            print(f"Error communicating with TinyM: {e}")
            
    def encoder_pub(self):
                         
        pub1 = rospy.Publisher('encoder_pos_tm1', Float64, queue_size=1)
        pub2 = rospy.Publisher('encoder_pos_tm2', Float64, queue_size=1)
        pub3 = rospy.Publisher('encoder_vel_tm1', Float64, queue_size=1)
        pub4 = rospy.Publisher('encoder_vel_tm2', Float64, queue_size=1)

        while not rospy.is_shutdown():
            try:
                enc_vel_estTM1 = self.tm3.encoder.velocity_estimate.magnitude
                enc_vel_estTM2 = self.tm2.encoder.velocity_estimate.magnitude
                
                enc_pos_estTM1 = self.tm3.encoder.position_estimate.magnitude               
                enc_pos_estTM2 = self.tm2.encoder.position_estimate.magnitude

                pub1.publish(Float64(enc_pos_estTM1))
                pub2.publish(Float64(enc_pos_estTM2))
                pub3.publish(Float64(enc_vel_estTM1))
                pub4.publish(Float64(enc_vel_estTM2))
                self.rate.sleep()
            
            except tinymovr.channel.ResponseError as e:
                print(f"Error communicating with TinyM: {e}")
            

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk, queue_size=1)
        
        self.encoder_pub()
        self.watchdog()

    def watchdog(self):
        self.tm2.watchdog.enabled = 1
        self.tm3.watchdog.enabled = 1
        
    def signal_handler(self, signum, frame):
        print("Stopping the program and idling the controller...")
        self.tm3.controller.idle()
        self.tm2.controller.idle()
        sys.exit(0)

if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
    except rospy.ROSInterruptException:
        pass

