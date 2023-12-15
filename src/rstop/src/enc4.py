#! /usr/bin/python3.9

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import math
import tf

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        
        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # Publish to /odom
        self.avg_vel_pub = rospy.Publisher("/avg_vel", Float64, queue_size=1)
        self.avg_angle_pub = rospy.Publisher("/avg_angle", Float64, queue_size=1)
        
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.pose = Pose()
        self.twist = Twist()
        self.vels = []
        self.angles = []

        self.br = tf.TransformBroadcaster() 

        self.wheel_distance = 0.95#0.68 #0.76  # Distance between wheels (assumed value)
        self.wheel_radius = 0.19  # Radius of each wheel (assumed value)
        self.theta = 0.0
        rospy.Subscriber('encoder_vel_tm1', Float64, self.enc_tm1_callback)
        rospy.Subscriber('encoder_vel_tm2', Float64, self.enc_tm2_callback)
        rospy.Subscriber('encoder_pos_tm1', Float64, self.enc_pos_tm1_callback)
        rospy.Subscriber('encoder_pos_tm2', Float64, self.enc_pos_tm2_callback)

    def enc_tm1_callback(self, data):
        self.enc_vel_tm1 = data.data

    def enc_tm2_callback(self, data):
        self.enc_vel_tm2 = data.data
        
    def enc_pos_tm1_callback(self, data):
        self.enc_pos_tm1 = data.data
        
    def enc_pos_tm2_callback(self, data):
        self.enc_pos_tm2 = data.data

    def calculate_wheel_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.vel1 = self.enc_vel_tm1*0.00103 #0.00095
        self.vel2 = self.enc_vel_tm2*0.00103 #0.00095

        linear_vel = (self.vel2 - self.vel1) / 2.0
        
        self.vels.append(linear_vel)
        
        if (len(self.vels) > 50):
            self.vels = self.vels[1:]
        
        avg_vel = Float64()
        avg_vel.data = sum(self.vels)/len(self.vels)
        self.avg_vel_pub.publish(avg_vel)

        angular_vel = (-self.vel1 -self.vel2) / self.wheel_distance
        self.angles.append(angular_vel)
        
        if (len(self.angles) > 50):
            self.angles = self.angles[1:]
            
        avg_angle = Float64()
        avg_angle.data = sum(self.angles)/len(self.angles)
        
        self.avg_angle_pub.publish(avg_angle)
        lateral_vel = angular_vel * self.wheel_distance / 2

        self.pose.position.x += (linear_vel * dt * math.cos(self.theta))
        self.pose.position.y += (linear_vel * dt * math.sin(self.theta)) #+ lateral_vel * dt 
        self.pose.position.z = 0  # Assuming no vertical movement

        self.theta += angular_vel*dt
        self.pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))
        print(self.theta)

        self.twist.linear.x = linear_vel
        self.twist.linear.y = lateral_vel
        self.twist.angular.z = angular_vel

        self.odom.header.stamp = current_time
        self.prev_time = current_time

    def publish_wheel_odometry(self):
        self.calculate_wheel_odometry()
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

    def create_base_link(self):
        pos = (self.odom.pose.pose.position.x, 
                self.odom.pose.pose.position.y, 
                self.odom.pose.pose.position.z)
        ori = (self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w)
        self.br.sendTransform(pos,
                         ori,
                         self.odom.header.stamp,
                         "base_link",
                         "odom")

    def main(self):
        rospy.loginfo("Wheel Odometry Node started.")
        rate = rospy.Rate(50) 
        while not rospy.is_shutdown():
            self.create_base_link()
            self.publish_wheel_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdometry()
        wheel_odometry.main()
    except rospy.ROSInterruptException:
        pass

