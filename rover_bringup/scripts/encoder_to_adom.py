#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import math

WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.08)
COUNTS_PER_REV = rospy.get_param('~cpr', 1024)
TRACK = rospy.get_param('~track', 0.7)

class EncoderOdom:
    def __init__(self):
        rospy.init_node('encoder_to_odom')
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=5)
        self.tf_b = tf.TransformBroadcaster()
        rospy.Subscriber('/encoders', Int32MultiArray, self.cb)
        self.last_counts = None
        self.x=0.0; self.y=0.0; self.th=0.0
        self.last_time = rospy.Time.now()

    def cb(self,msg):
        now = rospy.Time.now()
        if len(msg.data) < 6:
            rospy.logwarn('encoders msg length < 6')
            return
        counts = msg.data
        if self.last_counts is None:
            self.last_counts = counts
            self.last_time = now
            return
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        deltas = [counts[i]-self.last_counts[i] for i in range(6)]
        self.last_counts = counts
        left_counts = (deltas[0]+deltas[1]+deltas[2])/3.0
        right_counts = (deltas[3]+deltas[4]+deltas[5])/3.0
        left_rev = left_counts / COUNTS_PER_REV
        right_rev = right_counts / COUNTS_PER_REV
        left_dist = left_rev * 2*math.pi*WHEEL_RADIUS
        right_dist = right_rev * 2*math.pi*WHEEL_RADIUS
        d_center = (left_dist + right_dist)/2.0
        d_theta = (right_dist - left_dist) / TRACK
        dx = d_center * math.cos(self.th + d_theta/2.0)
        dy = d_center * math.sin(self.th + d_theta/2.0)
        self.x += dx
        self.y += dy
        self.th += d_theta
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0,0,self.th)
        odom.pose.pose.orientation = Quaternion(*quat)
        odom.twist.twist.linear.x = d_center/dt if dt>0 else 0.0
        odom.twist.twist.angular.z = d_theta/dt if dt>0 else 0.0
        self.pub.publish(odom)
        self.tf_b.sendTransform((self.x,self.y,0),(quat[0],quat[1],quat[2],quat[3]), now, 'base_link','odom')

if __name__=='__main__':
    EncoderOdom()
    rospy.spin()
