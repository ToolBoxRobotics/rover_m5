#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import math

WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.08)
TRACK = rospy.get_param('~track', 0.7)
WHEELBASE = rospy.get_param('~wheelbase', 0.9)

pub_fl_steer = None
pub_fr_steer = None
pub_rl_steer = None
pub_rr_steer = None

pub_fl_vel = None
pub_ml_vel = None
pub_rl_vel = None
pub_fr_vel = None
pub_mr_vel = None
pub_rr_vel = None


def twist_cb(msg):
    v = msg.linear.x
    omega = msg.angular.z
    if abs(omega) < 1e-6:
        steer = 0.0
    else:
        R = v / omega
        steer = math.atan(WHEELBASE / R)
    publish_all(v, steer)


def ack_cb(msg):
    v = msg.drive.speed
    steer = msg.drive.steering_angle
    publish_all(v, steer)


def publish_all(v, steer):
    wheel_omega = v / WHEEL_RADIUS
    pubs_w = [pub_fl_vel,pub_ml_vel,pub_rl_vel,pub_fr_vel,pub_mr_vel,pub_rr_vel]
    for p in pubs_w:
        p.publish(Float64(wheel_omega))
    pub_fl_steer.publish(Float64(steer))
    pub_fr_steer.publish(Float64(steer))
    pub_rl_steer.publish(Float64(steer))
    pub_rr_steer.publish(Float64(steer))


def main():
    global pub_fl_steer,pub_fr_steer,pub_rl_steer,pub_rr_steer
    global pub_fl_vel,pub_ml_vel,pub_rl_vel,pub_fr_vel,pub_mr_vel,pub_rr_vel
    rospy.init_node('ackermann_adapter')
    pub_fl_steer = rospy.Publisher('/fl_steer_controller/command', Float64, queue_size=1)
    pub_fr_steer = rospy.Publisher('/fr_steer_controller/command', Float64, queue_size=1)
    pub_rl_steer = rospy.Publisher('/rl_steer_controller/command', Float64, queue_size=1)
    pub_rr_steer = rospy.Publisher('/rr_steer_controller/command', Float64, queue_size=1)

    pub_fl_vel = rospy.Publisher('/fl_wheel_controller/command', Float64, queue_size=1)
    pub_ml_vel = rospy.Publisher('/ml_wheel_controller/command', Float64, queue_size=1)
    pub_rl_vel = rospy.Publisher('/rl_wheel_controller/command', Float64, queue_size=1)
    pub_fr_vel = rospy.Publisher('/fr_wheel_controller/command', Float64, queue_size=1)
    pub_mr_vel = rospy.Publisher('/mr_wheel_controller/command', Float64, queue_size=1)
    pub_rr_vel = rospy.Publisher('/rr_wheel_controller/command', Float64, queue_size=1)

    rospy.Subscriber('/cmd_vel', Twist, twist_cb)
    rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, ack_cb)
    rospy.spin()

if __name__=='__main__':
    main()
