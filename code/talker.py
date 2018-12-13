#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import math

msg = Pose()

def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    msg=data

def ruch_do_pozycji():
    rospy.init_node('zadane_polozenie', anonymous=True)

#    rospy.Subscriber("/elektron/mobile_base_controller/odom", Odometry, callback)
    rospy.Subscriber("new_pose", Pose, callback)

    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    twist = Twist()
    twist.linear.x = 0.2
    kat = math.atan2(msg.position.y, msg.position.x)
	twist.angular.z = kat
    while not rospy.is_shutdown():
        for i in range(10): 
            pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0.2

    # ?????????????????????
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    try:
        ruch_do_pozycji()
    except rospy.ROSInterruptException:
        pass