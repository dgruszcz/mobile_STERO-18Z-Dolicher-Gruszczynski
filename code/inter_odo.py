#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D, Vector3, Pose
from nav_msgs.msg import Odometry
import math
import tf_conversions
import PyKDL

pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
currentPosition = Pose()


def callback(data):
    rospy.loginfo("Dobra jedziem. Pozycja docelowa (%f, %f)", data.x, data.y)

    frequency = 20  # Czestotliwosc z jaka bedziemy nadawac zadana predkosc
    rate = rospy.Rate(frequency)  # 20hz
    constSpeed = 0.3

    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    kat = math.atan2(data.y - currentPosition.position.y, data.x - currentPosition.position.x)
    while True:
        twist.angular.z = constSpeed
        rotX, rotY, katRobota = PyKDL.Rotation().Quaternion(currentPosition.orientation.x, currentPosition.orientation.y,
                                                 currentPosition.orientation.z, currentPosition.orientation.w).GetRPY()
        if math.fabs(kat - katRobota) < 0.05:
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0

    lastDistance = distToTarget = math.sqrt(math.pow(data.y - currentPosition.position.y,2) + math.pow(data.x - currentPosition.position.x,2))

    while True:
        twist.linear.x = constSpeed  # 0.3m/s
        distToTarget = math.sqrt(math.pow(data.y - currentPosition.position.y,2) + math.pow(data.x - currentPosition.position.x,2))
        if lastDistance - distToTarget < 0:
            break
        pub.publish(twist)
        lastDistance = distToTarget
        rate.sleep()
    twist.linear.x = 0.0
    pub.publish(twist)


def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition.pose.pose



rospy.init_node('inter_odo', anonymous=True)
rospy.Subscriber("new_pose", Pose2D, callback)
rospy.Subscriber("elektron/mobile_base_controller/odom", Odometry, getCurrentPosition)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rospy.loginfo(rospy.get_caller_id() + "Dawaj pozycje bo czekam")
            rospy.spin()
    except rospy.ROSInterruptException:
        pass