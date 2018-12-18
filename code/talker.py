#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D, Vector3

import math

pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Dobra jedziem. Pozycja docelowa (%f, %f)", data.x, data.y)

    frequency = 20
    rate = rospy.Rate(frequency) # 50hz
    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    kat = math.atan2(data.y, data.x)


    constSpeed = 0.3

    twist.angular.z = constSpeed
    totalRotation = kat
    rotationTime = math.fabs(totalRotation/constSpeed) # Number of required seconds for travel
    iterationsTillRotated = int(rotationTime*frequency)
    if(iterationsTillRotated == 0):
        iterationsTillRotated = 1

    totalDistance = math.sqrt(pow(data.x, 2) + pow(data.y, 2))
    travelTime = totalDistance/constSpeed # Number of required seconds for travel
    iterationsTillGoal = int(travelTime*frequency)
    totalIterations = iterationsTillGoal + iterationsTillRotated
    print iterationsTillRotated, iterationsTillGoal, totalIterations
    for i in range(totalIterations):
        if i == iterationsTillRotated -1:
            twist.angular.z = 0.0
            twist.linear.x = constSpeed  # 0.1m/s
        pub.publish(twist)
        rate.sleep()
    twist.linear.x = 0.0
    pub.publish(twist)
    rate.sleep()


rospy.init_node('zadane_polozenie', anonymous=True)
rospy.Subscriber("new_pose", Pose2D, callback)



    

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            rospy.loginfo(rospy.get_caller_id() + "Dawaj pozycje bo czekam")
            rospy.spin()
    except rospy.ROSInterruptException:
        pass