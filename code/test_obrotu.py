#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D, Vector3, Pose
from nav_msgs.msg import Odometry
import math

pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
currentPosition = Pose()

def getRotZ(q):
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def rotate(kat, rotSpeed=0.3, frequency=20):
    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    twist.angular.z = rotSpeed
    rate = rospy.Rate(frequency)  # Nadawanie ze stala czestotliwoscia

    while True:
        twist.angular.z = rotSpeed

        # Pobieranie aktualnego obrotu robota i spradzenie czy osiagnal on zadany obrot (z zadana tolerancja)
        katRobota = getRotZ(currentPosition.orientation)
        if math.fabs(kat - katRobota) < 0.03:
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    pub.publish(twist)

def do360(rotSpeed=0.3, frequency=20):
    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    twist.angular.z = rotSpeed

    # Trzeba chwile poczekac, zeby robot mial szanse sie calkowicie obrocic, tzn by nastepna petla nie zatrzymala sie od razu
    iterationsToSkip = 10

    rate = rospy.Rate(frequency)  # Nadawanie ze stala czestotliwoscia

    for i in range(iterationsToSkip):
        pub.publish(twist)
        rate.sleep()


    while True:
        twist.angular.z = rotSpeed

        # Pobieranie aktualnego obrotu robota i spradzenie czy osiagnal on zadany obrot (z zadana tolerancja)
        katRobota = getRotZ(currentPosition.orientation)
        if math.fabs(0.0 - katRobota) < 0.01:
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    pub.publish(twist)



""" Funkcja sluzaca do odbierania wiadomosci z tematu /elektron/mobile_base_controller/odom. """
def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition.pose.pose



rospy.init_node('inter_odo', anonymous=True)
rospy.Subscriber("elektron/mobile_base_controller/odom", Odometry, getCurrentPosition)

if __name__ == '__main__':
    try:
        print "Wykonanie pelnego obrotu."

        # Robot obsluguje tylko katy od 0 do +/- pi, dlatego nie mozna wstawic np. 2pi
        #rotate(-0.05, rotSpeed=0.3)

        # Obrot (odczekiwanie odpowiedniego czasu + sprawdzenie)
        do360(rotSpeed=0.3)
        exit(0)

    except rospy.ROSInterruptException:
        pass