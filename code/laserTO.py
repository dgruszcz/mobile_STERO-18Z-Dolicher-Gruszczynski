#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D, Vector3, Pose
from nav_msgs.msg import Odometry
import math
import rosbag

pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
pubError = rospy.Publisher('errors', Float64, queue_size=10)
bag = rosbag.Bag('laserTO_odom_tune.bag', 'w')
currentPosition = Pose2D()
realPosition = Pose2D()

def getRotZ(q):
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def do360(rotSpeed=0.3, frequency=50):
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
        katRobota = currentPosition.theta
        if math.fabs(0.0 - katRobota) < 0.01:
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    pub.publish(twist)



""" Funkcja sluzaca do odbierania wiadomosci z tematu /pose2D oraz wysyla aktualny blad polozenia do tematu /errors """
def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition


""" Funkcja sluzaca do odbierania wiadomosci z tematu /gazebo_odom. """
def getRealPosition(newPosition):
    global realPosition
    realPosition.x = newPosition.pose.pose.position.x
    realPosition.y = newPosition.pose.pose.position.y
    realPosition.theta = getRotZ(newPosition.pose.pose.orientation)
    blad = Float64()
    blad.data = math.fabs(realPosition.theta - currentPosition.theta)
    pubError.publish(blad)
    bag.write('/errors', blad)
    bag.write('/gazebo_odom', Pose2D(realPosition.x, realPosition.y, realPosition.theta))
    bag.write('/enkoder', Pose2D(currentPosition.x, currentPosition.y, currentPosition.theta))


if __name__ == '__main__':
    try:
        rospy.init_node('inter_odo', anonymous=True)
        rospy.Subscriber("pose2D", Pose2D, getCurrentPosition)
        rospy.Subscriber("gazebo_odom", Odometry, getRealPosition)
        rospy.sleep(1)
        print "Wykonanie pelnego obrotu."
        # Obrot (odczekiwanie odpowiedniego czasu + sprawdzenie)
        do360(rotSpeed=0.3)
        rospy.sleep(1.5)
    except rospy.ROSInterruptException:
        pass
    finally:
        bag.close()
