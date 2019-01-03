#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D, Vector3, Pose
from nav_msgs.msg import Odometry
import math
import rosbag

pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
pubError = rospy.Publisher('errors', Float64, queue_size=10)
bag = rosbag.Bag('tuneTL.bag', 'w')
currentPosition = Pose()
realPosition = Pose()

def moveToGoal(goal, moveSpeed=0.3, frequency=50):
    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    twist.linear.x = moveSpeed
    rate = rospy.Rate(frequency)  # Nadawanie ze stala czestotliwoscia

    # Sprawdzenie czy robot osiagnal zadane polozenie odbywa sie poprzez sprawdzenie czy poprzednia odleglosc
    # robota od polozenia zadanego jest mniejsza od aktualnej (tzn czy robot minal cel)
    lastDistance = math.sqrt(math.pow(goal.y - currentPosition.position.y, 2) +
                             math.pow(goal.x - currentPosition.position.x, 2))
    while True:
        twist.linear.x = moveSpeed  # 0.3m/s
        distToTarget = math.sqrt(math.pow(goal.y - currentPosition.position.y, 2) +
                                 math.pow(goal.x - currentPosition.position.x, 2))
        if lastDistance - distToTarget < -0.01:
            break
        pub.publish(twist)
        lastDistance = distToTarget
        rate.sleep()

    # Robot osiagnal zadane polozenie
    twist.linear.x = 0.0
    pub.publish(twist)



""" Funkcja sluzaca do odbierania wiadomosci z tematu /elektron/mobile_base_controller/odom. """
def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition.pose.pose

""" Funkcja sluzaca do odbierania wiadomosci z tematu /gazebo_odom. """
def getRealPosition(newPosition):
    global realPosition
    realPosition = newPosition.pose.pose
    blad = Float64()
    blad.data = math.sqrt(pow(realPosition.position.x - currentPosition.position.x,2) +
                     pow(realPosition.position.y - currentPosition.position.y,2))
    pubError.publish(blad)
    bag.write('/errors', blad)
    bag.write('/gazebo_odom', Pose2D(realPosition.position.x, realPosition.position.y, 0))
    bag.write('/enkoder', Pose2D(currentPosition.position.x, currentPosition.position.y, 0))


if __name__ == '__main__':
    try:
        rospy.init_node('inter_odo', anonymous=True)
        rospy.Subscriber("elektron/mobile_base_controller/odom", Odometry, getCurrentPosition)
        rospy.Subscriber("gazebo_odom", Odometry, getRealPosition)
        rospy.sleep(1)
        print "Ruch do pozycji x: 1, y: 0 i z powrotem do x: 0, y: 0."
        # Ruch do pozycji x: 1, y: 0
        moveToGoal(Pose2D(1, 0, 0), moveSpeed=0.3)
        rospy.sleep(1)
        # Ruch do pozycji x: 0, y: 0
        moveToGoal(Pose2D(0, 0, 0), moveSpeed=-0.3)
        bag.close()
        exit(0)

    except rospy.ROSInterruptException:
        pass