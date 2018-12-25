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

def moveToGoal(data, moveSpeed=0.3, frequency=20):
    print "Dobra jedziem. Pozycja docelowa: ", data.x, data.y

    # Wybor kierunku obrotu
    if data.y > 0:
        rotSpeed = 0.3
    else:
        rotSpeed = -0.3
    # Predkosc ruchu
    moveSpeed = 0.3

    # Przeliczanie pozycji zadanej do ukladu wspolrzednych robota (zakladamy ze robot znajduje sie w pozycji (0,0,0)
    goal = Pose2D()
    katRobota = getRotZ(currentPosition.orientation)
    goal.x = data.x * math.cos(katRobota) - data.y * math.sin(katRobota) + currentPosition.position.x
    goal.y = data.x * math.sin(katRobota) + data.y * math.cos(katRobota) + currentPosition.position.y

    # Obliczanie kata do jakiego musi sie ustawic robot
    kat = math.atan2(goal.y - currentPosition.position.y, goal.x - currentPosition.position.x)

    # Tworzenie pustej wiadomosci
    twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    frequency = 20  # Czestotliwosc z jaka bedziemy nadawac zadana predkosc
    rate = rospy.Rate(frequency)  # 20hz
    while True:
        twist.angular.z = rotSpeed

        # Pobieranie aktualnego obrotu robota i spradzenie czy osiagnal on zadany obrot (z zadana tolerancja)
        katRobota = getRotZ(currentPosition.orientation)
        if math.fabs(kat - katRobota) < 0.03:
            break
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    rospy.sleep(1)

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

    # Robot osiagnal zadane polozenie - koniec
    twist.linear.x = 0.0
    pub.publish(twist)



""" Funkcja sluzaca do odbierania wiadomosci z tematu /elektron/mobile_base_controller/odom. """
def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition.pose.pose



rospy.init_node('inter_odo', anonymous=True)
rospy.Subscriber("elektron/mobile_base_controller/odom", Odometry, getCurrentPosition)

if __name__ == '__main__':
    try:
        print "Ruch po kwadracie o boku 1"

        # Ruch do pozycji x: 1, y: 0
        moveToGoal(Pose2D(1, 0, 0), moveSpeed=0.3)
        print 'Wracam'
        rospy.sleep(1)
        # Ruch do pozycji x: 1, y: 1
        moveToGoal(Pose2D(0, 1, 0), moveSpeed=0.3)
        rospy.sleep(1)
        # Ruch do pozycji x: 0, y: 1
        moveToGoal(Pose2D(0, 1, 0), moveSpeed=0.3)
        rospy.sleep(1)
        # Ruch do pozycji x: 0, y: 0
        moveToGoal(Pose2D(0, 1, 0), moveSpeed=0.3)
        exit(0)

    except rospy.ROSInterruptException:
        pass