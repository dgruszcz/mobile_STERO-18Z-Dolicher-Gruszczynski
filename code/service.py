#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Pose2D, Twist, Vector3
from nav_msgs.msg import Odometry
from stero_mobile_init.srv import ElektronSrv
from nav_msgs.srv import GetPlan, GetPlanResponse
import tf
import math

pub1 = rospy.Publisher('/created_paths', GetPlanResponse, queue_size=10)
pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
currentPosition = Pose()

def getRotZ(q):
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def createPlan(req):
    planer_server = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    pose = PoseStamped()
    tolerance = 0.0
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time(0)
    pose.pose = currentPosition
    #solver = tf.TransformerROS()
    #pose = solver.transformPose('map', pose)
    plan = planer_server(pose, req.goal, tolerance)
    for i in range(2, len(plan.plan.poses), 3):
        pub1.publish(plan)
        # Predkosc ruchu
        moveSpeed = 0.3

        # Przeliczanie pozycji zadanej do ukladu wspolrzednych robota (zakladamy ze robot znajduje sie w pozycji (0,0,0)
        goal = Pose2D()
        goal.x = plan.plan.poses[i].pose.position.x
        goal.y = plan.plan.poses[i].pose.position.y

        # Obliczanie kata do jakiego musi sie ustawic robot
        kat = math.atan2(goal.y - currentPosition.position.y, goal.x - currentPosition.position.x)

        diff = kat - getRotZ(currentPosition.orientation)

        diff += -2 * math.pi if diff > math.pi else (2 * math.pi if diff < -math.pi else 0)

        if diff < 0:
            rotSpeed = -0.4
        else:
            rotSpeed = 0.4

        # Tworzenie pustej wiadomosci
        twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        frequency = 50  # Czestotliwosc z jaka bedziemy nadawac zadana predkosc
        rate = rospy.Rate(frequency)  # 20hz
        while True:
            twist.angular.z = rotSpeed

            # Pobieranie aktualnego obrotu robota i spradzenie czy osiagnal on zadany obrot (z zadana tolerancja)
            katRobota = getRotZ(currentPosition.orientation)
            if math.fabs(kat - katRobota) < 0.05:
                break
            pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0

        # Sprawdzenie czy robot osiagnal zadane polozenie odbywa sie poprzez sprawdzenie czy poprzednia odleglosc
        # robota od polozenia zadanego jest mniejsza od aktualnej (tzn czy robot minal cel)
        lastDistance = math.sqrt(math.pow(goal.y - currentPosition.position.y, 2) +
                                 math.pow(goal.x - currentPosition.position.x, 2))
        while True:
            twist.linear.x = moveSpeed  # 0.3m/s
            distToTarget = math.sqrt(math.pow(goal.y - currentPosition.position.y, 2) +
                                     math.pow(goal.x - currentPosition.position.x, 2))
            if lastDistance - distToTarget < 0:
                break
            pub.publish(twist)
            lastDistance = distToTarget
            rate.sleep()

        # Robot osiagnal zadane polozenie - koniec
        twist.linear.x = 0.0
        pub.publish(twist)
    return True

""" Funkcja sluzaca do odbierania wiadomosci z tematu /elektron/mobile_base_controller/odom. """
def getCurrentPosition(newPosition):
    global currentPosition
    currentPosition = newPosition.pose.pose

def init():
    rospy.init_node('planner')
    rospy.Subscriber("elektron/mobile_base_controller/odom", Odometry, getCurrentPosition)
    s = rospy.Service('/our_service', ElektronSrv, createPlan)
    print "Ready to plan."
    rospy.spin()

if __name__ == "__main__":
    init()
