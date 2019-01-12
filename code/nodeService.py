#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Pose2D, Twist, Vector3
from nav_msgs.msg import Odometry
from stero_mobile_init.srv import ElektronSrv
from nav_msgs.srv import GetPlan, GetPlanResponse
import tf
import math

pub = rospy.Publisher('/created_paths', GetPlanResponse, queue_size=10)
currentPosition = Pose()


def createPlan(req):
    planer_server = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
    pose = PoseStamped()
    tolerance = 0.0
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time(0)
    pose.pose = currentPosition
    #solver = tf.TransformerROS()
    #pose = solver.transformPose('map', pose)
    state = planer_server(pose, req.goal, tolerance)
    print "Trajectory executed successfully."
    return True

""" Funkcja sluzaca do odbierania wiadomosci z tematu /elektron/mobile_base_controller/odom w celu pobrania aktualnej pozycji robota. """
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
