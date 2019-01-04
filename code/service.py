#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from stero_mobile_init.srv import ElektronSrv
from nav_msgs.srv import GetPlan
import tf

pub = rospy.Publisher('/created_paths', Path, queue_size=10)

def createPlan(req):
    planer_server = rospy.ServiceProxy('/global_plnaer/planer/make_plan', GetPlan)
    pose = PoseStamped()
    tolerance = 0.0
    pose.header.frame_id = '/base_link'
    solver = tf.TransformerROS()
    plan = planer_server(solver.transformPose('/map', pose), req.goal)
    pub.publish(plan)
    return True

def init():
    rospy.init_node('planner')
    s = rospy.Service('/our_service', ElektronSrv, createPlan)
    print "Ready to plan."
    rospy.spin()

if __name__ == "__main__":
    init()