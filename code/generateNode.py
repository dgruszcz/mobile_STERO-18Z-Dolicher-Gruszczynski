#!/usr/bin/env python

from nav_msgs.srv import *
import rospy

def handle_add_two_ints(req):

	act_pos = 
    planer_msg = rospy.ServiceProxy('/global_plnaer/planer/make_plan', GetPlan)
	pose = PoseStamped()
	pose.header.frame_id = "base_link"
    plan = planer_msg( transformPose( "map", pose), ElektronSrv.goal)
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('node_pos')
    s = rospy.Service('moveToXY', ElektronSrv, handle_add_two_ints)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()