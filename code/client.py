#!/usr/bin/env python

from nav_msgs.srv import *
import rospy
import PyKDL
from stero_mobile_init.srv import ElektronSrv
from geometry_msgs.msg import PoseStamped

def planingServer():
    rospy.wait_for_service('our_service')
    service = rospy.ServiceProxy('/our_service', ElektronSrv)
    pose = PoseStamped()
    while not rospy.is_shutdown():
        try:
            print 'Podaj pozycje zadana'
            pose.pose.position.x = input("Wprowadz x ")
            pose.pose.position.y = input("Wprowadz y ")
            theta = input("Wprowadz kat theta ")
            quat = PyKDL.Rotation().RotZ(theta).GetQuaternion()
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time(0)
            resp = service(pose)
        except SyntaxError:
            sys.exit()  

if __name__ == "__main__":
    planingServer()