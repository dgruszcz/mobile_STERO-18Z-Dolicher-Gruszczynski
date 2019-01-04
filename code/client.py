#!/usr/bin/env python

from nav_msgs.srv import *
from tf import *
import rospy
import PyKDL

def planingServer():
    rospy.wait_for_service('our_service')
    service = rospy.ServiceProxy('/our_service', ElektronSrv)
    pose = PoseStamped()
    while not rospy.is_shutdown():
        try:
            print 'Podaj pozycje zadana'
            pose.pose.position.y = input("Wprowadz x ")
            pose.pose.position.y = input("Wprowadz y ")
            theta = input("Wprowadz kat theta ")
            pose.pose.orientaion = PyKDL.RotZ(theta).GetQuaternion()
            resp = service(pose)
        except SyntaxError:
            sys.exit()  

if __name__ == "__main__":
    planingServer()