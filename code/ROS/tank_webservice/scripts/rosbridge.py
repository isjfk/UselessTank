#!/usr/bin/env python

import threading
import math
import json

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped

lock = threading.Lock()
position = { "x": 0, "y": 0 }
path = {}
target = None

tl = None

def startListener():
    rospy.init_node("tank_webservice")
    rospy.Subscriber("/move_base/global_costmap/footprint", PolygonStamped, footprintCallback)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, planCallback)

    global tl
    tl = tf.TransformListener()

    rospy.loginfo("[TankWebService] ROS Bridge start...")
    rospy.spin()

def footprintCallback(data):
    global tl

    now = rospy.Time.now()
    # Wait for transform to avoid TransformException: Lookup would require extrapolation into the past
    try:
        pos, quat = tl.lookupTransform("map", "base_link", rospy.Time(0))

        point = {}
        point["x"] = pos[0]
        point["y"] = pos[1]

        global lock
        global position
        with lock:
            position = point
    except:
        rospy.loginfo("Transform lookup failed, defer to next loop. It happens on startup.")

def planCallback(data):
    p = []

    for pose in data.poses:
        point = {}
        point["x"] = pose.pose.position.x
        point["y"] = pose.pose.position.y
        if (len(p) == 0) or (calcDistance(p[len(p) - 1], point) >= 0.1):
            p.append(point)

    if (len(p) > 0):
        lastPoint = p[len(p) - 1]
        lastPose = data.poses[len(data.poses) - 1]
        if (lastPoint["x"] != lastPose.pose.position.x) or (lastPoint["y"] != lastPose.pose.position.y):
            point = {}
            point["x"] = lastPose.pose.position.x
            point["y"] = lastPose.pose.position.y
            p.append(point)

    global lock
    global path
    with lock:
        path = p

def calcDistance(point1, point2):
    x1 = point1["x"]
    x2 = point2["x"]
    y1 = point1["y"]
    y2 = point2["y"]
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

def getPosition():
    global lock
    global position

    with lock:
        currPosition = position

    return currPosition

def getPath():
    global lock
    global path

    with lock:
        currPath = path

    return currPath

def setTarget(newTarget):
    global lock
    global target

    with lock:
        target = newTarget

