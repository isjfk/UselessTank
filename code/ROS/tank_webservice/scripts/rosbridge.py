#!/usr/bin/env python

import os
import math
import json
import yaml
import threading

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion

position = { 'x': 0, 'y': 0 , 'yaw': 0 }
path = {}

lock = threading.Lock()
tl = None
goalPub = None
initPosePub = None

def startListener():
    rospy.init_node('tank_webservice_node')
    rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, footprintCallback)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, planCallback)

    global goalPub
    goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    global initPosePub
    initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    global tl
    tl = tf.TransformListener()

    rospy.loginfo('[TankWebService] ROS Bridge start...')
    rospy.spin()

def footprintCallback(data):
    global tl

    now = rospy.Time.now()
    # Wait for transform to avoid TransformException: Lookup would require extrapolation into the past
    try:
        pos, quat = tl.lookupTransform('map', 'base_link', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(quat)

        tankPos = {}
        tankPos['x'] = pos[0]
        tankPos['y'] = pos[1]
        tankPos['yaw'] = math.degrees(euler[2])

        global lock
        global position
        with lock:
            position = tankPos
    except:
        rospy.loginfo('Transform lookup failed, defer to next loop. It happens on startup.')

def planCallback(data):
    p = []

    for pose in data.poses:
        point = {}
        point['x'] = pose.pose.position.x
        point['y'] = pose.pose.position.y
        if (len(p) == 0) or (calcDistance(p[len(p) - 1], point) >= 0.1):
            p.append(point)

    if (len(p) > 0):
        lastPoint = p[len(p) - 1]
        lastPose = data.poses[len(data.poses) - 1]
        if (lastPoint['x'] != lastPose.pose.position.x) or (lastPoint['y'] != lastPose.pose.position.y):
            point = {}
            point['x'] = lastPose.pose.position.x
            point['y'] = lastPose.pose.position.y
            p.append(point)

    global lock
    global path
    with lock:
        path = p

def calcDistance(point1, point2):
    x1 = point1['x']
    x2 = point2['x']
    y1 = point1['y']
    y2 = point2['y']
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

def getMapMeta():
    mapPath = getMapPath()
    metaPath = os.path.abspath(mapPath + '/map.yaml')
    meta = yaml.full_load(file(metaPath, 'r'))
    meta['imagePath'] = mapPath + '/' + meta['image']
    return meta

def getMapPath():
    mapPath = rospy.get_param('~map_path', getPackagePath() + '../tank_2dnav/map')
    print('map_path: ' + rospy.get_param('~map_path'))
    return mapPath

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

def tankGoto(x, y, yaw):
    if (yaw is None):
        yaw = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    pose.pose.orientation = Quaternion(*quat)

    global goalPub
    goalPub.publish(pose)

def getPackagePath():
    return os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/..')

