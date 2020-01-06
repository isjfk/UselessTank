#!/usr/bin/env python

import os
import math
import json
import yaml
import threading

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from urllib import pathname2url
gstlock = threading.Lock()
playbin = None

position = { 'x': 0, 'y': 0 , 'yaw': 0 }
path = []
mag = None
magScale = 0.05

navLock = threading.Lock()
moveBaseClient = None
tl = None
goalPub = None
initPosePub = None

def startListener():
    rospy.init_node('tank_webservice_node')
    rospy.Subscriber('/mag', MagneticField, magCallback)
    rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, footprintCallback)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, planCallback)

    global moveBaseClient
    moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    moveBaseClient.wait_for_server()

    global goalPub
    goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    global initPosePub
    initPosePub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    global tl
    tl = tf.TransformListener()

    playsoundInit()

    rospy.loginfo('[TankWebService] ROS Bridge start...')
    rospy.spin()

def magCallback(data):
    global mag
    global navLock
    with navLock:
        if (mag is None):
            mag = data
        else:
            mag.magnetic_field.x = mag.magnetic_field.x * (1 - magScale) + data.magnetic_field.x * magScale
            mag.magnetic_field.y = mag.magnetic_field.y * (1 - magScale) + data.magnetic_field.y * magScale
            mag.magnetic_field.z = mag.magnetic_field.z * (1 - magScale) + data.magnetic_field.z * magScale

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

        global navLock
        global position
        with navLock:
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

    global navLock
    global path
    with navLock:
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
    mapPath = rospy.get_param('~map_path', getPackagePath() + '/../tank_2dnav/map')
    return mapPath

def getPosition():
    global navLock
    global position

    with navLock:
        global mag
        currPosition = position

    return currPosition

def getPath():
    global navLock
    global path

    with navLock:
        currPath = path

    return currPath

def tankGoto(x, y, yaw):
    if (yaw is None):
        yaw = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    #pose = PoseStamped()
    #pose.header.stamp = rospy.Time.now()
    #pose.header.frame_id = 'map'
    #pose.pose.position.x = x
    #pose.pose.position.y = y
    #pose.pose.position.z = 0
    #pose.pose.orientation = Quaternion(*quat)

    #global goalPub
    #goalPub.publish(pose)

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation = Quaternion(*quat)

    global moveBaseClient
    moveBaseClient.send_goal(goal,
            active_cb = tankGotoActiveCallback,
            feedback_cb = tankGotoFeedbackCallback,
            done_cb = tankGotoDoneCallback)
    rospy.loginfo('[tank_webservice] Tank goto x[' + str(x) + '] y[' + str(y) + '] yaw[' + str(yaw) + ']')

def tankGotoActiveCallback():
    rospy.loginfo('[tank_webservice] Tank navigation started')
    playsound(getPackagePath() + '/resources/sound/transformers-autobots-roll-out.mp3')

def tankGotoFeedbackCallback(feedback):
    #rospy.loginfo('[tank_webservice] Tank navigation feedback: ' + str(feedback))
    pass

def tankGotoDoneCallback(state, result):
    rospy.loginfo('[tank_webservice] Tank navigation done: ' + str(state) + ' ' + str(result))
    playsound(getPackagePath() + '/resources/sound/transformers-stand.mp3')

def tankInitPose(x, y, yaw):
    global navLock
    global mag

    if (yaw is None):
        with navLock:
            if (mag is None):
                yaw = 0
            else:
                # Calculated yaw from compass is 0 = north, need to add (math.pi / 2) to convert to ROS north.
                yaw = -math.atan2(mag.magnetic_field.y, mag.magnetic_field.x) + (math.pi / 2)
                if (yaw < -math.pi):
                    yaw = yaw + math.pi*2
                elif (yaw > math.pi):
                    yaw = yaw - math.pi*2

    # Set yaw to 0 because compass in control board was interferenced. Delete following line after compass fixed.
    yaw = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0
    pose.pose.pose.orientation = Quaternion(*quat)

    global initPosePub
    initPosePub.publish(pose)
    rospy.loginfo('[tank_webservice] Set tank initial position to x[' + str(x) + '] y[' + str(y) + '] yaw[' + str(yaw) + ']')

def playsoundInit():
    Gst.init(None)
    global playbin
    playbin = Gst.ElementFactory.make('playbin', 'playbin')

def playsound(path):
    with gstlock:
        global playbin
        if path.startswith(('http://', 'https://')):
            playbin.props.uri = path
        else:
            playbin.props.uri = 'file://' + pathname2url(os.path.abspath(path))

        set_result = playbin.set_state(Gst.State.PLAYING)
        if set_result != Gst.StateChangeReturn.ASYNC:
            raise Exception("playbin.set_state returned " + repr(set_result))
        bus = playbin.get_bus()
        bus.poll(Gst.MessageType.EOS, Gst.CLOCK_TIME_NONE)
        playbin.set_state(Gst.State.NULL)

def getPackagePath():
    return os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/..')

