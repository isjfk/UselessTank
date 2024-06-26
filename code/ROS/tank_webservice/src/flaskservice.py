#!/usr/bin/env python3

import sys
import os
import math
import json
import traceback
import threading

from flask import Flask
from flask import request
from flask import jsonify
from flask import send_file
from flask_cors import CORS, cross_origin
from werkzeug.exceptions import HTTPException

import rosbridge

app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

flaskThread = None

@app.errorhandler(Exception)
def handle_exception(e):
    traceback.print_exception(type(e), e, e.__traceback__)
    return ('', 500)

@app.route('/')
def hello():
    return 'Tank WebService'

@app.route('/map/image')
def getMap():
    filePath = getMapFilePath()
    return send_file(filePath, mimetype='image/png', attachment_filename='map.png', cache_timeout=0)

def getMapFilePath():
    mapMeta = rosbridge.getMapMeta()
    return mapMeta.get('imagePath')

@app.route('/map/meta')
def getMapMeta():
    mapMeta = rosbridge.getMapMeta()
    del mapMeta['imagePath']
    return jsonify(mapMeta)

@app.route('/poi')
def getPoiList():
    filePath = getPoiListFilePath()
    return send_file(filePath, mimetype='application/json', attachment_filename='poiList.json', cache_timeout=0)

def getPoiListFilePath():
    return getPackagePath() + '/resources/poiList.json'

@app.route('/tank/position')
def getTankPosition():
    position = rosbridge.getPosition()
    return jsonify(position)

@app.route('/tank/path')
def getTankPath():
    path = rosbridge.getPath()
    return jsonify(path)

@app.route('/tank/all')
def getTankAll():
    position = rosbridge.getPosition()
    path = rosbridge.getPath()
    allData = {
        'position': position,
        'path': path
    }
    return jsonify(allData)

@app.route('/tank/action/goto', methods=['GET', 'POST'])
def tankGoto():
    pose = request.get_json()
    if (pose is not None):
        x = pose.get('x')
        y = pose.get('y')
        yaw = pose.get('yaw')
    else:
        x = float(request.args.get('x'))
        y = float(request.args.get('y'))
        yaw = float(request.args.get('yaw'))
    if (x is not None) and (y is not None):
        if (yaw is None):
            yaw = calcYawToClosestPoi(x, y)
        rosbridge.tankGoto(x, y, yaw)
    return ('', 204)

@app.route('/tank/action/initPose', methods=['GET', 'POST'])
def tankInitPose():
    pose = request.get_json()
    if (pose is not None):
        x = pose.get('x')
        y = pose.get('y')
        yaw = pose.get('yaw')
    else:
        x = float(request.args.get('x'))
        y = float(request.args.get('y'))
        yaw = float(request.args.get('yaw'))
    if (x is not None) and (y is not None):
        rosbridge.tankInitPose(x, y, yaw)
    return ('', 204)

@app.route('/tank/action/clearCostmaps', methods=['GET', 'POST'])
def tankClearCostmaps():
    rosbridge.tankClearCostmaps()
    return ('', 204)

def calcYawToClosestPoi(x, y):
    poiListPath = os.path.abspath(getPoiListFilePath())
    with open(poiListPath, 'r', encoding='utf-8') as poiListFile:
        poiList = json.load(poiListFile)
    yaw = 0
    minDist = sys.float_info.max
    for poi in poiList:
        poiX = poi.get('x')
        poiY = poi.get('y')
        poiDist = math.sqrt(math.pow(poiX - x, 2) + math.pow(poiY - y, 2))
        if (poiDist < minDist):
            minDist = poiDist
            yaw = math.atan2(poiY - y, poiX - x)
    return yaw

def flaskWorker():
    app.run(host='0.0.0.0')

def startFlask():
    global flaskThread
    flaskThread = threading.Thread(target=flaskWorker)
    flaskThread.setDaemon(True)
    flaskThread.start()

def waitFlaskExit():
    global flaskThread
    flaskThread.join()

def getPackagePath():
    return os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/..')

