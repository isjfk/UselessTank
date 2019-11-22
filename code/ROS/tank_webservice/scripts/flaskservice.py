#!/usr/bin/env python

import traceback
import threading
import json
from flask import Flask
from flask import request
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
    traceback.print_exc(e)
    return ('', 500)

@app.route('/')
def hello():
    return 'Tank WebService'

@app.route('/map')
def getMap():
    fileName = 'sapnj.png'
    return send_file(fileName, mimetype='image/png', attachment_filename='map.png')

@app.route('/poi')
def getPoiList():
    fileName = getPoiListFilePath()
    return send_file(fileName, mimetype='application/json', attachment_filename='poiList.json')

@app.route('/tank/position')
def getTankPosition():
    location = rosbridge.getPosition()
    return location

@app.route('/tank/path')
def getTankPath():
    path = rosbridge.getPath()
    return json.dumps(path)

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
        rosbridge.tankGoto(x, y, yaw)
    return ('', 204)

def getPoiListFilePath():
    return '../resources/poiList.json'

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

