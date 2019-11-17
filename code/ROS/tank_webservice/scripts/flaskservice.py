#!/usr/bin/env python

import threading
import json
from flask import Flask
from flask import request
from flask_cors import CORS, cross_origin
from flask import send_file

import rosbridge

app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

flaskThread = None

@app.route("/")
def hello():
    return "Hello World!"

@app.route("/map")
def getMap():
    fileName = "sapnj.png"
    return send_file(fileName, attachment_filename='map.png')

@app.route("/locations")
def getLocations():
    data = """
    {
        "home":{"name":"Home", "x": "110", "y": "290"},
        "rooms":[     
                    { "name": "Meeting Room 3.04", "x": "200", "y": "290", "id": "mr1" },
                    { "name": "Meeting Room 3.06", "x": "110", "y": "490", "id": "mr2" },
                    { "name": "Meeting Room 3.05", "x": "230", "y": "490", "id": "mr3" }
                ]
    }
    """
    return data 

@app.route("/tank/position")
def getTankPosition():
    location = rosbridge.getPosition()
    return location

@app.route("/tank/path")
def getTankPath():
    path = rosbridge.getPath()
    return json.dumps(path)

@app.route("/tank/action/goto")
def setTankTarget():
    targetRoute = request.args.get('point')
    print(targetRoute)

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

