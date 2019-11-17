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

@app.route("/")
def hello():
    return "Hello World!"

@app.route("/map")
def getMap():
    return send_file('sapnj.png', attachment_filename='python.png')

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

@app.route("/tank")
def getTankLocation():
    data = """
    {
        "tank":{"x":95,"y":165}
    }
    """
    return data
  

@app.route("/getRoute")
def getRoute():
    targetRoute = request.args.get('target')
    print(targetRoute)
    data = """
    {
        "routes":[     
                    { "x": "200", "y": "290", "id": "r1" },
                    { "x": "110", "y": "490", "id": "r2" },
                    { "x": "230", "y": "490", "id": "r3" }
                ]
    }
    """
    return data    

def flaskWorker():
    app.run(host='0.0.0.0')

def startFlask():

