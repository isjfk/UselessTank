from flask import Flask
from flask_cors import CORS, cross_origin
from flask import send_file
import json

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
                    { "name": "Meeting Room 3.04", "x": "200", "y": "290", "id": "r1" },
                    { "name": "Meeting Room 3.06", "x": "110", "y": "490", "id": "r2" },
                    { "name": "Meeting Room 3.05", "x": "230", "y": "490", "id": "r3" }
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
  

@app.route("/calcuateRoute")
def getRoute():
    return ""    


if __name__ == "__main__":
    app.run()