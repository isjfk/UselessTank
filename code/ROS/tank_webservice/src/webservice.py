#!/usr/bin/env python

import json
import rosbridge
import flaskservice

if __name__ == "__main__":
    flaskservice.startFlask()
    rosbridge.startListener()

