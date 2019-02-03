#!/usr/bin/env python
from websocket import create_connection
import json
import subprocess

"""
Prerequisites:
Choco - https://chocolatey.org/install#install-with-cmdexe
Python3 - choco install python
Websocket - pip install websocket-client
"""

ip = "10.8.30.11"
directory = "RaspberryPiCamera"
command = "cd {};make clean;make install".format(directory)

print("Connecting to Raspberry Pi Server @ {}".format(ip))
connection = create_connection("ws://" + ip)
print("Disabling Camera")
connection.send(json.dumps({"data":"visionDown"}))
print("Enabling write")
connection.send(json.dumps({"data":"systemWritable"}))
print("Sending *.cpp & *.h files to Raspberry Pi/{}".format(directory))
subprocess.run(["scp", "*.cpp", "*.h", "pi@{}:{}".format(ip, directory)])
print("Connecting to Pi & Building Code")
subprocess.run(["ssh", "-t", "pi@{}".format(ip), "'{}'".format(command)])    
print("Re-Enabling Camera")
connection.send(json.dumps({"data":"visionUp"}))



