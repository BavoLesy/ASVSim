import setup_path
import cosysairsim as airsim

import time
from cosysairsim.types import VesselControls, DisturbanceControls, Pose, Quaternionr, Vector3r, Vector2r
"""
This script places generates a port terrain and then spawns a few obstacles and then tests some disturbances. It is important that the PCG and PCGGeometryScriptInterop Plugins are enabled in the .uproject file. It is advised to test this PCG in an empty level. For more information on, please refer to the documentation. 
"""
# connect to the AirSim simulator 
client = airsim.VesselClient(ip="127.0.0.1") #to find this Host IP, see wsl_instructions.md  172.22.224.1 172.19.64.1
client.confirmConnection()
client.enableApiControl(True)

client.activateGeneration(True)
client.generatePortTerrain("port", -464588337, 10)

client.setDisturbanceControls("", DisturbanceControls(1000, 0, 0, 0, 0, 0))

vec = Vector3r()
quat = Quaternionr(0, 0, 0.7071, 0.7071)
pose = Pose(vec,quat)
client.simAddObstacle(Pose(),500)
client.simAddObstacle(Pose(Vector3r(0,0,190),Quaternionr(0,0,1,0)),1000,"")

client.simAddObstacle(Pose(Vector3r(-10265.0,-610.0,250),Quaternionr()),0,"buoy")
client.simAddObstacle(Pose(Vector3r(-7940.0,-280.0,250),Quaternionr()),0,"buoy")


time.sleep(3)
client.setVesselControls('', VesselControls(1, .5))
time.sleep(13)
client.setDisturbanceControls("", DisturbanceControls(1000, 3.14, 0, 0, 0, 0))
time.sleep(5)
client.setDisturbanceControls("", DisturbanceControls(0, 0, 0, 0, 0, 0))
time.sleep(5)
client.setDisturbanceControls("", DisturbanceControls(1000, 0, 0, 0, 0, 0))


