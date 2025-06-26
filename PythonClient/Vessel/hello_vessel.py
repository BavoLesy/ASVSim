"""
For connecting to the AirSim drone environment and testing API functionality
"""
import setup_path
import cosysairsim as airsim

import time
import os
import tempfile
import pprint
import numpy as np
from cosysairsim.types import VesselControls, DisturbanceControls, Pose, Quaternionr, Vector3r, Vector2r

# connect to the AirSim simulator
client = airsim.VesselClient(ip="127.0.0.1") #to find this Host IP, see wsl_instructions.md  172.22.224.1 172.19.64.1
client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())

state = client.getVesselState()
s = pprint.pformat(state)
print("state: %s" % s)
forces = [1, 1]
angles = [0.5, 0.5]
client.setVesselControls('', VesselControls(forces, angles))
print("Vessel Controls set!")

time.sleep(5)

state = client.getVesselState()
s = pprint.pformat(state)
print("state: %s" % s)





















# time.sleep(1.5)
# lidar_data = client.getLidarData()
# pointcloud = np.array(lidar_data.point_cloud, dtype=np.dtype("f4"))
# pointcloud = np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))  # reshape to 3D array
# groundtruth = np.array(lidar_data.groundtruth, dtype=np.dtype('S'))


# x = pointcloud[:,0]
# y = pointcloud[:,1]

# plt.scatter(y,x)
# # plt.show()



# print(dir(lidar_data))



# vessel_state = client.getVesselState()
# print(lidar_data.groundtruth)

# xy_cloud = pointcloud[:,:2]
# distances = np.linalg.norm(xy_cloud, axis = 1)
# mask = ~np.isin(lidar_data.groundtruth, list({"Drone1","Ground","out_of_range"}))
# filtered_distances = distances[mask]
# # print(filtered_distances)
# mask = (groundtruth != b'Drone1') & (groundtruth != b'Ground') & (groundtruth != b'out_of_range')
# pointcloud[~mask] = 0

# lidar2D = pointcloud[:, 0:2]
# #minpooling from 450 to 45 points
# lidar_distances = np.linalg.norm(lidar2D, axis=1)
# lidar_distances = np.reshape(lidar_distances, (450, 1))
# lidar_distances = np.reshape(lidar_distances, (45, 10))
# lidar_distances = np.min(lidar_distances, axis=1)
# print(lidar_distances)
# p1 = (86, -14.5)
# p2 = (90, -14.5)
# line_start = (1, 4)
# line_end = (4, 1)

# def crossed_line(p1, p2, line_start, line_end):
#     # p1, p2: agent's last and current position
#     # line_start, line_end: checkpoint gate
#     def ccw(A, B, C):
#         return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    
#     return (ccw(p1, line_start, line_end) != ccw(p2, line_start, line_end)) and \
#            (ccw(p1, p2, line_start) != ccw(p1, p2, line_end))

# # line_start = (goal[1]["x_val"],goal[1]["y_val"])
# # line_end = (goal[2]["x_val"],goal[2]["y_val"])

# answer = crossed_line(p1,p2,line_start,line_end)
# print(answer)
# print("end")


