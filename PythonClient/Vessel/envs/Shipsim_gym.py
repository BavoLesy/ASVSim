import gymnasium as gym
import numpy as np
import math 
from gymnasium import spaces
import time
import airsim
from airsim.types import VesselControls

class ShippingSim(gym.Env):
    def __init__(self, options=None):
        self.timestep = 0
        self.options = options
        self.prev_actions = np.zeros((2,))
        self.prev_distance_to_goal_x = 0
        self.prev_distance_to_goal_y = 0
        self.action_space = spaces.Box(low=np.array([0, 0.4]), high=np.array([1, 0.6]), shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=float("-inf"), high=float("inf"), shape=((2 + 3 + 2 + 3 + 2 + 45,)), dtype=np.float32)
        self.client = airsim.VesselClient(ip='127.0.0.1')
        self.client.confirmConnection()
        print("Connected to AirSim")
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.success = False
        self.collision = False
        self.goal_x = -60
        self.goal_y = -10


    def step(self, action):
        self.timestep += 1
        self.obs_time = time.time()
        self.client.setVesselControls('', VesselControls(float(action[0]), float(action[1])))
        obs = np.array(self.__observe(action), dtype=np.float32)
        reward, done = self.__reward(obs, action)
        time.sleep(max(0,1-(time.time()-self.obs_time)))
        if done:
            self.info = {"reward": reward, "thrust": action[0], "ruddder": action[1], "distance_to_goal_x": abs(self.distance_to_goal_x), "distance_to_goal_y": abs(self.distance_to_goal_y), "success": int(self.success), "collision": int(self.collision)}
        else:
            self.info = {"reward": reward, "thrust": action[0], "ruddder": action[1], "distance_to_goal_x": abs(self.distance_to_goal_x), "distance_to_goal_y": abs(self.distance_to_goal_y)}
        return obs, reward, done, False, self.info


    def reset(self, seed=None, options=None):
        self.timestep = 0
        self.client.setVesselControls('', VesselControls(0, 0.5))
        self.client.reset()
        print("Resetting")
        self.client.setVesselControls('', VesselControls(0, 0.5))
        self.prev_actions = np.zeros((2,))
        action = np.zeros((2,))
        self.obs_time = time.time()
        obs = np.array(self.__observe(action), dtype=np.float32)
        time.sleep(max(0,1-(time.time()-self.obs_time)))
        self.success = False
        self.collision = False
        return obs, {}


    def render(self):
        pass


    def __observe(self, action):
        vessel_state = self.client.getVesselState()
        lidar_data = self.client.getLidarData()
        pointcloud = np.array(lidar_data.point_cloud, dtype=np.dtype("f4"))
        pointcloud = np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))  # reshape to 3D array
        groundtruth = np.array(lidar_data.groundtruth, dtype=np.dtype('S'))
        #set all points with groundtruth b'Drone1' == Own vessel to 0,0,0 and points with b'Ground' to 0,0,0
        pointcloud[groundtruth == b'Drone1'] = 0
        pointcloud[groundtruth == b'Ground'] = 0
        lidar2D = pointcloud[:, 0:2]
        #minpooling from 450 to 45 points
        lidar_distances = np.linalg.norm(lidar2D, axis=1)
        lidar_distances = np.reshape(lidar_distances, (450, 1))
        lidar_distances = np.reshape(lidar_distances, (45, 10))
        lidar_distances = np.min(lidar_distances, axis=1)
        obs = np.zeros((3 + 2 + 3 + 2 + 45,))
        # Calculate the distance to the goal
        distance_to_goal_x = self.goal_x - vessel_state.kinematics_estimated.position.x_val
        distance_to_goal_y = self.goal_y - vessel_state.kinematics_estimated.position.y_val
        # Calculate the heading
        z = vessel_state.kinematics_estimated.orientation.z_val
        x = vessel_state.kinematics_estimated.orientation.x_val
        y = vessel_state.kinematics_estimated.orientation.y_val
        w = vessel_state.kinematics_estimated.orientation.w_val
        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        heading = -np.arctan2(siny, cosy) + math.pi
        # Calculate the linear velocity
        linear_velocity_x = vessel_state.kinematics_estimated.linear_velocity.x_val
        linear_velocity_y = vessel_state.kinematics_estimated.linear_velocity.y_val
        # Calculate the linear acceleration
        linear_acceleration_x = vessel_state.kinematics_estimated.linear_acceleration.x_val
        linear_acceleration_y = vessel_state.kinematics_estimated.linear_acceleration.y_val
        # Calculate the angular acceleration
        angular_acceleration_z = vessel_state.kinematics_estimated.angular_acceleration.z_val
        # Calculate the previous actions
        prev_actions = self.prev_actions
        # Update the previous actions
        self.prev_actions = action
        prev_distance_to_goal_x = self.prev_distance_to_goal_x
        prev_distance_to_goal_y = self.prev_distance_to_goal_y
        self.prev_distance_to_goal_x = distance_to_goal_x
        self.prev_distance_to_goal_y = distance_to_goal_y
        obs = np.concatenate([
        np.array([distance_to_goal_x, distance_to_goal_y, prev_distance_to_goal_x, prev_distance_to_goal_y, heading, linear_velocity_x, 
              linear_velocity_y, linear_acceleration_x, linear_acceleration_y, 
              angular_acceleration_z]),  
        prev_actions.flatten(), 
        lidar_distances.flatten()      
        ])
        return obs


    def __reward(self, obs, action):
        alpha = 0.01
        beta  = 0.01
        gamma = 0.00
        self.distance_to_goal_x = obs[0]
        self.distance_to_goal_y = obs[1]
        heading = obs[4]
        heading_to_goal = np.arctan2(self.distance_to_goal_y, self.distance_to_goal_x)

        distance = np.sqrt(self.distance_to_goal_x**2 + self.distance_to_goal_y**2) + (abs(heading - heading_to_goal) / 10)
        prev_distance = np.sqrt(self.prev_distance_to_goal_x**2 + self.prev_distance_to_goal_y**2)
        delta_thrust = np.square(action[0] - self.prev_actions[0])
        delta_rudder = np.square(action[1] - self.prev_actions[1])
        action_reward = -delta_thrust - delta_rudder
        self.reward = -alpha*distance - beta*(distance - prev_distance) - gamma*action_reward

        if distance < 10:
            end_ep = True
            self.success = True
            print("Success")
            self.reward += 500
        elif self.client.simGetCollisionInfo().has_collided:
            end_ep = True
            self.success = False
            self.collision = True
            self.reward -= 500
        elif self.timestep >= 200:
            end_ep = True
            self.success = False
            self.reward -= 500
        else:    
            end_ep = False

        return self.reward, end_ep




        