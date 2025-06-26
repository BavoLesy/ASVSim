import setup_path
import cosysairsim as airsim
import numpy as np
import math
import time

import gymnasium as gym
from gymnasium import spaces
from airgym.envs.airsim_env import AirSimEnv


class AirSimVesselEnv(AirSimEnv):
    def __init__(self, ip_address, image_shape=(84, 84, 1)):
        super().__init__(image_shape)
        self.image_shape = image_shape
        self.timestep = 0
        
        # Goal position
        self.goal_x = -60
        self.goal_y = -10
        
        # Previous state tracking
        self.prev_actions = np.zeros((2,))
        self.prev_distance_to_goal_x = 0
        self.prev_distance_to_goal_y = 0
        
        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
            "vessel_state": None,
            "success": False,
            "distance_to_goal_x": 0,
            "distance_to_goal_y": 0
        }

        # Initialize vessel client
        self.vessel = airsim.VesselClient(ip=ip_address)
        
        # Action space: [thrust, rudder]
        self.action_space = spaces.Box(
            low=np.array([0, 0.4]), 
            high=np.array([1, 0.6]), 
            shape=(2,), 
            dtype=np.float32
        )
        
        # Override observation space to match vessel state vector (57 elements)
        # 2 (goal distances) + 3 (prev distances + heading) + 2 (velocity) + 3 (acceleration) + 2 (prev actions) + 45 (lidar)
        self.observation_space = spaces.Box(
            low=float("-inf"), 
            high=float("inf"), 
            shape=(57,), 
            dtype=np.float32
        )
        
        self._setup_vessel()

        # For compatibility with AirSimEnv, though we'll override _get_obs
        self.image_request = airsim.ImageRequest(
            "4", airsim.ImageType.Scene, False, False
        )

    def __del__(self):
        self.vessel.reset()

    def _setup_vessel(self):
        """Initialize vessel connection and controls."""
        self.vessel.confirmConnection()
        print("Connected to AirSim Vessel")
        self.vessel.enableApiControl(True)
        self.vessel.armDisarm(True)
        
        # Set initial controls
        self.vessel.setVesselControls('', airsim.VesselControls(0, 0.5))

    def _get_obs(self):
        """Get vessel state and LiDAR observations."""
        # Get vessel state
        vessel_state = self.vessel.getVesselState()
        self.state["vessel_state"] = vessel_state
        
        # Update position
        self.state["prev_position"] = self.state["position"]
        self.state["position"] = np.array([
            vessel_state.kinematics_estimated.position.x_val,
            vessel_state.kinematics_estimated.position.y_val,
            vessel_state.kinematics_estimated.position.z_val
        ])
        
        # Get LiDAR data
        lidar_data = self.vessel.getLidarData()
        pointcloud = np.array(lidar_data.point_cloud, dtype=np.dtype("f4"))
        pointcloud = np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))
        groundtruth = np.array(lidar_data.groundtruth, dtype=np.dtype('S'))
        
        # Filter out own vessel and ground points
        pointcloud[groundtruth == b'Drone1'] = 0
        pointcloud[groundtruth == b'Ground'] = 0
        lidar2D = pointcloud[:, 0:2]
        
        # Convert to distance measurements and downsample
        lidar_distances = np.linalg.norm(lidar2D, axis=1)
        lidar_distances = np.reshape(lidar_distances, (450, 1))
        lidar_distances = np.reshape(lidar_distances, (45, 10))
        lidar_distances = np.min(lidar_distances, axis=1)
        
        # Calculate distances to goal
        distance_to_goal_x = self.goal_x - vessel_state.kinematics_estimated.position.x_val
        distance_to_goal_y = self.goal_y - vessel_state.kinematics_estimated.position.y_val
        
        self.state["distance_to_goal_x"] = distance_to_goal_x
        self.state["distance_to_goal_y"] = distance_to_goal_y
        
        # Calculate heading
        z = vessel_state.kinematics_estimated.orientation.z_val
        x = vessel_state.kinematics_estimated.orientation.x_val
        y = vessel_state.kinematics_estimated.orientation.y_val
        w = vessel_state.kinematics_estimated.orientation.w_val
        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        heading = -np.arctan2(siny, cosy) + math.pi
        
        # Get velocities and accelerations
        linear_velocity_x = vessel_state.kinematics_estimated.linear_velocity.x_val
        linear_velocity_y = vessel_state.kinematics_estimated.linear_velocity.y_val
        linear_acceleration_x = vessel_state.kinematics_estimated.linear_acceleration.x_val
        linear_acceleration_y = vessel_state.kinematics_estimated.linear_acceleration.y_val
        angular_acceleration_z = vessel_state.kinematics_estimated.angular_acceleration.z_val
        
        # Construct observation vector
        obs = np.concatenate([
            np.array([distance_to_goal_x, distance_to_goal_y, 
                     self.prev_distance_to_goal_x, self.prev_distance_to_goal_y, 
                     heading, linear_velocity_x, linear_velocity_y, 
                     linear_acceleration_x, linear_acceleration_y, 
                     angular_acceleration_z]),
            self.prev_actions.flatten(),
            lidar_distances.flatten()
        ])
        
        # Update previous distances
        self.prev_distance_to_goal_x = distance_to_goal_x
        self.prev_distance_to_goal_y = distance_to_goal_y
        
        # Check collision
        collision = self.vessel.simGetCollisionInfo().has_collided
        self.state["collision"] = collision
        
        return obs.astype(np.float32)

    def _do_action(self, action):
        """Execute vessel action."""
        # Set vessel controls
        vessel_controls = airsim.VesselControls(float(action[0]), float(action[1]))
        self.vessel.setVesselControls('', vessel_controls)
        
        # Update previous actions
        self.prev_actions = action.copy()
        
        # Sleep to maintain real-time factor
        time.sleep(0.1)

    def _compute_reward(self):
        """Compute reward based on current state."""
        alpha = 0.01
        beta = 0.01
        gamma = 0.00
        
        distance_to_goal_x = self.state["distance_to_goal_x"]
        distance_to_goal_y = self.state["distance_to_goal_y"]
        
        # Get heading for reward calculation
        vessel_state = self.state["vessel_state"]
        z = vessel_state.kinematics_estimated.orientation.z_val
        x = vessel_state.kinematics_estimated.orientation.x_val
        y = vessel_state.kinematics_estimated.orientation.y_val
        w = vessel_state.kinematics_estimated.orientation.w_val
        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        heading = -np.arctan2(siny, cosy) + math.pi
        
        heading_to_goal = np.arctan2(distance_to_goal_y, distance_to_goal_x)
        
        distance = np.sqrt(distance_to_goal_x**2 + distance_to_goal_y**2) + (abs(heading - heading_to_goal) / 10)
        prev_distance = np.sqrt(self.prev_distance_to_goal_x**2 + self.prev_distance_to_goal_y**2)
        
        # Action penalty (currently disabled with gamma=0)
        delta_thrust = np.square(self.prev_actions[0] - 0)  # Compare to neutral
        delta_rudder = np.square(self.prev_actions[1] - 0.5)  # Compare to straight
        action_reward = -delta_thrust - delta_rudder
        
        reward = -alpha * distance - beta * (distance - prev_distance) - gamma * action_reward
        
        # Check terminal conditions
        done = 0
        if distance < 10:
            done = 1
            self.state["success"] = True
            print("Success")
            reward += 500
        elif self.state["collision"]:
            done = 1
            self.state["success"] = False
            reward -= 500
        elif self.timestep >= 200:
            done = 1
            self.state["success"] = False
            reward -= 500
        
        return reward, done

    def step(self, action):
        """Step the environment."""
        self.timestep += 1
        
        # Execute action
        self._do_action(action)
        
        # Get observation
        obs = self._get_obs()
        
        # Compute reward
        reward, done = self._compute_reward()
        
        # Prepare info
        info = {
            "reward": reward,
            "thrust": action[0],
            "rudder": action[1],
            "distance_to_goal_x": abs(self.state["distance_to_goal_x"]),
            "distance_to_goal_y": abs(self.state["distance_to_goal_y"]),
            "success": int(self.state["success"]),
            "collision": int(self.state["collision"])
        }
        
        return obs, reward, done, info

    def reset(self):
        """Reset the environment."""
        self.timestep = 0
        
        # Reset vessel
        self.vessel.setVesselControls('', airsim.VesselControls(0, 0.5))
        self.vessel.reset()
        print("Resetting Vessel")
        self.vessel.setVesselControls('', airsim.VesselControls(0, 0.5))
        
        # Reset state
        self.prev_actions = np.zeros((2,))
        self.prev_distance_to_goal_x = 0
        self.prev_distance_to_goal_y = 0
        self.state["success"] = False
        self.state["collision"] = False
        
        # Get initial observation
        obs = self._get_obs()
        
        return obs
