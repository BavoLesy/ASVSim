import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import cosysairsim as airsim
from cosysairsim.types import VesselControls, Pose, Quaternionr, Vector3r, Vector2r
import numpy as np
import math
import time
import subprocess
from collections import deque

import gymnasium as gym
from gymnasium import spaces


class PCGVesselEnv(gym.Env):
    """Gymnasium environment for vessel RL training with PCG terrain randomization."""

    def __init__(
        self,
        ip_address="127.0.0.1",
        terrain_regen_interval=10,
        num_obstacles=4,
        num_dynamic_obstacles=0,
        goal_distance=2,
        max_timesteps=800,
        step_sleep=0.25,
        action_repeat=1,
        seed=42,
        sim_path="Blocks/Blocks.exe",
        sim_wait=10,
    ):
        super().__init__()

        self.ip_address = ip_address
        self.sim_path = sim_path
        self.sim_wait = sim_wait
        self.sim_proc = None  # managed externally or by _restart_sim
        self.terrain_regen_interval = terrain_regen_interval
        self.num_obstacles = num_obstacles
        self.num_dynamic_obstacles = num_dynamic_obstacles
        self.goal_distance = goal_distance
        self.max_timesteps = max_timesteps
        self.step_sleep = step_sleep
        self.action_repeat = action_repeat
        self.base_seed = seed

        self.n_stack = 1
        self.timestep = 0
        self.episode_count = 0
        self.rng = np.random.RandomState(seed)

        # Goal and terrain state
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.border_points = []
        self.initial_location = Vector2r(0.0, 0.0)
        self.spawned_obstacle_names = []

        # Waypoint navigation state
        # waypoints: list of (x, y) tuples — [intermediate, final_goal]
        self.waypoints = []
        self.current_waypoint_idx = 0
        # Position of last reached waypoint (or vessel start position)
        self.prev_waypoint_x = 0.0
        self.prev_waypoint_y = 0.0

        # Collision timestamp baseline (ignore collisions from before reset)
        self._reset_collision_ts = 0

        # Previous state tracking
        self.prev_actions = np.zeros(2)
        self.prev_distance_to_goal_x = 0.0
        self.prev_distance_to_goal_y = 0.0
        self._waypoint_start_distance = 1.0
        self._next_distance_milestone = None
        self._reward_prev_dist_x = 0.0
        self._reward_prev_dist_y = 0.0

        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
            "vessel_state": None,
            "success": False,
            "distance_to_goal_x": 0.0,
            "distance_to_goal_y": 0.0,
        }


        # Action space: [thrust, rudder_angle]
        self.action_space = spaces.Box(
            low=np.array([0.0, 0.48]),
            high=np.array([0.7, 0.52]),
            shape=(2,),
            dtype=np.float32,
        )

        # Single-frame observation: 54-dim vector
        # 2 prev waypoint dist + 2 curr waypoint dist + 2 next waypoint dist
        # + 1 heading_error + 2 heading (sin, cos) + 2 velocity + 3 acceleration
        # + 2 prev actions (thrust, rudder_angle)
        # + 36 lidar (3600 points min-pooled into 36 sectors of 10°)
        self.single_obs_size = 54

        # Frame stacking: concatenate n_stack frames
        self.frame_buffer = deque(maxlen=self.n_stack)
        self.observation_space = spaces.Box(
            low=float("-inf"),
            high=float("inf"),
            shape=(self.single_obs_size * self.n_stack,),
            dtype=np.float32,
        )

        # Labels to filter out of LiDAR (set to 0 = no obstacle)
        self.FILTER_LABELS = {
            b"Ground", b"milliampere",
            b"Landscape_0", b"Landscape_1", b"PCGVolume_1",
        }

        # Connect to simulator
        self.vessel = airsim.VesselClient(ip=ip_address)
        self._setup_vessel()

        # Activate PCG system once (landscape=False: landscape creation is editor-only)
        pcg_ok = self.vessel.activateGeneration(False)
        print(f"PCG activateGeneration returned: {pcg_ok}")
        # Terrain is generated on the first reset() call (episode 1)

    def _wait_collision_clear(self, timeout=2.0):
        """Poll until collision info is cleared after reset (max timeout seconds)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.vessel.simGetCollisionInfo().has_collided:
                return
            time.sleep(0.05)

    def _setup_vessel(self):
        self.vessel.confirmConnection()
        print("Connected to AirSim Vessel")
        self.vessel.enableApiControl(True)
        self.vessel.armDisarm(True)
        self.vessel.setVesselControls("", VesselControls([0, 0], [0.5, 0.5]))

    def _restart_sim(self, max_retries=3):
        """Kill the simulator, restart it, reconnect, and re-activate PCG."""
        for attempt in range(max_retries):
            try:
                print(f"SIM RESTART: attempt {attempt + 1}/{max_retries}...")

                # Kill any existing sim process
                if self.sim_proc is not None:
                    self.sim_proc.kill()
                    self.sim_proc.wait()
                else:
                    os.system("taskkill /F /IM Blocks.exe >nul 2>&1")
                time.sleep(3.0)

                # Start fresh sim
                self.sim_proc = subprocess.Popen(
                    [self.sim_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                wait_time = self.sim_wait + attempt * 10  # wait longer on retries
                print(f"SIM RESTART: new PID={self.sim_proc.pid}, waiting {wait_time}s...")
                time.sleep(wait_time)

                # Reconnect client
                self.vessel = airsim.VesselClient(ip=self.ip_address)
                self._setup_vessel()

                # Re-activate PCG
                pcg_ok = self.vessel.activateGeneration(False)
                print(f"SIM RESTART: PCG activateGeneration returned: {pcg_ok}")

                # Force terrain regeneration on next reset
                self._needs_terrain_regen = True
                print("SIM RESTART: complete")
                return  # success

            except Exception as e:
                print(f"SIM RESTART: attempt {attempt + 1} failed: {e}")
                if attempt == max_retries - 1:
                    print("SIM RESTART: all retries exhausted, giving up")
                    raise

    def _generate_terrain(self):
        """Generate new PCG terrain and fetch waypoints for each section up to goal_distance."""
        if self.terrain_regen_interval == 0:
            seed = -538846106
        else:
            seed = -464588337 + self.base_seed + self.episode_count
        print(f"Generating terrain with seed={seed}")

        time.sleep(2.0)  # let sim settle before PCG generation
        terrain_ok = self.vessel.generatePortTerrain("port", seed, 10, -45.0, 45.0, 5000.0, 10000.0)
        print(f"generatePortTerrain returned: {terrain_ok}")

        # Poll until final waypoint (section goal_distance) is valid
        max_goal_distance = 2000.0
        final_result = []
        goal_valid = False
        for attempt in range(10):
            time.sleep(2.0)
            final_result = self.vessel.getGoal(self.initial_location, self.goal_distance)
            if len(final_result) >= 1:
                gx, gy = final_result[0]["x_val"], final_result[0]["y_val"]
                dist = math.sqrt(gx**2 + gy**2)
                if gx == 0.0 and gy == 0.0:
                    print(f"  getGoal({self.goal_distance}) returned (0,0), retrying... ({attempt + 1}/10)")
                elif dist > max_goal_distance:
                    print(f"  getGoal({self.goal_distance}) returned unreasonable coords ({gx:.1f}, {gy:.1f}), retrying... ({attempt + 1}/10)")
                else:
                    goal_valid = True
                    break

        if not goal_valid:
            raise RuntimeError("PCG terrain generation failed — getGoal returned invalid coordinates after all retries. Check the packaged build includes the PCG plugin.")

        if len(final_result) >= 3:
            self.border_points = final_result[1:]

        # Fetch waypoints for each section (1 through goal_distance)
        self.waypoints = []
        for section in range(1, self.goal_distance + 1):
            result = self.vessel.getGoal(self.initial_location, section)
            if len(result) >= 1 and (result[0]["x_val"] != 0.0 or result[0]["y_val"] != 0.0):
                self.waypoints.append((result[0]["x_val"], result[0]["y_val"]))
            else:
                print(f"WARNING: getGoal section {section} returned (0,0), skipping")

        if not self.waypoints:
            raise RuntimeError("No valid waypoints returned from PCG terrain generation")

        self.goal_x = self.waypoints[-1][0]
        self.goal_y = self.waypoints[-1][1]
        wp_str = "  ".join(f"WP{i+1}=({x:.1f}, {y:.1f})" for i, (x, y) in enumerate(self.waypoints))
        print(f"{wp_str}")

    def _spawn_obstacles(self):
        """Spawn random obstacles between vessel and goal."""
        if self.goal_x == 0.0 and self.goal_y == 0.0:
            print("WARNING: goal is (0,0), skipping obstacle spawn to avoid physics explosion")
            return

        # Get current vessel position
        vessel_state = self.vessel.getVesselState()
        start_x = vessel_state.kinematics_estimated.position.x_val
        start_y = vessel_state.kinematics_estimated.position.y_val

        # Convert NED meters to UE centimeters
        goal_x_cm = self.goal_x * 100
        goal_y_cm = self.goal_y * 100
        start_x_cm = start_x * 100
        start_y_cm = start_y * 100

        for _ in range(self.num_obstacles):
            # Random fraction along the path (avoid placing too close to start/goal)
            frac = self.rng.uniform(0.15, 0.85)
            # Add lateral offset for variety
            lateral_offset = self.rng.uniform(-2000, 2000)

            obs_x = start_x_cm + frac * (goal_x_cm - start_x_cm)
            obs_y = start_y_cm + frac * (goal_y_cm - start_y_cm) + lateral_offset

            pose = Pose(Vector3r(obs_x, obs_y, 250), Quaternionr())
            self.vessel.simAddObstacle(pose, 0, "buoy")

        # Track obstacle names for cleanup
        self._update_obstacle_list()

    def _update_obstacle_list(self):
        """Find spawned obstacle actors by name pattern."""
        all_objects = self.vessel.simListSceneObjects(".*BP_BuoySpawn.*")
        self.spawned_obstacle_names = all_objects

    def _destroy_obstacles(self):
        """Remove all buoy obstacles currently in the scene."""
        # Always query the scene fresh — don't rely on cached names
        all_buoys = self.vessel.simListSceneObjects(".*BP_BuoySpawn.*")
        for name in all_buoys:
            self.vessel.simDestroyObject(name)
        self.spawned_obstacle_names = []

    def _spawn_dynamic_obstacles(self):
        """Spawn moving obstacles using the simulator's built-in movement."""
        if self.num_dynamic_obstacles <= 0:
            return

        vessel_state = self.vessel.getVesselState()
        start_x = vessel_state.kinematics_estimated.position.x_val
        start_y = vessel_state.kinematics_estimated.position.y_val

        start_x_cm = start_x * 100
        start_y_cm = start_y * 100
        goal_x_cm = self.goal_x * 100
        goal_y_cm = self.goal_y * 100

        for _ in range(self.num_dynamic_obstacles):
            frac = self.rng.uniform(0.2, 0.8)
            lateral_offset = self.rng.uniform(-2000, 2000)

            obs_x = start_x_cm + frac * (goal_x_cm - start_x_cm)
            obs_y = start_y_cm + frac * (goal_y_cm - start_y_cm) + lateral_offset

            speed = float(self.rng.uniform(500, 1500))
            pose = Pose(Vector3r(obs_x, obs_y, 250), Quaternionr())
            self.vessel.simAddObstacle(pose, speed, "boat")

    def _get_obs(self):
        """Get vessel state and LiDAR observations."""
        vessel_state = self.vessel.getVesselState()
        self.state["vessel_state"] = vessel_state

        # Update position
        self.state["prev_position"] = self.state["position"]
        self.state["position"] = np.array([
            vessel_state.kinematics_estimated.position.x_val,
            vessel_state.kinematics_estimated.position.y_val,
            vessel_state.kinematics_estimated.position.z_val,
        ])
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]

        if self.timestep <= 1:
            print(f"  DEBUG pos=({pos_x:.1f}, {pos_y:.1f}), wp={self.waypoints[self.current_waypoint_idx]}")

        # LiDAR data
        lidar_data = self.vessel.getLidarData()
        pointcloud = np.array(lidar_data.point_cloud, dtype=np.float32)
        pointcloud = np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))
        groundtruth = np.array(lidar_data.groundtruth, dtype=np.dtype("S"))

        # Compute 2D distances
        lidar2D = pointcloud[:, 0:2]
        lidar_distances = np.linalg.norm(lidar2D, axis=1)

        # Filter out unwanted labels (set to 0 = no obstacle)
        filter_mask = np.array([l in self.FILTER_LABELS for l in groundtruth])
        lidar_distances[filter_mask] = 0.0

        # Min obstacle distance for reward (exclude filtered points)
        non_zero = lidar_distances[lidar_distances > 0]
        self.min_obstacle_distance = float(np.min(non_zero)) if len(non_zero) > 0 else 999.0

        # Min-pool 3600 points into 36 sectors (each 10°, 100 points)
        lidar_distances = np.reshape(lidar_distances, (36, 100))
        lidar_for_pool = np.where(lidar_distances > 0, lidar_distances, np.inf)
        lidar_pooled = np.min(lidar_for_pool, axis=1)
        lidar_pooled[lidar_pooled == np.inf] = 0.0  # no obstacle in sector

        # Heading from quaternion
        z = vessel_state.kinematics_estimated.orientation.z_val
        x = vessel_state.kinematics_estimated.orientation.x_val
        y = vessel_state.kinematics_estimated.orientation.y_val
        w = vessel_state.kinematics_estimated.orientation.w_val
        siny = 2 * (w * z + x * y)
        cosy = 1 - 2 * (y * y + z * z)
        heading = (-np.arctan2(siny, cosy) + np.pi) % (2 * np.pi) - np.pi
        self.state["heading"] = heading

        # Current waypoint distances (active navigation target)
        curr_wp = self.waypoints[self.current_waypoint_idx]
        dx_curr = curr_wp[0] - pos_x
        dy_curr = curr_wp[1] - pos_y
        self.state["distance_to_goal_x"] = dx_curr
        self.state["distance_to_goal_y"] = dy_curr

        # Previous waypoint distances (where we came from)
        dx_prev = self.prev_waypoint_x - pos_x
        dy_prev = self.prev_waypoint_y - pos_y

        # Next waypoint distances (lookahead; zeros if no next waypoint)
        if self.current_waypoint_idx + 1 < len(self.waypoints):
            next_wp = self.waypoints[self.current_waypoint_idx + 1]
            dx_next = next_wp[0] - pos_x
            dy_next = next_wp[1] - pos_y
        else:
            dx_next, dy_next = 0.0, 0.0

        # Scalar distances to current and next waypoints
        distance_to_curr = np.sqrt(dx_curr**2 + dy_curr**2)
        distance_to_next = np.sqrt(dx_next**2 + dy_next**2) if dx_next != 0.0 or dy_next != 0.0 else 0.0

        # Heading error to current waypoint, normalized to [-pi, pi]
        goal_angle = np.arctan2(dy_curr, dx_curr)
        heading_error = (heading - goal_angle + np.pi) % (2 * np.pi) - np.pi

        # Velocities and accelerations
        linear_velocity_x = vessel_state.kinematics_estimated.linear_velocity.x_val
        linear_velocity_y = vessel_state.kinematics_estimated.linear_velocity.y_val
        linear_acceleration_x = vessel_state.kinematics_estimated.linear_acceleration.x_val
        linear_acceleration_y = vessel_state.kinematics_estimated.linear_acceleration.y_val
        angular_acceleration_z = vessel_state.kinematics_estimated.angular_acceleration.z_val

        obs = np.concatenate([
            np.array([
                dx_prev, dy_prev,
                dx_curr, dy_curr,
                distance_to_curr,
                dx_next, dy_next,
                distance_to_next,
                heading_error,
                np.sin(heading), np.cos(heading),
                linear_velocity_x, linear_velocity_y,
                linear_acceleration_x, linear_acceleration_y,
                angular_acceleration_z,
            ]),
            self.prev_actions.flatten(),
            lidar_pooled.flatten(),
        ])

        self.prev_distance_to_goal_x = dx_curr
        self.prev_distance_to_goal_y = dy_curr

        # Collision check — only count collisions that happened after reset
        col_info = self.vessel.simGetCollisionInfo()
        self.state["collision"] = (
            col_info.has_collided and col_info.time_stamp > self._reset_collision_ts
        )

        return obs.astype(np.float32)

    def _get_stacked_obs(self):
        """Get current observation and return frame-stacked version."""
        obs = self._get_obs()
        self.frame_buffer.append(obs)
        return np.concatenate(list(self.frame_buffer))

    def _do_action(self, action):
        thrust = float(action[0])
        angle = float(action[1])
        vessel_controls = VesselControls([0, thrust], [0, angle])
        for _ in range(self.action_repeat):
            self.vessel.setVesselControls("", vessel_controls)
            time.sleep(self.step_sleep)
        self.prev_actions = action.copy()

    def _compute_reward(self):
        dx = self.state["distance_to_goal_x"]
        dy = self.state["distance_to_goal_y"]
        distance = math.sqrt(dx**2 + dy**2)
        prev_distance = math.sqrt(
            self._reward_prev_dist_x**2 + self._reward_prev_dist_y**2
        )
        progress = prev_distance - distance  # positive = getting closer

        # Progress reward + time penalty (net negative unless making good progress)
        reward = 1.0 * progress - 0.1

        # Terminal conditions
        terminated = False
        truncated = False
        if self.state["collision"]:
            terminated = True
            self.state["success"] = False
            reward = -100.0
        elif distance < 10:
            is_final = self.current_waypoint_idx == len(self.waypoints) - 1
            if is_final:
                terminated = True
                self.state["success"] = True
                reward = 500.0
            else:
                # Intermediate waypoint reached — advance to next
                reached = self.waypoints[self.current_waypoint_idx]
                self.prev_waypoint_x = reached[0]
                self.prev_waypoint_y = reached[1]
                self.current_waypoint_idx += 1
                next_wp = self.waypoints[self.current_waypoint_idx]
                pos_x = self.state["position"][0]
                pos_y = self.state["position"][1]
                # Reset distance tracking so next step's progress is toward new waypoint
                self.prev_distance_to_goal_x = next_wp[0] - pos_x
                self.prev_distance_to_goal_y = next_wp[1] - pos_y
                self.state["distance_to_goal_x"] = next_wp[0] - pos_x
                self.state["distance_to_goal_y"] = next_wp[1] - pos_y
        elif self.timestep >= self.max_timesteps:
            truncated = True
            self.state["success"] = False

        return reward, terminated, truncated

    def step(self, action):
        try:
            return self._step_inner(action)
        except Exception as e:
            print(f"SIM ERROR in step(): {e}")
            self._restart_sim()
            # Return a truncated episode so training continues
            obs = np.zeros(self.single_obs_size * self.n_stack, dtype=np.float32)
            info = {
                "reward": 0.0, "thrust": 0.0, "rudder_angle": 0.5,
                "distance_to_goal_x": 0.0, "distance_to_goal_y": 0.0,
                "success": 0, "collision": 0,
                "episode_num": self.episode_count, "end_reason": "sim_crash",
                "waypoints_reached": 0,
            }
            return obs, 0.0, False, True, info

    def _step_inner(self, action):
        self.timestep += 1
        self._do_action(action)
        # Save previous distance BEFORE _get_obs overwrites it
        self._reward_prev_dist_x = self.prev_distance_to_goal_x
        self._reward_prev_dist_y = self.prev_distance_to_goal_y
        obs = self._get_stacked_obs()
        reward, terminated, truncated = self._compute_reward()

        end_reason = None
        if terminated:
            end_reason = "collision" if self.state["collision"] else "goal_reached"
        elif truncated:
            end_reason = "timeout"

        # Distance to final goal (always waypoints[-1]) for logging
        final_wp = self.waypoints[-1]
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]
        final_dist_x = final_wp[0] - pos_x
        final_dist_y = final_wp[1] - pos_y

        # How many waypoints were fully reached this episode
        waypoints_reached = (
            len(self.waypoints) if end_reason == "goal_reached"
            else self.current_waypoint_idx
        )

        if end_reason is not None:
            try:
                self.vessel.setVesselControls("", VesselControls([0, 0], [0.5, 0.5]))
                self.vessel.reset()
            except Exception:
                pass  # will be handled by next reset()
            final_dist = np.sqrt(final_dist_x**2 + final_dist_y**2)
            print(f"[Episode {self.episode_count}] End: {end_reason} | wps={waypoints_reached}/{len(self.waypoints)} | reward={reward:.1f} | steps={self.timestep} | final_dist={final_dist:.1f} | initial_dist={self.initial_goal_distance:.1f}")

        info = {
            "reward": reward,
            "thrust": action[0],
            "rudder_angle": action[1],
            "distance_to_goal_x": abs(final_dist_x),
            "distance_to_goal_y": abs(final_dist_y),
            "success": int(self.state["success"]),
            "collision": int(self.state["collision"]),
            "episode_num": self.episode_count,
            "end_reason": end_reason,
            "waypoints_reached": waypoints_reached,
        }

        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        try:
            return self._reset_inner(seed=seed, options=options)
        except Exception as e:
            print(f"SIM ERROR in reset(): {e}")
            self._restart_sim()
            # Retry reset after restart
            return self._reset_inner(seed=seed, options=options)

    def _reset_inner(self, seed=None, options=None):
        super().reset(seed=seed)
        self.timestep = 0
        self.episode_count += 1

        # Regenerate terrain: every N episodes, only on first episode if interval <= 0,
        # or forced after sim restart
        regen = getattr(self, '_needs_terrain_regen', False) or (
            self.episode_count == 1
            if self.terrain_regen_interval <= 0
            else self.episode_count % self.terrain_regen_interval == 1
        )
        self._needs_terrain_regen = False

        self.vessel.reset()
        self._wait_collision_clear()
        self.vessel.enableApiControl(True)
        self.vessel.armDisarm(True)
        if regen:
            self._generate_terrain()
            print(f"[Episode {self.episode_count}] Regenerated terrain")

        # Neutral controls
        self.vessel.setVesselControls("", VesselControls([0, 0], [0.5, 0.5]))

        # Clean up any leftover obstacles, then spawn fresh ones
        self._destroy_obstacles()
        self._spawn_obstacles()
        self._spawn_dynamic_obstacles()

        # Record collision timestamp so we ignore any stale/spawn collisions
        self._reset_collision_ts = self.vessel.simGetCollisionInfo().time_stamp

        # Reset waypoint navigation state
        self.current_waypoint_idx = 0
        self.prev_waypoint_x = 0.0
        self.prev_waypoint_y = 0.0

        # Reset tracking state
        self.prev_actions = np.zeros(2)
        self.prev_distance_to_goal_x = 0.0
        self.prev_distance_to_goal_y = 0.0
        self.state["success"] = False
        self.state["collision"] = False

        # Fill frame buffer with initial observation
        self.frame_buffer.clear()
        obs = self._get_obs()
        self.initial_goal_distance = np.sqrt(
            self.state["distance_to_goal_x"]**2 +
            self.state["distance_to_goal_y"]**2
        )
        self._waypoint_start_distance = self.initial_goal_distance
        # Set distance milestone based on final waypoint (round down to nearest 10m)
        final_wp = self.waypoints[-1]
        pos_x = self.state["position"][0]
        pos_y = self.state["position"][1]
        dist_to_final = np.sqrt((final_wp[0] - pos_x)**2 + (final_wp[1] - pos_y)**2)
        self._next_distance_milestone = (int(dist_to_final) // 20) * 20
        for _ in range(self.n_stack):
            self.frame_buffer.append(obs)
        return np.concatenate(list(self.frame_buffer)), {}

    def close(self):
        self.vessel.reset()
