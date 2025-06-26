# Vessel Reinforcement Learning

## Introduction

In the Python Client of IDLab-ShippingSim, we have implemented a simple reinforcement learning (RL) example that enables researchers and developers to train autonomous vessel navigation agents. The RL environment provides a simulation for training agents to navigate vessels to target destinations while avoiding obstacles and collisions.

The reinforcement learning system is located in `PythonClient/Vessel/`.

## Environment Overview (`Shipsim_gym.py`)

### Environment Specifications

The ShippingSim environment implements a continuous control task where an agent learns to navigate a vessel to a target location while avoiding obstacles. The agent was trained in a slightly modified version of the LakeEnv environment. Do note that this is just an example and the environment nor the reward and and action and observation spaces are optimized for RL training.

| Specification | Details |
|---------------|---------|
| **Action Space** | Box(2,) - [thrust, rudder] |
| **Action Range** | thrust: [0, 1], rudder: [0.4, 0.6] |
| **Observation Space** | Box(57,) - vessel state + LiDAR data |
| **Episode Length** | Maximum 200 timesteps |
| **Success Condition** | Reach within 10 meters of goal |

### Observation Space Details

The observation vector contains 57 elements:

```python
obs = [
    distance_to_goal_x,           # Current X distance to goal
    distance_to_goal_y,           # Current Y distance to goal  
    prev_distance_to_goal_x,      # Previous X distance to goal
    prev_distance_to_goal_y,      # Previous Y distance to goal
    heading,                      # Vessel heading (radians)
    linear_velocity_x,            # X-axis linear velocity
    linear_velocity_y,            # Y-axis linear velocity
    linear_acceleration_x,        # X-axis linear acceleration
    linear_acceleration_y,        # Y-axis linear acceleration
    angular_acceleration_z,       # Z-axis angular acceleration
    prev_thrust,                  # Previous thrust action
    prev_rudder,                  # Previous rudder action
    lidar_distances[0:45]         # 45 LiDAR distance measurements
]
```

### Action Space Details

| Action | Range | Description |
|--------|-------|-------------|
| `thrust` | [0, 1] | Forward propulsion control |
| `rudder` | [0.4, 0.6] | Steering control (0.5 = straight) |

## SAC Training (`sac_example.py`)

### Overview
The SAC (Soft Actor-Critic) implementation provides state-of-the-art continuous control learning for vessel navigation. SAC is particularly suitable for this task due to its sample efficiency and stability in continuous action spaces.

### Basic Usage

```python
import gymnasium as gym
from stable_baselines3 import SAC
from Vessel.envs.Shipsim_gym import ShippingSim

# Create environment
env = gym.make("ship-sim-v0")

# Initialize SAC agent
model = SAC(
    "MlpPolicy", 
    env, 
    verbose=1,
    tensorboard_log="./sac_ship_sim_tb/",
    batch_size=32,
    buffer_size=4000,
    learning_starts=500,
    train_freq=1,
    tau=0.010,
    target_entropy=-2,
    stats_window_size=10
)

# Train the agent
model.learn(total_timesteps=25000, log_interval=1)

# Save the trained model
model.save("sac_ship_sim_v0")
```

## Environment Configuration

### AirSim Settings

Ensure your `settings.json` includes proper vessel configuration:

```json
  "SimMode": "Vessel",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "MilliAmpere",
      "HydroDynamics": {
        "hydrodynamics_engine": "FossenCurrent"
      },
      "PawnPath": "DefaultVessel",
      "AutoCreate": true,
      "RC": {
        "RemoteControlID": 0
      },
      "Sensors": {
          "lidar1": {
            "SensorType": 6,
            "Enabled": true,
            "NumberOfChannels": 1,
            "RotationsPerSecond": 1,
            "MeasurementsPerCycle": 450,
            "range": 100000,
            "X": 0,
            "Y": 0,
            "Z": -3.2,
            "Roll": 0,
            "Pitch": 0,
            "Yaw": 0,
            "VerticalFOVUpper": -2,
            "VerticalFOVLower": -3,
            "GenerateNoise": false,
            "DrawDebugPoints": false,
            "HorizontalFOVStart": -180,
            "HorizontalFOVEnd": 180
          }
      }
  }
}
```

### Custom Goal Positions

Modify goal positions in the environment:

```python
class ShippingSim(gym.Env):
    def __init__(self, options=None):
        # ... existing initialization ...
        self.goal_x = -60  # Modify target X coordinate
        self.goal_y = -10  # Modify target Y coordinate
```

For additional examples and advanced usage, see the [Vessel API documentation](vessel_api.md) and [AirSim API reference](../apis.md).
