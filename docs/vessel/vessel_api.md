# How to Use Vessel in IDLab-ShippingSim

## Introduction

Cosys-AirSim provides comprehensive support for maritime vessel simulation with realistic hydrodynamics, environmental disturbances, and sensor modeling. The vessel API enables researchers and developers to create sophisticated maritime autonomous systems, test navigation algorithms, and develop computer vision applications for marine environments.

## Getting Started

By default, Cosys-AirSim prompts the user for which vehicle to use. You can easily change this by setting [SimMode](../settings.md#SimMode). For vessel simulation, set the SimMode in your [settings.json](../settings.md) file, which you can find in your `~/Documents/AirSim` folder:

```json
{
  "SettingsVersion": 2.0,
  "SimMode": "Vessel",
  "Vehicles": {
    "Vessel1": {
      "VehicleType": "MilliAmpere",
      "HydroDynamics": {
        "hydrodynamics_engine": "FossenCurrent"
      },
      "PawnPath": "DefaultVessel",
      "AutoCreate": true
    }
  }
}
```
Now when you restart IDLab-ShippingSim you should see the vessel spawned automatically.
There are a number of options in the settings file. Firstly, you can set which vehicle to use. In this case, we are using the `MilliAmpere` vessel, but we also support the `Qiuxin` and `Cybership2` vessels.We support different hydrodynamics models. In this case, we are using the `FossenCurrent` model, but we also support the `Fossen` model and the `AbkowitzLarge` model. Lastly, we have included 2 different blueprints for the vessel pawns: `CargoVessel` and `DefaultVessel`, which is a small fishers vessel.

# Controlling a Vessel

## Vessel API

### setVesselControls()
Controls vessel propulsion and steering. Use this to move and steer the vessel.

```python
from cosysairsim.types import VesselControls

controls = VesselControls(thrust=0.7, angle=0.6)  # 70% thrust, slight turn
client.setVesselControls('VesselName', controls)
```

**Parameters:**
- `rudder_thrust` (0.0-1.0): Thrust level (0.0 = stop, 1.0 = full power)
- `rudder_angle` (0.0-1.0): Steering angle (0.5 = straight, <0.5 = left, >0.5 = right)

### setDisturbanceControls()
Simulates environmental disturbances (wind, wave, current). Use this to test vessel behavior under different environmental conditions.

```python
from cosysairsim.types import DisturbanceControls
import math

disturbances = DisturbanceControls(
    wind_force=15.0, wind_angle=math.pi/4,    # Wind force (N) and direction (rad)
    current_force=5.0, current_angle=0.0      # Current force (N) and direction (rad)
)
client.setDisturbanceControls('VesselName', disturbances)
```

**Parameters:**
- Forces are in Newtons (N), angles in radians
- `wind_force/angle`: Wind pushing the vessel
- `wave_force/angle`: Wave forces affecting the vessel
- `current_force/angle`: Water current forces

### getVesselState()
Retrieves vessel position, orientation, and motion data. Use this to monitor the vessel's current state.

```python
state = client.getVesselState('VesselName')
pos = state.kinematics_estimated.position
vel = state.kinematics_estimated.linear_velocity

print(f"Position: ({pos.x_val}, {pos.y_val}, {pos.z_val})")
print(f"Velocity: ({vel.x_val}, {vel.y_val}, {vel.z_val})")
```

**Returns:**
- `kinematics_estimated`: Contains position, orientation, velocity, and acceleration data
- All positions are in world coordinates (meters)
- Velocities are in m/s, accelerations in m/s²

## Using APIs
You can control the vessel, get state and images by calling APIs in variety of client languages including Matlab and Python. Please see [APIs doc](../apis.md) for more details.

## Changing Views
By default camera will chase the car from the back. You can get the FPV view by pressing `F` key and switch back to chasing from back view by pressing `/` key. More keyboard shortcuts can be seen by pressing F1.

For additional examples and advanced usage patterns, see the [Vessel Data Generation documentation](data_generation.md) and [Reinforcement Learning documentation](reinforcement_learning.md) and explore the example scripts in `PythonClient/Vessel/`.
