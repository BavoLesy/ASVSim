# Vessel Data Generation Scripts

## Introduction

In the Python Client of Lab-ShippingSim, we have implemented a data generation suite that enables researchers and developers to generate datasets for computer vision models, object detection systems, and path planning algorithms in maritime environments.

The data generation system is located in `PythonClient/Vessel/data_generation/` and consists of three main components:

- **Dataset Generation**: Automated collection of RGB, segmentation, and depth images
- **Bounding Box Creation**: Object detection annotation toolss

## Prerequisites

Before using the data generation scripts, ensure you have the following dependencies installed:

### Python Dependencies
```bash
pip install cosysairsim
pip install opencv-python
pip install matplotlib
pip install pillow
pip install pandas
pip install numpy
pip install pdf2image  # For PDF visualization support
```


## Dataset Generation (`dataset_generation.py`)

### Overview
The dataset generation script automatically collects synchronized RGB, segmentation, and depth images from the vessel simulation. This is ideal for training computer vision models for maritime object detection and scene understanding.

### Basic Usage

```python
import cosysairsim as airsim
from dataset_generation import initialize_data_collection, generate_dataset

# Connect to AirSim
client = airsim.VesselClient()
client.confirmConnection()
client.enableApiControl(True)

# Configuration
dataset_path = "Vessel/data_generation/dataset"
num_images = 100
max_depth = 255  # Maximum depth range in meters

# Initialize data collection
segmentation_folder, rgb_folder, depth_folder = initialize_data_collection(client, dataset_path)

# Generate dataset
generate_dataset(client, segmentation_folder, rgb_folder, depth_folder, max_depth, num_images)
```

### Configuration Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `dataset_path` | string | Root directory for dataset storage | `"Vessel/data_generation/dataset"` |
| `num_images` | int | Number of images to collect | `10` |
| `max_depth` | int | Maximum depth camera range (meters) | `255` |

### Output Structure
The script creates a timestamped folder with the following structure:
```
dataset/
└── YYYY_MM_DD_HH_MM_SS/
    ├── rgb/                          # RGB images
    │   ├── 0.png
    │   ├── 1.png
    │   └── ...
    ├── segmentation/                 # Segmentation masks
    │   ├── 0.png
    │   ├── 1.png
    │   └── ...
    ├── depth/                        # Depth images
    │   ├── 0.png
    │   ├── 1.png
    │   └── ...
    └── segmentation_colormap_list.csv # Object-to-color mapping
```

### Camera Configuration
The script uses camera "4" by default. Ensure your `settings.json` includes a depth camera, segmentation camera and an RGB camera (0, 3, 5):

```json
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,   
        "Width": 1400,
        "Height": 900,
        "FOV_Degrees": 90,
        "MotionBlurAmount": 0
      },
      {
        "ImageType": 3,
        "Width": 1400,
        "Height": 900,
        "FOV_Degrees": 90,
      },
      {
        "ImageType": 5,
        "Width": 1400,
        "Height": 900,
        "FOV_Degrees": 90,
        "MotionBlurAmount": 0
      }
    ]
  }
```

## Bounding Box Generation (`bounding_boxes.py`)

### Overview
The bounding box script processes segmentation images to create object detection annotations. It generates bounding boxes around specified objects and can calculate distances using depth information.

### Basic Usage

```python
from bounding_boxes import (
    get_all_instances_of_object, 
    get_object_rgb, 
    create_bounding_boxes,
    save_bounding_boxes,
    visualize_bounding_boxes
)

# Specify dataset path
dataset_path = "Vessel/data_generation/dataset/2025_01_15_10_30_00/"

# Define objects to detect
object_names = get_all_instances_of_object("b'Boat")
object_names.extend(get_all_instances_of_object("b'Drone"))
object_names.extend(get_all_instances_of_object("b'StaticMeshActor_13"))

# Get RGB values for objects
object_rgb = get_object_rgb(object_names)

# Process images
for seg_image_file in os.listdir(os.path.join(dataset_path, "segmentation")):
    # Load images
    seg_image = Image.open(os.path.join(dataset_path, "segmentation", seg_image_file))
    depth_image = Image.open(os.path.join(dataset_path, "depth", seg_image_file))
    rgb_image = Image.open(os.path.join(dataset_path, "rgb", seg_image_file))
    
    # Create bounding boxes
    bounding_boxes = create_bounding_boxes(seg_image, object_rgb, depth_image)
    
    # Save annotations
    save_bounding_boxes(bounding_boxes, f"annotations/{seg_image_file.replace('.png', '.xml')}")
    
    # Visualize (optional)
    annotated_image = visualize_bounding_boxes(rgb_image, bounding_boxes, distance=True)
```

### Bounding Box Format
Bounding boxes are returned in the format `[x1, y1, x2, y2, distance]` where:
- `(x1, y1)`: Top-left corner coordinates
- `(x2, y2)`: Bottom-right corner coordinates  
- `distance`: Distance to object in meters (if depth available)

### XML Output Format
```xml
<bounding_boxes>
    <bounding_box name="b'Boat_01" x1="150" y1="200" x2="300" y2="400" distance="25"/>
    <bounding_box name="b'StaticMeshActor_13" x1="500" y1="100" x2="700" y2="350" distance="45"/>
</bounding_boxes>
```


## Integration with AirSim APIs

### Camera Setup
The data generation scripts work with AirSim's camera system. Configure cameras in `settings.json`:

```json
{
  "Vehicles": {
    "Vessel1": {
      "VehicleType": "Vessel",
      "Cameras": {
        "4": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90
            }
          ],
          "X": 0, "Y": 0, "Z": -2,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
```

### Image Request Types
```python
# RGB images
airsim.ImageRequest("4", airsim.ImageType.Scene, False, False)

# Segmentation masks  
airsim.ImageRequest("4", airsim.ImageType.Segmentation, False, False)

# Depth images
airsim.ImageRequest("4", airsim.ImageType.DepthVis, True)
```

## Troubleshooting

### Common Issues

#### Segmentation Color Mapping
**Issue**: Objects not detected in bounding box generation
**Solution**: Check `segmentation_colormap_list.csv` for correct object naming and RGB values.

#### Resolution Issues
**Issue**: Images are not the correct resolution
**Solution**: Check the `CaptureSettings` in `settings.json` for the correct resolution. Try changing the ImageType in settings.json to either "5" (Segmentation), "3" (DepthVis) or "0" (Scene) and restarting the simulation.

For additional examples and advanced usage, see the [Vessel API documentation](vessel_api.md) and [AirSim API reference](../apis.md). 