__author__ = "Bavo Lesy"
__credits__ = ["Bavo Lesy", "Siemen Herremans", "Robin Kerstens, Ali Anwar, Siegfried Mercelis"]
__version__ = "1.0.1"

import cv2
import numpy as np
from PIL import Image
import pandas as pd
import os
import xml.etree.ElementTree as ET

"""
This script contains the functions to create bounding boxes around specified objects on images captured within IDLab-ShippingSim

For this script to work, the following assumptions are made:
- Your dataset was captured using the dataset_generation.py (or similar) script
- You have a dataset folder which contains the segmentation image, the rgb image and the segmentation color map csv file
- The bounding boxes are returned in the format of [x1, y1, x2, y2] and are saved as a xml file
"""


def create_bounding_boxes(image, objects, depth_image):
    """
    Creates masks and bounding boxes for each object specified in the image 
    """
    image_np = np.array(image)  # Convert image to NumPy array (H, W, 3)
    bounding_boxes = {}

    for obj in objects:
        # Create boolean mask where pixels match the object color
        #get obj rgb from color map
        obj_rgb = obj[["R", "G", "B"]]
        mask = np.all(image_np == np.array(obj_rgb), axis=-1)

        if np.any(mask):  # Check if object exists in the image
            # Get bounding box coordinates
            y_indices, x_indices = np.where(mask)
            x1, x2 = x_indices.min(), x_indices.max()
            y1, y2 = y_indices.min(), y_indices.max()
            if x1 < x2 and y1 < y2:
                if depth_image is not None:
                    distance = get_distance_to_object(depth_image, [x1, y1, x2, y2])
                    bounding_boxes[obj["ObjectName"]] = [x1, y1, x2, y2, distance]  # Bounding box
                else:
                    bounding_boxes[obj["ObjectName"]] = [x1, y1, x2, y2]  # Bounding box        
    return bounding_boxes


def get_object_rgb(object_names):
    """
    This function returns the RGB value of the objects
    """
    color_map = pd.read_csv(os.path.join(dataset_path, "segmentation_colormap_list.csv"))
    object_rgb = []
    for object_name in object_names:
        index = color_map[color_map["ObjectName"] == object_name].index[0]
        object_rgb.append(color_map.iloc[index])
    return object_rgb

def get_all_instances_of_object(object_name):
    """
    Get all objects that start with the object name
    """
    color_map = pd.read_csv(os.path.join(dataset_path, "segmentation_colormap_list.csv"))
    #Get all object names that start with the object name
    object_names = color_map[color_map["ObjectName"].str.startswith(object_name)]["ObjectName"].tolist()
    return object_names
   
def visualize_bounding_boxes(image, bounding_boxes, distance=False):
    """
    This function visualizes the bounding boxes on the image
    """
    #convert image to np array
    image_np = np.array(image)
    for obj_rgb in bounding_boxes:
        if obj_rgb.startswith("b'StaticMeshActor_13") or obj_rgb.startswith("b'StaticMeshActor_14") or obj_rgb.startswith("b'StaticMeshActor_15"):
            color = (0, 165, 255)
            name = "Cargo Vessel"
        else:
            color = (255, 0, 0)
            name = "Vessel"
        cv2.rectangle(image_np, (bounding_boxes[obj_rgb][0], bounding_boxes[obj_rgb][1]), (bounding_boxes[obj_rgb][2], bounding_boxes[obj_rgb][3]), color, 5)
        #increase font size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.5
        font_thickness = 2
        
        cv2.putText(image_np, name, (bounding_boxes[obj_rgb][0], bounding_boxes[obj_rgb][1] - 10), font, font_scale, color, font_thickness, cv2.LINE_AA)
        if distance:
            cv2.putText(image_np, f"Distance: {bounding_boxes[obj_rgb][4]}m", (bounding_boxes[obj_rgb][0], bounding_boxes[obj_rgb][1] - 50), font, font_scale, color, font_thickness, cv2.LINE_AA)
    return image_np


def save_bounding_boxes(bounding_boxes, output_path):
    """
    This function saves the bounding boxes to a xml file
    """
    #create a xml file
    tree = ET.Element("bounding_boxes")
    for obj_rgb in bounding_boxes:
        bounding_box = bounding_boxes[obj_rgb]
        bounding_box_element = ET.SubElement(tree, "bounding_box")
        bounding_box_element.set("name", obj_rgb)
        bounding_box_element.set("x1", str(bounding_box[0]))
        bounding_box_element.set("y1", str(bounding_box[1]))
        bounding_box_element.set("x2", str(bounding_box[2]))
        bounding_box_element.set("y2", str(bounding_box[3]))
        if len(bounding_box) > 4:
            bounding_box_element.set("distance", str(bounding_box[4]))
    #save the xml file
    tree = ET.ElementTree(tree)
    tree.write(output_path)

def get_distance_to_object(image, bounding_box, max_distance=255):
    """
    This function returns the closest distance to the object in the image. Use the depth image to get the distance.
    """
    image_np = np.array(image)
    depth_image = image_np * (255 / max_distance)
    x1 = bounding_box[0]
    y1 = bounding_box[1]
    x2 = bounding_box[2]
    y2 = bounding_box[3]
    distance = int(depth_image[y1:y2, x1:x2].min())

    return distance

if __name__ == "__main__":
    dataset_path = "Vessel/data_generation/dataset/2025_04_10_13_14_00/"
    object_names = get_all_instances_of_object("b'Boat") #Every object that starts with Boat and b'Drone
    object_names.extend(get_all_instances_of_object("b'Drone")) #default name of pawn (own vessel)
    object_names.extend(get_all_instances_of_object("b'StaticMeshActor_13")) #default name of pawn (own vessel)
    object_names.extend(get_all_instances_of_object("b'StaticMeshActor_14")) #default name of pawn (own vessel)
    object_names.extend(get_all_instances_of_object("b'StaticMeshActor_15")) #default name of pawn (own vessel)
    object_rgb = get_object_rgb(object_names)
    for seg_image in os.listdir(os.path.join(dataset_path, "segmentation")):
        #create bounding boxes
        image = Image.open(os.path.join(dataset_path, "segmentation", seg_image))
        depth_image = Image.open(os.path.join(dataset_path, "depth", seg_image))
        bounding_boxes = create_bounding_boxes(image, object_rgb, depth_image)
        #save bounding boxes
        if not os.path.exists(os.path.join(dataset_path, "bounding_boxes")):
            os.makedirs(os.path.join(dataset_path, "bounding_boxes"))
        save_bounding_boxes(bounding_boxes, os.path.join(dataset_path, "bounding_boxes", seg_image.replace(".png", ".xml")))
        """Visualization of bounding boxes is also possible, but turned off by default due to the large number of images that might be generated"""
        #visualize bounding boxes on rgb image
        rgb_image = Image.open(os.path.join(dataset_path, "rgb", seg_image))
        image_np = visualize_bounding_boxes(rgb_image, bounding_boxes, distance=True)
        #show image
        Image.fromarray(image_np).show()



