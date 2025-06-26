__author__ = "Bavo Lesy"
__credits__ = ["Bavo Lesy", "Siemen Herremans", "Robin Kerstens, Ali Anwar, Siegfried Mercelis"]
__version__ = "1.0.1"
import cosysairsim as airsim

import time
import os
from cosysairsim.types import VesselControls, DisturbanceControls
from datetime import datetime
import numpy as np
from PIL import Image
import cv2

"""
This script is used to generate a dataset of rgb and segmentation images of the vessel. 
These images can then used to train computer vision models to detect the vessel in images. This approach may also work for other sensor modalities, such as lidar and radar. This script assumes the usage of the Segmentation Imagetype in settings.json and that the rgb and segmentation images are both collected using the same camera (same position, same resolution, same FOV).

If you face an issue with incorrect image resolution, try changing the ImageType in settings.json to either "5" (Segmentation), "3" (DepthVis) or "0" (Scene) and restarting the simulation.
"""

def initialize_data_collection(client, dataset_path):
    """
    This function initializes the data collection by creating the necessary folders and files.
    """
    #create path for current run
    current_run_path = os.path.join(dataset_path, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    os.makedirs(current_run_path, exist_ok=True)
    #create folders for segmentation and rgb images
    segmentation_folder = os.path.join(current_run_path, 'segmentation')
    rgb_folder = os.path.join(current_run_path, 'rgb')
    depth_folder = os.path.join(current_run_path, 'depth')
    os.makedirs(segmentation_folder, exist_ok=True)
    os.makedirs(rgb_folder, exist_ok=True)
    os.makedirs(depth_folder, exist_ok=True)
    #open the segmentation color map and get all objects in the scene
    colorMap = client.simGetSegmentationColorMap()
    currentObjectList = client.simListInstanceSegmentationObjects()
    #Map the object names to the color map
    with open(os.path.join(current_run_path, 'segmentation_colormap_list.csv'), 'w') as f:
        f.write("ObjectName,R,G,B\n")
        for index, item in enumerate(currentObjectList):
            f.write("%s,%s\n" % (item, ','.join([str(x) for x in colorMap[index,:]])))
    return segmentation_folder, rgb_folder, depth_folder


def generate_dataset(client, segmentation_folder, rgb_folder, depth_folder, max_depth, num_images):
    """
    This function generates the dataset by collecting the rgb and segmentation images.
    """
    #Set the vessel to a constant speed and heading
    thrust = [0.5, 0.5]
    angle = [0.5, 0.5]
    client.setVesselControls('', VesselControls(thrust, angle))    
    
    for i in range(num_images): 
        #Collect the images
        start_time = time.time()
        response = client.simGetImages([
            airsim.ImageRequest("4", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("4", airsim.ImageType.Segmentation, False, False),
            airsim.ImageRequest("4", airsim.ImageType.DepthVis, True),
            ])  
        #Save the rgb image
        rgb_image = np.frombuffer(response[0].image_data_uint8, dtype=np.uint8)
        rgb_image = rgb_image.reshape([response[0].height, response[0].width, 3])
        rgb_image = Image.fromarray(rgb_image, 'RGB')
        rgb_image.save(os.path.join(rgb_folder, '{}.png'.format(i)))
        #Save the segmentation image
        segmentation_image = np.frombuffer(response[1].image_data_uint8, dtype=np.uint8)
        segmentation_image = segmentation_image.reshape([response[1].height, response[1].width, 3])
        segmentation_image = Image.fromarray(segmentation_image, 'RGB')
        segmentation_image.save(os.path.join(segmentation_folder, '{}.png'.format(i)))
        #Save the depth image, is grayscale
        depth_image = np.array(response[2].image_data_float, dtype=np.float32)
        depth_image = depth_image.reshape([response[2].height, response[2].width])
        depth_image[depth_image > max_depth/100] = max_depth/100
        depth_image = np.array(depth_image*(25500/max_depth), dtype=np.uint8)
        cv2.imwrite(os.path.join(depth_folder, '{}.png'.format(i)), depth_image)
        #Check if the vessel has collided with an object
        if client.simGetCollisionInfo().has_collided:
            #If so, reset the vessel to the start position
            client.setVesselControls('', VesselControls(0, 0.5))
            client.reset()
            client.setVesselControls('', VesselControls(0.2, 0.5))
        end_time = time.time()
        #print(f"Time taken to collect images: {end_time - start_time} seconds")
        time.sleep(4 - (end_time - start_time)) #take pictures every 2 seconds


def capture_images(client,rgb_folder, segmentation_folder, num_images):
    """
    This function captures RGB images as fast as possible and then saves them in the rgb_folder sequentially after capture. only to be used for capturing fast and small amounts of data.
    """
    client.setVesselControls('', VesselControls(0.2, 0.5))    
    responses = []
    for i in range(num_images):    
        #Collect the images
        start_time = time.time()
        responses.append(client.simGetImages([
            airsim.ImageRequest("4", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("4", airsim.ImageType.Segmentation, False, False),
            ]))
        end_time = time.time()
        print(f"Time taken to collect images: {end_time - start_time} seconds")
    #save images in rgb_folder sequentially after capture
    for i in range(num_images):
        rgb_image = np.frombuffer(responses[i][0].image_data_uint8, dtype=np.uint8)
        rgb_image = rgb_image.reshape([responses[i][0].height, responses[i][0].width, 3])
        rgb_image = Image.fromarray(rgb_image, 'RGB')
        rgb_image.save(os.path.join(rgb_folder, '{}.png'.format(i)))
        segmentation_image = np.frombuffer(responses[i][1].image_data_uint8, dtype=np.uint8)
        segmentation_image = segmentation_image.reshape([responses[i][1].height, responses[i][1].width, 3])
        segmentation_image = Image.fromarray(segmentation_image, 'RGB')
        segmentation_image.save(os.path.join(segmentation_folder, '{}.png'.format(i)))

if __name__ == "__main__":
    dataset_path = "Vessel/data_generation/dataset"
    num_images = 10
    max_depth = 255 # distance how far you want depth camera to see. Higher distance means less precision.
    client = airsim.VesselClient()  
    client.confirmConnection()
    client.enableApiControl(True)
    segmentation_folder, rgb_folder, depth_folder = initialize_data_collection(client, dataset_path)
    generate_dataset(client, segmentation_folder, rgb_folder, depth_folder, max_depth, num_images)
    #capture_images(client, rgb_folder, segmentation_folder, num_images)