__author__ = "Bavo Lesy"
__version__ = "1.0.1"

import cv2
import numpy as np
"""
Script to emulate radar sensor data using lidar data and point spread functions.
"""

class simulateRadar:
    def __init__(self):
        self.ellipseWidth = 15000 #higher means smaller size 
        self.ellipseHeight = 5000 # higher means smaller size
        self.rangeFactor = 0.2 #range/size dependency

    def simulatePSF(self, points):
        gridSize = 900 # Define a suitable grid size, not too big --> slow
        radarPosition = np.array([450, 450])  # Radar position in the grid
        radarPSF = np.zeros((gridSize, gridSize), dtype=np.float32)
        lidar = np.zeros((gridSize, gridSize), dtype=np.float32)
        normalizedPoints = []
        # Normalize radarPosition to fit within the grid
        if(points.size > 0):
            minPoints = points.min(axis=0)
            rangePoints = points.max(axis=0) - minPoints
            normalizedRadarPosition = ((radarPosition - minPoints) / rangePoints)
            normalizedRadarPosition = np.round(normalizedRadarPosition * (gridSize - 1)).astype(int) + 1
            print(normalizedRadarPosition)

            normalizedPoints = ((points - minPoints) / rangePoints)
            normalizedPoints = np.round(normalizedPoints * (gridSize - 1)).astype(int) + 1
            
            # Use the class properties instead of local variables
            ellipseWidth = self.ellipseWidth
            ellipseHeight = self.ellipseHeight
            rangeFactor = self.rangeFactor

        for point in normalizedPoints:
            x, y = point
            
            # Calculate distance from radar position
            dirVector = point - radarPosition
            print(dirVector)
            perpVector = [-dirVector[1], dirVector[0]]
            angle = (np.arctan2(perpVector[1], perpVector[0]) * 180) / np.pi
            
            # Determine ellipse size based on distance from radar
            distance = np.linalg.norm(point - radarPosition)
            a = max(3, round((distance * rangeFactor) * gridSize / ellipseWidth))
            b = max(2, round((distance * rangeFactor) * gridSize / ellipseHeight))
            
            # Draw the ellipse
            center = (int(x), int(y))
            axes = (b, a)
            cv2.ellipse(radarPSF, center, axes, angle, 0, 360, 1.0, -1)
            cv2.circle(lidar, center, 1, 1.0, -1)

        radarPSF = np.stack((radarPSF, radarPSF, radarPSF), axis=2) * 255
        cv2.circle(radarPSF, (radarPosition[0], radarPosition[1]), 5, (0, 255, 0), -1)
        lidar = np.stack((lidar, lidar, lidar), axis=2) * 255
        cv2.circle(lidar, (radarPosition[0], radarPosition[1]), 5, (0, 255, 0), -1)
        radarPSF = radarPSF.astype(np.uint8)
        lidar = lidar.astype(np.uint8)
     
        return radarPSF, lidar
    
    



        
            




