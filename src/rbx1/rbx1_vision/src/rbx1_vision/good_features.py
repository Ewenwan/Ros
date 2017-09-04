#!/usr/bin/env python

""" good_features.py - Version 1.1 2013-12-20

    Locate the Good Features To Track in a video stream.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import cv2
import cv2.cv as cv
from rbx1_vision.ros2opencv2 import ROS2OpenCV2
import numpy as np

class GoodFeatures(ROS2OpenCV2):
    def __init__(self, node_name): 
        super(GoodFeatures, self).__init__(node_name)
          
        # Do we show text on the display?
        self.show_text = rospy.get_param("~show_text", True)
        
        # How big should the feature points be (in pixels)?
        self.feature_size = rospy.get_param("~feature_size", 1)
        
        # Good features parameters
        self.gf_maxCorners = rospy.get_param("~gf_maxCorners", 200)
        self.gf_qualityLevel = rospy.get_param("~gf_qualityLevel", 0.02)
        self.gf_minDistance = rospy.get_param("~gf_minDistance", 7)
        self.gf_blockSize = rospy.get_param("~gf_blockSize", 10)
        self.gf_useHarrisDetector = rospy.get_param("~gf_useHarrisDetector", True)
        self.gf_k = rospy.get_param("~gf_k", 0.04)
        
        # Store all parameters together for passing to the detector
        self.gf_params = dict(maxCorners = self.gf_maxCorners, 
                       qualityLevel = self.gf_qualityLevel,
                       minDistance = self.gf_minDistance,
                       blockSize = self.gf_blockSize,
                       useHarrisDetector = self.gf_useHarrisDetector,
                       k = self.gf_k)

        # Initialize key variables
        self.keypoints = list()
        self.detect_box = None
        self.mask = None
        
    def process_image(self, cv_image):
        try:
            # If the user has not selected a region, just return the image
            if not self.detect_box:
                return cv_image
    
            # Create a greyscale version of the image
            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Equalize the histogram to reduce lighting effects
            grey = cv2.equalizeHist(grey)
    
            # Get the good feature keypoints in the selected region
            keypoints = self.get_keypoints(grey, self.detect_box)
            
            # If we have points, display them
            if keypoints is not None and len(keypoints) > 0:
                for x, y in keypoints:
                    cv2.circle(self.marker_image, (x, y), self.feature_size, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
            
            # Process any special keyboard commands
            if self.keystroke != -1:
                try:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'c':
                        # Clear the current keypoints
                        keypoints = list()
                        self.detect_box = None
                except:
                    pass
        except:
            pass
                                
        return cv_image

    def get_keypoints(self, input_image, detect_box):
        # Initialize the mask with all black pixels
        self.mask = np.zeros_like(input_image)
 
        # Get the coordinates and dimensions of the detect_box
        try:
            x, y, w, h = detect_box
        except: 
            return None
        
        # Set the selected rectangle within the mask to white
        self.mask[y:y+h, x:x+w] = 255

        # Compute the good feature keypoints within the selected region
        keypoints = list()
        kp = cv2.goodFeaturesToTrack(input_image, mask = self.mask, **self.gf_params)
        if kp is not None and len(kp) > 0:
            for x, y in np.float32(kp).reshape(-1, 2):
                keypoints.append((x, y))
                
        return keypoints

if __name__ == '__main__':
    try:
        node_name = "good_features"
        GoodFeatures(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Good Features node."
        cv.DestroyAllWindows()
