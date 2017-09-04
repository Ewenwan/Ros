#!/usr/bin/env python

""" lk_tracker.py - Version 1.1 2013-12-20

    Based on the OpenCV lk_track.py demo code
    
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
import numpy as np
from rbx1_vision.good_features import GoodFeatures

class LKTracker(GoodFeatures):
    def __init__(self, node_name):
        super(LKTracker, self).__init__(node_name)
        
        self.show_text = rospy.get_param("~show_text", True)
        self.feature_size = rospy.get_param("~feature_size", 1)
                
        # LK parameters
        self.lk_winSize = rospy.get_param("~lk_winSize", (10, 10))
        self.lk_maxLevel = rospy.get_param("~lk_maxLevel", 2)
        self.lk_criteria = rospy.get_param("~lk_criteria", (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))
        
        self.lk_params = dict( winSize  = self.lk_winSize, 
                  maxLevel = self.lk_maxLevel, 
                  criteria = self.lk_criteria)    
        
        self.detect_interval = 1
        self.keypoints = None

        self.detect_box = None
        self.track_box = None
        self.mask = None
        self.grey = None
        self.prev_grey = None
            
    def process_image(self, cv_image):
        try:
            # If we don't yet have a detection box (drawn by the user
            # with the mouse), keep waiting
            if self.detect_box is None:
                return cv_image
    
            # Create a greyscale version of the image
            self.grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Equalize the grey histogram to minimize lighting effects
            self.grey = cv2.equalizeHist(self.grey)
            
            # If we haven't yet started tracking, set the track box to the
            # detect box and extract the keypoints within it
            if self.track_box is None or not self.is_rect_nonzero(self.track_box):
                self.track_box = self.detect_box
                self.keypoints = self.get_keypoints(self.grey, self.track_box)
            
            else:
                if self.prev_grey is None:
                    self.prev_grey = self.grey
        
                # Now that have keypoints, track them to the next frame
                # using optical flow
                self.track_box = self.track_keypoints(self.grey, self.prev_grey)
    
            # Process any special keyboard commands for this module
            if self.keystroke != -1:
                try:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'c':
                        # Clear the current keypoints
                        self.keypoints = None
                        self.track_box = None
                        self.detect_box = None
                except:
                    pass
                    
            self.prev_grey = self.grey
        except:
            pass
                
        return cv_image               
                    
    def track_keypoints(self, grey, prev_grey):
        # We are tracking points between the previous frame and the
        # current frame
        img0, img1 = prev_grey, grey
        
        # Reshape the current keypoints into a numpy array required
        # by calcOpticalFlowPyrLK()
        p0 = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)
        
        # Calculate the optical flow from the previous frame to the current frame
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
        
        # Do the reverse calculation: from the current frame to the previous frame
        try:
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            
            # Compute the distance between corresponding points in the two flows
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            
            # If the distance between pairs of points is < 1 pixel, set
            # a value in the "good" array to True, otherwise False
            good = d < 1
        
            # Initialize a list to hold new keypoints
            new_keypoints = list()
            
            # Cycle through all current and new keypoints and only keep
            # those that satisfy the "good" condition above
            for (x, y), good_flag in zip(p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                new_keypoints.append((x, y))
                
                # Draw the keypoint on the image
                cv2.circle(self.marker_image, (x, y), self.feature_size, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)
            
            # Set the global keypoint list to the new list    
            self.keypoints = new_keypoints
            
            # Convert the keypoints list to a numpy array
            keypoints_array = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)  
            
            # If we have enough points, find the best fit ellipse around them
            if len(self.keypoints) > 6:
                track_box = cv2.fitEllipse(keypoints_array)
            else:
                # Otherwise, find the best fitting rectangle
                track_box = cv2.boundingRect(keypoints_array)
        except:
            track_box = None
                        
        return track_box
    
if __name__ == '__main__':
    try:
        node_name = "lk_tracker"
        LKTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down LK Tracking node."
        cv.DestroyAllWindows()
    