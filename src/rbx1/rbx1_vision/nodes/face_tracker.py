#!/usr/bin/env python

""" face_tracker.py - Version 1.1 2013-12-20

    Combines the OpenCV Haar face detector with Good Features to Track and Lucas-Kanade
    optical flow tracking.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

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
from rbx1_vision.face_detector import FaceDetector
from rbx1_vision.lk_tracker import LKTracker

class FaceTracker(FaceDetector, LKTracker):
    def __init__(self, node_name):
        super(FaceTracker, self).__init__(node_name)
                    
        self.n_faces = rospy.get_param("~n_faces", 1)
        self.show_text = rospy.get_param("~show_text", True)
        self.feature_size = rospy.get_param("~feature_size", 1)
        self.face_tracking = True
        
        self.keypoints = list()
        self.detect_box = None
        self.track_box = None
        
        self.grey = None
        self.prev_grey = None
    
    def process_image(self, cv_image):
        try:
            # Create a greyscale version of the image
            self.grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Equalize the grey histogram to minimize lighting effects
            self.grey = cv2.equalizeHist(self.grey)
            
            # STEP 1: Detect the face if we haven't already
            if self.detect_box is None:
                self.detect_box = self.detect_face(self.grey)    
            else:
                # STEP 2: If we aren't yet tracking keypoints, get them now
                if self.track_box is None or not self.is_rect_nonzero(self.track_box):
                    self.track_box = self.detect_box
                    self.keypoints = self.get_keypoints(self.grey, self.track_box)
                
                # STEP 3: If we have keypoints, track them using optical flow
                if len(self.keypoints) > 0:
                    # Store a copy of the current grey image used for LK tracking 
                    if self.prev_grey is None:
                        self.prev_grey = self.grey
                        
                    self.track_box = self.track_keypoints(self.grey, self.prev_grey)
                else:
                    # We have lost all keypoints so re-detect he face
                    self.detect_box = None
            
            # Process any special keyboard commands for this module
            if self.keystroke != -1:
                try:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'c':
                        self.keypoints = list()
                        self.track_box = None
                        self.detect_box = None
                except:
                    pass
    
            # Set store a copy of the current image used for LK tracking         
            self.prev_grey = self.grey
            
        except AttributeError:
            pass
                
        return cv_image                
    
if __name__ == '__main__':
    try:
        node_name = "face_tracker"
        FaceTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face tracker node."
        cv.DestroyAllWindows()
