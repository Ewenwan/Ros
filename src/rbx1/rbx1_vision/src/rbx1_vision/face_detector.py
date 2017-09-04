#!/usr/bin/env python

""" face_detector.py - Version 1.1 2013-12-20

    Based on the OpenCV facedetect.py demo code
    
    Extends the ros2opencv2.py script which takes care of user input and image display
    
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

class FaceDetector(ROS2OpenCV2):
    def __init__(self, node_name):
        super(FaceDetector, self).__init__(node_name)
                  
        # Get the paths to the cascade XML files for the Haar detectors.
        # These are set in the launch file.
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")
        cascade_3 = rospy.get_param("~cascade_3", "")
        
        # Initialize the Haar detectors using the cascade files
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)
        self.cascade_3 = cv2.CascadeClassifier(cascade_3)
        
        # Set cascade parameters that tend to work well for faces.
        # Can be overridden in launch file
        self.haar_scaleFactor = rospy.get_param("~haar_scaleFactor", 1.3)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 3)
        self.haar_minSize = rospy.get_param("~haar_minSize", 30)
        self.haar_maxSize = rospy.get_param("~haar_maxSize", 150)
        
        # Store all parameters together for passing to the detector
        self.haar_params = dict(scaleFactor = self.haar_scaleFactor,
                                minNeighbors = self.haar_minNeighbors,
                                flags = cv.CV_HAAR_DO_CANNY_PRUNING,
                                minSize = (self.haar_minSize, self.haar_minSize),
                                maxSize = (self.haar_maxSize, self.haar_maxSize)
                                )
                        
        # Do we should text on the display?
        self.show_text = rospy.get_param("~show_text", True)
        
        # Intialize the detection box
        self.detect_box = None
        
        # Track the number of hits and misses
        self.hits = 0
        self.misses = 0
        self.hit_rate = 0

    def process_image(self, cv_image):
        try:
            # Create a greyscale version of the image
            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Equalize the histogram to reduce lighting effects
            grey = cv2.equalizeHist(grey)
                
            # Attempt to detect a face
            self.detect_box = self.detect_face(grey)
            
            # Did we find one?
            if self.detect_box is not None:
                self.hits += 1
            else:
                self.misses += 1
            
            # Keep tabs on the hit rate so far
            self.hit_rate = float(self.hits) / (self.hits + self.misses)
        except:
            pass
                    
        return cv_image

    def detect_face(self, input_image):
        # First check one of the frontal templates
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, **self.haar_params)
                                         
        # If that fails, check the profile template
        if len(faces) == 0 and self.cascade_3:
            faces = self.cascade_3.detectMultiScale(input_image, **self.haar_params)

        # If that also fails, check a the other frontal template
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image, **self.haar_params)

        # The faces variable holds a list of face boxes.
        # If one or more faces are detected, return the first one.  
        if len(faces) > 0:
            face_box = faces[0]
        else:
            # If no faces were detected, print the "LOST FACE" message on the screen
            if self.show_text:
                font_face = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                cv2.putText(self.marker_image, "LOST FACE!", 
                            (int(self.frame_size[0] * 0.65), int(self.frame_size[1] * 0.9)), 
                            font_face, font_scale, cv.RGB(255, 50, 50))
            face_box = None

        # Display the hit rate so far
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            cv2.putText(self.marker_image, "Hit Rate: " + 
                        str(trunc(self.hit_rate, 2)), 
                        (20, int(self.frame_size[1] * 0.9)), 
                        font_face, font_scale, cv.RGB(255, 255, 0))
        
        return face_box

        
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])
    
if __name__ == '__main__':
    try:
        node_name = "face_detector"
        FaceDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
