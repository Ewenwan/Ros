#!/usr/bin/python


""" template_tracker.py - Version 1.1 2013-12-20

    Based on the C++ FastMatchTemplate code by Tristen Georgiou:
    
    http://opencv.willowgarage.com/wiki/FastMatchTemplate
"""

from rbx1_vision.ros2opencv2 import ROS2OpenCV2
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point
import sys
import cv2.cv as cv
import cv2
import numpy as np
from math import pow

class TemplateTracker(ROS2OpenCV2):
    def __init__(self, node_name):
        ROS2OpenCV2.__init__(self, node_name)
        
        self.match_threshold = rospy.get_param("~match_threshold", 0.7)
        self.find_multiple_targets = rospy.get_param("~find_multiple_targets", False)
        self.n_pyr = rospy.get_param("~n_pyr", 2)
        self.min_template_size = rospy.get_param("~min_template_size", 25)

        self.scale_factor = rospy.get_param("~scale_factor", 1.2)
        self.scale_and_rotate = rospy.get_param("~scale_and_rotate", False)
        
        self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
        self.fov_width = rospy.get_param("~fov_width", 1.094)
        self.fov_height = rospy.get_param("~fov_height", 1.094)
        self.max_object_size = rospy.get_param("~max_object_size", 0.28)

        # Intialize the detection box
        self.detect_box = None
        
        self.detector_loaded = False
        
        rospy.loginfo("Waiting for video topics to become available...")

        # Wait until the image topics are ready before starting
        rospy.wait_for_message("input_rgb_image", Image)
        
        if self.use_depth_for_detection:
            rospy.wait_for_message("input_depth_image", Image)
            
        rospy.loginfo("Ready.")
        
    def process_image(self, cv_image):        
        # STEP 1. Load a detector if one is specified
        if not self.detector_loaded:
            self.detector_loaded = self.load_template_detector()
            
        # STEP 2: Detect the object
        self.detect_box = self.match_template(cv_image)
                
        return cv_image
    
    def load_template_detector(self):
        try:
            """ Read in the template image """              
            template_file = rospy.get_param("~template_file", "")
            
            self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR)
            
            cv2.imshow("Template", self.template)
            
            if self.scale_and_rotate:
                """ Compute the min and max scales """
                width_ratio = float(self.frame_size[0]) / self.template.shape[0]
                height_ratio = float(self.frame_size[1]) / self.template.shape[1]
                
                max_scale = 0.9 * min(width_ratio, height_ratio)
                
                max_template_dimension = max(self.template.shape[0], self.template.shape[1])
                min_scale = 1.1 * float(self.min_template_size) / max_template_dimension
                
                self.scales = list()
                scale = min_scale
                while scale < max_scale:
                    self.scales.append(scale)
                    scale *= self.scale_factor
                                    
                self.rotations = [-45, 0, 45]
            else:
                self.scales = [1]
                self.rotations = [0]
                                    
            self.last_scale = 0 # index in self.scales
            self.last_rotation = 0
            
            #self.rotations = [0]
            
            print self.scales
            print self.rotations
            
            return True
        except:
            rospy.loginfo("Exception loading face detector!")
            return False

    
    def match_template(self, frame):
        
        H,W = frame.shape[0], frame.shape[1]
        h,w = self.template.shape[0], self.template.shape[1]

        # Make sure that the template image is smaller than the source
        if W < w or H < h:
            rospy.loginfo( "Template image must be smaller than video frame." )
            return None
        
        if frame.dtype != self.template.dtype: 
            rospy.loginfo("Template and video frame must have same depth and number of channels.")
            return None
        
        # Create a copy of the frame to modify
        frame_copy = frame.copy()
        
        for i in range(self.n_pyr):
            frame_copy = cv2.pyrDown(frame_copy)
            
        template_height, template_width  = self.template.shape[:2]
        
        # Cycle through all scales starting with the last successful scale

        scales = self.scales[self.last_scale:] + self.scales[:self.last_scale - 1]

        # Track which scale and rotation gives the best match
        maxScore = -1
        best_s = 1
        best_r = 0
        best_x = 0
        best_y = 0
        
        for s in self.scales:
            for r in self.rotations:
                # Scale the template by s
                template_copy = cv2.resize(self.template, (int(template_width * s), int(template_height * s)))

                # Rotate the template through r degrees
                rotation_matrix = cv2.getRotationMatrix2D((template_copy.shape[1]/2, template_copy.shape[0]/2), r, 1.0)
                template_copy = cv2.warpAffine(template_copy, rotation_matrix, (template_copy.shape[1], template_copy.shape[0]), borderMode=cv2.BORDER_REPLICATE)
    
                # Use pyrDown() n_pyr times on the scaled and rotated template
                for i in range(self.n_pyr):
                    template_copy = cv2.pyrDown(template_copy)
                
                # Create the results array to be used with matchTempate()
                h,w = template_copy.shape[:2]
                H,W = frame_copy.shape[:2]
                
                result_width = W - w + 1
                result_height = H - h + 1
                
                try:
                    result_mat = cv.CreateMat(result_height, result_width, cv.CV_32FC1)
                    result = np.array(result_mat, dtype = np.float32)
                except:
                    continue
                
                # Run matchTemplate() on the reduced images
                cv2.matchTemplate(frame_copy, template_copy, cv.CV_TM_CCOEFF_NORMED, result)
                
                # Find the maximum value on the result map
                (minValue, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
                
                if maxValue > maxScore:
                    maxScore = maxValue
                    best_x, best_y = maxLoc
                    best_s = s
                    best_r = r
                    best_template = template_copy.copy()
                    self.last_scale = self.scales.index(s)
                    best_result = result.copy()
                
        # Transform back to original image sizes
        best_x *= int(pow(2.0, self.n_pyr))
        best_y *= int(pow(2.0, self.n_pyr))
        h,w = self.template.shape[:2]
        h = int(h * best_s)
        w = int(w * best_s)
        best_result = cv2.resize(best_result, (int(pow(2.0, self.n_pyr)) * best_result.shape[1], int(pow(2.0, self.n_pyr)) * best_result.shape[0]))
        display_result = np.abs(best_result)**3

        cv2.imshow("Result", display_result)
        best_template = cv2.resize(best_template, (int(pow(2.0, self.n_pyr)) * best_template.shape[1], int(pow(2.0, self.n_pyr)) * best_template.shape[0]))
        cv2.imshow("Best Template", best_template)
        
        #match_box = ((best_x + w/2, best_y + h/2), (w, h), -best_r)
        return (best_x, best_y, w, h)

if __name__ == '__main__':
    try:
        node_name = "template_tracker"
        TemplateTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down fast match template node."
      cv.DestroyAllWindows()

    