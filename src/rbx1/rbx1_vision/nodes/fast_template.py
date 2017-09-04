#!/usr/bin/env python

""" Based on the original C++ code referenced below """

"""***************************************************************************
 *            FastMatchTemplate.cpp
 *
 *
 *  Copyright  2010  Tristen Georgiou
 *  tristen_georgiou@hotmail.com
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
"""

from rbx1_vision.ros2opencv2 import ROS2OpenCV2
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point
import sys
import cv2.cv as cv
import cv2
import numpy as np
from math import pow

"""=============================================================================
// Assumes that source image exists and numDownPyrs > 1, no ROIs for either
//  image, and both images have the same depth and number of channels
"""

class FastMatchTemplate(ROS2OpenCV2):
    def __init__(self, node_name):
        
        ROS2OpenCV2.__init__(self, node_name)
        
        self.matchPercentage = rospy.get_param("~matchPercentage", 70)
        self.findMultipleTargets = rospy.get_param("~findMultipleTargets", False)
        self.numMaxima = rospy.get_param("~numMaxima", 1)
        self.numDownPyrs = rospy.get_param("~numDownPyrs", 2)
        self.searchExpansion = rospy.get_param("~searchExpansion", 15)
        
        self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
        self.fov_width = rospy.get_param("~fov_width", 1.094)
        self.fov_height = rospy.get_param("~fov_height", 1.094)
        self.max_object_size = rospy.get_param("~max_object_size", 0.28)

        self.foundPointsList = list()
        self.confidencesList = list()
        
        # Intialize the detection box
        self.detect_box = None
        
        # Initialize a couple of intermediate image variables
        self.grey = None
        self.small_image = None  
        
        # What kind of detector do we want to load
        self.detector_type = "template"
        self.detector_loaded = False
        
        rospy.loginfo("Waiting for video topics to become available...")

        # Wait until the image topics are ready before starting
        rospy.wait_for_message("input_rgb_image", Image)
        
        if self.use_depth_for_detection:
            rospy.wait_for_message("input_depth_image", Image)
            
        rospy.loginfo("Ready.")
        
    def process_image(self, cv_image):        
        #return_image = cv.CreateMat(self.frame_size[1], self.frame_size[0], cv.CV_8UC3)
        #cv.Copy(cv.fromarray(cv_image), return_image)

        # STEP 1. Load a detector if one is specified
        if self.detector_type and not self.detector_loaded:
            self.detector_loaded = self.load_detector(self.detector_type)
            
        # STEP 2: Detect the object
        self.detect_box = self.detect_roi(self.detector_type, cv_image)
                
        return cv_image
    
    def load_detector(self, detector):
        if detector == "template":
            #try:
            """ Read in the template image """              
            template_file = rospy.get_param("~template_file", "")
            
            #self.template =  cv2.equalizeHist(cv2.cvtColor(cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR), cv2.COLOR_BGR2GRAY))
            #self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_GRAYSCALE)
            #self.template = cv2.Sobel(self.template_image, cv.CV_32F, 1, 1)
            
            self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR)
            
            cv2.imshow("Template", self.template)
                            
            return True
            #except:
                #rospy.loginfo("Exception loading face detector!")
                #return False
        else:
            return False
        
    def detect_roi(self, detector, cv_image):
        if detector == "template":
            detect_box = self.match_template(cv_image)
        
        return detect_box
    
    def match_template(self, cv_image):
        frame = np.array(cv_image, dtype=np.uint8)
        
        W,H = frame.shape[1], frame.shape[0]
        w,h = self.template.shape[1], self.template.shape[0]
        width = W - w + 1
        height = H - h + 1

        # Make sure that the template image is smaller than the source
        if W < w or H < h:
            rospy.loginfo( "Template image must be smaller than video frame." )
            return False
        
        if frame.dtype != self.template.dtype: 
            rospy.loginfo("Template and video frame must have same depth and number of channels.")
            return False
    
        # Create copies of the images to modify
        frame_copy = frame.copy()
        template_copy = self.template.copy()
        
        # Down pyramid the images
        for k in range(self.numDownPyrs):
            # Start with the source image
            W  = (W  + 1) / 2
            H = (H + 1) / 2
                
            frame_small = np.array([H, W], dtype=frame.dtype)
            frame_small = cv2.pyrDown(frame_copy)
            
#            frame_window = "PyrDown " + str(k)
#            cv.NamedWindow(frame_window, cv.CV_NORMAL)
#            cv.ShowImage(frame_window, cv.fromarray(frame_small))
#            cv.ResizeWindow(frame_window, 640, 480)
    
            #  Prepare for next loop, if any
            frame_copy = frame_small.copy()
    
            #Next, do the target
            w  = (w  + 1) / 2
            h = (h + 1) / 2
    
            template_small = np.array([h, w], dtype=self.template.dtype)
            template_small = cv2.pyrDown(template_copy)
            
#            template_window = "Template PyrDown " + str(k)
#            cv.NamedWindow(template_window, cv.CV_NORMAL)
#            cv.ShowImage(template_window, cv.fromarray(template_small))
#            cv.ResizeWindow(template_window, 640, 480)
    
            # Prepare for next loop, if any
            template_copy = template_small.copy()
            
        # Perform the match on the shrunken images
        small_frame_width = frame_copy.shape[1]
        small_frame_height = frame_copy.shape[0]
        
        small_template_width = template_copy.shape[1]
        small_template_height = template_copy.shape[0]
    
        result_width = small_frame_width - small_template_width + 1
        result_height = small_frame_height - small_template_height + 1
    
        result_mat = cv.CreateMat(result_height, result_width, cv.CV_32FC1)
        result = np.array(result_mat, dtype = np.float32)

        cv2.matchTemplate(frame_copy, template_copy, cv.CV_TM_CCOEFF_NORMED, result)
        
        cv2.imshow("Result", result)
        
        return (0, 0, 100, 100)
        
#        # Find the best match location
#        (minValue, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)
#        
#       # Transform point back to original image
#        target_location = Point()
#        target_location.x, target_location.y = maxLoc
#        
#        return (target_location.x, target_location.y, w, h)
                
        # Find the top match locations
        locations = self.MultipleMaxLoc(result, self.numMaxima)

        foundPointsList = list()
        confidencesList = list()
        
        W,H = frame.shape[1], frame.shape[0]
        w,h = self.template.shape[1], self.template.shape[0]
        
        # Search the large images at the returned locations       
        for currMax in range(self.numMaxima):
            # Transform the point to its corresponding point in the larger image
            #locations[currMax].x *= int(pow(2.0, self.numDownPyrs))
            #locations[currMax].y *= int(pow(2.0, self.numDownPyrs))
            locations[currMax].x += w / 2
            locations[currMax].y += h / 2
    
            searchPoint = locations[currMax]
            
            print "Search Point", searchPoint
    
            # If we are searching for multiple targets and we have found a target or
            # multiple targets, we don't want to search in the same location(s) again
#            if self.findMultipleTargets and len(foundPointsList) != 0:
#                thisTargetFound = False
#                numPoints = len(foundPointsList)
#                
#                for currPoint in range(numPoints):
#                    foundPoint = foundPointsList[currPoint]
#                    if (abs(searchPoint.x - foundPoint.x) <= self.searchExpansion * 2) and (abs(searchPoint.y - foundPoint.y) <= self.searchExpansion * 2):
#                        thisTargetFound = True
#                        break
#    
#                # If the current target has been found, continue onto the next point
#                if thisTargetFound:
#                    continue
    
            # Set the source image's ROI to slightly larger than the target image,
            # centred at the current point
            searchRoi = RegionOfInterest()
            searchRoi.x_offset = searchPoint.x - w / 2 - self.searchExpansion
            searchRoi.y_offset = searchPoint.y - h / 2 - self.searchExpansion
            searchRoi.width = w + self.searchExpansion * 2
            searchRoi.height = h + self.searchExpansion * 2
            
            #print (searchRoi.x_offset, searchRoi.y_offset, searchRoi.width, searchRoi.height)
                
            # Make sure ROI doesn't extend outside of image
            if searchRoi.x_offset < 0: 
                searchRoi.x_offset = 0

            if searchRoi.y_offset < 0:
                searchRoi.y_offset = 0

            if (searchRoi.x_offset + searchRoi.width) > (W - 1):
                numPixelsOver = (searchRoi.x_offset + searchRoi.width) - (W - 1)
                print "NUM PIXELS OVER", numPixelsOver
                searchRoi.width -= numPixelsOver
    
            if (searchRoi.y_offset + searchRoi.height) > (H - 1):
                numPixelsOver = (searchRoi.y_offset + searchRoi.height) - (H - 1)
                searchRoi.height -= numPixelsOver
                
            mask = (searchRoi.x_offset, searchRoi.y_offset, searchRoi.width, searchRoi.height)
    
            frame_mat = cv.fromarray(frame)
        
            searchImage = cv.CreateMat(searchRoi.height, searchRoi.width, cv.CV_8UC3)
            searchImage = cv.GetSubRect(frame_mat, mask)
            searchArray = np.array(searchImage, dtype=np.uint8)
                       
            # Perform the search on the large images
            result_width = searchRoi.width - w + 1
            result_height = searchRoi.height - h + 1
    
            result_mat = cv.CreateMat(result_height, result_width, cv.CV_32FC1)
            result = np.array(result_mat, dtype = np.float32)
            
            cv2.matchTemplate(searchArray, self.template, cv.CV_TM_CCOEFF_NORMED, result)
    
            # Find the best match location
            (minValue, maxValue, minLoc, maxLoc) = cv2.minMaxLoc(result)

            maxValue *= 100
    
            # Transform point back to original image
            target_location = Point()
            target_location.x, target_location.y = maxLoc
            target_location.x += searchRoi.x_offset - w / 2 + self.searchExpansion
            target_location.y += searchRoi.y_offset - h / 2 + self.searchExpansion
    
            if maxValue >= self.matchPercentage:
                # Add the point to the list
                foundPointsList.append(maxLoc)
                confidencesList.append(maxValue)
    
                # If we are only looking for a single target, we have found it, so we
                # can return
                if not self.findMultipleTargets:
                    break
    
        if len(foundPointsList) == 0:
            rospy.loginfo("Target was not found to required confidence")
    
        return (target_location.x, target_location.y, w, h)
    
    def MultipleMaxLoc(self, result, numMaxima):
        # Initialize input variable locations
        #locations = np.empty((numMaxima, 2), dtype=np.uint8)
        locations = [Point()]*numMaxima
    
        # Create array for tracking maxima
        maxima = [0.0]*numMaxima
        
        result_width = result.shape[1]
        result_height = result.shape[0]
        
        # Extract the raw data for analysis
        for y in range(result_height):
            for x in range(result_width):
                data = result[y, x]
                
                # Insert the data value into the array if it is greater than any of the
                # other array values, and bump the other values below it, down
                for j in range(numMaxima):
                    # Require at least 50% confidence on the sub-sampled image
                    # in order to make this as fast as possible
                    if data > 0.5 and data > maxima[j]:
                        # Move the maxima down
                        k = numMaxima - 1
                        while k > j:
                            maxima[k] = maxima[k-1]
                            locations[k] = locations[k-1]
                            k = k - 1
    
                        # Insert the value
                        maxima[j] = data
                        locations[j].x = x
                        locations[j].y = y
                        break

        return locations
    
#//=============================================================================
#
#void
#DrawFoundTargets(Mat*                  image,
#                 const Size&           size,
#                 const vector<Point>&  pointsList,
#                 const vector<double>& confidencesList,
#                 int                   red,
#                 int                   green,
#                 int                   blue)
#{
#    int numPoints = pointsList.size();
#    for(int currPoint = 0; currPoint < numPoints; currPoint++)
#    {
#        const Point& point = pointsList[currPoint];
#
#        // write the confidences to stdout
#        rospy.loginfo("\nTarget found at (%d, %d), with confidence = %3.3f %%.\n",
#            point.x,
#            point.y,
#            confidencesList[currPoint]);
#
#        // draw a circle at the center
#        circle(*image, point, 2, CV_RGB(red, green, blue));
#
#        // draw a rectangle around the found target
#        Point topLeft;
#        topLeft.x = point.x - size.width / 2;
#        topLeft.y = point.y - size.height / 2;
#
#        Point bottomRight;
#        bottomRight.x = point.x + size.width / 2;
#        bottomRight.y = point.y + size.height / 2;
#
#        rectangle(*image, topLeft, bottomRight, CV_RGB(red, green, blue));
#    }
#}


def main(args):
      FMT = FastMatchTemplate("fast_match_template")
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down fast match template node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    