#!/usr/bin/env python

""" camshift_node.py - Version 1.1 2013-12-20

    Modification of the ROS OpenCV Camshift example using cv_bridge and publishing the ROI
    coordinates to the /roi topic.   
"""

import rospy
import cv2
from cv2 import cv as cv
from rbx1_vision.ros2opencv2 import ROS2OpenCV2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

class CamShiftNode(ROS2OpenCV2):
    def __init__(self, node_name):
        ROS2OpenCV2.__init__(self, node_name)

        self.node_name = node_name
        
        # The minimum saturation of the tracked color in HSV space,
        # as well as the min and max value (the V in HSV) and a 
        # threshold on the backprojection probability image.
        self.smin = rospy.get_param("~smin", 85)
        self.vmin = rospy.get_param("~vmin", 50)
        self.vmax = rospy.get_param("~vmax", 254)
        self.threshold = rospy.get_param("~threshold", 50)
                       
        # Create a number of windows for displaying the histogram,
        # parameters controls, and backprojection image
        cv.NamedWindow("Histogram", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Histogram", 700, 50)
        cv.NamedWindow("Parameters", 0)
        cv.MoveWindow("Parameters", 700, 325)
        cv.NamedWindow("Backproject", 0)
        cv.MoveWindow("Backproject", 700, 600)
        
        # Create the slider controls for saturation, value and threshold
        cv.CreateTrackbar("Saturation", "Parameters", self.smin, 255, self.set_smin)
        cv.CreateTrackbar("Min Value", "Parameters", self.vmin, 255, self.set_vmin)
        cv.CreateTrackbar("Max Value", "Parameters", self.vmax, 255, self.set_vmax)
        cv.CreateTrackbar("Threshold", "Parameters", self.threshold, 255, self.set_threshold)
        
        # Initialize a number of variables
        self.hist = None
        self.track_window = None
        self.show_backproj = False
    
    # These are the callbacks for the slider controls
    def set_smin(self, pos):
        self.smin = pos
        
    def set_vmin(self, pos):
        self.vmin = pos
            
    def set_vmax(self, pos):
       self.vmax = pos
       
    def set_threshold(self, pos):
        self.threshold = pos

    # The main processing function computes the histogram and backprojection
    def process_image(self, cv_image):
        try:
            # First blur the image
            frame = cv2.blur(cv_image, (5, 5))
            
            # Convert from RGB to HSV space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create a mask using the current saturation and value parameters
            mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))
            
            # If the user is making a selection with the mouse, 
            # calculate a new histogram to track
            if self.selection is not None:
                x0, y0, w, h = self.selection
                x1 = x0 + w
                y1 = y0 + h
                self.track_window = (x0, y0, x1, y1)
                hsv_roi = hsv[y0:y1, x0:x1]
                mask_roi = mask[y0:y1, x0:x1]
                self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX);
                self.hist = self.hist.reshape(-1)
                self.show_hist()
    
            if self.detect_box is not None:
                self.selection = None
            
            # If we have a histogram, track it with CamShift
            if self.hist is not None:
                # Compute the backprojection from the histogram
                backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
                
                # Mask the backprojection with the mask created earlier
                backproject &= mask
    
                # Threshold the backprojection
                ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv.CV_THRESH_TOZERO)
    
                x, y, w, h = self.track_window
                if self.track_window is None or w <= 0 or h <=0:
                    self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1
                
                # Set the criteria for the CamShift algorithm
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                
                # Run the CamShift algorithm
                self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)
    
                # Display the resulting backprojection
                cv2.imshow("Backproject", backproject)
        except:
            pass

        return cv_image
        
    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)
        

    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
            histimg_hsv = cv.CreateImage((320, 200), 8, 3)
            
            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)
    
            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv2.rectangle(histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv2.rectangle(histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)
    
            histimg = cv2.cvtColor(histimg_hsv, cv.CV_HSV2BGR)
            
            return histimg
         

if __name__ == '__main__':
    try:
        node_name = "camshift"
        CamShiftNode(node_name)
        try:
            rospy.init_node(node_name)
        except:
            pass
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

