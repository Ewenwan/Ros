#!/usr/bin/env python

""" video2ros.py - Version 1.1 2013-12-20

    Read in a recorded video file and republish as a ROS Image topic.
    
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
import sys
from cv2 import cv as cv
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Video2ROS:
    def __init__(self):
        rospy.init_node('video2ros', anonymous=False)
        
        rospy.on_shutdown(self.cleanup)
        
        """ Define the input (path to video file) as a ROS parameter so it
            can be defined in a launch file or on the command line """
        self.input = rospy.get_param("~input", "")
        
        """ Define the image publisher with generic topic name "output" so that it can
            be remapped in the launch file. """
        image_pub = rospy.Publisher("output", Image, queue_size=5)
        
        # The target frames per second for the video
        self.fps = rospy.get_param("~fps", 25)
        
        # Do we restart the video when the end is reached?
        self.loop = rospy.get_param("~loop", False)
        
        # Start the video paused?
        self.start_paused = rospy.get_param("~start_paused", False)
        
        # Show text feedback by default?
        self.show_text = rospy.get_param("~show_text", True)
        
        # Resize the original video?
        self.resize_video = rospy.get_param("~resize_video", False)
        
        # If self.resize_video is True, set the desired width and height here
        self.resize_width = rospy.get_param("~resize_width", 0)
        self.resize_height = rospy.get_param("~resize_height", 0)
        
        # Initialize a few variables
        self.paused = self.start_paused
        self.loop_video = True
        self.keystroke = None
        self.restart = False
        self.last_frame = None
        
        # Initialize the text font
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        
        # Define the capture object as pointing to the input file
        self.capture = cv2.VideoCapture(self.input)
        
        # Get the original frames per second
        fps = self.capture.get(cv.CV_CAP_PROP_FPS)
        
        # Get the original frame size
        self.frame_size = (self.capture.get(cv.CV_CAP_PROP_FRAME_HEIGHT), self.capture.get(cv.CV_CAP_PROP_FRAME_WIDTH))
        
        # Check that we actually have a valid video source
        if fps == 0.0:
            print "Video source", self.input, "not found!"
            return None        
        
        # Bring the fps up to the specified rate
        try:
            fps = int(fps * self.fps / fps)
        except:
            fps = self.fps
    
        # Create the display window
        cv.NamedWindow("Video Playback", True) # autosize the display
        cv.MoveWindow("Video Playback", 375, 25)

        # Create the CvBridge object
        bridge = CvBridge()
    
        # Enter the main processing loop
        while not rospy.is_shutdown():
            # Get the next frame from the video
            frame = self.get_frame()
            
            # Convert the frame to ROS format
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError, e:
                print e
            
            # Create a copy of the frame for displaying the text
            display_image = frame.copy()
            
            if self.show_text:
                cv2.putText(display_image, "FPS: " + str(self.fps), (10, 20), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, "Keyboard commands:", (20, int(self.frame_size[0] * 0.6)), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, " ", (20, int(self.frame_size[0] * 0.65)), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, "space - toggle pause/play", (20, int(self.frame_size[0] * 0.72)), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, "     r - restart video from beginning", (20, int(self.frame_size[0] * 0.79)), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, "     t - hide/show this text", (20, int(self.frame_size[0] * 0.86)), font_face, font_scale, cv.RGB(255, 255, 0))
                cv2.putText(display_image, "     q - quit the program", (20, int(self.frame_size[0] * 0.93)), font_face, font_scale, cv.RGB(255, 255, 0))
            
            # Merge the original image and the display image (text overlay)
            display_image = cv2.bitwise_or(frame, display_image)
            
            # Now display the image.
            cv2.imshow("Video Playback", display_image)
                    
            """ Handle keyboard events """
            self.keystroke = cv.WaitKey(1000 / fps)

            """ Process any keyboard commands """
            if self.keystroke != -1:
                try:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'q':
                        """ user has press the q key, so exit """
                        rospy.signal_shutdown("User hit q key to quit.")
                    elif cc == ' ':
                        """ Pause or continue the video """
                        self.paused = not self.paused
                    elif cc == 'r':
                        """ Restart the video from the beginning """
                        self.restart = True
                    elif cc == 't':
                        """ Toggle display of text help message """
                        self.show_text = not self.show_text
                except:
                    pass     
        
                    
    def get_frame(self):
        # If the video is paused, return the last frame
        if self.paused and not self.last_frame is None:
            frame = self.last_frame
        else:
            # Get the next frame
            ret, frame = self.capture.read()

        if frame is None:
            # If we've run out of frames, loop if set True
            if self.loop_video:
                self.restart = True     
        
        # Did we receive the restart command?
        if self.restart:
            # To restart, simply reinitialize the capture object
            self.capture = cv2.VideoCapture(self.input)
            self.restart = False
            ret, frame = self.capture.read()
        
        # Were we asked to resize the video?
        if self.resize_video:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height))
        
        # Store the last frame for when we pause
        self.last_frame = frame
            
        return frame
    
    def cleanup(self):
            print "Shutting down video2ros node."
            cv2.destroyAllWindows()

def main(args):
    help_message =  "Hot keys: \n" \
          "\tq     - quit the program\n" \
          "\tr     - restart video from beginning\n" \
          "\tspace - toggle pause/play\n"

    print help_message
    
    try:
        v2r = Video2ROS()
    except KeyboardInterrupt:
        print "Shutting down video2ros..."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
