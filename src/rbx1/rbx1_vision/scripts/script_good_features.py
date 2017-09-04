#!/usr/bin/env python

from cv2 import cv as cv
import cv2
from common import anorm, clock
from functools import partial
import numpy as np

help_message = '''Good Features to Track

USAGE: good_features.py [ <image> ]
'''

if __name__ == '__main__':
    import sys
    try: fn1 = sys.argv[1]
    except:
        fn1 = "test_images/mona_lisa_face.png"
        
    print help_message
    
    # Good features parameters
    gf_params = dict( maxCorners = 200, 
                   qualityLevel = 0.1,
                   minDistance = 7,
                   blockSize = 20,
                   useHarrisDetector = False,
                   k = 0.04 )

    img = cv2.imread(fn1, cv2.CV_LOAD_IMAGE_COLOR)
    grey = cv2.cvtColor(img, cv.CV_BGR2GRAY)
    
    start = clock()

    keypoints = cv2.goodFeaturesToTrack(grey, mask = None, **gf_params)

    if keypoints is not None:
        for x, y in np.float32(keypoints).reshape(-1, 2):
            cv2.circle(img, (x, y), 3, (0, 255, 0, 0), cv.CV_FILLED, 8, 0)    

    
    print "Elapsed time:", 1000 * (clock() - start), "milliseconds"
    cv2.imshow("Keypoints", img)
    cv2.waitKey()
