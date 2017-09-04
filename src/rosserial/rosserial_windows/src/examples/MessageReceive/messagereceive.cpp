/**
Software License Agreement (BSD)

\file      messagereceive.cpp
\authors   Kareem Shehata <kshehata@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <windows.h>

using std::string;

void estimated_pose_callback (const geometry_msgs::PoseWithCovarianceStamped & pose)
{
  printf ("Received pose %f, %f, %f\n", pose.pose.pose.position.x,
          pose.pose.pose.position.y, pose.pose.pose.position.z);
}

int _tmain (int argc, _TCHAR * argv[])
{
  ros::NodeHandle nh;
  char *ros_master = "1.2.3.4";

  printf ("Connecting to server at %s\n", ros_master);
  nh.initNode (ros_master);

  ros::Subscriber < geometry_msgs::PoseWithCovarianceStamped > 
    poseSub ("estimated_pose", &estimated_pose_callback);
  nh.subscribe (poseSub);

  printf ("Waiting to receive messages\n");
  while (1)
  {
    nh.spinOnce ();
    Sleep (100);
  }

  printf ("All done!\n");
  return 0;
}
