#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Clearpath Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

""" The node provided by this file exposes a helper service used by
the C++ rosserial_server. The service allows the C++ driver to look
up message definitions and hash strings which would not have been
known to it at compile time, allowing it to fully advertise topics
originating from microcontrollers.

This allows rosserial_server to be distributed in binary form. """

import rospy
from rosserial_msgs.srv import RequestMessageInfo
from rosserial_msgs.srv import RequestServiceInfo
from rosserial_python import load_message
from rosserial_python import load_service


class MessageInfoService(object):
  """  """
  def __init__(self):
    rospy.init_node("message_info_service")
    rospy.loginfo("rosserial message_info_service node")
    self.service = rospy.Service("message_info", RequestMessageInfo, self._message_info_cb)
    self.serviceInfoService = rospy.Service("service_info", RequestServiceInfo, self._service_info_cb)
    self.message_cache = {}
    self.service_cache = {}

  def _message_info_cb(self, req):
    package_message = tuple(req.type.split("/"))
    if not self.message_cache.has_key(package_message):
      rospy.loginfo("Loading module to return info on %s/%s." % package_message)
      msg = load_message(*package_message)
      self.message_cache[package_message] = (msg._md5sum, msg._full_text)
    else:
      rospy.loginfo("Returning info from cache on %s/%s." % package_message)
    return self.message_cache[package_message]

  def _service_info_cb(self, req):
    rospy.logdebug("req.service is %s" % req.service)
    package_service = tuple(req.service.split("/"))
    if not self.service_cache.has_key(package_service):
      rospy.loginfo("Loading module to return info on service %s/%s." % package_service)
      srv,mreq,mres = load_service(*package_service)
      self.service_cache[package_service] = (srv._md5sum,mreq._md5sum,mres._md5sum)
    else:
      rospy.loginfo("Returning info from cache on %s/%s." % package_service)
    return self.service_cache[package_service]

  def spin(self):
    rospy.spin()

if __name__=="__main__":
  MessageInfoService().spin()
