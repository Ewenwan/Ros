#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

__author__ = "astambler@willowgarage.com (Adam Stambler)"

from xbee import XBee
import serial

from rosserial_python import SerialClient
import rospy

import threading
import sys
import time
import struct

client_ports=  {}
clients = {}

debug = False;

import threading


class FakeSerial():
	def __init__(self, id, xbee):
		self.rxdata =''
		self.xbee  = xbee
		self.id = id
		self.lock = threading.Lock()
		self.timeout = 0.1
		
	def read(self, size = 1):
		t= 0
		counts = self.timeout/0.01
		#print "s%d   %s"%(size, self.rxdata)
		while( ( len(self.rxdata) < size ) and  (not rospy.is_shutdown()) ):
			time.sleep(0.01)
			t = t +1
			if (t >  counts):
				return ''
			
		with (self.lock):
			out = self.rxdata[:size]
			self.rxdata = self.rxdata[size:]
			
		#print "fake out " , out
		return out
		
	def write(self, data):
		if (debug):
			print "Sending ", [d for d in data]
		self.xbee.send('tx', frame_id='0', options="\x01", dest_addr=self.id,data=data)
		
	def putData(self, data):
		with (self.lock):
			self.rxdata = self.rxdata+data
	
	def flushInput(self):
		self.rxdata = ''

	# Returns the number of bytes available to be read
        def inWaiting(self):
            return len(self.rxdata)

if __name__== '__main__':
	print "RosSerial Xbee Network"
	rospy.init_node('xbee_network')
	sys.argv= rospy.myargv(argv=sys.argv) 
	
	xbee_port = ''
	network_ids = [] 
	
	if len(sys.argv) <3 :
		print """
This program connects to rosserial xbee nodes.  The program must be called
like :

./xbee_network.py <xbee_serial_port> ID1 [ ID2 ID3 ....] 
"""
		exit()
	else :
		xbee_port = sys.argv[1]		
		network_ids  = [ struct.pack('>h', int(id) ) for id in sys.argv[2:] ]
	
	print "Contacting Xbees : " , network_ids

		
	# Open serial port
	ser = serial.Serial(xbee_port, 57600, timeout=0.01)
	ser.flush()
	ser.flushInput()
	ser.flushOutput()
	time.sleep(1)
	# Create API object
	xbee = XBee(ser, escaped= True)
	
	for xid in network_ids:
		client_ports[xid] = FakeSerial(xid, xbee)
		time.sleep(.2)
		clients[xid] = SerialClient(client_ports[xid])
		 
	threads = [ threading.Thread(target=c.run) for c in clients.values()]
	
	for t in threads:
		t.deamon =True 
		t.start()

	while not rospy.is_shutdown():
		try:
			msg = xbee.wait_read_frame()
			if (debug):
				print "Received " , msg
	
			if  msg['id'] == 'rx':
				src = msg['source_addr']
				data = msg['rf_data']
				try:
					client_ports[src].putData(data)
				except KeyError as e:
					print "Rcv ID corrupted"
		except KeyboardInterrupt:
			break
	ser.close()
	
	print "Quiting the Sensor Network"


