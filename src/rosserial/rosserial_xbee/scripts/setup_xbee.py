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


import serial
import yaml
import sys
import time 

from optparse import OptionParser

#		

help = """
%prog [options] port my_adr

setup_xbee.py is a configuration script for Xbees.  It takes  
factory fresh xbee and programs it to work with your rosserial network.
If XBee is not factory fresh, use Digi's X-CTU software to program it.

    port :    serial port of port of the xbee (/dev/ttyUSB0)
    my_adr:   MY address is the 16 bit address of this xbee in the 
              network. This must be a unique address in the network.
              This address is always 0 for the coordinator.  """
parser = OptionParser(usage=help)


parser.add_option('-P', '--pan_id', action="store", type="int", dest="pan_id", default=1331, help="Pan ID of the xbee network.  This ID must be the same for all XBees in your network.")
parser.add_option('-c', '--channel', action="store", type="string", dest="channel", default="0D", help="Frequency channel for the xbee network. The channel value must be the same for all XBees in your network.")
parser.add_option('-C', '--coordinator', action="store_true", dest="coordinator", default=False, help="Configures the XBee as Coordinator for the network.  Only make the XBee connected to the computer a coordiantor.")



def send(port, cmd):
	for c in cmd+'\r':
		port.write(c)
		time.sleep(0.06)
		
def setAT(port, cmd):
	port.flushInput()
	send(port, 'AT'+cmd)
	rsp = port.readline()
	print rsp
	if 'OK' in rsp:
		return True
	else :
		return False

baud_lookup= { 1200   : 0, 
			   2400   : 1,
			   4800   : 2,
			   9600   : 3,
			   19200  : 4,
			   38400  : 5,
			   57600  : 6,
			   115200 : 7}


	
def beginAtMode(port):
	
	for i in range(0,3):
		port.write('+')
		time.sleep(0.05)
	time.sleep(1)
	if port.read(2) == 'OK':
		return True
	else :
		return False

if __name__ == '__main__':
	
	opts, args = parser.parse_args()
	
	if len(args) < 2:
		print "Not enough arguments!"
		exit()
	
	baud = 57600
	port_name = args[0]
	my_address = int(args[1])

	port = serial.Serial(port_name, baud, timeout=1.5)
	
	if beginAtMode(port):
		print "Connected to the XBee"
	else:
		print "Failed to connect to the XBee"
		exit()

	
	cmd = ''
	if (opts.coordinator):
		cmd += 'AP2,CE1,' #API mode 2, and enable coordinator
	
	cmd += 'MY%d,'%int(args[1]) #set the xbee address
	cmd += 'BD%d,'%baud_lookup[57600] #set the xbee to interface at 57600 baud
	cmd += 'ID%d,'%opts.pan_id
	cmd += 'CH%s,'%opts.channel
	cmd += 'DL0,'
	cmd += 'RN1,' #enables collision avoidance on first transmission
	cmd += 'RO5,' #sets packetization timeout to 5 characters
	cmd += 'WR' #wrtie the commands to nonvolatile memory

		
	if setAT(port, 'RE'): #reset the xbee
		print "XBee reset"
	else:
		print "Reset failed"
		exit()
	beginAtMode(port)
	time.sleep(1)
	print "Sending command : ", cmd

	if setAT(port, cmd):
		print "XBee sucessfully programed!"
	else:
		print "XBee programming failed.  Try again and then investigate using X-CTU"

	
		
