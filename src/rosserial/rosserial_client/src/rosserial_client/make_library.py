#!/usr/bin/env python

#####################################################################
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

from __future__ import print_function

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import roslib
import roslib.srvs
import roslib.message
import traceback

import os, sys, re

# for copying files
import shutil

def type_to_var(ty):
    lookup = {
        1 : 'uint8_t',
        2 : 'uint16_t',
        4 : 'uint32_t',
        8 : 'uint64_t',
    }
    return lookup[ty]

#####################################################################
# Data Types

class EnumerationType:
    """ For data values. """

    def __init__(self, name, ty, value):
        self.name = name
        self.type = ty
        self.value = value

    def make_declaration(self, f):
        f.write('      enum { %s = %s };\n' % (self.name, self.value))

class PrimitiveDataType:
    """ Our datatype is a C/C++ primitive. """

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.bytes = bytes

    def make_initializer(self, f, trailer):
        f.write('      %s(0)%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        f.write('      typedef %s _%s_type;\n      _%s_type %s;\n' % (self.type, self.name, self.name, self.name) )

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        if self.type != type_to_var(self.bytes):
            f.write('      union {\n')
            f.write('        %s real;\n' % self.type)
            f.write('        %s base;\n' % type_to_var(self.bytes))
            f.write('      } u_%s;\n' % cn)
            f.write('      u_%s.real = this->%s;\n' % (cn,self.name))
            for i in range(self.bytes):
                f.write('      *(outbuffer + offset + %d) = (u_%s.base >> (8 * %d)) & 0xFF;\n' % (i, cn, i) )
        else:
            for i in range(self.bytes):
                f.write('      *(outbuffer + offset + %d) = (this->%s >> (8 * %d)) & 0xFF;\n' % (i, self.name, i) )
        f.write('      offset += sizeof(this->%s);\n' % self.name)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","").split(".")[-1]
        if self.type != type_to_var(self.bytes):
            f.write('      union {\n')
            f.write('        %s real;\n' % self.type)
            f.write('        %s base;\n' % type_to_var(self.bytes))
            f.write('      } u_%s;\n' % cn)
            f.write('      u_%s.base = 0;\n' % cn)
            for i in range(self.bytes):
                f.write('      u_%s.base |= ((%s) (*(inbuffer + offset + %d))) << (8 * %d);\n' % (cn,type_to_var(self.bytes),i,i) )
            f.write('      this->%s = u_%s.real;\n' % (self.name, cn) )
        else:
            f.write('      this->%s =  ((%s) (*(inbuffer + offset)));\n' % (self.name,self.type) )
            for i in range(self.bytes-1):
                f.write('      this->%s |= ((%s) (*(inbuffer + offset + %d))) << (8 * %d);\n' % (self.name,self.type,i+1,i+1) )
        f.write('      offset += sizeof(this->%s);\n' % self.name)


class MessageDataType(PrimitiveDataType):
    """ For when our data type is another message. """

    def make_initializer(self, f, trailer):
        f.write('      %s()%s\n' % (self.name, trailer))

    def serialize(self, f):
        f.write('      offset += this->%s.serialize(outbuffer + offset);\n' % self.name)

    def deserialize(self, f):
        f.write('      offset += this->%s.deserialize(inbuffer + offset);\n' % self.name)


class AVR_Float64DataType(PrimitiveDataType):
    """ AVR C/C++ has no native 64-bit support, we automatically convert to 32-bit float. """

    def make_initializer(self, f, trailer):
        f.write('      %s(0)%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        f.write('      typedef float _%s_type;\n      _%s_type %s;\n' % (self.name, self.name, self.name) )

    def serialize(self, f):
        f.write('      offset += serializeAvrFloat64(outbuffer + offset, this->%s);\n' % self.name)

    def deserialize(self, f):
        f.write('      offset += deserializeAvrFloat64(inbuffer + offset, &(this->%s));\n' % self.name)


class StringDataType(PrimitiveDataType):
    """ Need to convert to signed char *. """

    def make_initializer(self, f, trailer):
        f.write('      %s("")%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        f.write('      typedef const char* _%s_type;\n      _%s_type %s;\n' % (self.name, self.name, self.name) )

    def serialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      uint32_t length_%s = strlen(this->%s);\n' % (cn,self.name))
        f.write('      varToArr(outbuffer + offset, length_%s);\n' % cn)
        f.write('      offset += 4;\n')
        f.write('      memcpy(outbuffer + offset, this->%s, length_%s);\n' % (self.name,cn))
        f.write('      offset += length_%s;\n' % cn)

    def deserialize(self, f):
        cn = self.name.replace("[","").replace("]","")
        f.write('      uint32_t length_%s;\n' % cn)
        f.write('      arrToVar(length_%s, (inbuffer + offset));\n' % cn)
        f.write('      offset += 4;\n')
        f.write('      for(unsigned int k= offset; k< offset+length_%s; ++k){\n'%cn) #shift for null character
        f.write('          inbuffer[k-1]=inbuffer[k];\n')
        f.write('      }\n')
        f.write('      inbuffer[offset+length_%s-1]=0;\n'%cn)
        f.write('      this->%s = (char *)(inbuffer + offset-1);\n' % self.name)
        f.write('      offset += length_%s;\n' % cn)


class TimeDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes):
        self.name = name
        self.type = ty
        self.sec = PrimitiveDataType(name+'.sec','uint32_t',4)
        self.nsec = PrimitiveDataType(name+'.nsec','uint32_t',4)

    def make_initializer(self, f, trailer):
        f.write('      %s()%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        f.write('      typedef %s _%s_type;\n      _%s_type %s;\n' % (self.type, self.name, self.name, self.name) )

    def serialize(self, f):
        self.sec.serialize(f)
        self.nsec.serialize(f)

    def deserialize(self, f):
        self.sec.deserialize(f)
        self.nsec.deserialize(f)


class ArrayDataType(PrimitiveDataType):

    def __init__(self, name, ty, bytes, cls, array_size=None):
        self.name = name
        self.type = ty
        self.bytes = bytes
        self.size = array_size
        self.cls = cls

    def make_initializer(self, f, trailer):
        if self.size == None:
            f.write('      %s_length(0), %s(NULL)%s\n' % (self.name, self.name, trailer))
        else:
            f.write('      %s()%s\n' % (self.name, trailer))

    def make_declaration(self, f):
        if self.size == None:
            f.write('      uint32_t %s_length;\n' % self.name)
            f.write('      typedef %s _%s_type;\n' % (self.type, self.name))
            f.write('      _%s_type st_%s;\n' % (self.name, self.name)) # static instance for copy
            f.write('      _%s_type * %s;\n' % (self.name, self.name))
        else:
            f.write('      %s %s[%d];\n' % (self.type, self.name, self.size))

    def serialize(self, f):
        c = self.cls(self.name+"[i]", self.type, self.bytes)
        if self.size == None:
            # serialize length
            f.write('      *(outbuffer + offset + 0) = (this->%s_length >> (8 * 0)) & 0xFF;\n' % self.name)
            f.write('      *(outbuffer + offset + 1) = (this->%s_length >> (8 * 1)) & 0xFF;\n' % self.name)
            f.write('      *(outbuffer + offset + 2) = (this->%s_length >> (8 * 2)) & 0xFF;\n' % self.name)
            f.write('      *(outbuffer + offset + 3) = (this->%s_length >> (8 * 3)) & 0xFF;\n' % self.name)
            f.write('      offset += sizeof(this->%s_length);\n' % self.name)
            f.write('      for( uint32_t i = 0; i < %s_length; i++){\n' % self.name)
            c.serialize(f)
            f.write('      }\n')
        else:
            f.write('      for( uint32_t i = 0; i < %d; i++){\n' % (self.size) )
            c.serialize(f)
            f.write('      }\n')

    def deserialize(self, f):
        if self.size == None:
            c = self.cls("st_"+self.name, self.type, self.bytes)
            # deserialize length
            f.write('      uint32_t %s_lengthT = ((uint32_t) (*(inbuffer + offset))); \n' % self.name)
            f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); \n' % self.name)
            f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); \n' % self.name)
            f.write('      %s_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); \n' % self.name)
            f.write('      offset += sizeof(this->%s_length);\n' % self.name)
            f.write('      if(%s_lengthT > %s_length)\n' % (self.name, self.name))
            f.write('        this->%s = (%s*)realloc(this->%s, %s_lengthT * sizeof(%s));\n' % (self.name, self.type, self.name, self.name, self.type))
            f.write('      %s_length = %s_lengthT;\n' % (self.name, self.name))
            # copy to array
            f.write('      for( uint32_t i = 0; i < %s_length; i++){\n' % (self.name) )
            c.deserialize(f)
            f.write('        memcpy( &(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))
            f.write('      }\n')
        else:
            c = self.cls(self.name+"[i]", self.type, self.bytes)
            f.write('      for( uint32_t i = 0; i < %d; i++){\n' % (self.size) )
            c.deserialize(f)
            f.write('      }\n')

#####################################################################
# Messages

class Message:
    """ Parses message definitions into something we can export. """
    global ROS_TO_EMBEDDED_TYPES

    def __init__(self, name, package, definition, md5):

        self.name = name            # name of message/class
        self.package = package      # package we reside in
        self.md5 = md5              # checksum
        self.includes = list()      # other files we must include

        self.data = list()          # data types for code generation
        self.enums = list()

        # parse definition
        for line in definition:
            # prep work
            line = line.strip().rstrip()
            value = None
            if line.find("#") > -1:
                line = line[0:line.find("#")]
            if line.find("=") > -1:
                try:
                    value = line[line.find("=")+1:]
                except:
                    value = '"' + line[line.find("=")+1:] + '"';
                line = line[0:line.find("=")]

            # find package/class name
            line = line.replace("\t", " ")
            l = line.split(" ")
            while "" in l:
                l.remove("")
            if len(l) < 2:
                continue
            ty, name = l[0:2]
            if value != None:
                self.enums.append( EnumerationType(name, ty, value))
                continue

            try:
                type_package, type_name = ty.split("/")
            except:
                type_package = None
                type_name = ty
            type_array = False
            if type_name.find('[') > 0:
                type_array = True
                try:
                    type_array_size = int(type_name[type_name.find('[')+1:type_name.find(']')])
                except:
                    type_array_size = None
                type_name = type_name[0:type_name.find('[')]

            # convert to C type if primitive, expand name otherwise
            try:
                code_type = ROS_TO_EMBEDDED_TYPES[type_name][0]
                size = ROS_TO_EMBEDDED_TYPES[type_name][1]
                cls = ROS_TO_EMBEDDED_TYPES[type_name][2]
                for include in ROS_TO_EMBEDDED_TYPES[type_name][3]:
                    if include not in self.includes:
                        self.includes.append(include)
            except:
                if type_package == None:
                    type_package = self.package
                if type_package+"/"+type_name not in self.includes:
                    self.includes.append(type_package+"/"+type_name)
                cls = MessageDataType
                code_type = type_package + "::" + type_name
                size = 0
            if type_array:
                self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
            else:
                self.data.append( cls(name, code_type, size) )

    def _write_serializer(self, f):
                # serializer
        f.write('    virtual int serialize(unsigned char *outbuffer) const\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.serialize(f)
        f.write('      return offset;\n');
        f.write('    }\n')
        f.write('\n')

    def _write_deserializer(self, f):
        # deserializer
        f.write('    virtual int deserialize(unsigned char *inbuffer)\n')
        f.write('    {\n')
        f.write('      int offset = 0;\n')
        for d in self.data:
            d.deserialize(f)
        f.write('     return offset;\n');
        f.write('    }\n')
        f.write('\n')

    def _write_std_includes(self, f):
        f.write('#include <stdint.h>\n')
        f.write('#include <string.h>\n')
        f.write('#include <stdlib.h>\n')
        f.write('#include "ros/msg.h"\n')

    def _write_msg_includes(self,f):
        for i in self.includes:
            f.write('#include "%s.h"\n' % i)

    def _write_constructor(self, f):
        f.write('    %s()%s\n' % (self.name, ':' if self.data else ''))
        if self.data:
            for d in self.data[:-1]:
                d.make_initializer(f, ',')
            self.data[-1].make_initializer(f, '')
        f.write('    {\n    }\n\n')

    def _write_data(self, f):
        for d in self.data:
            d.make_declaration(f)
        for e in self.enums:
            e.make_declaration(f)
        f.write('\n')

    def _write_getType(self, f):
        f.write('    const char * getType(){ return "%s/%s"; };\n'%(self.package, self.name))

    def _write_getMD5(self, f):
        f.write('    const char * getMD5(){ return "%s"; };\n'%self.md5)

    def _write_impl(self, f):
        f.write('  class %s : public ros::Msg\n' % self.name)
        f.write('  {\n')
        f.write('    public:\n')
        self._write_data(f)
        self._write_constructor(f)
        self._write_serializer(f)
        self._write_deserializer(f)
        self._write_getType(f)
        self._write_getMD5(f)
        f.write('\n')
        f.write('  };\n')

    def make_header(self, f):
        f.write('#ifndef _ROS_%s_%s_h\n'%(self.package, self.name))
        f.write('#define _ROS_%s_%s_h\n'%(self.package, self.name))
        f.write('\n')
        self._write_std_includes(f)
        self._write_msg_includes(f)

        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        self._write_impl(f)
        f.write('\n')
        f.write('}\n')

        f.write('#endif')

class Service:
    def __init__(self, name, package, definition, md5req, md5res):
        """
        @param name -  name of service
        @param package - name of service package
        @param definition - list of lines of  definition
        """

        self.name = name
        self.package = package

        sep_line = len(definition)
        sep = re.compile('---*')
        for i in range(0, len(definition)):
            if (None!= re.match(sep, definition[i]) ):
                sep_line = i
                break
        self.req_def = definition[0:sep_line]
        self.resp_def = definition[sep_line+1:]

        self.req = Message(name+"Request", package, self.req_def, md5req)
        self.resp = Message(name+"Response", package, self.resp_def, md5res)

    def make_header(self, f):
        f.write('#ifndef _ROS_SERVICE_%s_h\n' % self.name)
        f.write('#define _ROS_SERVICE_%s_h\n' % self.name)

        self.req._write_std_includes(f)
        includes = self.req.includes
        includes.extend(self.resp.includes)
        includes = list(set(includes))
        for inc in includes:
            f.write('#include "%s.h"\n' % inc)

        f.write('\n')
        f.write('namespace %s\n' % self.package)
        f.write('{\n')
        f.write('\n')
        f.write('static const char %s[] = "%s/%s";\n'%(self.name.upper(), self.package, self.name))

        def write_type(out, name):
            out.write('    const char * getType(){ return %s; };\n'%(name))
        _write_getType = lambda out: write_type(out, self.name.upper())
        self.req._write_getType = _write_getType
        self.resp._write_getType = _write_getType

        f.write('\n')
        self.req._write_impl(f)
        f.write('\n')
        self.resp._write_impl(f)
        f.write('\n')
        f.write('  class %s {\n' % self.name )
        f.write('    public:\n')
        f.write('    typedef %s Request;\n' % self.req.name )
        f.write('    typedef %s Response;\n' % self.resp.name )
        f.write('  };\n')
        f.write('\n')

        f.write('}\n')

        f.write('#endif\n')


#####################################################################
# Make a Library

def MakeLibrary(package, output_path, rospack):
    pkg_dir = rospack.get_path(package)

    # find the messages in this package
    messages = list()
    if os.path.exists(pkg_dir+"/msg"):
        print('Exporting %s\n'%package)
        sys.stdout.write('  Messages:')
        sys.stdout.write('\n    ')
        for f in os.listdir(pkg_dir+"/msg"):
            if f.endswith(".msg"):
                msg_file = pkg_dir + "/msg/" + f
                # add to list of messages
                print('%s,'%f[0:-4], end='')
                definition = open(msg_file).readlines()
                msg_class = roslib.message.get_message_class(package+'/'+f[0:-4])
                if msg_class:
                    md5sum = msg_class._md5sum
                    messages.append( Message(f[0:-4], package, definition, md5sum) )
                else:
                    err_msg = "Unable to build message: %s/%s\n" % (package, f[0:-4])
                    sys.stderr.write(err_msg)

    # find the services in this package
    if (os.path.exists(pkg_dir+"/srv/")):
        if messages == list():
            print('Exporting %s\n'%package)
        else:
            print('\n')
        sys.stdout.write('  Services:')
        sys.stdout.write('\n    ')
        for f in os.listdir(pkg_dir+"/srv"):
            if f.endswith(".srv"):
                srv_file = pkg_dir + "/srv/" + f
                # add to list of messages
                print('%s,'%f[0:-4], end='')
                definition, service = roslib.srvs.load_from_file(srv_file)
                definition = open(srv_file).readlines()
                srv_class = roslib.message.get_service_class(package+'/'+f[0:-4])
                if srv_class:
                    md5req = srv_class._request_class._md5sum
                    md5res = srv_class._response_class._md5sum
                    messages.append( Service(f[0:-4], package, definition, md5req, md5res ) )
                else:
                    err_msg = "Unable to build service: %s/%s\n" % (package, f[0:-4])
                    sys.stderr.write(err_msg)
        print('\n')
    elif messages != list():
        print('\n')

    # generate for each message
    output_path = output_path + "/" + package
    for msg in messages:
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        header = open(output_path + "/" + msg.name + ".h", "w")
        msg.make_header(header)
        header.close()

def rosserial_generate(rospack, path, mapping):
    # horrible hack -- make this die
    global ROS_TO_EMBEDDED_TYPES
    ROS_TO_EMBEDDED_TYPES = mapping

    # gimme messages
    failed = []
    for p in sorted(rospack.list()):
        try:
            MakeLibrary(p, path, rospack)
        except Exception as e:
            failed.append(p + " ("+str(e)+")")
            print('[%s]: Unable to build messages: %s\n' % (p, str(e)))
            print(traceback.format_exc())
    print('\n')
    if len(failed) > 0:
        print('*** Warning, failed to generate libraries for the following packages: ***')
        for f in failed:
            print('    %s'%f)
        raise Exception("Failed to generate libraries for: " + str(failed))
    print('\n')

def rosserial_client_copy_files(rospack, path):
    os.makedirs(path+"/ros")
    os.makedirs(path+"/tf")
    files = ['duration.cpp',
             'time.cpp',
             'ros/duration.h',
             'ros/msg.h',
             'ros/node_handle.h',
             'ros/publisher.h',
             'ros/service_client.h',
             'ros/service_server.h',
             'ros/subscriber.h',
             'ros/time.h',
             'tf/tf.h',
             'tf/transform_broadcaster.h']
    mydir = rospack.get_path("rosserial_client")
    for f in files:
        shutil.copy(mydir+"/src/ros_lib/"+f, path+f)

