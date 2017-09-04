/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*       * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Southwest Research Institute, nor the names
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LOG_WRAPPER_H_
#define LOG_WRAPPER_H_

#ifdef ROS
#include "ros/ros.h"
#endif

#ifdef MOTOPLUS
#include "motoPlus.h"
#endif

namespace industrial
{

/**
 * \brief Contains macro that wrap standard logging calls.  Wrapping logging
 * calls allows libraries to be used inside and outside the ROS framework.
 *
 * Macros are used because passing variable argument lists are much easier
 * than passing them through functions.
 */
namespace log_wrapper
{
    

// Define ROS if this library will execute under ROS
#ifdef ROS

// The LOG_COMM redirects to debug in ROS because ROS has
// debug filtering tools that allow the communications messages
// to be easily removed from the logs
#define LOG_COMM(format, ...)  \
  ROS_DEBUG(format, ##__VA_ARGS__)
  
#define LOG_DEBUG(format, ...)  \
  ROS_DEBUG(format, ##__VA_ARGS__)

#define LOG_INFO(format, ...)  \
  ROS_INFO(format, ##__VA_ARGS__)

#define LOG_WARN(format, ...)  \
  ROS_WARN(format, ##__VA_ARGS__)

#define LOG_ERROR(format, ...)  \
  ROS_ERROR(format, ##__VA_ARGS__)

#define LOG_FATAL(format, ...)  \
  ROS_FATAL(FATAL, ##__VA_ARGS__)

#elif defined(STDIOLOG)

#define LOG(level, format, ...) \
do \
{ \
  printf(level); \
  printf(": "); \
  printf(format, ##__VA_ARGS__); \
  printf("\n"); \
  } while (0)

// WARNING: LOG_COMM produces many messages and could slow down program
// execution on the robot.
#define LOG_COMM(format, ...)  LOG("COMM", format, ##__VA_ARGS__) 
#define LOG_DEBUG(format, ...) LOG("DEBUG", format, ##__VA_ARGS__) 
#define LOG_INFO(format, ...)  LOG("INFO", format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)  LOG("WARNING", format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) LOG("ERROR", format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) LOG("FATAL", format, ##__VA_ARGS__)

#else
// LOG DISABLED

#define LOG_COMM(format, ...)  
#define LOG_DEBUG(format, ...) 
#define LOG_INFO(format, ...)  
#define LOG_WARN(format, ...) 
#define LOG_ERROR(format, ...) 
#define LOG_FATAL(format, ...) 

#endif 



} // namespace industrial
} // namespace loge_wrapper

#endif /* LOG_WRAPPER_H_ */
