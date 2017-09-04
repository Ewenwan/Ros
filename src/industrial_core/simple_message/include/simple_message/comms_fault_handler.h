/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#ifndef COMMS_FAULT_HANDLER_H
#define COMMS_FAULT_HANDLER_H

namespace industrial
{
namespace comms_fault_handler
{

/**
 * \brief Interface definition for communications fault handler.  Defines the type
 * of communcations faults that can be handled and the function callbacks that should
 * be executed under the specific fault conditions.
 *
 */
class CommsFaultHandler

{
public:

  /**
   * \brief Send failure callback method.  This method will be executed in the event
   * that a comms send fails.
   *
   */
  virtual void sendFailCB()=0;

  /**
   * \brief Receive failure callback method.  This method will be executed in the event
   * that a comms receive fails.
   */
  virtual void receiveFailCB()=0;

  /**
   * \brief Connection failure callback method.  This method will be exectured in the 
   * event that a comms connection is lost.
   *
   */
  virtual void connectionFailCB()=0;

};

} //namespace comms_fault_handler
} //namespace industrial

#endif /* COMMS_FAULT_HANDLER_H */
