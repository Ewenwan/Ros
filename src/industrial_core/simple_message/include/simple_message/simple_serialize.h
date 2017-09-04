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

#ifndef SIMPLE_SERIALIZE_H
#define SIMPLE_SERIALIZE_H

#ifndef FLATHEADERS
#include "simple_message/byte_array.h"
#else
#include "byte_array.h"
#endif

#include "string.h"

namespace industrial
{
namespace simple_serialize
{


  /**
   * \brief Interface for loading and unloading a class to/from a ByteArray
   */
class SimpleSerialize
{
public:
  /**
   * \brief Virtual method for loading an object into a ByteArray
   *
   * This method should load all the required data to reconstruct the class
   * object into the buffer
   *
   * \param buffer pointer to ByteArray
   *
   * \return true on success, false otherwise (buffer not large enough)
   */
  virtual bool load(industrial::byte_array::ByteArray *buffer)=0;

  /**
   * \brief Virtual method for unloading an object from a ByteArray
   *
   * This method should unload all the required data to reconstruct
   * the class object (in place)
   *
   * \param buffer pointer to ByteArray
   *
   * \return true on success, false otherwise (buffer not large enough)
   */
  virtual bool unload(industrial::byte_array::ByteArray *buffer)=0;

  /**
   * \brief Virtual method returns the object size when packed into a
   * ByteArray
   *
   * \return object size (in bytes)
   */
  virtual unsigned int byteLength()=0;

};

} // namespace simple_serialize
} // namespace industrial

#endif //SIMPLE_SERIALIZE_H
