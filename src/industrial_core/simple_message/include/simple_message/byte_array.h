/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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

#ifndef BYTE_ARRAY_H
#define BYTE_ARRAY_H

#ifndef FLATHEADERS
#include "simple_message/shared_types.h"
#else
#include "shared_types.h"
#endif

#include <deque>
#include <vector>
#include "string.h"

namespace industrial
{
namespace simple_serialize
{
// Class declaration required for function prototypes below
class SimpleSerialize;
}
}

namespace industrial
{
namespace byte_array
{

/**
 * \brief The byte array wraps a dynamic array of bytes (i.e. char).
 *
 * It  provides convenient methods for loading and unloading
 * data types to and from the byte array.  The class acts as an
 * interface definition to raw data (in case the underlying structure of the
 * raw data changes).  It's intended use is for socket communications.
 *
 * By default data using the load/unload methods is appended/removed from the end
 * of the array.  Methods are also provided to load/unload from the front of the
 * array.  As long as the matching load/unload methods are used, this is
 * transparent to the user.
 *
 * The internals of ByteArray have been updated to use a dynamically-allocated
 * STL class, for safety and performance reasons.  This may limit cross-platform
 * usage of this class (e.g. in the MotoPlus compiler).
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class ByteArray
{
public:

  // Provides SimpleSerialize access to byte array internals
  friend class SimpleSerialize;

  /**
   * \brief Default constructor
   *
   * This method creates and empty byte array.
   *
   */
  ByteArray(void);

  /**
   * \brief Destructor
   *
   */
  ~ByteArray(void);

  /**
   * \brief Initializes or Reinitializes an empty buffer.
   *
   * This method resets the buffer size to zero (empty).
   *
   */
  void init();

  /**
   * \brief initializes byte array from char* buffer
   *
   * This method creates a byte array containing a copy of the passed
   * in buffer (up to byteSize)
   *
   * \param buffer pointer to byte buffer
   * \param byte_size size of buffer to copy.
   *
   * \return true on success, false otherwise (max array size exceeded).
   */
  bool init(const char* buffer, const industrial::shared_types::shared_int byte_size);

  /**
   * \brief Deep-Copy
   *
   * This method creates a byte array containing a deep copy of the
   * passed in buffer
   *
   * \param buffer buffer to copy
   *
   */
  void copyFrom(ByteArray & buffer);

  /**
   * \brief Copy to std::vector, for raw-ptr access
   *
   * This method copies the ByteArray data to a std::vector.
   *
   * \param out vector to copy into
   *
   */
  void copyTo(std::vector<char> & out);

  /**
   * \brief loads a boolean into the byte array
   *
   * \param value to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(industrial::shared_types::shared_bool value);

  /**
   * \brief loads a float on the byte array.  If byte swapping is
   * enabled, then the bytes are swapped (this assumes a common float
   * representation)
   *
   * \param value to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(industrial::shared_types::shared_real value);

  /**
   * \brief loads an integer into the byte array.  If byte swapping is
   * enabled, then the bytes are swapped.
   *
   * \param value to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(industrial::shared_types::shared_int value);

  /**
   * \brief loads a complex SimpleSerialize into the byte array
   *
   * \param value to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(industrial::simple_serialize::SimpleSerialize &value);

  /**
   * \brief loads a whole byte array into this byte array
   *
   * \param value to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(ByteArray &value);

  /**
   * \brief loads a void* (treated as char*) into the byte array.
   * WARNING: Byte swapping is not performed in this function.
   *
   * \param value to load
   * \byte_syze number of bytes to load
   *
   * \return true on success, false otherwise (max array size exceeded).
   * Value not loaded
   */
  bool load(void* value, const industrial::shared_types::shared_int byte_size);

  /**
   * \brief unloads a boolean value from the byte array
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(industrial::shared_types::shared_bool &value);

  /**
   * \brief unloads a double value from the byte array. If byte swapping is
   * enabled, then the bytes are swapped.
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(industrial::shared_types::shared_real &value);

  /**
   * \brief unloads an integer value from the byte array.  If byte swapping is
   * enabled, then the bytes are swapped.
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(industrial::shared_types::shared_int &value);

  /**
   * \brief unloads a complex SimpleSerialize value from the byte array
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(industrial::simple_serialize::SimpleSerialize &value);

  /**
   * \brief unloads a partial byte array from the byte array into the
   * passed in byte array (this is done using the byte array load method
   * so any data in the passed in byte array remains intact)
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(ByteArray &value, const industrial::shared_types::shared_int byte_size);

  /**
   * \brief unloads a void* (treated as char*) from the byte array.
   * WARNING: Byte swapping is not performed in this function.
   *
   * \param value to unload
   * \byte_syze number of bytes to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unload(void* value, const industrial::shared_types::shared_int byteSize);

  /**
   * \brief unloads a double value from the beginning of the byte array.
   * If byte swapping is enabled, then the bytes are swapped.
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unloadFront(industrial::shared_types::shared_real &value);

  /**
   * \brief unloads an integer value from the beginning of the byte array.
   * If byte swapping is enabled, then the bytes are swapped
   *
   * \param value value to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unloadFront(industrial::shared_types::shared_int &value);

  /**
   * \brief unloads a void* (treated as char*) from the beginning of the array.
   * WARNING: Byte swapping is not performed in this function.
   *
   * \param value to unload
   * \byte_syze number of bytes to unload
   *
   * \return true on success, false otherwise (array is empty)
   */
  bool unloadFront(void* value, const industrial::shared_types::shared_int byteSize);

  /**
   * \brief returns a char* pointer to the raw data.
   * WARNING: This method is meant for read-only operations
   *
   * \deprecated This is unsafe with dynamic buffer sizing.
   * Use copyTo(vector<char>) instead.
   *
   * \return char* pointer to the raw data
   */
   __attribute__((deprecated("This ptr will be invalid once buffer is changed.  Please use: copyTo(vector<char>) instead.")))
  char* getRawDataPtr();

  /**
   * \brief gets current buffer size
   *
   * \return buffer size
   */
  unsigned int getBufferSize();

  /**
   * \brief gets current buffer size
   *
   * \return buffer size
   */
  unsigned int getMaxBufferSize();

  /**
     * \brief returns true if byte swapping is enabled (this is a global
     * option set by compiler flag).  This function gives the status of the
     * compiler flag.
     *
     * \return true if byte swapping is enabled.
     */
  static bool isByteSwapEnabled();

private:

  /**
   * \brief internal data buffer
   */
  std::deque<char> buffer_;
  
  /**
   * \brief temporary continuous buffer for getRawDataPtr() use
   */
   std::vector<char> getRawDataPtr_buffer_;

#ifdef BYTE_SWAPPING
  /**
   * \brief Swaps byte of value (in place)
   *
   * \param value to swap
   * \param byteSize (in bytes)
   *
   */
  void swap(void *value, industrial::shared_types::shared_int byteSize);
#endif

};

} // namespace industrial
} // namespace byte_array

#endif //BYTE_ARRAY_H
