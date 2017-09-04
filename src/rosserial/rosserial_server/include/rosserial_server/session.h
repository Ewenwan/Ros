/**
 *
 *  \file
 *  \brief      Class representing a session between this node and a
 *              templated rosserial client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_SESSION_H
#define ROSSERIAL_SERVER_SESSION_H

#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/Log.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Time.h>

#include "rosserial_server/async_read_buffer.h"
#include "rosserial_server/topic_handlers.h"

namespace rosserial_server
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;

template<typename Socket>
class Session : boost::noncopyable
{
public:
  Session(boost::asio::io_service& io_service)
    : socket_(io_service),
      sync_timer_(io_service),
      require_check_timer_(io_service),
      ros_spin_timer_(io_service),
      async_read_buffer_(socket_, buffer_max,
                         boost::bind(&Session::read_failed, this,
                                     boost::asio::placeholders::error))
  {
    active_ = false;

    timeout_interval_ = boost::posix_time::milliseconds(5000);
    attempt_interval_ = boost::posix_time::milliseconds(1000);
    require_check_interval_ = boost::posix_time::milliseconds(1000);
    ros_spin_interval_ = boost::posix_time::milliseconds(10);
    require_param_name_ = "~require";

    nh_.setCallbackQueue(&ros_callback_queue_);

    // Intermittent callback to service ROS callbacks. To avoid polling like this,
    // CallbackQueue could in the future be extended with a scheme to monitor for
    // callbacks on another thread, and then queue them up to be executed on this one.
    ros_spin_timer_.expires_from_now(ros_spin_interval_);
    ros_spin_timer_.async_wait(boost::bind(&Session::ros_spin_timeout, this,
                                           boost::asio::placeholders::error));
  }

  Socket& socket()
  {
    return socket_;
  }

  void start()
  {
    ROS_DEBUG("Starting session.");

    callbacks_[rosserial_msgs::TopicInfo::ID_PUBLISHER]
        = boost::bind(&Session::setup_publisher, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_SUBSCRIBER]
        = boost::bind(&Session::setup_subscriber, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT+rosserial_msgs::TopicInfo::ID_PUBLISHER]
        = boost::bind(&Session::setup_service_client_publisher, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT+rosserial_msgs::TopicInfo::ID_SUBSCRIBER]
        = boost::bind(&Session::setup_service_client_subscriber, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_LOG]
        = boost::bind(&Session::handle_log, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_TIME]
        = boost::bind(&Session::handle_time, this, _1);

    active_ = true;
    attempt_sync();
    read_sync_header();
  }

  void stop()
  {
    // Abort any pending ROS callbacks.
    ros_callback_queue_.clear();

    // Abort active session timer callbacks, if present.
    sync_timer_.cancel();
    require_check_timer_.cancel();

    // Reset the state of the session, dropping any publishers or subscribers
    // we currently know about from this client.
    callbacks_.clear();
    subscribers_.clear();
    publishers_.clear();
    services_.clear();

    // Close the socket.
    socket_.close();
    active_ = false;
  }

  bool is_active()
  {
    return active_;
  }

  /**
   * This is to set the name of the required topics parameter from the
   * default of ~require. You might want to do this to avoid a conflict
   * with something else in that namespace, or because you're embedding
   * multiple instances of rosserial_server in a single process.
   */
  void set_require_param(std::string param_name)
  {
    require_param_name_ = param_name;
  }

private:
  /**
   * Periodic function which handles calling ROS callbacks, executed on the same
   * io_service thread to avoid a concurrency nightmare.
   */
  void ros_spin_timeout(const boost::system::error_code& error) {
    ros_callback_queue_.callAvailable();

    if (ros::ok())
    {
      // Call again next interval.
      ros_spin_timer_.expires_from_now(ros_spin_interval_);
      ros_spin_timer_.async_wait(boost::bind(&Session::ros_spin_timeout, this,
                                             boost::asio::placeholders::error));
    }
  }

  //// RECEIVING MESSAGES ////
  // TODO: Total message timeout, implement primarily in ReadBuffer.

  void read_sync_header() {
    async_read_buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  void read_sync_first(ros::serialization::IStream& stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xff) {
      async_read_buffer_.read(1, boost::bind(&Session::read_sync_second, this, _1));
    } else {
      read_sync_header();
    }
  }

  void read_sync_second(ros::serialization::IStream& stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xfe) {
      async_read_buffer_.read(5, boost::bind(&Session::read_id_length, this, _1));
    } else {
      read_sync_header();
    }
  }

  void read_id_length(ros::serialization::IStream& stream) {
    uint16_t topic_id, length;
    uint8_t length_checksum;

    // Check header checksum byte for length field.
    stream >> length >> length_checksum;
    if (length_checksum + checksum(length) != 0xff) {
      uint8_t csl = checksum(length);
      ROS_WARN("Bad message header length checksum. Dropping message from client. T%d L%d C%d %d", topic_id, length, length_checksum, csl);
      read_sync_header();
      return;
    } else {
      stream >> topic_id;
    }
    ROS_DEBUG("Received message header with length %d and topic_id=%d", length, topic_id);

    // Read message length + checksum byte.
    async_read_buffer_.read(length + 1, boost::bind(&Session::read_body, this,
                                                    _1, topic_id));
  }

  void read_body(ros::serialization::IStream& stream, uint16_t topic_id) {
    ROS_DEBUG("Received body of length %d for message on topic %d.", stream.getLength(), topic_id);

    ros::serialization::IStream checksum_stream(stream.getData(), stream.getLength());
    uint8_t msg_checksum = checksum(checksum_stream) + checksum(topic_id);

    if (msg_checksum != 0xff) {
      ROS_WARN("Rejecting message on topicId=%d, length=%d with bad checksum.", topic_id, stream.getLength());
    } else {
      if (callbacks_.count(topic_id) == 1) {
        try {
          callbacks_[topic_id](stream);
        } catch(ros::serialization::StreamOverrunException e) {
          if (topic_id < 100) {
            ROS_ERROR("Buffer overrun when attempting to parse setup message.");
            ROS_ERROR_ONCE("Is this firmware from a pre-Groovy rosserial?");
          } else {
            ROS_WARN("Buffer overrun when attempting to parse user message.");
          }
        }
      } else {
        ROS_WARN("Received message with unrecognized topicId (%d).", topic_id);
        // TODO: Resynchronize on multiples?
      }
    }

    // Kickoff next message read.
    read_sync_header();
  }

  void read_failed(const boost::system::error_code& error) {
    if (error == boost::system::errc::no_buffer_space) {
      // No worries. Begin syncing on a new message.
      ROS_WARN("Overrun on receive buffer. Attempting to regain rx sync.");
      read_sync_header();
    } else if (error) {
      // When some other read error has occurred, stop the session, which destroys
      // all known publishers and subscribers.
      ROS_WARN_STREAM("Socket asio error, closing socket: " << error);
      stop();
    }
  }

  //// SENDING MESSAGES ////

  void write_message(Buffer& message, const uint16_t topic_id) {
    uint8_t overhead_bytes = 8;
    uint16_t length = overhead_bytes + message.size();
    BufferPtr buffer_ptr(new Buffer(length));

    uint8_t msg_checksum;
    ros::serialization::IStream checksum_stream(message.size() > 0 ? &message[0] : NULL, message.size());

    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size());
    uint8_t msg_len_checksum = 255 - checksum(message.size());
    stream << (uint16_t)0xfeff << (uint16_t)message.size() << msg_len_checksum << topic_id;
    msg_checksum = 255 - (checksum(checksum_stream) + checksum(topic_id));

    memcpy(stream.advance(message.size()), &message[0], message.size());
    stream << msg_checksum;

    ROS_DEBUG_NAMED("async_write", "Sending buffer of %d bytes to client.", length);
    boost::asio::async_write(socket_, boost::asio::buffer(*buffer_ptr),
          boost::bind(&Session::write_completion_cb, this, boost::asio::placeholders::error, buffer_ptr));
  }

  void write_completion_cb(const boost::system::error_code& error,
                           BufferPtr buffer_ptr) {
    if (error) {
      if (error == boost::system::errc::io_error) {
        ROS_WARN_THROTTLE(1, "Socket write operation returned IO error.");
      } else if (error == boost::system::errc::no_such_device) {
        ROS_WARN_THROTTLE(1, "Socket write operation returned no device.");
      } else {
        ROS_WARN_STREAM_THROTTLE(1, "Unknown error returned during write operation: " << error);
      }
      stop();
    }
    // Buffer is destructed when this function exits and buffer_ptr goes out of scope.
  }

  //// SYNC WATCHDOG ////
  void attempt_sync() {
    request_topics();
    set_sync_timeout(attempt_interval_);
  }

  void set_sync_timeout(const boost::posix_time::time_duration& interval) {
    if (ros::ok())
    {
      sync_timer_.cancel();
      sync_timer_.expires_from_now(interval);
      sync_timer_.async_wait(boost::bind(&Session::sync_timeout, this,
            boost::asio::placeholders::error));
    }
  }

  void sync_timeout(const boost::system::error_code& error) {
    if (error != boost::asio::error::operation_aborted) {
      ROS_DEBUG("Sync with device lost.");
      stop();
    }
  }

  //// HELPERS ////
  void request_topics() {
    std::vector<uint8_t> message(0);
    ROS_DEBUG("Sending request topics message for VER2 protocol.");
    write_message(message, rosserial_msgs::TopicInfo::ID_PUBLISHER);

    // Set timer for future point at which to verify the subscribers and publishers
    // created by the client against the expected set given in the parameters.
    require_check_timer_.expires_from_now(require_check_interval_);
    require_check_timer_.async_wait(boost::bind(&Session::required_topics_check, this,
          boost::asio::placeholders::error));
  }

  void required_topics_check(const boost::system::error_code& error) {
    if (error != boost::asio::error::operation_aborted) {
      if (ros::param::has(require_param_name_)) {
        if (!check_set(require_param_name_ + "/publishers", publishers_) ||
            !check_set(require_param_name_ + "/subscribers", subscribers_)) {
          ROS_WARN("Connected client failed to establish the publishers and subscribers dictated by require parameter. Re-requesting topics.");
          request_topics();
        }
      }
    }
  }

  template<typename M>
  bool check_set(std::string param_name, M map) {
    XmlRpc::XmlRpcValue param_list;
    ros::param::get(param_name, param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < param_list.size(); ++i) {
      ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string required_topic((std::string(param_list[i])));
      // Iterate through map of registered topics, to ensure that this one is present.
      bool found = false;
      for (typename M::iterator j = map.begin(); j != map.end(); ++j) {
        if (nh_.resolveName(j->second->get_topic()) ==
            nh_.resolveName(required_topic)) {
          found = true;
          ROS_INFO_STREAM("Verified connection to topic " << required_topic << ", given in parameter " << param_name);
          break;
        }
      }
      if (!found) {
        ROS_WARN_STREAM("Missing connection to topic " << required_topic << ", required by parameter " << param_name);
        return false;
      }
    }
    return true;
  }

  static uint8_t checksum(ros::serialization::IStream& stream) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < stream.getLength(); ++i) {
      sum += stream.getData()[i];
    }
    return sum;
  }

  static uint8_t checksum(uint16_t val) {
    return (val >> 8) + val;
  }

  //// RECEIVED MESSAGE HANDLERS ////

  void setup_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    PublisherPtr pub(new Publisher(nh_, topic_info));
    callbacks_[topic_info.topic_id] = boost::bind(&Publisher::handle, pub, _1);
    publishers_[topic_info.topic_id] = pub;

    set_sync_timeout(timeout_interval_);
  }

  void setup_subscriber(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    SubscriberPtr sub(new Subscriber(nh_, topic_info,
        boost::bind(&Session::write_message, this, _1, topic_info.topic_id)));
    subscribers_[topic_info.topic_id] = sub;

    set_sync_timeout(timeout_interval_);
  }

  // When the rosserial client creates a ServiceClient object (and/or when it registers that object with the NodeHandle)
  // it creates a publisher (to publish the service request message to us) and a subscriber (to receive the response)
  // the service client callback is attached to the *subscriber*, so when we receive the service response
  // and wish to send it over the socket to the client,
  // we must attach the topicId that came from the service client subscriber message

  void setup_service_client_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, _2)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
    if (services_[topic_info.topic_name]->getRequestMessageMD5() != topic_info.md5sum) {
      ROS_WARN("Service client setup: Request message MD5 mismatch between rosserial client and ROS");
    } else {
      ROS_DEBUG("Service client %s: request message MD5 successfully validated as %s",
        topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
    set_sync_timeout(timeout_interval_);
  }

  void setup_service_client_subscriber(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);

    if (!services_.count(topic_info.topic_name)) {
      ROS_DEBUG("Creating service client for topic %s",topic_info.topic_name.c_str());
      ServiceClientPtr srv(new ServiceClient(
        nh_,topic_info,boost::bind(&Session::write_message, this, _1, _2)));
      services_[topic_info.topic_name] = srv;
      callbacks_[topic_info.topic_id] = boost::bind(&ServiceClient::handle, srv, _1);
    }
    // see above comment regarding the service client callback for why we set topic_id here
    services_[topic_info.topic_name]->setTopicId(topic_info.topic_id);
    if (services_[topic_info.topic_name]->getResponseMessageMD5() != topic_info.md5sum) {
      ROS_WARN("Service client setup: Response message MD5 mismatch between rosserial client and ROS");
    } else {
      ROS_DEBUG("Service client %s: response message MD5 successfully validated as %s",
        topic_info.topic_name.c_str(),topic_info.md5sum.c_str());
    }
    set_sync_timeout(timeout_interval_);
  }

  void handle_log(ros::serialization::IStream& stream) {
    rosserial_msgs::Log l;
    ros::serialization::Serializer<rosserial_msgs::Log>::read(stream, l);
    if(l.level == rosserial_msgs::Log::ROSDEBUG) ROS_DEBUG("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::INFO) ROS_INFO("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::WARN) ROS_WARN("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::ERROR) ROS_ERROR("%s", l.msg.c_str());
    else if(l.level == rosserial_msgs::Log::FATAL) ROS_FATAL("%s", l.msg.c_str());
  }

  void handle_time(ros::serialization::IStream& stream) {
    std_msgs::Time time;
    time.data = ros::Time::now();

    size_t length = ros::serialization::serializationLength(time);
    std::vector<uint8_t> message(length);

    ros::serialization::OStream ostream(&message[0], length);
    ros::serialization::Serializer<std_msgs::Time>::write(ostream, time);

    write_message(message, rosserial_msgs::TopicInfo::ID_TIME);

    // The MCU requesting the time from the server is the sync notification. This
    // call moves the timeout forward.
    set_sync_timeout(timeout_interval_);
  }

  Socket socket_;
  AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 1023 };
  bool active_;

  ros::NodeHandle nh_;
  ros::CallbackQueue ros_callback_queue_;

  boost::posix_time::time_duration timeout_interval_;
  boost::posix_time::time_duration attempt_interval_;
  boost::posix_time::time_duration require_check_interval_;
  boost::posix_time::time_duration ros_spin_interval_;
  boost::asio::deadline_timer sync_timer_;
  boost::asio::deadline_timer require_check_timer_;
  boost::asio::deadline_timer ros_spin_timer_;
  std::string require_param_name_;

  std::map<uint16_t, boost::function<void(ros::serialization::IStream&)> > callbacks_;
  std::map<uint16_t, PublisherPtr> publishers_;
  std::map<uint16_t, SubscriberPtr> subscribers_;
  std::map<std::string, ServiceClientPtr> services_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_SESSION_H
