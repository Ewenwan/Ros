/**
 *
 *  \file
 *  \brief      Classes which manage the Publish and Subscribe relationships
 *              that a Session has with its client.
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

#ifndef ROSSERIAL_SERVER_TOPIC_HANDLERS_H
#define ROSSERIAL_SERVER_TOPIC_HANDLERS_H

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/RequestMessageInfo.h>
#include <rosserial_msgs/RequestServiceInfo.h>
#include <topic_tools/shape_shifter.h>

namespace rosserial_server
{

class Publisher {
public:
  Publisher(ros::NodeHandle& nh, const rosserial_msgs::TopicInfo& topic_info) {
    if (!message_service_.isValid()) {
      // lazy-initialize the service caller.
      message_service_ = nh.serviceClient<rosserial_msgs::RequestMessageInfo>("message_info");
      if (!message_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for message_info service to become available.");
      }
    }

    rosserial_msgs::RequestMessageInfo info;
    info.request.type = topic_info.message_type;
    if (message_service_.call(info)) {
      if (info.response.md5 != topic_info.md5sum) {
        ROS_WARN_STREAM("Message" << topic_info.message_type  << "MD5 sum from client does not match that in system. Will avoid using system's message definition.");
        info.response.definition = "";
      }
    } else {
      ROS_WARN("Failed to call message_info service. Proceeding without full message definition.");
    }

    message_.morph(topic_info.md5sum, topic_info.message_type, info.response.definition, "false");
    publisher_ = message_.advertise(nh, topic_info.topic_name, 1);
  }

  void handle(ros::serialization::IStream stream) {
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, message_);
    publisher_.publish(message_);
  }

  std::string get_topic() {
    return publisher_.getTopic();
  }

private:
  ros::Publisher publisher_;
  topic_tools::ShapeShifter message_;

  static ros::ServiceClient message_service_;
};

ros::ServiceClient Publisher::message_service_;
typedef boost::shared_ptr<Publisher> PublisherPtr;


class Subscriber {
public:
  Subscriber(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t>& buffer)> write_fn)
    : write_fn_(write_fn) {
    ros::SubscribeOptions opts;
    opts.init<topic_tools::ShapeShifter>(
        topic_info.topic_name, 1, boost::bind(&Subscriber::handle, this, _1));
    opts.md5sum = topic_info.md5sum;
    opts.datatype = topic_info.message_type;
    subscriber_ = nh.subscribe(opts);
  }

  std::string get_topic() {
    return subscriber_.getTopic();
  }

private:
  void handle(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    size_t length = ros::serialization::serializationLength(*msg);
    std::vector<uint8_t> buffer(length);

    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

    write_fn_(buffer);
  }

  ros::Subscriber subscriber_;
  boost::function<void(std::vector<uint8_t>& buffer)> write_fn_;
};

typedef boost::shared_ptr<Subscriber> SubscriberPtr;

class ServiceClient {
public:
  ServiceClient(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn)
    : write_fn_(write_fn) {
    topic_id_ = -1;
    if (!service_info_service_.isValid()) {
      // lazy-initialize the service caller.
      service_info_service_ = nh.serviceClient<rosserial_msgs::RequestServiceInfo>("service_info");
      if (!service_info_service_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("Timed out waiting for service_info service to become available.");
      }
    }

    rosserial_msgs::RequestServiceInfo info;
    info.request.service = topic_info.message_type;
    ROS_DEBUG("Calling service_info service for topic name %s",topic_info.topic_name.c_str());
    if (service_info_service_.call(info)) {
      request_message_md5_ = info.response.request_md5;
      response_message_md5_ = info.response.response_md5;
    } else {
      ROS_WARN("Failed to call service_info service. The service client will be created with blank md5sum.");
    }
    ros::ServiceClientOptions opts;
    opts.service = topic_info.topic_name;
    opts.md5sum = service_md5_ = info.response.service_md5;
    opts.persistent = false; // always false for now
    service_client_ = nh.serviceClient(opts);
  }
  void setTopicId(uint16_t topic_id) {
    topic_id_ = topic_id;
  }
  std::string getRequestMessageMD5() {
    return request_message_md5_;
  }
  std::string getResponseMessageMD5() {
    return response_message_md5_;
  }

  void handle(ros::serialization::IStream stream) {
    // deserialize request message
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, request_message_);

    // perform service call
    // note that at present, at least for rosserial-windows a service call returns nothing,
    // so we discard the return value of this call() invocation.
    service_client_.call(request_message_, response_message_, service_md5_);

    // write service response over the wire
    size_t length = ros::serialization::serializationLength(response_message_);
    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, response_message_);
    write_fn_(buffer,topic_id_);
  }

private:
  topic_tools::ShapeShifter request_message_;
  topic_tools::ShapeShifter response_message_;
  ros::ServiceClient service_client_;
  static ros::ServiceClient service_info_service_;
  boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn_;
  std::string service_md5_;
  std::string request_message_md5_;
  std::string response_message_md5_;
  uint16_t topic_id_;
};

ros::ServiceClient ServiceClient::service_info_service_;
typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;

}  // namespace

#endif  // ROSSERIAL_SERVER_TOPIC_HANDLERS_H
