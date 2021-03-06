/**
 * MIT License
 *
 * Copyright (c) 2021 Aswath Muthuselvam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file talker.cpp
 * @author Aswath Muthuselvam
 * @date 08th November 2021
 * @brief Talker file, publishes string.
 *
 */

#include "beginner_tutorials/talker.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <string>

#include "beginner_tutorials/service.h"

bool MyService(beginner_tutorials::service::Request &req,     // NOLINT
               beginner_tutorials::service::Response &res) {  // NOLINT
  ROS_INFO_STREAM("Service /service is called");
  if (req.service_input.empty()) {
    ROS_ERROR_STREAM("Input string from ROS service is empty!");
    return false;
  } else {
    ROS_DEBUG_STREAM("Received message: " << req.service_input);
    ROS_WARN_STREAM("Publisher message will be changed.");
    msg.data = req.service_input;
    res.service_output = "Successfully changed message";
    return true;
  }
}

void BroadcastTF() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.1, 0.2, 0.0));
  tf::Quaternion q;
  q.setRPY(0.1, 0.1, 0.1);
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  msg.data = " Hello Terps! ";

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line. For programmatic remappings you can use a different version of init()
   * which takes remappings directly, but for most command-line programs,
   * passing argc and argv is the easiest way to do it.  The third argument to
   * init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last NodeHandle destructed will close down the node.
   */
  n = new ros::NodeHandle();

  n->getParam("message", msg.data);

  ros::Rate loop_rate(10);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  chatter_pub = n->advertise<std_msgs::String>("chatter", 1000);

  service = n->advertiseService("service", &MyService);

  count = 0;
  while (ros::ok()) {
    ss << msg.data << count;

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    BroadcastTF();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  delete n;

  return 0;
}
