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
 * @file pubsub_test.cpp
 * @author Aswath Muthuselvam
 * @date 15th November 2021
 * @brief Test node
 *
 */

// Include required headers
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "beginner_tutorials/service.h"
#include "std_msgs/String.h"

std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief Test for checking if service client has started.
 */
TEST(TestStub, TestPublisherExists) {
  ros::ServiceClient service_client =
      nh->serviceClient<beginner_tutorials::service>("service");
  bool exists(service_client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Test to check if service successfully changed the publisher message.
 */
TEST(TestStub, TestMessageChangeService) {
  ros::ServiceClient service_client =
      nh->serviceClient<beginner_tutorials::service>("service");
  beginner_tutorials::service::Request req;
  beginner_tutorials::service::Response res;
  req.service_input = "Changing service message by ROS";
  service_client.call(req, res);
  EXPECT_EQ("Successfully changed message", res.service_output);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker_node_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
