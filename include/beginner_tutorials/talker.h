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
 * @file talker.h
 * @author Aswath Muthuselvam
 * @date 08th November 2021
 * @brief Talker file heaader.
 *
 */

#ifndef INCLUDE_BEGINNER_TUTORIALS_TALKER_H_
#define INCLUDE_BEGINNER_TUTORIALS_TALKER_H_

#include "beginner_tutorials/service.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/*
 * @brief Service function that is called with /service, this function changes
 * the message according to the user's input.
 * @param req : Request object reference from service file, it contains the
 * parameters related to the service input.
 * @param res : Response object reference from service file, it contains the
 * parameteres of the service output.
 * @return bool flag indicating success or failure of service method.
 */
bool MyService(beginner_tutorials::service::Request &req,    // NOLINT
               beginner_tutorials::service::Response &res);  // NOLINT

ros::NodeHandle *n;  ///< The node handle of talker node. It contains publisher
                     ///< and service information.

/**
 * Publisher object which publishes the data under the given topic name.
 */
ros::Publisher chatter_pub;

ros::ServiceServer service;  ///< Service object

/**
 * A count of how many messages we have sent. This is used to create
 * a unique string for each message.
 */
int count;

/**
 * This is a message object. You stuff it with data, and then publish it.
 */
std_msgs::String msg;

/**
 * This is a stream object which stores a given sequence of characters.
 */
std::stringstream ss;

#endif  // INCLUDE_BEGINNER_TUTORIALS_TALKER_H_
