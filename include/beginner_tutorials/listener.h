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
 * @file listener.h
 * @author Aswath Muthuselvam
 * @date 08th November 2021
 * @brief Subscriber node header.
 *
 */

#ifndef INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_
#define INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_

#include <std_srvs/Empty.h>

#include "ros/ros.h"

/*
 * @brief Service function that is called with /service, this function changes
 * the message according to the user's input.
 * @return bool flag indicating success or failure of service method.
 */
bool KillService(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

ros::NodeHandle* n;  ///< Instance of Node Handler object

bool stop;  ///< Checks if kill signal is received from service.

ros::ServiceServer service;  ///< Service object

/**
 * This is a subscriber object, to receive the message published under a given
 * topic.
 */
ros::Subscriber sub;

#endif  // INCLUDE_BEGINNER_TUTORIALS_LISTENER_H_
