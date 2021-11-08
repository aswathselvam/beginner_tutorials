// Copyright 2021 Aswath Muthuselvam

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
