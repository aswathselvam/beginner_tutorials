// Copyright 2021 Aswath Muthuselvam

#ifndef INCLUDE_BEGINNER_TUTORIALS_TALKER_H_
#define INCLUDE_BEGINNER_TUTORIALS_TALKER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * Publisher object which publishes the data under the given topic name.
 */ 
ros::Publisher chatter_pub;

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
