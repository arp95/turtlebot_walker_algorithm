/**
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file       Walker.cpp
 *@author     Arpit Aggarwal
 *@copyright  MIT License
 *@brief      Describes Walker class functions.
 */

#include "Walker.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

void Walker::callback(const sensor_msgs::LaserScan::ConstPtr& data) {
    double minDistance = 0.2;
    for (int i = 0; i < data->ranges.size(); i++) {
        if (data->ranges[i] > minDistance) {
            minDistance = data->ranges[i];
        }
    }
    distance = minDistance;
}

Walker::Walker(ros::NodeHandle node) {
    // ROS LaserScan Subscriber
    ros::Subscriber laserSubscriber =
node.subscribe("/scan", 1000, &Walker::callback, this);

    // ROS Publisher
    ros::Publisher velocityPublisher =
node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    ros::Rate loopRate(4);
    while (ros::ok()) {
        // Declare and initialize twist
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        if (distance > 0.77) {
            ROS_INFO_STREAM("Moving Forward!");
            twist.linear.x = 0.12;
        } else {
            ROS_INFO_STREAM("Rotating!");
            twist.angular.z = 1.7;
        }
        velocityPublisher.publish(twist);
        ros::spinOnce();
        loopRate.sleep();
    }
}
