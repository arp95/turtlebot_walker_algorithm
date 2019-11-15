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
 *@file       Walker.h
 *@author     Arpit Aggarwal
 *@copyright  MIT License
 *@brief      Header file for Walker class.
 */

#ifndef CATKIN_WS_SRC_TURTLEBOT_WALKER_ALGORITHM_INCLUDE_WALKER_HPP_
#define CATKIN_WS_SRC_TURTLEBOT_WALKER_ALGORITHM_INCLUDE_WALKER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

class Walker {
    public:
      /**
       * @brief      Constructs the Walker object.
       * @param      node  ros::NodeHandle.
      */
      Walker(ros::NodeHandle node);

      /**
       * @brief      Callback for subscriber.
       * @param      data  sensor_msgs::LaserScan::ConstPtr.
      */
      void callback(const sensor_msgs::LaserScan::ConstPtr& data);
    
    private:
      // Variable to store distance.
      double distance; 
};
#endif  // CATKIN_WS_SRC_TURTLEBOT_WALKER_ALGORITHM_INCLUDE_WALKER_HPP_
