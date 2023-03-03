// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "object_stop/drive_object_stop.hpp"

#include <memory>

using namespace std::chrono_literals;

Object_stop::Object_stop()
: Node("drive_object_stop")
{

    scan_data_[0] = 0.0;
    scan_data_[1] = 0.0;
    scan_data_[2] = 0.0;

    robot_pose_ = 0.0;
    prev_robot_pose_ = 0.0;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  //publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  //subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", \
        rclcpp::SensorDataQoS(), \
        std::bind(
        &Object_stop::scan_callback, \
        this, \
        std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&Object_stop::odom_callback, this, std::placeholders::_1));

    update_timer_ = this->create_wall_timer(10ms, std::bind(&Object_stop::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Object_stop::~Object_stop()
{
    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

void Object_stop::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_pose_ = yaw;
}

void Object_stop::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint16_t scan_angle[3] = {0, 30, 330};

    for (int num = 0; num < 3; num++) {
        if (std::isinf(msg->ranges.at(scan_angle[num]))) {
            scan_data_[num] = msg->range_max;
        }
        else {
            scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
    }
}

void Object_stop::update_cmd_vel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);
}


void Object_stop::update_callback()
{
    static uint8_t turtlebot3_state_num = 0;
    //回転角
    double escape_range = 30.0 * DEG2RAD;

    double check_forward_dist = 0.6;
    double check_side_dist = 0.6;

    switch (turtlebot3_state_num) {
        case GET_TB3_DIRECTION:
            if (scan_data_[CENTER] > check_forward_dist) {
                if (scan_data_[LEFT] < check_side_dist) {
                    prev_robot_pose_ = robot_pose_;
                    turtlebot3_state_num = TB3_RIGHT_DRIVE;
                }
                else if (scan_data_[RIGHT] < check_side_dist) {
                    prev_robot_pose_ = robot_pose_;
                    turtlebot3_state_num = TB3_LEFT_DRIVE;
                }
                else {
                    turtlebot3_state_num =TB3_DRIVE_FORWARD;
                }
            }

            if (scan_data_[CENTER] < check_forward_dist) {
                prev_robot_pose_ = robot_pose_;
                turtlebot3_state_num = TB3_DRIVE_FORWARD;
            }
            break;

        case TB3_DRIVE_FORWARD:
            update_cmd_vel(0.3, 0.0);
            turtlebot3_state_num = GET_TB3_DIRECTION;
            break;

        case TB3_RIGHT_DRIVE:
            if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
                turtlebot3_state_num = GET_TB3_DIRECTION;
            }
            else {
                update_cmd_vel(0.0, 0.0);
            }
            break;

        case TB3_LEFT_DRIVE:
            if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
                turtlebot3_state_num = GET_TB3_DIRECTION;
            }
            else {
                update_cmd_vel(0.0, 0.0);
            }
            break;

        default:
            turtlebot3_state_num = GET_TB3_DIRECTION;
            break;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Object_stop>());
    rclcpp::shutdown();

    return 0;
}
