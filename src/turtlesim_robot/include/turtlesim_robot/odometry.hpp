#ifndef _TURTLESIM_ROBOT_ODOM_
#define _TURTLESIM_ROBOT_ODOM_

#include<memory>
#include"rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<random>
#include<string>
#include<iostream>


namespace turtlesim_robot
{
    class Odometry : public rclcpp::Node{

        public: 
        Odometry() : Node("_turtlesim_robot_odometry_node_") {
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(pub_topic_, 10);
            
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(sub_topic_,
                10, std::bind(&Odometry::poseCallback, this, std::placeholders::_1));

            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 
                10, std::bind(&Odometry::cmdVelCallback,this, std::placeholders::_1));
            
            // random_distribution_vx_{error_vx_systematic_, error_vx_random_};
            random_distribution_vx_ = std::normal_distribution<double>(error_vx_systematic_, error_vx_random_);

            // random_distribution_wz_{error_wz_systematic_, error_wz_random_};
            random_distribution_wz_ = std::normal_distribution<double>(error_wz_systematic_, error_wz_random_);
        }

        void poseCallback(turtlesim::msg::Pose::SharedPtr pose); 

        void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg); 

        private:
        std::string sub_topic_ = "/turtle1/pose"; 
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

        std::string pub_topic_ = "/turtle1/twist_topic";
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_; 

        std::string cmd_vel_topic_ = "/turtle1/cmd_vel";
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        double error_vx_systematic_ = 0.0; 
        double error_vx_random_ = 0.05; // 0.05;
        double error_wz_systematic_ = 0.0;
        double error_wz_random_ =  0.0349; //0.0349;
        std::default_random_engine random_generator_; 
        std::normal_distribution<double> random_distribution_vx_; 
        std::normal_distribution<double> random_distribution_wz_;

        double cmd_vel_vx_ = 0.; 
        double cmd_vel_wz_ = 0.; 
        rclcpp::Time cmd_vel_time_stampe_ ; 
        bool new_vel_msg_ = false; 

        // bool first_msg_ = true; 
        double old_pose_x_ = 0.; 
        double old_pose_y_ = 0.;
        double old_pose_theta_ = 0.; 
        rclcpp::Time old_pose_ts_ = this->get_clock()->now(); // init use now time-stamp 

    };
    
} // namespace turtlesim_robot

#endif