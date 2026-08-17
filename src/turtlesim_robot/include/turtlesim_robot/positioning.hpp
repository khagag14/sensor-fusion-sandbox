#ifndef _TURTLESIM_ROBOT_POSITIONING_
#define _TURTLESIM_ROBOT_POSITIONING_

#include<memory>
#include"rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<random>
#include<string>
#include "nav_msgs/msg/path.hpp"


namespace turtlesim_robot
{
    class Positioning : public rclcpp::Node{

        public: 
        Positioning() : Node("_turtlesim_robot_positioning_node_"){
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pub_topic_, 10);
            pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(sub_topic_,
                10, std::bind(&Positioning::poseCallback, this, std::placeholders::_1));
            
            // random noise 
            random_distribution_x_ = std::normal_distribution<double>(error_x_systematic_, error_x_random_);
            // random_distribution_x_{error_x_systematic_, error_x_random_};

            random_distribution_y_ = std::normal_distribution<double>(error_y_systematic_, error_y_random_);
            // random_distribution_y_{error_y_systematic_, error_y_random_};
            
            random_distribution_yaw_ = std::normal_distribution<double>(error_yaw_systematic_, error_yaw_random_);
            // random_distribution_yaw_{error_yaw_systematic_, error_yaw_random_};

            pose_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(pub_pose_path_topic, 10); 
            pose_path_msg_.header.frame_id = "map";
        }

        void poseCallback(turtlesim::msg::Pose::SharedPtr pose);

        private: 
        std::string pub_topic_ = "/turtle1/pose_topic";
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

        std::string sub_topic_ = "/turtle1/pose"; 
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;



        double error_x_systematic_ = 0.0; 
        double error_x_random_ = 0.05;

        double error_y_systematic_ = 0.0; 
        double error_y_random_ = 0.05;
        
        double error_yaw_systematic_ = 0.0;
        double error_yaw_random_ = 0.0349; // 2 deg

        std::default_random_engine random_generator_; 
        std::normal_distribution<double> random_distribution_x_; 
        std::normal_distribution<double> random_distribution_y_; 
        std::normal_distribution<double> random_distribution_yaw_;

        bool publish_path_topic = true; 
        std::string pub_pose_path_topic = "/turtle1/epose_path";
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub_;
        nav_msgs::msg::Path pose_path_msg_; 

    };

}// namespace turtlesim_robot


#endif