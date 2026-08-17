#include"positioning.hpp"

void turtlesim_robot::Positioning::poseCallback(turtlesim::msg::Pose::SharedPtr pose){
    // add noise 
    double pose_x  = pose->x + random_distribution_x_(random_generator_);
    double pose_y  = pose->y + random_distribution_y_(random_generator_);
    double pose_yaw = pose->theta  + random_distribution_yaw_(random_generator_);

    // build twist msg 
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now(); 
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = pose_x;
    msg.pose.pose.position.y = pose_y;
    msg.pose.pose.position.z = 0.;
    
    // 
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, pose_yaw);
    quat.normalize(); 
    msg.pose.pose.orientation = tf2::toMsg(quat);
    // 6 x 6
    msg.pose.covariance = {std::pow(random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
                            0., std::pow(random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., 0.,
                            0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.stddev(), 2)
                        };

    // publish 
    pose_pub_->publish(msg);

    if (publish_path_topic){
        // pub path, pose array 
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg.header;
        pose_stamped.pose = msg.pose.pose;
        pose_path_msg_.header.stamp = msg.header.stamp; 
        pose_path_msg_.poses.push_back(pose_stamped);
        pose_path_pub_->publish(pose_path_msg_);
    }
    
}

//todo : move it to seperate file 
int main(int argc, char* argv[]){
        rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<turtlesim_robot::Positioning>());
}