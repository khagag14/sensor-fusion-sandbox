#include"odometry.hpp"


void turtlesim_robot::Odometry::poseCallback(turtlesim::msg::Pose::SharedPtr pose){
    // gets time-stamp
    rclcpp::Time time_stampe = this->get_clock()->now();

    // comupte linear velocity with correct sign 
    double dt = (time_stampe - old_pose_ts_).seconds();
    dt = dt > 1e-5 ? dt :  1e-5; // avoid diff by zero 
    double vx = (((pose->x - old_pose_x_)*std::cos(pose->theta)) +((pose->y - old_pose_y_)*std::sin(pose->theta)))/dt;
    // std::cout <<  " v-x " << linear_velocity << std::endl;

    // add noise 
    // double linear_velocity  = pose->linear_velocity + random_distribution_vx_(random_generator_);
    double linear_velocity  = vx + random_distribution_vx_(random_generator_);
    double angular_velocity = pose->angular_velocity + random_distribution_wz_(random_generator_);

    // build twist msg 
    auto msg = geometry_msgs::msg::TwistWithCovarianceStamped(); 
    msg.header.stamp = time_stampe;
    msg.header.frame_id = "base_link"; 
    
    msg.twist.twist.angular.x = 0.;
    msg.twist.twist.angular.y = 0.;
    msg.twist.twist.angular.z = angular_velocity;

    msg.twist.twist.linear.x = linear_velocity;
    msg.twist.twist.linear.y = 0.;
    msg.twist.twist.linear.z = 0.;

    msg.twist.covariance = { std::pow(random_distribution_vx_.stddev(), 2), 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., std::pow(random_distribution_wz_.stddev(), 2)
                            };
    // // if (std::abs(cmd_vel_time_stampe_.seconds() - time_stampe.seconds()) < 0.01){
    // if (cmd_vel_vx_ < 0){ // todo : apply temporal time as well. 
    //     std::cout <<  " v-x " << pose->linear_velocity * -1  << ", w-z " << pose->angular_velocity << std::endl;
    //     // new_vel_msg_ = false; 
    // }else{
    //     std::cout <<  " v-x " << pose->linear_velocity << ", w-z " << pose->angular_velocity << std::endl;
    // }
    
    // update 
    old_pose_x_ = pose->x;
    old_pose_y_ = pose->y;
    old_pose_ts_ = time_stampe;
    
    // publish 
    twist_pub_->publish(msg);
}


void turtlesim_robot::Odometry::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg){

    cmd_vel_vx_ = msg->linear.x;
    cmd_vel_wz_ = msg->angular.z;
    cmd_vel_time_stampe_ = this->get_clock()->now();
    new_vel_msg_ = true; 
}

// todo : move it to seperate file 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<turtlesim_robot::Odometry>());
}