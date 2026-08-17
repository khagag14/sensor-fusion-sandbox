#include"data_type.hpp"
#include"ekf.hpp"

#include"rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"


class RunEkf : public rclcpp::Node{
    
    public:
    RunEkf(): Node("_run_ekf_node_"){

        ekf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pub_ekf_pose_topic, 10);

        ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(pub_ekf_path_topic, 10); 
        ekf_path_msg_.header.frame_id = "map";

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_,
                10, std::bind(&RunEkf::poseCallback, this, std::placeholders::_1));

        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic_,
                10, std::bind(&RunEkf::twistCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "EKF Node initialized.");
    }

    void twistCallback(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg){

        // Drop incoming velocities until we have an absolute pose anchor
        if (!ekf_.IsInitialized()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "Waiting for first Pose measurement to initialize EKF...");
            return;
        }

        // construct odom
        double t = rclcpp::Time(msg->header.stamp).seconds();
        
        Eigen::Vector3d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Eigen::Vector3d w(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        ODOM odom(t, v, w);

        //TODO : extract covarriance from the topic 
        Eigen::Vector3d v_std(std::sqrt(msg->twist.covariance[0] > 0 ? msg->twist.covariance[0] : 0.01),
                              std::sqrt(msg->twist.covariance[7] > 0 ? msg->twist.covariance[7] : 0.01),
                              std::sqrt(msg->twist.covariance[14] > 0 ? msg->twist.covariance[14] : 0.01));
        
        Eigen::Vector3d w_std(std::sqrt(msg->twist.covariance[21] > 0 ? msg->twist.covariance[21] : 0.01),
                              std::sqrt(msg->twist.covariance[28] > 0 ? msg->twist.covariance[28] : 0.01),
                              std::sqrt(msg->twist.covariance[35] > 0 ? msg->twist.covariance[35] : 0.01));
        
        // Single prediction step
        if (ekf_.Predict(odom, v_std, w_std)) {
            publishEkfPose(msg->header.stamp);
        }

        // ekf_.Predict(odom, v_std, w_std);
        // init_ekf = (!init_ekf) ? true : false;

    }

    void poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

        double time = rclcpp::Time(msg->header.stamp).seconds();

        Eigen::Quaterniond q(msg->pose.pose.orientation.w, 
                             msg->pose.pose.orientation.x, 
                             msg->pose.pose.orientation.y, 
                             msg->pose.pose.orientation.z);

        Eigen::Vector3d t(msg->pose.pose.position.x, 
                          msg->pose.pose.position.y, 
                          msg->pose.pose.position.z);

        Sophus::SE3d pose(q, t);

        // 1. If not initialized, set the starting state from the first pose measurement!
        if (!ekf_.IsInitialized()) {
            ekf_.Init(pose.so3(), pose.translation(), time);
            RCLCPP_INFO(this->get_logger(), "EKF Initialized from Pose Callback.");
        return;
        }

        // Extract measurement standard deviations
        Eigen::Vector3d trans_std(std::sqrt(msg->pose.covariance[0] > 0 ? msg->pose.covariance[0] : 0.05),
                                  std::sqrt(msg->pose.covariance[7] > 0 ? msg->pose.covariance[7] : 0.05),
                                  std::sqrt(msg->pose.covariance[14] > 0 ? msg->pose.covariance[14] : 0.05));

        Eigen::Vector3d angl_std(std::sqrt(msg->pose.covariance[21] > 0 ? msg->pose.covariance[21] : 0.01),
                                 std::sqrt(msg->pose.covariance[28] > 0 ? msg->pose.covariance[28] : 0.01),
                                 std::sqrt(msg->pose.covariance[35] > 0 ? msg->pose.covariance[35] : 0.01));

        if(ekf_.UpdateSE3(pose, trans_std, angl_std)){
            publishEkfPose(msg->header.stamp);
        }
        
    }

    void publishEkfPose(const rclcpp::Time& stamp){

        Sophus::SO3d R = ekf_.GetR();
        Eigen::Vector3d T = ekf_.GetP(); 
        Eigen::Matrix<double, 6, 6>cov = ekf_.GetCov();

        // build twist msg 
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped(); 
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        
        //
        msg.pose.pose.position.x = T.x();
        msg.pose.pose.position.y = T.y();
        msg.pose.pose.position.z = T.z();
        
        //
        Eigen::Quaterniond quat(R.unit_quaternion());
        msg.pose.pose.orientation.x = quat.x();
        msg.pose.pose.orientation.y = quat.y();
        msg.pose.pose.orientation.z = quat.z();
        msg.pose.pose.orientation.w = quat.w();

        // 6 x 6 
        msg.pose.covariance.fill(0.0);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                msg.pose.covariance[i * 6 + j] = cov(i, j);
            }
        }

        // publish 
        ekf_pose_pub_->publish(msg);

        // pub path, pose array 
        if (publish_path_topic_){
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = msg.header;
            pose_stamped.pose = msg.pose.pose;
            ekf_path_msg_.header.stamp = stamp; 
            ekf_path_msg_.poses.push_back(pose_stamped);
            ekf_path_pub_->publish(ekf_path_msg_);
        }
    }

    public: 
    
    std::string pose_topic_ = "/turtle1/pose_topic";
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    std::string twist_topic_ = "/turtle1/twist_topic";
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    std::string pub_ekf_pose_topic = "/turtle1/efk_pose";
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_pub_;

    std::string pub_ekf_path_topic = "/turtle1/efk_path";
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_pub_;
    nav_msgs::msg::Path ekf_path_msg_;

    bool publish_path_topic_ = true; 

    // ekf instance 
    Ekf ekf_; 
    // bool init_ekf = false; // make sure init prediction 
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RunEkf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}