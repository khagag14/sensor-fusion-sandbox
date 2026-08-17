#ifndef _EKF_3D_H_
#define _EKF_3D_H_

#include"data_type.hpp"

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>

#include<sophus/so3.hpp>
#include<sophus/se3.hpp>

class Ekf{
    
    public:

    Ekf(){
        is_initialized_ = false;
    }

    void Init(const Sophus::SO3d& R0, const Eigen::Vector3d& p0, double timestamp) {
        R_ = R0;
        p_ = p0;
        time_stamp_ = timestamp;
        cov_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.01; // Small initial variance
        is_initialized_ = true;
    }

    bool IsInitialized() const { return is_initialized_; } 

    bool Predict(const ODOM& odom, const Eigen::Vector3d& v_noise=Eigen::Vector3d::Zero(), 
                                        const Eigen::Vector3d& w_noise=Eigen::Vector3d::Zero()){

        if (!is_initialized_) return false;

        // if (is_first_frame){ // skip first 
        //     time_stamp_ = odom.timestampe_; 
        //     is_first_frame = false; 
        //     return false; 
        // }
        
        double dt = odom.timestampe_ - time_stamp_; 
        if (dt <= 0 ||  dt >= 0.1) { // 0.1 avoid bigger steps
            // todo : add logs 
            time_stamp_ = odom.timestampe_;
            return false;
        }

        // Transition Matrix F_ 6x6 
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
        F.template block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
        F.template block<3, 3>(0, 3) = -R_.matrix() * Sophus::SO3d::hat(odom.v_) * dt; 
        F.template block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() -  Sophus::SO3d::hat(odom.w_) * dt; 
        
        // Nominal Motion Prediction
        p_ = p_ + R_ * odom.v_ * dt;
        R_ = R_ * Sophus::SO3d::exp( odom.w_ * dt);

        // Update Q_ with vel noise 
        Eigen::Matrix<double, 6, 1> noise_vec; 
        /*
        *note : input stds are in velocity space, 
            need to be converted int translation/rotation space, 
                so we can add them to the propagated system cov 
        */
        noise_vec << std::pow(v_noise.x() * dt, 2), std::pow(v_noise.y()* dt, 2), std::pow(v_noise.z()* dt, 2), 
                    std::pow(w_noise.x()* dt, 2), std::pow(w_noise.y()* dt, 2), std::pow(w_noise.z()* dt, 2);
        Eigen::Matrix<double, 6, 6> Q = noise_vec.asDiagonal(); 

        // Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero(); 
        cov_ = F * cov_.eval() * F.transpose() + Q; // Update Motion Noise 
        
        time_stamp_ = odom.timestampe_;

        return true; 
    }

    bool UpdateSE3(const Sophus::SE3d& pose, const Eigen::Vector3d& trans_noise=Eigen::Vector3d::Zero(),
                                             const Eigen::Vector3d& angl_noise=Eigen::Vector3d::Zero()){

        if (!is_initialized_) return false;
        
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        H.template block<3, 3>(0, 0) = Eigen::Matrix<double,3, 3>::Identity(); 
        H.template block<3, 3>(3, 3) = Eigen::Matrix<double,3, 3>::Identity(); 
        
        Eigen::Matrix<double, 6, 1> noise_vec; 
        noise_vec << std::pow(trans_noise.x(), 2), std::pow(trans_noise.y(), 2), std::pow(trans_noise.z(), 2),
                        std::pow(angl_noise.x(), 2), std::pow(angl_noise.y(), 2), std::pow(angl_noise.z(), 2); 
        
        Eigen::Matrix<double, 6, 6> V = noise_vec.asDiagonal(); 

        // compute Kalman Gain 
        Eigen::Matrix<double, 6, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

        // Diff between prediction state and measurements
        Eigen::Matrix<double, 6, 1> innov = Eigen::Matrix<double, 6, 1>::Zero(); // dx 
        innov.template head<3>() = (pose.translation() - p_);
        // innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // wrong as it compute err in body-frame 
        innov.template tail<3>() = (pose.so3() * R_.inverse()).log();  // correct as it compute err in world-frame 

        // Update prediction 
        Eigen::Matrix<double, 6, 1> dx = K * innov; 
        p_ = p_ + dx.template head<3>();
        R_=  Sophus::SO3d::exp(dx.template tail<3>()) * R_;

        // Update Covariance 
        cov_ = (Eigen::Matrix<double, 6, 6>::Identity() - K) * cov_; 

        return true; 
    }

    // getter 
    Sophus::SO3d GetR() const { return R_; }
    Eigen::Vector3d GetP() const { return p_; }
    Eigen::Matrix<double, 6, 6> GetCov() const { return cov_; }

    // private: 
    public:
    // bool is_first_frame = true; 

    double time_stamp_ = 0.0; 
    Sophus::SO3d R_; 
    Eigen::Vector3d p_ = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 6> cov_ = Eigen::Matrix<double, 6, 6>::Identity(); 
    // Eigen::Matrix<double, 6, 6> Q_ = Eigen::Matrix<double, 6, 6>::Zero();

    bool is_initialized_ = false; 

};

#endif