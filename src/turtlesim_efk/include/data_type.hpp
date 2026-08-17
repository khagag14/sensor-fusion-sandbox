#ifndef _DATA_TYPE_H
#define _DATA_TYPE_H

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>

struct ODOM{

    ODOM() = default; 
    ODOM(double t, const Eigen::Vector3d& v, const Eigen::Vector3d& w) : timestampe_(t), v_(v), w_(w){}

    double timestampe_ = 0.0; 
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d w_ = Eigen::Vector3d::Zero(); 
};



#endif