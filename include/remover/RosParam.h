#pragma once
#ifndef _ROSPARAM_H_
#define _ROSPARAM_H_

#include "utility.h"

class RosNodeHandle
{
public:
    ros::NodeHandle nh_super;
};

class RosParam: public RosNodeHandle
{
public:
    
    ros::NodeHandle & nh;

    std::string load_dir;
    std::string scans_dir;
    std::string poses_path;
    std::string result_dir;

    bool is_large;

    std::vector<double> vec_transformation_Lidar2IMU;
    Eigen::Matrix4d transformation_Lidar2IMU;

    int start_idx;
    int end_idx;

    float downsample_voxel_size;

    float FOV_V;
    float FOV_H;
    std::pair<float, float> FOV;

    float remove_resize_ratio;

    bool need_revert;
    float revert_resize_ratio;
    
    float range_difference_ratio_threshold;

    float grid_size;
    float occupation_ratio;

public:
    RosParam();
};

#endif
