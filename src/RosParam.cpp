#include "remover/RosParam.h"


RosParam::RosParam()
: nh(nh_super)
{

    nh.param<std::string>("remover/load_dir", load_dir, "/");
    scans_dir = load_dir + "Scans";
    poses_path = load_dir + "poses.csv";
    nh.param<std::string>("remover/result_dir", result_dir, "/");

    nh.param<bool>("remover/is_large", is_large, false);

    nh.param<std::vector<double>>("remover/Transformation_LiDAR2IMU", vec_transformation_Lidar2IMU, std::vector<double>());
    transformation_Lidar2IMU = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vec_transformation_Lidar2IMU.data(), 4, 4);

    nh.param<int>("remover/start_idx", start_idx, 10);
    nh.param<int>("remover/end_idx", end_idx, 50);

    nh.param<float>("removert/downsample_voxel_size", downsample_voxel_size, 0.05);

    nh.param<float>("remover/FOV_V", FOV_V, 50.0);
    nh.param<float>("remover/FOV_H", FOV_H, 120.0);
    FOV = std::pair<float, float>(FOV_V, FOV_H);

    nh.param<float>("remover/remove_resize_ratio", remove_resize_ratio, 1.0);

    nh.param<bool>("remover/need_revert", need_revert, false);
    nh.param<float>("remover/revert_resize_ratio", revert_resize_ratio, 0.8);
    
    nh.param<float>("remover/range_difference_ratio_threshold", range_difference_ratio_threshold, 0.05);

    nh.param<float>("remover/grid_size", grid_size, 0.5);
    nh.param<float>("remover/occupation_ratio", occupation_ratio, 0.2);  

    usleep(100);
}

