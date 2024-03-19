#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <boost/format.hpp>

#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <utility>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <filesystem>

namespace fs = std::filesystem;

using std::ios;
using std::cout;
using std::cerr;
using std::endl;

using PointType = pcl::PointXYZI;

struct SphericalPoint {
    float azimuth;
    float elevation;
    float radius;
};

std::vector<double> split_line(std::string line, char _delimiter);

SphericalPoint Cartesian2Spherical(PointType point);

float radian2degree(float radian);

float degree2radian(float degree);

void seperateVector(std::vector<int> & vector_target, std::vector<int> vector_seperate);
void seperateVector(std::vector<int> vector_src, std::vector<int> vector_seperate, std::vector<int> & vector_target);
void seperateVector(int size, std::vector<int> vector_seperate, std::vector<int> & vector_target);

#endif
