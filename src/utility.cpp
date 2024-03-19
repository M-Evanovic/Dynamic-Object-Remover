#pragma once
#ifndef _REMOVER_H_
#define _REMOVER_H_

#include "remover/RosParam.h"
#include "remover/patchworkpp.hpp"

class Remover : public RosParam
{
private:
    std::vector<Eigen::Matrix4d> poses;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_without_ground;
    std::vector<pcl::PointCloud<PointType>::Ptr> scans_ground;

    std::vector<int> dynamic_point_indices;
    std::vector<int> static_point_indices;

    pcl::PointCloud<PointType>::Ptr global_map;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_global_map;

    pcl::PointCloud<PointType>::Ptr global_map_without_ground;
    pcl::PointCloud<PointType>::Ptr global_map_ground;

    pcl::PointCloud<PointType>::Ptr global_map_current;

    pcl::PointCloud<PointType>::Ptr global_map_dynamic;
    pcl::PointCloud<PointType>::Ptr ground_plane_points;
    pcl::PointCloud<PointType>::Ptr global_map_static;

    PatchWorkpp<PointType> groundSeperator;

public:
    Remover();
    ~Remover();

    void initialize_all(void);
    void loadPosesFromMatrix(std::string txt);
    void loadPosesFromQuaterniond(std::string txt);
    void loadScans(std::string dir);

    void octree_downsample(pcl::PointCloud<PointType>::Ptr& src, pcl::PointCloud<PointType>::Ptr& dst);
    void generateGlobalMap(void);

    cv::Mat scan2range(pcl::PointCloud<PointType>::Ptr scan, float resize_ratio);
    std::pair<cv::Mat, cv::Mat> map2range(pcl::PointCloud<PointType>::Ptr scan, float resize_ratio);
    void getSubMap(int index);
    void transformGlobalMap2Local(int index);
    std::vector<int> getDynamicPointIndexInEachScan(cv::Mat rangeImg_local, cv::Mat rangeImg_different, cv::Mat rangeImg_index);
    void detectDynamicPoint(void);

    void transformDynamicMap2Local(int index);
    void revertStaticPoint(void);

    void removePointUnderHeight(void);

    void separateStaticPointIndexFromGlobalIndex(void);

    void extractStaticMap(void);

    void run(void);
};

#endif
