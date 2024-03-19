#include "remover/Remover.h"

Remover::Remover() {
    initialize_all();
}


Remover::~Remover(){}

void Remover::initialize_all(void) {
    global_map.reset(new pcl::PointCloud<PointType>());
    kdtree_global_map.reset(new pcl::KdTreeFLANN<PointType>());
    global_map_without_ground.reset(new pcl::PointCloud<PointType>());
    global_map_ground.reset(new pcl::PointCloud<PointType>());
    global_map_current.reset(new pcl::PointCloud<PointType>());
    global_map_dynamic.reset(new pcl::PointCloud<PointType>());
    ground_plane_points.reset(new pcl::PointCloud<PointType>());
    global_map_static.reset(new pcl::PointCloud<PointType>());

    global_map->clear();
    global_map_without_ground->clear();
    global_map_ground->clear();
    global_map_current->clear();
    global_map_dynamic->clear();
    ground_plane_points->clear();
    global_map_static->clear();
}

void Remover::loadPosesFromMatrix(std::string txt) {
    std::cout << "Loading poses from : " << txt << endl;
    poses.clear();
    poses.reserve(3000);

    std::ifstream stream_in(txt);
    std::string line;

    int index = 0;

    while (std::getline(stream_in, line)) {
        index++;
        if (index < start_idx)
            continue;
        if (index > end_idx) 
            break;

        std::vector<double> line_each = split_line(line, ' ');

        if (line_each.size() == 12) {
            line_each.emplace_back(double(0.0));
            line_each.emplace_back(double(0.0));
            line_each.emplace_back(double(0.0));
            line_each.emplace_back(double(1.0));
        }
        
        Eigen::Matrix4d current_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(line_each.data(), 4, 4);

        poses.emplace_back(current_pose);
    }

    std::cout << poses.size() << " poses have been loaded" << endl;
}

void Remover::loadPosesFromQuaterniond(std::string txt) {
    std::cout << "Loading poses from : " << txt << endl;
    poses.clear();
    poses.reserve(3000);

    std::ifstream stream_in(txt);
    std::string line;

    int index = 0;

    while (std::getline(stream_in, line)) {
        index++;
        if (index < start_idx)
            continue;
        if (index > end_idx) 
            break;

        std::vector<double> line_each = split_line(line, ' ');

        Eigen::Translation3d t(line_each[2], line_each[3], line_each[4]);
        Eigen::Quaterniond q(line_each[8], line_each[5], line_each[6], line_each[7]);
        Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
        current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        current_pose.block<3, 1>(0, 3) = t.vector();
        
        poses.emplace_back(current_pose);
    }

    std::cout << poses.size() << " poses have been loaded" << endl;
}

void Remover::loadScans(std::string dir) {
    std::cout << "Loading scans from : " << dir << endl;
    for (int scan_index = start_idx; scans.size() < poses.size() && scan_index <= end_idx; scan_index++) {
        // std::string pcd_path = scans_dir + "/" + std::to_string(scan_index) + ".pcd";
        std::string pcd_path = (boost::format("%s/%06d.pcd") % scans_dir % scan_index).str();
        pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>);
        pcl::io::loadPCDFile(pcd_path, *points);

        pcl::VoxelGrid<PointType> downsample_filter;
        pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>);
        downsample_filter.setLeafSize(downsample_voxel_size, downsample_voxel_size, downsample_voxel_size);
        downsample_filter.setInputCloud(points);
        downsample_filter.filter(*scan);
        scans.emplace_back(scan);

        pcl::PointCloud<PointType> _scan = *scan;
        pcl::PointCloud<PointType> _scan_without_ground;
        pcl::PointCloud<PointType> _scan_ground;

        groundSeperator.EstimateGround(_scan, _scan_ground, _scan_without_ground);

        pcl::PointCloud<PointType>::Ptr scan_without_ground(new pcl::PointCloud<PointType>(_scan_without_ground));
        pcl::PointCloud<PointType>::Ptr scan_ground(new pcl::PointCloud<PointType>(_scan_ground));
        scans_without_ground.emplace_back(scan_without_ground);
        scans_ground.emplace_back(scan_ground);
    }
    std::cout << scans.size() << " scans have been loaded" << endl;
}

void Remover::octree_downsample(pcl::PointCloud<PointType>::Ptr& src, pcl::PointCloud<PointType>::Ptr& dst) {
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(downsample_voxel_size);
    octree.setInputCloud(src);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    dst->points.assign(centroids.begin(), centroids.end());    
    dst->width = 1; 
    dst->height = dst->points.size();
}

void Remover::generateGlobalMap(void) {
    cout << "Generating global map" << endl;
    for (int index = 0; index < poses.size(); index++) {
        Eigen::Matrix4d pose = poses.at(index);

        pcl::PointCloud<PointType>::Ptr scan = scans.at(index);
        pcl::PointCloud<PointType>::Ptr current_global_map(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*scan, *current_global_map, transformation_Lidar2IMU);
        pcl::transformPointCloud(*current_global_map, *current_global_map, pose);
        *global_map += *current_global_map;

        pcl::PointCloud<PointType>::Ptr scan_without_ground = scans_without_ground.at(index);
        pcl::PointCloud<PointType>::Ptr current_global_map_without_ground(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*scan_without_ground, *current_global_map_without_ground, transformation_Lidar2IMU);
        pcl::transformPointCloud(*current_global_map_without_ground, *current_global_map_without_ground, pose);
        *global_map_without_ground += *current_global_map_without_ground;

        pcl::PointCloud<PointType>::Ptr scan_ground = scans_ground.at(index);
        pcl::PointCloud<PointType>::Ptr current_global_map_ground(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*scan_ground, *current_global_map_ground, transformation_Lidar2IMU);
        pcl::transformPointCloud(*current_global_map_ground, *current_global_map_ground, pose);
        *global_map_ground += *current_global_map_ground;
    }

    if (is_large) {
        octree_downsample(global_map_without_ground, global_map_without_ground);
        kdtree_global_map->setInputCloud(global_map_without_ground);
    }

    std::string global_map_file_name = result_dir + "/" + "GlobalMap.pcd";
    pcl::io::savePCDFileBinary(global_map_file_name, *global_map);
    std::string global_map_without_ground_file_name = result_dir + "/" + "GlobalMap_without_ground.pcd";
    pcl::io::savePCDFileBinary(global_map_without_ground_file_name, *global_map_without_ground);
    std::string global_map_ground_file_name = result_dir + "/" + "GlobalMap_ground.pcd";
    pcl::io::savePCDFileBinary(global_map_ground_file_name, *global_map_ground);
    cout << "Generate global map successfully" << endl;
}

cv::Mat Remover::scan2range(pcl::PointCloud<PointType>::Ptr scan, float resize_ratio) {
    int img_rows = std::round(FOV_V * resize_ratio);
    int img_cols = std::round(FOV_H * resize_ratio);

    cv::Mat rangeImg_local = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(255.0));

    for (int index = 0; index < scan->points.size(); index++) {
        PointType current_point = scan->points[index];
        SphericalPoint current_spherical_point = Cartesian2Spherical(current_point);

        int row_lower_bound = 0;
        int row_upper_bound = img_rows - 1;
        int col_lower_bound = 0;
        int col_upper_bound = img_cols - 1;

        int row = int(std::min(std::max(
            std::round(img_rows * (1 - (radian2degree(current_spherical_point.elevation) + (FOV_V/float(2.0))) / (FOV_V - float(0.0)))),
            float(row_lower_bound)), float(row_upper_bound)));
        int col = int(std::min(std::max(
            std::round(img_cols * ((radian2degree(current_spherical_point.azimuth) + (FOV_H/float(2.0))) / (FOV_H - float(0.0)))), 
            float(col_lower_bound)), float(col_upper_bound)));

        float point_range = current_spherical_point.radius;

        if (point_range < rangeImg_local.at<float>(row, col)) {
            rangeImg_local.at<float>(row, col) = point_range;
        }
    }
    return rangeImg_local;
}

std::pair<cv::Mat, cv::Mat> Remover::map2range(pcl::PointCloud<PointType>::Ptr scan, float resize_ratio) {
    int img_rows = std::round(FOV_V * resize_ratio);
    int img_cols = std::round(FOV_H * resize_ratio);

    cv::Mat rangeImg_global = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(255.0));
    cv::Mat rangeImg_index = cv::Mat(img_rows, img_cols, CV_32FC1, cv::Scalar::all(0.0));

    for (int index = 0; index < scan->points.size(); index++) {
        PointType current_point = scan->points[index];
        SphericalPoint current_spherical_point = Cartesian2Spherical(current_point);

        int row_lower_bound = 0;
        int row_upper_bound = img_rows - 1;
        int col_lower_bound = 0;
        int col_upper_bound = img_cols - 1;

        int row = int(std::min(std::max(
            std::round(img_rows * (1 - (radian2degree(current_spherical_point.elevation) + (FOV_V/float(2.0))) / (FOV_V - float(0.0)))),
            float(row_lower_bound)), float(row_upper_bound)));
        int col = int(std::min(std::max(
            std::round(img_cols * ((radian2degree(current_spherical_point.azimuth) + (FOV_H/float(2.0))) / (FOV_H - float(0.0)))), 
            float(col_lower_bound)), float(col_upper_bound)));

        float point_range = current_spherical_point.radius;

        if (point_range < rangeImg_global.at<float>(row, col)) {
            rangeImg_global.at<float>(row, col) = point_range;
            rangeImg_index.at<int>(row, col) = index;
        }
    }
    return std::pair<cv::Mat, cv::Mat>(rangeImg_global, rangeImg_index);
}

void Remover::getSubMap(int index) {
    Eigen::Matrix4d pose = poses.at(index);
    PointType position;
    position.x = float(pose(0, 3));
    position.y = float(pose(1, 3));
    position.z = float(pose(2, 3));

    std::vector<int> subMap_indices;
    std::vector<float> points_distance;
    kdtree_global_map->radiusSearch(position, 100, subMap_indices, points_distance);

    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(global_map_without_ground);
    boost::shared_ptr<std::vector<int>> points_indices = boost::make_shared<std::vector<int>>(subMap_indices);
    extractor.setIndices(points_indices);
    extractor.setNegative(false);
    extractor.filter(*global_map_current);
}

void Remover::transformGlobalMap2Local(int index) {
    global_map_current->clear();
    Eigen::Matrix4d pose_inverse = poses.at(index).inverse();
    Eigen::Matrix4d transformation_IMU2Lidar = transformation_Lidar2IMU.inverse();
    if (is_large) {
        getSubMap(index);
        pcl::transformPointCloud(*global_map_current, *global_map_current, pose_inverse);
        pcl::transformPointCloud(*global_map_current, *global_map_current, transformation_IMU2Lidar);
    }
    else {
        pcl::transformPointCloud(*global_map_without_ground, *global_map_current, pose_inverse);
        pcl::transformPointCloud(*global_map_current, *global_map_current, transformation_IMU2Lidar);
    }
}

std::vector<int> Remover::getDynamicPointIndexInEachScan(cv::Mat rangeImg_local, cv::Mat rangeImg_different, cv::Mat rangeImg_index) {
    std::vector<int> dynamic_point_indices_in_this_scan;

    for (int row = 0; row < rangeImg_local.rows; row++) {
        for (int col = 0; col < rangeImg_local.cols; col++) {
            float range = rangeImg_local.at<float>(row, col);
            float range_different = rangeImg_different.at<float>(row, col);

            if (range_different < 100 && range_different > range * range_difference_ratio_threshold) {
                dynamic_point_indices_in_this_scan.emplace_back(rangeImg_index.at<int>(row, col));
            }
        }
    }

    return dynamic_point_indices_in_this_scan;
}

void Remover::detectDynamicPoint(void) {
    cout << "Detecting dynamic points" << endl;
    std::vector<int> dynamic_point_indices_temp;
    for (int index = 0; index < scans.size(); index++) {
        pcl::PointCloud<PointType>::Ptr scan_without_ground = scans_without_ground.at(index);
        cv::Mat rangeImg_local = scan2range(scan_without_ground, resize_ratio);
        transformGlobalMap2Local(index);
        std::pair<cv::Mat, cv::Mat> rangeImg = map2range(global_map_current, resize_ratio);
        cv::Mat rangeImg_global = rangeImg.first;
        cv::Mat rangeImg_index = rangeImg.second;
        cv::Mat rangeImg_different = cv::Mat(FOV_V, FOV_H, CV_32FC1, cv::Scalar::all(0.0));
        cv::absdiff(rangeImg_local, rangeImg_global, rangeImg_different);

        std::vector<int> dynamic_point_indexes_in_this_scan = getDynamicPointIndexInEachScan(rangeImg_local, rangeImg_different, rangeImg_index);

        dynamic_point_indices_temp.insert(dynamic_point_indices_temp.end(), dynamic_point_indexes_in_this_scan.begin(), dynamic_point_indexes_in_this_scan.end());
    }
    std::set<int> dynamic_point_indexes_set(dynamic_point_indices_temp.begin(), dynamic_point_indices_temp.end());
    for (auto index : dynamic_point_indexes_set) {
        dynamic_point_indices.emplace_back(index);
    }
    if (dynamic_point_indices.empty()) {
        cout << "no dynamic point" << endl;
        return;
    }

    pcl::ExtractIndices<PointType> extrator;
    boost::shared_ptr<std::vector<int>> _dynamic_point_indices = boost::make_shared<std::vector<int>>(dynamic_point_indices);
    extrator.setInputCloud(global_map_without_ground);
    extrator.setIndices(_dynamic_point_indices);
    extrator.setNegative(false);
    extrator.filter(*global_map_dynamic);
    
    std::string global_map_dynamic_file_name = result_dir + "/" + "Detected_Dynamic_Point.pcd";
    pcl::io::savePCDFileBinary(global_map_dynamic_file_name, *global_map_dynamic);
    cout << "Finish to detect dynamic point" << endl;
}

void Remover::removePointUnderHeight(void) {
    cout << "Generating second dynamic map" << endl;

    float min_x = 10000;
    float max_x = -10000;
    float min_y = 10000;
    float max_y = -10000;
    for (PointType point : *global_map) {
        if (point.x < min_x) {
            min_x = point.x;
        }
        if (point.x > max_x) {
            max_x = point.x;
        }
        if (point.y < min_y) {
            min_y = point.y;
        }
        if (point.y > max_y) {
            max_y = point.y;
        }
    }

    const int x_size = int(std::ceil((max_x - min_x) / grid_size));
    const int y_size = int(std::ceil((max_y - min_y) / grid_size));

    int* num_dynamic_points = new int[x_size * y_size];
    int* num_points = new int[x_size * y_size];

    bool* is_ground_grid = new bool[x_size * y_size];
    bool* is_dynamic_grid = new bool[x_size * y_size];

    float* height = new float[x_size * y_size];
    std::fill(height, height + x_size * y_size, -2.0);

    for (PointType point : *global_map_ground) {
        int x = int((point.x - min_x) / grid_size);
        int y = int((point.y - min_y) / grid_size);

        is_ground_grid[x + y * x_size] = 1; 

        is_ground_grid[x + (y-1) * x_size - 1] = 1;
        is_ground_grid[x + (y-1) * x_size] = 1;
        is_ground_grid[x + (y-1) * x_size + 1] = 1;
        is_ground_grid[x + y * x_size - 1] = 1;
        is_ground_grid[x + y * x_size + 1] = 1;
        is_ground_grid[x + (y+1) * x_size - 1] = 1;
        is_ground_grid[x + (y+1) * x_size] = 1;
        is_ground_grid[x + (y+1) * x_size + 1] = 1;
    }

    for (PointType point : *global_map_without_ground) {
        int x = int((point.x - min_x) / grid_size);
        int y = int((point.y - min_y) / grid_size);

        num_points[x + y * x_size]++;
    }

    for (PointType point : *global_map_dynamic) {
        int x = int((point.x - min_x) / grid_size);
        int y = int((point.y - min_y) / grid_size);
        
        num_dynamic_points[x + y * x_size]++;
        if (point.z > height[x + y * x_size]) {
            height[x + y * x_size] = point.z;
        }
        if (is_ground_grid[x + y * x_size] && is_dynamic_grid[x + y * x_size] == 0 
            && num_dynamic_points[x + y * x_size] / num_points[x + y * x_size] > occupation_ratio 
            && num_dynamic_points[x + y * x_size] > 10) {
            is_dynamic_grid[x + y * x_size] = 1;

            is_dynamic_grid[x + (y-1) * x_size - 1] = 1;
            is_dynamic_grid[x + (y-1) * x_size] = 1;
            is_dynamic_grid[x + (y-1) * x_size + 1] = 1;
            is_dynamic_grid[x + y * x_size - 1] = 1;
            is_dynamic_grid[x + y * x_size + 1] = 1;
            is_dynamic_grid[x + (y+1) * x_size - 1] = 1;
            is_dynamic_grid[x + (y+1) * x_size] = 1;
            is_dynamic_grid[x + (y+1) * x_size + 1] = 1;
        } 
    }

    for (int index = 0; index < global_map_without_ground->points.size(); index++) {
        PointType point = global_map_without_ground->points[index];
        int x = int((point.x - min_x) / grid_size);
        int y = int((point.y - min_y) / grid_size);

        if (is_dynamic_grid[x + y * x_size] == 1 && point.z < height[x + y * x_size]) {
            dynamic_point_indices.emplace_back(index);
        }
    }

    std::set<int> dynamic_point_indices_set(dynamic_point_indices.begin(), dynamic_point_indices.end());
    dynamic_point_indices.clear();
    for (auto index : dynamic_point_indices_set) {
        dynamic_point_indices.emplace_back(index);
    }
    if (dynamic_point_indices.empty()) {
        cout << "no dynamic point" << endl;
        return;
    }

    pcl::ExtractIndices<PointType> extrator;
    boost::shared_ptr<std::vector<int>> _dynamic_point_indices = boost::make_shared<std::vector<int>>(dynamic_point_indices);
    extrator.setInputCloud(global_map_without_ground);
    extrator.setIndices(_dynamic_point_indices);
    extrator.setNegative(false);
    extrator.filter(*global_map_dynamic);
    
    std::string global_map_dynamic_file_name = result_dir + "/" + "Final_Dynamic_Point.pcd";
    pcl::io::savePCDFileBinary(global_map_dynamic_file_name, *global_map_dynamic);
    cout << "Generate dynamic map successfully" << endl;
}

void Remover::separateStaticPointIndexFromGlobalIndex(void) {
    static_point_indices.clear();
    int index_seperate_ptr = 0;
    for (int index = 0; index < global_map_without_ground->points.size(); index++) {
        if (index != dynamic_point_indices[index_seperate_ptr]) {
            static_point_indices.emplace_back(index);
        }
        else {
            index_seperate_ptr++;
        }
    }
}

void Remover::extractStaticMap(void) {
    cout << "Generating final static map" << endl;
    separateStaticPointIndexFromGlobalIndex();

    pcl::ExtractIndices<PointType> extrator;
    boost::shared_ptr<std::vector<int>> _static_point_indices = boost::make_shared<std::vector<int>>(static_point_indices);
    extrator.setInputCloud(global_map_without_ground);
    extrator.setIndices(_static_point_indices);
    extrator.setNegative(false);
    extrator.filter(*global_map_static);
    
    *global_map_static += *global_map_ground;

    std::string global_map_static_file_name = result_dir + "/" + "Final_Static_Point.pcd";
    pcl::io::savePCDFileBinary(global_map_static_file_name, *global_map_static);
    cout << "Generate final static map successfully" << endl;
}

void Remover::run(void) {
    pcl::console::TicToc time;
    time.tic();
    // loadPosesFromMatrix(poses_path);
    loadPosesFromQuaterniond(poses_path);
    loadScans(scans_dir);
    generateGlobalMap();
    detectDynamicPoint();
    removePointUnderHeight();
    extractStaticMap();
    cout << "总耗时:";
    time.toc_print();
}
