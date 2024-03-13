/**
 * @file patchworkpp.hpp
 * @author Seungjae Lee
 * @brief
 * @version 0.1
 * @date 2022-07-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PATCHWORKPP_H
#define PATCHWORKPP_H

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <numeric>
#include <queue>
#include <mutex>


#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/search/impl/search.hpp>

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55
#define FLAT_ENOUGH 0.2
#define TOO_HIGH_ELEVATION 0.0
#define TOO_TILTED 1.0

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000


template <typename PointT>
struct RevertCandidate
{
    int concentric_idx;
    int sector_idx;
    double ground_flatness;
    double line_variable;
    Eigen::Vector4f pc_mean;
    pcl::PointCloud<PointT> regionwise_ground;

    RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var, Eigen::Vector4f _pc_mean, pcl::PointCloud<PointT> _ground)
        : concentric_idx(_c_idx), sector_idx(_s_idx), ground_flatness(_flatness), line_variable(_line_var), pc_mean(_pc_mean), regionwise_ground(_ground) {}
};

template <typename PointT>
class PatchWorkpp
{
public:
    struct Options
    {
        struct CZM
        {
            CZM()
            {
                num_sectors_each_zone = {16, 32, 54, 32};
                num_rings_each_zone = {2, 4, 4, 4};
                elevation_thr = {0.0, 0.0, 0.0, 0.0};
                flatness_thr = {0.0, 0.0, 0.0, 0.0};
            }
            int num_zones = 4;
            std::vector<int> num_sectors_each_zone;
            std::vector<int> num_rings_each_zone;
            std::vector<double> elevation_thr;
            std::vector<double> flatness_thr;
        };

        Options() {}

        bool verbose = false;
        double sensor_height = 1.7;
        int num_iter = 3;
        int num_lpr = 20;
        int num_min_pts = 10;
        double th_seeds = 0.3;
        double th_dist = 0.125;
        double th_seeds_v = 0.25;
        double th_dist_v = 0.1;
        double max_range = 80.0;
        double min_range = 2.7;
        double uprightness_thr = 0.707;
        double adaptive_seed_selection_margin = -1.2;
        double RNR_ver_angle_thr = -15.0;
        double RNR_intensity_thr = 0.2;
        int max_flatness_storage = 1000;
        int max_elevation_storage = 1000;

        bool enable_RNR = true;
        bool enable_RVPF = true;
        bool enable_TGR = true;
        CZM czm;
    };

public:
    typedef std::vector<pcl::PointCloud<PointT>> Ring;
    typedef std::vector<Ring> Zone;
    Options option_;

    PatchWorkpp(Options option = Options()) : option_(option)
    {
        // init params
        Initialization();
    }   

    void Initialization()
    {
        revert_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
        regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

        num_rings_of_interest_ = option_.czm.elevation_thr.size();

        double min_range_z2 = (7 * option_.min_range + option_.max_range) / 8.0; // 12.3625
        double min_range_z3 = (3 * option_.min_range + option_.max_range) / 4.0; // 22.025
        double min_range_z4 = (option_.min_range + option_.max_range) / 2.0;     // 41.35

        min_ranges_ = {option_.min_range, min_range_z2, min_range_z3, min_range_z4};
        sector_sizes_ = {2 * M_PI / option_.czm.num_sectors_each_zone.at(0), 2 * M_PI / option_.czm.num_sectors_each_zone.at(1),
                         2 * M_PI / option_.czm.num_sectors_each_zone.at(2),
                         2 * M_PI / option_.czm.num_sectors_each_zone.at(3)};
        ring_sizes_ = {(min_range_z2 - option_.min_range) / option_.czm.num_rings_each_zone.at(0),
                       (min_range_z3 - min_range_z2) / option_.czm.num_rings_each_zone.at(1),
                       (min_range_z4 - min_range_z3) / option_.czm.num_rings_each_zone.at(2),
                       (option_.max_range - min_range_z4) / option_.czm.num_rings_each_zone.at(3)};
        

        for (auto x : sector_sizes_) {

            std::cout << x << " ";
        }
        std::cout << std::endl;

        for (auto x : ring_sizes_) {

            std::cout << x << " ";
        }
        std::cout << std::endl;        
        
        for (int i = 0; i < option_.czm.num_zones; i++)
        {
            Zone z;
            InitializeZone(z, option_.czm.num_sectors_each_zone[i], option_.czm.num_rings_each_zone[i]);
            ConcentricZoneModel_.push_back(z);
        }
    }

    void EstimateGround(pcl::PointCloud<PointT> cloud_in,
                        pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground
                        /*, Eigen::VectorXf &ground_normal*/);

private:
    // Every private member variable is written with the undescore("_") in its end.

    std::recursive_mutex mutex_;

    int num_rings_of_interest_;

    std::vector<double> min_ranges_;
    std::vector<double> sector_sizes_;
    std::vector<double> ring_sizes_;

    std::vector<double> update_flatness_[4];
    std::vector<double> update_elevation_[4];

    float d_;
    Eigen::VectorXf normal_;

    Eigen::VectorXf singular_values_;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;

    // For visualization
    bool visualize_ = false;

    std::queue<int> noise_idxs_;

    std::vector<Zone> ConcentricZoneModel_;

    pcl::PointCloud<PointT> revert_pc_, reject_pc_, noise_pc_, vertical_pc_;
    pcl::PointCloud<PointT> ground_pc_;
    
    pcl::PointCloud<PointT> regionwise_ground_, regionwise_nonground_;

private:

    void InitializeZone(Zone &z, int num_sectors, int num_rings);

    void FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings);

    /**
     * @brief 清空初始化czm
     * 
     * @param czm 
     */
    void FlushPatches(std::vector<Zone> &czm);

    /**
     * @brief 将输入点云，将点云放入czm
     * 
     * @param src 
     * @param czm 
     * @param cloud_nonground 
     */
    void PC2CZM(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm, pcl::PointCloud<PointT> &cloud_nonground);

    /**
     * @brief 去除噪声点
     * 
     * @param cloud 
     * @param cloud_nonground 
     */
    void ReflectedNoiseRemoval(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_nonground);

    void TemporalGroundRevert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                                std::vector<double> ring_flatness, std::vector<RevertCandidate<PointT>> candidates,
                                int concentric_idx);

    void CalcMeanStdev(std::vector<double> vec, double &mean, double &stdev);

    void UpdateElevationThr();
    void UpdateFlatnessThr();

    double XY2Theta(const double &x, const double &y);

    double XY2Radius(const double &x, const double &y);

    /**
     * @brief 计算平面的法向量，截距
     * 
     * @param ground 
     */
    void EstimatePlane(const pcl::PointCloud<PointT> &ground);

    /**
     * @brief 
     * 
     * @param zone_idx 
     * @param src 
     * @param dst 
     * @param non_ground_dst 
     */
    void ExtractPiecewiseGround(
        const int zone_idx, const pcl::PointCloud<PointT> &src,
        pcl::PointCloud<PointT> &dst,
        pcl::PointCloud<PointT> &non_ground_dst);

    /**
     * @brief 找到平均高度下的点
     *
     * @param zone_idx
     * @param p_sorted
     * @param init_seeds
     */
    void ExtractInitialSeeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds);

    void ExtractInitialSeeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds, double th_seed);

    double CalculateCosineSimilarity(pcl::PointXYZINormal &pt1, pcl::PointXYZINormal &pt2);

    pcl::PointCloud<pcl::PointXYZINormal> RemoveDifferentPlanes(pcl::PointCloud<pcl::PointXYZINormal> &normals);


    Eigen::VectorXf FitGroundPlane(pcl::PointCloud<pcl::PointXYZINormal> &normals);

    Eigen::VectorXf FitRansacGroundPlane(pcl::PointCloud<PointT> &cloud);

};









template <typename PointT>
inline void PatchWorkpp<PointT>::InitializeZone(Zone &z, int num_sectors, int num_rings)
{
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring ring;
    for (int i = 0; i < num_sectors; i++)
    {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++)
    {
        z.emplace_back(ring);
    }
}

template <typename PointT>
inline void PatchWorkpp<PointT>::FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings)
{
    for (int i = 0; i < num_sectors; i++)
    {
        for (int j = 0; j < num_rings; j++)
        {
            if (!patches[j][i].points.empty())
                patches[j][i].points.clear();
        }
    }
}

template <typename PointT>
inline void PatchWorkpp<PointT>::FlushPatches(std::vector<Zone> &czm)
{
    for (int k = 0; k < option_.czm.num_zones; k++)
    {
        for (int i = 0; i < option_.czm.num_rings_each_zone[k]; i++)
        {
            for (int j = 0; j < option_.czm.num_sectors_each_zone[k]; j++)
            {
                if (!czm[k][i][j].points.empty()) {
                    czm[k][i][j].points.clear();
                }
                    
            }
        }
    }

    if (option_.verbose) {
        std::cout << "Flushed patches" << std::endl;
    }
        
}

template <typename PointT>
inline void PatchWorkpp<PointT>::EstimatePlane(const pcl::PointCloud<PointT> &ground)
{
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));

    if (normal_(2) < 0)
    {
        for (int i = 0; i < 3; i++)
        {
            normal_(i) *= -1;
        }
    }

    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
}

template <typename PointT>
inline void PatchWorkpp<PointT>::ExtractInitialSeeds(
    const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
    pcl::PointCloud<PointT> &init_seeds, double th_seed)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    int init_idx = 0;

    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); i++)
        {
            if (p_sorted.points[i].z < option_.adaptive_seed_selection_margin * option_.sensor_height)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < option_.num_lpr; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+option_.th_seeds
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seed)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template <typename PointT>
inline void PatchWorkpp<PointT>::ExtractInitialSeeds(
    const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
    pcl::PointCloud<PointT> &init_seeds)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    int init_idx = 0;

    if (zone_idx == 0)
    {
        for (int i = 0; i < p_sorted.points.size(); i++)
        {
            if (p_sorted.points[i].z < option_.adaptive_seed_selection_margin * option_.sensor_height)
            {
                ++init_idx;
            }
            else
            {
                break;
            }
        }
    }

    // Calculate the mean height value.
    for (int i = init_idx; i < p_sorted.points.size() && cnt < option_.num_lpr; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+option_.th_seeds
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + option_.th_seeds)
        {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}

template <typename PointT>
inline void PatchWorkpp<PointT>::ReflectedNoiseRemoval(pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_nonground)
{
    for (int i = 0; i < cloud_in.size(); i++)
    {
        double r = sqrt(cloud_in[i].x * cloud_in[i].x + cloud_in[i].y * cloud_in[i].y);
        double z = cloud_in[i].z;
        double ver_angle_in_deg = atan2(z, r) * 180 / M_PI;

        if (ver_angle_in_deg < option_.RNR_ver_angle_thr && z < -option_.sensor_height - 0.8 && cloud_in[i].intensity < option_.RNR_intensity_thr)
        {
            cloud_nonground.push_back(cloud_in[i]);
            noise_pc_.push_back(cloud_in[i]);
            noise_idxs_.push(i);
        }
    }

    if (option_.verbose) {
        std::cout << "[ RNR ] Num of noises : " << noise_pc_.points.size() << std::endl;
    }
        
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/

template <typename PointT>
inline void PatchWorkpp<PointT>::EstimateGround(
    pcl::PointCloud<PointT> cloud_in,
    pcl::PointCloud<PointT> &cloud_ground,
    pcl::PointCloud<PointT> &cloud_nonground
    /*, Eigen::VectorXf &ground_normal*/)
{

    std::unique_lock<std::recursive_mutex> lock(mutex_);

    cloud_ground.clear();
    cloud_nonground.clear();
    pcl::PointCloud<pcl::PointXYZINormal> normals;
    
    
    // 1. Reflected Noise Removal (RNR) 去噪声，特别异常的点
    if (option_.enable_RNR)
    {
        ReflectedNoiseRemoval(cloud_in, cloud_nonground);
    }

    // 2. Concentric Zone Model (CZM)
    FlushPatches(ConcentricZoneModel_); // 清空czm
    PC2CZM(cloud_in, ConcentricZoneModel_, cloud_nonground);

    int concentric_idx = 0;

    std::vector<RevertCandidate<PointT>> candidates;
    std::vector<double> ringwise_flatness;

    for (int zone_idx = 0; zone_idx < option_.czm.num_zones; ++zone_idx)
    {
        auto zone = ConcentricZoneModel_[zone_idx];

        for (int ring_idx = 0; ring_idx < option_.czm.num_rings_each_zone[zone_idx]; ++ring_idx)
        {
            for (int sector_idx = 0; sector_idx < option_.czm.num_sectors_each_zone[zone_idx]; ++sector_idx)
            {
                if (zone[ring_idx][sector_idx].points.size() < option_.num_min_pts)
                {
                    cloud_nonground += zone[ring_idx][sector_idx];
                    continue;
                }

                // --------- region-wise sorting (faster than global sorting method) ---------------- //
                // sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(), PointZCmp<PointT>);
                std::sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].points.end(), [](auto a, auto b){ return a.z < b.z;});

                // ---------------------------------------------------------------------------------- //
                ExtractPiecewiseGround(zone_idx, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                // Status of each patch
                // used in checking uprightness, elevation, and flatness, respectively
                const double ground_uprightness = normal_(2);
                const double ground_elevation = pc_mean_(2, 0);
                const double ground_flatness = singular_values_.minCoeff();
                const double line_variable = singular_values_(1) != 0 ? singular_values_(0) / singular_values_(1) : std::numeric_limits<double>::max();

                double heading = 0.0;
                for (int i = 0; i < 3; i++) {
                    heading += pc_mean_(i, 0) * normal_(i);
                }

                if ((pc_mean_(0, 0) * pc_mean_(0, 0) + pc_mean_(1, 0) * pc_mean_(1, 0)) < 36) {
                    pcl::PointXYZINormal tmp_p;
                    tmp_p.x = pc_mean_(0, 0);
                    tmp_p.y = pc_mean_(1, 0);
                    tmp_p.z = pc_mean_(2, 0);
                    tmp_p.normal_x = normal_(0);
                    tmp_p.normal_y = normal_(1);
                    tmp_p.normal_z = normal_(2);
                    tmp_p.curvature = d_;
                    normals.points.emplace_back(tmp_p);
                }


                /*
                    About 'is_heading_outside' condidition, heading should be smaller than 0 theoretically.
                    ( Imagine the geometric relationship between the surface normal vector on the ground plane and
                        the vector connecting the sensor origin and the mean point of the ground plane )

                    However, when the patch is far awaw from the sensor origin,
                    heading could be larger than 0 even if it's ground due to lack of amount of ground plane points.

                    Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near condition )
                */
                bool is_upright = ground_uprightness > option_.uprightness_thr;
                bool is_not_elevated = ground_elevation < option_.czm.elevation_thr[concentric_idx];
                bool is_flat = ground_flatness < option_.czm.flatness_thr[concentric_idx];
                bool is_near_zone = concentric_idx < num_rings_of_interest_;
                bool is_heading_outside = heading < 0.0;

                /*
                    Store the elevation & flatness variables
                    for A-GLE (Adaptive Ground Likelihood Estimation)
                    and TGR (Temporal Ground Revert). More information in the paper Patchwork++.
                */
                if (is_upright && is_not_elevated && is_near_zone)
                {
                    update_elevation_[concentric_idx].push_back(ground_elevation);
                    update_flatness_[concentric_idx].push_back(ground_flatness);

                    ringwise_flatness.push_back(ground_flatness);
                }

                // Ground estimation based on conditions
                if (!is_upright)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (!is_near_zone)
                {
                    cloud_ground += regionwise_ground_;
                }
                else if (!is_heading_outside)
                {
                    cloud_nonground += regionwise_ground_;
                }
                else if (is_not_elevated || is_flat)
                {
                    cloud_ground += regionwise_ground_;
                }
                else
                {
                    RevertCandidate<PointT> candidate(concentric_idx, sector_idx, ground_flatness, line_variable, pc_mean_, regionwise_ground_);
                    candidates.push_back(candidate);
                }
                // Every regionwise_nonground is considered nonground.
                cloud_nonground += regionwise_nonground_;

            }


            if (!candidates.empty())
            {
                if (option_.enable_TGR)
                {
                    TemporalGroundRevert(cloud_ground, cloud_nonground, ringwise_flatness, candidates, concentric_idx);
                }
                else
                {
                    for (size_t i = 0; i < candidates.size(); i++)
                    {
                        cloud_nonground += candidates[i].regionwise_ground;
                    }
                }

                candidates.clear();
                ringwise_flatness.clear();
            }


            concentric_idx++;
        }


        // std::cout << option_.czm.num_rings_each_zone[zone_idx] << "  " << option_.czm.num_sectors_each_zone[zone_idx] << std::endl;
    }

    UpdateElevationThr();
    UpdateFlatnessThr();


    // TODO 平面拟合
    
    // if (normals.size() < 5) {
    //      // auto aaa = RemoveDifferentPlanes(normals);
    //     ground_normal = FitGroundPlane(normals);
    // } else {
    //     ground_normal = FitRansacGroundPlane(cloud_ground);
    // }
   
    

    // std::cout << normals.size() << std::endl;

    revert_pc_.clear();
    reject_pc_.clear();
    noise_pc_.clear();
    vertical_pc_.clear();

}


template <typename PointT>
double PatchWorkpp<PointT>::CalculateCosineSimilarity(pcl::PointXYZINormal &pt1, pcl::PointXYZINormal &pt2) {
    double dot_product = pt1.x * pt2.x + pt1.y * pt2.y + pt1.z * pt2.z;
    double norm1 = std::sqrt(pt1.x * pt1.x + pt1.y * pt1.y + pt1.z * pt1.z);
    double norm2 = std::sqrt(pt2.x * pt2.x + pt2.y * pt2.y + pt2.z * pt2.z);
    return dot_product / (norm1 * norm2);
}



template <typename PointT>
pcl::PointCloud<pcl::PointXYZINormal> PatchWorkpp<PointT>::RemoveDifferentPlanes(pcl::PointCloud<pcl::PointXYZINormal> &normals) {
    pcl::PointCloud<pcl::PointXYZINormal> after_planes;
    std::vector<int> is_different(normals.size(), 0);
    // 对每一对法向量计算相似度，并标记差异较大的法向量
    for (size_t i = 0; i < normals.size(); ++i) {
        for (size_t j = i + 1; j < normals.size(); ++j) {
            double similarity = CalculateCosineSimilarity(normals[i], normals[j]);
            if (similarity < 0.9) {
                is_different[i]++;
                is_different[j]++;
            }
        }
    }

    for (size_t i = 0; i < is_different.size(); ++i) {
        if ((is_different[i] / is_different.size()) < 0.3) {
            after_planes.push_back(normals[i]);
        }
    }

    return after_planes;
}



template <typename PointT>
Eigen::VectorXf PatchWorkpp<PointT>::FitGroundPlane(pcl::PointCloud<pcl::PointXYZINormal> &normals) {
    Eigen::VectorXf plane = Eigen::VectorXf::Constant(4, 0);;
    for (auto pt : normals) {
        plane[0] += pt.normal_x;
        plane[1] += pt.normal_y;
        plane[2] += pt.normal_z;
        plane[3] += pt.curvature;
    }

    plane[0] /= normals.size();
    plane[1] /= normals.size();
    plane[2] /= normals.size();
    plane[3] /= normals.size();

    // std::cout << "--  " << plane.transpose() << std::endl;
    return plane;
}


template <typename PointT>
Eigen::VectorXf PatchWorkpp<PointT>::FitRansacGroundPlane(pcl::PointCloud<PointT> &cloud) {
    typename pcl::PointCloud<PointT>::Ptr restricted_cloud_ptr(new pcl::PointCloud<PointT>());
    for (auto pt : cloud) {
        if ((pt.x * pt.x + pt.y * pt.y) < 36) {
            restricted_cloud_ptr->push_back(pt);
        }
    }
    Eigen::VectorXf normal = Eigen::VectorXf::Constant(7, 0);;

    // RANSAC

    typename pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(restricted_cloud_ptr));

    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }
    // std::cout << "++  " << coeffs.transpose() << std::endl;

    return coeffs;

}


template <typename PointT>
inline void PatchWorkpp<PointT>::UpdateElevationThr(void)
{
    for (int i = 0; i < num_rings_of_interest_; i++)
    {
        if (update_elevation_[i].empty())
            continue;

        double update_mean = 0.0, update_stdev = 0.0;
        CalcMeanStdev(update_elevation_[i], update_mean, update_stdev);
        if (i == 0)
        {
            option_.czm.elevation_thr[i] = update_mean + 3 * update_stdev;
            option_.sensor_height = -update_mean;
        }
        else {
            option_.czm.elevation_thr[i] = update_mean + 2 * update_stdev;
        }
            

        // if (option_.verbose) std::cout << "elevation threshold [" << i << "]: " << option_.czm.elevation_thr[i] << std::endl;

        int exceed_num = update_elevation_[i].size() - option_.max_elevation_storage;
        if (exceed_num > 0) {
            update_elevation_[i].erase(update_elevation_[i].begin(), update_elevation_[i].begin() + exceed_num);
        }
            
    }

    if (option_.verbose)
    {
        std::cout << "sensor height: " << option_.sensor_height << std::endl;
        std::cout << (boost::format("option_.czm.elevation_thr  :   %0.4f,  %0.4f,  %0.4f,  %0.4f") % option_.czm.elevation_thr[0] % option_.czm.elevation_thr[1] % option_.czm.elevation_thr[2] % option_.czm.elevation_thr[3]).str() << std::endl;
    }

    return;
}

template <typename PointT>
inline void PatchWorkpp<PointT>::UpdateFlatnessThr(void)
{
    for (int i = 0; i < num_rings_of_interest_; i++)
    {
        if (update_flatness_[i].empty())
        {break;}
        if (update_flatness_[i].size() <= 1)
        {break;}

        double update_mean = 0.0, update_stdev = 0.0;
        CalcMeanStdev(update_flatness_[i], update_mean, update_stdev);
        option_.czm.flatness_thr[i] = update_mean + update_stdev;

        // if (option_.verbose) { std::cout << "flatness threshold [" << i << "]: " << option_.czm.flatness_thr[i] << std::endl; }

        int exceed_num = update_flatness_[i].size() - option_.max_flatness_storage;
        if (exceed_num > 0){
            update_flatness_[i].erase(update_flatness_[i].begin(), update_flatness_[i].begin() + exceed_num);
        }
            
    }

    if (option_.verbose)
    {
        std::cout << (boost::format("option_.czm.flatness_thr   :   %0.4f,  %0.4f,  %0.4f,  %0.4f") % option_.czm.flatness_thr[0] % option_.czm.flatness_thr[1] % option_.czm.flatness_thr[2] % option_.czm.flatness_thr[3]).str() << std::endl;
    }

    return;
}

template <typename PointT>
inline void PatchWorkpp<PointT>::TemporalGroundRevert(pcl::PointCloud<PointT> &cloud_ground, pcl::PointCloud<PointT> &cloud_nonground,
                                                        std::vector<double> ring_flatness, std::vector<RevertCandidate<PointT>> candidates,
                                                        int concentric_idx)
{
    if (option_.verbose) {
        std::cout << "\033[1;34m"
                  << "=========== Temporal Ground Revert (TGR) ==========="
                  << "\033[0m" << std::endl;

    }


    double mean_flatness = 0.0, stdev_flatness = 0.0;
    CalcMeanStdev(ring_flatness, mean_flatness, stdev_flatness);

    if (option_.verbose)
    {
        std::cout << "[" << candidates[0].concentric_idx << ", " << candidates[0].sector_idx << "]"
             << " mean_flatness: " << mean_flatness << ", stdev_flatness: " << stdev_flatness << std::endl;
    }

    for (size_t i = 0; i < candidates.size(); i++)
    {
        RevertCandidate<PointT> candidate = candidates[i];

        // Debug
        if (option_.verbose)
        {
            std::cout << "\033[1;33m" << candidate.sector_idx << "th flat_sector_candidate"
                 << " / flatness: " << candidate.ground_flatness
                 << " / line_variable: " << candidate.line_variable
                 << " / ground_num : " << candidate.regionwise_ground.size()
                 << "\033[0m" << std::endl;
        }

        double mu_flatness = mean_flatness + 1.5 * stdev_flatness;
        double prob_flatness = 1 / (1 + exp((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10)));

        if (candidate.regionwise_ground.size() > 1500 && candidate.ground_flatness < option_.th_dist * option_.th_dist) {
            prob_flatness = 1.0;
        }
            

        double prob_line = 1.0;
        if (candidate.line_variable > 8.0) //&& candidate.line_dir > M_PI/4)// candidate.ground_elevation > option_.czm.elevation_thr[concentric_idx])
        {
            // if (option_.verbose) std::cout << "line_dir: " << candidate.line_dir << std::endl;
            prob_line = 0.0;
        }

        bool revert = prob_line * prob_flatness > 0.5;

        if (concentric_idx < num_rings_of_interest_)
        {
            if (revert)
            {
                if (option_.verbose)
                {
                    std::cout << "\033[1;32m"
                         << "REVERT TRUE"
                         << "\033[0m" << std::endl;
                }

                revert_pc_ += candidate.regionwise_ground;
                cloud_ground += candidate.regionwise_ground;
            }
            else
            {
                if (option_.verbose)
                {
                    std::cout << "\033[1;31m"
                         << "FINAL REJECT"
                         << "\033[0m" << std::endl;
                }
                reject_pc_ += candidate.regionwise_ground;
                cloud_nonground += candidate.regionwise_ground;
            }
        }
    }

    if (option_.verbose) {
        std::cout << "\033[1;34m"
                  << "===================================================="
                  << "\033[0m" << std::endl;
    }

}

// For adaptive
template <typename PointT>
inline void PatchWorkpp<PointT>::ExtractPiecewiseGround(
    const int zone_idx, const pcl::PointCloud<PointT> &src,
    pcl::PointCloud<PointT> &dst,
    pcl::PointCloud<PointT> &non_ground_dst)
{

    // 0. Initialization
    if (!ground_pc_.empty()) {
        ground_pc_.clear();
    }
        
    if (!dst.empty()) {
        dst.clear();
    }
        
    if (!non_ground_dst.empty()) {
        non_ground_dst.clear();
    }

    // 1. Region-wise Vertical Plane Fitting (R-VPF)
    // : removes potential vertical plane under the ground plane
    pcl::PointCloud<PointT> src_wo_verticals;
    src_wo_verticals = src;

    if (option_.enable_RVPF)
    {
        for (int i = 0; i < option_.sensor_height; i++)
        {
            ExtractInitialSeeds(zone_idx, src_wo_verticals, ground_pc_, option_.th_seeds_v);
            EstimatePlane(ground_pc_);

            if (zone_idx == 0 && normal_(2) < option_.uprightness_thr)
            {
                pcl::PointCloud<PointT> src_tmp;
                src_tmp = src_wo_verticals;
                src_wo_verticals.clear();

                Eigen::MatrixXf points(src_tmp.points.size(), 3);
                int j = 0;
                for (auto &p : src_tmp.points)
                {
                    points.row(j++) << p.x, p.y, p.z;
                }
                // ground plane model
                Eigen::VectorXf result = points * normal_;

                for (int r = 0; r < result.rows(); r++)
                {
                    if (result[r] < option_.th_dist_v - d_ && result[r] > -option_.th_dist_v - d_)
                    {
                        non_ground_dst.points.push_back(src_tmp[r]);
                        vertical_pc_.points.push_back(src_tmp[r]);
                    }
                    else
                    {
                        src_wo_verticals.points.push_back(src_tmp[r]);
                    }
                }
            } else {
                break;
            }
                
        }
    }

    ExtractInitialSeeds(zone_idx, src_wo_verticals, ground_pc_);
    EstimatePlane(ground_pc_);

    // 2. Region-wise Ground Plane Fitting (R-GPF)
    // : fits the ground plane

    // pointcloud to matrix
    Eigen::MatrixXf points(src_wo_verticals.points.size(), 3);
    int j = 0;
    for (auto &p : src_wo_verticals.points)
    {
        points.row(j++) << p.x, p.y, p.z;
    }

    for (int i = 0; i < option_.sensor_height; i++)
    {

        ground_pc_.clear();

        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (i < option_.sensor_height - 1)
            {
                if (result[r] < option_.th_dist - d_)
                {
                    ground_pc_.points.push_back(src_wo_verticals[r]);
                }
            }
            else
            { // Final stage
                if (result[r] < option_.th_dist - d_)
                {
                    dst.points.push_back(src_wo_verticals[r]);
                }
                else
                {
                    non_ground_dst.points.push_back(src_wo_verticals[r]);
                }
            }
        }

        if (i < option_.sensor_height - 1)
        {
            EstimatePlane(ground_pc_);
        }    
        else {
            EstimatePlane(dst);
        }
            
    }
}



template <typename PointT>
inline void PatchWorkpp<PointT>::CalcMeanStdev(std::vector<double> vec, double &mean, double &stdev)
{
    if (vec.size() <= 1)
        return;

    mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();

    for (int i = 0; i < vec.size(); i++)
    {
        stdev += (vec.at(i) - mean) * (vec.at(i) - mean);
    }
    stdev /= vec.size() - 1;
    stdev = sqrt(stdev);
}

template <typename PointT>
inline double PatchWorkpp<PointT>::XY2Theta(const double &x, const double &y)
{ // 0 ~ 2 * PI
    // if (y >= 0) {
    //     return atan2(y, x); // 1, 2 quadrant
    // } else {
    //     return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    // }

    double angle = atan2(y, x);
    return angle > 0 ? angle : 2 * M_PI + angle;
}

template <typename PointT>
inline double PatchWorkpp<PointT>::XY2Radius(const double &x, const double &y)
{
    return sqrt(pow(x, 2) + pow(y, 2));
}

template <typename PointT>
inline void PatchWorkpp<PointT>::PC2CZM(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm, pcl::PointCloud<PointT> &cloud_nonground)
{
    for (int i = 0; i < src.size(); i++)
    {
        if ((!noise_idxs_.empty()) && (i == noise_idxs_.front()))
        {
            noise_idxs_.pop();
            continue;
        }

        PointT pt = src.points[i];

        double r = XY2Radius(pt.x, pt.y);
        if ((r <= option_.max_range) && (r > option_.min_range))
        {
            double theta = XY2Theta(pt.x, pt.y);

            int zone_idx = 0;
            if (r < min_ranges_[1]) {
                zone_idx = 0;
            } else if (r < min_ranges_[2]) {
                zone_idx = 1;
            } else if (r < min_ranges_[3]) {
                zone_idx = 2;
            } else {
                zone_idx = 3;
            }
            int ring_idx = std::min(static_cast<int>(((r - min_ranges_[zone_idx]) / ring_sizes_[zone_idx])), option_.czm.num_rings_each_zone[zone_idx] - 1);
            int sector_idx = std::min(static_cast<int>((theta / sector_sizes_[zone_idx])), option_.czm.num_sectors_each_zone[zone_idx] - 1);

            czm[zone_idx][ring_idx][sector_idx].points.emplace_back(pt);
        }
        else
        {
            cloud_nonground.push_back(pt);
        }
    }

    if (option_.verbose) {
        std::cout << "[ CZM ] Divides pointcloud into the concentric zone model" << std::endl;
    }   
        
}

#endif
