#pragma once

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <tf2_ros/transform_broadcaster.h>

#include "aloam_velodyne/common.h"
#include "scancontext/Scancontext.h"

class SCPGONode : public rclcpp::Node
{
public:
    explicit SCPGONode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~SCPGONode() override;

private:
    // ----------------------------------------------------------------------
    // parameters (all loaded from YAML in loadParams())
    // ----------------------------------------------------------------------
    // output
    std::string save_directory_;
    std::string pg_kitti_format_, pg_scans_directory_, odom_kitti_format_;

    // keyframe
    double keyframe_meter_gap_;
    double keyframe_deg_gap_, keyframe_rad_gap_;

    // scan context
    double sc_dist_thres_, sc_max_radius_;
    double loop_time_gap_;
    float scancontext_filter_size_;

    // gps
    bool use_gps_;
    double gps_time_tolerance_;

    // noise models
    double prior_noise_xyz_, prior_noise_rpy_;
    double odom_noise_xyz_, odom_noise_rpy_;
    double loop_noise_score_;
    double gps_noise_xy_, gps_noise_altitude_;

    // isam2
    double isam_relinearize_threshold_;
    int isam_relinearize_skip_;

    // icp
    float icp_filter_size_;
    double icp_max_correspondence_distance_;
    int icp_max_iterations_;
    double icp_transformation_epsilon_;
    double icp_euclidean_fitness_epsilon_;
    int icp_ransac_iterations_;
    float icp_fitness_score_threshold_;
    int history_keyframe_search_num_;

    // frequencies (Hz)
    double loop_closure_frequency_;
    double isam_frequency_;
    double viz_path_frequency_;
    double viz_map_frequency_;
    int viz_map_skip_frames_;

    // threads
    int num_cores_transform_;
    int num_cores_icp_;

    // map viz
    float mapviz_filter_size_;

    // topics
    std::string sub_lidar_topic_, sub_odom_topic_, sub_gps_topic_;
    std::string pub_odom_topic_, pub_repub_odom_topic_, pub_path_topic_;
    std::string pub_map_topic_, pub_loop_scan_topic_, pub_loop_submap_topic_;

    // ----------------------------------------------------------------------
    // state
    // ----------------------------------------------------------------------
    double translation_accumulated_ = 1000000.0; // large value means must add the first given frame.
    double rotation_accumulated_ = 1000000.0;    // large value means must add the first given frame.
    bool is_now_keyframe_ = false;

    Pose6D odom_pose_prev_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose6D odom_pose_curr_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> full_res_buf_;
    std::queue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gps_buf_;
    std::queue<std::pair<int, int>> sc_loop_icp_buf_;

    std::mutex mtx_buf_;
    std::mutex mtx_kf_;

    double time_laser_odometry_ = 0.0;
    double time_laser_ = 0.0;

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_laser_clouds_;
    std::vector<Pose6D> keyframe_poses_;
    std::vector<Pose6D> keyframe_poses_updated_;
    std::vector<double> keyframe_times_;
    int recent_idx_updated_ = 0;

    gtsam::NonlinearFactorGraph gt_sam_graph_;
    bool gt_sam_graph_made_ = false;
    gtsam::Values initial_estimate_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::Values isam_current_estimate_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;
    gtsam::noiseModel::Base::shared_ptr robust_gps_noise_;

    pcl::VoxelGrid<PointType> down_size_filter_scancontext_;
    SCManager sc_manager_;

    pcl::VoxelGrid<PointType> down_size_filter_icp_;
    std::mutex mtx_posegraph_;
    std::mutex mtx_recent_pose_;

    pcl::PointCloud<PointType>::Ptr laser_cloud_map_pgo_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> down_size_filter_map_pgo_;

    sensor_msgs::msg::NavSatFix::ConstSharedPtr curr_gps_;
    bool has_gps_for_this_kf_ = false;
    bool gps_offset_initialized_ = false;
    double gps_altitude_init_offset_ = 0.0;
    double recent_optimized_x_ = 0.0;
    double recent_optimized_y_ = 0.0;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_map_aft_pgo_, pub_loop_scan_local_, pub_loop_submap_local_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_odom_aft_pgo_, pub_odom_repub_verifier_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pub_path_aft_pgo_;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> sub_laser_cloud_full_res_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_laser_odometry_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> sub_gps_;

    std::fstream pg_time_save_stream_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::atomic<bool> is_running_{true};
    std::thread thread_pg_, thread_lcd_, thread_icp_, thread_isam_, thread_viz_map_, thread_viz_path_;

    // ----------------------------------------------------------------------
    // setup
    // ----------------------------------------------------------------------
    void loadParams();
    void initNoises();

    // ----------------------------------------------------------------------
    // subscription callbacks
    // ----------------------------------------------------------------------
    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr _laserOdometry);
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _laserCloudFullRes);
    void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr _gps);

    // ----------------------------------------------------------------------
    // helpers
    // ----------------------------------------------------------------------
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D &tf);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn);
    void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &submap_size, const int &root_idx);
    std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx);
    void saveOdometryVerticesKITTIformat(const std::string &_filename);

    // ----------------------------------------------------------------------
    // pose graph / output
    // ----------------------------------------------------------------------
    void runISAM2opt();
    void updatePoses();
    void performSCLoopClosure();
    void pubPath();
    void pubMap();

    // ----------------------------------------------------------------------
    // processing threads
    // ----------------------------------------------------------------------
    void process_pg();
    void process_lcd();
    void process_icp();
    void process_isam();
    void process_viz_path();
    void process_viz_map();
};
