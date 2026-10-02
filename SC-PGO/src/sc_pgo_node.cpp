#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <optional>
#include <cstdlib>
#include <limits>

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
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "scancontext/Scancontext.h"

#include "sc_pgo/sc_pgo_node.hpp"

using namespace gtsam;

using std::cout;
using std::endl;

// ---------------------------------------------------------------------------
// file-local pure helpers (no class state)
// ---------------------------------------------------------------------------

static double toSec(const builtin_interfaces::msg::Time &stamp)
{
    const double sec = (double)stamp.sec;
    const double nsec = (double)stamp.nanosec;
    return (double)sec + 1e-9 * (double)nsec;
}

static std::string padZeros(int val, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

static gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D &p)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
} // Pose6DtoGTSAMPose3

static void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for (const auto &key_value : _estimates)
    {
        auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
        if (!p)
            continue;

        const Pose3 &pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

static Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom

static Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

// ---------------------------------------------------------------------------
// SCPGONode
// ---------------------------------------------------------------------------

SCPGONode::SCPGONode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("laserPGO", options)
{
    loadParams();

    // gtasm 因子图初始化
    ISAM2Params parameters;
    parameters.relinearizeThreshold = isam_relinearize_threshold_;
    parameters.relinearizeSkip = isam_relinearize_skip_;
    isam_ = std::make_unique<ISAM2>(parameters);
    initNoises();

    sc_manager_.setSCdistThres(sc_dist_thres_);
    sc_manager_.setMaximumRadius(sc_max_radius_);
    sc_manager_.setLoopTimeGap(loop_time_gap_);

    // 降采样设置
    down_size_filter_scancontext_.setLeafSize(scancontext_filter_size_, scancontext_filter_size_, scancontext_filter_size_);
    down_size_filter_icp_.setLeafSize(icp_filter_size_, icp_filter_size_, icp_filter_size_);

    // 地图可视化滤波尺寸
    down_size_filter_map_pgo_.setLeafSize(mapviz_filter_size_, mapviz_filter_size_, mapviz_filter_size_);

    // 数据记录
    sub_laser_cloud_full_res_ = create_subscription<sensor_msgs::msg::PointCloud2>(sub_lidar_topic_, 100, std::bind(&SCPGONode::laserCloudFullResHandler, this, std::placeholders::_1));
    sub_laser_odometry_ = create_subscription<nav_msgs::msg::Odometry>(sub_odom_topic_, 100, std::bind(&SCPGONode::laserOdometryHandler, this, std::placeholders::_1));
    sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(sub_gps_topic_, 100, std::bind(&SCPGONode::gpsHandler, this, std::placeholders::_1));

    // 输出
    pub_odom_aft_pgo_ = create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic_, 100);
    pub_odom_repub_verifier_ = create_publisher<nav_msgs::msg::Odometry>(pub_repub_odom_topic_, 100);
    pub_path_aft_pgo_ = create_publisher<nav_msgs::msg::Path>(pub_path_topic_, 100);
    pub_map_aft_pgo_ = create_publisher<sensor_msgs::msg::PointCloud2>(pub_map_topic_, 100);

    pub_loop_scan_local_ = create_publisher<sensor_msgs::msg::PointCloud2>(pub_loop_scan_topic_, 100);
    pub_loop_submap_local_ = create_publisher<sensor_msgs::msg::PointCloud2>(pub_loop_submap_topic_, 100);

    // 多线程，核心
    thread_pg_ = std::thread(&SCPGONode::process_pg, this);       // pose graph construction,  添加因子
    thread_lcd_ = std::thread(&SCPGONode::process_lcd, this);     // loop closure detection, 定时回环检测  scan context, 不做优化
    thread_icp_ = std::thread(&SCPGONode::process_icp, this);     // loop constraint calculation via icp, 定时 icp 获取因子图观测
    thread_isam_ = std::thread(&SCPGONode::process_isam, this);   // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added.
                                                                  // process isam 用于优化位姿
    thread_viz_map_ = std::thread(&SCPGONode::process_viz_map, this);   // visualization - map (low frequency because it is heavy)
    thread_viz_path_ = std::thread(&SCPGONode::process_viz_path, this); // visualization - path (high frequency)
}

SCPGONode::~SCPGONode()
{
    is_running_ = false;
    if (thread_pg_.joinable())
        thread_pg_.join();
    if (thread_lcd_.joinable())
        thread_lcd_.join();
    if (thread_icp_.joinable())
        thread_icp_.join();
    if (thread_isam_.joinable())
        thread_isam_.join();
    if (thread_viz_map_.joinable())
        thread_viz_map_.join();
    if (thread_viz_path_.joinable())
        thread_viz_path_.join();
}

void SCPGONode::loadParams()
{
    // 输出
    declare_parameter("save_directory", std::string("/"));
    save_directory_ = get_parameter("save_directory").get_parameter_value().get<std::string>();
    pg_kitti_format_ = save_directory_ + "optimized_poses.txt";
    odom_kitti_format_ = save_directory_ + "odom_poses.txt";
    pg_time_save_stream_ = std::fstream(save_directory_ + "times.txt", std::fstream::out);
    pg_time_save_stream_.precision(std::numeric_limits<double>::max_digits10);
    pg_scans_directory_ = save_directory_ + "Scans/";
    auto unused = system((std::string("exec rm -r ") + pg_scans_directory_).c_str());
    unused = system((std::string("mkdir -p ") + pg_scans_directory_).c_str());

    // 关键帧
    declare_parameter("keyframe_meter_gap", 2.0); // pose assignment every k m move
    keyframe_meter_gap_ = get_parameter("keyframe_meter_gap").get_parameter_value().get<double>();
    declare_parameter("keyframe_deg_gap", 10.0); // pose assignment every k deg rot
    keyframe_deg_gap_ = get_parameter("keyframe_deg_gap").get_parameter_value().get<double>();
    keyframe_rad_gap_ = deg2rad(keyframe_deg_gap_);

    // scan context
    declare_parameter("sc_dist_thres", 0.2);
    sc_dist_thres_ = get_parameter("sc_dist_thres").get_parameter_value().get<double>();
    declare_parameter("sc_max_radius", 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor
    sc_max_radius_ = get_parameter("sc_max_radius").get_parameter_value().get<double>();
    declare_parameter("loop_time_gap", 30.0); // a loop candidate keyframe must be >= this many seconds older than the query
    loop_time_gap_ = get_parameter("loop_time_gap").get_parameter_value().get<double>();
    declare_parameter("scancontext_filter_size", 0.4);
    scancontext_filter_size_ = static_cast<float>(get_parameter("scancontext_filter_size").get_parameter_value().get<double>());

    // gps
    declare_parameter("use_gps", true);
    use_gps_ = get_parameter("use_gps").get_parameter_value().get<bool>();
    declare_parameter("gps_time_tolerance", 0.1);
    gps_time_tolerance_ = get_parameter("gps_time_tolerance").get_parameter_value().get<double>();

    // 噪声模型
    declare_parameter("prior_noise_xyz", 1e-12);
    prior_noise_xyz_ = get_parameter("prior_noise_xyz").get_parameter_value().get<double>();
    declare_parameter("prior_noise_rpy", 1e-12);
    prior_noise_rpy_ = get_parameter("prior_noise_rpy").get_parameter_value().get<double>();
    declare_parameter("odom_noise_xyz", 1e-6);
    odom_noise_xyz_ = get_parameter("odom_noise_xyz").get_parameter_value().get<double>();
    declare_parameter("odom_noise_rpy", 1e-4);
    odom_noise_rpy_ = get_parameter("odom_noise_rpy").get_parameter_value().get<double>();
    declare_parameter("loop_noise_score", 0.5);
    loop_noise_score_ = get_parameter("loop_noise_score").get_parameter_value().get<double>();
    declare_parameter("gps_noise_xy", 1.0e9);
    gps_noise_xy_ = get_parameter("gps_noise_xy").get_parameter_value().get<double>();
    declare_parameter("gps_noise_altitude", 250.0);
    gps_noise_altitude_ = get_parameter("gps_noise_altitude").get_parameter_value().get<double>();

    // isam2
    declare_parameter("isam_relinearize_threshold", 0.01);
    isam_relinearize_threshold_ = get_parameter("isam_relinearize_threshold").get_parameter_value().get<double>();
    declare_parameter("isam_relinearize_skip", 1);
    isam_relinearize_skip_ = get_parameter("isam_relinearize_skip").get_parameter_value().get<int>();

    // icp
    declare_parameter("icp_filter_size", 0.4);
    icp_filter_size_ = static_cast<float>(get_parameter("icp_filter_size").get_parameter_value().get<double>());
    declare_parameter("icp_max_correspondence_distance", 150.0);
    icp_max_correspondence_distance_ = get_parameter("icp_max_correspondence_distance").get_parameter_value().get<double>();
    declare_parameter("icp_max_iterations", 100);
    icp_max_iterations_ = get_parameter("icp_max_iterations").get_parameter_value().get<int>();
    declare_parameter("icp_transformation_epsilon", 1e-6);
    icp_transformation_epsilon_ = get_parameter("icp_transformation_epsilon").get_parameter_value().get<double>();
    declare_parameter("icp_euclidean_fitness_epsilon", 1e-6);
    icp_euclidean_fitness_epsilon_ = get_parameter("icp_euclidean_fitness_epsilon").get_parameter_value().get<double>();
    declare_parameter("icp_ransac_iterations", 0);
    icp_ransac_iterations_ = get_parameter("icp_ransac_iterations").get_parameter_value().get<int>();
    declare_parameter("icp_fitness_score_threshold", 0.3);
    icp_fitness_score_threshold_ = static_cast<float>(get_parameter("icp_fitness_score_threshold").get_parameter_value().get<double>());
    declare_parameter("history_keyframe_search_num", 25);
    history_keyframe_search_num_ = get_parameter("history_keyframe_search_num").get_parameter_value().get<int>();

    // 频率
    declare_parameter("loop_closure_frequency", 1.0);
    loop_closure_frequency_ = get_parameter("loop_closure_frequency").get_parameter_value().get<double>();
    declare_parameter("isam_frequency", 1.0);
    isam_frequency_ = get_parameter("isam_frequency").get_parameter_value().get<double>();
    declare_parameter("viz_path_frequency", 10.0);
    viz_path_frequency_ = get_parameter("viz_path_frequency").get_parameter_value().get<double>();
    declare_parameter("viz_map_frequency", 0.1);
    viz_map_frequency_ = get_parameter("viz_map_frequency").get_parameter_value().get<double>();
    declare_parameter("viz_map_skip_frames", 2);
    viz_map_skip_frames_ = get_parameter("viz_map_skip_frames").get_parameter_value().get<int>();

    // 线程核数
    declare_parameter("num_cores_transform", 16);
    num_cores_transform_ = get_parameter("num_cores_transform").get_parameter_value().get<int>();
    declare_parameter("num_cores_icp", 8);
    num_cores_icp_ = get_parameter("num_cores_icp").get_parameter_value().get<int>();

    // 地图可视化
    declare_parameter("mapviz_filter_size", 0.4);
    mapviz_filter_size_ = static_cast<float>(get_parameter("mapviz_filter_size").get_parameter_value().get<double>());

    // 话题名
    declare_parameter("sub_lidar_topic", std::string("/cloud_registered_body"));
    sub_lidar_topic_ = get_parameter("sub_lidar_topic").get_parameter_value().get<std::string>();
    declare_parameter("sub_odom_topic", std::string("/aft_mapped_to_init"));
    sub_odom_topic_ = get_parameter("sub_odom_topic").get_parameter_value().get<std::string>();
    declare_parameter("sub_gps_topic", std::string("/gps/fix"));
    sub_gps_topic_ = get_parameter("sub_gps_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_odom_topic", std::string("/aft_pgo_odom"));
    pub_odom_topic_ = get_parameter("pub_odom_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_repub_odom_topic", std::string("/repub_odom"));
    pub_repub_odom_topic_ = get_parameter("pub_repub_odom_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_path_topic", std::string("/aft_pgo_path"));
    pub_path_topic_ = get_parameter("pub_path_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_map_topic", std::string("/aft_pgo_map"));
    pub_map_topic_ = get_parameter("pub_map_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_loop_scan_topic", std::string("/loop_scan_local"));
    pub_loop_scan_topic_ = get_parameter("pub_loop_scan_topic").get_parameter_value().get<std::string>();
    declare_parameter("pub_loop_submap_topic", std::string("/loop_submap_local"));
    pub_loop_submap_topic_ = get_parameter("pub_loop_submap_topic").get_parameter_value().get<std::string>();
}

void SCPGONode::initNoises()
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << prior_noise_xyz_, prior_noise_xyz_, prior_noise_xyz_, prior_noise_rpy_, prior_noise_rpy_, prior_noise_rpy_;
    prior_noise_ = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << odom_noise_xyz_, odom_noise_xyz_, odom_noise_xyz_, odom_noise_rpy_, odom_noise_rpy_, odom_noise_rpy_;
    odom_noise_ = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = loop_noise_score_; // constant is ok...
    gtsam::Vector robustNoiseVector6(6);       // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

    double bigNoiseTolerentToXY = gps_noise_xy_;                                                  // 1e9
    double gpsAltitudeNoiseScore = gps_noise_altitude_;                                           // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3);                                                          // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore;     // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robust_gps_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));

} // initNoises

void SCPGONode::laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr _laserOdometry)
{
    mtx_buf_.lock();
    // RCLCPP_INFO(rclcpp::get_logger("scpgo"),"laserOdometryHandler CALLBACK");
    odometry_buf_.push(_laserOdometry);
    mtx_buf_.unlock();
} // laserOdometryHandler

void SCPGONode::laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _laserCloudFullRes)
{
    mtx_buf_.lock();
    // std::cout << "laserCloudFullResHandler CALLBACK" <<"\n";
    // RCLCPP_INFO(rclcpp::get_logger("scpgo"),"laserCloudFullResHandler CALLBACK");
    full_res_buf_.push(_laserCloudFullRes);
    mtx_buf_.unlock();
} // laserCloudFullResHandler

void SCPGONode::gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr _gps)
{
    if (use_gps_)
    {
        mtx_buf_.lock();
        gps_buf_.push(_gps);
        mtx_buf_.unlock();
    }
} // gpsHandler

pcl::PointCloud<PointType>::Ptr SCPGONode::local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D &tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = num_cores_transform_;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void SCPGONode::pubPath()
{
    // pub odom and path
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    mtx_kf_.lock();
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx = 0; node_idx < recent_idx_updated_; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D &pose_est = keyframe_poses_updated_.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "camera_init";
        odomAftPGOthis.child_frame_id = "/aft_pgo";
        odomAftPGOthis.header.stamp = rclcpp::Time(keyframe_times_.at(node_idx) * 1e9);
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        // geometry_msgs::msg::Quaternion q_msg;
        // tf2::convert(q, q_msg);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mtx_kf_.unlock();
    pub_odom_aft_pgo_->publish(odomAftPGO); // last pose
    pub_path_aft_pgo_->publish(pathAftPGO);  // poses

    if (!tf_broadcaster_)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    // tf::Transform transform;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftPGO.header.stamp;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "/aft_pgo";
    transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transform.transform.rotation.x = odomAftPGO.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftPGO.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftPGO.pose.pose.orientation.z;
    transform.transform.rotation.w = odomAftPGO.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform);
} // pubPath

void SCPGONode::updatePoses()
{
    mtx_kf_.lock();
    for (int node_idx = 0; node_idx < int(isam_current_estimate_.size()); node_idx++)
    {
        Pose6D &p = keyframe_poses_updated_[node_idx];
        p.x = isam_current_estimate_.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isam_current_estimate_.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isam_current_estimate_.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isam_current_estimate_.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isam_current_estimate_.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isam_current_estimate_.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mtx_kf_.unlock();

    mtx_recent_pose_.lock();
    const gtsam::Pose3 &lastOptimizedPose = isam_current_estimate_.at<gtsam::Pose3>(int(isam_current_estimate_.size()) - 1);
    recent_optimized_x_ = lastOptimizedPose.translation().x();
    recent_optimized_y_ = lastOptimizedPose.translation().y();

    recent_idx_updated_ = int(keyframe_poses_updated_.size()) - 1;

    mtx_recent_pose_.unlock();
} // updatePoses

void SCPGONode::runISAM2opt()
{
    // called when a variable added
    isam_->update(gt_sam_graph_, initial_estimate_);
    isam_->update();

    gt_sam_graph_.resize(0);
    initial_estimate_.clear();

    isam_current_estimate_ = isam_->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr SCPGONode::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
        transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw());

    int numberOfCores = num_cores_icp_;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom->x + transCur(0, 1) * pointFrom->y + transCur(0, 2) * pointFrom->z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom->x + transCur(1, 1) * pointFrom->y + transCur(1, 2) * pointFrom->z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom->x + transCur(2, 1) * pointFrom->y + transCur(2, 2) * pointFrom->z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void SCPGONode::loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &submap_size, const int &root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframe_laser_clouds_.size()))
            continue;

        mtx_kf_.lock();
        *nearKeyframes += *local2global(keyframe_laser_clouds_[keyNear], keyframe_poses_updated_[root_idx]);
        mtx_kf_.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    down_size_filter_icp_.setInputCloud(nearKeyframes);
    down_size_filter_icp_.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud

std::optional<gtsam::Pose3> SCPGONode::doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx)
{
    // parse pointclouds
    int historyKeyframeSearchNum = history_keyframe_search_num_; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

    // loop verification
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pub_loop_scan_local_->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pub_loop_submap_local_->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
    icp.setRANSACIterations(icp_ransac_iterations_);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    float loopFitnessScoreThreshold = icp_fitness_score_threshold_; // user parameter but fixed low value is safe.
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    }
    else
    {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void SCPGONode::saveOdometryVerticesKITTIformat(const std::string &_filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for (const auto &_pose6d : keyframe_poses_)
    {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void SCPGONode::process_pg()
{
    while (is_running_ && rclcpp::ok())
    {
        while (!odometry_buf_.empty() && !full_res_buf_.empty())
        {
            //
            // pop and check keyframe is or not
            //
            mtx_buf_.lock();
            while (!odometry_buf_.empty() && toSec(odometry_buf_.front()->header.stamp) < toSec(full_res_buf_.front()->header.stamp))
                odometry_buf_.pop();
            if (odometry_buf_.empty())
            {
                mtx_buf_.unlock();
                break;
            }

            // Time equal check
            time_laser_odometry_ = toSec(odometry_buf_.front()->header.stamp);
            time_laser_ = toSec(full_res_buf_.front()->header.stamp);
            // TODO

            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*full_res_buf_.front(), *thisKeyFrame);
            full_res_buf_.pop();

            Pose6D pose_curr = getOdom(odometry_buf_.front());
            odometry_buf_.pop();

            // find nearest gps
            double eps = gps_time_tolerance_; // find a gps topioc arrived within eps second
            while (!gps_buf_.empty())
            {
                auto thisGPS = gps_buf_.front();
                auto thisGPSTime = toSec(thisGPS->header.stamp);
                if (abs(thisGPSTime - time_laser_odometry_) < eps)
                {
                    curr_gps_ = thisGPS;
                    has_gps_for_this_kf_ = true;
                    break;
                }
                else
                {
                    has_gps_for_this_kf_ = false;
                }
                gps_buf_.pop();
            }
            mtx_buf_.unlock();

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            //
            odom_pose_prev_ = odom_pose_curr_;
            odom_pose_curr_ = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev_, odom_pose_curr_); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z); // note: absolute value.
            translation_accumulated_ += delta_translation;
            rotation_accumulated_ += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

            if (translation_accumulated_ > keyframe_meter_gap_ || rotation_accumulated_ > keyframe_rad_gap_)
            {
                is_now_keyframe_ = true;
                translation_accumulated_ = 0.0; // reset
                rotation_accumulated_ = 0.0;    // reset
            }
            else
            {
                is_now_keyframe_ = false;
            }

            if (!is_now_keyframe_)
                continue;

            if (!gps_offset_initialized_)
            {
                if (has_gps_for_this_kf_)
                { // if the very first frame
                    gps_altitude_init_offset_ = curr_gps_->altitude;
                    gps_offset_initialized_ = true;
                }
            }

            //
            // Save data and Add consecutive node
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            down_size_filter_scancontext_.setInputCloud(thisKeyFrame);
            down_size_filter_scancontext_.filter(*thisKeyFrameDS);

            mtx_kf_.lock();
            keyframe_laser_clouds_.push_back(thisKeyFrameDS);
            keyframe_poses_.push_back(pose_curr);
            keyframe_poses_updated_.push_back(pose_curr); // init
            keyframe_times_.push_back(time_laser_odometry_);

            sc_manager_.makeAndSaveScancontextAndKeys(*thisKeyFrameDS, time_laser_odometry_);

            mtx_kf_.unlock();

            const int prev_node_idx = keyframe_poses_.size() - 2;
            const int curr_node_idx = keyframe_poses_.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if (!gt_sam_graph_made_ /* prior node */)
            {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframe_poses_.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtx_posegraph_.lock();
                {
                    // prior factor
                    gt_sam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, prior_noise_));
                    initial_estimate_.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();
                }
                mtx_posegraph_.unlock();

                gt_sam_graph_made_ = true;

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            }
            else /* consecutive node (and odom factor) after the prior added */
            {    // == keyframePoses.size() > 1
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframe_poses_.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframe_poses_.at(curr_node_idx));

                mtx_posegraph_.lock();
                {
                    // odom factor
                    gt_sam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odom_noise_));

                    // gps factor
                    if (has_gps_for_this_kf_)
                    {
                        double curr_altitude_offseted = curr_gps_->altitude - gps_altitude_init_offset_;
                        mtx_recent_pose_.lock();
                        gtsam::Point3 gpsConstraint(recent_optimized_x_, recent_optimized_y_, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set)
                        mtx_recent_pose_.unlock();
                        gt_sam_graph_.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robust_gps_noise_));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initial_estimate_.insert(curr_node_idx, poseTo);
                    // runISAM2opt();
                }
                mtx_posegraph_.unlock();

                if (curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pg_scans_directory_ + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan
            pg_time_save_stream_ << time_laser_ << std::endl;                                            // path
        }

        // ps.
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void SCPGONode::performSCLoopClosure()
{
    // do not try too early: need at least one keyframe old enough (>= loop_time_gap_) to be a loop candidate
    mtx_kf_.lock();
    bool has_old_enough_kf = keyframe_times_.size() >= 2 &&
                             (keyframe_times_.back() - keyframe_times_.front() >= loop_time_gap_);
    mtx_kf_.unlock();
    if (!has_old_enough_kf)
        return;
    // scan context 检测回环
    auto detectResult = sc_manager_.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    // 如果检测到有两帧回环就记录下来
    if (SCclosestHistoryFrameID != -1)
    {
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframe_poses_.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mtx_buf_.lock();
        sc_loop_icp_buf_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mtx_buf_.unlock();
    }
} // performSCLoopClosure

void SCPGONode::process_lcd()
{
    float loopClosureFrequency = static_cast<float>(loop_closure_frequency_); // can change
    rclcpp::Rate rate(loopClosureFrequency);
    while (is_running_ && rclcpp::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void SCPGONode::process_icp()
{
    while (is_running_ && rclcpp::ok())
    {
        while (!sc_loop_icp_buf_.empty())
        {
            if (sc_loop_icp_buf_.size() > 30)
            {
                RCLCPP_WARN(get_logger(), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mtx_buf_.lock();
            std::pair<int, int> loop_idx_pair = sc_loop_icp_buf_.front();
            sc_loop_icp_buf_.pop();
            mtx_buf_.unlock();

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if (relative_pose_optional)
            {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtx_posegraph_.lock();
                gt_sam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robust_loop_noise_));
                // runISAM2opt();
                mtx_posegraph_.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void SCPGONode::process_viz_path()
{
    float hz = static_cast<float>(viz_path_frequency_);
    rclcpp::Rate rate(hz);
    while (is_running_ && rclcpp::ok())
    {
        rate.sleep();
        if (recent_idx_updated_ > 1)
        {
            pubPath();
        }
    }
}

void SCPGONode::process_isam()
{
    float hz = static_cast<float>(isam_frequency_);
    rclcpp::Rate rate(hz);
    while (is_running_ && rclcpp::ok())
    {
        rate.sleep();
        if (gt_sam_graph_made_)
        {
            mtx_posegraph_.lock();
            runISAM2opt();
            cout << "running isam2 optimization ..." << endl;
            mtx_posegraph_.unlock();

            saveOptimizedVerticesKITTIformat(isam_current_estimate_, pg_kitti_format_); // pose
            saveOdometryVerticesKITTIformat(odom_kitti_format_);                        // pose
        }
    }
}

void SCPGONode::pubMap()
{
    int SKIP_FRAMES = viz_map_skip_frames_; // sparse map visulalization to save computations
    int counter = 0;

    laser_cloud_map_pgo_->clear();

    mtx_kf_.lock();
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx = 0; node_idx < recent_idx_updated_; node_idx++)
    {
        if (counter % SKIP_FRAMES == 0)
        {
            *laser_cloud_map_pgo_ += *local2global(keyframe_laser_clouds_[node_idx], keyframe_poses_updated_[node_idx]);
        }
        counter++;
    }
    mtx_kf_.unlock();

    down_size_filter_map_pgo_.setInputCloud(laser_cloud_map_pgo_);
    down_size_filter_map_pgo_.filter(*laser_cloud_map_pgo_);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laser_cloud_map_pgo_, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    pub_map_aft_pgo_->publish(laserCloudMapPGOMsg);
}

void SCPGONode::process_viz_map()
{
    float vizmapFrequency = static_cast<float>(viz_map_frequency_); // 0.1 means run onces every 10s
    rclcpp::Rate rate(vizmapFrequency);
    while (is_running_ && rclcpp::ok())
    {
        rate.sleep();
        if (recent_idx_updated_ > 1)
        {
            pubMap();
        }
    }
} // pointcloud_viz
