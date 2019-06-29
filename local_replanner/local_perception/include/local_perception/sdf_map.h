#ifndef _SDF_MAP_H

#define _SDF_MAP_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <iostream>
#include <tuple>
#include <queue>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <local_perception/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

class SDFMap
{
private:
  // data are saved in vector
  std::vector<double> occupancy_buffer, occupancy_buffer_neg, distance_buffer, distance_buffer_neg, distance_buffer_all,
      tmp_buffer1, tmp_buffer2;

  std::vector<double> occupancy_buffer_inflate_;

  // map property
  Eigen::Vector3d origin, map_size;
  Eigen::Vector3d min_range, max_range;  // map range in pos
  Eigen::Vector3i grid_size;             // map range in index
  double resolution, resolution_inv;
  Eigen::Vector3i min_vec, max_vec;  // the min and max updated range, unit is 1
  double truncated_distance = 20.0;

  Eigen::Vector3d last_fill_pt;

  inline bool isInMap(Eigen::Vector3d pos);
  inline void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
  inline void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  bool checkPeak(Eigen::Vector3d pos);

public:
  SDFMap()
  {
  }
  ~SDFMap()
  {
  }

  // occupancy management
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  // distance field management
  inline double getDistance(Eigen::Vector3d pos);
  inline double getDistance(Eigen::Vector3i id, int sign = 1);
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  // /inline void setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);

  void updateESDF3d();
  void getSliceESDF(const double height, const double res, Eigen::Vector4d range, vector<Eigen::Vector3d>& slice,
                    vector<Eigen::Vector3d>& grad,
                    int sign = 1);  // 1 pos, 2 neg, 3 combined

  /* ---------- try to fill local minima ---------- */
  vector<Eigen::Vector3d> findPeaks(const Eigen::Vector3d& pt);
  bool fillLocalMinima(const vector<Eigen::Vector3d>& peaks, Eigen::Vector3d& center, Eigen::Vector3d& scale);

  bool tryFillMinima(const Eigen::Vector3d& pt, Eigen::Vector3d& center, Eigen::Vector3d& cube_len);

  /* ---------- new ros wrapper ---------- */
  // public:
  void initMap(ros::NodeHandle& nh);

  void publishMap();

  void publishMapInflate();

  void publishESDF();

  void publishUpdateRange();

  void checkDist();

  bool hasDepthObservation()
  {
    return has_first_depth_;
  }

  typedef std::shared_ptr<SDFMap> Ptr;

private:
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

  void depthCallback(const sensor_msgs::ImageConstPtr& img);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);

  void projectDepthImage();

  void raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  void clearAndInflateLocalMap();

  int setCacheOccupancy(Eigen::Vector3d pos, int occ);

  ros::NodeHandle node_;

  /* ---------- message synchronization ---------- */
  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  int pose_type_;
  string input_data_type_;

  ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
  ros::Publisher map_pub_, esdf_pub_, test_pub_, map_inf_pub_, update_range_pub_;
  ros::Timer occ_timer_, esdf_timer_;

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Quaterniond camera_q_, last_camera_q_;

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  bool occ_need_update_, esdf_need_update_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  /* ============================== map fusion and esdf ============================== */
  /* ---------- local map fusion ---------- */
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  vector<int> cache_hit_, cache_all_;
  vector<int> cache_traverse_, cache_rayend_;
  int raycast_num_;

  double inflate_val_;

  queue<Eigen::Vector3i> cache_voxel_;

  /* ---------- esdf update ---------- */
  Eigen::Vector3i esdf_min_, esdf_max_;
  double esdf_inflate_;
  int local_map_margin_;
  double ground_z_;

  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;

  /* ---------- parameter, filtering and probability update ---------- */
  double cx_, cy_, fx_, fy_;
  Eigen::Vector3d sensor_range_;

  /* projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  double min_ray_length_, max_ray_length_;

  double slice_height_, cut_height_;
  bool show_esdf_time_, show_occ_time_;

  bool use_uniform_update_ = true;
  string frame_id_;

  Eigen::Vector3d* output_buffer;
  int output_points_cnt;
};

#endif