#ifndef CASCADECONTROLLERNODE_HPP
#define CASCADECONTROLLERNODE_HPP

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

#include <sensor_msgs/Joy.h> //! @note for dji_sdk controller command

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>

#include <chrono>
#include <pthread.h>
#include <queue>

#include <cascade_controller/paramsConfig.h>
#include <dynamic_reconfigure/server.h>

#include "cascadecontrol.hpp"
#include "frequencyalign.hpp"

class CascadeControllerNode : public nodelet::Nodelet
{
public:
  CascadeControllerNode();
  virtual ~CascadeControllerNode();

public:
  virtual void onInit();

public:
  //! @note callbacks
  void enable_callback(const std_msgs::Bool::ConstPtr& enableSwitch);

  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void odom_correction_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void flight_status_callback(const std_msgs::UInt8::ConstPtr& flightstatus);
  void imu_acc_world_callback(const sensor_msgs::Imu::ConstPtr& imu);
  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);
  void view_callback(const geometry_msgs::Point::ConstPtr& view);
  void thrust_callback(const std_msgs::Float64::ConstPtr& force);
  void gain_callback(const std_msgs::Float64MultiArray::ConstPtr& data);
  void position_command_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void pid_param_callback(cascade_controller::paramsConfig& config,
                          uint32_t                          level);

public:
  //! @note getters
  double getMass() const;

public:
  //! @note setters
  void setMass(double value);

  Eigen::Matrix3d getMatrixParam(ros::NodeHandle& nh,
                                 Eigen::Matrix3d& value_default,
                                 std::string      name);

private:
  void update_position(const Eigen::Vector3d& data);
  void update_velocity(const Eigen::Vector3d& data);
  void update_acceleration(const Eigen::Vector3d& data);

  bool feed_position();
  bool feed_velocity();
  bool feed_acceleration();

private:
  double force2Percentage(const double force) const;
  void thrustObserver(mocka::State state, double djiThrust);

private:
  mocka::CascadeController* ctrl;

  bool enable;
  bool insky;
  bool debug;
  bool debug2;

  //! @todo remove such a dirty code
  bool firstObserve;
  int  dirtyCounter;

  std::queue<Eigen::Vector3d> unusedPosition;
  std::queue<Eigen::Vector3d> unusedVelocity;
  std::queue<Eigen::Vector3d> unusedAcceleration;

  double lastIntergrade;

  //! @todo remove below
  typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
  typedef std::chrono::duration<double>                      DT;

  mocka::FrequencyAlign alignPV;
  TimePoint             lastPositionTime;
  Eigen::Vector3d       lastPosition;
  mocka::FrequencyAlign alignVA;
  TimePoint             lastVelocityTime;
  Eigen::Vector3d       lastVelocity;
  //! @todo remove above

  ros::Publisher so3_command_pub;
  ros::Publisher attitude_command_pub;
  ros::Publisher palstance_command_pub;
  ros::Publisher manual_command_pub;
  ros::Publisher translation_error_pub;

  ros::Publisher debug_velocity_error_pub;
  ros::Publisher debug_acceleration_desire_pub;
  ros::Publisher debug_euler_angles_pub;
  ros::Publisher debug_maxthrust_pub;
  ros::Publisher debug_mass_pub;
  ros::Publisher debug_fc_euler_angles_pub;
  ros::Publisher debug_odom_euler_angles_pub;

  ros::Publisher debug_dpe_pub;
  ros::Publisher debug_dve_pub;
  ros::Publisher debug_input_a_pub;
  ros::Publisher debug_feed_p_pub;
  ros::Publisher debug_feed_v_pub;
  ros::Publisher debug_feed_a_pub;
  ros::Publisher debug_align_p_pub;
  ros::Publisher debug_align_v_pub;

  ros::Subscriber enable_control_sub;
  ros::Subscriber flightstatus_sub;

  ros::Subscriber position_command_sub;
  ros::Subscriber odometry_command_sub;

  ros::Subscriber odometry_sub;      // pose velocity quaternion
  ros::Subscriber imu_acc_world_sub; // acc palstance quaternion
  ros::Subscriber imu_sub;           // acc palstance quaternion

  ros::Subscriber view_sub;
  ros::Subscriber thrust_sub;
  ros::Subscriber gain_sub;

  pthread_mutex_t correctionMutex;

  Eigen::Vector3d    feedVelocity;
  Eigen::Vector3d    feedAcceleration;
  Eigen::Quaterniond odomQuaternion;
  Eigen::Quaterniond fcQuaternion;
  //! @note use error Quaternion to fix all rotation error is imposible,
  //! due to FC's Quaternion is not good, so we just fix yaw part
  Eigen::Quaterniond errorQuaternion; // FC internal to Odom feed
  bool               resetQuaternion;
  bool               isSimulation;

  Eigen::Vector3d    tempViewpoint;
  Eigen::Vector3d    correctionPosition;
  Eigen::Vector3d    correctionVelocity;
  Eigen::Quaterniond correctionQuaternion;
  Eigen::Vector3d    correctionPalstance;
  double             correctionThrust;

  //! @so3 command info
  std::string frame_id;
  double      mass;
  double      maxThrust; // unit is m/s^2
  double      current_yaw;
  double      kR[3];
  double      kOm[3];
  double      corrections[3];

  dynamic_reconfigure::Server<cascade_controller::paramsConfig> server;
  dynamic_reconfigure::Server<cascade_controller::paramsConfig>::CallbackType f;
};

#endif // CASCADECONTROLLERNODE_HPP
