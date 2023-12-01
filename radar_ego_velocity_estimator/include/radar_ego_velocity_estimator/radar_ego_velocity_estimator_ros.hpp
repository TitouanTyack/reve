// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#ifndef RADAR_EGO_VELOCITY_ESTIMATOR_ROS
#define RADAR_EGO_VELOCITY_ESTIMATOR_ROS

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <angles/angles.h>
#include <radar_ego_velocity_estimator/data_types.hpp>
#include <radar_ego_velocity_estimator/simple_profiler.hpp>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_config_ros.hpp>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
// #include <radar_ego_velocity_estimator/RadarEgoVelocityEstimatorConfig.h>
// #include <radar_ego_velocity_estimator/ros_helper.h>

namespace reve
{
/**
 * @brief The RadarEgoVelocityEstimatorRos class provides a ROS interface for the RadarBodyVelocityEstimator
 */
class RadarEgoVelocityEstimatorRos: public rclcpp::Node
{
public:
  RadarEgoVelocityEstimatorRos()
  : rclcpp::Node("radar_ego_velocity_estimator_ros"){

    RadarEgoVelocityEstimatorConfig config_;
    config_.min_dist                           = this->declare_parameter("min_dist", 0.25);
    config_.max_dist                           = this->declare_parameter("max_dist", 100.0);
    config_.min_db                             = this->declare_parameter("min_db", 5.0);
    config_.elevation_thresh_deg               = this->declare_parameter("elevation_thresh_deg", 60.0);
    config_.azimuth_thresh_deg                 = this->declare_parameter("azimuth_thresh_deg", 60.0);
    config_.filter_min_z                       = this->declare_parameter("filter_min_z", -100.0);
    config_.filter_max_z                       = this->declare_parameter("filter_max_z", 100.0);
    config_.doppler_velocity_correction_factor = this->declare_parameter("doppler_velocity_correction_factor", 1.0);
    config_.thresh_zero_velocity               = this->declare_parameter("thresh_zero_velocity", 0.05);
    config_.allowed_outlier_percentage         = this->declare_parameter("allowed_outlier_percentage", 0.25);
    config_.sigma_zero_velocity_x              = this->declare_parameter("sigma_zero_velocity_x", 0.025);
    config_.sigma_zero_velocity_y              = this->declare_parameter("sigma_zero_velocity_y", 0.025);
    config_.sigma_zero_velocity_z              = this->declare_parameter("sigma_zero_velocity_z", 0.025);
    config_.sigma_offset_radar_x               = this->declare_parameter("sigma_offset_radar_x", 0.05);
    config_.sigma_offset_radar_y               = this->declare_parameter("sigma_offset_radar_y", 0.025);
    config_.sigma_offset_radar_z               = this->declare_parameter("sigma_offset_radar_z", 0.05);
    config_.max_sigma_x                        = this->declare_parameter("max_sigma_x", 0.2);
    config_.max_sigma_y                        = this->declare_parameter("max_sigma_y", 0.15);
    config_.max_sigma_z                        = this->declare_parameter("max_sigma_z", 0.2);
    config_.max_r_cond                         = this->declare_parameter("max_r_cond", 1.0e3);
    config_.use_cholesky_instead_of_bdcsvd     = this->declare_parameter("use_cholesky_instead_of_bdcsvd", true);
    config_.use_ransac                         = this->declare_parameter("use_ransac", true);
    config_.outlier_prob                       = this->declare_parameter("outlier_prob", 0.4);
    config_.success_prob                       = this->declare_parameter("success_prob", 0.9999);
    config_.N_ransac_points                    = this->declare_parameter("N_ransac_points", 3);
    config_.inlier_thresh                      = this->declare_parameter("inlier_thresh", 0.15);
    config_.use_odr                            = this->declare_parameter("use_odr", false);
    config_.sigma_v_d                          = this->declare_parameter("sigma_v_d", 0.125);
    config_.min_speed_odr                      = this->declare_parameter("min_speed_odr", 4.0);
    config_.model_noise_offset_deg             = this->declare_parameter("model_noise_offset_deg", 2.0);
    config_.model_noise_scale_deg              = this->declare_parameter("model_noise_scale_deg", 10.0);
    

    // reconfigure_server_.setCallback(boost::bind(&RadarEgoVelocityEstimatorRos::reconfigureCallback, this, _1, _2));

    run_without_trigger = this->declare_parameter("run_without_trigger",false);

    if(run_without_trigger)
      RCLCPP_WARN_STREAM(this->get_logger(), kPrefix << "Running without radar trigger");

    using std::placeholders::_1;
    sub_radar_scan_    = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "radar/scan", 10, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarScan, this, _1));
    sub_radar_trigger_ = this->create_subscription<std_msgs::msg::Header>(
      "radar/trigger", 10, std::bind(&RadarEgoVelocityEstimatorRos::callbackRadarTrigger, this, _1));
    
    pub_twist_                  = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);
    pub_twist_with_covaricance_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("pub_twist_with_covaricance", 10);
    pub_twist_ground_truth_     = this->create_publisher<geometry_msgs::msg::TwistStamped>("ground_truth/twist_radar", 10);
    reconfigureCallback(config_);

    RCLCPP_INFO(this->get_logger(), "radar_ego_velocity_estimator_ros node started");
  }
  protected:
  /**
   * @brief Pocesses a whole rosbag
   * @note Use sleep_ms to limit the processing speed
   * @param rosbag_path    full path to the rosbag
   * @param bag_start      start time for processing (skip the first bag_start seconds)
   * @param bag_duration   duration of processing (stop bag_duration after bag_start)
   * @param sleep_ms       sleep for these mulliseconds after each radar scan was processed
   */
  // void runFromRosbag(const std::string& rosbag_path,
  //                    const double bag_start,
  //                    const double bag_duration,
  //                    const double sleep_ms);
  /**
   * @brief Reconfigure callback
   */
  void reconfigureCallback(RadarEgoVelocityEstimatorConfig& config);

  /**
   * @brief Does the acutal processing using the RadarBodeyVelocityEstimator class
   * @param radar_scan     radar scan message
   * @param trigger_stamp  trigger time stamp of radar scan used to stamp the estimated velocity
   */
  void processRadarData(const sensor_msgs::msg::PointCloud2& radar_scan, const rclcpp::Time& trigger_stamp);

  /**
   * @brief Imu message callback, called by ros::spin (ros mode) or runFromRosbag
   */
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

  /**
   * @brief Radar scan message callback, alled by ros::spin (ros mode) or runFromRosbag
   */
  void callbackRadarScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& radar_scan_msg);

  /**
   * @brief Radar trigger header message callback, called by ros::spin (ros mode) or runFromRosbag
   */
  void callbackRadarTrigger(const std_msgs::msg::Header::ConstSharedPtr& trigger_msg);

  const std::string kPrefix = "[]: ";

  // parameters_cb reconfigure_server_;
    /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters parameters vector to look at
   */  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_cb_handle_;
  rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &parameters);

  RadarEgoVelocityEstimator estimator_;

  SimpleProfiler profiler;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_radar_scan_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_radar_trigger_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_with_covaricance_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_ground_truth_;

  bool run_without_trigger = false;

  std::mutex mutex_;
  std::unique_ptr<rclcpp::Time> trigger_stamp;
};

}  // namespace reve
#endif