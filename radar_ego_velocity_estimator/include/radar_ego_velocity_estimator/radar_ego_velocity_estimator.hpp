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

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>

#include <radar_ego_velocity_estimator/data_types.hpp>
#include <radar_ego_velocity_estimator/radar_point_cloud.hpp>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_config_ros.hpp>

#include <chrono>

// #include <radar_ego_velocity_estimator/ros_helper.hpp>

// #include <radar_ego_velocity_estimator/RadarEgoVelocityEstimatorConfig.h>

namespace reve
{
struct RadarEgoVelocityEstimatorIndices
{
  uint azimuth   = 0;
  uint elevation = 1;
  uint x_r       = 2;
  uint y_r       = 3;
  uint z_r       = 4;
  uint peak_db   = 5;
  uint r_x       = 6;
  uint r_y       = 7;
  uint r_z       = 8;
  uint v_d       = 9;
  uint noise_db  = 10;
};

class RadarEgoVelocityEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief RadarEgoVelocityEstimator constructor
   */
  RadarEgoVelocityEstimator() {}
  
  /**
   * @brief Reconfigure callback
   * @param config  has to contain RadarEgoVelocityEstimatorConfig
   * @return
   */
  void configure(RadarEgoVelocityEstimatorConfig& config);
  /**
   * @brief Estimates the radar ego velocity based on a single radar scan
   * @param[in] radar_scan_msg       radar scan
   * @param[out] v_r                 estimated radar ego velocity
   * @param[out] sigma_v_r           estimated sigmas of ego velocity
   * @param[out] inlier_radar_scan   inlier point cloud
   * @returns true if estimation successful
   */
  bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg, Vector3& v_r, Matrix3& P_v_r);
  bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg, Vector3& v_r, Vector3& sigma_v_r);
  bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Matrix3& P_v_r,
                sensor_msgs::msg::PointCloud2& inlier_radar_msg);
  bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Vector3& sigma_v_r,
                sensor_msgs::msg::PointCloud2& inlier_radar_msg);
  bool estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg,
                Vector3& v_r,
                Matrix3& P_v_r,
                pcl::PointCloud<RadarPointCloudType>& inlier_radar,
                const Matrix3& C_stab_r = Matrix3::Identity());

private:
  /**
   * @brief Implementation of the ransac based estimation
   * @param[in] radar_data          matrix of parsed radar scan --> see
   * RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param[out] inlier_idx_best    idices of inlier
   * @returns true if estimation successful
   */
  bool solve3DLsqRansac(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r, std::vector<uint>& inlier_idx_best);

  /**
   * @brief Estimates the radar ego velocity using all mesurements provided in
   * radar_data
   * @param[in] radar_data          matrix of parsed radar scan --> see
   * RadarEgoVelocityEstimatorIndices
   * @param[out] v_r                estimated radar ego velocity
   * @param[out] sigma_v_r          estimated sigmas of ego velocity
   * @param estimate_sigma          if true sigma will be estimated as well
   * @returns true if estimation successful
   */
  bool solve3DLsq(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r, bool estimate_sigma = true);

  bool solve3DOdr(const Matrix& radar_data, Vector3& v_r, Matrix3& P_v_r);
  /**
   * @brief Helper function which estiamtes the number of RANSAC iterations
   */
  void setRansacIter()
  {
    ransac_iter_ = uint((std::log(1.0 - config_.success_prob)) /
                        std::log(1.0 - std::pow(1.0 - config_.outlier_prob, config_.N_ransac_points)));
  }

  const std::string kPrefix = "[RadarEgoVelocityEstimator]: ";
  const RadarEgoVelocityEstimatorIndices idx_;

  RadarEgoVelocityEstimatorConfig config_;
  uint ransac_iter_ = 0;

  std::unique_ptr<std::chrono::steady_clock::time_point> timestamp_;
};
}  // namespace reve
