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

#include <random>
#include <algorithm>
#include <angles/angles.h>

// #include <radar_ego_velocity_estimator/ros_helper.hpp>
#include <radar_ego_velocity_estimator/math_helper.hpp>
#include <radar_ego_velocity_estimator/radar_body_velocity_estimator.hpp>

using namespace reve;

RadarBodyVelocityEstimator::RadarBodyVelocityEstimator(rclcpp::Node node, const bool load_param_without_reconfigure)
{
  bool success = true;

  Vector3 l_b_r;
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "l_b_r_x", l_b_r.x());
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "l_b_r_y", l_b_r.y());
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "l_b_r_z", l_b_r.z());

  Quaternion q_b_r;
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "q_b_r_w", q_b_r.w());
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "q_b_r_x", q_b_r.x());
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "q_b_r_y", q_b_r.y());
  success &= getRosParameter(node, kPrefix, RosParameterType::Required, "q_b_r_z", q_b_r.z());

  if (load_param_without_reconfigure)
  {
    // clang-format off
    RadarEgoVelocityEstimatorConfig config_;

    config_.min_dist                           = node.declare_parameter("min_dist", 0.25);
    config_.max_dist                           = node.declare_parameter("max_dist", 100.0);
    config_.min_db                             = node.declare_parameter("min_db", 5.0);
    config_.elevation_thresh_deg               = node.declare_parameter("elevation_thresh_deg", 60.0);
    config_.azimuth_thresh_deg                 = node.declare_parameter("azimuth_thresh_deg", 60.0);
    config_.filter_min_z                       = node.declare_parameter("filter_min_z", -100.0);
    config_.filter_max_z                       = node.declare_parameter("filter_max_z", 100.0);
    config_.doppler_velocity_correction_factor = node.declare_parameter("doppler_velocity_correction_factor", 1.0);
    config_.thresh_zero_velocity               = node.declare_parameter("thresh_zero_velocity", 0.05);
    config_.allowed_outlier_percentage         = node.declare_parameter("allowed_outlier_percentage", 0.25);
    config_.sigma_zero_velocity_x              = node.declare_parameter("sigma_zero_velocity_x", 0.025);
    config_.sigma_zero_velocity_y              = node.declare_parameter("sigma_zero_velocity_y", 0.025);
    config_.sigma_zero_velocity_z              = node.declare_parameter("sigma_zero_velocity_z", 0.025);
    config_.sigma_offset_radar_x               = node.declare_parameter("max_sigma_x", 0.2);
    config_.sigma_offset_radar_y               = node.declare_parameter("max_sigma_y", 0.15);
    config_.sigma_offset_radar_z               = node.declare_parameter("max_sigma_z", 0.2);
    config_.max_sigma_x                        = node.declare_parameter("max_r_cond", 1.0e3);
    config_.max_sigma_y                        = node.declare_parameter("use_cholesky_instead_of_bdcsvd", true);
    config_.max_sigma_z                        = node.declare_parameter("use_ransac", true);
    config_.max_r_cond                         = node.declare_parameter("outlier_prob", 0.4);
    config_.use_cholesky_instead_of_bdcsvd     = node.declare_parameter("success_prob", 0.9999);
    config_.use_ransac                         = node.declare_parameter("N_ransac_points", 3);
    config_.outlier_prob                       = node.declare_parameter("inlier_thresh", 0.15);
    config_.success_prob                       = node.declare_parameter("sigma_offset_radar_x", 0.05);
    config_.N_ransac_points                    = node.declare_parameter("sigma_offset_radar_y", 0.025);
    config_.inlier_thresh                      = node.declare_parameter("sigma_offset_radar_z", 0.05);
    config_.use_odr                            = node.declare_parameter("use_odr", false);
    config_.sigma_v_d                          = node.declare_parameter("min_speed_odr", 4.0);
    config_.min_speed_odr                      = node.declare_parameter("sigma_v_d", 0.125);
    config_.model_noise_offset_deg             = node.declare_parameter("model_noise_offset_deg", 2.0);
    config_.model_noise_scale_deg              = node.declare_parameter("model_noise_scale_deg", 10.0);
    // clang-format on
    configure(config);
  }

  assert(success && "Failed to load all rosparameters --> check error message above");

  T_b_r_.translation() = l_b_r;
  T_b_r_.linear()      = Matrix3(q_b_r);
}

bool RadarBodyVelocityEstimator::estimate(const sensor_msgs::msg::PointCloud2& radar_scan_msg,
                                          const Vector3& w_b,
                                          Vector3& v_b_r,
                                          Matrix3& P_v_b)
{
  Vector3 v_r;
  Matrix3 P_v_r;

  if (radar_ego_velocity_estimator_.estimate(radar_scan_msg, v_r, P_v_r))
  {
    // v_b & sigma_v_b
    const Vector3 v_b_w = math_helper::skewVec(w_b) * T_b_r_.translation();
    v_b_r               = T_b_r_.linear() * v_r - v_b_w;
    P_v_b               = T_b_r_.linear() * P_v_r * T_b_r_.linear().transpose();

    return true;
  }

  return false;
}
