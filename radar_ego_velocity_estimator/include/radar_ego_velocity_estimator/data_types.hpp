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

#include <angles/angles.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace reve
{
typedef double Real;

typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::Matrix<double, 7, 1> Vector7;
typedef Eigen::Matrix<double, 11, 1> Vector11;
typedef Eigen::VectorXd Vector;
typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Isometry3d Isometry;

struct ImuData
{
  ImuData() : dt{0} {}
  ImuData(const double dt, const Vector3& a_b_ib, const Vector3& w_b_ib) : dt{dt}, a_b_ib{a_b_ib}, w_b_ib{w_b_ib} {}

  Real dt;         // [s]
  Vector3 a_b_ib;  // [m/s^2]
  Vector3 w_b_ib;  // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuDataStamped
{
  ImuDataStamped() : dt{0} {}
  ImuDataStamped(const rclcpp::Time& time_stamp,
                 const std::string frame_id,
                 const double dt,
                 const Vector3& a_b_ib,
                 const Vector3& w_b_ib) :
    time_stamp{time_stamp},
    frame_id{frame_id},
    dt{dt},
    a_b_ib{a_b_ib},
    w_b_ib{w_b_ib}
  {
  }

  ImuDataStamped(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, const Real dt) :
    time_stamp{imu_msg->header.stamp},
    frame_id{imu_msg->header.frame_id},
    dt{dt},
    a_b_ib{Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z)},
    w_b_ib{Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)}
  {
  }

  sensor_msgs::msg::Imu toImuMsg()
  {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp          = time_stamp;
    imu_msg.angular_velocity.x    = w_b_ib.x();
    imu_msg.angular_velocity.y    = w_b_ib.y();
    imu_msg.angular_velocity.z    = w_b_ib.z();
    imu_msg.linear_acceleration.x = a_b_ib.x();
    imu_msg.linear_acceleration.y = a_b_ib.y();
    imu_msg.linear_acceleration.z = a_b_ib.z();
    return imu_msg;
  }

  rclcpp::Time time_stamp;  // ros::Time
  std::string frame_id;  // frame id
  Real dt;               // [s]
  Vector3 a_b_ib;        // [m/s^2]
  Vector3 w_b_ib;        // [rad/s]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace reve
