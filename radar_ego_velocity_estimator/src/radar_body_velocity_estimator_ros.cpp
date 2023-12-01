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

// #include <rosbag/bag.h>
// #include <rosbag/view.h>


// #include <radar_ego_velocity_estimator/ros_helper.h>

#include <radar_ego_velocity_estimator/radar_body_velocity_estimator_ros.hpp>

using namespace reve;

// void RadarBodyVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
//                                                   const double bag_start,
//                                                   const double bag_duration,
//                                                   const double sleep_ms)
// {
//   rosbag::Bag source_bag;
//   source_bag.open(rosbag_path, rosbag::bagmode::Read);
//   std::vector<std::string> topics;
//   topics.push_back(sub_imu_.getTopic());
//   topics.push_back(sub_radar_scan_.getTopic());
//   topics.push_back(sub_radar_trigger_.getTopic());
//   topics.push_back(pub_twist_ground_truth_.getTopic());

//   rosbag::View view(source_bag, rosbag::TopicQuery(topics));

//   auto first_timestamp = rclcpp::Time(0, 1);

//   for (const rosbag::MessageInstance& m : view)
//   {
//     if (!ros::ok())
//       break;

//     if (first_timestamp == rclcpp::Time(0, 1))
//       first_timestamp = m.getTime();

//     if ((m.getTime() - first_timestamp).toSec() < bag_start)
//       continue;

//     if ((m.getTime() - first_timestamp).toSec() > bag_duration)
//       break;

//     const auto topic = m.getTopic();
//     if (topic == sub_imu_.getTopic())
//     {
//       const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
//       if (imu_msg != NULL)
//         callbackImu(imu_msg);
//     }
//     else if (topic == sub_radar_scan_.getTopic())
//     {
//       const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
//       if (radar_scan != NULL)
//       {
//         callbackRadarScan(radar_scan);
//         if (sleep_ms > 0)
//           ros::Duration(sleep_ms / 1.0e3).sleep();
//       }
//     }
//     else if (topic == sub_radar_trigger_.getTopic())
//     {
//       const auto radar_trigger_msg = m.instantiate<std_msgs::Header>();
//       if (radar_trigger_msg != NULL)
//         callbackRadarTrigger(radar_trigger_msg);
//     }
//     else if (topic == pub_twist_ground_truth_.getTopic())
//     {
//       const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
//       if (msg)
//         pub_twist_ground_truth_.publish(msg);
//     }

//     ros::spinOnce();
//   }

//   ROS_INFO("%s Final Runtime statistics: %s",
//            kPrefix.c_str(),
//            profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
// }

void RadarBodyVelocityEstimatorRos::processRadarData(const sensor_msgs::msg::PointCloud2& radar_scan,
                                                     const Vector3 w_b,
                                                     const rclcpp::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, w_b, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp         = trigger_stamp;
    msg.header.frame_id      = "body";
    msg.twist.twist.linear.x = v_b_r.x();
    msg.twist.twist.linear.y = v_b_r.y();
    msg.twist.twist.linear.z = v_b_r.z();

    msg.twist.twist.angular.x = w_b.x();
    msg.twist.twist.angular.y = w_b.y();
    msg.twist.twist.angular.z = w_b.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    pub_twist_body_->publish(msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    RCLCPP_ERROR_STREAM(this->get_logger(), kPrefix << "Radar ego velocity estimation failed");
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       5000,
                       "%s Runtime statistics: %s",
                       kPrefix.c_str(),
                       profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarBodyVelocityEstimatorRos::callbackImu(const sensor_msgs::msg::Imu::ConstPtr& imu_msg)
{
  mutex_.lock();
  w_b_imu       = Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
  stamp_w_b_imu = imu_msg->header.stamp;

  if (!gyro_offset_initialized_)
  {
    if (!gyro_calib_start_)
      gyro_calib_start_ = std::make_unique<rclcpp::Time>(imu_msg->header.stamp);
    rclcpp::Duration init_delay = *gyro_calib_start_ - imu_msg->header.stamp;
    if (std::abs(init_delay.seconds()) < gyro_init_s_)
    {
      gyro_calib.emplace_back(w_b_imu);
    }
    else
    {
      Vector3 w_sum(0, 0, 0);
      for (const auto& w : gyro_calib) w_sum += w;
      offset_gyro = w_sum / gyro_calib.size();

      RCLCPP_INFO_STREAM(this->get_logger(), kPrefix << "Initialized gyro offset: " << offset_gyro.transpose());
      gyro_offset_initialized_ = true;
    }
  }
  mutex_.unlock();
}

void RadarBodyVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::msg::PointCloud2::ConstPtr& radar_scan_msg)
{
  mutex_.lock();

  if (gyro_offset_initialized_)
  {
    if (run_without_trigger)
    {
      // no trigger available --> use most recent omege measurement
      // catch bug of ti_mmwave driver --> time stamp is 0 :(
      if (radar_scan_msg->header.stamp.sec == 0)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "Time stamp of radar scan pcl is 0 using most recent IMU data as timestamp!");
        processRadarData(*radar_scan_msg, w_b_imu, stamp_w_b_imu);
      }
      else
      {
        processRadarData(*radar_scan_msg, w_b_imu, radar_scan_msg->header.stamp);
      }
    }
    else
    {
      if (trigger_stamp > rclcpp::Time(0, 1))
        processRadarData(*radar_scan_msg, w_b_radar, trigger_stamp);
      else
        RCLCPP_ERROR_STREAM(this->get_logger(), kPrefix << "Unable to process radar scan, no trigger message received!");
      trigger_stamp = rclcpp::Time(0, 1);
    }
  }

  mutex_.unlock();
}

void RadarBodyVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::msg::Header::ConstPtr& trigger_msg)
{
  mutex_.lock();
  w_b_radar     = w_b_imu - offset_gyro;
  trigger_stamp = trigger_msg->stamp;
  mutex_.unlock();
}
