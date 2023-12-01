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
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator_ros.hpp>

using namespace reve;

// void RadarEgoVelocityEstimatorRos::runFromRosbag(const std::string& rosbag_path,
//                                                  const double bag_start,
//                                                  const double bag_duration,
//                                                  const double sleep_ms)
// {
//   rosbag::Bag source_bag;
//   source_bag.open(rosbag_path, rosbag::bagmode::Read);
//   std::vector<std::string> topics;
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
//     if (topic == sub_radar_scan_.getTopic())
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
//     {pub_twist_ground_truth_->getop
//       const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
//       if (msg)
//         pub_twist_ground_truth_->publish(msg);
//     }

//     ros::spinOnce();
//   }

//   RCLCPP_INFO(this->get_logger(),"%s Final Runtime statistics: %s",
//            kPrefix.c_str(),
//            profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
// }

void RadarEgoVelocityEstimatorRos::reconfigureCallback(RadarEgoVelocityEstimatorConfig& config)
{
  estimator_.configure(config);
}


void RadarEgoVelocityEstimatorRos::processRadarData(const sensor_msgs::msg::PointCloud2& radar_scan,
                                                    const rclcpp::Time& trigger_stamp)
{
  Vector3 v_b_r;
  Matrix3 P_v_b_r;
  profiler.start("ego_velocity_estimation");
  if (estimator_.estimate(radar_scan, v_b_r, P_v_b_r))
  {
    profiler.stop("ego_velocity_estimation");

    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covaricance_msg;
    twist_with_covaricance_msg.header.stamp         = trigger_stamp;
    twist_with_covaricance_msg.header.frame_id      = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
    twist_with_covaricance_msg.twist.twist.linear.x = v_b_r.x();
    twist_with_covaricance_msg.twist.twist.linear.y = v_b_r.y();
    twist_with_covaricance_msg.twist.twist.linear.z = v_b_r.z();

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp     = trigger_stamp;
    twist_msg.header.frame_id  = (radar_scan.header.frame_id.empty())? "radar" : radar_scan.header.frame_id;
    twist_msg.twist.linear.x   = v_b_r.y();
    twist_msg.twist.linear.y   = -v_b_r.x();
    twist_msg.twist.linear.z   = v_b_r.z();

    for (uint l = 0; l < 3; ++l)
      for (uint k = 0; k < 3; ++k) twist_with_covaricance_msg.twist.covariance.at(l * 6 + k) = P_v_b_r(l, k);
    
    pub_twist_->publish(twist_msg);
    pub_twist_with_covaricance_->publish(twist_with_covaricance_msg);
  }
  else
  {
    profiler.stop("ego_velocity_estimation");
    // ROS_ERROR_STREAM(kPrefix << "Radar ego velocity estimation failed");
    RCLCPP_ERROR_STREAM(this->get_logger(), "Radar ego velocity estimation failed");
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       1000,
                       "Runtime statistics: %s",
                       profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());;
  // ROS_INFO_THROTTLE(5,
                    // "%s Runtime statistics: %s",
                    // kPrefix.c_str(),
                    // profiler.getStatistics("ego_velocity_estimation").toStringMs().c_str());
}

void RadarEgoVelocityEstimatorRos::callbackRadarScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& radar_scan_msg)
{
  mutex_.lock();

  if (run_without_trigger)
  {
    // no trigger available --> use most recent omege measurement
    // catch bug of ti_mmwave driver --> time stamp is 0 :(
    if (radar_scan_msg->header.stamp.sec == 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
                           *this->get_clock(),
                           1000,
                           "Time stamp of radar scan pcl is 0 using current ros time!");
      processRadarData(*radar_scan_msg, this->get_clock()->now());
    }
    else
    {
      processRadarData(*radar_scan_msg, radar_scan_msg->header.stamp);
    }
  }
  else
  {
    if (trigger_stamp)
      processRadarData(*radar_scan_msg, *trigger_stamp);
    else
      RCLCPP_ERROR_STREAM(this->get_logger(), kPrefix << "Unable to process radar scan, no trigger message received!");
    trigger_stamp.reset();
  }

  mutex_.unlock();
}

void RadarEgoVelocityEstimatorRos::callbackRadarTrigger(const std_msgs::msg::Header::ConstSharedPtr& trigger_msg)
{
  mutex_.lock();
  if(trigger_stamp)
  {
    *trigger_stamp = trigger_msg->stamp;
  }
  else
  {
    trigger_stamp = std::make_unique<rclcpp::Time>(trigger_msg->stamp);
  }
    
  mutex_.unlock();
}
