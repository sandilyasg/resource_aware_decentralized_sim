/*  ADAPTED FROM LASER_FILTERS ROS PACKAGE
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by Eurotec B.V.
 *  Copyright (c) 2020, Eurotec B.V.
 *  All rights reserved.
 *
 * multirobot_pos_filter.cpp
 */

// #include <multirobot_laserscan_filter/speckle_filter.h>
#include <multirobot_laserscan_filter/multirobot_pos_filter.h>

#include <ros/node_handle.h>

namespace multirobot_laserscan_filter
{
MultirobotPosFilter::MultirobotPosFilter()
{
  validator_ = 0;
}

MultirobotPosFilter::~MultirobotPosFilter()
{
  if (!validator_)
  {
    delete validator_;
  }
}

bool MultirobotPosFilter::configure()
{
  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<multirobot_laserscan_filter::SpeckleFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<multirobot_laserscan_filter::SpeckleFilterConfig>::CallbackType f;
  f = boost::bind(&multirobot_laserscan_filter::MultirobotPosFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  getParam("filter_type", config_.filter_type);
  getParam("max_range", config_.max_range);
  getParam("max_range_difference", config_.max_range_difference);
  getParam("filter_window", config_.filter_window);
  dyn_server_->updateConfig(config_);
  return true;
}

bool MultirobotPosFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan)
{
  output_scan = input_scan;
  std::vector<bool> valid_ranges(output_scan.ranges.size(), false);

  /*Check if range size is big enough to use the filter window */
  if (output_scan.ranges.size() <= config_.filter_window + 1)
  {
    ROS_ERROR("Scan ranges size is too small: size = %i", output_scan.ranges.size());
    return false;
  }

  for (size_t idx = 0; idx < output_scan.ranges.size() - config_.filter_window + 1; ++idx)
  {
    bool window_valid = validator_->checkWindowValid(
          output_scan, idx, config_.filter_window, config_.max_range_difference);

    // Actually set the valid ranges (do not set to false if it was already valid or out of range)
    for (size_t neighbor_idx_or_self_nr = 0; neighbor_idx_or_self_nr < config_.filter_window; ++neighbor_idx_or_self_nr)
    {
      size_t neighbor_idx_or_self = idx + neighbor_idx_or_self_nr;
      if (neighbor_idx_or_self < output_scan.ranges.size())  // Out of bound check
      {
        bool out_of_range = output_scan.ranges[neighbor_idx_or_self] > config_.max_range;
        valid_ranges[neighbor_idx_or_self] = valid_ranges[neighbor_idx_or_self] || window_valid || out_of_range;
      }
    }
  }

  for (size_t idx = 0; idx < valid_ranges.size(); ++idx)
  {
    if (!valid_ranges[idx])
    {
      output_scan.ranges[idx] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

void MultirobotPosFilter::reconfigureCB(multirobot_laserscan_filter::SpeckleFilterConfig& config, uint32_t level)
{
  config_ = config;

  switch (config_.filter_type) {
    case multirobot_laserscan_filter::SpeckleFilter_RadiusOutlier:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new multirobot_laserscan_filter::RadiusOutlierWindowValidator();
      break;

    case multirobot_laserscan_filter::SpeckleFilter_Distance:
      if (validator_)
      {
        delete validator_;
      }
      validator_ = new multirobot_laserscan_filter::DistanceWindowValidator();
      break;

    default:
      break;
  }

}
}
