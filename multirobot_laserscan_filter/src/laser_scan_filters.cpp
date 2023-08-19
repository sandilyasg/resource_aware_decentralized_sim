/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "multirobot_laserscan_filter/median_filter.h"
#include "multirobot_laserscan_filter/array_filter.h"
#include "multirobot_laserscan_filter/intensity_filter.h"
#include "multirobot_laserscan_filter/range_filter.h"
#include "multirobot_laserscan_filter/scan_mask_filter.h"
#include "multirobot_laserscan_filter/scan_shadows_filter.h"
#include "multirobot_laserscan_filter/footprint_filter.h"
#include "multirobot_laserscan_filter/interpolation_filter.h"
#include "multirobot_laserscan_filter/angular_bounds_filter.h"
#include "multirobot_laserscan_filter/angular_bounds_filter_in_place.h"
#include "multirobot_laserscan_filter/box_filter.h"
// #include "multirobot_laserscan_filter/polygon_filter.h"
// #include "multirobot_laserscan_filter/speckle_filter.h"
#include "multirobot_laserscan_filter/scan_blob_filter.h"
// #include "multirobot_laserscan_filter/sector_filter.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

#include "pluginlib/class_list_macros.h"


PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserMedianFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserArrayFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanIntensityFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanRangeFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanAngularBoundsFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanAngularBoundsFilterInPlace, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanFootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::ScanShadowsFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::InterpolationFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanBoxFilter, filters::FilterBase<sensor_msgs::LaserScan>)
// PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanPolygonFilter, filters::FilterBase<sensor_msgs::LaserScan>)
// PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanSpeckleFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanMaskFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::ScanBlobFilter, filters::FilterBase<sensor_msgs::LaserScan>)
// PLUGINLIB_EXPORT_CLASS(multirobot_laserscan_filter::LaserScanSectorFilter, filters::FilterBase<sensor_msgs::LaserScan>)
