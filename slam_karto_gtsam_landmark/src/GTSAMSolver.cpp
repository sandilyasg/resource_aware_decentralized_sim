/*********************************************************************
 *
 *  Copyright (c) 2017, Saurav Agarwal
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Saurav Agarwal */

/*********************************************************************
 *
 * Copyright © 2023, Sandilya Sai Garimella
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Sandilya Sai Garimella */

#include <limits>
#include <open_karto/Karto.h>
#include <ros/console.h>
#include "GTSAMSolver.h"
#include <boost/format.hpp> // Include this header for formatted printing

using namespace falkolib;

GTSAMSolver::GTSAMSolver()
{
  using namespace gtsam;

  // add the prior on the first node which is known
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));

  graph_.emplace_shared<PriorFactor<Pose2>>(0, Pose2(0, 0, 0), priorNoise);
  scan1_ = LaserScan(0.0, 2.0 * M_PI, 360);

  // Initialize the FALKO extractor and set its parameters
  fe_.setMinExtractionRange(0.05);
  fe_.setMaxExtractionRange(4.0);
  fe_.enableSubbeam(true);
  fe_.setNMSRadius(0.025); // Non-Maxima Suppression Radius
  fe_.setNeighB(0.05);
  fe_.setBRatio(3);
  fe_.setGridSectors(16);

  bufferMapKeypoints = std::unordered_map<int, TransformedKeypoint>() ;
  observedMapKeypoints = std::unordered_map<int, TransformedKeypoint>() ;
}

GTSAMSolver::~GTSAMSolver()
{
}

void GTSAMSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector &GTSAMSolver::GetCorrections() const
{
  return corrections_;
}


void GTSAMSolver::Compute()
{
  using namespace gtsam;

  // corrections_.clear();
  // graphNodes_.clear();
  correctedGraphNodes_.clear();
  // result_.clear();

  LevenbergMarquardtParams parameters;

  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;

  // Do not perform more than N iteration steps
  parameters.maxIterations = 500;

  // Create the optimizer ...
  LevenbergMarquardtOptimizer optimizer(graph_, initialGuessValues_, parameters);

  // ... and optimize
  // Values result_ = optimizer.optimize();
  result_ = optimizer.optimize();

  // Calculate and print marginal covariances for each value in the result_
  Marginals marginals(graph_, result_);
  result_.print("COMPUTE1 Final result_:\n"); // prints Value, and post-optimization pose/point

  for (const auto& value : result_) {
      Symbol key = value.key;
      auto marginalCovariance = marginals.marginalCovariance(key);
      

      // Create a formatted string for printing the information
      std::stringstream ss;
      ss << "Key ID: " << key.index() << ", ";

      // std::cout << "value.value.dim()" << value.value.dim() << std::endl;

      // Check if it's a Pose2 or Point2 value
      if (value.value.dim() == 3) {
        auto pose2 = value.value.cast<Pose2>();
        ss << "Optimized Position (x, y, theta): (" << pose2.x() << ", " << pose2.y() <<", " << pose2.theta() <<")";
      } 
      else if (value.value.dim() == 2) {
        auto point2 = value.value.cast<Point2>();
        ss << "Optimized Position (x, y): (" << point2.x() << ", " << point2.y() << ")";
      }

      // Print the key ID, optimized position, and marginal covariance in the same line
      ROS_INFO_STREAM(boost::format("%s, Marginal Covariance:\n%s") % ss.str() % marginalCovariance);
    }

  // Iterate through all edges in the factor graph
  // for (const auto& edge : graph_.edges()) {

  //   // Check for pose-pose edges
  //   auto posePoseFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose2, Pose2>>(edge);
  //   if (posePoseFactor) {
  //     Symbol pose1Key = posePoseFactor->key1();
  //     Symbol pose2Key = posePoseFactor->key2();
  //     std::cout << "Edge between poses: " << pose1Key << " and " << pose2Key << std::endl;
  //   }

  //   // Check for point-point edges 
  //   // auto pointPointFactor = boost::dynamic_pointer_cast<BetweenFactor<Point2, Point2>>(edge);
  //   // if (pointPointFactor) {
  //   //   Symbol point1Key = pointPointFactor->key1();
  //   //   Symbol point2Key = pointPointFactor->key2();
  //   //   std::cout << "Edge between points: " << point1Key << " and " << point2Key << std::endl;
  //   // }

  //   // Check for pose-point edges
  //   auto posePointFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose2, Point2>>(edge);
  //   if (posePointFactor) {
  //     Symbol poseKey = posePointFactor->key1();
  //     Symbol pointKey = posePointFactor->key2();
  //     std::cout << "Edge between pose: " << poseKey  
  //               << " and point: " << pointKey << std::endl;
  //   }
  // }

  Values::ConstFiltered<Pose2> viewPose2 = result_.filter<Pose2>();
  Values::ConstFiltered<Point2> viewPoint2 = result_.filter<Point2>();
  // ROS_INFO("GTSAMSolver::Compute() checkpoint: GTSAM pose graph LM optimization COMPLETED");

  for (const Values::ConstFiltered<Point2>::KeyValuePair &key_value : viewPoint2) {
    ROS_INFO("map feature key: %d", key_value.key);
    correctedGraphNodes_.push_back(Eigen::Vector2d(key_value.value.x(), key_value.value.y()));
  }

  // put values into corrections container
  for (const Values::ConstFiltered<Pose2>::KeyValuePair &key_value : viewPose2)
  {
    karto::Pose2 pose(key_value.value.x(), key_value.value.y(), key_value.value.theta());

    corrections_.push_back(std::make_pair(key_value.key, pose));

    // graphNodes_.push_back(Eigen::Vector2d(key_value.value.x(), key_value.value.y()));
    // correctedGraphNodes_.push_back(Eigen::Vector2d(key_value.value.x(), key_value.value.y()));
  }

  // result_.print("COMPUTE2 Final result_:\n");
}


void GTSAMSolver::AddRobotPoseNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
{

  using namespace gtsam;

  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();

  auto robotPoseUniqueId1 = pVertex->GetObject()->GetUniqueId();
  // std::cout << "robotPoseUniqueId1: " << robotPoseUniqueId1 << std::endl;

  // Pose2 here is gtsam::Pose2.
  initialGuessValues_.insert(pVertex->GetObject()->GetUniqueId(), Pose2(odom.GetX(), odom.GetY(), odom.GetHeading()));

  graphNodes_.push_back(Eigen::Vector2d(odom.GetX(), odom.GetY()));

  ROS_DEBUG("[gtsam] Adding robot pose node %d.", pVertex->GetObject()->GetUniqueId());
}


void GTSAMSolver::AddPosetoPoseConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
{
  using namespace gtsam;

  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();

  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();
  // std::cout << "Edge between poses: " << sourceID << " and " << targetID << std::endl;

  // Set the measurement (poseGraphEdge distance between vertices)
  karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();

  // Set the covariance of the measurement
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance();

  Eigen::Matrix<double, 3, 3> cov;

  cov(0, 0) = precisionMatrix(0, 0);
  cov(0, 1) = cov(1, 0) = precisionMatrix(0, 1);
  cov(0, 2) = cov(2, 0) = precisionMatrix(0, 2);
  cov(1, 1) = precisionMatrix(1, 1);
  cov(1, 2) = cov(2, 1) = precisionMatrix(1, 2);
  cov(2, 2) = precisionMatrix(2, 2);

  noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Covariance(cov);

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph_.emplace_shared<BetweenFactor<Pose2>>(sourceID, targetID, Pose2(diff.GetX(), diff.GetY(), diff.GetHeading()), model);

  // Add the constraint to the optimizer
  ROS_DEBUG("[gtsam] Adding Edge from node %d to node %d.", sourceID, targetID);
}


void GTSAMSolver::AddMapFeatureNodeAndConstraint(karto::Vertex<karto::LocalizedRangeScan>* pVertex) {

  auto robotPoseUniqueId2 = pVertex->GetObject()->GetUniqueId();
  // std::cout << "robotPoseUniqueId2: " << robotPoseUniqueId2 << std::endl;

  // std::cout << "GTSAMSolver::AddMapFeatureNodeAndConstraint function checkpoint" << std::endl;
  // ROS_INFO("GTSAMSolver::AddMapFeatureNodeAndConstraint function checkpoint");
  using namespace falkolib;

  std::vector<FALKO> keypoints1;

  // !!!NOTE SSG: This is where keypoint feature detection is used, add relevant keypoint features here.
  karto::Pose2 robotCorrectedPose = pVertex->GetObject()->GetCorrectedPose();
  karto::LocalizedRangeScan* localizedLaserScan = pVertex->GetObject();

  karto::RangeReadingsVector rangeReadings = localizedLaserScan->GetRangeReadingsVector();

  // need to convert rangeReadings to std::vector<double> to pass into falkolib extractor
  scan1_.fromRanges(rangeReadings);  // converts the scan ranges into Cartesian coordinates. could use GetPointReadings()
  fe_.extract(scan1_, keypoints1); 

  // Extract indices of keypoints
  std::vector<int> keypointIndices;
  for (const auto& keypoint : keypoints1) {
        keypointIndices.push_back(keypoint.index);
    }

  kt_double robot_x = robotCorrectedPose.GetX();
  kt_double robot_y = robotCorrectedPose.GetY();
  kt_double robot_theta = robotCorrectedPose.GetHeading();

  // Create transformation matrix
  Eigen::Matrix3d transform;
  transform << std::cos(robot_theta), -std::sin(robot_theta), robot_x,
              std::sin(robot_theta), std::cos(robot_theta), robot_y,
              0, 0, 1;

  std::vector<TransformedKeypoint> transformedKeypoints; // keypoints which includes new ones and previously observed ones

  // find transformed keypoints (contains new keypoints and previously observed keypoints)
  for (const auto& keypointIndex : keypointIndices) {
    kt_double range = rangeReadings[keypointIndex];
    kt_double angle = localizedLaserScan->GetAngleMin() + keypointIndex * localizedLaserScan->GetAngleIncrement();

    // Calculate Cartesian coordinates in the laser's frame
    double xLaser = range * std::cos(angle);
    double yLaser = range * std::sin(angle);

    // Apply transformation from laser frame to world frame
    Eigen::Vector3d pointLaser(xLaser, yLaser, 1.0);
    Eigen::Vector3d pointWorld = transform * pointLaser;

    karto::Point2 transformedPoint2;
    transformedPoint2.SetX(pointWorld[0]);
    transformedPoint2.SetY(pointWorld[1]);

    TransformedKeypoint transformedKeypoint;
    transformedKeypoint.point = transformedPoint2;
    transformedKeypoint.range = range;
    transformedKeypoint.bearing = angle;
    transformedKeypoints.push_back(transformedKeypoint);  // transformedKeypoints is the set of newly observed keypoints
  }
  
  // std::vector<TransformedKeypoint> observedKeypoints;

  // CHECK IF transformed point WAS ALREADY OBSERVED
  // If not observed, add newly observed keypoint as new node to pose graph
  // Add keypoints to buffer points
  for (const auto& transformedKeyPoint : transformedKeypoints) {
    bool isDuplicate = false;
    const karto::Point2& transformedPoint = transformedKeyPoint.point;

    // Check if the transformed point is within the specified distance of any point in bufferMapKeypoints
    // CHECK IF transformed point WAS ALREADY OBSERVED
    for (const auto& bufferPointPair : bufferMapKeypoints) {
      TransformedKeypoint bufferPoint = bufferPointPair.second;
      double distance = std::hypot(transformedPoint.GetX() - bufferPoint.point.GetX(), transformedPoint.GetY() - 
      bufferPoint.point.GetY());

      if (distance <= 0.15) {
        isDuplicate = true;
        // add previously observed point to observedMapKeypoints
        int pointId = bufferPointPair.first;
        observedMapKeypoints[pointId] = bufferPoint;
        // observedMapKeypoints[pointId] = transformedKeyPoint;
        break;
      }
    }

    // Add the transformed point to bufferMapKeypoints if it is not a duplicate
    if (!isDuplicate) {
      // Create a new Symbol using the nextPointIndex_
      gtsam::Symbol symbol('l', nextPointId_);  // need to save this nextPointId_

      // Assign the transformed point to the Symbol in initialGuessValues_
      initialGuessValues_.insert(symbol, gtsam::Point2(transformedPoint.GetX(), transformedPoint.GetY()));

      // std::cout << "Edge between pose and landmark: " << robotPoseUniqueId2 << " and " << "l" + std::to_string(nextPointId_) << std::endl;
      // Assign a unique ID to the point
      bufferMapKeypoints[nextPointId_] = transformedKeyPoint;
      // add transformedKeyPoint to observed keypoints as a newly observed keypoint 
      observedMapKeypoints[nextPointId_] = transformedKeyPoint;

      graphNodes_.push_back(Eigen::Vector2d(transformedPoint.GetX(), transformedPoint.GetY()));
      nextPointId_++;
    }
  }

  // iterate over bufferMapKeypoints and add constraints from robot pose to all previously observed keypoints
  // we DO NOT want to add constraints from robot pose to all previously observed keypoints. We only
  // want contraints from the current robot pose to current observed keypoints, which may include
  // new keypoints and buffer keypoints

  // for (const auto& bufferPoint : bufferMapKeypoints) {
  //   // std::unordered_map<int, TransformedKeypoint> bufferMapKeypoints;
  // //   struct TransformedKeypoint {
  // //   karto::Point2 point;
  // //   kt_double range;
  // //   kt_double bearing;
  // // };
  //   int bufferPointId = bufferPoint.first;
  //   gtsam::Symbol landmarkN('l', bufferPointId);
  //   gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(bufferPoint.second.bearing);

  //   graph_.emplace_shared<BearingRangeFactor<gtsam::Pose2,gtsam::Point2>>(robotPoseUniqueId2, 
  //                                                     landmarkN, bearing, bufferPoint.second.range, measurementNoise_);
  // }

  // clear this vector so remains from previous iteration do not affect actually observed keypoints
  observed_keypoint_ids_.clear();

  for (const auto& observedPoint : observedMapKeypoints) {
    int observedPointId = observedPoint.first;
    // ROS_INFO("map feature observedPointId: %d", observedPointId);
    observed_keypoint_ids_.push_back(observedPointId);

    gtsam::Symbol landmarkN('l', observedPointId);
    gtsam::Rot2 bearing = gtsam::Rot2::fromAngle(observedPoint.second.bearing);

    graph_.emplace_shared<BearingRangeFactor<gtsam::Pose2,gtsam::Point2>>(robotPoseUniqueId2, 
    landmarkN, bearing, observedPoint.second.range, measurementNoise_);

    std::cout << "Edge between pose and landmark: " << robotPoseUniqueId2 << " and " << "l" + std::to_string(observedPointId) << std::endl;
  }

  // ROS_DEBUG("[gtsam] Adding map feature node %d.", pVertex->GetObject()->GetUniqueId());
  // ROS_INFO("GTSAMSolver::AddMapFeatureNodeAndConstraint checkpoint: map feature nodes and constraints added");
}


void GTSAMSolver::getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &edges)
{
  using namespace gtsam;

  // nodes = graphNodes_;
  nodes = correctedGraphNodes_;
}

void GTSAMSolver::printOptimizedResult() {
  using namespace gtsam;
  result_.print("PRINT OPTIMIZED RESULT Final result:\n");
}

void GTSAMSolver::getOptimizedResult(gtsam::Values &result) {
  using namespace gtsam;
  result = result_;
  // std::cout << "debug checkpoint 1" << std::endl;
  // result_.print("GET OPTIMIZED RESULT Final result:\n");
}

void GTSAMSolver::getGTSAMGraph(gtsam::NonlinearFactorGraph &graph) {
  using namespace gtsam;
  graph = graph_;
  // graph.print("GET GTSAM GRAPHGraph Contents:\n"); // gets printed correctly
}

void GTSAMSolver::getObservedKeyIds(std::vector<int> &obs_key_ids) {
  obs_key_ids = observed_keypoint_ids_;
}

// gtsam::Values GTSAMSolver::getOptimizedResult() {
//   using namespace gtsam;
//   // return result_;
//   // result = result_;
//   return result_;
// }

// gtsam::NonlinearFactorGraph GTSAMSolver::getGTSAMGraph() {
//   using namespace gtsam;
//   // graph = graph_;
//   return graph_;
// }