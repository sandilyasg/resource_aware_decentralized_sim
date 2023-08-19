/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

/* Authors: Saurav Agarwal */


#ifndef KARTO_GTSAMSolver_H
#define KARTO_GTSAMSolver_H

#include <open_karto/Mapper.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a PriorFactor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// We will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Finally, once all of the factors have been added to our factor graph, we want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// We will use the common nonlinear optimizer Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <falkolib/Feature/FALKO.h>
#include <falkolib/Feature/CGH.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKOExtractor.h>

#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

#include <math.h>
#include <unordered_map>
using namespace falkolib;
using namespace gtsam;

typedef std::vector<karto::Matrix3> CovarianceVector;

/**
 * @brief Wrapper for GTSAM to interface with Open Karto
 */
class GTSAMSolver : public karto::ScanSolver
{
  public:

    GTSAMSolver();
    
    virtual ~GTSAMSolver();

  public:
    
    /**
     * @brief Clear the vector of corrections
     * @details Empty out previously computed corrections
     */
    virtual void Clear();
    
    /**
     * @brief Solve the SLAM back-end
     * @details Calls GTSAM to solve the SLAM back-end
     */
    virtual void Compute();
    
    /**
     * @brief Get the vector of corrections
     * @details Get the vector of corrections
     * @return Vector with corrected poses
     */
    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    /**
     * @brief Add a node to pose-graph
     * @details Add a node which is a ROBOT POSE to the pose-graph
     * @param pVertex the node to be added in
     */
    virtual void AddRobotPoseNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);
    
    /**
     * @brief Add an edge constraint to pose-graph
     * @details Adds a relative pose measurement constraint between two poses in the graph
     * @param pEdge [description]
     */
    virtual void AddPosetoPoseConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    /**
     * @brief Add a node to pose-graph
     * @details Add a node which is a MAP FEATURE and pose-to-map-feature constraints to the pose-graph
     * @param pVertex the node to be added in
     */
    virtual void AddMapFeatureNodeAndConstraint(karto::Vertex<karto::LocalizedRangeScan>* pVertex);


    /**
     * @brief Add an edge constraint to pose-graph
     * @details Adds a relative pose measurement constraint between ROBOT POSE and MAP FEATURE
     * @param pEdge [description]
     */
    // virtual void AddPosetoMapFeatureConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    /**
     * @brief Get the pose-graph 
     * @details Get the underlying graph from gtsam, return the graph of constraints
     * 
     * @param g the graph
     */
    void getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > &edges);

    /**
     * @brief Get the optimized results (which includes poses and points) and the GTSAM graph
     */
    virtual void getOptimizedResult(gtsam::Values &result);
    
    virtual void printOptimizedResult();
    // gtsam::Values getOptimizedResult();

    /**
     * @brief Get the GTSAM NonlinearFactorGraph
     */
    virtual void getGTSAMGraph(gtsam::NonlinearFactorGraph &graph);
    // gtsam::NonlinearFactorGraph getGTSAMGraph();

    virtual void getObservedKeyIds(std::vector<int> &obs_key_ids);

     /**
     * @brief Struct for storing transformed point (in world frame), range, and bearing 
     */
    struct TransformedKeypoint {
      karto::Point2 point;
      kt_double range;
      kt_double bearing;
    };

  protected:

    std::unordered_map<int, TransformedKeypoint> bufferMapKeypoints;
    std::unordered_map<int, TransformedKeypoint> observedMapKeypoints;
    // std::unordered_map<gtsam::Symbol, TransformedKeypoint> bufferMapKeypoints;
    int nextPointId_ = 0;

    // 0.00524 rad std on bearing, 0.003 meters on range
    gtsam::noiseModel::Diagonal::shared_ptr measurementNoise_ = 
                                      noiseModel::Diagonal::Sigmas(Vector2(0.00524, 0.003));  


  private:
    
    karto::ScanSolver::IdPoseVector corrections_;

    gtsam::NonlinearFactorGraph graph_;

    gtsam::Values initialGuessValues_;
    gtsam::Values result_ ;

    std::vector<Eigen::Vector2d> graphNodes_;
    std::vector<Eigen::Vector2d> correctedGraphNodes_;
    std::vector<int> observed_keypoint_ids_;

    FALKOExtractor fe_;
    LaserScan scan1_;

};

#endif // KARTO_GTSAMSolver_H

