/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Badger Technologies LLC
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: C. Andy Martin
 *********************************************************************/
#ifndef EDGE_3D_COSTMAP_NONLETHAL_H_
#define EDGE_3D_COSTMAP_NONLETHAL_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>

// costmap 3d
#include <costmap_3d/costmap_3d_query.h>

namespace teb_local_planner
{

/**
 * @class Edge3DCostmapNonlethalOnly
 * @brief Edge defining the cost function for navigating around nonlethal only
 *        obstacles (detected objects that are not considered lethal) using
 *        distance queries on a 3D costmap.
 */
class Edge3DCostmapNonlethalOnly : public BaseTebUnaryEdge<1, const void *, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  Edge3DCostmapNonlethalOnly()
  {
  }

  /**
   * @brief Construct edge.
   */
  Edge3DCostmapNonlethalOnly(costmap_3d::Costmap3DQueryPtr costmap_3d_query,
                             double obstacle_dist,
                             double obstacle_cost_exponent)
  {
    costmap_3d_query_ = costmap_3d_query;
    min_obstacle_dist_ = obstacle_dist;
    obstacle_cost_exponent_ = obstacle_cost_exponent;
  }

  inline void computeErrorImpl(bool reuse_results=false)
  {
    const VertexPose* vertex_pose = static_cast<const VertexPose*>(_vertices[0]);
    geometry_msgs::Pose pose_msg;
    vertex_pose->pose().toPoseMsg(pose_msg);

    costmap_3d::Costmap3DQuery::DistanceOptions dopts;
    dopts.reuse_past_result = reuse_results;
    dopts.signed_distance = true;
    dopts.query_obstacles = costmap_3d::Costmap3DQuery::NONLETHAL_ONLY;
    double signed_dist = costmap_3d_query_->footprintDistance(pose_msg, dopts);

    _error[0] = penaltyBoundFromBelow(signed_dist, min_obstacle_dist_, 0.0);

    if (obstacle_cost_exponent_ != 1.0 && min_obstacle_dist_ > 0.0 && signed_dist >= 0.0)
    {
      // Optional non-linear cost.
      _error[0] = min_obstacle_dist_ * std::pow(_error[0] / min_obstacle_dist_, obstacle_cost_exponent_);
    }

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "Edge3DCostmapNonlethalOnly::computeError() _error[0]=%f\n",_error[0]);
  }

  void computeError()
  {
    return computeErrorImpl();
  }

  virtual void linearizeOplus()
  {
    //Xi - estimate the jacobian numerically
    VertexPose* vi = static_cast<VertexPose*>(_vertices[0]);

    if (vi->fixed())
      return;

#ifdef G2O_OPENMP
    vi->lockQuadraticForm();
#endif

    const double delta = 1e-5;
    const double scalar = 1.0 / (delta);
    // calculate the derivative to the base pose each time, reducing total queries
    computeError();
    ErrorVector errorBeforeNumeric = _error;

    double add_vi[VertexPose::Dimension];
    std::fill(add_vi, add_vi + VertexPose::Dimension, 0.0);
    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexPose::Dimension; ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi);
      // reuse the previous calculation to save time estimating the derivative
      computeErrorImpl(true);
      vi->pop();
      add_vi[d] = 0.0;

      _jacobianOplusXi.col(d) = scalar * (_error - errorBeforeNumeric);
    } // end dimension
    _error = errorBeforeNumeric;
  }

protected:
  costmap_3d::Costmap3DQueryPtr costmap_3d_query_;
  double min_obstacle_dist_;
  double obstacle_cost_exponent_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}  // namespace teb_local_planner

#endif  // EDGE_3D_COSTMAP_NONLETHAL_H_
