/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
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
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_2D_COSTMAP_H_
#define EDGE_2D_COSTMAP_H_

#include <cmath>
#include <boost/shared_ptr.hpp>

//#include <teb_local_planner/obstacles.h>
//#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

namespace teb_local_planner
{

struct Circle
{
  Circle(double center_x, double center_y, double radius)
  {
    position[0] = center_x;
    position[1] = center_y;
    this->radius = radius;
  }

  Circle(const Eigen::Vector2d& position, double radius)
  {
    this->position = position;
    this->radius = radius;
  }

  Eigen::Vector2d position;
  double radius;
};

/**
 * @class Edge2DCostmap
 * @brief Edge defining the cost function for navigating around obstacles
 *        as defined by a 2D costmap.
 */
class Edge2DCostmap : public BaseTebUnaryEdge<1, const void *, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  Edge2DCostmap()
  {
    costmap_ = NULL;
    _measurement = NULL;
    robot_model_local_.push_back(Circle(+0.16, 0.0, 0.26));
    robot_model_local_.push_back(Circle(-0.16, 0.0, 0.26));
  }

  void computeError();

#ifdef USE_ANALYTIC_JACOBI
#if 0

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgePointObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    Eigen::Vector2d deltaS = *_measurement - bandpt->position();
    double angdiff = atan2(deltaS[1],deltaS[0])-bandpt->theta();

    double dist_squared = deltaS.squaredNorm();
    double dist = sqrt(dist_squared);

    double aux0 = sin(angdiff);
    double dev_left_border = penaltyBoundFromBelowDerivative(dist*fabs(aux0),cfg_->obstacles.min_obstacle_dist,cfg_->optim.penalty_epsilon);

    if (dev_left_border==0)
    {
      _jacobianOplusXi( 0 , 0 ) = 0;
      _jacobianOplusXi( 0 , 1 ) = 0;
      _jacobianOplusXi( 0 , 2 ) = 0;
      return;
    }

    double aux1 = -fabs(aux0) / dist;
    double dev_norm_x = deltaS[0]*aux1;
    double dev_norm_y = deltaS[1]*aux1;

    double aux2 = cos(angdiff) * g2o::sign(aux0);
    double aux3 = aux2 / dist_squared;
    double dev_proj_x = aux3 * deltaS[1] * dist;
    double dev_proj_y = -aux3 * deltaS[0] * dist;
    double dev_proj_angle = -aux2;

    _jacobianOplusXi( 0 , 0 ) = dev_left_border * ( dev_norm_x + dev_proj_x );
    _jacobianOplusXi( 0 , 1 ) = dev_left_border * ( dev_norm_y + dev_proj_y );
    _jacobianOplusXi( 0 , 2 ) = dev_left_border * dev_proj_angle;
  }
#endif
#endif

  void setParameters(const TebConfig& cfg)
  {
    cfg_ = &cfg;
  }

  void setObstacles(boost::shared_ptr<std::vector<Eigen::Vector2d>> obstacles)
  {
    obstacles_ = obstacles;
  }

  static boost::shared_ptr<std::vector<Eigen::Vector2d>> costmapToObstacles(costmap_2d::Costmap2D* costmap);


protected:
  void setRobotWorldPosition(const Eigen::Vector2d& position, double theta);
  double getDistance(const Eigen::Vector2d& obstacle);

  costmap_2d::Costmap2D* costmap_;
  boost::shared_ptr<std::vector<Eigen::Vector2d>> obstacles_;
  std::vector<Circle> robot_model_local_;
  std::vector<Circle> robot_model_world_;

public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace

#endif
