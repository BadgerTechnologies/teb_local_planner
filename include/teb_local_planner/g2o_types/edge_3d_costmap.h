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
#ifndef EDGE_3D_COSTMAP_H_
#define EDGE_3D_COSTMAP_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>

// costmap 3d
#include <costmap_3d/costmap_3d_query.h>

namespace teb_local_planner
{

/**
 * @class Edge3DCostmap
 * @brief Edge defining the cost function for navigating around obstacles
 *        as defined by distance queries on a 3D costmap.
 */
class Edge3DCostmap : public BaseTebUnaryEdge<1, const void *, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  Edge3DCostmap()
  {
  }

  /**
   * @brief Construct edge.
   */
  Edge3DCostmap(costmap_3d::Costmap3DQueryPtr costmap_3d_query)
  {
    costmap_3d_query_ = costmap_3d_query;
  }

  void setCostmap3D(costmap_3d::Costmap3DQueryPtr costmap_3d_query)
  {
    costmap_3d_query_ = costmap_3d_query;
  }

  void computeError();

protected:
  costmap_3d::Costmap3DQueryPtr costmap_3d_query_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace

#endif
