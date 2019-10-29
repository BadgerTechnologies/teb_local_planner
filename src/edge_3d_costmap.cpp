#include <teb_local_planner/g2o_types/edge_3d_costmap.h>

namespace teb_local_planner
{

void Edge3DCostmap::computeError()
{
  const VertexPose* vertex_pose = static_cast<const VertexPose*>(_vertices[0]);
  geometry_msgs::Pose pose_msg;
  vertex_pose->pose().toPoseMsg(pose_msg);

  double closest_dist = costmap_3d_query_->footprintDistance(pose_msg);

  double exponential_error = exp(-(closest_dist - cfg_->optim.weight_costmap_3d_exponential_shift));
  double linear_error = std::max(0.0, (1 - closest_dist * cfg_->optim.weight_costmap_3d_linear_slope));
  double error = cfg_->optim.weight_costmap_3d_linear * linear_error +
    cfg_->optim.weight_costmap_3d_exponential * exponential_error;

#if 0
  {
    const Eigen::Vector2d robot_position = pose->position();
    ROS_WARN_THROTTLE(1, "RLH: Error (dos-circs) (%ld obsts): %.3f ((%f, %f), RP(%f,%f), dist:%f)", obstacles_->size(), error, closest_obst[0], closest_obst[1], robot_position[0], robot_position[1], closest_dist);
  }
#endif

  _error[0] = error;
}

};  // namespace
