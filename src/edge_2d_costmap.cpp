#include <cmath>
#include <vector>

//#include <teb_local_planner/obstacles.h>
//#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

#include <teb_local_planner/g2o_types/edge_2d_costmap.h>
#include <Eigen/Geometry> 
namespace teb_local_planner
{


boost::shared_ptr<std::vector<Eigen::Vector2d>>
Edge2DCostmap::costmapToObstacles(costmap_2d::Costmap2D* costmap)
{
  if (!costmap)
  {
    ROS_ERROR("costmap is null");
    return NULL;
  }
      
  boost::shared_ptr<std::vector<Eigen::Vector2d>> obstacles =
    boost::make_shared<std::vector<Eigen::Vector2d>>();
  for(std::size_t i = 0; i < costmap->getSizeInCellsX(); i++)
  {
    for(std::size_t j = 0; j < costmap->getSizeInCellsY(); j++)
    {
      int value = costmap->getCost(i,j);
      if(value == costmap_2d::LETHAL_OBSTACLE)
      {
        double x, y;
        costmap->mapToWorld((unsigned int)i, (unsigned int)j, x, y);
        Eigen::Vector2d obstacle_position(x, y);
        obstacles->push_back(obstacle_position);
      }
    }  // for j
  }  // for i
  return obstacles;
}

void Edge2DCostmap::setRobotWorldPosition(const Eigen::Vector2d& position, double theta)
{
  // Set the robot_model_world_ from robot_model_local_ and the position
  Eigen::Rotation2D<double> rotation(theta);
  robot_model_world_.clear();
  int index = 0;
  for (auto circle = robot_model_local_.begin(); circle != robot_model_local_.end(); ++circle)
  {
    Eigen::Vector2d rotated_local_center = rotation * circle->position;
    Circle wc(rotated_local_center + position, circle->radius);
    //ROS_INFO("RLH: World Circle %d: (%f,%f)-%f. (RP:(%f,%f), %fdeg)", index, wc.position[0], wc.position[1], wc.radius, position[0], position[1], theta * 180 / 3.14159);
    robot_model_world_.push_back(wc);
    index++;
  }  // for circle
}

double Edge2DCostmap::getDistance(const Eigen::Vector2d& obstacle)
{
  double closest_dist = 1000;
  for (auto circle = robot_model_world_.begin(); circle != robot_model_world_.end(); ++circle)
  {
    double dist = (obstacle - circle->position).norm() - circle->radius;
    if (dist < closest_dist)
    {
      closest_dist = dist;
    }
  }  // for circle
  //ROS_INFO_THROTTLE(1, "RLH: returning distance: %f", closest_dist);
  return closest_dist;
}

void Edge2DCostmap::computeError()
{
  const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
  double closest_dist = 1000;
  Eigen::Vector2d closest_obst = *obstacles_->begin();
  setRobotWorldPosition(pose->position(), pose->theta());
  
  for(std::vector<Eigen::Vector2d>::iterator it = obstacles_->begin(); it != obstacles_->end(); ++it)
  {
    double dist = getDistance(*it);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_obst = *it;
    }
  }  // for

  double exponential_error = exp(-(closest_dist - cfg_->optim.weight_costmap_exponential_shift));
  double linear_error = std::max(0.0, (1 - closest_dist * cfg_->optim.weight_costmap_linear_slope));
  double error = cfg_->optim.weight_costmap_linear * linear_error +
    cfg_->optim.weight_costmap_exponential * exponential_error;
  
  {
    const Eigen::Vector2d robot_position = pose->position();
    ROS_WARN_THROTTLE(1, "RLH: Error (dos-circs) (%ld obsts): %.3f ((%f, %f), RP(%f,%f), dist:%f)", obstacles_->size(), error, closest_obst[0], closest_obst[1], robot_position[0], robot_position[1], closest_dist);
  }
  
  _error[0] = error;
}

};  // namespace
