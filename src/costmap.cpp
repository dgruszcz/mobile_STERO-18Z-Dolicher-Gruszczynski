#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cost_handler");
  ros::NodeHandle n;
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  costmap_2d::Costmap2DROS local_costmap("local_costmap", buffer);
  costmap_2d::Costmap2DROS global_costmap("global_costmap", buffer);
  global_planner::GlobalPlanner globalPlanner("global_planner", global_costmap.getCostmap(), global_costmap.getGlobalFrameID());
  base_local_planner::TrajectoryPlannerROS localPlanner;
  localPlanner.initialize("local_planner", &buffer, &local_costmap);
  

  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
 
    ROS_INFO("Krece w petli. Juz %d raz.\n", count+1);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

