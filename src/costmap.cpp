#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "stero_mobile_init/ElektronSrv.h"
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>

base_local_planner::TrajectoryPlannerROS *localPlanner;
global_planner::GlobalPlanner *globalPlanner;
std::vector<geometry_msgs::PoseStamped> plan;
ros::Publisher pub;

bool plan_and_execute(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res)
{
  ROS_INFO("request: x=%f, y=%f", req.start.pose.position.x, req.start.pose.position.y);
  if(!globalPlanner->makePlan(req.start, req.goal, plan)){
    ROS_INFO("Unable to calculate plan. Try again");
    return false;
  }
  ROS_INFO("Plan calculated successfully");
  res.plan.poses = plan;
  // I teraz trzeba ogarnac lokalny planer
  geometry_msgs::Twist vel;
  localPlanner->setPlan(plan);
  ros::Rate loop_rate(100);
  while(!localPlanner->isGoalReached()){
    localPlanner->computeVelocityCommands(vel);
    pub.publish(vel);
    if (vel.linear.x == 0 && vel.angular.z == 0)
      return true;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_executor");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("/global_planner/planner/make_plan", plan_and_execute);
  pub = n.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel", 1000);
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  costmap_2d::Costmap2DROS local_costmap("local_costmap", buffer);
  costmap_2d::Costmap2DROS global_costmap("global_costmap", buffer);
  globalPlanner = new global_planner::GlobalPlanner();
  localPlanner = new base_local_planner::TrajectoryPlannerROS();
  globalPlanner->initialize("global_planner", global_costmap.getCostmap(), "map");
  localPlanner->initialize("local_planner", &buffer, &local_costmap);
  while(ros::ok()){
    ros::spin();
  }
  return 0;
}

