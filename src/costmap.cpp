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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <vector>

base_local_planner::TrajectoryPlannerROS *localPlanner;
global_planner::GlobalPlanner *globalPlanner;
std::vector<geometry_msgs::PoseStamped> plan;
ros::Publisher pub;
geometry_msgs::Pose currentPosition = geometry_msgs::Pose();



void do360(float rotSpeed=0.3, float frequency=20);
float getRotZ(geometry_msgs::Quaternion);

bool plan_and_execute(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res)
{
  ROS_INFO("request: x=%f, y=%f", req.start.pose.position.x, req.start.pose.position.y);
  if(!globalPlanner->makePlan(req.start, req.goal, plan)){
    ROS_INFO("Unable to calculate plan. Try again");
    return false;
  }
  ROS_INFO("Plan calculated successfully");
  res.plan.poses = plan;
  geometry_msgs::Twist vel = geometry_msgs::Twist();
  localPlanner->setPlan(plan);
  ros::Rate loop_rate(1000);
  int counter = 0;
  bool state;
  while(!localPlanner->isGoalReached() && ros::ok()){
    state = localPlanner->computeVelocityCommands(vel);
    pub.publish(vel);
    ROS_INFO("state: %d", state);
    if (!state){
        localPlanner->setPlan(plan);
//        counter++;
//        if (counter == 20){
//            ROS_INFO("Proba uwolnienia z zakleszczenia\n Wykonanie pelnego obrotu");
//            do360();
//            localPlanner->setPlan(plan);
//            counter = 0;
//        }
//    } else {
//        counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Cel osiagniety");
  return true;
}

void getCurrentPosition(nav_msgs::Odometry newPosition){
    currentPosition = newPosition.pose.pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_executor");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("/global_planner/planner/make_plan", plan_and_execute);
  pub = n.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("elektron/mobile_base_controller/odom", 1000, getCurrentPosition);
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


float getRotZ(geometry_msgs::Quaternion q){
    float siny_cosp;
    float cosy_cosp;
    float yaw;
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
}

void do360(float rotSpeed, float frequency){
    geometry_msgs::Twist twist = geometry_msgs::Twist();
    twist.angular.z = rotSpeed;
    float startingAngle;
    startingAngle = getRotZ(currentPosition.orientation);

    // Trzeba chwile poczekac, zeby robot mial szanse sie calkowicie obrocic, tzn by nastepna petla nie zatrzymala sie od razu
    int iterationsToSkip = 10;

    ros::Rate rate(frequency);  // Nadawanie ze stala czestotliwoscia

    for (int i=0; i < iterationsToSkip; ++i){
        pub.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    float katRobota;
    while (1){
        twist.angular.z = rotSpeed;

        // Pobieranie aktualnego obrotu robota i spradzenie czy osiagnal on zadany obrot (z zadana tolerancja)
        katRobota = getRotZ(currentPosition.orientation);
        if (fabs(startingAngle - katRobota) < 0.01)
            break;
        pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }
    twist.angular.z = 0.0;
    pub.publish(twist);
}
