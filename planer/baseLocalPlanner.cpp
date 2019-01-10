#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>

...

tf::TransformListener tf(ros::Duration(10));
costmap_2d::Costmap2DROS costmap("<nasza costmapa>", tf);

base_local_planner::TrajectoryPlannerROS tp;
tp.initialize("my_trajectory_planner", &tf, &costmap);
