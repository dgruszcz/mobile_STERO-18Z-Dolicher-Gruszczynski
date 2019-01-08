#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "l_cosT_handler");
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);
  costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    local_costmap.start()
    global_costmap.start()

    ROS_INFO("Krece w petli. Juz %d raz.\n", count+1);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

