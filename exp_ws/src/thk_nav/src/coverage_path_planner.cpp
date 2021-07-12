#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::ServiceClient client;
ros::Publisher path_pub;
ros::Subscriber goal_sub;

nav_msgs::Path get_path(geometry_msgs::Pose &start, geometry_msgs::Pose &goal) {
  geometry_msgs::PoseStamped start_stamped;
  start_stamped.header.seq = 0;
  start_stamped.header.stamp = ros::Time::now();
  start_stamped.header.frame_id = "map";
  start_stamped.pose = start;

  geometry_msgs::PoseStamped goal_stamped;
  goal_stamped.header.seq = 0;
  goal_stamped.header.stamp = ros::Time::now();
  goal_stamped.header.frame_id = "map";
  goal_stamped.pose = goal;

  nav_msgs::GetPlan srv;
  srv.request.start = start_stamped;
  srv.request.goal = goal_stamped;
  srv.request.tolerance = 1.5;

  ROS_INFO("Make plan: %d", (client.call(srv) ? 1 : 0));
  ROS_INFO("Plan size: %d", srv.response.plan.poses.size());
  return srv.response.plan;
}

int add_path(geometry_msgs::Pose &start, geometry_msgs::Pose &goal,
             nav_msgs::Path &path)
{
  nav_msgs::Path cur_path;
  cur_path = get_path(start, goal);
  for (auto pose : cur_path.poses) {
    path.poses.push_back(pose);
  }
}

int rot(float x, float y, float theta, float &x_ret, float &y_ret) {
  x_ret = x * cos(theta) - y * sin(theta);
  y_ret = x * sin(theta) + y * cos(theta);

}

bool First_Start = true;
geometry_msgs::Pose start, goal;
// resulting path passing all subscribed goals
nav_msgs::Path path;
void goal_callback(geometry_msgs::PoseWithCovarianceStamped msg) {
  ROS_INFO("Receive new pose");
  if (First_Start) {
    start = msg.pose.pose;
    First_Start = false;
    return;
  }
  goal = msg.pose.pose;
  add_path(start, goal, path);
  start = goal;
  path.header.frame_id = "map";
  path_pub.publish(path);
  ROS_INFO("path length %d", path.poses.size());
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_path_planner");
  ros::NodeHandle nh;
  client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  path_pub = nh.advertise<nav_msgs::Path>("/my_path", 100);
  goal_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/test/goal", 100, goal_callback);

  
  ros::spin();
}