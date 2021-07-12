#inlcude <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

sensor_msgs::Imu imu;
sensor_msgs::PointCloud2 

void imu_callback(const sensor_msgs::Imu &msg) {

}

void pc_callback(const sensor_msgs::PointCloud2 &msg) {

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "timestamp_changer");
  ros::NodeHandle nh;
  ros::Subscriber imu_sub = nh.subscribe("/os1_cloud_node/imu/data", 100, imu_callback);
  ros::Subscriber pc_sub = nh.subscribe("/os1_cloud_node/points", 100, pc_callback);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/changed_", 100);
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/changed_", 100);
  
  


  ros::spin();
  return 0;


}