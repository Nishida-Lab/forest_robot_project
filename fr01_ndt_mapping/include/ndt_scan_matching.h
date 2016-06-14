#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
/* #include <velodyne_pointcloud/point_types.h> */
/* #include <velodyne_pointcloud/rawdata.h> */

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/impl/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/time_synchronizer.h>

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NDTScanMatching
{
public:
  NDTScanMatching();
  void scan_matching_callback(const sensor_msgs::PointCloud2::ConstPtr& points,
                              const nav_msgs::Odometry::ConstPtr& odom);
  void getRPY(const geometry_msgs::Quaternion &q,
              double &roll,double &pitch,double &yaw);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Rate rate_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PC2andOdomSyncPolicy;
  message_filters::Synchronizer<PC2andOdomSyncPolicy> sync_;
  //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_;
  
  ros::Publisher point_cloud_pub_;
  pcl::PointCloud<pcl::PointXYZI> last_scan_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_yaw_;
  double last_yaw_;

  Position current_pos_;
  Position previous_pos_;
  Position guess_pos_;

  geometry_msgs::PoseStamped last_pose_;
  tf::TransformBroadcaster br_;
  tf::TransformListener tf_;
  int initial_scan_loaded_;
  int count_;
};
