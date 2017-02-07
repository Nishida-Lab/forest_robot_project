#ifndef LUM_SLAM_H
#define LUM_SLAM_H

#include <iostream>
#include <string>
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
#include <tf/message_filter.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/impl/transforms.hpp>

#include <message_filters/subscriber.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/time_synchronizer.h>

#include <boost/thread.hpp>

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class LocalMatchingCloud
{
public:
  LocalMatchingCloud()
    : saved_counter_(0), header_("default_")
  {
    limit_matching_count_ = 100;
    counter_ = 0;
  }
  LocalMatchingCloud(int limit_matching_count)
    : limit_matching_count_(limit_matching_count), saved_counter_(0), header_("default_"), counter_(0)
  {
  }
  ~LocalMatchingCloud()
  {
  }
  void setLimitMatchingCount(int limit_matching_count)
  {
    limit_matching_count_ = limit_matching_count;
  }
  void setInitCount(int init_count)
  {
    counter_ = init_count;
  }
  void setSavePointCloudHeader(std::string header)
  {
    header_ = header;
  }
  void addPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud)
  {
    if(counter_ >= 0) {
      pointcloud_ += *source_cloud;
      ROS_INFO_STREAM("Added source_cloud to pointcloud");
      counter_ += 1;
    } else {
      counter_ += 1;
    }
  }
  void resetCounter()
  {
    counter_ = 0;
  }
  bool isAccumulated()
  {
    if (counter_ >= limit_matching_count_) {
      return true;
    } else {
      return false;
    }
  }
  void resetPointCloud()
  {
    pointcloud_.clear();
    counter_ = 0;
  }
  pcl::PointCloud<pcl::PointXYZI> getLocalMatchingCloud()
  {
    return pointcloud_;
  }
  std::string IntToString(int number)
  {
    std::stringstream ss;
    ss << number;
    return ss.str();
  }
  void savePointCloud()
  {
    std::string dstfilename = header_ + IntToString(saved_counter_) + ".pcd";
    pcl::io::savePCDFileASCII(dstfilename, pointcloud_);
    ROS_INFO_STREAM("Saved " << pointcloud_.points.size() << "data points to result.");
    ROS_INFO_STREAM("Saved to " << dstfilename);
    saved_counter_ += 1;
  }
  int saved_counter_;
  int counter_;
  int limit_matching_count_;
  std::string header_;
  pcl::PointCloud<pcl::PointXYZI> pointcloud_;
};

class LumSLAM
{
public:
  LumSLAM();
  ~LumSLAM();
  void init();
  void scanMatchingCallback(const sensor_msgs::PointCloud2::ConstPtr& points);
  void getRPY(const geometry_msgs::Quaternion &q,
              double &roll,double &pitch,double &yaw);
  void publishLoop(double transform_publish_period);
  void publishTransform();
  void startLiveSlam();
  void startReplay(const std::string &bag_name);
  void savePointCloud(std::string dstfilename);
  void savePointCloud(int saved_counter_, pcl::PointCloud<pcl::PointXYZI> pointcloud);
  std::string timeToStr();
  void outputPose(int saved_counter, Position pose);
private:
  std::string getTimeAsString();
  void cropBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Rate rate_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_sub_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;

  ros::Publisher point_cloud_pub_;
  ros::Publisher icp_point_cloud_pub_;
  pcl::PointCloud<pcl::PointXYZI> last_scan_;
  pcl::PointCloud<pcl::PointXYZI> icp_last_scan_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_yaw_;
  double last_yaw_;

  Position current_pos_;
  Position previous_pos_;
  Position guess_pos_;

  Position icp_current_pos_;
  Position icp_previous_pos_;
  Position icp_guess_pos_;
  
  geometry_msgs::PoseStamped last_pose_;
  tf::TransformBroadcaster *br_;
  tf::TransformListener tf_;
  tf::Transform map2ndt_odom_;
  tf::Transform map2icp_odom_;
  int initial_scan_loaded_;
  int count_;

  boost::thread* transform_thread_;

  double transform_publish_period_;
  double tf_delay_;

  std::string scanner_frame_;
  std::string scanner_topic_;
  std::string base_frame_;
  std::string ndt_odom_frame_;
  std::string icp_odom_frame_;
  std::string map_frame_;

  boost::mutex map2odom_mutex_;

  int skip_num_;
  int saved_counter_;
  std::vector<LocalMatchingCloud> local_matching_clouds_;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > local_matched_clouds_;

  std::ofstream pose_output_;
  std::string pose_output_file_name_;
  int keep_clouds_size_;
};

#endif /* LUM_SLAM_H */
