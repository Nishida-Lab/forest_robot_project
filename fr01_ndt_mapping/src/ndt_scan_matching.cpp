#include <ndt_scan_matching.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <boost/progress.hpp>
#include <pcl/filters/extract_indices.h>
#define foreach BOOST_FOREACH

NDTScanMatching::NDTScanMatching()
  : rate_(10)
{
  init();
}

void NDTScanMatching::init()
{
  ros::NodeHandle n("~");
  // rosparam の設定
  scanner_frame_ = n.param<std::string>("scanner_frame", "scanner_link");
  ROS_INFO_STREAM("scanner_frame : " << scanner_frame_);
  scanner_topic_ = n.param<std::string>("scanner_topic", "/point_cloud");
  ROS_INFO_STREAM("scanner_topic : " << scanner_topic_);
  base_frame_ = n.param<std::string>("base_frame", "base_footprint");
  ROS_INFO_STREAM("base_frame : " << base_frame_);
  odom_frame_ = n.param<std::string>("odom_frame", "ndt_odom");
  ROS_INFO_STREAM("odom_frame : " << odom_frame_);
  map_frame_ = n.param<std::string>("map_frame", "map");
  ROS_INFO_STREAM("map_frame : " << map_frame_);
  n.param("transform_publish_period", transform_publish_period_, 0.05);
  ROS_INFO_STREAM("transform_publish_period : " << transform_publish_period_);
  n.param<int>("skip_num", skip_num_, 0);
  ROS_INFO_STREAM("skip_num : " << skip_num_);
  tf_delay_ = transform_publish_period_;

  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_match_point_cloud", 1);
  br_ = new tf::TransformBroadcaster();

 
  offset_x_ = 0;
  offset_y_ = 0;
  offset_z_ = 0;
  offset_yaw_ = 0;
  last_yaw_ = 0;
  current_pos_.x = 0;
  current_pos_.y = 0;
  current_pos_.z = 0;
  current_pos_.roll = 0;
  current_pos_.pitch = 0;
  current_pos_.yaw = 0;

  previous_pos_.x = 0;
  previous_pos_.y = 0;
  previous_pos_.z = 0;
  previous_pos_.roll = 0;
  previous_pos_.pitch = 0;
  previous_pos_.yaw = 0;

  last_pose_.pose.position.x = 0;
  last_pose_.pose.position.y = 0;
  last_pose_.pose.position.z = 0;
  last_pose_.pose.orientation.x = 0;
  last_pose_.pose.orientation.y = 0;
  last_pose_.pose.orientation.z = 0;
  last_pose_.pose.orientation.w = 1;

  guess_pos_.x = 0;
  guess_pos_.y = 0;
  guess_pos_.z = 0;
  guess_pos_.roll = 0;
  guess_pos_.pitch = 0;
  guess_pos_.yaw = 0;

  initial_scan_loaded_ = 0;
  count_ = 0;

  map2ndt_odom_.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  map2ndt_odom_.setRotation(q);

  // NDT setting
  ndt_.setTransformationEpsilon (0.01);
  ndt_.setStepSize (0.1);
  ndt_.setResolution (1.5);
  ndt_.setMaximumIterations (30);
}

void NDTScanMatching::startLiveSlam()
{
  point_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, scanner_topic_, 5);
  point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_sub_, tf_, base_frame_, 5);
  point_cloud_filter_->registerCallback(boost::bind(&NDTScanMatching::scanMatchingCallback, this, _1));
  transform_thread_ = new boost::thread(boost::bind(&NDTScanMatching::publishLoop, this, transform_publish_period_));
}

void NDTScanMatching::startReplay(const std::string &bag_name)
{
  double transform_publish_period;
  transform_thread_ = new boost::thread(boost::bind(&NDTScanMatching::publishLoop, this, transform_publish_period_));

  ros::NodeHandle private_nh_("~");

  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(scanner_topic_);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  //Store up to 5 messages and there error message
  std::queue<std::pair<sensor_msgs::PointCloud2::ConstPtr, std::string> > s_queue;
  boost::progress_display show_progress(viewall.size());
  int replay_count = 0;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if(cur_tf != NULL){
      for (size_t i = 0;  i < cur_tf->transforms.size(); ++i ) {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }
    sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
    if(s != NULL){
      if(!(ros::Time(s->header.stamp)).is_zero()){
        s_queue.push(std::make_pair(s, ""));
      }
      // Just like in live processing, only process the latest 5 scans
      if(s_queue.size() > 5){
        ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
        s_queue.pop();
      }
      // ignoring un-timestamped tf data
    }
    // Only process a scan if it has tf data
    while(!s_queue.empty())
    {
      if(replay_count < skip_num_){
        replay_count += 1;
        s_queue.pop();
        continue;
      }else{
        replay_count = 0;
        try {
          tf::StampedTransform t;
          tf_.lookupTransform(s_queue.front().first->header.frame_id, base_frame_,
                              s_queue.front().first->header.stamp, t);
          this->scanMatchingCallback(s_queue.front().first);
          // br_->sendTransform(tf::StampedTransform(map2ndt_odom_, ros::Time::now(),
          //                                       map_frame_, odom_frame_));
          s_queue.pop();
        } catch (tf::TransformException& e) {
          s_queue.front().second = std::string(e.what());
          break;
        } catch(...){
          ROS_WARN_STREAM("Any exception");
          break;
        }
      }
      ++show_progress;
    }
  }
  bag.close();
  //savePointCloud();
}

void NDTScanMatching::getRPY(const geometry_msgs::Quaternion &q,
                             double &roll,double &pitch,double &yaw){
  tf::Quaternion tfq(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
}


void NDTScanMatching::cropBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered)
{
  pcl::search::KdTree<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  pcl::PointXYZI search_point;
  search_point.x = 0;
  search_point.y = 0;
  search_point.z = 0;
  double search_radius = 0.5;
  std::vector<float> point_radius_squared_distance;
  pcl::PointIndices::Ptr point_idx_radius_search(new pcl::PointIndices());

  if ( kdtree.radiusSearch (search_point, search_radius, point_idx_radius_search->indices, point_radius_squared_distance) > 0 )
  {
    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(point_idx_radius_search);
    extractor.setNegative(true);
    extractor.filter(*cloud_filtered);
  }
}

void NDTScanMatching::scanMatchingCallback(const sensor_msgs::PointCloud2::ConstPtr& points)
{
  ros::Time scan_time = ros::Time::now();

  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

  // ROSのメッセージからPCLの形式に変換
  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_pc(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*points, *trans_pc);
  // robotにあたった点群を除去する
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_radius_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  this->cropBox(trans_pc, cloud_radius_filtered);
  // 点群をhokuyo3d座標系からbase_link座標系に変換
  try {
    pcl_ros::transformPointCloud(base_frame_, points->header.stamp, *cloud_radius_filtered, scanner_frame_, scan, tf_);
    //pcl_ros::transformPointCloud(base_frame_, points->header.stamp, *trans_pc, scanner_frame_, scan, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }catch(ros::Exception e){
    ROS_ERROR("ros::Exception %s", e.what());
  }catch(std::exception &e){
    std::cerr << e.what() << std::endl;
    exit(-1);
  } catch(...){
    std::cerr << "Exception Occured!!!" << std::endl;
    exit(-1);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  if(initial_scan_loaded_ == 0)
  {
    last_scan_ = *scan_ptr;
    initial_scan_loaded_ = 1;
    //ROS_INFO_STREAM("Initial scan loaded.");
    return;
  }
  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (1.0, 1.0, 1.0);
  approximate_voxel_filter.setInputCloud (scan_ptr);
  approximate_voxel_filter.filter (*filtered_cloud_ptr);
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  // ndt_.setTransformationEpsilon (0.01);
  // // Setting maximum step size for More-Thuente line search.
  // ndt_.setStepSize (0.1);
  // //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  // ndt_.setResolution (1.0);
  // // Setting max number of registration iterations.
  // ndt_.setMaximumIterations (30);
  // Setting point cloud to be aligned.
  
  ndt_.setInputSource (filtered_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(last_scan_));
  ndt_.setInputTarget (last_scan_ptr);
  //ndt_.setInputSource(scan_ptr);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(last_scan_));
  // Setting point cloud to be aligned to.
  //ndt_.setInputTarget (last_scan_ptr);

  tf::Matrix3x3 init_rotation;

  // 一個前のposeと引き算してx, y ,zの偏差を出す
  offset_x_ = 0;//odom->pose.pose.position.x - last_pose_.pose.position.x;
  offset_y_ = 0;//odom->pose.pose.position.y - last_pose_.pose.position.y;
  double roll, pitch, yaw = 0;
  //getRPY(odom->pose.pose.orientation, roll, pitch, yaw);
  offset_yaw_ = 0;//yaw - last_yaw_;

  guess_pos_.x = previous_pos_.x + offset_x_;
  guess_pos_.y = previous_pos_.y + offset_y_;

  guess_pos_.z = previous_pos_.z;
  guess_pos_.roll = previous_pos_.roll;
  guess_pos_.pitch = previous_pos_.pitch;
  guess_pos_.yaw = previous_pos_.yaw + offset_yaw_;
  Eigen::AngleAxisf init_rotation_x(guess_pos_.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pos_.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pos_.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(guess_pos_.x, guess_pos_.y, guess_pos_.z);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  ros::Time ndt_start = ros::Time::now();
  ndt_.align (*output_cloud_ptr, init_guess);
  ros::Duration ndt_delta_t = ros::Time::now() - ndt_start;
 

  t = ndt_.getFinalTransformation();

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*scan_ptr, *output_cloud_ptr, t);

  tf::Matrix3x3 tf3d;
  tf3d.setValue(
		static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
		static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
		static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

  current_pos_.x = t(0, 3);
  current_pos_.y = t(1, 3);
  current_pos_.z = t(2, 3);

  std::cout << "/////////////////////////////////////////////" << std::endl;
  std::cout << "count : " << count_ << std::endl;
  std::cout << "Process time : " << ndt_delta_t.toSec() << std::endl;
  std::cout << "NDT has converged :" << ndt_.hasConverged () << std::endl;
  std::cout << " score : " << ndt_.getFitnessScore () << std::endl;
  std::cout << "x : " << current_pos_.x << std::endl;
  std::cout << "y : " << current_pos_.y << std::endl;
  std::cout << "z : " << current_pos_.z << std::endl;
  //std::cout << "/////////////////////////////////////////////" << std::endl;

  tf3d.getRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw, 1);

  map2ndt_odom_mutex_.lock();
  //transform.setOrigin(tf::Vector3(current_pos_.x, current_pos_.y, current_pos_.z));
  map2ndt_odom_.setOrigin(tf::Vector3(current_pos_.x, current_pos_.y, current_pos_.z));
  q.setRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw);
  //transform.setRotation(q);
  map2ndt_odom_.setRotation(q);
  map2ndt_odom_mutex_.unlock();
  // "map"に対する"base_link"の位置を発行する
  // br_.sendTransform(tf::StampedTransform(transform, scan_time, "map", "ndt_base_link"));

  sensor_msgs::PointCloud2 scan_matched;
  //pcl::toROSMsg(*output_cloud_ptr, scan_matched);
  pcl::toROSMsg(scan, scan_matched);
  //scan_matched = *points;
  
  scan_matched.header.stamp = scan_time;
  //scan_matched.header.frame_id = "matched_point_cloud";
  scan_matched.header.frame_id = odom_frame_;

  point_cloud_pub_.publish(scan_matched);

  // Update position and posture. current_pos -> previous_pos
  previous_pos_ = current_pos_;

  // save current scan
  last_scan_ += *output_cloud_ptr;
  //last_pose_.pose = odom->pose.pose;
  last_yaw_ = yaw;
  count_++;
}

void NDTScanMatching::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

void NDTScanMatching::publishTransform()
{
  map2ndt_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  br_->sendTransform(tf::StampedTransform(map2ndt_odom_, tf_expiration,
                                          map_frame_, odom_frame_));
  map2ndt_odom_mutex_.unlock();
}

void NDTScanMatching::savePointCloud(std::string dstfilename)
{
  // std::string resultfilepath = ros::package::getPath("fr01_ndt_mapping");
  // resultfilepath += "/result";
  // resultfilepath += getTimeAsString();
  // resultfilepath += ".pcd";
  pcl::io::savePCDFileASCII(dstfilename, last_scan_);
  ROS_INFO_STREAM("Saved " << last_scan_.points.size() << "data points to result.");
  ROS_INFO_STREAM("Saved to " << dstfilename);
}
