#include <fr01_lum_slam/lum_slam.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <boost/progress.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <fstream>

#define foreach BOOST_FOREACH


LumSLAM::LumSLAM()
  : rate_(10),saved_counter_(0)
{
  init();
  // local_matching_clouds_.resize(2);
  // local_matching_clouds_[0].setSavePointCloudHeader("first_");
  // local_matching_clouds_[0].setInitCount(0);
  // local_matching_clouds_[0].setLimitMatchingCount(30);
  // local_matching_clouds_[1].setSavePointCloudHeader("second_");
  // local_matching_clouds_[1].setInitCount(-15);
  // local_matching_clouds_[1].setLimitMatchingCount(30);
  pose_output_file_name_ = ros::package::getPath("fr01_lum_slam") + "/poseoutput_" + timeToStr() + ".csv";
  local_matched_clouds_.resize(100);
}

void LumSLAM::init()
{
  ros::NodeHandle n("~");
  // rosparam の設定
  scanner_frame_ = n.param<std::string>("scanner_frame", "scanner_link");
  ROS_INFO_STREAM("scanner_frame : " << scanner_frame_);
  scanner_topic_ = n.param<std::string>("scanner_topic", "/point_cloud");
  ROS_INFO_STREAM("scanner_topic : " << scanner_topic_);
  base_frame_ = n.param<std::string>("base_frame", "base_footprint");
  ROS_INFO_STREAM("base_frame : " << base_frame_);
  ndt_odom_frame_ = n.param<std::string>("ndt_odom_frame", "ndt_odom");
  ROS_INFO_STREAM("ndt_odom_frame : " << ndt_odom_frame_);
  icp_odom_frame_ = n.param<std::string>("icp_odom_frame", "icp_odom");
  ROS_INFO_STREAM("icp_odom_frame : " << icp_odom_frame_);
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
  ndt_.setResolution (1.0);
  ndt_.setMaximumIterations (30);
}

LumSLAM::~LumSLAM()
{
  std::cout << "Pose saved file : " << pose_output_file_name_ << std::endl;
}

void LumSLAM::startLiveSlam()
{
  point_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, scanner_topic_, 5);
  point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_sub_, tf_, base_frame_, 5);
  point_cloud_filter_->registerCallback(boost::bind(&LumSLAM::scanMatchingCallback, this, _1));
  transform_thread_ = new boost::thread(boost::bind(&LumSLAM::publishLoop, this, transform_publish_period_));
}

void LumSLAM::startReplay(const std::string &bag_name)
{
  double transform_publish_period;
  transform_thread_ = new boost::thread(boost::bind(&LumSLAM::publishLoop, this, transform_publish_period_));

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
    // for (size_t i = 0; i < 10; ++i) {
    //   s_queue.pop();
    // }

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

void LumSLAM::getRPY(const geometry_msgs::Quaternion &q,
                             double &roll,double &pitch,double &yaw){
  tf::Quaternion tfq(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
}


void LumSLAM::cropBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
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

void LumSLAM::scanMatchingCallback(const sensor_msgs::PointCloud2::ConstPtr& points)
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
    ROS_ERROR("%s", e.what());
    return;
  } catch(...){
    std::cerr << "Exception Occured!!!" << std::endl;
    exit(-1);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  if(initial_scan_loaded_ == 0)
  {
    last_scan_ = *scan_ptr;
    initial_scan_loaded_ = 1;
    local_matched_clouds_[0] = *scan_ptr;
    count_++;
    //ROS_INFO_STREAM("Initial scan loaded.");
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(last_scan_));

  ndt_.setInputSource(scan_ptr);
  ndt_.setInputTarget(last_scan_ptr);

  tf::Matrix3x3 init_rotation;
  guess_pos_ = previous_pos_; // ほとんど移動していないという仮定

  Eigen::AngleAxisf init_rotation_x(guess_pos_.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pos_.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pos_.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(guess_pos_.x, guess_pos_.y, guess_pos_.z);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ros::Time matching_start = ros::Time::now();
  ndt_.align(*output_cloud_ptr, init_guess); // ndt でマッチングする
  t = ndt_.getFinalTransformation(); // ndtで変換行列をゲットする
  ros::Duration matching_delta_t = ros::Time::now() - matching_start;

  // for NDT
  tf::Matrix3x3 tf3d;
  tf3d.setValue(
                static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

  current_pos_.x = t(0, 3);
  current_pos_.y = t(1, 3);
  current_pos_.z = t(2, 3);
  tf3d.getRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw, 1);

  map2ndt_odom_.setOrigin(tf::Vector3(current_pos_.x, current_pos_.y, current_pos_.z));
  q.setRPY(current_pos_.roll, current_pos_.pitch, current_pos_.yaw);
  map2ndt_odom_.setRotation(q);

  std::cout << "/////////////////////////////////////////////" << std::endl;
  std::cout << "count : " << count_ << std::endl;
  std::cout << "Process time : " << matching_delta_t.toSec() << std::endl;
  std::cout << "NDT has converged: " << ndt_.hasConverged() << " score: "
            << ndt_.getFitnessScore() << std::endl;
  std::cout << "x : " << current_pos_.x << std::endl;
  std::cout << "y : " << current_pos_.y << std::endl;
  std::cout << "z : " << current_pos_.z << std::endl;
  //std::cout << "/////////////////////////////////////////////" << std::endl;

  sensor_msgs::PointCloud2 scan_matched;
  //pcl::toROSMsg(*output_cloud_ptr, scan_matched);
  pcl::toROSMsg(scan, scan_matched);

  scan_matched.header.stamp = scan_time;
  scan_matched.header.frame_id = ndt_odom_frame_;
  point_cloud_pub_.publish(scan_matched);
  // Update position and posture. current_pos -> previous_pos
  previous_pos_ = current_pos_;

  // save current scan
  last_scan_ += *output_cloud_ptr;

  local_matched_clouds_[count_%10] = *output_cloud_ptr;
  if ((count_%9) == 0) {
    pcl::PointCloud<pcl::PointXYZI> sum_pointcloud;
    for (int i = 0; i < 10; ++i) {
      sum_pointcloud += local_matched_clouds_[i];
    }
    saved_counter_++;
    this->savePointCloud(saved_counter_, sum_pointcloud);
    this->outputPose(saved_counter_, current_pos_);
    sum_pointcloud.clear();
  }

  count_++;
}

void LumSLAM::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

void LumSLAM::publishTransform()
{
  //map2odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  br_->sendTransform(tf::StampedTransform(map2ndt_odom_, tf_expiration,
                                          map_frame_, ndt_odom_frame_));
  //map2odom_mutex_.unlock();
}

void LumSLAM::savePointCloud(std::string dstfilename)
{
  // std::string resultfilepath = ros::package::getPath("fr01_ndt_mapping");
  // resultfilepath += "/result";
  // resultfilepath += getTimeAsString();
  // resultfilepath += ".pcd";
  pcl::io::savePCDFileASCII(dstfilename, last_scan_);
  ROS_INFO_STREAM("Saved " << last_scan_.points.size() << "data points to result.");
  ROS_INFO_STREAM("Saved to " << dstfilename);
}

void LumSLAM::savePointCloud(int saved_counter_, pcl::PointCloud<pcl::PointXYZI> pointcloud)
{
  std::stringstream ss;
  ss << std::setw(10) << std::setfill('0') << saved_counter_ << ".pcd";
  pcl::io::savePCDFileASCII(ros::package::getPath("fr01_lum_slam") + "/" +ss.str(), pointcloud);
  std::cout << "Saved : " << ss.str() << std::endl;
  return;
}

void LumSLAM::outputPose(int saved_counter, Position pose)
{
  pose_output_.open(pose_output_file_name_.c_str(), std::ios::app);
  pose_output_ << saved_counter_ << " " << pose.x << " " << pose.y
               << " " << pose.z << " " << pose.roll << " " << pose.pitch
               << " " << pose.yaw << std::endl;
  pose_output_.close();
}

std::string LumSLAM::timeToStr()
{
  std::stringstream msg;
  const boost::posix_time::ptime now=
    boost::posix_time::second_clock::local_time();
  boost::posix_time::time_facet *const f=
    new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
  msg.imbue(std::locale(msg.getloc(),f));
  msg << now;
  return msg.str();
}
