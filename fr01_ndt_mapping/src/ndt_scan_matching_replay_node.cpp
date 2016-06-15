#include <ndt_scan_matching.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_scan_matching");

  NDTScanMatching ndt_scan_matcher;
  std::string bagfilepath = ros::package::getPath("fr01_bag");
  ndt_scan_matcher.startReplay(bagfilepath+"/bag/test2.bag", "/hokuyo3d/hokuyo_cloud2");
  ros::spin();

  return 0;
}
