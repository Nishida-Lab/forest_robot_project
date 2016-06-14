#include <ndt_scan_matching.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_scan_matching");

  NDTScanMatching ndt_scan_matcher;

  ros::spin();

  return 0;
}
