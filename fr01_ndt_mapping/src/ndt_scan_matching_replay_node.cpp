#include <ndt_scan_matching.h>
#include <ros/package.h>
#include <boost/program_options.hpp>

std::string getTimeAsString()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y%m%d%I%M%S",timeinfo);
  std::string str(buffer);
  return str;
}


int main(int argc, char **argv)
{
  std::string default_dst_filename = ros::package::getPath("fr01_ndt_mapping");
  default_dst_filename += "/result";
  default_dst_filename += getTimeAsString();
  default_dst_filename += ".pcd";

  boost::program_options::options_description desc("Options");
  desc.add_options()
    ("help", "Print help message")
    ("bag_filename", boost::program_options::value<std::string>()->required(), "ros bag filename")
    ("dst_filename", boost::program_options::value<std::string>()->default_value(default_dst_filename), "result pcd filename");

  boost::program_options::variables_map vm;
  try {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    if( vm.count("help") ){
      std::cout << "NDT based 3D mapping replay node" << std::endl;
      std::cerr << desc << std::endl;
      return 0;
    }
    boost::program_options::notify(vm);
  } catch (boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return -1;
  }

  std::string bagfilename = vm["bag_filename"].as<std::string>();
  std::string dstfilename = vm["dst_filename"].as<std::string>();

  ros::init(argc, argv, "ndt_scan_matching");

  NDTScanMatching ndt_scan_matcher;

  ndt_scan_matcher.startReplay(bagfilename);
  ndt_scan_matcher.savePointCloud(dstfilename);
  ros::spin();

  return 0;
}
