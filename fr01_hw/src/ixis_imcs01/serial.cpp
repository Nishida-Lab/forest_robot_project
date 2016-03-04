#include <ixis_imcs01/serial.hpp>

SerialPort::SerialPort()
{
}

SerialPort::SerialPort(std::string portName)
{
  openPort(portName);
}

void SerialPort::openPort(std::string portName)
{
  fd_ = open(portName.c_str(), O_RDWR);
  if(fd_ < 0)
    {
      ROS_WARN("%s: Open error", portName.c_str());
      exit(-1);
    }
  if (ioctl(fd_, URBTC_CONTINUOUS_READ) < 0){
    ROS_WARN("ioctl: URBTC_CONTINUOUS_READ error");
    exit(1);
  }
  if (ioctl(fd_, URBTC_BUFREAD) < 0){
    ROS_WARN("ioctl: URBTC_CONTINUOUS_READ error");
    exit(1);
  }  
}

SerialPort::~SerialPort()
{
  close(fd_);
}
