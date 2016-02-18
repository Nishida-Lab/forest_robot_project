#ifndef SERIAL_H_
#define SERIAL_H_

#include <iostream>
#include <string>
#include <exception>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <system_error>

#include <ros/ros.h>

#include "imcs01_driver/driver/urbtc.h"   /* Linux specific part */
#include "imcs01_driver/driver/urobotc.h" /* OS independent part */

class SerialPort{
public:
  SerialPort();
  SerialPort(std::string portName);
  ~SerialPort();
  void openPort(std::string portName);
public:
  int fd_;
 private:
  
};

#endif
