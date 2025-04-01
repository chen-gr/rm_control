#include "rm_dart/dart_arm.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_control");
  SerialControl node;
  ros::spin();
  return 0;
}