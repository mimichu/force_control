/*
    Hardware wrapper for force-control.
    Includes a robot arm and a wrist-mounted ft sensor.
*/
#pragma once
// #include <hardware_interface/robot_hw.h>

#include <hardware_interfaces/ft_interfaces.h>
#include <hardware_interfaces/robot_interfaces.h>
#include <ros/ros.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

typedef std::chrono::high_resolution_clock Clock;

class ForceControlHardware {
 public:
  ForceControlHardware();
  ~ForceControlHardware();
  bool init(ros::NodeHandle &root_nh, Clock::time_point time0, FTInterfaces *ft,
            RobotInterfaces *robot);
  void getPose(double *pose);
  int getWrench(double *wrench);  // get the wrench in tool frame
  int getState(double *pose, double *wrench);
  void setPose(const double *pose_set);

  FTInterfaces *_ft;
  RobotInterfaces *_robot;
};
