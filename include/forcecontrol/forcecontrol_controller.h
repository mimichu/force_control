#pragma once
#ifndef _FORCECONTROL_CONTROLLER_H_
#define _FORCECONTROL_CONTROLLER_H_


#include <fstream>
#include <chrono>

#include <Eigen/Geometry>

#include <forcecontrol/forcecontrol_hardware.h>

class ForceControlController
{
public:
  ForceControlController();
  ~ForceControlController();
  bool init(ros::NodeHandle& root_nh, ForceControlHardware *hw, std::chrono::high_resolution_clock::time_point time0);

  ///
  /// Set the pose command. Next update() will use this command.
  /// WARNING remember to call update() after setPose, before updateAxis.
  ///
  void setPose(const float *pose);
  void setForce(const float *force);
  bool update(const ros::Time& time, const ros::Duration& period);
  void updateAxis(Eigen::Matrix3f T, int n_af);
  ///
  /// Reset the intermediate variables in the control law, including setting
  ///     all position offsets to zero. "previous pose command" is set to the
  ///     robot's current pose.
  /// It's as if the robot was commanded to its current pose and stabilized.
  /// Call reset() only if the next action is computed based on the robot's
  ///     current pose instead of a pre-planned trajectory.
  void reset(); // reset the state variables in the control law
  void displayStates();

  float *_pose_user_input;
  Eigen::Matrix<float, 6, 1> _wrench_Tr_set;

  // parameters
  float _dt; // used for integration/differentiation
  Eigen::Matrix<float, 6, 1> _damping_coef;
  Eigen::Matrix<float, 6, 6> _ToolInertiaMatrix;
  Eigen::Matrix<float, 6, 6> _ToolStiffnessMatrix;
  float _kForceControlPGain, _kForceControlIGain, _kForceControlDGain;

  float _FC_I_Limit;

  // Controller internal
  Eigen::Matrix3f _T;
  int _n_af;
  Eigen::Matrix4f _SE3_WToffset;
  float *_pose_sent_to_robot;
  Eigen::Vector3f _v_force_selection;
  Eigen::Vector3f _v_velocity_selection;

  Eigen::Vector3f _f_TErr;
  Eigen::Vector3f _f_TErr_I;
  Eigen::Vector3f _f_TAll_old{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _v_T{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _v_T_old{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _p_W{0.0f, 0.0f, 0.0f};

private:
  ForceControlHardware *_hw;

  // misc
  std::chrono::high_resolution_clock::time_point _time0; ///< high resolution timer.
  ofstream _file;
  bool _print_flag;

};

#endif // _FORCECONTROL_CONTROLLER_H_
