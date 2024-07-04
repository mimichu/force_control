#include <RobotUtilities/TimerLinux.h>
#include <RobotUtilities/utilities.h>
#include <force_control/force_control_controller.h>

#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <string>

using RUT::Matrix4d;
using RUT::Matrix6d;
using RUT::MatrixXd;
using RUT::Vector6d;

Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                          "]");

struct ForceControlController::Implementation
{
  Implementation();
  ~Implementation();
  bool initialize(
      const ForceControlControllerConfig &force_control_controller_config,
      const RUT::TimePoint &time_initial_0, const double *pose_current);

  void setRobotStatus(const double *pose_WT, const double *wrench_WT);
  void setRobotReference(const double *pose_WT, const double *wrench_WTr);
  void setForceControlledAxis(const Matrix6d &Tr_new, int n_af);
  int step(double *pose_to_send);
  void reset();
  void displayStates();

  ForceControlControllerConfig config{};

  // internal controller states
  Matrix6d Tr{};
  Matrix6d Tr_inv{};
  Vector6d v_force_selection{};
  Vector6d v_velocity_selection{};
  Matrix6d diag_force_selection{};
  Matrix6d diag_velocity_selection{};
  Matrix6d m_anni{};

  Matrix4d SE3_WTref{};
  Matrix4d SE3_WT{};
  Matrix4d SE3_TrefTadj{};
  Matrix4d SE3_WTadj{};
  Matrix4d SE3_TTadj{};
  Matrix4d SE3_WT_cmd{};
  Vector6d spt_TTadj{};
  Vector6d spt_TTadj_new{};
  Matrix6d Adj_WT{};
  Matrix6d Adj_TW{};
  Matrix6d Jac_v_spt{};
  Matrix6d Jac_v_spt_inv{};

  Vector6d v_spatial_WT{};
  Vector6d v_body_WT{};
  Vector6d v_body_WT_ref{};
  Vector6d v_Tr{};
  Vector6d vd_Tr{};
  Vector6d wrench_T_Err_prev{};
  Vector6d wrench_T_Err_I{};

  Vector6d wrench_T_fb{}; // force feedback measured in tool frame
  Vector6d wrench_Tr_cmd{};
  Vector6d wrench_T_spring{};
  Vector6d wrench_Tr_spring{};
  Vector6d wrench_Tr_fb{};
  Vector6d wrench_T_cmd{};
  Vector6d wrench_T_Err{};
  Vector6d wrench_T_PID{};
  Vector6d wrench_Tr_PID{};
  Vector6d wrench_Tr_Err{};
  Vector6d wrench_Tr_damping{};
  Vector6d wrench_Tr_All{};

  // misc
  RUT::Timer timer{};
  std::ofstream log_file{};
};

ForceControlController::Implementation::Implementation() {}

ForceControlController::Implementation::~Implementation()
{
  if (config.log_to_file)
    log_file.close();
}

bool ForceControlController::Implementation::initialize(
    const ForceControlControllerConfig &force_control_controller_config,
    const RUT::TimePoint &time_initial_0, const double *pose_current)
{
  std::cout << "[ForceControlController] Begin initialization.\n";
  config = force_control_controller_config;
  timer.tic(time_initial_0);

  reset();

  if (config.log_to_file)
  {
    log_file.open(config.log_file_path);
    if (log_file.is_open())
      std::cout << "[ForceControlController] log file opened successfully at "
                << config.log_file_path << std::endl;
    else
      std::cerr << "[ForceControlController] Failed to open log file at "
                << config.log_file_path << std::endl;
  }
  return true;
}

void ForceControlController::Implementation::setRobotStatus(
    const double *pose_WT, const double *wrench_WT)
{
  SE3_WT = RUT::posemm2SE3(pose_WT);
  wrench_T_fb(0) = -wrench_WT[0];
  wrench_T_fb(1) = -wrench_WT[1];
  wrench_T_fb(2) = -wrench_WT[2];
  wrench_T_fb(3) = -wrench_WT[3];
  wrench_T_fb(4) = -wrench_WT[4];
  wrench_T_fb(5) = -wrench_WT[5];
}

void ForceControlController::Implementation::setRobotReference(
    const double *pose_WT, const double *wrench_WTr)
{
  SE3_WTref = RUT::posemm2SE3(pose_WT);

  wrench_Tr_cmd(0) = wrench_WTr[0];
  wrench_Tr_cmd(1) = wrench_WTr[1];
  wrench_Tr_cmd(2) = wrench_WTr[2];
  wrench_Tr_cmd(3) = wrench_WTr[3];
  wrench_Tr_cmd(4) = wrench_WTr[4];
  wrench_Tr_cmd(5) = wrench_WTr[5];
}

// After axis update, the goal pose with offset should be equal to current pose
// in the new velocity controlled axes. To satisfy this requirement, we need to
// change SE3_TrefTadj accordingly
void ForceControlController::Implementation::setForceControlledAxis(
    const Matrix6d &Tr_new, int n_af)
{
  v_force_selection = Vector6d::Zero();
  v_velocity_selection = Vector6d::Ones();
  for (int i = 0; i < n_af; ++i)
  {
    v_force_selection(i) = 1;
    v_velocity_selection(i) = 0;
  }
  diag_force_selection = v_force_selection.asDiagonal();
  diag_velocity_selection = v_velocity_selection.asDiagonal();

  m_anni = diag_velocity_selection * Tr * Jac_v_spt;
  spt_TTadj_new =
      (Matrix6d::Identity() - RUT::pseudoInverse(m_anni, 1e-6) * m_anni) *
      spt_TTadj;
  SE3_TrefTadj = SE3_WT * RUT::spt2SE3(spt_TTadj_new) * RUT::SE3Inv(SE3_WTref);

  // project these into force space
  wrench_T_Err_I = Tr_inv * diag_force_selection * Tr * wrench_T_Err_I;
  wrench_T_Err_prev = Tr_inv * diag_force_selection * Tr * wrench_T_Err_prev;

  Tr = Tr_new;
  Tr_inv = Tr.inverse();

  if (std::isnan(SE3_TrefTadj(0, 0)))
  {
    std::cerr << "\nThe computed offset has NaN." << std::endl;
    std::cerr << "SE3_WT:\n"
              << SE3_WT.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TTadj:\n"
              << SE3_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj:\n"
              << spt_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt_inv:\n"
              << Jac_v_spt_inv.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt:\n"
              << Jac_v_spt.format(MatlabFmt) << std::endl;
    std::cerr << "m_anni:\n"
              << m_anni.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj_new:\n"
              << spt_TTadj_new.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TrefTadj:\n"
              << SE3_TrefTadj.format(MatlabFmt) << std::endl;
    std::cerr << "\nNow paused at setForceControlledAxis()";
    getchar();
  }
}

// clang-format off
/*
 *
    force control law
        Frames:
            W: world frame
            T: current tool frame
            Tr: transformed generalized space
        Frame suffixes
            fb: feedback (default, often omitted)
            ref: user provided reference, target
            adj: offset adjusted (command tool frame)
            cmd: output command, to be sent to the robot
        Quantities:
            SE3: 4x4 homogeneous coordinates
            se3: 6x1 twist coordinate of SE3
            spt: 6x1 special twist: 3x1 position, 3x1 exponential coordinate for rotation
            td: 6x1 time derivative of twist.
            v: 6x1 velocity measured in an inertia frame.
              v_body: body velocity.
              v_spatial: spatial velocity.
            wrench: 6x1 wrench. Makes work with body velocity
            Jac_v_spt: 6x6 jacobian from body velocity to spt:
                Jac_v_spt * body velocity = spt time derivative
            Tr: 6x6 transformation matrix. Describes the force-velocity decomposition
 *
 */
// clang-format on
int ForceControlController::Implementation::step(double *pose_to_send)
{
  // ----------------------------------------
  //  Compute Forces in Generalized space
  // ----------------------------------------
  /* Position updates */
  SE3_WTadj = SE3_TrefTadj * SE3_WTref;
  SE3_TTadj = RUT::SE3Inv(SE3_WT) * SE3_WTadj; // aka SE3_S_err
  spt_TTadj = RUT::SE32spt(SE3_TTadj);

  Jac_v_spt_inv = RUT::JacobianSpt2BodyV(SE3_WT.block<3, 3>(0, 0));
  Jac_v_spt = Jac_v_spt_inv.inverse();

  Adj_WT = RUT::SE32Adj(SE3_WT);
  Adj_TW = RUT::SE32Adj(RUT::SE3Inv(SE3_WT));

  /* Velocity updates */
  v_body_WT = Adj_TW * v_spatial_WT;
  v_Tr = Tr * v_body_WT;

  /* Wrench updates */
  wrench_T_spring = Jac_v_spt * config.compliance6d.stiffness * spt_TTadj;
  wrench_Tr_spring = Tr * wrench_T_spring;

  /* Force error, PID force control */
  wrench_T_cmd = Tr_inv * wrench_Tr_cmd;
  wrench_T_Err = wrench_T_cmd - wrench_T_fb;
  wrench_T_Err_I += wrench_T_Err;
  RUT::truncate6d(&wrench_T_Err_I, -config.direct_force_control_I_limit,
                  config.direct_force_control_I_limit);

  wrench_T_PID.head(3) =
      config.direct_force_control_gains.P_trans * wrench_T_Err.head(3) +
      config.direct_force_control_gains.I_trans * wrench_T_Err_I.head(3) +
      config.direct_force_control_gains.D_trans *
          (wrench_T_Err.head(3) - wrench_T_Err_prev.head(3));
  wrench_T_PID.tail(3) =
      config.direct_force_control_gains.P_rot * wrench_T_Err.tail(3) +
      config.direct_force_control_gains.I_rot * wrench_T_Err_I.tail(3) +
      config.direct_force_control_gains.D_rot *
          (wrench_T_Err.tail(3) - wrench_T_Err_prev.tail(3));
  wrench_Tr_PID = Tr * wrench_T_PID;
  wrench_T_Err_prev = wrench_T_Err;
  wrench_Tr_Err = Tr * wrench_T_Err;

  wrench_Tr_damping = -Tr * config.compliance6d.damping * v_body_WT;

  wrench_Tr_All = diag_force_selection * (wrench_Tr_spring + wrench_Tr_Err +
                                          wrench_Tr_PID + wrench_Tr_damping);

  // ----------------------------------------
  //  force to velocity
  // ----------------------------------------

  /* Newton's Law */
  //  Axes are no longer independent when we take
  //      rotation in to consideration.
  //  Newton's Law in body (Tool) frame:
  //      W=M*vd
  //          W: body wrench
  //          M: Inertia matrix in body frame
  //          vd: body velocity time derivative
  //  Newton's law in transformed space
  //      TW=TMTinv Tvd
  //      W_Tr = TMTinv vd_Tr
  vd_Tr = (Tr * config.compliance6d.inertia * Tr_inv)
              .fullPivLu()
              .solve(wrench_Tr_All);

  // Velocity in the force-controlled direction: integrate acc computed from
  // Newton's law
  v_Tr += config.dt * vd_Tr;
  v_Tr = diag_force_selection *
         v_Tr; // clean up velocity in the velocity-controlled direction

  // Velocity in the velocity-controlled direction: derive from reference pose
  v_body_WT_ref = Jac_v_spt_inv * spt_TTadj /
                  config.dt; // reference velocity, derived from reference pose
  v_Tr += diag_velocity_selection * Tr * v_body_WT_ref;

  v_spatial_WT = Adj_WT * Tr_inv * v_Tr;

  // ----------------------------------------
  //  velocity to pose
  // ----------------------------------------
  SE3_WT_cmd = SE3_WT + RUT::wedge6(v_spatial_WT) * SE3_WT * config.dt;
  RUT::SE32Posemm(SE3_WT_cmd, pose_to_send);

  double timenow = timer.toc_ms();

  if (std::isnan(pose_to_send[0]))
  {
    std::cerr << "==================== pose is nan. =====================\n";
    displayStates();
    std::cerr << "Press ENTER to continue..." << std::endl;
    getchar();
    return false;
  }

  std::cout << "[ForceControlController] Update at " << timenow << " ms."
            << std::endl;
  if (config.log_to_file)
  {
    log_file << timenow << " ";
    RUT::stream_array_in(log_file, SE3_WT.block<3, 1>(0, 3), 3);
    RUT::stream_array_in(log_file, SE3_WTref.block<3, 1>(0, 3), 3);
    RUT::stream_array_in6d(log_file, wrench_T_fb);
    RUT::stream_array_in6d(log_file, wrench_Tr_All);
    RUT::stream_array_in(log_file, pose_to_send, 7);
    log_file << std::endl;
  }
  return true;
}

void ForceControlController::Implementation::reset()
{
  Tr = Matrix6d::Identity();
  Tr_inv = Matrix6d::Identity();
  v_force_selection = Vector6d::Zero();
  v_velocity_selection = Vector6d::Ones();
  diag_force_selection = Matrix6d::Zero();
  diag_velocity_selection = Matrix6d::Identity();
  m_anni = Matrix6d::Identity();

  SE3_WTref = Matrix4d::Identity();
  SE3_WT = Matrix4d::Identity();
  SE3_TrefTadj = Matrix4d::Identity();
  SE3_WTadj = Matrix4d::Identity();
  SE3_TTadj = Matrix4d::Identity();
  SE3_WT_cmd = Matrix4d::Identity();
  spt_TTadj = Vector6d::Zero();
  spt_TTadj_new = Vector6d::Zero();
  Adj_WT = Matrix6d::Identity();
  Adj_TW = Matrix6d::Identity();
  Jac_v_spt = Matrix6d::Identity();
  Jac_v_spt_inv = Matrix6d::Identity();

  v_spatial_WT = Vector6d::Zero();
  v_body_WT = Vector6d::Zero();
  v_body_WT_ref = Vector6d::Zero();
  v_Tr = Vector6d::Zero();
  vd_Tr = Vector6d::Zero();
  wrench_T_Err_prev = Vector6d::Zero();
  wrench_T_Err_I = Vector6d::Zero();

  wrench_T_fb = Vector6d::Zero();
  wrench_Tr_cmd = Vector6d::Zero();
  wrench_T_spring = Vector6d::Zero();
  wrench_Tr_spring = Vector6d::Zero();
  wrench_Tr_fb = Vector6d::Zero();
  wrench_T_cmd = Vector6d::Zero();
  wrench_T_Err = Vector6d::Zero();
  wrench_T_PID = Vector6d::Zero();
  wrench_Tr_PID = Vector6d::Zero();
  wrench_Tr_Err = Vector6d::Zero();
  wrench_Tr_damping = Vector6d::Zero();
  wrench_Tr_All = Vector6d::Zero();
}

void ForceControlController::Implementation::displayStates()
{
  std::cout << "================= Parameters ================== " << std::endl;
  std::cout << "dt: " << config.dt << std::endl;
  std::cout << "log_to_file: " << config.log_to_file << std::endl;
  std::cout << "log_file_path: " << config.log_file_path << std::endl;
  std::cout << "compliance6d.stiffness: "
            << config.compliance6d.stiffness.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.damping: "
            << config.compliance6d.damping.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.inertia: "
            << config.compliance6d.inertia.format(MatlabFmt) << std::endl;
  std::cout << "direct_force_control_gains.P_trans: "
            << config.direct_force_control_gains.P_trans << std::endl;
  std::cout << "direct_force_control_gains.I_trans: "
            << config.direct_force_control_gains.I_trans << std::endl;
  std::cout << "direct_force_control_gains.D_trans: "
            << config.direct_force_control_gains.D_trans << std::endl;
  std::cout << "direct_force_control_gains.P_rot: "
            << config.direct_force_control_gains.P_rot << std::endl;
  std::cout << "direct_force_control_gains.I_rot: "
            << config.direct_force_control_gains.I_rot << std::endl;
  std::cout << "direct_force_control_gains.D_rot: "
            << config.direct_force_control_gains.D_rot << std::endl;
  std::cout << "direct_force_control_I_limit: "
            << config.direct_force_control_I_limit.format(MatlabFmt)
            << std::endl;
  std::cout << "================= Internal states ================== "
            << std::endl;
  std::cout << "Tr: " << Tr.format(MatlabFmt) << std::endl;
  std::cout << "Tr_inv: " << Tr_inv.format(MatlabFmt) << std::endl;
  std::cout << "v_force_selection: " << v_force_selection.format(MatlabFmt)
            << std::endl;
  std::cout << "v_velocity_selection: "
            << v_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_force_selection: "
            << diag_force_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_velocity_selection: "
            << diag_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "m_anni: " << m_anni.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTref: " << SE3_WTref.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT: " << SE3_WT.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TrefTadj: " << SE3_TrefTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTadj: " << SE3_WTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TTadj: " << SE3_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT_cmd: " << SE3_WT_cmd.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj: " << spt_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj_new: " << spt_TTadj_new.format(MatlabFmt)
            << std::endl;
  std::cout << "Adj_WT: " << Adj_WT.format(MatlabFmt) << std::endl;
  std::cout << "Adj_TW: " << Adj_TW.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt: " << Jac_v_spt.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt_inv: " << Jac_v_spt_inv.format(MatlabFmt)
            << std::endl;
  std::cout << "v_spatial_WT: " << v_spatial_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT: " << v_body_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT_ref: " << v_body_WT_ref.format(MatlabFmt)
            << std::endl;
  std::cout << "v_Tr: " << v_Tr.format(MatlabFmt) << std::endl;
  std::cout << "vd_Tr: " << vd_Tr.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err_prev: " << wrench_T_Err_prev.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_Err_I: " << wrench_T_Err_I.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_fb: " << wrench_T_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_cmd: " << wrench_Tr_cmd.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_spring: " << wrench_T_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_spring: " << wrench_Tr_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_fb: " << wrench_Tr_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_cmd: " << wrench_T_cmd.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err: " << wrench_T_Err.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_PID: " << wrench_T_PID.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_PID: " << wrench_Tr_PID.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_Err: " << wrench_Tr_Err.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_damping: " << wrench_Tr_damping.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_All: " << wrench_Tr_All.format(MatlabFmt)
            << std::endl;
}

ForceControlController::ForceControlController()
    : m_impl{std::make_unique<Implementation>()} {}
ForceControlController::~ForceControlController() = default;

bool ForceControlController::init(const ForceControlControllerConfig &config,
                                  const RUT::TimePoint &time0,
                                  const double *pose_current)
{
  m_impl->initialize(config, time0, pose_current);

  double *wrench_zero = new double[6];
  for (int i = 0; i < 6; ++i)
  {
    wrench_zero[i] = 0.;
  }
  double *pose_out = new double[6];
  setRobotStatus(pose_current, wrench_zero);
  setRobotReference(pose_current, wrench_zero);
  step(pose_out);
  setForceControlledAxis(Matrix6d::Identity(), 0);

  std::cout << "[ForceControlController] initialization is done." << std::endl;
  return true;
}

void ForceControlController::setRobotStatus(const double *pose_WT,
                                            const double *wrench_WT)
{
  m_impl->setRobotStatus(pose_WT, wrench_WT);
}

void ForceControlController::setRobotReference(const double *pose_WT,
                                               const double *wrench_WTr)
{
  m_impl->setRobotReference(pose_WT, wrench_WTr);
}

void ForceControlController::setForceControlledAxis(const Matrix6d &Tr_new,
                                                    int n_af)
{
  m_impl->setForceControlledAxis(Tr_new, n_af);
}

int ForceControlController::step(double *pose_to_send)
{
  return m_impl->step(pose_to_send);
}
