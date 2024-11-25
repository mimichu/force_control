#include "force_control/config_deserialize.h"
#include <RobotUtilities/spatial_utilities.h>

#include <yaml-cpp/yaml.h>

#include "force_control/admittance_controller.h"

template <>
bool deserialize(const YAML::Node& node,
                 AdmittanceController::AdmittanceControllerConfig& config) {
  try {
    config.dt = node["dt"].as<double>();
    config.log_to_file = node["log_to_file"].as<bool>();
    config.log_file_path = node["log_file_path"].as<std::string>();
    config.compliance6d.stiffness = RUT::deserialize_vector<RUT::Vector6d>(
                                        node["compliance6d"]["stiffness"])
                                        .asDiagonal();
    config.compliance6d.damping =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["damping"])
            .asDiagonal();
    config.compliance6d.inertia =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["inertia"])
            .asDiagonal();
    config.max_spring_force_magnitude =
        node["max_spring_force_magnitude"].as<double>();
    config.direct_force_control_gains.P_trans =
        node["direct_force_control_gains"]["P_trans"].as<double>();
    config.direct_force_control_gains.I_trans =
        node["direct_force_control_gains"]["I_trans"].as<double>();
    config.direct_force_control_gains.D_trans =
        node["direct_force_control_gains"]["D_trans"].as<double>();
    config.direct_force_control_gains.P_rot =
        node["direct_force_control_gains"]["P_rot"].as<double>();
    config.direct_force_control_gains.I_rot =
        node["direct_force_control_gains"]["I_rot"].as<double>();
    config.direct_force_control_gains.D_rot =
        node["direct_force_control_gains"]["D_rot"].as<double>();
    config.direct_force_control_I_limit =
        RUT::deserialize_vector<RUT::Vector6d>(
            node["direct_force_control_I_limit"]);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  return true;
}
