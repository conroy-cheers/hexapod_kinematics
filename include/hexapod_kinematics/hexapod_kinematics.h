// Copyright (c) 2021 Conroy Cheers

#ifndef HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_
#define HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <string>

namespace hexkins
{

#define NUM_STRUTS 6

class Exception : public std::exception
{
public:
  Exception(int error_code, const std::string & message) noexcept;
  virtual ~Exception() = default;
  const char * what() const noexcept override;

private:
  int error_code;
  std::string message;
};

class ConvergenceFailure : public Exception
{
public:
  explicit ConvergenceFailure(const std::string & message) noexcept;
  virtual ~ConvergenceFailure() = default;
};

#define HEXKINS_INVALID_JOINT_LENGTH 0
#define HEXKINS_MAX_ERROR_EXCEEDED 1
#define HEXKINS_TOO_MANY_ITERATIONS 2
#define HEXKINS_ZERO_STRUT_VECTOR 3

struct HexapodConfig
{
  // base joint locations in base frame
  std::array<Eigen::Vector3d, NUM_STRUTS> base_joints;
  // platform joint locations in platform frame
  std::array<Eigen::Vector3d, NUM_STRUTS> platform_joints;

  double kins_max_allowed_error = 500.0;
  int kins_max_iterations = 120;
  double kins_convergence_criterion = 1e-9;

  int kins_fwd_max_retries = 10;

  bool lead_correction_enable = false;
  double lead_correction_screw_lead = 0.0;
};

Eigen::Quaterniond rpy_to_quaternion(Eigen::Vector3d rpy);

Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond quat);

std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
forward_kinematics(
  const HexapodConfig & config,
  const std::array<double, NUM_STRUTS> & joints,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation);

std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
forward_kinematics(
  const HexapodConfig & config,
  const std::array<double, NUM_STRUTS> & joints,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & base_joint_directions,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & platform_joint_directions);

std::array<double, NUM_STRUTS> inverse_kinematics(
  const HexapodConfig & config,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation);
std::array<double, NUM_STRUTS> inverse_kinematics(
  const HexapodConfig & config,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & base_joint_directions,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & platform_joint_directions);

}  // namespace hexkins

#endif  // HEXAPOD_KINEMATICS__HEXAPOD_KINEMATICS_H_
