/********************************************************************
 * hexapod_kinematics.cpp
 *
 * Kinematics for a generalised hexapod machine
 * A C++/Eigen port of earlier works by Andrew Kyrychenko & R. Brian Register.
 *
 * License: GPLv2
 *
 * Copyright (c) 2004 All rights reserved.
 *
*********************************************************************/

#include "hexapod_kinematics/hexapod_kinematics.h"

#include <Eigen/Dense>
#include <exception>
#include <stdexcept>
#include <vector>
#include <tuple>
#include <iostream>
#include <random>
#include <string>

namespace hexkins
{

Exception::Exception(int error_code, const std::string & message) noexcept
: error_code(error_code), message(message) {}

const char * Exception::what() const noexcept
{
  return this->message.c_str();
}

ConvergenceFailure::ConvergenceFailure(const std::string & message) noexcept
: Exception(HEXKINS_MAX_ERROR_EXCEEDED, message) {}

Eigen::Quaterniond rpy_to_quaternion(Eigen::Vector3d rpy)
{
  Eigen::AngleAxisd roll_angle(rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(rpy[2], Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond quat)
{
  return quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

static std::array<Eigen::Vector3d, NUM_STRUTS> dummy_joint_info;

double strut_length_correction(
  const double & screw_lead,
  const Eigen::Vector3d & StrutVectUnit,
  const Eigen::Matrix3d & RMatrix,
  const Eigen::Vector3d & joint_unit_base,
  const Eigen::Vector3d & joint_unit_platform)
{
  Eigen::Vector3d nb2, nb3, na1, na2;
  double dotprod;

  /* define base joints axis vectors */
  nb2 = joint_unit_base.cross(StrutVectUnit);
  nb3 = StrutVectUnit.cross(nb2);
  nb3.normalize();

  /* define platform joints axis vectors */
  na1 = RMatrix * joint_unit_platform;
  na2 = na1.cross(StrutVectUnit);
  na2.normalize();

  /* define dot product */
  dotprod = nb3.dot(na2);

  return screw_lead * asin(dotprod) / (2 * EIGEN_PI);
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
forward_kinematics_impl(
  const HexapodConfig & config,
  const std::array<double, NUM_STRUTS> & joints,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & base_joint_directions,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & platform_joint_directions)
{
  Eigen::Vector3d aw;
  Eigen::Vector3d InvKinStrutVect, InvKinStrutVectUnit;
  Eigen::Vector3d q_trans, RMatrix_a, RMatrix_a_cross_Strut;

  Eigen::Matrix<double, NUM_STRUTS, NUM_STRUTS> Jacobian;
  Eigen::Matrix<double, NUM_STRUTS, NUM_STRUTS> InverseJacobian;
  Eigen::Matrix<double, NUM_STRUTS, 1> StrutLengthDiff;
  Eigen::Matrix<double, NUM_STRUTS, 1> delta;
  double InvKinStrutLength;
  double conv_err = 1.0;

  Eigen::Matrix3d RMatrix;
  Eigen::Vector3d q_RPY;

  int iterate = 1;
  int i;
  int iteration = 0;

  /* abort on obvious problems, like joints <= 0 */
  /* FIXME-- should check against triangle inequality, so that joints
   are never too short to span shared base and platform sides */
  if (joints[0] <= 0.0 || joints[1] <= 0.0 || joints[2] <= 0.0 || joints[3] <= 0.0 ||
    joints[4] <= 0.0 || joints[5] <= 0.0)
  {
    throw Exception(HEXKINS_INVALID_JOINT_LENGTH, "Non-positive joint length present");
  }

  /* assign a,b,c to roll, pitch, yaw angles */
  q_RPY = quaternion_to_rpy(current_orientation);

  /* Assign translation values in pos to q_trans */
  q_trans = current_position;

  /* Enter Newton-Raphson iterative method   */
  while (iterate) {
    /* check for large error and return error flag if no convergence */
    if ((conv_err > +config.kins_max_allowed_error) ||
      (conv_err < -config.kins_max_allowed_error))
    {
      /* we can't converge */
      throw ConvergenceFailure("Failed to converge");
    }

    iteration++;

    /* check iteration to see if the kinematics can reach the
   convergence criterion and return error flag if it can't */
    if (iteration > config.kins_max_iterations) {
      /* we can't converge */
      throw Exception(HEXKINS_TOO_MANY_ITERATIONS, "Too many iterations");
    }

    /* Convert q_RPY to Rotation Matrix */
    RMatrix = rpy_to_quaternion(q_RPY).toRotationMatrix();

    /* compute StrutLengthDiff[] by running inverse kins on Cartesian
    estimate to get joint estimate, subtract joints to get joint deltas,
    and compute inv J while we're at it */
    for (i = 0; i < NUM_STRUTS; i++) {
      RMatrix_a = RMatrix * config.platform_joints[i];
      aw = q_trans + RMatrix_a;
      InvKinStrutVect = aw - config.base_joints[i];

      Eigen::Vector3d normed = InvKinStrutVect.normalized();
      if (normed == InvKinStrutVect) {
        // zero vector
        throw Exception(HEXKINS_ZERO_STRUT_VECTOR, "A strut vector has zero length");
      }
      InvKinStrutVectUnit = normed;
      InvKinStrutLength = InvKinStrutVect.norm();

      if (config.lead_correction_enable) {
        InvKinStrutLength += strut_length_correction(
          config.lead_correction_screw_lead, InvKinStrutVectUnit, RMatrix, base_joint_directions[i],
          platform_joint_directions[i]);
      }

      StrutLengthDiff[i] = InvKinStrutLength - joints[i];

      /* Determine RMatrix_a_cross_strut */
      RMatrix_a_cross_Strut = RMatrix_a.cross(InvKinStrutVectUnit);

      /* Build Inverse Jacobian Matrix */
      InverseJacobian.row(i).segment(0, 3) = InvKinStrutVectUnit;
      InverseJacobian.row(i).segment(3, 3) = RMatrix_a_cross_Strut;
    }

    /* invert Inverse Jacobian */
    Jacobian = InverseJacobian.inverse();

    /* multiply Jacobian by LegLengthDiff */
    delta = Jacobian * StrutLengthDiff;

    /* subtract delta from last iterations pos values */
    q_trans -= delta.segment(0, 3);
    q_RPY -= delta.segment(3, 3);

    /* determine value of conv_error (used to determine if no convergence) */
    conv_err = 0.0;
    for (i = 0; i < NUM_STRUTS; i++) {
      conv_err += fabs(StrutLengthDiff[i]);
    }

    /* enter loop to determine if a strut needs another iteration */
    iterate = 0;     /*assume iteration is done */
    for (i = 0; i < NUM_STRUTS; i++) {
      if (fabs(StrutLengthDiff[i]) > config.kins_convergence_criterion) {
        iterate = 1;
      }
    }
  }   /* exit Newton-Raphson Iterative loop */

  Eigen::Quaterniond outputOrientation = rpy_to_quaternion(q_RPY);
  Eigen::Vector3d outputPosition = q_trans;

  /* throw error if max iterations exceeded */
  if (iteration > config.kins_max_iterations) {
    throw Exception(HEXKINS_TOO_MANY_ITERATIONS, "Too many iterations");
  }

  return {outputPosition, outputOrientation};
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
forward_kinematics(
  const HexapodConfig & config,
  const std::array<double, NUM_STRUTS> & joints,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & base_joint_directions,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & platform_joint_directions)
{
  Eigen::Vector3d try_pos = current_position;
  Eigen::Quaterniond try_ori = current_orientation;

  /* Outer retry loop to explore local minima */
  unsigned seed = 42;
  std::default_random_engine gen(seed);
  double avg_arm_length = std::accumulate(
    joints.begin(),
    joints.end(), 0.0, [](double a, double b) {return a + b;}) / NUM_STRUTS;
  double jitter_amt = avg_arm_length / 50.0;
  std::normal_distribution<double> pos_jt(-jitter_amt, jitter_amt);
  std::normal_distribution<double> rot_jt(-0.2, 0.2);

  Eigen::Vector3d output_pos;
  Eigen::Quaterniond output_ori;
  bool converged;

  for (int retries = 0; retries < config.kins_fwd_max_retries; ++retries) {
    converged = true;
    try {
      std::tie(output_pos, output_ori) = forward_kinematics_impl(
        config, joints, try_pos, try_ori,
        base_joint_directions, platform_joint_directions);
    } catch (ConvergenceFailure & e) {
      converged = false;
    }

    double pos_diff = (output_pos - current_position).norm();
    if (converged && (pos_diff < 5e-1) && output_ori.isApprox(current_orientation, 1e-1)) {
      return {output_pos, output_ori};
    }

    // retry with jitter applied to input
    try_pos = current_position + Eigen::Vector3d({pos_jt(gen), pos_jt(gen), pos_jt(gen)});

    auto random_rot = Eigen::AngleAxisd(rot_jt(gen), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(rot_jt(gen), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rot_jt(gen), Eigen::Vector3d::UnitZ());
    try_ori = current_orientation * random_rot;
  }

  throw ConvergenceFailure("Failed to converge after retries.");
}

// alternate version, for use with strut length correction disabled
std::tuple<Eigen::Vector3d, Eigen::Quaterniond>
forward_kinematics(
  const HexapodConfig & config,
  const std::array<double, NUM_STRUTS> & joints,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation)
{
  if (!config.lead_correction_enable) {
    return forward_kinematics(
      config, joints, current_position, current_orientation,
      dummy_joint_info, dummy_joint_info);
  } else {
    throw std::invalid_argument(
            "Lead correction is enabled in config. Joint info must be provided.");
  }
}

/* the inverse kinematics take world coordinates and determine joint values,
   given the inverse kinematics flags to resolve any ambiguities. The forward
   flags are set to indicate their value appropriate to the world coordinates
   passed in. */
std::array<double, NUM_STRUTS> inverse_kinematics(
  const HexapodConfig & config,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & base_joint_directions,
  const std::array<Eigen::Vector3d, NUM_STRUTS> & platform_joint_directions)
{
  Eigen::Vector3d aw, temp;
  Eigen::Vector3d InvKinStrutVect, InvKinStrutVectUnit;
  Eigen::Matrix3d RMatrix;
  int i;
  double InvKinStrutLength, corr;

  std::array<double, NUM_STRUTS> strut_lengths;

  /* define Rotation Matrix */
  RMatrix = current_orientation.toRotationMatrix();

  /* enter for loop to calculate joints (strut lengths) */
  for (i = 0; i < NUM_STRUTS; i++) {
    /* convert location of platform strut end from platform to world coordinates */
    aw = current_position + (RMatrix * config.platform_joints[i]);

    /* define strut lengths */
    InvKinStrutVect = aw - config.base_joints[i];
    InvKinStrutLength = InvKinStrutVect.norm();

    if (config.lead_correction_enable) {
      /* enable strut length correction */
      /* define unit strut vector */
      InvKinStrutVectUnit = InvKinStrutVect.normalized();
      if (InvKinStrutVectUnit == InvKinStrutVect) {
        // zero vector; something is wrong
        throw Exception(HEXKINS_ZERO_STRUT_VECTOR, "A strut vector has zero length");
      }
      /* define correction value and corrected joint lengths */
      corr = strut_length_correction(
        config.lead_correction_screw_lead, InvKinStrutVectUnit, RMatrix, base_joint_directions[i],
        platform_joint_directions[i]);
      InvKinStrutLength += corr;
    }

    strut_lengths[i] = InvKinStrutLength;
  }

  return strut_lengths;
}

// alternate version, for use with strut length correction disabled
std::array<double, NUM_STRUTS> inverse_kinematics(
  const HexapodConfig & config,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation)
{
  if (!config.lead_correction_enable) {
    return inverse_kinematics(
      config, current_position, current_orientation, dummy_joint_info, dummy_joint_info);
  } else {
    throw std::invalid_argument(
            "Lead correction is enabled in config. Joint info must be provided.");
  }
}

std::array<Eigen::Vector3d, NUM_STRUTS> get_joint_vectors(
  const HexapodConfig & config,
  const Eigen::Vector3d & current_position,
  const Eigen::Quaterniond & current_orientation)
{
  Eigen::Vector3d aw;
  Eigen::Matrix3d RMatrix;
  std::array<Eigen::Vector3d, NUM_STRUTS> struts;

  /* define Rotation Matrix */
  RMatrix = current_orientation.toRotationMatrix();

  /* enter for loop to calculate joints (strut lengths) */
  for (int i = 0; i < NUM_STRUTS; i++) {
    /* convert location of platform strut end from platform to world coordinates */
    aw = current_position + (RMatrix * config.platform_joints[i]);
    struts[i] = aw - config.base_joints[i];
  }

  return struts;
}

}  // namespace hexkins
