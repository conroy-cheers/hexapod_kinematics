// Copyright (c) 2021 Conroy Cheers

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>
#include <hexapod_kinematics/hexapod_kinematics.h>
#include <vector>
#include <iostream>
#include <random>

using Catch::Matchers::Approx;


void test_rpy_to_quaternion(const Eigen::Vector3d & rpy, const std::vector<double> & expected)
{
  auto quat = hexkins::rpy_to_quaternion(rpy);
  std::vector<double> quat_vector({quat.w(), quat.x(), quat.y(), quat.z()});
  REQUIRE_THAT(quat_vector, Approx(expected).margin(0.001));
}


void test_rpy(const Eigen::Vector3d & rpy)
{
  auto quat = hexkins::rpy_to_quaternion(rpy);
  auto rpy_converted = hexkins::quaternion_to_rpy(quat);

  std::vector<double> orig({rpy[0], rpy[1], rpy[2]});
  std::vector<double> converted({rpy_converted[0], rpy_converted[1], rpy_converted[2]});

  REQUIRE_THAT(converted, Approx(orig).margin(0.001));
}


void test_inverse_kinematics(
  const hexkins::HexapodConfig & config,
  const Eigen::Vector3d & pos,
  const Eigen::Vector3d & rpy,
  const std::vector<double> & expected)
{
  auto quat = hexkins::rpy_to_quaternion(rpy);
  auto strut_lengths_array = hexkins::inverse_kinematics(config, pos, quat);
  std::vector<double> strut_lengths(strut_lengths_array.begin(), strut_lengths_array.end());
  REQUIRE_THAT(strut_lengths, Approx(expected).margin(0.001));
}


TEST_CASE("rpy conversions", "[main]") {
  SECTION("roll") {
    test_rpy({0.2, 0, 0});
  }
  SECTION("pitch") {
    test_rpy({0, 0.2, 0});
  }
  SECTION("yaw") {
    test_rpy({0, 0, 0.2});
  }
  SECTION("roll+pitch") {
    test_rpy({0.2, 0.2, 0});
  }
  SECTION("roll+yaw") {
    test_rpy({0.2, 0, 0.2});
  }
  SECTION("pitch+yaw") {
    test_rpy({0, 0.2, 0.2});
  }
  SECTION("roll+pitch+yaw") {
    test_rpy({0.2, 0.2, 0.2});
  }
}


TEST_CASE("rpy to external quaternion", "[main]") {
  SECTION("base") {
    test_rpy_to_quaternion({0, 0, 0}, {1, 0, 0, 0});
  }
  SECTION("roll") {
    test_rpy_to_quaternion({0.2, 0, 0}, {0.995, 0.099833, 0, 0});
  }
  SECTION("pitch") {
    test_rpy_to_quaternion({0, 0.2, 0}, {0.995, 0, 0.099833, 0});
  }
  SECTION("yaw") {
    test_rpy_to_quaternion({0, 0, 0.2}, {0.995, 0, 0, 0.099833});
  }
  SECTION("roll+pitch") {
    test_rpy_to_quaternion({0.2, 0.2, 0}, {0.99003, 0.099335, 0.099335, -0.0099667});
  }
  SECTION("roll+yaw") {
    test_rpy_to_quaternion({0.2, 0, 0.2}, {0.99003, 0.099335, 0.0099667, 0.099335});
  }
  SECTION("pitch+yaw") {
    test_rpy_to_quaternion({0, 0.2, 0.2}, {0.99003, -0.0099667, 0.099335, 0.099335});
  }
  SECTION("roll+pitch+yaw") {
    test_rpy_to_quaternion({0.2, 0.2, 0.2}, {0.98608, 0.088921, 0.10876, 0.088921});
  }
}


TEST_CASE("kinematics", "[main]") {
  hexkins::HexapodConfig config;
  config.base_joints = {{
    {2.57940852063914, 0.797904557985617, 0},
    {1.98070987733051, 1.83488102661872, 0},
    {-1.98070987733051, 1.83488102661872, 0},
    {-2.57940852063914, 0.797904557985617, 0},
    {-0.598698643308632, -2.63278558460434, 0},
    {0.598698643308631, -2.63278558460434, 0},
  }};
  config.platform_joints = {{
    {0.955336489125606, -0.295520206661340, 0},
    {0.221740238262456, 0.975105772075681, 0},
    {-0.221740238262455, 0.975105772075681, 0},
    {-0.955336489125606, -0.295520206661339, 0},
    {-0.733596250863151, -0.679585565414341, 0},
    {0.733596250863150, -0.679585565414341, 0},
  }};
  config.kins_max_iterations = 1000;
  config.kins_fwd_max_retries = 20;

  SECTION("inverse kinematics base") {
    test_inverse_kinematics(
      config, {0, 0, 3}, {0, 0, 0}, {3.5823, 3.5823, 3.5823, 3.5823, 3.5823, 3.5823});
  }
  SECTION("inverse kinematics yaw") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0, 0, 0.3}, {3.0629, 3.3441, 3.0629, 3.3441, 3.0629, 3.3441});
  }
  SECTION("inverse kinematics pitch") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0, 0.2, 0}, {3.0385, 3.1433, 3.2126, 3.3362, 3.2908, 3.0614});
  }
  SECTION("inverse kinematics roll") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0.2, 0, 0}, {3.1273, 3.3351, 3.3351, 3.1273, 3.0789, 3.0789});
  }
  SECTION("inverse kinematics roll+pitch") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0.2, 0.2, 0}, {2.9993, 3.2785, 3.3903, 3.2822, 3.1948, 2.9688});
  }
  SECTION("inverse kinematics roll+yaw") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0.2, 0, 0.2}, {3.0446, 3.4353, 3.2588, 3.2356, 2.993, 3.19});
  }
  SECTION("inverse kinematics pitch+yaw") {
    test_inverse_kinematics(
      config, {0, 0, 2.5}, {0, 0.2, 0.2}, {2.9529, 3.2524, 3.1306, 3.4376, 3.2136, 3.1703});
  }

  SECTION("forward kinematics") {
    unsigned seed = 42;
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> pos_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> pos_fuzz(-0.05, 0.05);
    std::uniform_real_distribution<double> ori_dist(-0.15, 0.15);
    std::uniform_real_distribution<double> ori_fuzz(-0.05, 0.05);
    std::uniform_real_distribution<double> arm_fuzz(-0.00001, 0.00001);

    constexpr int samples = 50000;
    int failures = 0;

    for (int i = 0; i < samples; ++i) {
      // generate random position + orientation
      Eigen::Vector3d pos = {pos_dist(gen), pos_dist(gen), pos_dist(gen) + 2.5};
      Eigen::Vector3d rpy = {ori_dist(gen), ori_dist(gen), ori_dist(gen)};
      auto ori = hexkins::rpy_to_quaternion(rpy);

      // compute arm lengths for this pose
      auto strut_lengths = hexkins::inverse_kinematics(config, pos, ori);

      // apply fuzz to returned arm lengths
      std::transform(
        strut_lengths.begin(), strut_lengths.end(), strut_lengths.begin(),
        [&](double d) {return d + arm_fuzz(gen);});

      // generate fuzzed position + orientation
      Eigen::Vector3d fuzz_pos = pos +
        Eigen::Vector3d({pos_fuzz(gen), pos_fuzz(gen), pos_fuzz(gen)});
      Eigen::Vector3d fuzz_rpy = rpy +
        Eigen::Vector3d({ori_fuzz(gen), ori_fuzz(gen), ori_fuzz(gen)});

      // attempt to calculate position from fuzzed arm lengths
      Eigen::Vector3d fwd_pos;
      Eigen::Quaterniond fwd_ori;
      try {
        std::tie(fwd_pos, fwd_ori) = hexkins::forward_kinematics(
          config, strut_lengths, fuzz_pos,
          hexkins::rpy_to_quaternion(fuzz_rpy));

        REQUIRE(fwd_ori.isApprox(ori, 1e-3));
        REQUIRE((fwd_pos - pos).norm() < 1e-3);
      } catch (const hexkins::Exception & e) {
        std::ostringstream err_msg;
        err_msg << "Forward kinematics failed: \"" << e.what() << "\".\nGiven position:\n" <<
          fuzz_pos << "\nGiven orientation:\n" << fuzz_rpy << "\nTrue position:\n" <<
          pos << "\nTrue orientation:\n" << rpy;
        ++failures;
      }
    }
    REQUIRE((double) failures / (double) samples < 0.01);  // require <1% failure rate
  }
}
