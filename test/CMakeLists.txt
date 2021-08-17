# Testing library
fetchcontent_declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.0.0-preview3)
fetchcontent_makeavailable(Catch2)

add_executable(test_kinematics test_kinematics.cpp)
target_include_directories(test_kinematics PUBLIC ../include)
# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(test_kinematics PRIVATE hexapod_kinematics Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(test_kinematics)
