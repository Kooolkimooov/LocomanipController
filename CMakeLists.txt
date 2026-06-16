cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME locomanip_controller)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/LocomanipController)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(locomanip_controller LANGUAGES CXX)

OPTION(ENABLE_CNOID "Install Choreonoid files" OFF)
OPTION(ENABLE_MUJOCO "Install MuJoCo files" ON)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(ENABLE_CNOID)
  set(CNOID_ROS_UTILS cnoid_ros2_utils)
else()
  set(CNOID_ROS_UTILS "")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  message(WARNING "CATKIN_DEVEL_PREFIX is defined, but this package is being migrated to ROS 2")
endif()

set(DOXYGEN_HTML_OUTPUT doxygen-html)
set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/LocomanipController.yaml")

add_project_dependency(baseline_walking_controller REQUIRED)
if(ENABLE_CNOID AND NOT "${CNOID_ROS_UTILS}" STREQUAL "")
  # CnoidRosUtils might not have a ROS 2 equivalent with the same name, 
  # but we'll try to find it if needed or skip if it's not converted yet.
  # For now, let's assume it's handled by add_project_dependency or find_package.
endif()

configure_file(etc/LocomanipController.in.yaml ${CONFIG_OUT} @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

configure_file(etc/mc_rtc.in.yaml etc/mc_rtc.yaml @ONLY)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/LocomanipController DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")

add_subdirectory(description)

if(ENABLE_CNOID)
  add_subdirectory(cnoid)
endif()

if(ENABLE_MUJOCO)
  add_subdirectory(mujoco)
endif()

if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

set(PACKAGE_EXTRA_MACROS "set(LOCOMANIP_CONTROLLER_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/locomanip_controller/states\")
set(LOCOMANIP_CONTROLLER_STATES_FILES \"\${LOCOMANIP_CONTROLLER_STATES_LIBRARIES}/data\")")

ament_package()
