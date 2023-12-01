cmake_minimum_required(VERSION 3.8)
project(radar_ego_velocity_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

enable_language(Fortran CXX)

add_library(odrpack
    src/odrpack/d_odr.f
    src/odrpack/d_mprec.f
    src/odrpack/lpkbls.f
    src/odrpack/dlunoc.f
    src/odrpack/real_precision.f
)


# add_library(${PROJECT_NAME}
#    src/simple_profiler.cpp
#    src/radar_point_cloud.cpp
#    src/odr.cpp
#    src/radar_ego_velocity_estimator.cpp
#    src/radar_body_velocity_estimator.cpp
#    src/radar_ego_velocity_estimator_ros.cpp
#    src/radar_body_velocity_estimator_ros.cpp
# )




# radar_ego_velocity_estimation_ros_node
add_executable(radar_ego_velocity_estimation_ros_node 
  src/nodes/radar_ego_velocity_estimation_ros_node.cpp
  src/simple_profiler.cpp
  src/radar_point_cloud.cpp
  src/odr.cpp
  src/radar_ego_velocity_estimator.cpp
  src/radar_ego_velocity_estimator_ros.cpp
  # src/radar_body_velocity_estimator.cpp
  # src/radar_body_velocity_estimator_ros.cpp
)
target_include_directories(radar_ego_velocity_estimation_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(radar_ego_velocity_estimation_ros_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  radar_ego_velocity_estimation_ros_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "angles"
  "pcl_ros"
  "Eigen3"
  "tf2_eigen"
  "tf2_ros"
)
target_link_libraries(radar_ego_velocity_estimation_ros_node
  odrpack
  )

# add_executable(radar_ego_velocity_estimation_rosbag_node src/nodes/radar_ego_velocity_estimation_rosbag_node.cpp )
# target_link_libraries(radar_ego_velocity_estimation_rosbag_node ${PROJECT_NAME})

# add_executable(radar_body_velocity_estimation_ros_node src/nodes/radar_body_velocity_estimation_ros_node.cpp )
# target_link_libraries(radar_body_velocity_estimation_ros_node ${PROJECT_NAME})

# add_executable(radar_body_velocity_estimation_rosbag_node src/nodes/radar_body_velocity_estimation_rosbag_node.cpp )
# target_link_libraries(radar_body_velocity_estimation_rosbag_node ${PROJECT_NAME})

install(TARGETS radar_ego_velocity_estimation_ros_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

## qtcreator project tree
# FILE(GLOB_RECURSE LibFiles "include/*")
# add_custom_target(headers SOURCES ${LibFiles})

# file(GLOB_RECURSE CFG "cfg/*")
# add_custom_target(CFG SOURCES ${CFG})

# file(GLOB_RECURSE LAUNCH "launch/*")
# add_custom_target(LAUNCH SOURCES ${LAUNCH})

# file(GLOB_RECURSE PYTHON "python/*")
# add_custom_target(PYTHON SOURCES ${PYTHON})

# file(GLOB_RECURSE CONFIG "config/*")
# add_custom_target(CONFIG SOURCES ${CONFIG})
