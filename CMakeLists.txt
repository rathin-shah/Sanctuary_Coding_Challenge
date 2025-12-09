cmake_minimum_required(VERSION 3.5)
project(planar_kinematics)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl REQUIRED)
if(NOT orocos_kdl_FOUND)
  find_package(Orocos-KDL REQUIRED)
endif()
message(STATUS "KDL include dirs: ${orocos_kdl_INCLUDE_DIRS}")
message(STATUS "KDL libraries: ${orocos_kdl_LIBRARIES}")
message(STATUS "Orocos-KDL include: ${Orocos-KDL_INCLUDE_DIRS}")
message(STATUS "Orocos-KDL libs: ${Orocos-KDL_LIBRARIES}")
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(GTest REQUIRED)

include_directories(
  include
  /usr/include             # <-- ADD THIS LINE
  /usr/include/eigen3
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)



# library (header-only file used, but create a convenience library)
add_library(planar3r_lib INTERFACE)
target_include_directories(planar3r_lib INTERFACE include)

find_library(URDFDOM_MODEL_LIB urdfdom_model)
find_library(URDFDOM_WORLD_LIB urdfdom_world)
# fk node
add_executable(fk_node src/fk_node.cpp)
ament_target_dependencies(fk_node rclcpp geometry_msgs orocos_kdl kdl_parser)
target_link_libraries(fk_node
  planar3r_lib
  Eigen3::Eigen
  orocos-kdl
  ${URDFDOM_MODEL_LIB}
  ${URDFDOM_WORLD_LIB}
)

# ik node
add_executable(ik_node src/ik_node.cpp)
ament_target_dependencies(ik_node rclcpp geometry_msgs)
target_link_libraries(ik_node planar3r_lib)

# # example executable (non-ROS)
# add_executable(example_main src/example_main.cpp)
# target_link_libraries(example_main planar3r_lib)

# tests
if (GTest_FOUND)
  enable_testing()
  ament_add_gtest(test_planar3r test/test_planar3r.cpp)
  if (TARGET test_planar3r)
    target_link_libraries(test_planar3r planar3r_lib)
  endif()
endif()

install(TARGETS fk_node ik_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
