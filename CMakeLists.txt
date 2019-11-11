cmake_minimum_required(VERSION 2.8.3)
project(arm_behavior)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
#if(COMPILER_SUPPORTS_MARCH_NATIVE)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#endif()

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  roscpp
  rviz_visual_tools
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  moveit_msgs
  tf2
  tf2_geometry_msgs
  tf
)


find_package(Boost REQUIRED)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




#ARM TEST NODES
#add_executable(arm_sequence src/arm_sequence.cpp 
#              classes/arm_nodes.cpp 
#              classes/arm_class.cpp
##              classes/entropy.cpp classes/binsegmentation.cpp classes/pointpose.cpp 
#              classes/other_nodes.cpp 
#              classes/pointcloud_node.cpp 
#)
#target_link_libraries(arm_sequence ${catkin_LIBRARIES})
#target_link_libraries (arm_sequence ${PCL_LIBRARIES})
#target_link_libraries(arm_sequence ${Boost_LIBRARIES})


#add_executable(test_arm_nodes src/test_arm_nodes.cpp 
#              classes/arm_nodes.cpp 
#              classes/arm_class.cpp)
#target_link_libraries(test_arm_nodes ${catkin_LIBRARIES})


add_executable(gripper_sequence src/gripper_sequence.cpp 
              classes/gripper_nodes.cpp 
              classes/gripper_class.cpp)
target_link_libraries(gripper_sequence ${catkin_LIBRARIES})

add_executable(full_sequence src/full_sequence.cpp 
              classes/arm_nodes.cpp 
              classes/arm_class.cpp
              classes/entropy.cpp classes/binsegmentation.cpp classes/pointpose.cpp 
              classes/other_nodes.cpp 
              classes/pointcloud_node.cpp
              classes/gripper_nodes.cpp 
              classes/gripper_class.cpp 
)
target_link_libraries(full_sequence ${catkin_LIBRARIES})
target_link_libraries (full_sequence ${PCL_LIBRARIES})
target_link_libraries(full_sequence ${Boost_LIBRARIES})

add_executable(cloud_sequence src/cloud_sequence.cpp 
              classes/entropy.cpp classes/binsegmentation.cpp classes/pointpose.cpp 
              classes/other_nodes.cpp 
              classes/pointcloud_node.cpp
)
target_link_libraries(cloud_sequence ${catkin_LIBRARIES})
target_link_libraries (cloud_sequence ${PCL_LIBRARIES})
target_link_libraries(cloud_sequence ${Boost_LIBRARIES})
