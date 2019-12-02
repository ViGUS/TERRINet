cmake_minimum_required(VERSION 2.8.3)
project(pid)

add_definitions(-std=c++11)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "release")
ENDIF(NOT DEFINED CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
			std_msgs
			sensor_msgs
			geometry_msgs
			std_srvs
			tf
			roscpp
			rospy
			gazebo_ros
			message_generation
			ardrone_autonomy
			uav_abstraction_layer
)




## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Float32Param.srv
#   Service2.srv
 )



generate_messages(
    DEPENDENCIES std_msgs 
)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs std_srvs gazebo_ros message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
pid/PID.cpp pid/docker_pid.cpp 
)

file(GLOB_RECURSE PID_FILES "pid/*.cpp" "pid/*.h" "pid/*.inl" )
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(ual_pid pid/ual_pid.cpp pid/PID.cpp)
target_link_libraries(ual_pid ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/pid ${PROJECT_SOURCE_DIR}/include)
add_dependencies(ual_pid pid_node_generate_messages_cpp)

add_executable(docker_pid pid/docker_pid.cpp pid/PID.cpp)
target_link_libraries(docker_pid ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/pid ${PROJECT_SOURCE_DIR}/include)
add_dependencies(docker_pid pid_generate_messages_cpp)



add_executable(alex pid/alex.cpp pid/PID.cpp)
target_link_libraries(alex ${catkin_LIBRARIES})
include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/pid ${PROJECT_SOURCE_DIR}/include)
add_dependencies(alex pid_node_generate_messages_cpp)
