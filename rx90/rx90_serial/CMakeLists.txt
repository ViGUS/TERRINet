cmake_minimum_required(VERSION 2.8.3)
project(rx90_serial)

 if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -Wl,--no-as-needed)
 endif()

add_definitions(-lpthread)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  gazebo_msgs
)
find_package(Threads REQUIRED)

find_package(serial REQUIRED)

generate_messages(DEPENDENCIES std_msgs gazebo_msgs)

catkin_package(

)

include_directories(
   ${catkin_INCLUDE_DIRS}
)


#file(GLOB_RECURSE SRC_FILES_1 "src/*.cpp" "include/*.h" "include/*.inl" )
file(GLOB_RECURSE SRC_FILES_1 "src/test_image.cpp" "src/Rx90.cpp" "src/Rx90.h" )
file(GLOB_RECURSE SRC_FILES_2 "src/test.cpp" "src/Rx90.cpp" "src/Rx90.h" )
file(GLOB_RECURSE SRC_FILES_3 "src/test_file_image.cpp" "src/Rx90.cpp" "src/Rx90.h" )

#file(GLOB_RECURSE SRC_FILES "src/*.cpp" "include/*.h" "include/*.inl" )

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


find_package(realsense2 HINTS "/usr/local/lib/cmake/realsense2")
include_directories(${realsense2_INCLUDE_DIRS})
link_directories(${realsense2_LIBRARY})

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs features2d xfeatures2d calib3d video videoio)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})
# Collect all sources


# Add Executable
#add_executable(${PROJECT_NAME} ${SRC_FILES_1})
#include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)

#target_link_libraries(${PROJECT_NAME} pthread serial)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

include_directories(PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include)


add_executable(rx90_serial_test_image ${SRC_FILES_1})
target_link_libraries(rx90_serial_test_image pthread )
target_link_libraries(rx90_serial_test_image ${catkin_LIBRARIES})
target_link_libraries(rx90_serial_test_image ${serial_LIBRARIES})
target_link_libraries(rx90_serial_test_image ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(rx90_serial_test_image ${GAZEBO_LIBRARIES})
target_link_libraries(rx90_serial_test_image ${realsense2_LIBRARY})
target_link_libraries(rx90_serial_test_image ${OpenCV_LIBS})
add_dependencies(rx90_serial_test_image rx90_serial_generate_messages_cpp)

add_executable(rx90_serial_test ${SRC_FILES_2})
target_link_libraries(rx90_serial_test pthread )
target_link_libraries(rx90_serial_test ${catkin_LIBRARIES})
target_link_libraries(rx90_serial_test ${serial_LIBRARIES})
target_link_libraries(rx90_serial_test ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(rx90_serial_test ${GAZEBO_LIBRARIES})
target_link_libraries(rx90_serial_test ${realsense2_LIBRARY})
target_link_libraries(rx90_serial_test ${OpenCV_LIBS})
add_dependencies(rx90_serial_test rx90_serial_generate_messages_cpp)


add_executable(rx90_serial_file_image ${SRC_FILES_3})
target_link_libraries(rx90_serial_file_image pthread )
target_link_libraries(rx90_serial_file_image ${catkin_LIBRARIES})
target_link_libraries(rx90_serial_file_image ${serial_LIBRARIES})
target_link_libraries(rx90_serial_file_image ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(rx90_serial_file_image ${GAZEBO_LIBRARIES})
target_link_libraries(rx90_serial_file_image ${realsense2_LIBRARY})
target_link_libraries(rx90_serial_file_image ${OpenCV_LIBS})
add_dependencies(rx90_serial_file_image rx90_serial_generate_messages_cpp)

