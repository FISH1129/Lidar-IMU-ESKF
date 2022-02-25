<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
=======
cmake_minimum_required(VERSION 3.5.1)
project(lidar_localization)

#set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -lpthread -lprotobuf")
add_compile_options(-std=c++11)
#add_compile_options(-lprotobuf -std=c++11 -lpthread)
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
<<<<<<< HEAD
  pcl_ros
  geometry_msgs
  sensor_msgs
  nav_msgs
=======
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
  tf
  eigen_conversions
  message_generation 
  std_srvs
<<<<<<< HEAD
  roslib
=======
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    # IMU:
    IMUGNSSMeasurement.msg
    # lidar:
    LidarMeasurement.msg
    # ESKF covariance monitor:
    ESKFStd.msg
    # EKF covariance monitor:
    EKFStd.msg
    # synced GNSS-odo measurement:
    PosVel.msg
    # synced GNSS-odo-mag measurement:
    PosVelMag.msg
)

add_service_files(
   FILES
<<<<<<< HEAD
   saveMap.srv
   optimizeMap.srv
   # saveOdomotry.srv
=======
   saveOdometry.srv
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
<<<<<<< HEAD
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)
include(cmake/sophus.cmake)
include(cmake/OpenCV.cmake)
=======
include(cmake/geographic.cmake)
include(cmake/protobuf.cmake)
include(cmake/PCL.cmake)
include(cmake/sophus.cmake)
include(cmake/g2o.cmake)
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
<<<<<<< HEAD
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

# add_executable(GroundPlaneFit_node src/apps/GroundPlaneFit_node.cpp ${ALL_SRCS})
# add_dependencies(GroundPlaneFit_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(GroundPlaneFit_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(loop_closing_node src/apps/loop_closing_node.cpp ${ALL_SRCS})
add_dependencies(loop_closing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_closing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(matching_node src/apps/matching_node.cpp ${ALL_SRCS})
add_dependencies(matching_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(matching_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
=======
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(kitti_filtering_node src/apps/kitti_filtering_node.cpp ${ALL_SRCS})
add_dependencies(kitti_filtering_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(kitti_filtering_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
        data_pretreat_node

        kitti_filtering_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
