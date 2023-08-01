list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# for ubuntu 18.04, update gcc/g++ to 9, and download tbb2018 from
# https://github.com/oneapi-src/oneTBB/releases/download/2018/tbb2018_20170726oss_lin.tgz,
# extract it into CUSTOM_TBB_DIR 
# specifiy tbb2018, e.g. CUSTOM_TBB_DIR=/home/idriver/Documents/tbb2018_20170726oss
if (CUSTOM_TBB_DIR)
    set(TBB2018_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2018_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB2018_INCLUDE_DIR})
    link_directories(${TBB2018_LIBRARY_DIR})
endif ()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
        geometry_msgs
        nav_msgs
        sensor_msgs
        rclcpp
        rclpy
        std_msgs
        pcl_ros
        tf2_ros
        tf2_eigen
        livox_interfaces
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_msg
  msg/Pose6D.msg
  DEPENDENCIES
    geometry_msgs
)

install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        include
)
