add_library(${PROJECT_NAME}
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
