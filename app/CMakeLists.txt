add_executable(run_mapping_online run_mapping_online.cc)
rosidl_target_interfaces(run_mapping_online ${PROJECT_NAME}_msg "rosidl_typesupport_cpp")
add_dependencies(run_mapping_online ${PROJECT_NAME})
ament_target_dependencies(run_mapping_online ${dependencies})
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )

# TODO: Convert offline mapping app to ROS2   
# add_executable(run_mapping_offline run_mapping_offline.cc)
# target_link_libraries(run_mapping_offline
#         ${PROJECT_NAME} gflags
#        )

install(TARGETS
        run_mapping_online
        DESTINATION lib/${PROJECT_NAME}/
        )