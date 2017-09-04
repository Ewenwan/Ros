# 
# Temporary workaround for issue ros-industrial/industrial_core#46.
# 
message(STATUS "industrial_robot_client: work around for #46")
if (DEFINED industrial_robot_client_LIBRARY_DIRS)
else()
    message(FATAL_ERROR "industrial_robot_client_LIBRARY_DIRS not set, "
        "have you find_package()-ed it?")
endif()
link_directories(${industrial_robot_client_LIBRARY_DIRS})
