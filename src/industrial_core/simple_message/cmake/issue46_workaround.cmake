# 
# Temporary workaround for issue ros-industrial/industrial_core#46.
# 
message(STATUS "simple_message: work around for #46")
if (DEFINED simple_message_LIBRARY_DIRS)
else()
    message(FATAL_ERROR "simple_message_LIBRARY_DIRS not set, "
        "have you find_package()-ed it?")
endif()
link_directories(${simple_message_LIBRARY_DIRS})
