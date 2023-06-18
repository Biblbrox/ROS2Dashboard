# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/ROS2Dashboard_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/ROS2Dashboard_autogen.dir/ParseCache.txt"
  "ROS2Dashboard_autogen"
  )
endif()
