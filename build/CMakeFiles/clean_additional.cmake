# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/cube_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/cube_autogen.dir/ParseCache.txt"
  "cube_autogen"
  )
endif()
