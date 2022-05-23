cmake_minimum_required(VERSION 3.5)
project(hal_xsens_mti_620)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic 
  -O3 -mtune=native -march=native
  #-g -ggdb
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
)

include_directories( SYSTEM
  /usr/local/xsens/include
)

link_directories(
  /usr/local/xsens/lib
)

add_library(HALMti620 SHARED src/xsens_mti620.cpp src/xdacallback.cpp)
target_link_libraries(HALMti620 
	-lxsensdeviceapi
	-lxstypes
)
ament_target_dependencies(HALMti620 rclcpp rcutils std_msgs sensor_msgs geometry_msgs)

rclcpp_components_register_node(
  HALMti620
  PLUGIN "hal::XsensMti620"
  EXECUTABLE HALMti620Executable)

install(TARGETS HALMti620
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
