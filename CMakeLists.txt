cmake_minimum_required(VERSION 3.8)
project(radeye_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Radeye.msg"
  "srv/ClearRadiationData.srv"
  "srv/GenCSV.srv"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
