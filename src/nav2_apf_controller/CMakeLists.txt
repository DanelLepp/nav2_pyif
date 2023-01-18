cmake_minimum_required(VERSION 3.8)
project(nav2_apf_controller)
set(WRAPPERS "python_wrappers")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_executable(apf_test src/apf_test.cpp)

set(library_name nav2_apf_controller)

# target_include_directories(${library_name} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${WRAPPERS}>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include/${WRAPPERS}>)
# target_compile_features(${library_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS apf_test
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
)

set(dependencies
  nav2_common
  nav2_core
  nav2_costmap_2d
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  geometry_msgs
  pluginlib
  PythonLibs
  tf2_ros
  builtin_interfaces
)



add_library(${library_name} SHARED
        src/${library_name}.cpp
)

target_include_directories(apf_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(apf_test
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC 
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

pluginlib_export_plugin_description_file(nav2_core ${library_name}.xml)

install(TARGETS ${library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
