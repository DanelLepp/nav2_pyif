cmake_minimum_required(VERSION 3.8)
project(nav2_pyif_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(nav2_pyif REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

set(library_name nav2_pyif_controller)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
)

set(dependencies
  nav2_pyif
  nav2_common
  nav2_core
  pluginlib
  PythonLibs
)

add_library(${library_name} SHARED
  src/${library_name}.cpp
)

add_executable(apf_test test/apf_test.cpp src/nav2_pyif_controller.cpp)

ament_target_dependencies(apf_test
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
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
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
