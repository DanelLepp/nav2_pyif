cmake_minimum_required(VERSION 3.8)
project(nav2_pyif)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_compile_definitions(
  PYTHON_LIB="${PYTHON_LIBRARIES}"
)


set(library_name nav2_pyif)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
)

set(dependencies
  nav_msgs
  geometry_msgs
  std_msgs
  PythonLibs
)

add_library(${library_name} SHARED
  src/python_interface.cpp
  src/geo_msgs.cpp
  src/nav_msgs.cpp
  src/std_msgs.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

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

ament_export_include_directories(include ${PYTHON_INCLUDE_DIRS})
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
