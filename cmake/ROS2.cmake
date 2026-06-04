#
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# the rosbag api changed between distros
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR
      $ENV{ROS_DISTRO} STREQUAL "galactic")
    add_definitions(-DUSE_OLD_ROSBAG_API)
  endif()
else()
  message(ERROR "ROS_DISTRO environment variable is not set!")
endif()

add_definitions(-DRESCALE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(OpenCV REQUIRED)

set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "dvs_msgs"
  "prophesee_event_msgs"
  "event_camera_msgs"
  "event_camera_codecs"
  "rosbag2_storage"
  "rosbag2_cpp")

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL "0.26.1")
  add_definitions(-DUSE_ROSBAG2_STORAGE_RECV_TIME)
endif()

# -------- conversion tools
ament_auto_add_executable(legacy_to_bag src/legacy_to_bag_ros2.cpp)

# -------- republish node and composable
ament_auto_add_library(republish SHARED src/republish_composable.cpp)
target_include_directories(republish PRIVATE include)
rclcpp_components_register_nodes(republish "event_camera_legacy_tools::RepublishComposable")

ament_auto_add_executable(republish_node  src/republish_node_ros2.cpp)


# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  legacy_to_bag
  republish_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  republish
  DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_black()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
