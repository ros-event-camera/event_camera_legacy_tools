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

add_definitions(-DUSING_ROS_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dvs_msgs
  prophesee_event_msgs
  event_camera_msgs
  event_camera_codecs
  rosbag)

find_package(OpenCV REQUIRED)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# --------- conversion tools

add_executable(legacy_to_bag src/legacy_to_bag_ros1.cpp)
target_link_libraries(legacy_to_bag ${catkin_LIBRARIES})

# --------- republish node and nodelet
add_library(republish_nodelet src/republish_nodelet.cpp)
target_link_libraries(republish_nodelet ${catkin_LIBRARIES})

add_executable(republish_node src/republish_node_ros1.cpp)
target_link_libraries(republish_node ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS republish_nodelet republish_node legacy_to_bag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch")

#############
## Testing ##
#############

# To be done...
