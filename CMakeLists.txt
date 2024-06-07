# Copyright (C) 2024 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(dvl)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -O2 -fPIC -g)
add_compile_options(${CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(hippo_common REQUIRED)

include_directories(
  include
)

################################################################################
# dvl library
################################################################################
set(lib_name dvl)
add_library(${lib_name}
  STATIC
  src/dvl.cpp
)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/esc_serial/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${lib_name} nlohmann_json::nlohmann_json)
ament_target_dependencies(${lib_name}
  rclcpp
  dvl_msgs
  std_srvs
  hippo_common
)

ament_export_libraries(${lib_name}_export HAS_LIBRARY_TARGET)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

set(exec_name dvl_node)
add_executable(${exec_name}
  src/dvl_node.cpp
  src/dvl_node_params.cpp
)
ament_target_dependencies(${exec_name}
  rclcpp
  dvl_msgs
)
target_link_libraries(${exec_name}
  nlohmann_json::nlohmann_json
  dvl
)

install(TARGETS
  ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


