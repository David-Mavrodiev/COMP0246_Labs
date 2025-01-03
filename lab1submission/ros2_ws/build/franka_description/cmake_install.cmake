# Install script for directory: /Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/install/franka_description")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/" TYPE DIRECTORY FILES
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/launch"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/robots"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/end_effectors"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/meshes"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/rviz"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/urdfs"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/franka_description")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/franka_description")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/environment" TYPE FILE FILES "/Users/davidm/miniforge3/envs/roboenv-py3.10/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/environment" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/environment" TYPE FILE FILES "/Users/davidm/miniforge3/envs/roboenv-py3.10/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/environment" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_index/share/ament_index/resource_index/packages/franka_description")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description/cmake" TYPE FILE FILES
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_core/franka_descriptionConfig.cmake"
    "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/ament_cmake_core/franka_descriptionConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_description" TYPE FILE FILES "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/src/COMP0246_Labs/franka_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/build/franka_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
