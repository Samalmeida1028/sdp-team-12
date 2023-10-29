# Install script for directory: /mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/e/UMass_Amherst/SDP/sdp-team-12/install/basic_mobile_robot")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE DIRECTORY FILES
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/config"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/launch"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/maps"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/models"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/params"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/rviz"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/src"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/worlds"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/basic_mobile_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/basic_mobile_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot/environment" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot/environment" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_index/share/ament_index/resource_index/packages/basic_mobile_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot/cmake" TYPE FILE FILES
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_core/basic_mobile_robotConfig.cmake"
    "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/ament_cmake_core/basic_mobile_robotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_mobile_robot" TYPE FILE FILES "/mnt/e/UMass_Amherst/SDP/sdp-team-12/basic_mobile_robot/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/mnt/e/UMass_Amherst/SDP/sdp-team-12/build/basic_mobile_robot/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
