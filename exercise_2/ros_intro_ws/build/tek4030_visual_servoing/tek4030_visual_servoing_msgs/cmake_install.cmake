# Install script for directory: /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tek4030_visual_servoing_msgs/msg" TYPE FILE FILES
    "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_msgs/msg/ImageFeaturePoints.msg"
    "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_msgs/msg/FeatureVector.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tek4030_visual_servoing_msgs/cmake" TYPE FILE FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_msgs/catkin_generated/installspace/tek4030_visual_servoing_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/include/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/share/roseus/ros/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/share/common-lisp/ros/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/share/gennodejs/ros/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/python2.7/dist-packages/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/python2.7/dist-packages/tek4030_visual_servoing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_msgs/catkin_generated/installspace/tek4030_visual_servoing_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tek4030_visual_servoing_msgs/cmake" TYPE FILE FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_msgs/catkin_generated/installspace/tek4030_visual_servoing_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tek4030_visual_servoing_msgs/cmake" TYPE FILE FILES
    "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_msgs/catkin_generated/installspace/tek4030_visual_servoing_msgsConfig.cmake"
    "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_msgs/catkin_generated/installspace/tek4030_visual_servoing_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tek4030_visual_servoing_msgs" TYPE FILE FILES "/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_msgs/package.xml")
endif()

