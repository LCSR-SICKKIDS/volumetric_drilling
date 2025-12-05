cmake_minimum_required(VERSION 3.3)
project(volumetric_drilling_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

# first test for ROS1
set (ROS1_depends std_msgs geometry_msgs)

find_package (catkin QUIET
  COMPONENTS message_generation ${ROS1_depends})

# ROS 1
if (catkin_FOUND)

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    Index.msg
    Voxels.msg
    DrillSize.msg
    VolumeInfo.msg
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages (DEPENDENCIES ${ROS1_depends})
  
  catkin_package (
    CATKIN_DEPENDS message_runtime ${ROS1_depends}
    )
  
else (catkin_FOUND)
  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS2_depends ament_cmake rclcpp std_msgs geometry_msgs builtin_interfaces rosidl_default_generators)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

    rosidl_generate_interfaces(volumetric_drilling_msgs
      msg/Index.msg
      msg/Voxels.msg
      msg/DrillSize.msg
      msg/VolumeInfo.msg
      DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
      )

    ament_package ()

  endif (ament_cmake_FOUND)

endif (catkin_FOUND)