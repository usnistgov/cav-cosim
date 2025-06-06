cmake_minimum_required(VERSION 3.5)
project(carla_ad_agent)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

  catkin_python_setup()

  catkin_package(CATKIN_DEPENDS rospy)

  if(CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
  endif()

  include_directories(${catkin_INCLUDE_DIRS})

  install(PROGRAMS src/carla_ad_agent/ad_agent.py
                   src/carla_ad_agent/local_planner.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

elseif(${ROS_VERSION} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rclpy REQUIRED)
  ament_export_dependencies(rclpy)
        
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
        
  ament_package()
        
endif()
