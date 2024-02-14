cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(waypoints_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server 
						rclcpp 
						std_msgs 
						rclcpp_action 
						geometry_msgs 
						nav_msgs   
						rosidl_default_generators 
						action_msgs
						tf2
						waypoints_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # GTest stuff
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/tortoisebot_waypoints_test.cpp
      )

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

  install(TARGETS
        ${PROJECT_NAME}_test
		DESTINATION lib/${PROJECT_NAME})

  # GTest stuff
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  tortoisebot_action_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
