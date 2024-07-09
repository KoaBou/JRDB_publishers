project(jrdb_publishers)

cmake_minimum_required(VERSION 3.5)

#message("Your C++ compiler supports these C++ features:")
#foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
#  message("${i}")
#endforeach()



find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system) 
# find_package(PCL COMPONENTS common REQUIRED)
find_package(PCL REQUIRED)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(dependencies
    OpenCV
    geometry_msgs
    visualization_msgs
    pcl_conversions
    rcl_interfaces
    rclcpp
    rcutils
    sensor_msgs
    std_msgs
    tf2 
    tf2_ros
    tf2_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

set(ROS2_JRDB_PUBLISHERS_SOURCES
  src/node/jrdb_publishers_node.cpp
)

add_library(${PROJECT_NAME} SHARED ${ROS2_JRDB_PUBLISHERS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(jrdb_publisher
  src/jrdb_publishers_main.cpp
)
ament_target_dependencies(jrdb_publisher ${dependencies})
target_compile_definitions(jrdb_publisher PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


target_link_libraries(jrdb_publisher
  ${PROJECT_NAME}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  jrdb_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()