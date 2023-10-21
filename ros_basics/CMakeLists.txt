cmake_minimum_required(VERSION 3.0.2)
project(ros_basics)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Build listener

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(system_clock_node src/System_Clock.cpp)
target_link_libraries(system_clock_node ${catkin_LIBRARIES})

add_executable(counter_node src/Counter.cpp)
target_link_libraries(counter_node ${catkin_LIBRARIES})

add_executable(sumup_node src/Sumup.cpp)
target_link_libraries(sumup_node ${catkin_LIBRARIES})
