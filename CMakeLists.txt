cmake_minimum_required(VERSION 3.0.2)
project(intention_recognition_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  intention_recognition_msgs
  actionlib 
  tables_demo_planning
)

catkin_package(
  CATKIN_DEPENDS roscpp intention_recognition_msgs actionlib tables_demo_planning
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(RRL_SOURCE
src/rrlib/rrlib_hotel.cpp
src/rrlib/rr_parser.cpp
src/rrlib/rrlib.cpp
src/rageplan/hotel_robot.cpp
src/rageplan/hotel_worker.cpp
src/rageplan/beliefstate.cpp
src/rageplan/coord.cpp
src/rageplan/ftable.cpp
src/rageplan/mcts.cpp
src/rageplan/node.cpp
src/rageplan/simulator.cpp
src/rageplan/utils.cpp
)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(${PROJECT_NAME}_node src/IR_ROS_NODE.cpp ${RRL_SOURCE})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
