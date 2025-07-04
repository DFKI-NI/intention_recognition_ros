cmake_minimum_required(VERSION 3.5.1)
project(intention_recognition_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  actionlib
  actionlib_msgs
  tables_demo_planning
)

add_message_files(
  DIRECTORY msg
  FILES
  IntentionObs.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  Intention.srv
)

add_action_files(
  DIRECTORY action
  FILES
  PlanAndExecute.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime actionlib actionlib_msgs tables_demo_planning
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

add_dependencies(${PROJECT_NAME}_node intention_recognition_ros_generate_messages_cpp)

catkin_install_python(PROGRAMS
  nodes/activity_recognition_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
