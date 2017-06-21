cmake_minimum_required(VERSION 2.8.3)
project(p3at_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  tf 
  turtlesim
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_message_files(
	FILES
	Num.msg
)

add_service_files(
	FILES
	AddTwoInts.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)



add_executable(samson1 src/samson1.cpp)
target_link_libraries(samson1
	${catkin_LIBRARIES}
)
add_dependencies(samson1 RosAria)

add_executable(samson2 src/samson2.cpp)
target_link_libraries(samson2
	${catkin_LIBRARIES}
)
add_dependencies(samson2 RosAria)


add_executable(atratores src/atratores.cpp)
target_link_libraries(atratores
	${catkin_LIBRARIES}
)
add_dependencies(atratores RosAria)

add_executable(robot_atrator src/robot_atrator.cpp)
target_link_libraries(robot_atrator
	${catkin_LIBRARIES}
)
add_dependencies(robot_atrator RosAria)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster
	${catkin_LIBRARIES}
)


add_executable(particle_filter src/particle_filter.cpp)
target_link_libraries( particle_filter
	${catkin_LIBRARIES}
)

add_dependencies(particle_filter RosAria)

