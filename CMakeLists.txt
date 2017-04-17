cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

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
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker
	${catkin_LIBRARIES}
)

add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener
	${catkin_LIBRARIES}
)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

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


