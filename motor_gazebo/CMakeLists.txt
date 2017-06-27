cmake_minimum_required(VERSION 2.8.3)

project(motor_gazebo)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS)

install(DIRECTORY launch
				DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
