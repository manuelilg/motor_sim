cmake_minimum_required(VERSION 2.8.3)

project(motor_description)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS)

install(DIRECTORY launch urdf rviz
				DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
