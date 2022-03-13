cmake_minimum_required(VERSION 3.0.2)

project(project_1)
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs
	tf
	geometry_msgs 
	nav_msgs
	message_generation
	message_filters
	dynamic_reconfigure
	)

generate_dynamic_reconfigure_options(
		cfg/integration.cfg
)

add_service_files(
		FILES
		reset.srv
		reset_general.srv
)

add_message_files(
	FILES 
	CustomOdometry.msg
	Speed.msg
	)

generate_messages(
	DEPENDENCIES 
	nav_msgs
	geometry_msgs
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
	message_runtime
	message_filters
	geometry_msgs
	)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(AgileXCore src/AgileXCore.cpp)
target_link_libraries(AgileXCore ${catkin_LIBRARIES})

add_executable(LateralWheelSync src/LateralWheelSync.cpp)
target_link_libraries(LateralWheelSync ${catkin_LIBRARIES})

add_executable(GlobalSync src/GlobalSync.cpp)
target_link_libraries(GlobalSync ${catkin_LIBRARIES})

add_executable(BaselineCalculator src/BaselineCalculator.cpp)
target_link_libraries(BaselineCalculator ${catkin_LIBRARIES})

add_executable(GearRatioCalculator src/GearRatioCalculator.cpp)
target_link_libraries(GearRatioCalculator ${catkin_LIBRARIES})


# Add this only for nodes that requires the specific custom message generated in the package
add_dependencies(LateralWheelSync project_1_generate_messages_cpp)
add_dependencies(GlobalSync project_1_generate_messages_cpp)
add_dependencies(BaselineCalculator project_1_generate_messages_cpp)
add_dependencies(AgileXCore project_1_generate_messages_cpp)
add_dependencies(GearRatioCalculator project_1_generate_messages_cpp)

#For dynamic reconfigure
add_dependencies(AgileXCore ${PROJECT_NAME}_gencfg)

#for services
add_dependencies(AgileXCore ${catkin_EXPORTED_TARGETS})


