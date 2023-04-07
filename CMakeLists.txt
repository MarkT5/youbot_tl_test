cmake_minimum_required(VERSION 2.8)
project(youbot_tl_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs sensor_msgs std_msgs 
cv_bridge image_transport) # genmsg


## Declare a catkin package
catkin_package()

include_directories(
include 
include/urg 
${catkin_INCLUDE_DIRS}
)


set(SOURCE
   Helper.cpp
   main.cpp 
)

set_source_files_properties(${SOURCE} PROPERTIES LANGUAGE CXX )



set(EXT_LIBS
${PROJECT_SOURCE_DIR}/lib/liblua.a #can't use just relative path, coz it is later used in another program (linker)
)

add_executable(${PROJECT_NAME} ${SOURCE})	

	
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


 
