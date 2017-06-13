cmake_minimum_required(VERSION 2.8.3)
project(gk_node C CXX)

set(username $ENV{USER})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

if(${username} STREQUAL pi)
#To compile in raspberry pi, including only the messages and services
   find_package(catkin REQUIRED COMPONENTS
	   roscpp
	   std_msgs
	   message_generation	
	   genmsg
   )
else()
   find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation	
	sensor_msgs
	genmsg	
	cv_bridge
	image_transport
   )
endif()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need



############################ DEPENDENCIES ############################
######################################################################

set(PROJECT_DIRECTORY ${gk_node_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
set(home $ENV{HOME})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username} "   " ${PROJECT_DIRECTORY} )


      #Linking steps for OpenCV Libraries
      ########################################
      find_package(OpenCV REQUIRED)
      include_directories(${OpenCV_INCLUDE_DIRS})
      set (opencv_LIBRARIES ${OpenCV_LIBS})
      ########################################

   # Linking steps for Flycapture Libraries
   ########################################
   include_directories(/usr/include/flycapture)
   set(flycap_LIBRARIES /usr/lib/libflycapture.so)
   ########################################

   # Linking steps for Freenect Libraries #
   ########################################
   find_package(libfreenect REQUIRED)
   include_directories(/usr/include/libusb-1.0)
   include_directories(${FREENECT_INCLUDE_DIRS})
   set(freenect_LIBRARIES ${FREENECT_LIBRARIES} libfreenect_sync.so)
   ########################################


    
	include_directories(${PROJECT_DIRECTORY}/goalkeeper_node/include)
	set(GK_DIR ${PROJECT_DIRECTORY}/goalkeeper_node)
	add_executable(goalkeeper_node ${GK_DIR}/src/main.cpp ${GK_DIR}/src/kinectvision.cpp ${GK_DIR}/src/lidarlocalization.cpp )
	set_target_properties(goalkeeper_node PROPERTIES COMPILE_FLAGS "-fPIC")
	target_compile_features(goalkeeper_node PRIVATE cxx_range_for)
	target_link_libraries(goalkeeper_node ${opencv_LIBRARIES} ${catkin_LIBRARIES} ${freenect_LIBRARIES} )
	########################################

