cmake_minimum_required(VERSION 2.8.3)
project(minho_team_ros CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation	
	sensor_msgs
	genmsg	
	cv_bridge
	image_transport
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(
		FILES
		pose.msg
		position.msg
      controlInfo.msg
      robotInfo.msg
      refBoxInfo.msg
      teleop.msg
      hardwareInfo.msg
      range.msg
      label.msg
      visionHSVConfig.msg
      mirrorConfig.msg
      imageConfig.msg
      imuConfig.msg
      omniConfig.msg
      velocity.msg
      interestPoint.msg
      goalKeeperInfo.msg
      interAgentInfo.msg
      aiInfo.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   requestOmniVisionConf.srv
   requestResetEncoders.srv
   requestResetIMU.srv
   requestIMULinTable.srv
   requestOmniProps.srv
   requestReloc.srv
   requestKick.srv
   requestImage.srv
   requestExtendedDebug.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
	generate_messages( DEPENDENCIES std_msgs sensor_msgs )

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
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
		DEPENDS sensor_msgs
	)


############################ DEPENDENCIES ############################
######################################################################

set(PROJECT_DIRECTORY ${minho_team_ros_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
set(username $ENV{USER})
set(home $ENV{HOME})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username})

if(${username} STREQUAL lar)
   if(EXISTS "/opt/Qt" AND IS_DIRECTORY "/opt/Qt")
      set(CURRQTPREFIX /opt/Qt)
      set(CMAKE_PREFIX_PATH ${CURRQTPREFIX})
      find_package(Qt5 COMPONENTS Core REQUIRED)
      set(CMAKE_INCLUDE_CURRENT_DIR ON)
      set(CMAKE_AUTOMOC ON)
      set(QT_DIR ${CURRQTPREFIX}/include)
      include_directories(${QT_DIR} ${QT_DIR}/QtCore)
      set(qt_LIBRARIES libQt5Core.so )
      link_directories(${CURRQTPREFIX}/lib)
   endif()
else()
   #Linking steps for Qt Libraries
   ########################################
   find_package(Qt5 COMPONENTS Core REQUIRED)
   add_definitions (${QT_DEFINITIONS})
   set(CMAKE_INCLUDE_CURRENT_DIR ON)
   set(CMAKE_AUTOMOC ON)
   set(CMAKE_AUTOUIC ON)
   set(CMAKE_AUTORCC ON)
   set(qt_LIBRARIES Qt5::Core)
   include_directories(${Qt5Core_INCLUDE_DIRS})
   ########################################
endif()

# Linking steps for Flycapture Libraries
########################################
include_directories(/usr/include/flycapture)
set(flycap_LIBRARIES /usr/lib/libflycapture.so)
########################################ca

# Linking steps for Freenect Libraries #
########################################
find_package(libfreenect REQUIRED)
include_directories(/usr/include/libusb-1.0)
include_directories(${FREENECT_INCLUDE_DIRS})
set(freenect_LIBRARIES ${FREENECT_LIBRARIES})
########################################

#Linking steps for OpenCV Libraries
########################################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set (opencv_LIBRARIES ${OpenCV_LIBS})
########################################

## Linking steps for CGAL Libraries ##
########################################
find_package(CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})
include_directories (${GMP_INCLUDE_DIRS})
include_directories (${MPFR_INCLUDE_DIRS})
set (cgal_LIBRARIES ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -frounding-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
########################################

######################################################################


##################### COMPILING SHARED LIBRARIES ####################
######################################################################

############# LOCALIZATION #############
########################################
set(LOCALIZATION_DIR ${PROJECT_DIRECTORY}/localization_node)
set(MLIB_DIR ${PROJECT_DIRECTORY}/lib)
include_directories(${LOCALIZATION_DIR}/include)
set(LOCALIZATION_SRC_DIR ${LOCALIZATION_DIR}/src)

##### libRLE #####
##################
add_library(RLE SHARED ${LOCALIZATION_SRC_DIR}/libs/Angle.cpp ${LOCALIZATION_SRC_DIR}/libs/Vec.cpp
${LOCALIZATION_SRC_DIR}/libs/ScanLines.cpp ${LOCALIZATION_SRC_DIR}/libs/RLE.cpp)
set_target_properties( RLE
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(RLE PRIVATE cxx_range_for)
target_link_libraries(RLE ${opencv_LIBRARIES})

### libGigECam ###
##################
add_library(GigECam SHARED ${LOCALIZATION_SRC_DIR}/libs/blackflycam.cpp ${LOCALIZATION_SRC_DIR}/libs/pid.cpp)
set_target_properties( GigECam
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
)
target_compile_features(GigECam PRIVATE cxx_range_for)
target_link_libraries(GigECam ${flycap_LIBRARIES} ${opencv_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(GigECam RLE)
########################################

######################################################################

####################### COMPILING ROBOT NODES ########################
######################################################################

############# LOCALIZATION #############
########################################
# Add new source files to folder /src and add them here for compilation
set(localization_SRC ${LOCALIZATION_SRC_DIR}/main.cpp
${LOCALIZATION_SRC_DIR}/kmeans.cpp ${LOCALIZATION_SRC_DIR}/imageprocessor.cpp
${LOCALIZATION_SRC_DIR}/configserver.cpp ${LOCALIZATION_SRC_DIR}/localization.cpp)
# Add new header files to folder /include and add them here for compilation
set(localization_MOC_H ${LOCALIZATION_DIR}/include/configserver.h ${LOCALIZATION_DIR}/include/localization.h)
qt5_wrap_cpp(localization_CPP ${localization_MOC_H})
add_executable(localization_node ${localization_SRC} ${localization_CPP})
set_target_properties(localization_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(localization_node PRIVATE cxx_range_for)
set(minho_LIBRARIES ${MLIB_DIR}/libGigECam.so ${MLIB_DIR}/libRLE.so)
target_link_libraries(localization_node ${catkin_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES} ${flycap_LIBRARIES} ${minho_LIBRARIES})
add_dependencies(localization_node ${PROJECT_NAME}_gencpp GigECam RLE)
########################################

############### COMS NODE ##############
########################################
set(COMS_DIR ${PROJECT_DIRECTORY}/coms_node)
include_directories(${PROJECT_DIRECTORY}/coms_node/include)
add_executable(coms_node ${COMS_DIR}/src/main.cpp ${COMS_DIR}/src/thpool.c ${COMS_DIR}/src/multicast.cpp)
set_target_properties(coms_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(coms_node PRIVATE cxx_range_for)
target_link_libraries(coms_node ${catkin_LIBRARIES} )
add_dependencies(coms_node ${PROJECT_NAME}_gencpp)
########################################

############# CONTROL NODE #############
########################################
set(CONTROL_DIR ${PROJECT_DIRECTORY}/control_node)
include_directories(${PROJECT_DIRECTORY}/control_node/include)
add_executable(control_node ${CONTROL_DIR}/src/main.cpp ${CONTROL_DIR}/src/behavior.cpp ${CONTROL_DIR}/src/comsmatlab.cpp)
set_target_properties(control_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(control_node PRIVATE cxx_range_for)
target_link_libraries(control_node ${catkin_LIBRARIES} ${qt_LIBRARIES} ${cgal_LIBRARIES})
add_dependencies(control_node ${PROJECT_NAME}_gencpp)
########################################

######################################################################
