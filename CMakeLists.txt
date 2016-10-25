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
      imgRequest.msg
      range.msg
      label.msg
      visionHSVConfig.msg
      mirrorConfig.msg
      imageConfig.msg
      imuConfig.msg
      omniConfig.msg
      velocity.msg
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


## DEPENDENCIES ##
######################################################################
set(PROJECT_DIRECTORY ~/catkin_ws/src/minho_team_ros)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(username $ENV{USER})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username})
#Linking steps for Qt5
########################################
if(${username} STREQUAL lar)
set(CURRQTPREFIX /opt/Qt)
else()
set(CURRQTPREFIX /opt/Qt/5.7/gcc_64)
endif()

MESSAGE( STATUS "-----> Current Qt Installation Path : " ${CURRQTPREFIX})
set(CMAKE_PREFIX_PATH ${CURRQTPREFIX})
set(Qt5SerialPort_DIR ${CURRQTPREFIX}/lib/cmake/Qt5SerialPort)
find_package(Qt5 COMPONENTS Core SerialPort Network REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(QT_DIR ${CURRQTPREFIX}/include)
include_directories(${QT_DIR} ${QT_DIR}/QtCore ${QT_DIR}/QtConcurrent ${QT_DIR}/QtNetwork ${QT_DIR}/QtSerialPort ${QT_DIR}/QtWebSockets ${QT_DIR}/QtXml)
set(qt_LIBRARIES libQt5SerialPort.so libQt5Core.so libQt5Network.so)
link_directories(${CURRQTPREFIX}/lib)
add_definitions(-DQT_THREAD_SUPPORT)
########################################

#Linking steps for Flycapture Libraries
########################################
include_directories(/usr/include/flycapture)
set(flycap_LIBRARIES libflycapture.so)
link_directories(/usr/lib)
########################################

#Linking steps for libfreenect Libraries
########################################
include_directories(/usr/local/include/libfreenect)
include_directories(/usr/include/libusb-1.0)
set(freenect_LIBRARIES libfreenect.so libfreenect_sync.so)
link_directories(/usr/local/lib)
########################################

#Linking steps for OpenCV Libraries
########################################
include_directories(/usr/local/include/opencv /usr/local/include/opencv2)
set (opencv_LIBRARIES libopencv_core.so libopencv_highgui.so libopencv_imgproc.so)
link_directories(/usr/local/lib)
link_directories(/usr/lib/x86_64-linux-gnu)
########################################
######################################################################

## HARDWARE ##
######################################################################
set(HARDWARE_DIR ${PROJECT_DIRECTORY}/hardware_node)
include_directories(${HARDWARE_DIR}/include)
set(HARDWARE_SRC_DIR ${HARDWARE_DIR}/src)
# Add new source files to folder /src and add them here for compilation
set(hardware_SRC ${HARDWARE_SRC_DIR}/main_hardware.cpp ${HARDWARE_SRC_DIR}/hardware.cpp)
# Add new header files to folder /include and add them here for compilation
set(hardware_MOC_H ${HARDWARE_DIR}/include/hardware.h)
qt5_wrap_cpp(hardware_CPP ${hardware_MOC_H})
add_executable(hardware_node ${hardware_SRC} ${hardware_CPP})
set_target_properties(hardware_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(hardware_node PRIVATE cxx_range_for)
target_link_libraries(hardware_node ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(hardware_node ${PROJECT_NAME}_gencpp)
######################################################################

## LOCALIZATION ##
######################################################################
set(LOCALIZATION_DIR ${PROJECT_DIRECTORY}/localization_node)
include_directories(${LOCALIZATION_DIR}/include)
set(LOCALIZATION_SRC_DIR ${LOCALIZATION_DIR}/src)
# Add new source files to folder /src and add them here for compilation
set(localization_SRC ${LOCALIZATION_SRC_DIR}/main_localization.cpp ${LOCALIZATION_SRC_DIR}/Angle.cpp 
${LOCALIZATION_SRC_DIR}/Vec.cpp ${LOCALIZATION_SRC_DIR}/RLE.cpp ${LOCALIZATION_SRC_DIR}/ScanLines.cpp 
${LOCALIZATION_SRC_DIR}/kmeans.cpp ${LOCALIZATION_SRC_DIR}/blackflydriver.cpp 
${LOCALIZATION_SRC_DIR}/blackflycamera.cpp ${LOCALIZATION_SRC_DIR}/imageprocessor.cpp
${LOCALIZATION_SRC_DIR}/configserver.cpp ${LOCALIZATION_SRC_DIR}/localization.cpp)
# Add new header files to folder /include and add them here for compilation
set(localization_MOC_H ${LOCALIZATION_DIR}/include/blackflycamera.h ${LOCALIZATION_DIR}/include/blackflydriver.h ${LOCALIZATION_DIR}/include/configserver.h ${LOCALIZATION_DIR}/include/localization.h)
qt5_wrap_cpp(localization_CPP ${localization_MOC_H})
add_executable(localization_node ${localization_SRC} ${localization_CPP})
set_target_properties(localization_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(localization_node PRIVATE cxx_range_for)
target_link_libraries(localization_node ${catkin_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES} ${flycap_LIBRARIES})
add_dependencies(localization_node ${PROJECT_NAME}_gencpp)
######################################################################

## HARDWARE TEST NODE ##
######################################################################
set(HW_TEST_DIR ${PROJECT_DIRECTORY}/hw_test_node)
add_executable(hw_test_node ${HW_TEST_DIR}/main.cpp)
set_target_properties(hw_test_node PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(hw_test_node PRIVATE cxx_range_for)
target_link_libraries(hw_test_node ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(hw_test_node ${PROJECT_NAME}_gencpp)
######################################################################
