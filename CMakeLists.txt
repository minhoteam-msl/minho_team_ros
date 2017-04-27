cmake_minimum_required(VERSION 2.8.3)
project(minho_team_ros C CXX)

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
      controlConfig.msg
      visualizerConfig.msg
      pathData.msg
      segment.msg
      PID.msg
      cameraProperty.msg
      ControlerError.msg
      ROI.msg
      baseStationInfo.msg
      obstacle.msg
      worldConfig.msg
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
   requestControlConfig.srv
   requestCamPID.srv
   requestCamProperty.srv
   requestROI.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
if(${username} STREQUAL pi)
	generate_messages( DEPENDENCIES std_msgs )
else()
	generate_messages( DEPENDENCIES std_msgs sensor_msgs )
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
if(${username} STREQUAL pi)
   catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs
	)
else()
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
		DEPENDS sensor_msgs
	)
endif()


############################ DEPENDENCIES ############################
######################################################################

set(PROJECT_DIRECTORY ${minho_team_ros_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
set(home $ENV{HOME})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username})

if(${username} STREQUAL pi)
MESSAGE( STATUS "-----> Compiling only messages and services for RaspiHw")
else()
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

   ### Linking steps for OpenCV Libraries ##
   ########################################
   include_directories(/usr/local/include/opencv /usr/local/include/opencv2)
   set (opencv_LIBRARIES /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so)
   ########################################



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

      #Linking steps for OpenCV Libraries
      ########################################
      find_package(OpenCV REQUIRED)
      include_directories(${OpenCV_INCLUDE_DIRS})
      set (opencv_LIBRARIES ${OpenCV_LIBS})
      ########################################

   endif()

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

   include_directories($ENV{HOME}/Common)
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

   ##### libBlob #####
   ##################
   add_library(Blob SHARED ${LOCALIZATION_SRC_DIR}/libs/Blob.cpp)
   set_target_properties( Blob
       PROPERTIES
       LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
   )
   target_compile_features(Blob PRIVATE cxx_range_for)
   target_link_libraries(Blob RLE ${opencv_LIBRARIES})
   add_dependencies(Blob RLE)

   ### libGigECam ###
   ##################
   add_library(GigECam SHARED ${LOCALIZATION_SRC_DIR}/libs/blackflycam.cpp ${LOCALIZATION_SRC_DIR}/libs/pid.cpp)
   set_target_properties( GigECam
       PROPERTIES
       LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
   )
   target_compile_features(GigECam PRIVATE cxx_range_for)
   target_link_libraries(GigECam ${flycap_LIBRARIES} ${opencv_LIBRARIES} ${catkin_LIBRARIES})
   add_dependencies(GigECam RLE Blob)
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
   set(minho_LIBRARIES ${MLIB_DIR}/libGigECam.so ${MLIB_DIR}/libRLE.so ${MLIB_DIR}/libBlob.so)
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
   include_directories(/usr/local/include)
   link_directories(/usr/local/lib)
   set(CONTROL_DIR ${PROJECT_DIRECTORY}/control_node)
   include_directories(${PROJECT_DIRECTORY}/control_node/include)
   add_executable(control_node ${CONTROL_DIR}/src/main.cpp ${CONTROL_DIR}/src/behavior.cpp 
   ${CONTROL_DIR}/src/fundamental.cpp ${CONTROL_DIR}/src/motion.cpp 
   ${CONTROL_DIR}/src/voronoi.cpp ${CONTROL_DIR}/src/dijkstrashortestpath.cpp)
   set_target_properties(control_node PROPERTIES COMPILE_FLAGS "-fPIC")
   target_compile_features(control_node PRIVATE cxx_range_for)
   target_link_libraries(control_node ${catkin_LIBRARIES} ${qt_LIBRARIES} ${cgal_LIBRARIES} libsisl.a)
   add_dependencies(control_node ${PROJECT_NAME}_gencpp)
   ########################################


	############# AI NODE #############
	###################################
	include_directories(/usr/local/include)
	link_directories(/usr/local/lib)
	set(AI_DIR ${PROJECT_DIRECTORY}/ai_node)
	include_directories(${PROJECT_DIRECTORY}/ai_node/include)
	add_executable(ai_node ${AI_DIR}/src/main.cpp ${AI_DIR}/src/ai.cpp ${AI_DIR}/src/role.cpp ${AI_DIR}/src/rstop.cpp ${AI_DIR}/src/rgkeeper.cpp
	${AI_DIR}/src/rstriker.cpp ${AI_DIR}/src/rsupstriker.cpp)
	set_target_properties(ai_node PROPERTIES COMPILE_FLAGS "-fPIC")
	target_compile_features(ai_node PRIVATE cxx_range_for)
	target_link_libraries(ai_node ${catkin_LIBRARIES} ${qt_LIBRARIES})
	add_dependencies(ai_node ${PROJECT_NAME}_gencpp)
	########################################
	
	############# GOALKEEPER NODE #############
	###########################################
	set(GK_DIR ${PROJECT_DIRECTORY}/goalkeeper_node)
	
	### libGigECam ###
   ##################
   add_library(kinectVision SHARED ${GK_DIR}/src/kinectvision.cpp)
   set_target_properties( kinectVision
       PROPERTIES
       LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
   )
   target_compile_features(kinectVision PRIVATE cxx_range_for)
   target_link_libraries(kinectVision ${flycap_LIBRARIES} ${opencv_LIBRARIES} ${catkin_LIBRARIES} ${freenect_LIBRARIES})
   ########################################
   
   ### libGigECam ###
   ##################
   add_library(lidarLoc SHARED ${GK_DIR}/src/lidarlocalization.cpp)
   set_target_properties( lidarLoc
       PROPERTIES
       LIBRARY_OUTPUT_DIRECTORY ${MLIB_DIR}
   )
   target_compile_features(lidarLoc PRIVATE cxx_range_for)
   target_link_libraries(lidarLoc ${flycap_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES} ${catkin_LIBRARIES} ${freenect_LIBRARIES})
   ########################################
   
	include_directories(${PROJECT_DIRECTORY}/goalkeeper_node/include)
	add_executable(goalkeeper_node ${GK_DIR}/src/main.cpp)
	set_target_properties(goalkeeper_node PROPERTIES COMPILE_FLAGS "-fPIC")
	target_compile_features(goalkeeper_node PRIVATE cxx_range_for)
	target_link_libraries(goalkeeper_node ${catkin_LIBRARIES} ${MLIB_DIR}/libkinectVision.so ${MLIB_DIR}/liblidarLoc.so)
	add_dependencies(goalkeeper_node ${PROJECT_NAME}_gencpp lidarLoc kinectVision)
	########################################


######################################################################
endif()
