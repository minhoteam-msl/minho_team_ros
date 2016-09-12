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
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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


set(PROJECT_DIRECTORY ~/catkin_ws/src/minho_team_ros)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_PREFIX_PATH /opt/Qt/5.7/gcc_64)
find_package(Qt5 COMPONENTS Core SerialPort Network REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(QT_DIR /opt/Qt/5.7/gcc_64/include)
include_directories(${QT_DIR} ${QT_DIR}/QtCore ${QT_DIR}/QtConcurrent ${QT_DIR}/QtDesigner ${QT_DIR}/QtDesignerComponents
${QT_DIR}/QtGui ${QT_DIR}/QtNetwork ${QT_DIR}/QtSerialPort ${QT_DIR}/QtWebSockets ${QT_DIR}/QtWidgets ${QT_DIR}/QtXml)
set(qt_LIBRARIES libQt5SerialPort.so libQt5Core.so libQt5Widgets.so libQt5Network.so libQt5Gui.so libQt5Qml.so libQt5Quick.so)
link_directories(/opt/Qt/5.7/gcc_64/lib)
add_definitions(-DQT_THREAD_SUPPORT)
set(CMAKE_AUTOUIC ON)

set(TELEOP_DIR ${PROJECT_DIRECTORY}/tools/minho_teleop)
set(teleop_MOC_H ${TELEOP_DIR}/include/mainwindow.h)
qt5_wrap_cpp(teleop_CPP ${teleop_MOC_H})
set(teleop_SRC ${TELEOP_DIR}/src/main.cpp ${TELEOP_DIR}/src/mainwindow.cpp ${TELEOP_DIR}/src/qrc_resources.cpp)
include_directories(${TELEOP_DIR}/include)
add_executable(minho_teleop ${teleop_SRC} ${teleop_CPP}) 
set_target_properties(minho_teleop PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(minho_teleop PRIVATE cxx_range_for)
target_link_libraries(minho_teleop ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(minho_teleop minho_team_ros_generate_messages_cpp)
