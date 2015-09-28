cmake_minimum_required(VERSION 2.8.3)
project(rtslamros)

## Add cmake modules from the package
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tools/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  tf
  vikit_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread system filesystem regex serialization program_options)

## Look for Jafar library
##   A separation into "REQUIRED (...) COMPONENTS (...)" should be able to separate
##   optional components from required ones, but it's not so easy to do that.
find_package(Jafar REQUIRED kernel jmath image correl rtslam COMPONENTS gdhe qdisplay)

## Look for GDHE library (TODO: Change it for optional)
find_package(GDHE REQUIRED)

## Define precompiler flag needed to use GDHE Display
if(GDHE_FOUND AND Jafar_gdhe_FOUND)
  message(STATUS "Found GDHE and Jafar GDHE module. Defining -DHAVE_MODULE_GDHE")
  add_definitions(-DHAVE_MODULE_GDHE)
endif(GDHE_FOUND AND Jafar_gdhe_FOUND)

## Look for Viam library (TODO: Change it for optional)
find_package(Viam REQUIRED)
if(Viam_FOUND)
  add_definitions(-DHAVE_VIAM)
endif(Viam_FOUND)

## Look for OpenCV
find_package(OpenCV REQUIRED)

## Look for Qt4 libs (TODO: Change it for optional)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui )
include(${QT_USE_FILE})
## This definition is needed to make QT work with 3rd party
## signals/slots libraries (like boost::signals, used by tf)
add_definitions(-DQT_NO_KEYWORDS)

## Define precompiler flag needed to use Qt Display
if(QT4_FOUND AND Jafar_qdisplay_FOUND)
  message(STATUS "Found Qt4 and Jafar QDisplay module. Defining -DHAVE_MODULE_QDISPLAY")
  add_definitions(-DHAVE_MODULE_QDISPLAY)
endif(QT4_FOUND AND Jafar_qdisplay_FOUND)

## Disable debug messages from RT-SLAM's headers
add_definitions(-DJFR_NDEBUG)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
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
#  LIBRARIES rtslamros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Jafar_INCLUDE_DIRS}
  ${GDHE_INCLUDE_DIR}
  ${GDHE_INCLUDE_DIR}/gdhe
  ${QT_INCLUDES}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rtslamros
#   src/${PROJECT_NAME}/rtslamros.cpp
# )
add_library(rtslamros_hardware
  src/hardwareSensorMtiRos.cpp
  src/hardwareSensorCameraRos.cpp
)

add_library(rtslamros_core
  src/monoInertialSlam.cpp
  src/configEstimation.cpp
  src/configMonoInertial.cpp

)

## Declare a cpp executable
# add_executable(rtslamros_node src/rtslamros_node.cpp)
# add_executable(jafardemo_node src/jafardemo_node.cpp)
# add_executable(simplertslamros_node src/simplertslamros_node.cpp)
add_executable(monoinertial_node src/monoinertial_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rtslamros_node rtslamros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rtslamros_node
#   rtslamros_hardware
#   ${Jafar_LIBRARIES}
#   ${GDHE_LIBRARY}
#   ${OpenCV_LIBS}
#   ${QT_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# target_link_libraries(jafardemo_node
#   ${Jafar_LIBRARIES}
#   ${GDHE_LIBRARY}
#   ${QT_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# target_link_libraries(simplertslamros_node
#   rtslamros_hardware
#   ${Jafar_LIBRARIES}
#   ${GDHE_LIBRARY}
#   ${QT_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

target_link_libraries(monoinertial_node
  rtslamros_core
  rtslamros_hardware
  ${Jafar_LIBRARIES}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rtslamros rtslamros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtslamros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
