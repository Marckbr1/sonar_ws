cmake_minimum_required(VERSION 3.0.2)
project(quad_amcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  roscpp
  rospy
  dynamic_reconfigure
  rosbag
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
#find_package(OpenCV REQUIRED )
find_package( OpenCV 4.5.1 REQUIRED )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (PCL 1.2 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

find_package( OpenMP )
#if(OPENMP_FOUND)
#  message(STATUS "OPENMP FOUND")
#  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
#  set(OpenMP_LIBS gomp)
#endif()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Set debug mode
set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  StampedInteger.msg
  StampedArrayFloat.msg
#   Message1.msg
#   Message2.msg
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
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/ImageLoader.cfg
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quad_amcl
  CATKIN_DEPENDS
  image_transport
  roscpp
  rospy
  message_runtime
#  DEPENDS
#  OpenMP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src
  src/ImageEnhancement
  src/ImageEnhancement/PointCloudMatch
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/quad_amcl.cpp
# )

add_library(csv_reader
  src/libs/CSVReader/CSVData.cpp
  src/libs/CSVReader/CSVReader.cpp
  src/libs/CSVReader/CSVReader2.cpp
  src/libs/CSVReader/DataExtractorUtilities.cpp
)

target_include_directories (csv_reader PUBLIC src/libs/CSVReader/)

# === Lib Dataset Reader ====
add_library(dataset_reader
  src/libs/DatasetReader/DatasetReader.cpp)

add_dependencies(dataset_reader csv_reader)
target_link_libraries(dataset_reader csv_reader)

target_include_directories (dataset_reader PUBLIC src/libs/DatasetReader/)

# ===== Lib Config Loader =======
add_library(config_loader
  src/libs/ConfigLoader/Automaton.cpp
  src/libs/ConfigLoader/AutomatonEdge.cpp
  src/libs/ConfigLoader/ConfigLoader.cpp
)

target_include_directories (config_loader PUBLIC src/libs/ConfigLoader/)

# ===== Lib Satellite Manager =======
add_library(satellite_manager
  src/libs/SatelliteManager/SatelliteManager.cpp
)

add_dependencies(satellite_manager config_loader)
target_link_libraries(satellite_manager config_loader)

target_include_directories (satellite_manager PUBLIC src/libs/SatelliteManager/)

# ====== FGR ======
set(FastGlobalRegistration_SOURCE_DIR
   "/home/marck/source/")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_library(FastGlobalRegistrationLib STATIC
  ${FastGlobalRegistration_SOURCE_DIR}/FastGlobalRegistration/app.cpp
  ${FastGlobalRegistration_SOURCE_DIR}/FastGlobalRegistration/app.h)

# set include directories
include_directories(
  ${FastGlobalRegistration_SOURCE_DIR}
#  ${FastGlobalRegistration_SOURCE_DIR}/External
#  ${FastGlobalRegistration_SOURCE_DIR}/External/Eigen
#  ${FastGlobalRegistration_SOURCE_DIR}/External/flann
  ${FastGlobalRegistration_SOURCE_DIR}/FastGlobalRegistration
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/quad_amcl_node.cpp)

add_executable(image_loader
  src/main_image_loader.cpp
  src/ImageLoader.cpp)

add_executable(data_2_bag
 src/Data2Bag/Data2Bag.cpp)

add_executable(particle_filter
  src/ParticleFilter/mainParticleFilter.cpp
  src/ParticleFilter/ParticleFilter.cpp
  src/ParticleFilter/SimplePF.cpp
  src/ParticleFilter/Odometry.cpp
  src/ParticleFilter/ParticleFilterTester.cpp
  src/ParticleFilter/ParticleFilterNode.cpp)

add_executable(son_enhancement
  src/ImageEnhancement/main_son_enhancement.cpp
  src/ImageEnhancement/SonEnhancementNode.cpp
  src/ImageEnhancement/SonImgQueue.cpp
  src/ImageEnhancement/SonImg.cpp
  src/ImageEnhancement/SonMatch.cpp
  src/ImageEnhancement/SonMap.cpp
  src/ImageEnhancement/PointCloudMatch/PointCloudMatch.cpp
  src/ImageEnhancement/PointCloudMatch/ICPMatch.cpp
  src/ImageEnhancement/PointCloudMatch/FGRMatch.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(image_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(data_2_bag
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(particle_filter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_dependencies(son_enhancement
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(image_loader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(data_2_bag
  dataset_reader
  satellite_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(particle_filter
  satellite_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_compile_options(son_enhancement
  PRIVATE ${OpenMP_FLAGS})

target_link_libraries(son_enhancement
  satellite_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_LIBRARIES}
  ${PCL_LIBRARIES}
  FastGlobalRegistrationLib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_quad_amcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
