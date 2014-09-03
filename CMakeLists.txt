cmake_minimum_required(VERSION 2.8.3)
project(karto_plugins)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pluginlib
  roscpp
  open_karto
  slam_karto
  vertigo_slam
  interactive_markers
)

find_package(Eigen REQUIRED)
find_package(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES karto_plugins
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN_INCLUDE_DIRS} 
  ${CSPARSE_INCLUDE_DIR})

add_library(karto_plugins
   src/g2o_solver.cpp
   src/vertigo_maxmix_solver.cpp
   src/vertigo_switchable_solver.cpp
)

add_library(g2o_calibration ${G2O_LIB_TYPE}
  src/calibration/gm2dl_io.cpp 
  src/calibration/edge_se2_pure_calib.cpp 
  src/calibration/closed_form_calibration.cpp 
  src/calibration/sclam_helpers.cpp 
  src/calibration/types_sclam2d.cpp 
  src/calibration/robot_laser_sclam.cpp)

add_executable(sclam src/calibration/sclam_pure_calibration.cpp)

target_link_libraries(g2o_calibration ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBRARIES})
target_link_libraries(sclam g2o_calibration)

## Specify libraries to link a library or executable target against
target_link_libraries(karto_plugins
  ${catkin_LIBRARIES} ${G2O_LIBRARIES} g2o_calibration
 )

## Mark executables and/or libraries for installation
install(TARGETS karto_plugins 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sclam
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
