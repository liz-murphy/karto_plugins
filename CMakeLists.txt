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

## Specify libraries to link a library or executable target against
target_link_libraries(karto_plugins
  ${catkin_LIBRARIES} ${G2O_LIBRARIES} 
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
