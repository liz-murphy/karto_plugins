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
  srba
)

find_package(Eigen REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(MRPT REQUIRED gui graphs graphslam)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES karto_plugins
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN_INCLUDE_DIRS} 
  ${CSPARSE_INCLUDE_DIR}
  ${MRPT_INCLUDE_DIRS})

add_library(karto_plugins
   src/g2o_solver.cpp
   src/vertigo_maxmix_solver.cpp
   src/vertigo_switchable_solver.cpp
   src/srba_solver.cpp
)

target_link_libraries(karto_plugins
    ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${MRPT_LIBS} 
 )


add_executable(test_srba test/test_srba.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(test_srba
    ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${MRPT_LIBS} karto_plugins 
 )

## Mark executables and/or libraries for installation
install(TARGETS karto_plugins 
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES karto_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
