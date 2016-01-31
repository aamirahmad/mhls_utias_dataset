cmake_minimum_required(VERSION 2.8.3)
project(mhls_utias_dataset)


find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  message_generation
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


link_libraries(csparse cxsparse g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)


FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


SET (G2O_LIBS g2o_simulator
g2o_types_icp
g2o_types_sim3
g2o_incremental
g2o_types_slam3d_addons
g2o_types_sba
g2o_interactive
g2o_types_slam2d_addons
g2o_solver_slam2d_linear
g2o_types_sclam2d
g2o_types_data
g2o_types_slam3d
g2o_types_slam2d
# g2o_viewer
g2o_solver_cholmod
g2o_solver_pcg
g2o_solver_dense
g2o_solver_csparse
#g2o_solver_eigen
g2o_solver_structure_only
g2o_cli
g2o_core
g2o_stuff
g2o_interface
g2o_parser
g2o_ext_freeglut_minimal
g2o_opengl_helper
g2o_csparse_extension)

add_library(G2O_LIBS STATIC IMPORTED)


add_executable(mhls_utias_dataset src/mhls_utias_dataset.cpp)
add_dependencies(mhls_utias_dataset mhls_utias_dataset_generate_messages_cpp)
target_link_libraries(mhls_utias_dataset ${G2O_LIBS} ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

