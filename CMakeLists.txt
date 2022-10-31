cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(PROJECT_NAME "voxblox")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig)
pkg_check_modules(LIBGLOG REQUIRED libglog)
pkg_check_modules(LIBGFLAGS REQUIRED gflags)

include_directories(include/)

add_library(voxblox STATIC
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)
target_link_libraries(voxblox PRIVATE glog)
target_link_libraries(voxblox PRIVATE gflags)
target_include_directories(voxblox PRIVATE ./minkindr/minkindr/include/)
