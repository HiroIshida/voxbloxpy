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
include_directories(./minkindr/minkindr/include/)

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

option(BUILD_TEST "build google test" ON)
if(BUILD_TEST)
    find_package(GTest REQUIRED)
    enable_testing()

    function(setup_voxblox_test test_name test_src)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} voxblox ${GTEST_LIBRARIES} pthread)
        target_include_directories(${test_name} PUBLIC ${GTEST_INCLUDE_DIRS})
        gtest_discover_tests(${test_name})
    endfunction()

    setup_voxblox_test(test_approx_hash_array test/test_approx_hash_array.cc)
    setup_voxblox_test(test_bucket_queue test/test_bucket_queue.cc)
    #setup_voxblox_test(test_clear_spheres test/test_clear_spheres.cc)
    setup_voxblox_test(test_layer test/test_layer.cc)
    #setup_voxblox_test(test_layer_utils test/test_layer_utils.cc)
    #setup_voxblox_test(test_load_esdf test/test_load_esdf.cc)
    #setup_voxblox_test(test_merge_integration test/test_merge_integration.cc)
    #setup_voxblox_test(test_protobuf test/test_protobuf.cc)
    #setup_voxblox_test(test_sdf_integration test/test_sdf_integrators.cc)
    #setup_voxblox_test(test_tsdf_interpolator test/test_tsdf_interpolator.cc)
    setup_voxblox_test(test_tsdf_map test/test_tsdf_map.cc)
endif(BUILD_TEST)
