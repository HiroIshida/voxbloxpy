cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(PROJECT_NAME "voxbloxpy")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(voxblox/include/)
include_directories(voxblox/minkindr/minkindr/include/)

add_subdirectory(voxblox)
add_subdirectory(pybind11)
pybind11_add_module(_voxbloxpy wrapper.cpp)
target_link_libraries(_voxbloxpy PRIVATE voxblox)

set(CMD_SITE_PKG "import site; import sys;sys.stdout.write(site.getusersitepackages())")

#execute_process(
#    set(PYTHON_EXECUTABLE "/bin/python3")
#    COMMAND ${PYTHON_EXECUTABLE} -c "${CMD_SITE_PKG}"
#    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_INSTALL_DIR
#    )
