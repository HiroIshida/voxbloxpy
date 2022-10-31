#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"

namespace py = pybind11;
using namespace voxblox;


void setup_tsdf_layer(){
  float voxel_size = 0.01;
  float truncation_distance = voxel_size * 0.4;
  int voxels_per_side = 30;

  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = truncation_distance;
  config.integrator_threads = 1;

  Layer<TsdfVoxel> fast_layer(voxel_size, voxels_per_side);
  FastTsdfIntegrator fast_integrator(config, &fast_layer);
}

PYBIND11_MODULE(_voxbloxpy, m) {
    m.doc() = "voxblox python wrapper";
    m.def("setup_tsdf_layer", &setup_tsdf_layer);
}
