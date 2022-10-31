#include <Eigen/Core>
#include <iostream>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "voxblox/core/common.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

namespace py = pybind11;
using namespace voxblox;

class PyEsdfMap {
 public:
  std::shared_ptr<EsdfMap> esdf_map_;
  float voxel_size_;
  int voxels_per_side_;

 public:
  PyEsdfMap(float voxel_size, int voxels_per_side) : PyEsdfMap(Layer<EsdfVoxel>(voxel_size, voxels_per_side)) {}

  PyEsdfMap(const Layer<EsdfVoxel>& esdf_layer) {
    this->esdf_map_.reset(new EsdfMap(esdf_layer));
    this->voxel_size_ = esdf_layer.voxel_size();
    this->voxels_per_side_ = esdf_layer.voxels_per_side();
  }

  void update(std::vector<double> camera_pose, std::vector<std::vector<double>> point_cloud)
  {
    const auto p = Point(camera_pose[0], camera_pose[1], camera_pose[2]);
    const auto q = Quaternion(
        camera_pose[3], camera_pose[4], camera_pose[5], camera_pose[6]);
    const auto transform = Transformation(q, p);

    const size_t n_point = point_cloud.size();

    auto point_cloud_eigen = AlignedVector<Point>(n_point);
    for(size_t i = 0; i < n_point; ++i) {
      const auto & pt = point_cloud[i];
      Point point_eigen(pt[0], pt[1], pt[2]);
      point_cloud_eigen[i] = point_eigen;
    }
  }

  std::vector<double> get_sd_batch(const std::vector<std::vector<double>> pts,
                                   double fill_value)
  {
    std::vector<double> values(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto& pt = pts[i];
      Eigen::Vector3d pt_eigen;
      pt_eigen << pt[0], pt[1], pt[2];
      const bool success =
          esdf_map_->getDistanceAtPosition(pt_eigen, &values[i]);
      if (!success) {
        values[i] = fill_value;
      }
    }
    return values;
  }
};

PyEsdfMap get_test_esdf() {
  float voxel_size = 0.01;
  int voxels_per_side = 30;
  float esdf_max_distance = 4.0f;
  SimulationWorld world;

  // Create a test environment.
  // It consists of a 10x10x7 m environment with an object in the middle.
  Point min_bound(-5.0, -5.0, -1.0);
  Point max_bound(5.0, 5.0, 6.0);
  world.setBounds(min_bound, max_bound);
  Point cylinder_center(0.0, 0.0, 2.0);
  FloatingPoint cylinder_radius = 2;
  FloatingPoint cylinder_height = 4;
  world.addObject(std::unique_ptr<Object>(new Cylinder(
      cylinder_center, cylinder_radius, cylinder_height, Color::Red())));
  world.addGroundLevel(0.0);

  // Finally, get the GT.
  std::unique_ptr<Layer<EsdfVoxel>> esdf_gt;
  esdf_gt.reset(new Layer<EsdfVoxel>(voxel_size, voxels_per_side));
  world.generateSdfFromWorld(esdf_max_distance, esdf_gt.get());

  // convert to esdfmap
  const auto esdf_map = PyEsdfMap(*esdf_gt);
  esdf_map.esdf_map_->block_size();
  return esdf_map;
}

void setup_tsdf_layer() {
  float voxel_size = 0.01;
  float truncation_distance = voxel_size * 0.4;
  int voxels_per_side = 30;
  float esdf_max_distance = 4.0f;

  // setup tsdf
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = truncation_distance;
  config.integrator_threads = 1;

  Layer<TsdfVoxel> tsdf_layer(voxel_size, voxels_per_side);
  FastTsdfIntegrator tsdf_integrator(config, &tsdf_layer);

  // setup esdf
  Layer<EsdfVoxel> batch_layer(voxel_size, voxels_per_side);

  EsdfIntegrator::Config esdf_config;
  esdf_config.max_distance_m = esdf_max_distance;
  esdf_config.default_distance_m = esdf_max_distance;
  esdf_config.min_distance_m = truncation_distance / 2.0;
  esdf_config.min_diff_m = 0.0;
  esdf_config.full_euclidean_distance = false;
  esdf_config.add_occupied_crust = false;
  esdf_config.multi_queue = true;
  EsdfIntegrator batch_integrator(esdf_config, &tsdf_layer, &batch_layer);
}

PYBIND11_MODULE(_voxbloxpy, m) {
  m.doc() = "voxblox python wrapper";
  m.def("setup_tsdf_layer", &setup_tsdf_layer);
  m.def("get_test_esdf", &get_test_esdf);

  py::class_<PyEsdfMap>(m, "EsdfMap")
      .def(py::init<float, int>())
      .def("get_sd_batch", &PyEsdfMap::get_sd_batch)
      .def("update", &PyEsdfMap::update)
      .def_readonly("voxel_size", &PyEsdfMap::voxel_size_)
      .def_readonly("voxels_per_side", &PyEsdfMap::voxels_per_side_);

}
