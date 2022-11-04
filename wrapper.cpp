#include <Eigen/Core>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "voxblox/core/common.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/core/tsdf_map.h"
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


class PyTsdfMap {

public:
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;
  float voxel_size_;
  int voxels_per_side_;

public:
  PyTsdfMap(float voxel_size, int voxels_per_side, TsdfIntegratorType integrator_type)
    : PyTsdfMap(Layer<TsdfVoxel>(voxel_size, voxels_per_side), integrator_type) {}

  PyTsdfMap(const Layer<TsdfVoxel>& tsdf_layer, TsdfIntegratorType integrator_type) {
    tsdf_map_.reset(new TsdfMap(tsdf_layer));
    voxel_size_ = tsdf_layer.voxel_size();
    voxels_per_side_ = tsdf_layer.voxels_per_side();

    TsdfIntegratorBase::Config config;
    config.default_truncation_distance = 4 * tsdf_layer.voxel_size();
    config.integrator_threads = 1;

    if(integrator_type == TsdfIntegratorType::kSimple){
      tsdf_integrator_.reset(new SimpleTsdfIntegrator(config, tsdf_map_->getTsdfLayerPtr()));
    }else if(integrator_type == TsdfIntegratorType::kMerged){
      tsdf_integrator_.reset(new MergedTsdfIntegrator(config, tsdf_map_->getTsdfLayerPtr()));
    }else if(integrator_type == TsdfIntegratorType::kFast){
      tsdf_integrator_.reset(new FastTsdfIntegrator(config, tsdf_map_->getTsdfLayerPtr()));
    }else{
      throw std::invalid_argument("invalid initegrator type");
    }
  }

  void update(std::vector<double> camera_pose, std::vector<std::vector<double>> point_cloud_std)
  {
    const auto p = Point(camera_pose[0], camera_pose[1], camera_pose[2]);
    const auto q = Quaternion(
        camera_pose[3], camera_pose[4], camera_pose[5], camera_pose[6]);
    const auto transform = Transformation(q, p);

    const size_t n_point = point_cloud_std.size();

    Pointcloud point_cloud = AlignedVector<Point>(n_point);
    auto colors = AlignedVector<Color>(n_point);
    for(size_t i = 0; i < n_point; ++i) {
      const auto & pt = point_cloud_std[i];
      Point point(pt[0], pt[1], pt[2]);
      point_cloud[i] = point;
      colors[i] = Color(); // currently color is just black
    }
    tsdf_integrator_->integratePointCloud(transform, point_cloud, colors);
  }

};

class PyEsdfMap {
 public:
  std::shared_ptr<EsdfMap> esdf_map_;
  std::shared_ptr<EsdfIntegrator> esdf_integrator_;  // using unique_ptr is more natual but it implicitly delete the copy constructor... so ...
  std::shared_ptr<PyTsdfMap> tsdf_map_;
  float voxel_size_;
  int voxels_per_side_;

 public:
  PyEsdfMap(float voxel_size, int voxels_per_side, float clear_sphere_radius, float occupied_sphere_radius, TsdfIntegratorType tsdf_integrator_type)
    : PyEsdfMap(Layer<EsdfVoxel>(voxel_size, voxels_per_side), clear_sphere_radius, occupied_sphere_radius, tsdf_integrator_type) {}

  PyEsdfMap(const Layer<EsdfVoxel>& esdf_layer, float clear_sphere_radius, float occupied_sphere_radius, TsdfIntegratorType tsdf_integrator_type) {
    esdf_map_.reset(new EsdfMap(esdf_layer));
    voxel_size_ = esdf_layer.voxel_size();
    voxels_per_side_ = esdf_layer.voxels_per_side();
    tsdf_map_.reset(new PyTsdfMap(esdf_layer.voxel_size(), esdf_layer.voxels_per_side(), tsdf_integrator_type));

    EsdfIntegrator::Config esdf_config;
    esdf_config.max_distance_m = 4.0f;
    esdf_config.default_distance_m = 4.0f;
    esdf_config.clear_sphere_radius = clear_sphere_radius;
    esdf_config.occupied_sphere_radius = occupied_sphere_radius;

    // setting this value as this is very important especially using smaller voxels.
    // do not use the default value.
    const float truncation_distance = 4 * voxel_size_;  // DO NOT CHANGE
    esdf_config.min_distance_m = truncation_distance / 2.0;  // DO NOT CHANGE

    const auto tsdf_layer_ptr = tsdf_map_->tsdf_map_->getTsdfLayerPtr();
    const auto esdf_layer_ptr = esdf_map_->getEsdfLayerPtr();
    esdf_integrator_.reset(new EsdfIntegrator(esdf_config, tsdf_layer_ptr, esdf_layer_ptr));
  }

  int getNumberOfAllocatedBlocks()
  {
    return esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks();
  }

  std::tuple<
    std::vector<std::vector<double>>,
    std::vector<double>,
    std::vector<bool>
    > get_voxel_info(bool only_observed)
  {
    std::vector<std::vector<double>> origins;
    std::vector<double> values;
    std::vector<bool> observed;

    auto esdf_layer_ptr = esdf_map_->getEsdfLayerPtr();

    size_t vps = esdf_layer_ptr->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    BlockIndexList block_list;
    esdf_layer_ptr->getAllAllocatedBlocks(&block_list);
    for (const BlockIndex& block_index : block_list) {
      const Block<EsdfVoxel>& block = esdf_layer_ptr->getBlockByIndex(block_index);

      for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
        const Point & coord = block.computeCoordinatesFromLinearIndex(linear_index);

        const EsdfVoxel & voxel = block.getVoxelByLinearIndex(linear_index);
        const bool is_pushable = only_observed && voxel.observed || !only_observed;
        if(is_pushable){
          origins.push_back(std::vector<double>{coord.x(), coord.y(), coord.z()});
          values.push_back(voxel.distance);
          observed.push_back(voxel.observed);
        }
      }
    }

    return std::make_tuple(origins, values, observed);
  }

  std::vector<std::vector<double>> get_block_origins()
  {
    std::vector<std::vector<double>> origins;
    BlockIndexList block_list;
    auto esdf_layer_ptr = esdf_map_->getEsdfLayerPtr();
    esdf_layer_ptr->getAllAllocatedBlocks(&block_list);
    for (const BlockIndex& block_index : block_list) {
      const Block<EsdfVoxel>& block = esdf_layer_ptr->getBlockByIndex(block_index);
      const Point block_origin = block.origin();
      const std::vector<double> origin{block_origin.x(), block_origin.y(), block_origin.z()};
      origins.push_back(origin);
    }
    return origins;
  }

  void update(std::vector<double> camera_pose, std::vector<std::vector<double>> point_cloud)
  {
    tsdf_map_->update(camera_pose, point_cloud);
    esdf_integrator_->updateFromTsdfLayer(true);
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

PyEsdfMap get_test_esdf(float voxel_size, int num_poses, int resol_x, int resol_y) {
  // originally 
  // voxel_size = 0.1 
  // num_poses = 50
  // resol_x = 320
  // resol_y = 240
  
  int voxels_per_side = 16;
  float esdf_max_distance = 4.0f;
  float truncation_distance = 4 * voxel_size; 
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

  // Next, generate poses evenly spaced in a circle around the object.
  FloatingPoint radius = 6.0;
  FloatingPoint height = 2.0;
  AlignedVector<Transformation> poses;
  poses.reserve(num_poses);

  FloatingPoint max_angle = 2 * M_PI;

  FloatingPoint angle_increment = max_angle / num_poses;
  for (FloatingPoint angle = 0.0; angle < max_angle;
       angle += angle_increment) {
    // Generate a transformation to look at the center pose.
    Point position(radius * sin(angle), radius * cos(angle), height);
    Point facing_direction = cylinder_center - position;

    FloatingPoint desired_yaw = 0.0;
    if (std::abs(facing_direction.x()) > 1e-4 ||
        std::abs(facing_direction.y()) > 1e-4) {
      desired_yaw = atan2(facing_direction.y(), facing_direction.x());
    }

    // Face the desired yaw and pitch forward a bit to get some of the floor.
    Quaternion rotation =
        Quaternion(Eigen::AngleAxis<FloatingPoint>(-0.1, Point::UnitY())) *
        Eigen::AngleAxis<FloatingPoint>(desired_yaw, Point::UnitZ());

    poses.emplace_back(Transformation(rotation, position));
  }
  std::cout << "**created camera pose list. total num is " << poses.size() << std::endl;

  // TSDF layer + integrator
  TsdfIntegratorBase::Config config;
  config.default_truncation_distance = truncation_distance;
  config.integrator_threads = 1;
  Layer<TsdfVoxel> tsdf_layer(voxel_size, voxels_per_side);
  MergedTsdfIntegrator tsdf_integrator(config, &tsdf_layer);

  // ESDF layers
  Layer<EsdfVoxel> incremental_layer(voxel_size, voxels_per_side);

  EsdfIntegrator::Config esdf_config;
  esdf_config.max_distance_m = esdf_max_distance;
  esdf_config.default_distance_m = esdf_max_distance;
  esdf_config.min_distance_m = truncation_distance / 2.0;
  esdf_config.min_diff_m = 0.0;
  esdf_config.full_euclidean_distance = false;
  esdf_config.add_occupied_crust = false;
  esdf_config.multi_queue = true;
  EsdfIntegrator incremental_integrator(esdf_config, &tsdf_layer,
                                        &incremental_layer);

  Eigen::Vector2i depth_camera_resolution_= Eigen::Vector2i(resol_x, resol_y);
  float fov_h_rad = 2.61799;
  float max_dist = 10.0;
  for (size_t i = 0; i < poses.size(); i++) {
    std::cout << "**iter num " << i << std::endl;
    Pointcloud ptcloud, ptcloud_C;
    Colors colors;

    world.getPointcloudFromTransform(poses[i], depth_camera_resolution_,
                                      fov_h_rad, max_dist, &ptcloud, &colors);
    transformPointcloud(poses[i].inverse(), ptcloud, &ptcloud_C);
    tsdf_integrator.integratePointCloud(poses[i], ptcloud_C, colors);
    std::cout << "**update tsdf" << std::endl;

    // Update the incremental integrator.
    constexpr bool clear_updated_flag = true;
    incremental_integrator.updateFromTsdfLayer(clear_updated_flag);
    std::cout << "**update esdf" << std::endl;
  }

  const auto esdf_map = PyEsdfMap(incremental_layer, 1.5, 4.0, TsdfIntegratorType::kFast);
  std::cout << "**finish creating esdf map"<< std::endl;
  return esdf_map;
}


PYBIND11_MODULE(_voxbloxpy, m) {
  m.doc() = "voxblox python wrapper";
  m.def("get_test_esdf", &get_test_esdf);

  py::enum_<TsdfIntegratorType>(m, "TsdfIntegratorType")
      .value("kSimple", TsdfIntegratorType::kSimple)
      .value("kMerged", TsdfIntegratorType::kMerged)
      .value("kFast", TsdfIntegratorType::kFast);

  py::class_<PyTsdfMap>(m, "TsdfMap")
      .def(py::init<float, int, TsdfIntegratorType>())
      .def("update", &PyTsdfMap::update)
      .def_readonly("voxel_size", &PyTsdfMap::voxel_size_)
      .def_readonly("voxels_per_side", &PyTsdfMap::voxels_per_side_);

  py::class_<PyEsdfMap>(m, "EsdfMap")
      .def(py::init<float, int, float, float, TsdfIntegratorType>())
      .def("get_sd_batch", &PyEsdfMap::get_sd_batch)
      .def("update", &PyEsdfMap::update)
      .def("get_num_alloc_block", &PyEsdfMap::getNumberOfAllocatedBlocks)
      .def("get_block_origins", &PyEsdfMap::get_block_origins)
      .def("get_voxel_info", &PyEsdfMap::get_voxel_info)
      .def_readonly("voxel_size", &PyEsdfMap::voxel_size_)
      .def_readonly("voxels_per_side", &PyEsdfMap::voxels_per_side_);

}
