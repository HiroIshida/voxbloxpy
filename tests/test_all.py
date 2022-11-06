import pytest
import numpy as np
from voxbloxpy import EsdfMap, IntegratorType
from utils import Camera, CameraConfig, RayMarchingConfig, SDF

from skrobot.model import Sphere

from typing import List, Tuple

import numpy as np
import tqdm
from skrobot.model.primitives import Sphere


class TestGroup:
    radius: float = 0.4
    resol: float = 0.02

    @staticmethod
    def generate_camera_pcloud_pairs(
        sdf: SDF, camera_config: CameraConfig, rm_config: RayMarchingConfig, nx: int = 20, nz: int = 3
    ) -> List[Tuple[Camera, np.ndarray]]:
        thetas = np.linspace(0, 2 * np.pi, nx)
        phis = np.linspace(-0.7 * np.pi, 0.7 * np.pi, nz)

        pairs = []
        for theta in thetas:
            for phi in phis:
                pos = np.array([np.cos(theta) * np.cos(phi), np.sin(theta) * np.cos(phi), np.sin(phi)])
                camera = Camera(pos, config=camera_config)
                camera.look_at(np.zeros(3), horizontal=True)
                pts_global = camera.generate_point_cloud(sdf, rm_config, hit_only=True)
                pairs.append((camera, pts_global))
        return pairs

    @staticmethod
    def create_sphere_points(radius: float) -> np.ndarray:
        pts = []
        for theta in np.linspace(0, 2 * np.pi, 100):
            for phi in np.linspace(-0.4 * np.pi, 0.4 * np.pi, 10):
                pos = (
                    np.array(
                        [np.cos(theta) * np.cos(phi), np.sin(theta) * np.cos(phi), np.sin(phi)]
                    )
                    * radius
                )
                pts.append(pos)
        pts_sphere = np.array(pts)
        return pts_sphere

    @pytest.fixture()
    def esdf(self):
        sphere = Sphere(self.radius, with_sdf=True)
        pairs = self.generate_camera_pcloud_pairs(
            sphere.sdf, CameraConfig(resolx=128, resoly=96), RayMarchingConfig(max_dist=3.0), nz=5
        )
        esdf = EsdfMap.create(self.resol, integrator_type=IntegratorType.SIMPLE)
        for camera, cloud_global in tqdm.tqdm(pairs):
            cloud_camera = camera.inverse_transform_vector(cloud_global)
            esdf.update(camera.get_voxbloxpy_camera_pose(), cloud_camera)
        yield esdf

    def test_generate_point_cloud(self):  # test of utils (not main code)
        sphere = Sphere(0.2, pos=(0.2, 0.2, 0.2), with_sdf=True)
        pairs = self.generate_camera_pcloud_pairs(
            sphere.sdf, CameraConfig(resolx=64, resoly=48), RayMarchingConfig(max_dist=3.0)
        )
        pts_concat = np.concatenate([p[1] for p in pairs])

        sphere.sdf(pts_concat)

    def test_synthetic_esdf_value(self, esdf: EsdfMap):

        def check(dist_from_surface: float, admissible_rate: float):
            pts_sphere = self.create_sphere_points(self.radius + dist_from_surface)
            sd_values = esdf.get_sd_batch(pts_sphere)
            success_rate = np.sum(np.abs(sd_values - dist_from_surface) < self.resol * 2) / len(pts_sphere)
            assert success_rate > admissible_rate

        check(-0.04, 0.99)
        check(0.0, 0.99)
        check(0.05, 0.95)
        check(0.1, 0.9)
        check(0.2, 0.7)

    def test_quantization(self, esdf: EsdfMap):
        grid = esdf.get_voxel_info().get_boundary_grid(grid_size=0.01)
        grid_sdf = esdf.get_grid_sdf(grid)
        grid_sdf.get_quantized()
