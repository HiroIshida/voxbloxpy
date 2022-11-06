import copy
from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np
from skrobot.coordinates.geo import orient_coords_to_axis
from skrobot.coordinates.math import matrix2quaternion, rotation_matrix_from_axis
from skrobot.model.primitives import Axis, Coordinates
from voxbloxpy.core import CameraPose


class SDF(Protocol):
    def __call__(self, pts: np.ndarray) -> np.ndarray:
        ...


@dataclass
class CameraConfig:
    resolx: int = 640
    resoly: int = 480
    fovx: float = np.deg2rad(57.0)
    fovy: float = np.deg2rad(43.0)


@dataclass
class RayMarchingConfig:
    surface_closeness_threashold: float = 1e-3
    max_iter: int = 30
    max_dist: float = 2.0
    terminate_active_rate_threshold: float = 0.005


class Camera(Axis):
    config: CameraConfig

    def __init__(
        self, camera_pos: Optional[np.ndarray] = None, config: Optional[CameraConfig] = None
    ):
        if config is None:
            config = CameraConfig()
        super().__init__(axis_radius=0.02, axis_length=0.3, pos=camera_pos)
        self.config = config

    def look_at(self, p: np.ndarray, horizontal: bool = True) -> None:
        if np.all(p == self.worldpos()):
            return
        diff = p - self.worldpos()
        orient_coords_to_axis(self, diff, axis="x")
        if horizontal:
            co = Coordinates(
                pos=self.worldpos(), rot=rotation_matrix_from_axis(diff, [0, 0, 1], axes="xz")
            )
            self.newcoords(co)

    def generate_point_cloud(
        self,
        sdf: SDF,
        rm_config: RayMarchingConfig,
        hit_only: bool = False,
    ) -> np.ndarray:
        """Generate a point cloud wrt global coordinate"""
        co_proj_plane = self.copy_worldcoords()
        co_proj_plane.translate(np.array([1.0, 0.0, 0.0]))

        half_width = np.tan(self.config.fovx * 0.5)
        half_height = np.tan(self.config.fovy * 0.5)

        wlin = np.linspace(-half_width, half_width, self.config.resolx)
        hlin = np.linspace(-half_height, half_height, self.config.resoly)
        W, H = np.meshgrid(wlin, hlin)

        # First, create a points w.r.t projection plane
        grid_wh_local = np.array(list(zip(W.flatten(), H.flatten())))
        grid_local = np.hstack((np.zeros((len(grid_wh_local), 1)), grid_wh_local))
        grid_global = co_proj_plane.transform_vector(grid_local)

        ray_start = self.worldpos()
        ray_starts = np.tile(np.expand_dims(ray_start, axis=0), (len(grid_global), 1))
        ray_directions = grid_global - ray_start

        norms = np.linalg.norm(ray_directions, axis=1)
        ray_directions_unit = ray_directions / norms[:, None]

        dists = self.ray_marching(ray_starts, ray_directions_unit, sdf, rm_config)
        clouds = ray_starts + dists[:, None] * ray_directions_unit

        if hit_only:
            hit_indices = dists < rm_config.max_dist - 1e-3
            clouds = clouds[hit_indices]
        return clouds

    @staticmethod
    def ray_marching(
        pts_starts, direction_arr_unit, f_sdf, rm_config: RayMarchingConfig
    ) -> np.ndarray:
        pts_starts = copy.deepcopy(pts_starts)
        ray_tips = pts_starts
        n_point = len(pts_starts)
        active_flags = np.ones(n_point).astype(bool)
        frying_dists = np.zeros(n_point)
        for _ in range(rm_config.max_iter):
            active_indices = np.where(active_flags)[0]

            sd_vals_active = f_sdf(ray_tips[active_indices])
            diff = direction_arr_unit[active_flags] * sd_vals_active[:, None]
            ray_tips[active_indices] += diff
            frying_dists[active_indices] += sd_vals_active

            is_close_enough = sd_vals_active < rm_config.surface_closeness_threashold
            active_flags[active_indices[is_close_enough]] = False

            is_far_enough = frying_dists[active_indices] > rm_config.max_dist
            active_flags[active_indices[is_far_enough]] = False
            frying_dists[active_indices[is_far_enough]] = rm_config.max_dist
            ray_tips[active_indices[is_far_enough]] = np.inf

            active_ratio = np.sum(active_flags) / n_point
            if active_ratio < rm_config.terminate_active_rate_threshold:
                break

        frying_dists[active_indices] = rm_config.max_dist  # type: ignore
        modified_dists = np.minimum(frying_dists, rm_config.max_dist)
        return modified_dists

    def get_voxbloxpy_camera_pose(self) -> CameraPose:
        pos = self.worldpos()
        rot = self.worldrot()
        quat = matrix2quaternion(rot)
        return CameraPose(pos, quat)
