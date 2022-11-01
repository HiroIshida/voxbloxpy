from dataclasses import dataclass
from typing import Any, Tuple

import numpy as np
import plotly.graph_objects as go

from . import _voxbloxpy


@dataclass
class CameraPose:
    position: np.ndarray
    quaternion_wxyz: np.ndarray

    def __post_init__(self):
        assert len(self.position) == 3
        assert len(self.quaternion_wxyz) == 4


@dataclass
class Grid:
    lb: np.ndarray
    ub: np.ndarray
    sizes: Tuple[int, int, int]

    def get_meshgrid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        xlin, ylin, zlin = [np.linspace(self.lb[i], self.ub[i], self.sizes[i]) for i in range(3)]
        X, Y, Z = np.meshgrid(xlin, ylin, zlin)
        return X, Y, Z


@dataclass
class GridSDF:
    grid: Grid
    values: np.ndarray
    fill_value: float

    def __post_init__(self):
        assert np.prod(self.grid.sizes) == len(self.values)

    def render_volume(self, isomin: float = -0.5, isomax: float = 2.0) -> None:
        X, Y, Z = self.grid.get_meshgrid()
        fig = go.Figure(
            data=go.Volume(
                x=X.flatten(),
                y=Y.flatten(),
                z=Z.flatten(),
                value=self.values,
                isomin=isomin,
                isomax=isomax,
                opacity=0.05,
                surface_count=10,
                colorscale="jet",
            )
        )
        fig.show()


@dataclass
class VoxelInfos:
    origins: np.ndarray
    dists: np.ndarray
    observed: np.ndarray

    def __post_init__(self):
        assert self.origins.ndim == 2
        assert self.origins.shape[1] == 3
        assert len(self.origins) == len(self.dists)
        assert len(self.origins) == len(self.observed)

    def filter(self, min_val: float, max_val: float) -> "VoxelInfos":
        indices = self.observed & (self.dists > min_val) & (self.dists < max_val)
        return VoxelInfos(self.origins[indices], self.dists[indices], self.observed[indices])


@dataclass
class EsdfMap:
    esdf_map_: Any

    @classmethod
    def create(cls, voxel_size: float, voxels_per_side: int = 16) -> "EsdfMap":
        esdf_map_raw = _voxbloxpy.EsdfMap(voxel_size, voxels_per_side)
        return cls(esdf_map_raw)

    def update(self, camera_pose: CameraPose, point_cloud: np.ndarray) -> None:
        assert point_cloud.ndim == 2
        assert point_cloud.shape[1] == 3

        pose_vec = np.hstack((camera_pose.position, camera_pose.quaternion_wxyz))
        is_wrt_camera = True  # currently False is buggy
        self.esdf_map_.update(pose_vec, point_cloud, is_wrt_camera)

    def get_block_origins(self) -> np.ndarray:
        return self.esdf_map_.get_block_origins()

    def get_voxel_info(self, observed_only: bool = True) -> VoxelInfos:
        ret = self.esdf_map_.get_voxel_info(observed_only)
        ret = [np.array(e) for e in ret]
        return VoxelInfos(*ret)

    def get_sd_batch(self, points: np.ndarray, fill_value: float = np.nan) -> np.ndarray:
        assert points.ndim == 2
        assert points.shape[1] == 3
        fill_value_internal = 10000  # something too large
        dists = np.array(self.esdf_map_.get_sd_batch(points, fill_value_internal))
        dists[dists > fill_value_internal - 1] = fill_value
        return dists

    def get_grid_sdf(self, grid: Grid, fill_value: float = np.nan) -> GridSDF:
        X, Y, Z = grid.get_meshgrid()
        pts = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))
        values = self.get_sd_batch(pts, fill_value=fill_value)
        return GridSDF(grid, values, fill_value)

    @property
    def voxel_size(self) -> int:
        return self.esdf_map_.voxel_size

    @property
    def voxels_per_side(self) -> int:
        return self.esdf_map_.voxels_per_side


def get_test_esdf(
    voxel_size: float = 0.05, voxels_per_side: int = 10, pixel_x: int = 640, pixel_y: int = 480
) -> EsdfMap:
    esdf_map_raw = _voxbloxpy.get_test_esdf(voxel_size, voxels_per_side, pixel_x, pixel_y)
    esdf_map_wrapped = EsdfMap(esdf_map_raw)
    return esdf_map_wrapped
