import copy
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Tuple

import numpy as np
import plotly.graph_objects as go
from scipy.interpolate import RegularGridInterpolator

from . import _voxbloxpy


class IntegratorType(Enum):
    SIMPLE = _voxbloxpy.TsdfIntegratorType.kSimple
    FAST = _voxbloxpy.TsdfIntegratorType.kFast
    MERGED = _voxbloxpy.TsdfIntegratorType.kMerged


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

    def get_linspaces(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        xlin, ylin, zlin = [np.linspace(self.lb[i], self.ub[i], self.sizes[i]) for i in range(3)]
        return xlin, ylin, zlin

    def get_meshgrid(self, indexing: str = "xy") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        xlin, ylin, zlin = self.get_linspaces()
        X, Y, Z = np.meshgrid(xlin, ylin, zlin, indexing=indexing)
        return X, Y, Z

    @property
    def resolutsions(self) -> np.ndarray:
        resols = (self.ub - self.lb) / (np.array(self.sizes) - 1.0)
        return resols


@dataclass
class GridSDF:
    grid: Grid
    values: np.ndarray
    fill_value: float
    itp: Optional[RegularGridInterpolator] = None
    create_itp_lazy: bool = False

    def get_quantized(self) -> "GridSDF":
        """quantize the sd values to well compressed by gzip"""
        ret = copy.deepcopy(self)
        resol_fine = np.min(self.grid.resolutsions) * 0.1
        resol_rough = 0.01
        indices_fine = ret.values <= 0.1
        indices_rough = ret.values > 0.1
        copy.deepcopy(ret.values)
        ret.values[indices_fine] = (
            np.floor_divide(ret.values[indices_fine], resol_fine) * resol_fine
        )
        ret.values[indices_rough] = (
            np.floor_divide(ret.values[indices_rough], resol_rough) * resol_rough
        )
        return ret

    def __post_init__(self):
        assert np.prod(self.grid.sizes) == len(self.values)
        if not self.create_itp_lazy:
            self._compute_interpolator()

    def is_finite(self) -> bool:
        return bool(np.all(np.isfinite(self.values)))

    def __call__(self, pts: np.ndarray) -> np.ndarray:
        assert pts.ndim == 2
        if self.itp is None:
            self._compute_interpolator()
            assert self.itp is not None
        return self.itp(pts)

    def _compute_interpolator(self, bounds_error: bool = False):
        xlin, ylin, zlin = self.grid.get_linspaces()
        data = np.reshape(self.values, (len(xlin), len(ylin), len(zlin)))
        self.itp = RegularGridInterpolator(
            (xlin, ylin, zlin), data, bounds_error=bounds_error, fill_value=self.fill_value
        )

    def render_volume(
        self, isomin: float = -0.5, isomax: float = 2.0, show: bool = True
    ) -> go.Figure:
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
                colorscale="jet_r",
            )
        )
        if show:
            fig.show()
        return fig

    def render_surface(self, value: float = 0.0, show: bool = True) -> go.Figure:
        X, Y, Z = self.grid.get_meshgrid()
        fig = go.Figure(
            go.Isosurface(
                x=X.flatten(),
                y=Y.flatten(),
                z=Z.flatten(),
                value=self.values,
                isomin=value,
                isomax=value,
                opacity=1.0,
                surface_count=1,
            )
        )
        if show:
            fig.show()
        return fig


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

    def get_boundary(self) -> Tuple[np.ndarray, np.ndarray]:
        """get the boundary of the observed region"""
        observed_origins = self.origins[self.observed]
        lb = np.min(observed_origins, axis=0)
        ub = np.max(observed_origins, axis=0)
        return lb, ub

    def get_boundary_grid(self, grid_size: float, margin=0.1) -> Grid:
        lb, ub = self.get_boundary()
        lb - margin
        ub + margin
        grid_nums = np.ceil(((ub - lb) / grid_size) + 1).astype(int)
        return Grid(lb, ub, grid_nums)


@dataclass
class EsdfMap:
    esdf_map_: Any

    @classmethod
    def create(
        cls,
        voxel_size: float,
        voxels_per_side: int = 16,
        clear_sphere_radius: float = 1.5,
        occupied_sphere_radius: float = 4.0,
        integrator_type: IntegratorType = IntegratorType.FAST,
    ) -> "EsdfMap":
        esdf_map_raw = _voxbloxpy.EsdfMap(
            voxel_size,
            voxels_per_side,
            clear_sphere_radius,
            occupied_sphere_radius,
            integrator_type.value,
        )
        return cls(esdf_map_raw)

    def update(self, camera_pose: CameraPose, point_cloud: np.ndarray) -> None:
        assert point_cloud.ndim == 2
        assert point_cloud.shape[1] == 3

        pose_vec = np.hstack((camera_pose.position, camera_pose.quaternion_wxyz))
        self.esdf_map_.update(pose_vec, point_cloud)

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

    def get_grid_sdf(
        self, grid: Grid, fill_value: float = np.nan, create_itp_lazy: bool = False
    ) -> GridSDF:
        X, Y, Z = grid.get_meshgrid(indexing="ij")
        pts = np.array(list(zip(X.flatten(), Y.flatten(), Z.flatten())))
        values = self.get_sd_batch(pts, fill_value=fill_value)
        return GridSDF(grid, values, fill_value, create_itp_lazy=create_itp_lazy)

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
