import argparse
import numpy as np

from voxbloxpy import CameraPose, EsdfMap, Grid


def create_esdf(sphere: bool = True, debug_view: bool = True):
    quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
    pos = np.array([-1.0, 0.6, 0.0])
    camera_pose = CameraPose(pos, quat_wxyz)

    r_sphere = 0.5
    ylin = np.linspace(-1.0, 1.0, 400)
    zlin = np.linspace(-1.0, 1.0, 400)
    y_grid, z_grid = np.meshgrid(ylin, zlin)

    if sphere:
        pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
        inside_circle = np.sum(pts_yz_plane**2, axis=1) < r_sphere - 0.02
        pts_yz_plane_inner = pts_yz_plane[inside_circle]
        pts_x = -np.sqrt(1.0 - np.sum(pts_yz_plane_inner**2, axis=1)) + 1.0  # 1.0 for camera

        pts = np.zeros((len(pts_x), 3))
        pts[:, 0] = pts_x
        pts[:, 1:] = pts_yz_plane_inner
    else:
        pts_yz_plane = np.array(list(zip(y_grid.flatten(), z_grid.flatten())))
        pts_x = np.zeros(len(pts_yz_plane))
        pts = np.zeros((len(pts_x), 3))
        pts[:, 0] = pts_x
        pts[:, 1:] = pts_yz_plane

    if debug_view:
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
        plt.show()

    esdfmap = EsdfMap.create(0.05, 32)
    esdfmap.update(camera_pose, pts)

    return esdfmap


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true", help="visualize")
    args = parser.parse_args()
    visualize: bool = args.visualize

    esdf = create_esdf(sphere=True, debug_view=False)

    if visualize:
        grid = Grid(np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 0.5]), (60, 60, 60))
        grid_sdf = esdf.get_grid_sdf(grid)
        grid_sdf.render_volume()
