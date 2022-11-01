import argparse
import numpy as np

from voxbloxpy import CameraPose, EsdfMap


def create_esdf(sphere: bool = True, debug_view: bool = True):
    quat_wxyz = np.array([0.0, 1.0, 0.0, 0.0])
    pos = np.array([-1.0, 0.0, 0.0])
    camera_pose = CameraPose(pos, quat_wxyz)

    r_sphere = 0.5
    ylin = np.linspace(-1.0, 1.0, 100)
    zlin = np.linspace(-1.0, 1.0, 100)
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
    block_origins = esdf.get_block_origins()
    print(len(block_origins))

    info = esdf.get_voxel_info()
    info0 = info.filter(-0.1, -0.0)
    info1 = info.filter(0.0, 0.1)
    info2 = info.filter(0.1, 0.2)
    info3 = info.filter(0.2, 0.3)
    info4 = info.filter(0.3, 0.4)

    if visualize:
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.scatter(info0.origins[:, 0], info0.origins[:, 1], info0.origins[:, 2], c="yellow")
        ax.scatter(info1.origins[:, 0], info1.origins[:, 1], info1.origins[:, 2], c="red")
        ax.scatter(info2.origins[:, 0], info2.origins[:, 1], info2.origins[:, 2], c="blue")
        ax.scatter(info3.origins[:, 0], info3.origins[:, 1], info3.origins[:, 2], c="green")
        ax.scatter(info4.origins[:, 0], info4.origins[:, 1], info4.origins[:, 2], c="black")
        plt.show()
