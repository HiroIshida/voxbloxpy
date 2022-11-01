import argparse

import numpy as np

from voxbloxpy import Grid, get_test_esdf

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualize", action="store_true", help="visualize")
    args = parser.parse_args()
    visualize: bool = args.visualize

    esdf = get_test_esdf(0.05, 6, 640, 480)
    print("finish creating esdf")

    N = 60
    lb = np.array([-3.0, -3.0, -2.0])
    ub = np.array([3.0, 3.0, 4.0])
    grid = Grid(lb, ub, (N, N, N))
    gridsdf = esdf.get_grid_sdf(grid)

    if visualize:
        gridsdf.render_volume()
