#! /bin/python3

from open3d import *
import numpy as np
import argparse

def main(pcd_path):
    cloud = io.read_point_cloud(pcd_path) # Read point cloud
    # cloud.paint_uniform_color([0, 255, 0]) # Set color to green
    cloud.colors = utility.Vector3dVector(np.random.uniform(0, 1, size=(len(cloud.points), 3)))
    voxel_grid = geometry.VoxelGrid.create_from_point_cloud(cloud,
                                                                voxel_size=0.5)
    visualization.draw_geometries([voxel_grid])    # Visualize point cloud

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 3D point cloud in pcd format using Open3D")
    parser.add_argument("pcd_path", type=str, help="Path to the .pcd file.")

    args = parser.parse_args()
    main(args.pcd_path)
