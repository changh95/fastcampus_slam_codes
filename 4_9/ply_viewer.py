#! /bin/python3

from open3d import *    
import argparse

def main(pcd_path):
    cloud = io.read_point_cloud(pcd_path) # Read point cloud
    visualization.draw_geometries([cloud])    # Visualize point cloud      

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 3D point cloud in pcd format using Open3D")
    parser.add_argument("pcd_path", type=str, help="Path to the .pcd file.")
    
    args = parser.parse_args()
    main(args.pcd_path)
