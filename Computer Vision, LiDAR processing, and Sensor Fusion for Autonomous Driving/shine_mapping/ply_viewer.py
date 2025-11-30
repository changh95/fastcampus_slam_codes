#! /bin/python3

from open3d import *    
import argparse

def main(ply_path):
    cloud = io.read_point_cloud(ply_path) # Read point cloud
    visualization.draw_geometries([cloud])    # Visualize point cloud      

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 3D point cloud in ply format using Open3D")
    parser.add_argument("ply_path", type=str, help="Path to the .ply file.")
    
    args = parser.parse_args()
    main(args.ply_path)
