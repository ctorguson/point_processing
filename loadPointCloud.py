#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import sys

def load_point_cloud(tempFile):
    try:
        point_cloud = o3d.io.read_point_cloud(tempFile)
        return point_cloud
    except Exception as e:
        print(f"Error reading point cloud: {e}")
        sys.exit(1)

def main():
    tempFile = '/Users/ciaratorguson/point_processing/tempPointCloud.ply'
    point_cloud = load_point_cloud(tempFile)
    if point_cloud is not None:
        points = np.asarray(point_cloud.points)
        if len(points) == 0:
            print("Warning: Loaded point cloud contains no points.")
            sys.exit(1)
        for point in points:
            print("{:.6f} {:.6f} {:.6f}".format(point[0], point[1], point[2]))

if __name__ == "__main__":
    main()
