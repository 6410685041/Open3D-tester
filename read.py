
import open3d as o3d
import numpy as np

print("..1")
pcd = o3d.io.read_point_cloud("kota_circuit2.ply")
print(pcd)
o3d.io.write_point_cloud("kota_circuit3.ply", pcd, write_ascii=True) 

