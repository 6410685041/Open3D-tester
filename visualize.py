import open3d as o3d

pcd = o3d.io.read_point_cloud("data/kota-3.ply")
o3d.visualization.draw_geometries([pcd])