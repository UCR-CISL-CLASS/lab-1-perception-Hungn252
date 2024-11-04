from mmdet3d.apis import init_model, inference_detector
import numpy as np
import open3d as o3d
import pdb
'''
# Load binary point cloud
bin_pcd = np.fromfile('000008.bin', dtype=np.float32)

# Reshape and drop reflection values
points = bin_pcd.reshape((-1, 4))[:, 0:3]

# Convert to Open3D point cloud
o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))



# Save to whatever format you like
o3d.io.write_point_cloud("pointcloud.pcd", o3d_pcd)

pcd = o3d.io.read_point_cloud("pointcloud.pcd")
o3d.visualization.draw_geometries([pcd])
'''
config_file = 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py'
checkpoint_file = 'hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth'
model = init_model(config_file, checkpoint_file)
result=inference_detector(model, '000008.bin')
print("Result",result)
'o3d.io.write_point_cloud("pointcloud.pcd", result)'

