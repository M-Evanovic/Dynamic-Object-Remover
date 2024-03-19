# Dynamic-Object-Remover
remove dynamic object in 3D pointcloud map

# Prerequisites
data  
├ poses.txt / pose.csv  
├ Scans  
│  ├ 000001.pcd  
│  ├ 000002.pcd  
│  ├ 000003.pcd  
│  ├ ...  

# Get start
roslaunch remover remove.launch  


# Params
./config/params.yaml  

load_dir: data的路径  
result_dir: 结果保存路径  
is_large: 雷达帧数多，调为True  
Transformation_LiDAR2IMU: 雷达到IMU的标定矩阵  
start_idx, end_idx: 起始帧与结束帧  
FOV_V, FOV_H: 垂直与水平视场角  
resize_ratio: 越大动态点检测能力越强
need_revert: 是否恢复点
grid_size: 动态点检测到，越大滤除能力越强  
