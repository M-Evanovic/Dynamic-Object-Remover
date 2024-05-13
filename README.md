# *Removert*
## Preparations
```
data  
├ poses.txt / pose.csv  
├ Scans  
│  ├ 000001.pcd  
│  ├ 000002.pcd  
│  ├ 000003.pcd  
│  ├ ...  
```


## How to use 
- First, compile the source 
```
$ mkdir ~/remover_ws/src
$ cd ~/remover_ws/src
$ git clone https://github.com/M-Evanovic/Remove-then-Revert.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
- Before to start the launch file, you need to replace data paths in the `config/params.yaml` file.  
Edit `config/params.yaml` to set parameters:  
```
load_dir: data的路径  
result_dir: 结果保存路径  
is_large: 雷达帧数多、地图大，可调为True  
Transformation_LiDAR2IMU: 雷达到IMU的标定矩阵  
start_idx, end_idx: 起始帧与结束帧索引  
FOV_V, FOV_H: 垂直与水平视场角  
resize_ratio: 分辨率比例。越大动态点检测能力越强  
need_revert: 是否恢复点  
grid_size: 栅格大小。动态点检测到，越大滤除能力越强  
```

- Then, you can start the *Removert*
```
$ roslaunch remover remove.launch 
```



## Reference
[https://github.com/url-kaist/patchwork-plusplus]  
[https://github.com/gisbi-kim/removert]

