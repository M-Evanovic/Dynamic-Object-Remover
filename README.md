# *Removert*

## What is removert?
- Static map construction in the wild. 
- A dynamic points removing tool by constructing a static map
- The name is from the abbreviation of our title "***Remov***e-then-re***vert***" (IROS 2020): [paper](https://irap.kaist.ac.kr/publications/gskim-2020-iros.pdf), [video](https://youtu.be/M9PEGi5fAq8)

## What can we do using removert? 
- We can easily construct and save a static map. 
- We can easily parse dynamic points 


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



## Cite *Removert*
```
@INPROCEEDINGS { gskim-2020-iros,
    AUTHOR = { Giseop Kim and Ayoung Kim },
    TITLE = { Remove, then Revert: Static Point cloud Map Construction using Multiresolution Range Images },
    BOOKTITLE = { Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS) },
    YEAR = { 2020 },
    MONTH = { Oct. },
    ADDRESS = { Las Vegas },
    NOTE = { Accepted. To appear. },
}
```

## License
 <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is supported by Naver Labs Corporation and by the National Research Foundation of Korea (NRF). This work is also licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.

## TODO (in order)
#### Near future 
- Full sequence cleaned-scan saver by automatically iterating batches (because using 50-100 scans for a single batch is recommended for computation speed)
- Adding revert steps (I think certainly removing dynamic points is generally more worthy for many applications, so reverting step is omitted currently)
- Automatically parse dynamic segments from the dynamic points in a scan (e.g., using DBSCAN on dynamic points in a scan)
- [x] Exmaples from MulRan dataset (for showing removert's availability for various LiDAR configurations) — see this [tutorial](https://youtu.be/UiYYrPMcIRU) 
- [x] (scan, pose) pair saver using SC-LeGO-LOAM or [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM#applications), which includes a loop closing that can make a globally consistent map. — see this [tutorial](https://youtu.be/UiYYrPMcIRU)
- Examples from the arbitrary datasets using the above input data pair saver.
- Providing a SemanticKITTI (as a truth) evaluation tool (i.e., calculating the number of points of TP, FP, TN, and FN) 
- (Not certain now) Changing all floats to double

#### Future 
- Real-time LiDAR SLAM integration for better odometry robust to dynamic objects in urban sites (e.g., with LIO-SAM in the Riverside sequences of MulRan dataset)
- Multi-session (i.e., inter-session) change detection example
- Defining and measuring the quality of a static map
- Using the above measure, deciding when removing can be stopped with which resolution (generally 1-3 removings are empirically enough but for highly crowded environments such as urban roads) 
