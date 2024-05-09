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

