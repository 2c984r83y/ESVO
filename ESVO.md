# ESVO
## Installations
### New ROS workspace for catkin build
```bash
mkdir -p ~/rosCatkinBuild/src
cd rosCatkinBuild/src
catkin_init_workspace
cd ..
catkin build
```
### Install rpg_dvs_ros
>https://github.com/uzh-rpg/rpg_dvs_ros  

### Clone the repository
```bash
cd ~/rosCatkinBuild/src
git clone git clone https://github.com/HKUST-Aerial-Robotics/ESVO.git
```
### Install dependence
```bash
sudo apt install python3-catkin-tools python3-osrf-pycommon
cd ~/rosCatkinBuild/src
sudo apt-get install python3-vcstool
vcs-import < ESVO/dependencies.yaml
sudo apt-get install autoconf
```
Install yaml-cpp
```bash
cd ~/catkin_ws/src 
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j8
```
### Fix bug
```
In file included from /home/yousa/rosCatkinBuild/src/ESVO/esvo_core/src/esvo_Mapping.cpp:13:
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h: In member function ‘std::vector<int> pcl::VoxelGrid<PointT>::getNeighborCentroidIndices(const PointT&, const MatrixXi&) const’:
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h:340:21: error: ‘Index’ is not a member of ‘Eigen’
  340 |         for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
      |                     ^~~~~
```
```
In file included from /home/yousa/rosCatkinBuild/src/ESVO/esvo_core/src/esvo_Mapping.cpp:13:
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h: In member function ‘std::vector<int> pcl::VoxelGrid<pcl::PCLPointCloud2>::getNeighborCentroidIndices(float, float, float, const MatrixXi&) const’:
/usr/include/pcl-1.10/pcl/filters/voxel_grid.h:670:21: error: ‘Index’ is not a member of ‘Eigen’
  670 |         for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
      |                     ^~~~~
```
```bash
sudo gedit /usr/include/pcl-1.10/pcl/filters/voxel_grid.h
```
Line 340 and 670:
```cpp
        //for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
        for (int ni = 0; ni < relative_coordinates.cols (); ni++)
```
### Install ESVO
```bash
cd ~/rosCatkinBuild
catkin build esvo_time_surface esvo_core
```

## Run ESVO

### Donwload dataset
>https://sites.google.com/view/esvo-project-page/home  

### Run Timesurface

**TODO:研究如何在launch中给rqt传参**
**已修复**
launch file最后几行给rqt传参有bug

```bash
gedit \home\yousa\rosCatkinBuild\src\ESVO\esvo_time_surface\launch\stereo_time_surface.launch
```
所以去掉这几行，手动在终端中启动rqt
```bash
  <!-- Visualization -->
  <node pkg="rqt_gui" type="rqt_gui" name="rqt_gui"
        args="--perspective-file $(find esvo_time_surface)/esvo_time_surface.perspective" />
```
#### 修改bag文件地址
```bash
gedit yousa\rosCatkinBuild\src\ESVO\esvo_time_surface\launch\rosbag_launcher\hkust\hkust_lab.launch
```
修改其中hkust_lab.bag 地址，其他launch file同理
#### Timesurface  
```bash
roslaunch esvo_time_surface hkust_lab.launch
roslaunch esvo_time_surface stereo_time_surface.launch
```
### Run ESVO

**TODO：如何在orbslam3中sub timesurface topic**
**修改orbslam3订阅的topic名称，但是timesuface有拖影，还不如用原始图像
注：要在同一个workspace才能传递topic，所以需要在.zshrc中`source rosCatkinBuild/devel/setup.zsh`  
然后注释掉`source rosworkspace/devel/setup.zsh`**

一开始运行时报错：
```bash
[rviz-7] killing on exit
[rqt_gui-6] killing on exit
[esvo_Tracking-5] killing on exit
[esvo_Mapping-4] killing on exit
[global_timer-3] killing on exit
[TimeSurface_right-2] killing on exit
[TimeSurface_left-1] killing on exit
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
shutting down processing monitor...
... shutting down processing monitor complete
done
```

void test(int a, int b, int c)
boost::bind(test, 1, _1, _2)得到一个函数对象b，当我们调用b(3,4)时，相当于调用test(1,3,4)
boost::bind(test, _2, 3, _1)得到一个函数对象b，当我们调用b(3,4)时，相当于调用test(4,3,3)

解决方案：（有点莫名其妙少了几行代码）
>https://github.com/HKUST-Aerial-Robotics/ESVO/blob/master/esvo_core/src/esvo_Mapping.cpp  

代码ctrl+c ctrl+v复制下来，就能跑了???
### Run ESVO
#### HKUST DATASET 
**有bug**
```bash
roslaunch esvo_core system_hkust.launch
roslaunch esvo_time_surface system_hkust.launch
```
#### RPG DATASET
```bash
roslaunch esvo_core system_rpg.launch 
roslaunch esvo_time_surface rpg_bin.launch 
```
#### UPENN DATASET
```bash
roslaunch esvo_core system_upenn.launch
roslaunch esvo_time_surface upenn_indoor_flying1.launch
```


