# MAP IO

---

# 1.功能描述
hpa地图模块SDK
[SDK文档](https://ffotfsfj55.feishu.cn/wiki/wikcnwnW8RNYDFdV4HVCyufyE6b)

---

# 2.依赖

## 2.1 C++14 or C++0x Compiler
c++14及以上版本。

## 2.1 ROS
[ROS安装指南](https://ffotfsfj55.feishu.cn/wiki/wikcn7AVgqUMFP5MGGnP6k1VyTc?from=from_copylink)

## 2.2 Eigen
Eigen3.2.92及以上。
```
sudo apt-get install libeigen3-dev
```

# 3.编译和运行
pc端ros程序编译命令
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="map_io"
```
生成的库文件路径： lib/ros/libmap_io.so

pc端不依赖ROS版本
需要将map_io.h中第18行（#define USE_ROS ///默认编译的为ROS版）注释掉
```bash
mkdir build
cd build
cmake -DWITH_ROS=false ..
make -j install
```
生成的库文件路径： lib/libmap_io.so
默认的地图路径在：~/maps/ (如果需要修改需要重新编译)

tda4端程序编译命令
```$bash
bash ${sdk_dir}/build.sh HA1  #${sdk_dir}为本地的tda4_env路径
```
生成的库文件路径： lib/tda4/libmap_io.so lib/tda4/libmap_io.a

# 其他

## 1. 例程

### 1.1 读取地图信息(read)
功能说明：读取地图数据并打印

运行命令
```
rosrun map_io read {map_name.bin}
```
eg.
```plain text
map version: 100111
-------------map header------------
id: 0 name: map0_2023-4-11-10-16-29.bin
map lat lon alt: 
  31.09772806  121.52265721 13.5690002441
map attitude: 
  q: x y z w -0 -0 -0.096631705761 0.99532020092
  yaw pitch roll: -11.0904836655             -0             -0
map start p: 
  18.0929 9.32789 1.34083
map start attitude: 
  q: x y z w -0.00472156 -0.00390031 0.71859 0.695407
  yaw pitch roll:   91.8781 0.0779868 -0.697437
-------------map path------------
path points num: 427
 18.0861 -9.44081 -13.3556
  q: x y z w 0.69597 -0.718045 -0.00355151 0.00494362
  yaw pitch roll:   -91.788 -0.123529   179.313
 direction(-1: backward, 0: standstill, 1: forward): 1
...
-------------semantic map------------
semantic mark num: 68
 -id: 1 type: 0 flag: 0
  center:    12.48 -12.1506 -1.26927
  q: x y z w 0.708565 0.705646 3.08447e-08 -3.09724e-08
  yaw pitch roll:      89.7634 -5.00891e-06         -180
point size: 4............................
  point:       0 1.29882 13.2264
  std, flag: 14.9686 9.98177       0 32571
  point: -1.05782e-22 -1.05782e-22            0
  std, flag: 1.23751 13.3062 11.0648 1092607535
  point:           0 4.56417e-41 4.56417e-41
  std, flag: -1.05782e-22            0      1.23973 1067878321
  point:  11.005 14.9778       0
  std, flag:            0  4.56417e-41 -1.05782e-22 0
...
```

### 1.2 写入地图信息(write)
功能说明：随机生成地图信息并写入到地图存储路径，生成文件名为test.bin

运行命令
```
rosrun map_io write
```

log内容和read函数完全一致，对于同一个数据包，读写的程序的log应该一致，可以使用
以下命令对比输入内容是否一致
```bash
diff file1 file2
```