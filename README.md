# 1. Installation

Tested on Raspberry Pi 5, Trixie

## Prerequisites

- ROS2\
Jazzy, see [rospian](https://github.com/rospian/rospian-repo). Lyrical might work but not tested, see [Ar-Ray-code](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)

- [GTSAM](https://github.com/borglab/gtsam) >= 4.2
```bash
sudo apt-get install libtbb-dev libboost-all-dev
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF ..
make
sudo make install
```
- [OpenCV](https://github.com/opencv/opencv) >= 3.4
```bash
sudo apt install libopencv-dev libopencv-contrib-dev
```
- [OpenGV](https://github.com/laurentkneip/opengv)
```bash
git clone https://github.com/laurentkneip/opengv.git
cd opengv
mkdir build
cd build
# Replace path to your GTSAM's Eigen
cmake -DEIGEN_INCLUDE_DIR=/home/pi/gtsam/gtsam/3rdparty/Eigen -DEIGEN_INCLUDE_DIRS=/home/pi/gtsam/gtsam/3rdparty/Eigen ..
```
- [Glog](http://rpg.ifi.uzh.ch/docs/glog.html), [Gflags](https://gflags.github.io/gflags/)
```bash
sudo apt-get install libgflags-dev libgoogle-glog-dev
```

## Build Kimera-VIO
```bash
git checkout openmv_ae3
cd Kimera-VIO
mkdir build
cd build
cmake ..
make
```

# 2. Usage
```bash
cd Kimera-VIO
./scripts/monoVIO_openmv_ae3.bash
```
