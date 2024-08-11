# Dolbat 2024 Future Automotive SW Competition Repository
한양대학교 자동차지능연구실 학부연구생 박성훈 지도하에 만듬 ㅇㅇ

## Prerequisites
* for Ubuntu 20.04 (noetic)
```bash
# for rosserial package
$ sudo apt-get install ros-noetic-rosserial-arduino
$ sudo apt-get install ros-noetic-rosserial
```

## launch
### for arduino node (rosserial package)
```bash
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
```
### for lane detection node (sliding windows algorithm)
```bash
$ roslaunch lane_detection lane_detection.launch
```

## etc
### how to covert .mp4 to .bag
* must have video mp4 file in resources folder
* check file names and dirs in source code 
```bash
$ python3 utils/video_2_rosbag.py
```

### competetion link
http://futurecar.cafe24.com/core/?cid=101&uid=6&role=view#
