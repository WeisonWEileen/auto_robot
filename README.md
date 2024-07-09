### 
ros2 bag录制场上数据，场后开启

```
sudo apt-get install ros-humble-ros2bag \
                     ros-humble-rosbag2-converter-default-plugins \
                     ros-humble-rosbag2-storage-default-plugins
```

开启

```
ros2 bag record /topic_name
```