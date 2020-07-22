# 如何解码pcap包并将Pointcloud发送到ROS



## 1 简介

​	本文档将向您展示如何解码pcap包并将pointcloud发送到ROS。 在阅读本文档之前，请确保您已阅读LiDAR用户指南和 [参数简介](doc/intro/parameter_intro.md) 。



## 2 步骤

1. 获取LiDAR msop端口号，difop端口号和设备ip地址 

2. 设置配置文件的 *common* 部分

3. 设置配置文件的 *lidar-driver* 部分

4. 设置配置文件的 *lidar-ros* 部分

5. 运行 demo



请按照上述步骤进行高级开发，详细信息如下：



#### 步骤1

​		假设您已经根据雷达用户手册连接雷达并设置好您的电脑的IP地址。那么现在您可以使用RSView软件查看点云。此时您应该已经知道雷达的msop端口号和difop端口号，默认端口是*msop = 6699* ， *difop = 7788*。如果您还不清楚上述内容，请查看雷达用户手册。



#### 步骤2

​	设置配置文件的 *common* 部分

```yaml
common:
    msg_source: 3                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	由于消息来自pcap包，因此请设置 *msg_source = 3* 。

​    我们想将点云发送到ROS，因此设置 *send_points_ros = true* 。 

​    请确保 *pcap_directory* 是正确的。



#### 步骤3

​	设置配置文件的 *lidar-driver* 部分

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

​	将 *lidar_type* 设置为您的LiDAR类型。（RS16，RS32，RSBP，RS128）

​    设置 *device_ip* 为您的LiDAR的IP地址。默认值为 *device_ip = 192.168.1.200* 。

​	设置 *msop_port* 和 *difop_port* 为您的LiDAR端口号。 默认值为 *msop = 6699 和 difop = 7788* 。

​	

#### 步骤4

​	设置配置文件的  *lidar-ros*  部分

```yaml
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
```

​	将 *ros_send_points_topic* 设置为您要发送的话题。 



#### 步骤5

​	运行 demo



---



**这是整个配置文件的概述**

```yaml
common:
    msg_source: 3                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file

lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60021                     #The port number used for receiving pointcloud 
      points_send_port: 60021                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```







 