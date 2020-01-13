# 引言

用户可以根据此文档制作及安装ascbot小车操作系统。若用户SD卡中预置的系统损坏，可以根据此文档重新制作完好的系统。

Atlas 200 DK的使用教程参见华为官网教程：[华为官方教程](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/overview)



# 1. Atlas 200 DK系统SD卡制作

参见[华为官网制作sd卡教程](https://ascend.huawei.com/doc/atlas200dk/1.3.0.0/zh/zh-cn_topic_0195268775.html) 
开发板用的Atlas 200 DK运行包是mini_developerkit-1.1.T22.B883.rar，其他的版本没有测试。


# 2. Atlas 200 DK 安装ROS系统

​       将SD卡插入Atlas 200 DK，启动并按照华为教程将Atlas 200 DK接入internet,指的是能够链接外网。
​       首先进入开发板root用户下

        $ su -
      
​       安装ROS系统，开发用的版本是ros1 kentic的版本，如果有安装失败，命令行会有对应的错误输出；
​       ROS的安装可以参照 [ROS官方教程](http://wiki.ros.org/kinetic/Installation/Ubuntu), [中文安装教程](https://www.ncnynl.com/archives/201801/2273.html)。
​       同时按照官方的教程，在开发板中创建ros的工作空间；按照如下的方式创建工作空间也可以：
​       


        $ mkdir -p ~/home/catkin_ws/src
        $ cd ~/home/catkin_ws/src
        $ catkin_init_workspace
        $ cd ..
        $ catkin_make
        $ source /opt/ros/kinetic/setup.bash
        $ source devel/setup.bash

  要想保证工作空间已配置正确需确保ROS_PACKAGE_PATH环境变量包含你的工作空间目录，采用以下命令查看：

        $ echo $su ROS_PACKAGE_PATH
        /root/home/catkin_ws/src/:/opt/ros/kinetic/share


​        
​       



# 3. Ascbot程序的下载和安装
1）首先把无线路由器的锁关掉，以防终端链接不成功wifi

​2）首先在PC端上面下载最新[ Release版本 ](https://thundercomm.s3.ap-northeast-1.amazonaws.com/public/Ascbot/AscbotAppRelease.zip) 版本，解压缩到本地路径，例如~/

     Ubuntu:~ Ubuntu$ ls -l ~
     total 0
     drwx------@   3 Ubuntu  staff    96 10  8 17:57 Applications
     drwxr-xr-x@   8 Ubuntu  staff   256 12 17 13:50 AscbotAppRelease

3）用scp 命令把解压出来的文件夹（例如~/AscbotAppRelease） 从PC拷贝到Atlas 200 DK的 HwHiAiUser@192.168.3.9:~ 目录下
    
     Ubuntu:~ Ubuntu$ scp -r ~/AscbotAppRelease HwHiAiUser@192.168.3.9:~
     HwHiAiUser@192.168.3.9's password: 
     fixStartupOptions.sh                                    100%  726   190.1KB/s   00:00    
     libRFInferenceEngine.so                                 100% 4363KB   2.1MB/s   00:02    
     road_object_following_graph.config                      100% 2072   395.7KB/s   00:00 
     ...
     killAllProcess.sh                                       100%  235   110.9KB/s   00:00    
     rosAutoInstall.sh                                       100% 4705     1.5MB/s   00:00    

4）HwHiAiUser用户下执行升级操作，终端输入命令 su - ，输入密码

    ssh HwHiAiUser@192.168.3.9
    HwHiAiUser@192.168.3.9's password: 
    Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.1.46+ aarch64)
    * Documentation:  https://help.ubuntu.com
    * Management:     https://landscape.canonical.com
    * Support:        https://ubuntu.com/advantage
    Last login: Tue Aug  6 15:46:51 2019 from 192.168.3.2

5）HwHiAiUser用户下执行升级操作，终端输入命令 su - ，输入密码


    HwHiAiUser@davinci-mini:~$ su -
    Password: 
    root@davinci-mini:~# pwd
    /root

6）拷贝安装文件夹到 ~/home/catkin_ws， 然后执行脚本  AscbotAppRelease/rosAutoInstall.sh，完成安装以及部署。确认输出信息，确认所有组件安装成功。
7）重启Atlas 200 DK.

    root@davinci-mini:~# cd home/catkin_ws/
    root@davinci-mini:~/home/catkin_ws# cp -r /home/HwHiAiUser/AscbotAppRelease ./
    root@davinci-mini:~/home/catkin_ws# ls -l
    total 20
    drwxr-xr-x  4 root root 4096 Aug  6 16:04 AscbotAppRelease
    ...
    
    root@davinci-mini:~/home/catkin_ws# cd AscbotAppRelease/
    root@davinci-mini:~/home/catkin_ws/AscbotAppRelease# ls
    AscbotCameraRtspService  fixStartupOptions.sh  modelConfig.txt
    AscbotSource             killAllProcess.sh     rosAutoInstall.sh
    
    root@davinci-mini:~/home/catkin_ws/AscbotAppRelease# ./rosAutoInstall.sh 
    ...
    [100%] Built target phone_talker_node
    [100%] Built target roslog_generate_messages
    /root/home/catkin_ws/AscbotAppRelease
    start source some bashrc
    source end
    all program installed,you can test all
    tips!!!!!!!! you try to deal this command what commanc is 'source /root/home/catkin_ws/devel/setup.bash' in the terminal by yourself
    =====================end========================
    print params 2
    tips!!  ok, you can reboot.
    root@davinci-mini:~/home/catkin_ws/AscbotAppRelease# 


 


# 4. 安装手机端控制应用
​        1）下载手机端控制应用，[下载地址](https://thundercomm.s3.ap-northeast-1.amazonaws.com/public/Ascbot/ascbot-remotectrlv1.0.apk)

​        2）安装：ascbot-remotectrl.apk 到手机
​        
​        3）手机扫码安装：
​        ![ascbot-remotectrl.apk](./AscbotImg/hrobot-remotecontrol-app.png) 



# 5. 系统SD卡备份和恢复

- 备份

  1）确认使用的PC支持dd命令, 将上面做好的SD卡插入读卡器，连接到PC，手动unmount SD卡文件系统

  2）执行 dd if=/dev/sdb  of=./backup/AscbotOS-YYYYMMDD.img

- 恢复

  dd if=./backup/AscbotOS-YYYYMMDD.img of=/dev/sdb

  注：假定SD卡挂载节点是sdb，需要根据实际挂载节点修改

  
