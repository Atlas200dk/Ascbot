# 引言

用户可以根据此文档制作及安装ascbot小车操作系统。若用户SD卡中预置的系统损坏，可以根据此文档重新制作完好的系统。

Atlas 200 DK的使用教程参见华为官网教程：[华为官方教程](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/overview)



# 1. Atlas 200 DK系统SD卡制作

​        参见[华为官网制作sd卡教程](https://ascend.huawei.com/doc/atlas200dk/1.3.0.0/zh/zh-cn_topic_0195268775.html) 



# 2. Atlas 200 DK 安装ROS系统

​       将SD卡插入Atlas 200 DK，启动并按照华为教程将Atlas 200 DK接入internet。
​       安装ROS系统，ROS的安装可以参照[ROS官方教程](http://wiki.ros.org/kinetic/Installation/Ubuntu)




# 3. Ascbot程序的下载和安装
​       1）首先在PC端上面，从码云上下载最新[release](https://gitee.com/Atlas200DK/Ascbot) 版本，解压缩到本地路径 ./ascbot

​       2）在PC端通过ssh远程登录到Atlas 200 DK 系统   用户名： root    密码：Mind@123

​       3）把scp 命令把 ./ascbot目录 从PC拷贝到Atlas 200 DK的  /home/root/ 目录

​       4）执行脚本  /home/root/ascbot/install_version.sh，完成安装以及部署。

​              确认输出信息，确认所有组件安装成功。

​       5）上电重启Atlas 200 DK



# 4. 安装手机端控制应用
​        1）下载手机端控制应用ascbot-remotectrl.apk ，[下载地址](https://gitee.com/Atlas200DK/Ascbot/raw/master/Release/ascbot-remotectrlv1.0.apk)

​        2）安装：ascbot-remotectrlv1.0.apk 到手机



# 5. 系统SD卡备份和恢复

- 备份

  1）确认使用的PC支持dd命令, 将上面做好的SD卡插入读卡器，连接到PC，手动unmount SD卡文件系统

  2）执行 dd if=/dev/sdb  of=./backup/AscbotOS-YYYYMMDD.img

- 恢复

  dd if=./backup/AscbotOS-YYYYMMDD.img of=/dev/sdb

  注：假定SD卡挂载节点是sdb，需要根据实际挂载节点修改

  

