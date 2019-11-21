# Ascbot智能小车开发环境搭建指南	



Ascbot智能小车的开发包括算法、系统应用和训练服务器环境等j搭建方法说明。



# 1. Atlas 200 DK开发环境搭建
MindStudio及DDK的安装具体请参考[华为Atlas官网](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/zh-cn_topic_0178961797.html)网站



# 2. 算法训练服务器环境搭建

算法训练的环境搭建请参考[Caffe安装](https://github.com/holzers/caffe-ssd/blob/master/docs/installation.md) 



# 3. Atlas 200 DK的ROS环境搭建

如果用Ascbot自带的系统sd卡，ros环境已经具备，请跳过ros环境搭建步骤。

首先，连接小车无线路由器，配置无线路由器接入互联网，从无线路由器web页面查看Atlas 200 DK的ip,具体请参考华为无线路由器的使用说明。



## 3.1 登录Atlas 200 DK

PC连接小车路由器后，在PC端打开命令行终端，输入以下命令：

```
$ ssh HwHiAiUser@192.168.x.x
```

输入密码‘Mind@123’，按下enter键。 
申请root权限，所以在终端输入命令：

```
$ su -
```
输入密码"Mind@123"，若提示失败，请重新尝试登录。 
 ros环境安装需要dns，修改的文件名是/etc/resolve.conf，在终端输入命令：

```
$  vim /etc/resolv.conf 
```
在文件最后添加以下文字, 保存文件。
```
 nameserver 8.8.8.8
 nameserver 8.8.4.4
 nameserver 127.0.0.1
```


## 3.2 更换Atlas 200 DK的安装源
添加安装源，打开 /etc/apt/sources.list，添加以下内容，并保存。

        deb http://ports.ubuntu.com/ubuntu-ports/ xenial main
        deb http://ports.ubuntu.com/ubuntu-ports/ xenial-security main
        deb http://ports.ubuntu.com/ubuntu-ports/ xenial-updates main
        deb http://ports.ubuntu.com/ubuntu-ports/ xenial main
        deb http://ports.ubuntu.com/ubuntu-ports/ xenial-security main
        deb http://ports.ubuntu.com/ubuntu-ports/ xenial-updates main
        deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial main multiverse restricted universe
        deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-security main multiverse restricted universe
        deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-updates main multiverse restricted universe
        deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-backports main multiverse restricted universe
        deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial main multiverse restricted universe
        deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-security main multiverse restricted universe
        deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-updates main multiverse restricted universe
        deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ xenial-backports main multiverse restricted universe
        deb-src http://archive.ubuntu.com/ubuntu xenial main restricted #Added by software-properties
        deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted
        deb-src http://mirrors.aliyun.com/ubuntu/ xenial main restricted multiverse universe #Added by software-properties
        deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted
        deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted multiverse universe #Added by software-properties
        deb http://mirrors.aliyun.com/ubuntu/ xenial universe
        deb http://mirrors.aliyun.com/ubuntu/ xenial-updates universe
        deb http://mirrors.aliyun.com/ubuntu/ xenial multiverse
        deb http://mirrors.aliyun.com/ubuntu/ xenial-updates multiverse
        deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse #Added by software-properties
        deb http://archive.canonical.com/ubuntu xenial partner
        deb-src http://archive.canonical.com/ubuntu xenial partner
        deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted
        deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted multiverse universe #Added by software-properties
        deb http://mirrors.aliyun.com/ubuntu/ xenial-security universe
        deb http://mirrors.aliyun.com/ubuntu/ xenial-security multiverse
        deb-src http://mirrors.ustc.edu.cn/ubuntu-ports/ xenial-updates main multiverse restricted universe 
更新系统源： 

```apt update
$ apt update
```



## 3.3 Atlas 200 DK 安装ros

ros的安装可以参照[ros官方教程](wiki.ros.org/kinetic/Installation/Ubuntu)



## 3.4 Atlas 200 DK  创建ros工作空间

ros环境和驱动已经搭建完毕，创建工作空间：

```
    $ mkdir -p home/catkin_ws/src
    $ cd  home/catkin_ws/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    $ source /opt/ros/kinetic/setup.bash
```



## 4. Atlas 200 DK 安装小车部件驱动程序
### 4.1 OLED屏驱动安装
先安装curl, curl是按封装好的http请求应用程序，依次输入以下命令：

```
$ apt-get install -y curl
$ curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
$ python get-pip.py
```
安装python的依赖包：

```
$ apt-get install -y python-dev
```

OLED显示屏型号 SSD1306，安装Adafruit-SSD1306驱动：

``` $ pip install Adafruit-SSD1306
$ pip install Adafruit-SSD1306
```



## 4.2 安装motor驱动

安装git，在终端输入命令：

~~~$ apt-get install git
 $ apt-get install git
~~~

创建源码保存路径，并下载源码 ：

```
 $ cd ~
 $ mkdir backup
 $ cd backup
```
~~~
 $ git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
 $ python setup.py build
 $ python setup.py install
 $ apt install python-opencv
 $ pip install ipywidgets
 $ sudo apt-get install libjpeg8-dev
 $ sudo apt-get install libpng12-dev
 $ sudo apt-get install libfreetype6-dev
 $ sudo apt-get install zlib1g-dev
 $ sudo apt-get install libwebp-dev
 $ sudo apt-get install libtiff5-dev
 $ sudo apt-get install libopenjpeg-dev
 $ sudo apt-get install libzip-dev
 $ pip install Pillow
 $ pip install traitlets
~~~



下载Ascbot0.3.0-py2.7.egg.tar.gzip，并解压，下载地址：  [Ascbot0.3.0-py2.7.egg](https://gitee.com/Atlas200DK/Ascbot/raw/master/Release/ascbot-0.3.0-py2.7.egg.tar.gzip) 

```
 $ git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
 $ tar zxvf Ascbot-0.3.0-py2.7.egg.tar.gzip
 $ easy_install Ascbot-0.3.0-py2.7.egg                  
```
### 4.2   设置小车使用的I2C总线
Atlas 200 DK的扩展有3组 I2C总线，选择的I2C-2，输入以下命令设置。

      $ cd /usr/local/lib/python2.7/dist-packages/ascbot
      $ vim robot.py
~~~
   找到行： i2c_bus = traitlets.Integer(default_value=1).tag(config=True)

   修改为： i2c_bus = traitlets.Integer(default_value=2).tag(config=True)
~~~

~~~
 $ vim init__.py
~~~

~~~
   找到行： from .object_detection import ObjectDetector 
   注释掉： #from .object_detection import ObjectDetector                
~~~

## 5  ros工作空间和节点的创建
### 5.1  	创建ros工作空间
ros环境和驱动已经搭建完毕，下面创建工作空间，在终端依次输入以下命令：

     $ mkdir -p home/catkin_ws/src
     $ cd  home/catkin_ws/src
     $ catkin_init_workspace
     $ cd ..
     $ catkin_make
     $ source devel/setup.bash
     $ source /opt/ros/kinetic/setup.bash

## 6 手机APP开发环境搭建

具体请参考[Google Android应用开发指南](https://developer.android.google.cn)











