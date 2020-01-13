# Ascbot智能小车开发环境搭建指南	



Ascbot智能小车的开发包括算法、系统应用和训练服务器环境等j搭建方法说明。



# 1. Atlas 200 DK开发环境搭建
MindStudio及DDK的安装具体请参考[华为Atlas官网](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/zh-cn_topic_0178961797.html)网站



# 2. 算法训练服务器环境搭建

算法训练的环境搭建请参考[Caffe安装](https://github.com/holzers/caffe-ssd/blob/master/docs/installation.md) 



# 3. Atlas 200 DK的ROS环境搭建

如果用Ascbot自带的系统sd卡，ros环境已经具备，请跳过ros环境搭建步骤。

如果是按照文档第二项【依照Ascbot智能小车系统制作指南创建系统SD卡】制的卡，则跳过ros环境搭建。

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
添加安装源，打开 /etc/apt/sources.list，添加以下国内源到e文件中，可以根据实际速度测试结果设置比较快的源，并保存。

        ##中科大源
        deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
        deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
        deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
        deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
        deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
        deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
        deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
        deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
        deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
        deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
        
        ##阿里云的源
        deb http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
        deb http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
        deb http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
        deb http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
        deb http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
        deb-src http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
                
        ##163源
        deb http://mirrors.163.com/ubuntu/ bionic main restricted universe multiverse
        deb http://mirrors.163.com/ubuntu/ bionic-security main restricted universe multiverse
        deb http://mirrors.163.com/ubuntu/ bionic-updates main restricted universe multiverse
        deb http://mirrors.163.com/ubuntu/ bionic-proposed main restricted universe multiverse
        deb http://mirrors.163.com/ubuntu/ bionic-backports main restricted universe multiverse
        deb-src http://mirrors.163.com/ubuntu/ bionic main restricted universe multiverse
        deb-src http://mirrors.163.com/ubuntu/ bionic-security main restricted universe multiverse
        deb-src http://mirrors.163.com/ubuntu/ bionic-updates main restricted universe multiverse
        deb-src http://mirrors.163.com/ubuntu/ bionic-proposed main restricted universe multiverse
        deb-src http://mirrors.163.com/ubuntu/ bionic-backports main restricted universe multiverse

更新系统源： 

       apt update

## 3.3 Atlas 200 DK 安装ros
将SD卡插入Atlas 200 DK，启动并按照华为教程将Atlas 200 DK接入internet,指的是能够链接外网。
首先进入开发板root用户下 

        $ su -
 在root用户下，安装ROS系统，开发用的版本是ros1 kentic的版本，如果有安装失败，命令行会有对应的错误输出；ROS的安装可以参照[ROS官方教程](http://wiki.ros.org/kinetic/Installation/Ubuntu),[中文安装教程](https://www.ncnynl.com/archives/201801/2273.html)。
       同时按照官方的教程，在开发板中创建ros的工作空间；也可按照第3.4节查看创建工作空间。
        
## 3.4 Atlas 200 DK  创建ros工作空间
此部分请移步Ascbot-system-install-guide.md文档查看。


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

    $ pip install Adafruit-SSD1306

## 4.2 安装motor驱动

安装git，在终端输入命令：

~~~
$ apt-get install git
~~~
创建源码保存路径，并下载源码 ：

```
 $ cd ~
 $ mkdir backup
 $ cd backup
 $ git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
 $ python setup.py build
 $ python setup.py install
 $ sudo apt install python-opencv libjpeg8-dev libpng12-dev libfreetype6-dev zlib1g-dev libwebp-dev libtiff5-dev  libopenjpeg-dev libzip-dev
 $ pip install Pillow ipywidgets traitlets
```
  
下载ascbot0.3.0-py2.7.egg.tar，并解压，下载地址：  [Ascbot0.3.0-py2.7.egg](https://gitee.com/Atlas200DK/Ascbot/raw/master/Release/ascbot-0.3.0-py2.7.egg.tar) 
```
 $ tar -xvf ascbot-0.3.0-py2.7.egg.tar
 $ easy_install ascbot-0.3.0-py2.7.egg                  
```
## 4.3   设置小车使用的I2C总线
Atlas 200 DK的扩展有3组 I2C总线，选择的I2C-2，输入以下命令设置。

      $ cd /usr/local/lib/python2.7/dist-packages/ascbot
      $ vim robot.py

   找到行： i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
   修改为： i2c_bus = traitlets.Integer(default_value=2).tag(config=True)
 ~~~
     $ vim __init__.py
 ~~~
 找到行： from .object_detection import ObjectDetector 
 注释掉： #from .object_detection import ObjectDetector                
 注意：若没发现此行，可忽略。

# 5  手机APP开发环境搭建

具体请参考[Google Android应用开发指南](https://developer.android.google.cn)











