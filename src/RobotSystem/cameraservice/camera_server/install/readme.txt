
//install cross-compile tool and DDK&&Mindsport

sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

//编译 camera_server
$cd camera_server
$make

//编译 rtsp_server
$ cd live_server/live
$ ./genMakefiles HwHiAi
$ make
$ cd ..
$ make

//copy camera_server&& rtsp_server to install folder

$ cp camera_server/out/camera_server  install
$ cp live_server/rtsp-server  install
$ chmod 777 install/*


//atlas310 开发板中创建运行目录
$ mkdir -p /home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice

//在pc端将运行所需文件拷贝到运行目录

$ scp -r install/* HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice

//atlas310
1.设置环境变量
export LD_LIBRARY_PATH=/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice:$LD_LIBRARY_PATH

2.运行rtsp-server

3.运行camera-server






