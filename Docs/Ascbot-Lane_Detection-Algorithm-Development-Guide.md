#       Ascbot智能小车车道线检测算法开发指南



# 概述

车道线检测算法的目的，是让小车自动循道行驶，它可以自动检测道路中间的虚线，并沿着直道和弯道都可以准确无误地行驶。

**阅读此文档前需要具备和学习的知识：**

1. Linux 的基本操作
2. 简单的 Python 编码
3. 了解深度学习的基础知识和 Caffe 框架
4. 华为 MindStudio 的使用

**本文章节对应的操作平台**

| 一级步骤           | 执行位置        | 备注 |
| ------------------ | --------------- | ---- |
| 数据采集           | 小车ascbot      |      |
| 算法训练和模型选择 | PC（ubuntu OS） |      |
| 算法训练和模型选择 | 算法训练服务器  |      |
| 模型转换           | PC（ubuntu OS） |      |
| 模型部署           | 小车ascbot      |      |


# 1 数据采集

## 1.1 数据采集环境准备

1. 启动小车，从远程ssh登陆小车系统

2. 下载在小车上拍摄视频的程序 ascendcamera（[下载地址](https://gitee.com/Atlas200DK/Ascbot/raw/master/Release/AscendCameraV1.zip)

3. 准备车道线采集环境插入车道图片

## 1.2 设计数据采集场景

数据采集需要考虑两个关键要素

+ 数据的多样性

+ 图像的数量。

关于数据的多样性，是指数据采集的过程中需要充分考虑各种的变量。我们的实验表明，下列变量是比较重要的因素：

- 光线
- 小车位于车道哪个位置（偏左、偏右、居中）
- 小车前进方向和小车与车道角度（直行、S形行驶）
- 视野内有无障碍物
- 车道周边的环境

关于数据的数量，在保证数据多样性的前提下，完整车道均匀采集500张就可以看到效果，大于2000张是达到相对理想效果。

## 1.2 采集方法

### Step 1 将小车放置到车道上，远程SSH登录小车系统，手动推动小车按照直行和S形在车道上行进并同时录制视频。

```
ssh HwHiAiUser@192.168.3.9
./ascendcamera -v -c 0 --fps 20 -t 60 -w 1280 -h 720 -0 video_name.h264
```

其中，`--fps`是每秒钟帧数（1-20），代表采样频率；`-t`是采样时长；`-w`和`-h`是采样分辨率。

### Step 2 将视频拷贝到PC(ubuntu OS)，使用ffmpeg 抽取视频帧。

```
ffmpeg -i video.h264 -vf fps=2 -q 0 ../video_name_frame%04d.jpg
```

其中`fps=2`代表每一秒钟的视频片段中，抽取两帧图像。更改这个数字可以得到不同数量的图像。

# 2 数据标注

## 2.1 部署数据标注工具 ATool

下载 ATools [下载地址](https://gitee.com/Atlas200DK/Ascbot/raw/master/Release/ATool.tar.gz)，编译并安装：

```
cd ATool
mkdir build
cmake ..
make
```

## 2.2 开始标注

将抽取到的图像放到 `ATool/img` 文件夹下，启动标注工具，`./build/ATool` 。

鼠标所在的位置有一个移动的点，从屏幕下方的中间（代表小车摄像机的位置）到鼠标之间有一条辅助线，代表小车的前进方向。屏幕中间有一条水平的辅助线，标注方法如下：

1. 在辅助线上下，选择一个道路中间的白点，点击鼠标
2. 按 N 切换下一张图像
3. 等所有图像标记完成后，软件自动退出，标记结果位于 `ATool/label`

如果有标记错误的点，只需要在正确的点重新点击一次，错误的点即可自动删除。

# 3 算法训练和模型选择

## 3.1 数据集划分

将数据集划分为训练集和验证集，将图片放置于各自独立的文件夹，并分别生成文件列表。

```
cd code/road_following
python 1_generate_list.py
```

## 3.2 数据增强

数据增强是将训练数据做一些颜色、亮度、几何形状等方面的微调，目的是增加模型的泛化能力。

```
cd code/tool/data_augmentation
python da_road_following.py
```

注意：请在`1. 数据集划分`执行完毕后，再执行`2. 数据增强`。因为数据增强应该在训练集，而不是验证集上执行。

## 3.3 转换 HDF5 文件

Caffe 中使用 LMDB 作为默认的二进制文件格式，但 LMDB 格式只支持单标签的标注。而在我们的任务中，类别标签有两个数字，分别为预测点的 x、y 坐标。所以需要用到 HDF5，一种支持多标签的二进制文件格式，来存储图片和对应的坐标。

```
cd code/road_following
python 2_convert_hdf5.py
```

## 3.4 设置训练超参数

训练所需的超参数包括：

- 优化器
- 学习率
- 学习率衰减策略
- 训练轮数 Epoch
- 训练步数 Iteration
- 训练多久保存一次模型快照

`code/road_following/resnet-18/ResNet-18-Caffemodel-on-ImageNet/solver_adam_poly.prototxt` 中已经配置好这些参数，你可以使用这些超参数直接开始训练。当然，你也可以修改后再训练。


## 3.5 开始训练

```
cd code/road_following/ResNet-18-Caffemodel-on-ImageNet
sh ./train.sh
```

训练结束后，我们会得到一组以 caffemodel 为后缀的文件，保存路径在 `solver_adam_poly.prototxt` 中指定。

## 3.6  模型选择和验证

在训练过程中，会生成多个模型权重文件。在所有保存的模型中，根据训练过程中打印的log，选择 validation loss 最小的模型作为最优模型。

在已经划分好的验证集上，可以加载模型并执行推理过程，并将预测结果可视化。

```
cd code/validation
python regression_pycaffe.py
```

执行上面这段代码，将会在屏幕上显示验证集中，每张图片的预测点和标记点，并打印出最终的损失函数值。通常情况下，我们选择损失函数值最小的最优模型。

# 4. 模型转换

关键的步骤如下：

1. 创建 Mind Studio 工程
    - 1.1 模型编排
    - 1.2 导入训练完成的 caffemodel 权重文件和 prototxt 网络定义文件
    - 1.3 模型转换、自定义数据集的制作
    - 1.4 修改后处理中的输出层
2. 验证模型的运行结果是否准确。
3. 导出 om 文件

模型转换的具体过程，请参考：[Ascend 开发文档](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/zh-cn_topic_0194969548.html)

与开发文档中唯一不同的一点是，由于回归模型的目的不是预测概率，而是预测坐标，所以模型的输出层不是 prob 而是 fc2 ，请在 Mind Studio 中将输出层的名字从默认的 prob 修改为 fc2 。

# 5. 模型部署

部署步骤：

1. 替换 road_following_graph.config 文件中指定的模型文件，如：/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/road_following_model.om

2. 编译 HiRoadFollowing SDK 并运行测试算法

3. 实际部署时模型存放的路径可修改 graph.config 文件
