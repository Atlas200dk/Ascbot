
# 1. 算法介绍

## 1.1 算法概述
Thsmartbot智能小车防碰撞与防跌落场景中，应用了二分类算法和物体识别算法，其中二分类算法的类别为安全、危险，用来区分前方是否为桌子边缘，即是否有跌落的危险；物体识别算法能够识别的物体为模型小人、模型木箱、手机、水瓶、手、书本、小车七种物体，算法能够识别出视野中出现的这几种物体的类别、置信度和位置信息。
## 1.2 算法应用场景
防碰撞与防跌落场景介绍：当小车前方距离桌子边缘小于13cm时（下称防跌落危险），自动向右转向，直至小车车头方向距离桌子边缘大于13cm（下称防跌落安全）继续直行，走到另一个防跌落危险处，自动右转到安全方向直行，以此类推；当小车防跌落安全，但是前方有障碍物时，判断障碍物的位置和与小车的距离，智能选择不同的转向（障碍物在左边，选择右转；障碍物在右边，选择左转），而既出现防跌落危险情况，又出现障碍物，则优先判定防跌落危险，保证小车不会跌落。
# 2. 数据采集
数据的质量直接影响了算法训练的结果，采集方案的指定和采集手法则直接影响的数据质量。
## 2.1 防跌落数据采集
防跌落数据分为“危险”和“安全”数据，其中小车前方距离桌子边缘小于13cm时，判定为“危险”数据；反之则为“安全”数据。采集时，为了方便横向推动小车，可用透明胶带或其他表面光滑的物体附着于小车轮子上，减少车轮滑动摩擦阻力。
采集方式为，执行小车中的脚本文件录制整段“危险”或“安全”视频，再用ffmpeg软件将视频切分为图片进行标注, 脚本文件路径为：/home/HwHiAiUser/Data_collection_Huawei/ascend_workspace/ascendcamera/out/ascendcamera.sh 
执行示例为：./ascendcamera -v -c 0 --fps 20 -t 20 -w 1280 -h 720 -o nature_danger_edge1_30_20s.h264
（参数--fps 20为录制视频的fps，参数-t 20为录制视频时长，参数-w 1280为录制视频的宽，参数-h 1280为录制视频的高，参数-o nature_danger_edge1_30_20s.h264为输出视频的名字，默认输出在脚本同一目录下）

为了方便之后标注，可将视频的名字与“危险”或“安全”的类别进行关联，并且尽量表示出更多的拍摄信息，本教程中的命名规则如下，（如果不用下述命名规则，则需要对标记程序进行自适应的修改。）比如想表达如下信息：自然光下_危险视频_第一个桌面边缘_小车车头与桌面呈30度角_拍摄了20s，对应的文件名可以为：nature_danger_edge1_30_20s.h264
注意，为了在防跌落算法不受障碍物的影响，需要在拍摄视频时，随机添加几种障碍物在小车视野范围内。
## 2.2 防碰撞数据采集
防碰撞数据需要采集所有希望防碰撞的障碍物，将障碍物放置在桌子上、地面上等其他位置进行多角度的拍摄。建议将采用两种拍摄手法：
- 一：将障碍物放在桌面或地面上，小车车头正对障碍物，以障碍物为圆心，13-5cm为半径，令小车做圆周运动，环绕拍摄障碍物的各个角度。
- 二：将障碍物放在桌面或地面上，首先让小车车头正对障碍物，然后令车头向左右各摆动大概60度，拍摄障碍物出现在镜头不同位置的数据。
注意将多种障碍物组合进行拍摄，当两个障碍物相互重叠时，标记在前面的障碍物。
## 2.3 脚本示例
通过上述1,2两步可以得到大量的视频数据，通过视频转图片程序(h264_to_jpg.py)将视频数据转换成图片数据，还可以通过数据增强程序(data_enhance.py)对图片进行随机饱和度、随机亮度、随机对比度、随机锐度、随机左右反转、模糊化来对数据进行增强。  
两个程序的用法示例如下：  
&emsp;h264_to_jpg.py:  
&emsp;&emsp;功能：将video_folder文件夹中的每个视频按照fps为4进行抽帧，输出图片路径为视频同一路径，输出图片名称为：视频名称_递加数字.jpg。  
&emsp;&emsp;示例：python3 h264_to_jpg.py ./video_folder 4  

&emsp;data_enhance.py:  
&emsp;&emsp;功能：将data_folder文件夹中的数据进行增强，输出图片路径为（视频路径_增强方式）文件夹，输出图片名称为：原图片名称_增强方式_递加数字.jpg。  
&emsp;&emsp;示例：python2 data_enhance.py ./data_folder  
# 3 数据标注
防跌落功能的实现依靠Resnet18分类算法，防碰撞功能的实现依靠了VGG16-SSD目标识别算法，故防跌落数据要标注分类信息（“危险”类别和“安全”类别），防碰撞数据要标注图片中希望识别物体的位置信息（用一个矩形框框柱物体，得到物体的坐标）。

## 3.1 防跌落数据标注
防跌落功能的实现依靠Resnet18分类算法，故防跌落数据要标注为分类信息（“危险”类别和“安全”类别）即可。防跌落数据采集的命名规则已经与标注类别进行了关联，现在只需要将抽帧出来的图像数据集按照名字进行标注即可，下面提供的标注程序(split_list_label_2class.py)不仅可以进行标注，还可以直接将完整的数据集分割为训练集文件夹与测试集文件夹，并且生成训练集数据列表和测试集数据列表。
## 3.2 防碰撞数据标注
防碰撞数据采集了大量希望识别物体的数据，通过网络开源的标注工具(比如Labelimg)对数据进行标注（标注的原则是，物体框要尽量的与物体边缘相切），得到具有标注信息的json格式文件。
## 3.3 脚本示例
&emsp;split_list_label_2class.py：  

&emsp;&emsp;功能：将dataset_folder总数据集进行标注，按照9:1分割为训练集与测试集，并且生成对应的文件列表和标记列表  
&emsp;&emsp;示例：python2 split_list_label_2class.py ./dataset_folder 如果希望更改训练集与测试集的比重，请自行更改程序中的注释部分(注释内容为：# change train/test at here (xxx,train proportion, test proportion)
# 4 数据格式转换
合适的数据格式，可以大幅度提升训练速度，也对训练结果有一些提升。
## 4.1 防跌落数据格式转换
现在已经有了训练集数据、训练集标记列表、测试集数据、测试集标记列表，可以直接通过caffe自带的工具转换为lmdb格式数据，也可以使用下面提供的脚本，要注意将脚本中caffe的路径更改为自己的caffe路径。
## 4.2 防碰撞数据格式转换
主要包含以下几部分：
- labelmap_voc.prototxt
- Annotations文件夹
- JPEGImages文件夹

labelmap_voc.prototxt：
用于描述类别信息，如果类别数目是4，则label从1开始，（0默认作为背景类别）如下所示：
```c++
item {
label: 1
name: 'cup'
display_name: "cup"
}

item {
label: 2
name: 'car'
display_name: "car"
}

item {
label: 3
name: 'box'
display_name: "box"
}

item {
label: 4
name: 'person'
display_name: "person"
}
```
JPEGImages文件夹：
主要提供的是PASCAL VOC所提供的所有的图片信息，包括训练图片，测试图片，这些图像就是用来进行训练和测试验证的图像数据。
Annotations文件夹：
主要存放xml格式的标签文件，每个xml对应JPEGImage中的一张图片，示例如下：
```xml
<annotation>  
   	 <folder>VOC2012</folder>                             
   	 <filename>2007000392.jpg</filename>      //文件名  
    <source>                                  //图像来源（不重要）  
        <database>The VOC2007 Database</database>  
        <annotation>PASCAL VOC2007</annotation>  
        <image>flickr</image>  
    </source>  
    <size>                                    //图像尺寸（长宽以及通道数）                        
        <width>500</width>  
        <height>332</height>  
        <depth>3</depth>  
    </size>  
    <segmented>1</segmented>        //是否用于分割（在图像物体识别中01无所谓）  
    <object>                        //检测到的物体  
        <name>car</name>            //物体类别   
        <difficult>0</difficult>    //目标是否难以识别（0表示容易识别）  
        <bndbox>                //bounding-box（包含左下角和右上角xy坐标）  
            <xmin>100</xmin>  
            <ymin>96</ymin>  
            <xmax>355</xmax>  
            <ymax>324</ymax>  
        </bndbox>  
    </object>  
    <object>             	       //检测到多个物体  
        <name>person</name> 
        <difficult>0</difficult>  
        <bndbox>  
            <xmin>198</xmin>  
            <ymin>58</ymin>  
            <xmax>286</xmax>  
            <ymax>197</ymax>  
        </bndbox>  
    </object>  
</annotation>  
```
最后，create_annoset.py文件用于将PASCAL VOC数据集转换为lmdb数据格式。
## 4.3 脚本示例
&emsp;generate_lmdb_meanfile.sh  
&emsp;&emsp;功能：用训练/测试集文件夹生成lmdb格式数据集，注意，不要更改3.3脚本示例split_list_label_2class.py生成的训练/测试数据列表内容和路径，此脚本内部解析使用到了该文件。  
&emsp;&emsp;示例：./generate_lmdb_meanfile.sh ./dataset_train/ ./dataset_test/  

&emsp;create_annoset.py  
&emsp;&emsp;功能：主要用于生成pascal voc数据集，用于将原始的sql生成结果和对应的图像数据生成标准的pascal voc格式    
&emsp;&emsp;示例：python create_annoset.py    
# 5 算法训练
算法网络的增删会影响最终的训练结果，而训练超参数的设定则可以帮助快速让训练达到理想效果。
## 5.1 防跌落算法训练
如上所述，防跌落算法为Resnet18分类算法，首先在github下载好caffe Resnet18的训练文件，修改如下：

对于Resnet18_class_train.prototxt（示例文件已提供）
将Data层的mean_file字段后修改为上述生成的meanfile文件路径，data_source语句块下source字段后修改为上述生成的lmdb文件路径，注意区别TRAIN和TEST。
将InnerProduct层的num_output字段后数字修改为2（2分类即2个输出）。
为了防止过拟合，在最后两个卷积层后增加dropout层，dropout_ratio为0.1即可。

对于Resnet18_class_solver.prototxt（示例文件已提供）
```c++
net: "./Resnet18_class_train.prototxt"
test_iter: 75
test_interval: 640
base_lr: 0.0001
momentum: 0.9
weight_decay: 0.0002
lr_policy: "fixed"
display: 200
max_iter: 20000
snapshot: 5000
snapshot_prefix: "./collision_avoidance"
type: "Adam"
solver_mode: GPU
```
其中，test_iter >= 测试集数据数量 / 测试batch size；test_interval >= 训练集数据数量 / 训练batch size。
## 5.2 防碰撞算法训练
防碰撞算法采用以缩减版的vgg16 net作为前向网络结合ssd net作为完整的目标识别训练神经网	络。在基本不降低准确率的前提下，为了提高识别效率，本训练网络的输入采用220x220尺寸，	相对于300x300的输入，识别速度提高2倍。训练在caffe平台上。训练的类别一共14类（包含	背景类，背景类对应的label默认为0）。
训练用的sover.prototxt内容为：
```c++
train_net: "/xxx /models/VGGNet/VOC0712/SSD_220x220/train.prototxt"
test_net: "/xxx/models/VGGNet/VOC0712/SSD_220x220/test.prototxt"
test_iter: 11
test_interval: 4000
base_lr: 0.00249999994412
display: 10
max_iter: 200000
lr_policy: "multistep"
gamma: 0.10000000149
Momentum: 0.899999976158
weight_decay: 0.000500000023749
snapshot: 4000
snapshot_prefix: "/xxx/models/VGGNet/VOC0712/SSD_220x220/VGG_VOC0712_SSD_220x220"
solver_mode: GPU 
debug_info: false
snapshot_after_train: true
test_initialization: false
average_loss: 10
stepvalue: 4000
stepvalue: 10000
stepvalue: 20000
stepvalue: 40000
stepvalue: 80000
iter_size: 4
type: "SGD"
eval_type: "detection"
ap_version: "11point"
```
# 6 算法模型转换
保存上一节算法训练得出的caffemodel文件和deploy.protortxt文件，其中deploy.prototxt文件已经给出。然后将上述两文件导入Mindstudio软件中得到davinci模型，点击右侧My Models后面的加号显示下图，其中Model File和Weight File分别选择deploy.prototxt文件和caffemodel文件  
![AscbotFallingCollisionAvoidense1](./Ascbotimg/AscbotFallingCollisionAvoidense1.png)
<center>AscbotFallingCollisionAvoidense1</center>
如果训练caffemodel文件时减去了均值，可以点击Optional Options选项填写要减去的均值，最后点击ok按钮，即可生成davinci模型文件。
![AscbotFallingCollisionAvoidense2](./AscbotFallingCollisionAvoidense2.png)
<center>AscbotFallingCollisionAvoidense2</center>
将工程在板子上跑过以后，到对应路径下获得davinci模型文件，文件后缀为.om  

# 7 算法部署及应用
将上述转换好的davinci模型文件复制到Thmartcar如下路径：
/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspaces/cameraservice

注意把对应配置文件中的model_path路径后的文件名改为想替换的om文件名，例如把collision_avoidance_model.om文件放到了如上路径，则在collision_avoidance_graph.config文件中搜索model_path，将同一items中的value的值修改如下：/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspaces/cameraservice/collision_avoidance_model.om
