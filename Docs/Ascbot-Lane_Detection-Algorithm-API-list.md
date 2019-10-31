
# 车道线检测算法API清单
+ 接口简介
+ 算法工作流程
+ 接口说明
+ 数据类型
+ 编译方法
+ 示例

## 接口简介
车道线检测算法SDK是对车道线检测算法的进行了封装，使用Ascend开发套件DDK（Device Development Kit），提供了简单的API接口，方便用户调用、并快速集成开发基于车道线检测的算法的应用。

## 算法工作流程
![sdk work flow](./AscbotImg/Ascbot-sdk-workflow-01.png)
+ 目前支持的输入图像格式是YUV420。
+ 预处理中包含图像缩放、格式转换、归一化等。
+ 后处理包含算法推理结果的变换等。
+ SDK使用方法（调用流程）请参考示例代码。

## 接口说明
车道线检测算法
### API列表
函数|参数|返回值|说明
--|:--:|:--:|:--:
HiRoadFollowing|无|无|构造函数
~HiRoadFollowing|无|无|析构函数
init|std::string& config_path|bool|初始化算法
deInit|无|bool|反初始化是否成功
predict|HIImgData& frame|bool|车道线检测函数，输入图像数据是YUV420格式
isInit|无|bool|是否初始化成功

### 接口详细说明
---
+ 函数原型
   + bool init(const std::string& config_path);
+ 函数说明
   + 初始化算法，创建算法上下文环境。
+ 参数说明
   + std::string& config_path : 指定配置文件road_following_graph.config的路径。
+ 返回值说明
   + 返回值如果是true则成功，否则失败。

---
+ 函数原型
   + bool deInit();
+ 函数说明
   + 反初始化算法,销毁算法上下文环境。
+ 参数说明
   + 无
+ 返回值说明
   + 返回值如果是true则成功，否则失败。

---
+ 函数原型
   + void setCallbackFunction(rf_callback_t func);
+ 函数说明
   + 指定回调函数，算法推理结果会通过回调函数通知调用者。
+ 参数说明
   + func : 回调函数的实现,如：
```
int on_result_update(const std::vector<hi::RFData>& results) {
    // 获取算法结果
    ......
    return 0;
}
```
+ 返回值说明
   + 无

---
+ 函数原型
   + bool isInit() const;
+ 函数说明
   + 返回ture或false,说明算法是否已经初始化成功。
+ 参数说明
   + 无
+ 返回值说明
   + 返回值如果是true则成功，否则失败。

## 数据类型
---
Hi::HIImgData

+ 功能说明
   + HIImgData是对输入数据的封装，包含图像的宽、高、类型、数据等信息。
+ 成员介绍
+ 定义原型

---
Hi::RFData

+ 功能说明
   + 算法返回数据结构定义，包含算法检测到的车道线位置，角度等信息。
+ 成员介绍
   + x : 检测到车道线x点坐标
   + y: 预测到车道线y点坐标
   + angle: 预测到的车道线上的点与当前位置的夹角

**定义原型**
```
struct RFData {
    float x = 0.0f; // The x coordinate of point of lane line predicted by the algorithm
    float y = 0.0f; // The y coordinate of point of lane line predicted by the algorithm
    float angle = 0.0f;  // Travel angle of car predicted by algorith, range of values [-90,90]
    int64_t timestamp = 0;  // The input image timestamp.
    // debug info
    float raw_x = 0.0f;
    float raw_y = 0.0f;
};
```

## 编译方法
```
./build.sh
```
编译前需要将build.sh和Makefile文件中的LD_LIBRARY_PATH和DDK_HOME路径修改为编译机器对应的DDK安装目录。
DDK安装方法可以参考[DDK 部署](https://ascend.huawei.com/doc/Atlas200DK/1.3.0.0/zh/zh-cn_topic_0195268768.html)。

## 示例
```
#include <unistd.h>
#include <sys/time.h>
#define __USE_XOPEN
#include <time.h>

#include "hiaiengine/ai_memory.h"

#include "HiRoadFollowing.h"
#include "file_utils.h"
#include "main.h"

int64_t getCurrentTime_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

int on_result_update(const std::vector<hi::RFData>& results) {
    std::cout << "on_result_update size:" << results.size() << std::endl;
    int64_t time = getCurrentTime_ms();
    for (int i = 0; i < results.size(); i++) {
        const hi::RFData& fr = results.at(i);
        std::cout << "x:" << fr.x << ",y:" << fr.y << ", angle:" << fr.angle << std::endl;
        int64_t diff = time - fr.timestamp;
        std::cout << "raw x:" << fr.raw_x << ", y:" << fr.raw_y << "timestamp:" << fr.timestamp
                  << ",time=" << diff << std::endl;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    std::cout << "main" << std::endl;
    std::cout << "arg0:" << argv[0] << std::endl;
    std::cout << "arg1:" << argv[1] << std::endl;
    std::cout << "arg2 w:" << std::atoi(argv[2]) << std::endl;
    std::cout << "arg3 height:" << std::atoi(argv[3]) << std::endl;
    std::cout << "arg3 size:" << std::atoi(argv[4]) << std::endl;

    hi::HiRoadFollowing roadDetector;
    roadDetector.init("/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/road_following_graph.config");
    roadDetector.setCallbackFunction(on_result_update);

    std::string file = std::string(argv[1]);
    uint32_t file_width = std::atoi(argv[2]);
    uint32_t file_height = std::atoi(argv[3]);
    uint32_t file_size = std::atoi(argv[4]);
    uint8_t* imageBufferPtr = NULL;
    // run on same side with dvpp need to make the mem align to 128(dvpp need)
    imageBufferPtr = reinterpret_cast<uint8_t*>(memalign(128, file_size));

    utils_GetImageBuffer(file.c_str(), imageBufferPtr, file_size, 0);
    int iter = 100;
    while (iter-- > 0 || true) {
        std::cout << "times: " << iter << std::endl;
        int64_t time = getCurrentTime_ms();
        hi::HIImgData image(file_width, file_height, TYPE_YUV_NV12, imageBufferPtr, file_width, time);
        roadDetector.predict(image);
        std::cout << "main predict finish" << std::endl;
        usleep(50000);
    }

    roadDetector.deInit();

    std::cout << "finish!!!" << std::endl;
    return 1;
}
```
