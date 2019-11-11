/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#include <unistd.h>
#include <sys/time.h>
#define __USE_XOPEN
#include <time.h>

#include "hiaiengine/ai_memory.h"

#include "HiRoadObjectFollowing.h"
#include "file_utils.h"
#include "main.h"

static const char* g_label_str[] = {"cup",   "car",  "box",  "person",     "hand", "bottle",
                                    "phone", "book", "line", "left_round", "right_round", "edge",
                                    "light"};

int64_t getCurrentTime_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

#define FRAME_WIDTH     1280
#define FRAME_HEIGHT    720
/** Calculate object target angle, the range of return value is [-90,90] */
float cal_object_following_angle(const std::vector<hi::RoadObjectData>& results) {
    for (int i = 0; i < results.size(); i++) {
        const hi::RoadObjectData& object = results.at(i);
        // xxx is object lable id, such as box is "2", bottle is "5"
        if (object.lable == xxx && object.confidence > 0.5f) {
            float center_x = object.x + object.width / 2;
            float center_y = object.y + object.height / 2;
            double atanx = (center_x - FRAME_WIDTH / 2) / (FRAME_HEIGHT - center_y);
            float angle = atan(atanx) * 180 / M_PI;
            return angle;
        }
    }
}

int on_result_update(const std::vector<hi::RoadObjectData>& results) {
    std::cout << "on_result_update size:" << results.size() << std::endl;
    int64_t time = getCurrentTime_ms();
    for (int i = 0; i < results.size(); i++) {
        const hi::RoadObjectData& object = results.at(i);
        if (object.confidence > 0.5f) {
            if (object.lable < 11) std::cout << g_label_str[object.lable] << std::endl;
            int64_t diff = time - object.timestamp;
            std::cout << "object" << i << ": lable=" << object.lable << ", box[" << object.x << ","
                      << object.y << "," << object.width << "," << object.height
                      << "] conf=" << object.confidence << ", timestamp:" << object.timestamp
                      << ",time=" << diff << std::endl;
        }
    }

    // get object's angle
    float angle = cal_object_following_angle(results);
    return 0;
}

int main(int argc, char* argv[]) {
    std::cout << "main" << std::endl;
    hi::HiRoadObjectFollowing objectDetector;
    objectDetector.init(
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
        "road_object_following_graph.config");
    objectDetector.setCallbackFunction(on_result_update);

    std::string file =
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
        "1920_1280_bin.bin";
    uint32_t file_width = 1920;
    uint32_t file_height = 1080;
    uint32_t file_size = 3133440;
    uint8_t* imageBufferPtr = NULL;
    // run on same side with dvpp need to make the mem align to 128(dvpp need)
    imageBufferPtr = reinterpret_cast<uint8_t*>(memalign(128, file_size));

    utils_GetImageBuffer(file.c_str(), imageBufferPtr, file_size, 0);
    int iter = 50000;
    while (iter-- > 0 || true) {
        std::cout << "times: " << iter << std::endl;
        int64_t time = getCurrentTime_ms();
        hi::HIImgData image(file_width, file_height, TYPE_YUV_NV12, imageBufferPtr, file_width,
            time);
        objectDetector.predict(image);
        std::cout << "main predict finish" << std::endl;
        usleep(30000);
    }

    objectDetector.deInit();

    std::cout << "finish!!!" << std::endl;
    return 1;
}
