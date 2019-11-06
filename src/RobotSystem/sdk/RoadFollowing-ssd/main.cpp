/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#include <unistd.h>

#include "hiaiengine/ai_memory.h"

#include "main.h"
#include "HiRoadFollowing.h"
#include "file_utils.h"

static const char* g_label_str[] = {"person", "cup", "bottle", "box", "phone"};

int on_result_update(const std::vector<hi::RFData>& results) {
    std::cout << "on_result_update size:" << results.size() << std::endl;
    for (int i = 0; i < results.size(); i++) {
        const hi::RFData& fr = results.at(i);
        std::cout << "x:" << fr.x << ",y:" << fr.y << ", angle:" << fr.angle << std::endl;
        std::cout << "raw x:" << fr.raw_x << ", y:" << fr.raw_y << std::endl;
        std::cout << "timestamp:" << fr.timestamp << std::endl;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    std::cout << "main" << std::endl;
    hi::HiRoadFollowing roadDetector;
    roadDetector.init(
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/road_following_graph.config");
    roadDetector.setCallbackFunction(on_result_update);

    std::string file =
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
        "nature_0obj_anticw_left_0_008_nv12.bin";
    uint32_t file_width = 1280;
    uint32_t file_height = 720;
    uint32_t file_size = 1382400;
    uint8_t* imageBufferPtr = NULL;
    // run on same side with dvpp need to make the mem align to 128(dvpp need)
    imageBufferPtr = reinterpret_cast<uint8_t*>(memalign(128, file_size));

    utils_GetImageBuffer(file.c_str(), imageBufferPtr, file_size, 0);
    hi::HIImgData image(file_width, file_height, TYPE_YUV_NV12, imageBufferPtr, file_width, 7877866543);
    std::cout << "main input timestamp:" << image.timestamp() << std::endl;
    int iter = 100;
    while (iter-- > 0) {
        std::cout << "times: " << iter << std::endl;
        roadDetector.predict(image);
        std::cout << "main predict finish" << std::endl;
        usleep(30000);
    }

    roadDetector.deInit();

    std::cout << "finish!!!" << std::endl;
    return 1;
}
