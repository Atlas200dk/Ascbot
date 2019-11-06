#include <unistd.h>

#include "hiaiengine/ai_memory.h"

#include "HiCollisionAvoidance.h"
#include "file_utils.h"
#include "main.h"

int on_result_update(const std::vector<hi::CollisionData>& results) {
    std::cout << "on_result_update size:" << results.size() << std::endl;
    for (int i = 0; i < results.size(); i++) {
        const hi::CollisionData& coll = results.at(i);
        std::cout << "coll status:" << coll.status << std::endl;
        for (std::map<hi::CollisionStatus, float>::const_iterator iter = coll.status_map.begin();
             iter != coll.status_map.end(); iter++) {
            std::cout << iter->first << "->" << iter->second << std::endl;
        }
        std::cout << "timestamp:" << coll.timestamp << std::endl;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    std::cout << "main" << std::endl;
    hi::HiCollisionAvoidance roadDetector;
    roadDetector.init(
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
        "collision_avoidance_graph.config");
    roadDetector.setCallbackFunction(on_result_update);

    std::string file =
        "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
        "light_danger0_13dis_right4_60_0_22_dark_yuv.bin";
    uint32_t file_width = 1280;
    uint32_t file_height = 720;
    uint32_t file_size = 1382400;
    uint8_t* imageBufferPtr = NULL;
    // run on same side with dvpp need to make the mem align to 128(dvpp need)
    imageBufferPtr = reinterpret_cast<uint8_t*>(memalign(128, file_size));

    utils_GetImageBuffer(file.c_str(), imageBufferPtr, file_size, 0);
    hi::HIImgData image(file_width, file_height, TYPE_YUV_NV12, imageBufferPtr, file_width,
                        2231234);
    int iter = 5;
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
