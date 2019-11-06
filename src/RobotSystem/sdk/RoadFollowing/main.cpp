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
