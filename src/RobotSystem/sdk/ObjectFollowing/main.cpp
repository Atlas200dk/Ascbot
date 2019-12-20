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

#include "HiObjectFollowing.h"
#include "file_utils.h"
#include "main.h"

static const char* g_label_str[] = {"person", "cup", "bottle", "box", "phone"};

int on_result_update(const std::vector<hi::ObjectData>& results) {
  std::cout << "on_result_update size:" << results.size() << std::endl;
  for (int i = 0; i < results.size(); i++) {
    const hi::ObjectData& object = results.at(i);
    if (object.lable < 5) std::cout << g_label_str[object.lable] << std::endl;
    std::cout << "object" << i << ": lable=" << object.lable << ", box["
              << object.x << "," << object.y << "," << object.width << ","
              << object.height << "] conf=" << object.confidence << std::endl;
    std::cout << "timestamp:" << object.timestamp << std::endl;
  }
  return 0;
}

int main(int argc, char* argv[]) {
  std::cout << "main" << std::endl;
  hi::HiObjectFollowing objectDetector;
  objectDetector.init(
      "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
      "object_following_graph.config");
  objectDetector.setCallbackFunction(on_result_update);

  std::string file =
      "/home/HwHiAiUser/HIAI_PROJECTS/workspace_mind_studio/HRobot/"
      "dangerous-box-video4-015_nv12.bin";
  uint32_t file_width = 1280;
  uint32_t file_height = 720;
  uint32_t file_size = 1382400;
  uint8_t* imageBufferPtr = NULL;
  // run on same side with dvpp need to make the mem align to 128(dvpp need)
  imageBufferPtr = (reinterpret_cast<uint8_t*>(memalign(128, file_size)));

  utils_GetImageBuffer(file.c_str(), imageBufferPtr, file_size, 0);
  hi::HIImgData image(file_width, file_height, TYPE_YUV_NV12, imageBufferPtr,
                      file_width, 1222091);
  int iter = 5;
  while (iter-- > 0) {
    std::cout << "times: " << iter << std::endl;
    objectDetector.predict(image);
    std::cout << "main predict finish" << std::endl;
    usleep(30000);
  }

  objectDetector.deInit();

  std::cout << "finish!!!" << std::endl;
  return 1;
}
