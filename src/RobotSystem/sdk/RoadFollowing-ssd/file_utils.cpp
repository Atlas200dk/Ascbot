#include "file_utils.h"

/**
define error code for HIAI_ENGINE_LOG
**/
#define USE_DEFINE_ERROR 0x6001

enum { HIAI_IDE_ERROR_CODE, HIAI_IDE_INFO_CODE, HIAI_IDE_WARNING_CODE };

HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_ERROR, HIAI_IDE_ERROR, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_INFO, HIAI_IDE_INFO, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_WARNING, HIAI_IDE_WARNING, "");

/**
* @brief: get the image buffer
* @[in]: path, the image path;
* @[in]: imageBufferPtr, the point of image buffer;
* @[in]: imageBufferLen, the buffer length;
* @[in]: frameId, the start of file offset
* @[return]: bool, if success return true, else return false
*/
bool utils_GetImageBuffer(const char* path, uint8_t* imageBufferPtr, uint32_t imageBufferLen,
                          uint32_t frameId) {
    bool ret = false;
    FILE* file = fopen64(path, "r");
    if (NULL == file) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[Mind_road_following_dataset] Error: open file %s failed",
                        path);
        return ret;
    }
    do {
        uint32_t imageFseek = ((uint32_t)frameId) * ((uint32_t)imageBufferLen);
        if (0 != fseeko64(file, imageFseek, SEEK_SET)) {
            HIAI_ENGINE_LOG(
                HIAI_IDE_ERROR,
                "[Mind_road_following_dataset] fseeko64 offset = %u failed in GetImageBuffer",
                frameId * imageBufferLen);
            break;
        }
        if (imageBufferLen != fread(imageBufferPtr, 1, imageBufferLen, file)) {
            HIAI_ENGINE_LOG(
                HIAI_IDE_ERROR,
                "[Mind_road_following_dataset] fread length = %u failed in GetImageBuffer",
                imageBufferLen);
            break;
        }
        ret = true;
    } while (0);

    fclose(file);
    return ret;
}
