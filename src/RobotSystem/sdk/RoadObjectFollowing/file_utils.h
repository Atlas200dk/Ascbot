#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fstream>
#include <memory>
#include <sstream>
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/log.h"

/**
* @brief: get the image buffer
* @[in]: path, the image path;
* @[in]: imageBufferPtr, the point of image buffer;
* @[in]: imageBufferLen, the buffer length;
* @[in]: frameId, the start of file offset
* @[return]: bool, if success return true, else return false
*/
bool utils_GetImageBuffer(const char* path, uint8_t* imageBufferPtr, uint32_t imageBufferLen,
                          uint32_t frameId);
