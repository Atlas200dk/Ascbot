/**
* @file ImagePreProcess_1.cpp
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-4-25
*/

#include "ImagePreProcess_1.h"
#include <math.h>
#include <sstream>
#include "custom/toolchain/ide_daemon_api.h"
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/ai_types.h"
#include "hiaiengine/c_graph.h"
#include "hiaiengine/log.h"

HIAI_REGISTER_DATA_TYPE("ScaleInfoT", ScaleInfoT);
HIAI_REGISTER_DATA_TYPE("NewImageParaT", NewImageParaT);
HIAI_REGISTER_DATA_TYPE("BatchImageParaWithScaleT", BatchImageParaWithScaleT);
HIAI_REGISTER_SERIALIZE_FUNC("EvbImageInfo", EvbImageInfo, GetEvbImageInfoSearPtr,
                             GetEvbImageInfoDearPtr);

static const int SEND_DATA_SLEEP_MS = 100000;
static const int DVPP_SUPPORT_MAX_WIDTH = 4096;
static const int DVPP_SUPPORT_MIN_WIDTH = 16;
static const int DVPP_SUPPORT_MAX_HEIGHT = 4096;
static const int DVPP_SUPPORT_MIN_HEIGHT = 16;

#define CHECK_ODD(NUM) (((NUM) % 2 != 0) ? (NUM) : ((NUM)-1))
#define CHECK_EVEN(NUM) (((NUM) % 2 == 0) ? (NUM) : ((NUM)-1))

RFImagePreProcess::~RFImagePreProcess() {
    if (NULL != m_pidvppapi_) {
        DestroyDvppApi(m_pidvppapi_);
        m_pidvppapi_ = NULL;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1]ImagePreProcess_1 Engine Destory");
}

HIAI_StatusT RFImagePreProcess::Init(const hiai::AIConfig& config,
                                     const std::vector<hiai::AIModelDescription>& model_desc) {
    if (NULL == m_dvpp_config_) {
        m_dvpp_config_ = std::make_shared<DvppConfig>();
        if (NULL == m_dvpp_config_ || NULL == m_dvpp_config_.get()) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[ImagePreProcess_1] call m_dvpp_config_ make_shared failed");
            return HIAI_ERROR;
        }
    }

    // get config from ImagePreProcess_1 Property setting of user.
    std::stringstream ss;
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        ss << item.value();
        if ("point_x" == name) {
            ss >> (*m_dvpp_config_).point_x;
        } else if ("point_y" == name) {
            ss >> (*m_dvpp_config_).point_y;
        } else if ("crop_width" == name) {
            ss >> (*m_dvpp_config_).crop_width;
        } else if ("crop_height" == name) {
            ss >> (*m_dvpp_config_).crop_height;
        } else if ("self_crop" == name) {
            ss >> (*m_dvpp_config_).self_crop;
        } else if ("dump_value" == name) {
            ss >> (*m_dvpp_config_).dump_value;
        } else if ("project_name" == name) {
            ss >> (*m_dvpp_config_).project_name;
        } else if ("resize_height" == name) {
            ss >> (*m_dvpp_config_).resize_height;
        } else if ("resize_width" == name) {
            ss >> (*m_dvpp_config_).resize_width;
        } else if ("crop_before_resize" == name) {
            ss >> (*m_dvpp_config_).crop_before_resize;
        } else if ("yuv420_need" == name) {
            ss >> (*m_dvpp_config_).yuv420_need;
        } else if ("v_before_u" == name) {
            ss >> (*m_dvpp_config_).v_before_u;
        } else if ("transform_flag" == name) {
            ss >> (*m_dvpp_config_).transform_flag;
        } else if ("dvpp_parapath" == name) {
            ss >> (*m_dvpp_config_).dvpp_para;
        } else {
            std::string userDefined = "";
            ss >> userDefined;
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "userDefined:name[%s], value[%s]", name.c_str(),
                            userDefined.c_str());
        }
        ss.clear();
    }

    // check crop param
    if (__need_crop()) {
        if (m_dvpp_config_->point_x > DVPP_SUPPORT_MAX_WIDTH ||
            m_dvpp_config_->crop_width > DVPP_SUPPORT_MAX_WIDTH ||
            m_dvpp_config_->crop_width < DVPP_SUPPORT_MIN_WIDTH ||
            m_dvpp_config_->point_y > DVPP_SUPPORT_MAX_HEIGHT ||
            m_dvpp_config_->crop_height > DVPP_SUPPORT_MAX_HEIGHT ||
            m_dvpp_config_->crop_height < DVPP_SUPPORT_MIN_HEIGHT) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "crop param error");
            return HIAI_ERROR;
        }
    }

    if (DVPP_SUCCESS != CreateDvppApi(m_pidvppapi_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]Create DVPP m_pidvppapi_ fail");
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

// jpeg pic process flow:
//  1. DVPP_CTL_JPEGD_PROC
//  2. DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM
//  3. DVPP_CTL_VPC_PROC
int RFImagePreProcess::__handle_jpeg(const ImageData<u_int8_t>& img) {
    if (NULL == m_pidvppapi_) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] m_pidvppapi_ is null!\n");
        return HIAI_ERROR;
    }

    struct JpegdIn jpegdInData;                               // input data
    jpegdInData.jpegData = (unsigned char*)(img.data.get());  // the pointer addr of jpeg pic data
    jpegdInData.jpegDataSize = img.size;                      // the size of jpeg pic data
    jpegdInData.isYUV420Need =
        false;  // (*m_dvpp_config_).yuv420_need;true:output yuv420 data, otherwize:raw format.
    jpegdInData.isVBeforeU = true;  // currently, only support V before U, reserved

    struct JpegdOut jpegdOutData;  // output data

    dvppapi_ctl_msg dvppApiCtlMsg;  // create inputdata and outputdata for jpegd process
    dvppApiCtlMsg.in = reinterpret_cast<void*>(&jpegdInData);
    dvppApiCtlMsg.in_size = sizeof(struct JpegdIn);
    dvppApiCtlMsg.out = reinterpret_cast<void*>(&jpegdOutData);
    dvppApiCtlMsg.out_size = sizeof(struct JpegdOut);

    if (0 != DvppCtl(m_pidvppapi_, DVPP_CTL_JPEGD_PROC,
                     &dvppApiCtlMsg)) {  // if this single jpeg pic is processed with error, return
                                         // directly, and then process next pic if there any.
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] JPEGDERROR, FrameID:%u",
                        m_imageFrameID);
        jpegdOutData.cbFree();  // release memory from caller.
        return HIAI_ERROR;
    }
// std::cout << "__handle_jpeg org w:" << img.width << ",h:" << img.height << ",size:" << img.size
// <<  std::endl;
// std::cout << "__handle_jpeg decode align w:" << jpegdOutData.imgWidthAligned << ",h:" <<
// jpegdOutData.imgHeightAligned << ",size:" << jpegdOutData.yuvDataSize << std::endl;
#if 0
    std::ofstream out_file_nv12("./0704-1140-data2-118_nv12.bin", std::ofstream::out);
    uint8_t* nv12_ptr = jpegdOutData.yuvData;
    for (int i = 0; i < jpegdOutData.imgHeightAligned; i++) {
        for (int j = 0; j < jpegdOutData.imgWidthAligned; j++) {
            out_file_nv12 << *nv12_ptr++;
        }
    }
    for (int i = 0; i < jpegdOutData.imgHeightAligned / 2; i++) {
        for (int j = 0; j < jpegdOutData.imgWidthAligned / 2; j++) {
            uint8_t v = *nv12_ptr++;
            uint8_t u = *nv12_ptr++;
            out_file_nv12 << u;
            out_file_nv12 << v;
        }
    }
    out_file_nv12.close();
#endif
    int ret = __handle_vpc_with_param(jpegdOutData.yuvData, jpegdOutData.imgWidthAligned,
                                      jpegdOutData.imgHeightAligned, jpegdOutData.yuvDataSize, img,
                                      FILE_TYPE_PIC_JPEG, jpegdOutData.outFormat, 0);
    jpegdOutData.cbFree();  // release memory from caller.
    if (HIAI_OK != ret) {   // if vpc process with error, return directly, and then process next pic
                            // if there any.
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]VPCERROR, FrameID:%u", m_imageFrameID);
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

int RFImagePreProcess::__update_crop_para(const Rectangle<Point2D>& rect) {
    (*m_dvpp_config_).point_x = static_cast<int>(rect.anchor_lt.x);
    (*m_dvpp_config_).point_y = static_cast<int>(rect.anchor_lt.y);
    (*m_dvpp_config_).crop_width = static_cast<int>(rect.anchor_rb.x - rect.anchor_lt.x);
    (*m_dvpp_config_).crop_height = static_cast<int>(rect.anchor_rb.y - rect.anchor_lt.y);

    HIAI_ENGINE_LOG("[ImagePreProcess_1]: %d, %d, %d, %d", (*m_dvpp_config_).point_x,
                    (*m_dvpp_config_).point_y, (*m_dvpp_config_).crop_width,
                    (*m_dvpp_config_).crop_height);

    if ((*m_dvpp_config_).point_x < 0 || (*m_dvpp_config_).point_x % 2 != 0 ||
        (*m_dvpp_config_).point_y < 0 || (*m_dvpp_config_).point_y % 2 != 0 ||
        (*m_dvpp_config_).crop_height <= 0 || (*m_dvpp_config_).crop_height % 2 != 0 ||
        (*m_dvpp_config_).crop_width <= 0 || (*m_dvpp_config_).crop_width % 2 != 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]Some crop parms are invalid!");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

bool RFImagePreProcess::__send_data() {
    // std::cout << "T:" << std::this_thread::get_id() << " " << "ImagePreProcess_1::__send_data" <<
    // std::endl;
    if (NULL == m_dvpp_out_) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]Nothing to send!");
        return false;
    }

    if (m_dvpp_out_->v_img.size() > 0) {
        m_dvpp_out_->b_info.batch_size = m_dvpp_out_->v_img.size();
        HIAI_StatusT hiai_ret = HIAI_OK;
        do {
            hiai_ret = SendData(0, "BatchImageParaWithScaleT",
                                std::static_pointer_cast<void>(m_dvpp_out_));
            if (HIAI_QUEUE_FULL == hiai_ret) {
                HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1] queue full, sleep 200ms");
                usleep(SEND_DATA_INTERVAL_MS);
            }
        } while (hiai_ret == HIAI_QUEUE_FULL);

        if (HIAI_OK != hiai_ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] SendData failed! error code: %d",
                            hiai_ret);
            return false;
        }
    }
    return true;
}

void RFImagePreProcess::__clear_data() {
    m_dvpp_in_ = NULL;
    m_dvpp_out_ = NULL;
}

// png pic process flow:
//  1. DVPP_CTL_PNGD_PROC
//  2. DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM
//  3. DVPP_CTL_VPC_PROC
int RFImagePreProcess::__handle_png(const ImageData<u_int8_t>& img) {
    if (NULL == m_pidvppapi_) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] m_pidvppapi_ is null");
        return HIAI_ERROR;
    }

    struct PngInputInfoAPI inputPngData;                               // input data
    inputPngData.inputData = reinterpret_cast<void*>(img.data.get());  // input data pointer addr
    inputPngData.inputSize = img.size;                                 // input data length
    inputPngData.transformFlag = (*m_dvpp_config_).transform_flag;     // whether to transform

    struct PngOutputInfoAPI outputPngData;  // output data

    dvppapi_ctl_msg dvppApiCtlMsg;
    dvppApiCtlMsg.in = reinterpret_cast<void*>(&inputPngData);
    dvppApiCtlMsg.in_size = sizeof(struct PngInputInfoAPI);
    dvppApiCtlMsg.out = reinterpret_cast<void*>(&outputPngData);
    dvppApiCtlMsg.out_size = sizeof(struct PngOutputInfoAPI);

    if (0 != DvppCtl(m_pidvppapi_, DVPP_CTL_PNGD_PROC,
                     &dvppApiCtlMsg)) {  // if this single jpeg pic is processed with error, return
                                         // directly, and process next pic
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]PNGDERROR, FrameID:%u", m_imageFrameID);
        HIAI_DVPP_DFree(outputPngData.address);  // release memory from caller.
        outputPngData.address = NULL;
        return HIAI_ERROR;
    }

    int ret = __handle_vpc_with_param(
        (unsigned char*)outputPngData.outputData, outputPngData.widthAlign, outputPngData.highAlign,
        outputPngData.outputSize, img, FILE_TYPE_PIC_PNG, outputPngData.format, 0);
    HIAI_DVPP_DFree(outputPngData.address);  // release memory from caller.
    outputPngData.address = NULL;
    if (HIAI_OK != ret) {  // if vpc process with error, return directly, and then process next.
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]VPCERROR, FrameID:%u", m_imageFrameID);
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

bool RFImagePreProcess::__need_crop() {
    bool crop = true;
    if (-1 == (*m_dvpp_config_).point_x || -1 == (*m_dvpp_config_).point_y ||
        -1 == (*m_dvpp_config_).crop_width || -1 == (*m_dvpp_config_).crop_height) {
        crop = false;
    }
    return crop;
}

bool RFImagePreProcess::__process_crop(VpcUserCropConfigure& area, const int& width,
                                       const int& height, const int& real_width,
                                       const int& real_height) {
    // default no crop
    uint32_t leftOffset = 0;
    uint32_t rightOffset = real_width - 1;
    uint32_t upOffset = 0;
    uint32_t downOffset = real_height - 1;

    // user crop
    if (__need_crop()) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1] User crop, System crop");
        // amend input offset to avoid the processed range to exceed real range of image
        if ((m_dvpp_config_->point_x + m_dvpp_config_->crop_width) > real_width) {
            m_dvpp_config_->crop_width = real_width - m_dvpp_config_->point_x - 1;
        }
        if ((m_dvpp_config_->point_y + m_dvpp_config_->crop_height) > height) {
            m_dvpp_config_->crop_height = height - m_dvpp_config_->point_y - 1;
        }

        leftOffset = m_dvpp_config_->point_x;
        rightOffset = leftOffset + m_dvpp_config_->crop_width;
        upOffset = m_dvpp_config_->point_y;
        downOffset = upOffset + m_dvpp_config_->crop_height;
        HIAI_ENGINE_LOG(
            HIAI_IDE_INFO,
            "[ImagePreProcess_1] leftOffset: %u, rightOffset: %u, upOffset: %u, downOffset: %u",
            leftOffset, rightOffset, upOffset, downOffset);
    }

    // check param validity(range---max:4096*4096, min:16*16), leftOffset and upOffset cannot larger
    // than real width and height
    if (leftOffset >= real_width || upOffset >= real_height ||
        rightOffset - leftOffset > DVPP_SUPPORT_MAX_WIDTH ||
        rightOffset - leftOffset < DVPP_SUPPORT_MIN_WIDTH ||
        downOffset - upOffset > DVPP_SUPPORT_MAX_HEIGHT ||
        downOffset - upOffset < DVPP_SUPPORT_MIN_HEIGHT) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "crop range error");
        return false;
    }

    // restriction: leftOffset and upOffset of inputputArea must be even, rightOffset and downOffset
    // of inputputArea must be odd.
    area.leftOffset = CHECK_EVEN(leftOffset);
    area.rightOffset = CHECK_ODD(rightOffset);
    area.upOffset = CHECK_EVEN(upOffset);
    area.downOffset = CHECK_ODD(downOffset);
    return true;
}

int RFImagePreProcess::__handle_vpc(const ImageData<u_int8_t>& img, int64_t timestamp) {
    // std::cout << "__handle_vpc" << std::endl;
    if (NULL == img.data || NULL == img.data.get()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]img.data is null, ERROR");
        return HIAI_ERROR;
    }

    if (IMAGE_TYPE_NV12 != (ImageTypeT)img.format) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]the format is not yuv, ERROR");
        return HIAI_ERROR;
    }

    unsigned char* align_buffer = img.data.get();
    int align_width = img.width;
    int align_high = img.height;
    int align_image_len =
        align_width * align_high * 3 / 2;  // the size of yuv data is 1.5 times of width*height
    bool alignMmapFlag = false;
    // std::cout << "__handle_vpc width:" << img.width << ", height:" << img.height << std::endl;
    // if width or height is not align, copy memory
    if (0 != (img.width % VPC_OUT_WIDTH_STRIDE) || 0 != (img.height % VPC_OUT_HIGH_STRIDE)) {
        alignMmapFlag = true;
        align_width = ALIGN_UP(img.width, VPC_OUT_WIDTH_STRIDE);
        align_high = ALIGN_UP(img.height, VPC_OUT_HIGH_STRIDE);
        align_image_len = align_width * align_high * 3 / 2;
        align_buffer = (unsigned char*)HIAI_DVPP_DMalloc(align_image_len);
        if (NULL == align_buffer) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[ImagePreProcess_1]HIAI_DVPP_DMalloc align_buffer is null, ERROR");
            return HIAI_ERROR;
        }
        for (int i = 0; i < img.height; i++) {
            int ret = memcpy_s(align_buffer + i * align_width, align_width,
                               img.data.get() + i * img.width, img.width);
            if (0 != ret) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]memcpy_s error in copy Y");
                HIAI_DVPP_DFree(align_buffer);
                align_buffer = NULL;
                return HIAI_ERROR;
            }
        }
        for (int i = 0; i < img.height / 2; i++) {
            int ret =
                memcpy_s(align_buffer + i * align_width + align_width * align_high, align_width,
                         img.data.get() + i * img.width + img.width * img.height, img.width);
            if (0 != ret) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]memcpy_s error in copy UV");
                HIAI_DVPP_DFree(align_buffer);
                align_buffer = NULL;
                return HIAI_ERROR;
            }
        }
    } else {
        // std::cout << "__handle_vpc memcpy before, align_image_len:" << align_image_len <<
        // std::endl;
        // std::cout << "__handle_vpc img.size:" << img.size << std::endl;
        alignMmapFlag = true;
        align_buffer = (unsigned char*)HIAI_DVPP_DMalloc(align_image_len);
        int ret = memcpy_s(align_buffer, img.size, img.data.get(), img.size);
        // std::cout << "__handle_vpc memcpy after" << std::endl;
        if (0 != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]memcpy_s error in copy UV");
            HIAI_DVPP_DFree(align_buffer);
            align_buffer = NULL;
            return HIAI_ERROR;
        }
    }

    int ret = __handle_vpc_with_param(align_buffer, align_width, align_high, align_image_len, img,
                                      FILE_TYPE_YUV, 3, timestamp);
    if (HIAI_OK != ret) {  // if vpc process with error, return directly, and then process next.
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] call DVPP_CTL_VPC_PROC process faild");
        if (alignMmapFlag) {  // release memory
            HIAI_DVPP_DFree(align_buffer);
            align_buffer = NULL;
        }
        return HIAI_ERROR;
    }
    if (alignMmapFlag) {  // release memory
        HIAI_DVPP_DFree(align_buffer);
        align_buffer = NULL;
    }
    return HIAI_OK;
}

int RFImagePreProcess::__handle_vpc_with_param(const unsigned char* buffer, const int& width,
                                               const int& height, const int64_t& buffer_size,
                                               const ImageData<u_int8_t>& img,
                                               const FILE_TYPE& type, const int& format,
                                               int64_t timestamp) {
    // std::cout << "T:" << std::this_thread::get_id() << " __handle_vpc_with_param " << "type:" <<
    // (int)type << ",format:" << format << std::endl;
    int real_width = img.width;
    int real_height = img.height;

    HIAI_ENGINE_LOG("[ImagePreProcess_1] real_width: %d, real_height: %d, width: %d, height: %d",
                    real_width, real_height, width, height);

    if (NULL == m_pidvppapi_ || NULL == buffer) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]m_pidvppapi_ is null");
        return HIAI_ERROR;
    }

    VpcUserImageConfigure userImage;
    string paraSetPath[1];
    if (type == FILE_TYPE_PIC_JPEG) {
        switch (format) {  // format is responding to color sapce after decoder, which is needed to
                           // match the input color space of vpc
            case DVPP_JPEG_DECODE_OUT_YUV444:
                userImage.inputFormat = INPUT_YUV444_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV422_H2V1:
                userImage.inputFormat = INPUT_YUV422_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV420:
                userImage.inputFormat = INPUT_YUV420_SEMI_PLANNER_VU;
                break;
            case DVPP_JPEG_DECODE_OUT_YUV400:
                userImage.inputFormat = INPUT_YUV400;
                break;
            default:
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]Jpegd out format[%d] is error",
                                format);
                break;
        }
    } else if (type == FILE_TYPE_PIC_PNG) {
        switch (format) {
            case DVPP_PNG_DECODE_OUT_RGB:
                userImage.inputFormat = INPUT_RGB;
                break;
            case DVPP_PNG_DECODE_OUT_RGBA:
                userImage.inputFormat = INPUT_RGBA;
                break;
            default:
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]Pngd out format[%d] is error",
                                format);
                break;
        }
    } else {
        userImage.inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
    }
    userImage.widthStride = width;
    userImage.heightStride = height;
    userImage.outputFormat = OUTPUT_YUV420SP_UV;
    userImage.bareDataAddr = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(buffer));
    userImage.bareDataBufferSize = buffer_size;

    VpcUserRoiConfigure roiConfigure;
    roiConfigure.next = nullptr;
    VpcUserRoiInputConfigure* inputConfigure = &roiConfigure.inputConfigure;
    inputConfigure->cropArea.leftOffset = 0;
    inputConfigure->cropArea.rightOffset = real_width - 1;
    inputConfigure->cropArea.upOffset = 0;
    inputConfigure->cropArea.downOffset = real_height - 1;

    uint32_t resize_width = (uint32_t)m_dvpp_config_->resize_width;
    uint32_t resize_height = (uint32_t)m_dvpp_config_->resize_height;
    if (0 == resize_width || 0 == resize_height) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                        "[ImagePreProcess_1] user donnot need resize, resize width/height use real "
                        "size of pic");
        resize_width = real_width;
        resize_height = real_height;
    }
    if (resize_width > DVPP_SUPPORT_MAX_WIDTH || resize_width < DVPP_SUPPORT_MIN_WIDTH ||
        resize_height > DVPP_SUPPORT_MAX_HEIGHT || resize_height < DVPP_SUPPORT_MIN_HEIGHT) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1]resize range error, resize_width:%u, resize_height:%u",
                        resize_width, resize_height);
        return HIAI_ERROR;
    }

    VpcUserRoiOutputConfigure* outputConfigure = &roiConfigure.outputConfigure;
    if (__process_crop(inputConfigure->cropArea, width, height, real_width, real_height)) {
        // restriction: leftOffset and upOffset of outputArea must be even, rightOffset and
        // downOffset of outputArea must be odd.
        outputConfigure->outputArea.leftOffset = 0;
        outputConfigure->outputArea.rightOffset = CHECK_ODD(resize_width - 1);
        outputConfigure->outputArea.upOffset = 0;
        outputConfigure->outputArea.downOffset = CHECK_ODD(resize_height - 1);
        outputConfigure->widthStride = ALIGN_UP(resize_width, VPC_OUT_WIDTH_STRIDE);
        outputConfigure->heightStride = ALIGN_UP(resize_height, VPC_OUT_HIGH_STRIDE);
        outputConfigure->bufferSize =
            outputConfigure->widthStride * outputConfigure->heightStride * 3 / 2;
        outputConfigure->addr =
            static_cast<uint8_t*>(HIAI_DVPP_DMalloc(outputConfigure->bufferSize));
        // outputConfigure->addr = static_cast<uint8_t*>(HIAI_DMalloc(outputConfigure->bufferSize,
        // hiai::MALLOC_DEFAULT_TIME_OUT, hiai::HIAI_MEMORY_HUGE_PAGE));
        if (nullptr == outputConfigure->addr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]outputConfigure->addr is null");
            return HIAI_ERROR;
        }
        HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                        "[ImagePreProcess_1]inputConfigure cropArea:%u, %u, %u, %u, "
                        "outputConfigure outputArea:%u, %u, %u, %u, stride:%u, %u, %u",
                        inputConfigure->cropArea.leftOffset, inputConfigure->cropArea.rightOffset,
                        inputConfigure->cropArea.upOffset, inputConfigure->cropArea.downOffset,
                        outputConfigure->outputArea.leftOffset,
                        outputConfigure->outputArea.rightOffset,
                        outputConfigure->outputArea.upOffset,
                        outputConfigure->outputArea.downOffset, outputConfigure->widthStride,
                        outputConfigure->heightStride, outputConfigure->bufferSize);
    } else {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]__process_crop error");
        return HIAI_ERROR;
    }
    userImage.roiConfigure = &roiConfigure;

    if (!m_dvpp_config_->dvpp_para.empty()) {
        paraSetPath[0] += m_dvpp_config_->dvpp_para.c_str();
        userImage.yuvScalerParaSetAddr = reinterpret_cast<uint64_t>(paraSetPath);
        userImage.yuvScalerParaSetSize = 1;
        userImage.yuvScalerParaSetIndex = 0;
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1_1]dvpp_para:%s",
                        m_dvpp_config_->dvpp_para.c_str());
    }

    dvppapi_ctl_msg dvppApiCtlMsg;
    dvppApiCtlMsg.in = reinterpret_cast<uint8_t*>(&userImage);
    dvppApiCtlMsg.in_size = sizeof(VpcUserImageConfigure);
    if (0 != DvppCtl(m_pidvppapi_, DVPP_CTL_VPC_PROC, &dvppApiCtlMsg)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] call dvppctl process:VPC faild");
        HIAI_DVPP_DFree(outputConfigure->addr);
        outputConfigure->addr = nullptr;
        return HIAI_ERROR;
    }

    if (NULL == m_dvpp_out_) {
        m_dvpp_out_ = std::make_shared<BatchImageParaWithScaleT>();
        if (NULL == m_dvpp_out_ || NULL == m_dvpp_out_.get()) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[ImagePreProcess_1] call m_dvpp_out_ make_shared failed");
            HIAI_DVPP_DFree(outputConfigure->addr);
            outputConfigure->addr = nullptr;
            return HIAI_ERROR;
        }
        m_dvpp_out_->b_info = m_dvpp_in_->b_info;
        m_dvpp_out_->b_info.frame_ID.clear();
    }

    std::shared_ptr<NewImageParaT> image = std::make_shared<NewImageParaT>();
    if (NULL == image || NULL == image.get()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] call image make_shared failed");
        HIAI_DVPP_DFree(outputConfigure->addr);
        outputConfigure->addr = nullptr;
        return HIAI_ERROR;
    }

    image->img.width =
        outputConfigure->outputArea.rightOffset - outputConfigure->outputArea.leftOffset + 1;
    image->img.height =
        outputConfigure->outputArea.downOffset - outputConfigure->outputArea.upOffset + 1;
    image->img.size = outputConfigure->bufferSize;
    image->img.channel = img.channel;
    image->img.format = img.format;
    image->scale_info.scale_width = (1.0 * image->img.width) / img.width;
    image->scale_info.scale_height = (1.0 * image->img.height) / img.height;
    image->resize_info.resize_width = m_dvpp_config_->resize_width;
    image->resize_info.resize_height = m_dvpp_config_->resize_height;
    image->crop_info.point_x = m_dvpp_config_->point_x;
    image->crop_info.point_y = m_dvpp_config_->point_y;
    image->crop_info.crop_width = m_dvpp_config_->crop_width;
    image->crop_info.crop_height = m_dvpp_config_->crop_height;
    image->timestamp = timestamp;
    // std::cout << "__handle_vpc_with_param " << "w:" << image->img.width << ",h:" <<
    // image->img.height << ",channel:" << image->img.channel << ",size:" <<  image->img.size <<
    // std::endl;
    // std::cout << "format:" << image->img.format << " scale w:" << image->scale_info.scale_width
    // << ",h:" << image->scale_info.scale_height << std::endl;
    // std::cout << "resize w:" << image->resize_info.resize_width << ",h:" <<
    // image->resize_info.resize_height << std::endl;
    // std::cout << "crop (" << image->crop_info.point_x << "," << image->crop_info.point_y << ","
    // << image->crop_info.crop_width << "," << image->crop_info.crop_height << std::endl;
    // std::cout << "stride w:" << outputConfigure->widthStride << ",h:" <<
    // outputConfigure->heightStride << ",size:" << outputConfigure->bufferSize << std::endl;

    uint8_t* tmp = nullptr;
    try {
        tmp = new uint8_t[image->img.size];
    } catch (const std::bad_alloc& e) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1] failed to allocate buffer for out_buffer");
        return HIAI_ERROR;
    }
    std::shared_ptr<u_int8_t> out_buffer(tmp);
    if (NULL == out_buffer || NULL == out_buffer.get()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] call out_buffer shared_ptr failed");
        HIAI_DVPP_DFree(outputConfigure->addr);
        outputConfigure->addr = nullptr;
        return HIAI_ERROR;
    }
    int ret = memcpy_s(out_buffer.get(), image->img.size, outputConfigure->addr, image->img.size);
    HIAI_DVPP_DFree(outputConfigure->addr);
    outputConfigure->addr = nullptr;
    if (ret != 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1] memcpy_s out buffer data error");
        out_buffer = nullptr;
        return HIAI_ERROR;
    }
    image->img.data = out_buffer;
    m_dvpp_out_->v_img.push_back(*image);
    m_dvpp_out_->b_info.frame_ID.push_back(m_imageFrameID);

    if (1 == m_dvpp_config_->dump_value) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                        "[ImagePreProcess_1]preprocess need dump, width:%u, height:%u",
                        image->img.width, image->img.height);
        DvppPreprocessInfo info = {resize_width,
                                   resize_height,
                                   outputConfigure->widthStride,
                                   outputConfigure->heightStride,
                                   m_imageFrameID,
                                   m_orderInFrame};
        return storePreprocessImage(out_buffer.get(), image->img.size, info);
    }

    return HIAI_OK;
}

int RFImagePreProcess::storePreprocessImage(const u_int8_t* out_buffer, const uint32_t& size,
                                            const DvppPreprocessInfo& info) {
    if (NULL == out_buffer) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1]storePreprocessImage, out_buffer is null!");
        return HIAI_ERROR;
    }
    // important:after dvpp, the output data is that width is aligned with 128, and height is
    // aligned with 16.
    std::string fileName = "/projects/" + m_dvpp_config_->project_name +
                           "/out/PreProcess/ImagePreProcess_1/" + std::to_string(m_imageFrameID) +
                           "_" + std::to_string(m_orderInFrame + 1) + "_preprocessYUV";
    IDE_SESSION session = ideOpenFile(NULL, reinterpret_cast<const char*>(fileName.c_str()));
    HIAI_ENGINE_LOG("save fileName:%s!", fileName.c_str());
    if (NULL == session) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1]storePreprocessImage, ideOpenFile is null!");
        return HIAI_ERROR;
    }

    uint32_t sendSize = 0;
    char* tmp = const_cast<char*>(reinterpret_cast<const char*>(out_buffer));
    while (sendSize < size) {
        uint32_t everySize =
            (sendSize + SEND_BUFFER_SIZE > size) ? (size - sendSize) : SEND_BUFFER_SIZE;
        if (IDE_DAEMON_NONE_ERROR != ideWriteFile(session, tmp, everySize)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[ImagePreProcess_1]storePreprocessImage, ideWriteFile error!");
            ideCloseFile(session);
            return HIAI_ERROR;
        }
        tmp += everySize;
        sendSize += everySize;
    }

    HIAI_ENGINE_LOG(
        "preprocess info, resize_width:%u, resize_height:%u, preprocess_width:%u, "
        "preprocess_height:%u, frameID:%u, order:%u",
        info.resize_width, info.resize_height, info.preprocess_width, info.preprocess_height,
        info.frameID, info.orderInFrame);
    // add info of yuv file to file end which are used to transfer to jpg
    if (IDE_DAEMON_NONE_ERROR != ideWriteFile(session, &info, sizeof(info))) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1]storePreprocessImage, ideWriteFile error!");
        ideCloseFile(session);
        return HIAI_ERROR;
    }

    usleep(SEND_DATA_SLEEP_MS);  // assure ide-daemon-host catch all data.
    if (IDE_DAEMON_NONE_ERROR != ideCloseFile(session)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImagePreProcess_1]storePreprocessImage, ideCloseFile error!");
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(
        HIAI_IDE_INFO,
        "[ImagePreProcess_1]storePreprocessImage, storage success!, sendSize:%u, size:%u", sendSize,
        size);
    return HIAI_OK;
}

int RFImagePreProcess::__handle_dvpp() {
    if (NULL == m_dvpp_in_ || NULL == m_dvpp_in_.get()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]__hanlde_dvpp, input data is null!");
        return HIAI_ERROR;
    }

    int index_crop = 0;
    int i = 0;
    m_orderInFrame = 0;
    for (std::vector<NewImageParaT>::iterator iter = m_dvpp_in_->v_img.begin();
         iter != m_dvpp_in_->v_img.end(); ++iter, i++) {
        m_imageFrameID = m_dvpp_in_->b_info.frame_ID[i];
        if (IMAGE_TYPE_JPEG == (ImageTypeT)(*iter).img.format) {
            if (NULL != m_dvpp_crop_in_ && NULL != m_dvpp_crop_in_.get()) {
                if (m_dvpp_crop_in_->v_location.empty() ||
                    m_dvpp_crop_in_->v_location[index_crop].range.empty()) {
                    continue;
                }
                for (std::vector<Rectangle<Point2D>>::iterator iterRect =
                         m_dvpp_crop_in_->v_location[index_crop].range.begin();
                     iterRect != m_dvpp_crop_in_->v_location[index_crop].range.end(); ++iterRect) {
                    m_orderInFrame++;
                    if (HIAI_OK != __update_crop_para(*iterRect) ||
                        HIAI_OK != __handle_jpeg((*iter).img)) {
                        HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                        "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                        continue;
                    }
                }
            } else {
                if (HIAI_OK != __handle_jpeg((*iter).img)) {
                    HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                    "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                    continue;
                }
            }
        } else if (IMAGE_TYPE_PNG == (ImageTypeT)(*iter).img.format) {
            if (NULL != m_dvpp_crop_in_ && NULL != m_dvpp_crop_in_.get()) {
                if (m_dvpp_crop_in_->v_location.empty() ||
                    m_dvpp_crop_in_->v_location[index_crop].range.empty()) {
                    continue;
                }
                for (std::vector<Rectangle<Point2D>>::iterator iterRect =
                         m_dvpp_crop_in_->v_location[index_crop].range.begin();
                     iterRect != m_dvpp_crop_in_->v_location[index_crop].range.end(); ++iterRect) {
                    m_orderInFrame++;
                    if (HIAI_OK != __update_crop_para(*iterRect) ||
                        HIAI_OK != __handle_png((*iter).img)) {
                        HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                        "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                        continue;
                    }
                }
            } else {
                if (HIAI_OK != __handle_png((*iter).img)) {
                    HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                    "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                    continue;
                }
            }
        } else {  // default IMAGE_TYPE_NV12
            if (NULL != m_dvpp_crop_in_ && NULL != m_dvpp_crop_in_.get()) {
                if (m_dvpp_crop_in_->v_location.empty() ||
                    m_dvpp_crop_in_->v_location[index_crop].range.empty()) {
                    continue;
                }
                for (std::vector<Rectangle<Point2D>>::iterator iterRect =
                         m_dvpp_crop_in_->v_location[index_crop].range.begin();
                     iterRect != m_dvpp_crop_in_->v_location[index_crop].range.end(); ++iterRect) {
                    m_orderInFrame++;
                    if (HIAI_OK != __update_crop_para(*iterRect) ||
                        HIAI_OK != __handle_vpc((*iter).img, (*iter).timestamp)) {
                        HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                        "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                        continue;
                    }
                }
            } else {
                if (HIAI_OK != __handle_vpc((*iter).img, (*iter).timestamp)) {
                    HIAI_ENGINE_LOG(HIAI_IDE_WARNING,
                                    "[ImagePreProcess_1]Handle One Image Error, Continue Next");
                    continue;
                }
            }
        }
        index_crop++;
    }

    return HIAI_OK;
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS: realize RFImagePreProcess
* @[in]: define a input and output,
*        and register Engine named RFImagePreProcess
*/
HIAI_IMPL_ENGINE_PROCESS("RFImagePreProcess", RFImagePreProcess, INPUT_SIZE) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1] start process!");
    int err_code = HIAI_OK;
    std::shared_ptr<BatchImageParaWithScaleT> error_handler = NULL;

    if (arg0 != NULL) {
        std::shared_ptr<BatchImageParaWithScaleT> dataInput =
            std::static_pointer_cast<BatchImageParaWithScaleT>(arg0);
        if (!isSentinelImage(dataInput)) {
            if (m_data_input_in_ != NULL) {
                if (m_data_input_in_->b_info.batch_ID == dataInput->b_info.batch_ID &&
                    !dataInput->v_img.empty() && !dataInput->b_info.frame_ID.empty()) {
                    m_data_input_in_->v_img.push_back(dataInput->v_img[0]);
                    m_data_input_in_->b_info.frame_ID.push_back(dataInput->b_info.frame_ID[0]);
                }
            } else {
                m_data_input_in_ = dataInput;
            }
            if (m_data_input_in_->v_img.size() != m_data_input_in_->b_info.batch_size) {
                HIAI_ENGINE_LOG(
                    HIAI_IDE_INFO, "[ImagePreProcess_1] Wait for other %d batch image info!",
                    (m_data_input_in_->b_info.batch_size - m_data_input_in_->v_img.size()));
                return HIAI_OK;
            }
            input_que_.PushData(0, m_data_input_in_);
            m_data_input_in_ = NULL;
        } else {
            input_que_.PushData(0, arg0);
        }
    }
#if INPUT_SIZE == 2
    input_que_.PushData(1, arg1);
    if (!input_que_.PopAllData(m_dvpp_in_, m_dvpp_crop_in_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[ImagePreProcess_1]fail to pop all data");
        return HIAI_ERROR;
    }
#else
    if (!input_que_.PopAllData(m_dvpp_in_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_WARNING, "[ImagePreProcess_1]fail to pop all data");
        return HIAI_ERROR;
    }
#endif
    // add sentinel image for showing this data in dataset are all sended, this is last step.
    // std::cout << __FUNCTION__ << ":" << __LINE__ << " isSentinelImage:" <<
    // (isSentinelImage(m_dvpp_in_) ? "true" : "false") << std::endl;
    if (isSentinelImage(m_dvpp_in_)) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1]sentinel Image, process over.");
        HIAI_StatusT hiai_ret = HIAI_OK;
        do {
            hiai_ret =
                SendData(0, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(m_dvpp_in_));
            if (HIAI_OK != hiai_ret) {
                if (HIAI_ENGINE_NULL_POINTER == hiai_ret || HIAI_HDC_SEND_MSG_ERROR == hiai_ret ||
                    HIAI_HDC_SEND_ERROR == hiai_ret || HIAI_GRAPH_SRC_PORT_NOT_EXIST == hiai_ret ||
                    HIAI_GRAPH_ENGINE_NOT_EXIST == hiai_ret) {
                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                                    "[ImagePreProcess_1] SendData error[%d], break.", hiai_ret);
                    break;
                }
                HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                                "[ImagePreProcess_1] SendData return value[%d] not OK, sleep 200ms",
                                hiai_ret);
                usleep(SEND_DATA_INTERVAL_MS);
            }
        } while (HIAI_OK != hiai_ret);
        return hiai_ret;
    }
    // std::cout << "T:" << std::this_thread::get_id() << " isSentinelImage if after" << std::endl;

    m_dvpp_out_ = NULL;

    // 1.check the pending data is jpeg, png, yuv or none
    // 2.preprocess before vpc(DVPP_CTL_JPEGD_PROC, DVPP_CTL_PNGD_PROC, or something else)
    // 3.calculate width, height, and so on, according to the user config of crop and resize, and
    // DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM
    // 4.DVPP_CTL_VPC_PROC after create dvppapi_ctl_msg para
    // 5.dump yuv data to project dir out after dvpp, according to dump_value config(from device to
    // IDE with IDE-deamon-client&IDE-deamon-hiai process)
    // 6.send data to next engine after the process of dvpp
    err_code = __handle_dvpp();
    if (HIAI_ERROR == err_code) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[ImagePreProcess_1]dvpp process error!");
        goto ERROR;
    }

    if (!__send_data()) {  // send to next engine after dvpp process
        goto ERROR;
    }
    __clear_data();
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImagePreProcess_1] end process!");
    return HIAI_OK;

ERROR:
    // send null to next node to avoid blocking when to encounter abnomal situation.
    SendData(0, "BatchImageParaWithScaleT", std::static_pointer_cast<void>(error_handler));
    __clear_data();
    return HIAI_ERROR;
}
