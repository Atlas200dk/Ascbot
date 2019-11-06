/**
* @file dvpp_engine.h
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-4-25
*/

#ifndef _IMAGEPREPROCESS_1_H_
#define _IMAGEPREPROCESS_1_H_

#include "BatchImageParaWithScale.h"
#include "dvpp/Vpc.h"
#include "dvpp/idvppapi.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/engine.h"
#include "hiaiengine/multitype_queue.h"

#define INPUT_SIZE 1
#define OUTPUT_SIZE 1

using hiai::Engine;
using hiai::AIConfig;
using hiai::AIModelDescription;
using hiai::ImageData;
using hiai::BatchInfo;
using hiai::BatchImagePara;
using hiai::BatchDetectedObjectPara;
using hiai::Rectangle;
using hiai::Point2D;

typedef struct DvppConfig {
    int point_x;      // the abscissa of crop(must be even)
    int point_y;      // the ordinate of crop(must be even)
    int crop_width;   // the width of crop(must be even)
    int crop_height;  // the height of crop(must be even)
    int self_crop;    // user defined
    int dump_value;   // the switch of dump image after preProcess, 0:no dump, 1:dump
    std::string project_name;
    float resize_width;
    float resize_height;

    bool crop_before_resize;  // whether resize is before crop

    bool yuv420_need;  // whether to need yuv420 semi-planar to output data
                       // Jpegd support raw and yuv420sp output, while op_type is set to 1
    bool
        v_before_u;  // output sp format, uv order. whether v is before u, while op_type is set to 1
    int transform_flag;     // format transform flag, while op_type is set to 2
    std::string dvpp_para;  // dvpp para path file name

    DvppConfig() {
        point_x = -1;
        point_y = -1;
        crop_width = -1;
        crop_height = -1;
        self_crop = 1;
        dump_value = 0;
        project_name = "";
        resize_width = 0;
        resize_height = 0;
        crop_before_resize = true;
        yuv420_need = false;
        v_before_u = true;
        transform_flag = 0;
        dvpp_para = "";
    }
} DvppConfig;

typedef struct DvppPreprocessInfo {
    uint32_t resize_width;
    uint32_t resize_height;
    uint32_t preprocess_width;
    uint32_t preprocess_height;
    uint32_t frameID;
    uint32_t orderInFrame;
    uint32_t reserverd[2];
} DvppPreprocessInfo;

typedef enum {
    YUV420_SEMI_PLANNAR = 0,  // 0
    YUV422_SEMI_PLANNAR,
    YUV444_SEMI_PLANNAR,
    YUV422_PACKED,
    YUV444_PACKED,
    RGB888_PACKED,  // 5
    XRGB8888_PACKED,
    YUV400_SEMI_PLANNAR,
} Imge_Type;

typedef enum {
    RANK_TYPE_NV12 = 0,
    RANK_TYPE_NV21,
    RANK_TYPE_YUYV,
    RANK_TYPE_YVYU,
    RANK_TYPE_UYVY,
    RANK_TYPE_YUV,

    RANK_TYPE_RGB,  // 6
    RANK_TYPE_BGR,
    RANK_TYPE_RGBA,
    RANK_TYPE_BGRA,
    RANK_TYPE_ARGB,

    RANK_TYPE_ABGR,  // 11
} IMAGE_RANK_TYPE;

// self_defined, reason as follows by dvpp, png format pic only supports RGB & RGBA
enum pngd_color_space { DVPP_PNG_DECODE_OUT_RGB = 2, DVPP_PNG_DECODE_OUT_RGBA = 6 };

typedef enum {
    FILE_TYPE_PIC_JPEG = 0x1,
    FILE_TYPE_PIC_PNG,
    FILE_TYPE_YUV,
    FILE_TYPE_MAX
} FILE_TYPE;

// Define RFImagePreProcess
class RFImagePreProcess : public Engine {
private:
    const int SEND_BUFFER_SIZE = 1024 * 1024;  // transfer max limit each once:1024KB

public:
    RFImagePreProcess()
        : input_que_(INPUT_SIZE),
          m_dvpp_config_(NULL),
          m_dvpp_out_(NULL),
          m_dvpp_in_(NULL),
          m_pidvppapi_(NULL),
          m_dvpp_crop_in_(NULL),
          m_imageFrameID(0),
          m_orderInFrame(0),
          m_data_input_in_(NULL) {
    }
    ~RFImagePreProcess();
    HIAI_StatusT Init(const AIConfig& config, const std::vector<AIModelDescription>& model_desc);
    HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

private:
    bool __need_crop();
    void __clear_data();
    int __handle_jpeg(const ImageData<u_int8_t>& img);
    int __handle_png(const ImageData<u_int8_t>& img);
    int __handle_vpc(const ImageData<u_int8_t>& img, int64_t timestamp);
    int __handle_vpc_with_param(const unsigned char* buffer, const int& width, const int& height,
                                const int64_t& buffer_size, const ImageData<u_int8_t>& img,
                                const FILE_TYPE& type, const int& format, int64_t timestamp);
    bool __process_crop(VpcUserCropConfigure& area, const int& width, const int& height,
                        const int& real_width, const int& real_height);
    bool __send_data();
    int storePreprocessImage(const u_int8_t* out_buffer, const uint32_t& size,
                             const DvppPreprocessInfo& info);
    int __handle_dvpp();
    int __update_crop_para(const Rectangle<Point2D>& rect);

private:
    std::shared_ptr<DvppConfig> m_dvpp_config_;
    std::shared_ptr<BatchImageParaWithScaleT> m_dvpp_out_;
    std::shared_ptr<BatchImageParaWithScaleT> m_dvpp_in_;
    std::shared_ptr<BatchImageParaWithScaleT> m_data_input_in_;
    std::shared_ptr<BatchDetectedObjectPara<Rectangle<Point2D>, float>> m_dvpp_crop_in_;
    IDVPPAPI* m_pidvppapi_;
    uint32_t m_imageFrameID;
    uint32_t m_orderInFrame;  // multi net for mark order from one image cutout
    hiai::MultiTypeQueue input_que_;
};

#endif  // _IMAGEPREPROCESS_1_H_
