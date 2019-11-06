#ifndef _BATCHIMAGEPARAWITHSCALE_H_
#define _BATCHIMAGEPARAWITHSCALE_H_

#include <sys/stat.h>
#include <unistd.h>
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/status.h"

using hiai::BatchInfo;
using hiai::IMAGEFORMAT;
using hiai::ImageData;

/**
define error code for HIAI_ENGINE_LOG
**/
#define USE_DEFINE_ERROR 0x6001

enum { HIAI_IDE_ERROR_CODE, HIAI_IDE_INFO_CODE, HIAI_IDE_WARNING_CODE };

HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_ERROR, HIAI_IDE_ERROR, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_INFO, HIAI_IDE_INFO, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_WARNING, HIAI_IDE_WARNING, "");

typedef struct ScaleInfo {
    float scale_width = 1;
    float scale_height = 1;
} ScaleInfoT;
template <class Archive>
void serialize(Archive& ar, ScaleInfoT& data) {
    ar(data.scale_width, data.scale_height);
}

typedef struct ResizeInfo {
    uint32_t resize_width = 0;
    uint32_t resize_height = 0;
} ResizeInfoT;
template <class Archive>
void serialize(Archive& ar, ResizeInfo& data) {
    ar(data.resize_width, data.resize_height);
}

typedef struct CropInfo {
    int point_x = -1;
    int point_y = -1;
    int crop_width = -1;
    int crop_height = -1;
} CropInfoT;
template <class Archive>
void serialize(Archive& ar, CropInfo& data) {
    ar(data.point_x, data.point_y, data.crop_width, data.crop_height);
}

typedef struct NewImagePara {
    hiai::FrameInfo f_info;
    hiai::ImageData<u_int8_t> img;
    ScaleInfoT scale_info;
    ResizeInfo resize_info;
    CropInfo crop_info;
    int64_t timestamp;
} NewImageParaT;

template <class Archive>
void serialize(Archive& ar, NewImageParaT& data) {
    ar(data.f_info, data.img, data.scale_info, data.resize_info, data.crop_info);
}

typedef struct NewImagePara2 {
    hiai::FrameInfo f_info;
    hiai::ImageData<float> img;
    ScaleInfoT scale_info;
} NewImageParaT2;

template <class Archive>
void serialize(Archive& ar, NewImageParaT2& data) {
    ar(data.f_info, data.img, data.scale_info);
}

typedef struct BatchImageParaWithScale {
    hiai::BatchInfo b_info;
    std::vector<NewImageParaT> v_img;
} BatchImageParaWithScaleT;

template <class Archive>
void serialize(Archive& ar, BatchImageParaWithScaleT& data) {
    ar(data.b_info, data.v_img);
}

struct ImageAll {
    int width_org;
    int height_org;
    int channal_org;
    ImageData<float> image;
};

template <class Archive>
void serialize(Archive& ar, ImageAll& data) {
    ar(data.width_org, data.height_org, data.channal_org, data.image);
}

struct BatchImageParaScale {
    BatchInfo b_info;
    std::vector<ImageAll> v_img;
};

template <class Archive>
void serialize(Archive& ar, BatchImageParaScale& data) {
    ar(data.b_info, data.v_img);
}

typedef enum ImageType {
    IMAGE_TYPE_RAW = -1,
    IMAGE_TYPE_NV12 = 0,
    IMAGE_TYPE_JPEG,
    IMAGE_TYPE_PNG,
    IMAGE_TYPE_BMP,
    IMAGE_TYPE_TIFF,
    IMAGE_TYPE_VIDEO = 100
} ImageTypeT;

struct EvbImageInfo {
    bool is_first;
    bool is_last;
    uint32_t batch_size;
    uint32_t batch_index;
    uint32_t max_batch_size;
    uint32_t batch_ID;
    uint32_t frame_ID;
    int format;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t size = 0;
    u_int8_t* pucImageData;
};

// define road following detect output result struct.
typedef struct hiai::Line<float> Point2f;
typedef struct RFResult {
    float data1 = 0.0f;
    std::vector<Point2f> data;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t format = 0;
    float scale_width = 0.0f;
    float scale_height = 0.0f;
    int64_t timestamp = 0;
} RFResultT;

// define road following detect input data struct.
typedef struct RFInput {
    std::string file;
    int width = 0;
    int height = 0;
    int size = 0;
    int format = IMAGE_TYPE_NV12;
} RFInputT;

const int SEND_DATA_INTERVAL_MS = 200000;
static const mode_t PERMISSION = 0700;
static const mode_t FIlE_PERMISSION = 0600;

// The new version of serialize function
extern void GetEvbImageInfoSearPtr(void* input_ptr, std::string& ctrl_str, uint8_t*& data_ptr,
                                   uint32_t& data_len);

extern bool isSentinelImage(const std::shared_ptr<BatchImageParaWithScaleT> image_handle);

// The new version of deserialize function
extern std::shared_ptr<void> GetEvbImageInfoDearPtr(const char* ctrl_ptr, const uint32_t& ctr_len,
                                                    const uint8_t* data_ptr,
                                                    const uint32_t& data_len);

typedef struct Output {
    int32_t size;
    std::string name;
    std::shared_ptr<u_int8_t> data;
} OutputT;

template <class Archive>
void serialize(Archive& ar, OutputT& data) {
    ar(data.size);
    ar(data.name);
    if (data.size > 0 && data.data.get() == nullptr) {
        data.data.reset(new u_int8_t[data.size]);
    }

    ar(cereal::binary_data(data.data.get(), data.size * sizeof(u_int8_t)));
}

typedef struct EngineTrans {
    bool status;
    std::string msg;
    hiai::BatchInfo b_info;
    uint32_t size;
    std::vector<OutputT> output_data_vec;
    std::vector<NewImageParaT> v_img;
} EngineTransT;

template <class Archive>
void serialize(Archive& ar, EngineTransT& data) {
    ar(data.status, data.msg, data.b_info, data.size, data.output_data_vec, data.v_img);
}

typedef struct {
    std::string tfilename;
    int format;
    int height;
    int width;
} ImageInfor;

/**
* @brief: get the result file name from the image name
* @[in]: imgFullPath: the image file path
* @[in]: postfix: the type of the result file
*/
extern std::string GenTfileName(std::string imgFullPath, std::string postfix);

/**
* @brief: get the image information from the info_file generated by dataset engine
* @[in]: info_file: the info file path
* @[in]: postfix: the type of the result file
*/
extern std::unordered_map<int, ImageInfor> SetImgPredictionCorrelation(std::string info_file,
                                                                       std::string postfix);

/**
* @brief: get the caffe layer name and index
* @[in]: in_name: the name of tensor
* @[in]: index: the index of output tensor
* @[in]: out_name: the caffe layer name
*/
extern void GetLayerName(const std::string in_name, std::string& index, std::string& out_name);

/**
* @brief: create folder to store the detection results
* the folder name on the host will be "result_files/enginename"
*/
extern HIAI_StatusT CreateFolder(std::string folderPath, mode_t mode);

/**
* @brief: get the information file path in the dataset folder
* @[in]: value, path configuration string
* @[return]: string, info file path
*/
extern std::string GetInfoFilePath(const std::string pathConfig);
#endif  // _BATCHIMAGEPARAWITHSCALE_H_
