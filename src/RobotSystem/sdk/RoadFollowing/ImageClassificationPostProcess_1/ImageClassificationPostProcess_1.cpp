/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-30
*/
#include "ImageClassificationPostProcess_1.h"
#include <hiaiengine/log.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

HIAI_REGISTER_DATA_TYPE("EngineTransT", EngineTransT);
// HIAI_REGISTER_DATA_TYPE("RFResultT", RFResultT);

HIAI_StatusT RFPostProcess::Init(const hiai::AIConfig& config,
                                 const std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] start init!");
    hiai::AIStatus ret = hiai::SUCCESS;
    if (postprocess_config_ == nullptr) {
        postprocess_config_ = std::make_shared<PostprocessConfig>();
    }
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        std::string name = item.name();
        if (name == "path") {
            postprocess_config_->path = item.value();
        } else if (name == "output_name") {
            postprocess_config_->output_node = item.value();
        }
    }

#ifdef DUMP_RESULT_FILE
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] config path:%s!",
                    postprocess_config_->path.c_str());
    std::string info_file_ = GetInfoFilePath(postprocess_config_->path);
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] info file:%s!",
                    info_file_.c_str());
    id_img_correlation.clear();
    id_img_correlation = SetImgPredictionCorrelation(info_file_, ".txt");
    if (id_img_correlation.empty()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1] scan info file failed!");
        return HIAI_ERROR;
    }
#endif
    uint32_t graph_id = Engine::GetGraphId();
    std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(graph_id);
    if (nullptr == graph) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Fail to get the graph");
        return HIAI_ERROR;
    }
#ifdef DUMP_RESULT_FILE
    std::ostringstream deviceId;
    deviceId << graph->GetDeviceID();
    string device_dir = RESULT_FOLDER + "/" + deviceId.str();
    store_path = device_dir + "/" + ENGINE_NAME;
    if (HIAI_OK != CreateFolder(RESULT_FOLDER, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(device_dir, PERMISSION)) {
        return HIAI_ERROR;
    }
    if (HIAI_OK != CreateFolder(store_path, PERMISSION)) {
        return HIAI_ERROR;
    }
#endif
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] end init!");
    return HIAI_OK;
}

/**
* @brief: send sentinel image to inform the graph to destroy
*/
HIAI_StatusT RFPostProcess::SendSentinelImage() {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                    "[ImageClassificationPostProcess_1] sentinel image, process success!");
    std::shared_ptr<RFResultT> result_data(new RFResultT());
    HIAI_StatusT hiai_ret = HIAI_OK;
    do {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                        "[ImageClassificationPostProcess_1] sentinel image, process success!");
        hiai_ret = SendData(0, "RFResultT", std::static_pointer_cast<void>(result_data));
        if (HIAI_OK != hiai_ret) {
            HIAI_ENGINE_LOG(
                HIAI_IDE_INFO,
                "[ImageClassificationPostProcess_1] SendData return value[%d] not OK, sleep 200ms",
                hiai_ret);
            usleep(SEND_DATA_INTERVAL_MS);
        } else if (HIAI_ENGINE_NULL_POINTER == hiai_ret || HIAI_HDC_SEND_MSG_ERROR == hiai_ret ||
                   HIAI_HDC_SEND_ERROR == hiai_ret || HIAI_GRAPH_SRC_PORT_NOT_EXIST == hiai_ret ||
                   HIAI_GRAPH_ENGINE_NOT_EXIST == hiai_ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[ImageClassificationPostProcess_1] SendData error[%d], break.",
                            hiai_ret);
            break;
        }
    } while (HIAI_OK != hiai_ret);
    return hiai_ret;
}

HIAI_IMPL_ENGINE_PROCESS("RFPostProcess", RFPostProcess, INPUT_SIZE) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1] start process!");

    HIAI_StatusT hiai_ret = HIAI_OK;
    if (nullptr == arg0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1] fail to process invalid message");
        return HIAI_ERROR;
    }
    std::shared_ptr<EngineTransT> tran = std::static_pointer_cast<EngineTransT>(arg0);
    std::vector<OutputT> output_data_vec = tran->output_data_vec;
    // add sentinel image for showing this data in dataset are all sended, this is
    // last step.
    BatchImageParaWithScaleT image_handle = {tran->b_info, tran->v_img};
    if (isSentinelImage(std::make_shared<BatchImageParaWithScaleT>(image_handle))) {
        return SendSentinelImage();
    }
    if (!tran->status) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, tran->msg.c_str());
        return HIAI_ERROR;
    }
    bool validate_name = false;
    OutputT out;
    for (int i = 0; i < output_data_vec.size(); i++) {
        out = output_data_vec[i];
        std::string index;
        std::string cur_name;
        GetLayerName(out.name, index, cur_name);
        std::string match_name = postprocess_config_->output_node;
        if (match_name == cur_name) {
            validate_name = true;
            break;  // here assume that the classification network only has one output
        }
    }
    if (validate_name == false) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1]: the output node name doesn't exist");
        return HIAI_ERROR;
    }

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[ImageClassificationPostProcess_1]: %s", out.name.c_str());
    int size = out.size / sizeof(float);
    if (size <= 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1]: the OutPutT size less than 0!");
        return HIAI_ERROR;
    }
    float* result = nullptr;
    try {
        result = new float[size];
    } catch (const std::bad_alloc& e) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1]:failed to allocate result!");
        return HIAI_ERROR;
    }
    int ret = memcpy_s(result, sizeof(float) * size, out.data.get(), sizeof(float) * size);
    if (ret != 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[ImageClassificationPostProcess_1]: memcpy_s out data error!");
        delete[] result;
        return HIAI_ERROR;
    }
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "run to postprocess: size %d, batch_size %d", size,
                    tran->b_info.batch_size);

    std::shared_ptr<RFResultT> result_data(new RFResultT());
    Point2f point;
    point.start = result[0];
    point.end = result[1];
    result_data->data.push_back(point);
    if (tran->v_img.size() > 0) {
        result_data->width = tran->v_img[0].img.width;
        result_data->height = tran->v_img[0].img.height;
        float scale_width = tran->v_img[0].scale_info.scale_width;
        float scale_height = tran->v_img[0].scale_info.scale_height;
        int resize_width = tran->v_img[0].resize_info.resize_width;
        int resize_height = tran->v_img[0].resize_info.resize_height;
        result_data->scale_width = scale_width;
        result_data->scale_height = scale_height;
        result_data->timestamp = tran->v_img[0].timestamp;
    }
    // std::cout << "output width:" << result_data->width << ",height:" <<
    // result_data->height << std::endl;
    // std::cout << "output input scale width:" << scale_width << ", height:" <<
    // scale_height << std::endl;
    // std::cout << "output input resize width:" << resize_width << ", height:" <<
    // resize_height << std::endl;
    SendData(0, "RFResultT", std::static_pointer_cast<void>(result_data));
    return HIAI_OK;
}
