/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-22
*/
#include "MindInferenceEngine_1.h"
#include <hiaiengine/ai_types.h>
#include <hiaiengine/log.h>
#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "hiaiengine/ai_memory.h"

static const int IMAGE_INFO_DATA_NUM = 3;
HIAI_REGISTER_DATA_TYPE("EngineTransT", EngineTransT);
HIAI_REGISTER_DATA_TYPE("OutputT", OutputT);
HIAI_REGISTER_DATA_TYPE("ScaleInfoT", ScaleInfoT);
HIAI_REGISTER_DATA_TYPE("NewImageParaT", NewImageParaT);
HIAI_REGISTER_DATA_TYPE("BatchImageParaWithScaleT", BatchImageParaWithScaleT);
HIAI_REGISTER_SERIALIZE_FUNC("EvbImageInfo", EvbImageInfo, GetEvbImageInfoSearPtr,
                             GetEvbImageInfoDearPtr);

/**
* @brief: clear buffer in vector
*/
void ObjectInferenceEngine::ClearOutData() {
    input_data_vec.clear();
    // release outData pre allocate memmory
    for (auto buffer : m_outData_) {
        if (buffer != nullptr) {
            hiai::HIAIMemory::HIAI_DFree(buffer);
            buffer = nullptr;
        }
    }
    m_outData_.clear();
}

/**
* @brief: init, inherited from hiaiengine lib
*/
HIAI_StatusT ObjectInferenceEngine::Init(const hiai::AIConfig& config,
                                         const std::vector<hiai::AIModelDescription>& model_desc) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] start init!");
    hiai::AIStatus ret = hiai::SUCCESS;

    if (nullptr == ai_model_manager_) {
        ai_model_manager_ = std::make_shared<hiai::AIModelManager>();
    }

    std::vector<hiai::AIModelDescription> model_desc_vec;
    hiai::AIModelDescription model_desc_;

    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        if (item.name() == "model_path") {
            std::string model_path = item.value();
            if (model_path.empty()) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] model_path not exist!");
                return HIAI_ERROR;
            }
            model_desc_.set_path(model_path);
            int modelNameStartPos = model_path.find_last_of("/\\");
            int modelNameEndPos = model_path.find_last_of(".");
            if (std::string::npos != modelNameStartPos && std::string::npos != modelNameEndPos &&
                modelNameEndPos > modelNameStartPos) {
                modelName_ = model_path.substr(modelNameStartPos + 1,
                                               modelNameEndPos - modelNameStartPos - 1);
            }
        } else if (item.name() == "passcode") {
            std::string passcode = item.value();
            model_desc_.set_key(passcode);
        } else if (item.name() == "batch_size") {
            std::stringstream ss(item.value());
            ss >> batch_size;
            if (batch_size <= 0) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] batch_size <= 0!");
                return HIAI_ERROR;
            }
        }
    }
    model_desc_.set_name(modelName_);
    model_desc_vec.push_back(model_desc_);
    ret = ai_model_manager_->Init(config, model_desc_vec);
    if (hiai::SUCCESS != ret) {
        return HIAI_ERROR;
    }

    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] end init!");
    return HIAI_OK;
}

/**
* @brief: handle the exceptions when the dataset batch failed
* @in: error_msg: the error message
*/
void ObjectInferenceEngine::HandleExceptions(std::string error_msg) {
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, error_msg.c_str());
    tran_data->status = false;
    tran_data->msg = error_msg;
    // send null to next node to avoid blocking when to encounter abnomal situation.
    auto ret = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tran_data));
    if (ret != HIAI_OK) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] send data failed!");
    }
}

/**
* @brief: send sentinel image to inform the graph to destroy
*/
HIAI_StatusT ObjectInferenceEngine::SendSentinelImage() {
    tran_data->status = true;
    tran_data->msg = "sentinel Image";
    tran_data->b_info = image_handle->b_info;
    HIAI_StatusT hiai_ret = HIAI_OK;
    do {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] sentinel image, process success!");
        hiai_ret = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tran_data));
        if (HIAI_OK != hiai_ret) {
            if (HIAI_ENGINE_NULL_POINTER == hiai_ret || HIAI_HDC_SEND_MSG_ERROR == hiai_ret ||
                HIAI_HDC_SEND_ERROR == hiai_ret || HIAI_GRAPH_SRC_PORT_NOT_EXIST == hiai_ret ||
                HIAI_GRAPH_ENGINE_NOT_EXIST == hiai_ret) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                                "[MindInferenceEngine_1] SendData error[%d], break.", hiai_ret);
                break;
            }
            HIAI_ENGINE_LOG(
                HIAI_IDE_INFO,
                "[MindInferenceEngine_1] SendData return value[%d] not OK, sleep 200ms", hiai_ret);
            usleep(SEND_DATA_INTERVAL_MS);
        }
    } while (HIAI_OK != hiai_ret);
    return hiai_ret;
}

/**
* @brief: prepare the data buffer for image information
* @in: input_buffer: buffer pointer
* @in: image_number: total number of received images
* @in: batch_begin: the index of the first image of each batch
* @in: image_size: size of each image
* @return: HIAI_StatusT
*/
HIAI_StatusT ObjectInferenceEngine::PrepareInputBuffer(uint8_t* input_buffer,
                                                       const int image_number,
                                                       const int batch_begin,
                                                       const int image_size) {
    // 1.prepare input buffer for each batch
    // the loop for each image
    if (input_buffer == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] ERROR, input_buffer is nullptr");
        return HIAI_ERROR;
    }
    for (int j = 0; j < batch_size; j++) {
        if (batch_begin + j < image_number) {
            if (memcpy_s(input_buffer + j * image_size, image_size,
                         image_handle->v_img[batch_begin + j].img.data.get(), image_size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                                "[MindInferenceEngine_1] ERROR, copy image buffer failed");
                return HIAI_ERROR;
            }
        } else {
            if (memset_s(input_buffer + j * image_size, image_size, static_cast<char>(0),
                         image_size)) {
                HIAI_ENGINE_LOG(
                    HIAI_IDE_ERROR,
                    "[MindInferenceEngine_1] ERROR, batch padding for image data failed");
                return HIAI_ERROR;
            }
        }
    }
    return HIAI_OK;
}

/**
* @brief: prepare the data buffer for image information
* @in: input_buffer2: buffer pointer
* @in: image_number: total number of received images
* @in: batch_begin: the index of the first image of each batch
* @in: multi_input_2: the second input received from the previous engine
* @return: HIAI_StatusT
*/
HIAI_StatusT ObjectInferenceEngine::PrepareInforInput(
    uint8_t* input_buffer2, const int image_number, const int batch_begin,
    std::shared_ptr<hiai::BatchRawDataBuffer> multi_input_2) {
    int each_size;
    // the loop for each info
    for (int j = 0; j < batch_size; j++) {
        if (batch_begin + j < image_number) {
            hiai::RawDataBuffer _input_arg_2 = multi_input_2->v_info[batch_begin + j];
            each_size = _input_arg_2.len_of_byte;
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] info each input size: %d",
                            each_size);
            if (memcpy_s(input_buffer2 + j * each_size, each_size, _input_arg_2.data.get(),
                         each_size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                                "[MindInferenceEngine_1] ERROR, copy info buffer failed");
                return HIAI_ERROR;
            }
        } else {
            float info_tmp[3] = {0.0, 0.0, 0.0};
            each_size = sizeof(info_tmp);
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] info padding size: %d",
                            each_size);
            if (memcpy_s(input_buffer2 + j * each_size, each_size, info_tmp, each_size)) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                                "[MindInferenceEngine_1] ERROR, padding info buffer failed");
                return HIAI_ERROR;
            }
        }
    }
    return HIAI_OK;
}

/**
* @brief: call ai model manager to do the prediction
* @return: HIAI_StatusT
*/
HIAI_StatusT ObjectInferenceEngine::Predict() {
    HIAI_StatusT ret = HIAI_OK;

    std::vector<hiai::TensorDimension> inputTensorVec;
    std::vector<hiai::TensorDimension> outputTensorVec;
    ret = ai_model_manager_->GetModelIOTensorDim(modelName_, inputTensorVec, outputTensorVec);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(this, HIAI_IDE_ERROR, "hiai ai model manager init fail");
        return HIAI_ERROR;
    }

    // pre malloc OutData
    HIAI_StatusT hiai_ret = HIAI_OK;
    for (uint32_t index = 0; index < outputTensorVec.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        uint8_t* buf = nullptr;
        hiai_ret = hiai::HIAIMemory::HIAI_DMalloc(outputTensorVec[index].size, (void*&)buf, 1000);
        if (hiai_ret != HIAI_OK || buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] HIAI_DMalloc failed.");
            ClearOutData();
            return HIAI_ERROR;
        }
        m_outData_.push_back(buf);
        std::shared_ptr<hiai::IAITensor> outputTensor =
            hiai::AITensorFactory::GetInstance()->CreateTensor(outputTensorDesc, buf,
                                                               outputTensorVec[index].size);
        shared_ptr<hiai::AINeuralNetworkBuffer> nn_tensor =
            static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensor);
        nn_tensor->SetName(outputTensorVec[index].name);
        output_data_vec.push_back(outputTensor);
    }

    // put buffer to FrameWork directly, InputSize has only one
    hiai::AITensorDescription inputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
    for (int i = 0; i < predict_input_data_.size(); i++) {
        std::map<uint8_t*, int> tmp = predict_input_data_[i];
        for (std::map<uint8_t*, int>::iterator it = tmp.begin(); it != tmp.end(); ++it) {
            shared_ptr<hiai::IAITensor> inputTensor =
                hiai::AITensorFactory::GetInstance()->CreateTensor(
                    inputTensorDesc, reinterpret_cast<void*>(it->first), it->second);
            input_data_vec.push_back(inputTensor);  // AIModelManager push input data
        }
    }

    hiai::AIContext ai_context;
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] ai_model_manager_->Process start!");
    ret = ai_model_manager_->Process(ai_context, input_data_vec, output_data_vec, 0);
    if (hiai::SUCCESS != ret) {
        ClearOutData();
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] ai_model_manager Process failed");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

/**
* @brief: set the tran_data with the result of this batch
* @in: index of the begin of this batch
* @return: HIAI_StatusT
*/
HIAI_StatusT ObjectInferenceEngine::SetOutputStruct(const int batch_begin) {
    for (int i = 0; i < output_data_vec.size(); ++i) {
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] build: %d", i);
        std::shared_ptr<hiai::AINeuralNetworkBuffer> result_tensor =
            std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(output_data_vec[i]);
        auto tensor_size = result_tensor->GetSize();
        if (memcpy_s(
                tran_data->output_data_vec[i].data.get() + batch_begin / batch_size * tensor_size,
                tensor_size, result_tensor->GetBuffer(), tensor_size)) {
            return HIAI_ERROR;
        }
        HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] build: %d, number: %d",
                        tensor_size, batch_begin / batch_size * tensor_size);
    }
    return HIAI_OK;
}

/**
* @brief: send the predicted result for one batch
*/
void ObjectInferenceEngine::SendResult() {
    HIAI_StatusT hiai_ret = HIAI_OK;
    do {
        hiai_ret = SendData(0, "EngineTransT", std::static_pointer_cast<void>(tran_data));
        if (HIAI_QUEUE_FULL == hiai_ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] queue full, sleep 200ms");
            usleep(200000);
        }
    } while (hiai_ret == HIAI_QUEUE_FULL);
    if (HIAI_OK != hiai_ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] SendData failed! error code: %d",
                        hiai_ret);
    }
}

/**
* @brief: set the frame ID as -1 to indicate this model batch failed
* @in: index of the begin of this batch
*/
void ObjectInferenceEngine::HandleModelBatchFailure(const int batch_begin, const int image_number) {
    for (int i = 0; i < batch_size; i++) {
        if (batch_begin + i < image_number) {
            tran_data->b_info.frame_ID[i + batch_begin] = -1;
        }
    }
}

/**
* @ingroup hiaiengine
* @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
* @[in]: Define an input port, an output port,
*        And the Engine is registered, its called "HIAIMultiEngineExample"
*/
HIAI_IMPL_ENGINE_PROCESS("ObjectInferenceEngine", ObjectInferenceEngine, INPUT_SIZE) {
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] start process!");
    hiai::AIStatus ret = hiai::SUCCESS;
    HIAI_StatusT hiai_ret = HIAI_OK;
    std::lock_guard<std::mutex> lk(memoryRecursiveMutex_);
    if (tran_data == nullptr) {
        tran_data = std::make_shared<EngineTransT>();
    }
    // 1.PreProcess:Framework input data
    if (nullptr != arg0) {
        std::shared_ptr<BatchImageParaWithScaleT> dataInput =
            std::static_pointer_cast<BatchImageParaWithScaleT>(arg0);

        if (!isSentinelImage(dataInput)) {
            if (m_data_input_in_ != nullptr) {
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
                    HIAI_IDE_INFO, "[MindInferenceEngine_1] Wait for other %d batch image info!",
                    (m_data_input_in_->b_info.batch_size - m_data_input_in_->v_img.size()));
                return HIAI_OK;
            }
            input_que_.PushData(0, m_data_input_in_);
            m_data_input_in_ = nullptr;
        } else {
            input_que_.PushData(0, arg0);
        }
    }

    image_handle = nullptr;

#if INPUT_SIZE < 3
    if (!input_que_.PopAllData(image_handle)) {
        HandleExceptions("[MindInferenceEngine_1] fail to PopAllData");
        return HIAI_ERROR;
    }
#endif

#if (INPUT_SIZE == 3)
    DEFINE_MULTI_INPUT_ARGS_POP(3);
#endif

#if (INPUT_SIZE == 4)
    DEFINE_MULTI_INPUT_ARGS_POP(4);
#endif

#if (INPUT_SIZE == 5)
    DEFINE_MULTI_INPUT_ARGS_POP(5);
#endif

#if (INPUT_SIZE == 6)
    DEFINE_MULTI_INPUT_ARGS_POP(6);
#endif

#if (INPUT_SIZE == 7)
    DEFINE_MULTI_INPUT_ARGS_POP(7);
#endif

#if (INPUT_SIZE == 8)
    DEFINE_MULTI_INPUT_ARGS_POP(8);
#endif

#if (INPUT_SIZE == 9)
    DEFINE_MULTI_INPUT_ARGS_POP(9);
#endif

#if (INPUT_SIZE == 10)
    DEFINE_MULTI_INPUT_ARGS_POP(10);
#endif

    if (nullptr == image_handle) {
        HandleExceptions("[MindInferenceEngine_1] Image_handle is nullptr");
        return HIAI_ERROR;
    }
    // add sentinel image for showing this data in dataset are all sended, this is last step.
    if (isSentinelImage(image_handle)) {
        return SendSentinelImage();
    }

    int image_number = image_handle->v_img.size();
#if (INPUT_SIZE == 3)
    if (nullptr == _multi_input_2) {
        HandleExceptions("[MindInferenceEngine_1] fail to process invalid message");
        return HIAI_ERROR;
    }
    int info_number = _multi_input_2->v_info.size();
    if (info_number != image_number) {
        HandleExceptions(
            "[MindInferenceEngine_1] ERROR the number of image data and information data doesn't "
            "match!");
    }
    int _input_buffer2_size = sizeof(float) * IMAGE_INFO_DATA_NUM * batch_size;
    uint8_t* _input_buffer2 = nullptr;
    hiai_ret = hiai::HIAIMemory::HIAI_DMalloc(_input_buffer2_size, (void*&)_input_buffer2, 1000);
    if (hiai_ret != HIAI_OK || _input_buffer2 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[MindInferenceEngine_1] HIAI_DMalloc _input_buffer2 failed.");
        return HIAI_ERROR;
    }
#endif

    int image_size = image_handle->v_img[0].img.size * sizeof(uint8_t);
    int _input_buffer1_size = image_size * batch_size;
    if (_input_buffer1_size <= 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] _input_buffer1_size <= 0");
        return HIAI_ERROR;
    }
    uint8_t* _input_buffer1 = nullptr;
    hiai_ret = hiai::HIAIMemory::HIAI_DMalloc(_input_buffer1_size, (void*&)_input_buffer1, 1000);
    if (hiai_ret != HIAI_OK || _input_buffer1 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[MindInferenceEngine_1] HIAI_DMalloc _input_buffer1 failed.");
#if (INPUT_SIZE == 3)
        hiai::HIAIMemory::HIAI_DFree(_input_buffer2);
        _input_buffer2 = nullptr;
#endif
        return HIAI_ERROR;
    }

    int cnt_batch = image_handle->b_info.batch_size / batch_size;
    if (image_handle->b_info.batch_size % batch_size != 0) {
        cnt_batch++;
    }

    tran_data->b_info = image_handle->b_info;
    tran_data->v_img = image_handle->v_img;
    tran_data->status = true;
    tran_data->b_info.max_batch_size = cnt_batch * batch_size;

    // the loop for each batch
    for (int i = 0; i < image_number; i += batch_size) {
        predict_input_data_.clear();
        // 1.prepare input buffer for each batch
        if (HIAI_OK != PrepareInputBuffer(_input_buffer1, image_number, i, image_size)) {
            HandleModelBatchFailure(i, image_number);
            continue;
        }
        std::map<uint8_t*, int> input1;
        input1.insert(std::make_pair(_input_buffer1, _input_buffer1_size));
        predict_input_data_.push_back(input1);
#if (INPUT_SIZE == 2)
        DEFINE_MULTI_INPUT_ARGS(2);
#endif

#if (INPUT_SIZE == 3)
        // int each_size;
        if (HIAI_OK != PrepareInforInput(_input_buffer2, image_number, i, _multi_input_2)) {
            // HandleExceptions("[MindInferenceEngine_1] batch " +
            // std::to_string(tran_data->b_info.batch_ID) + " failed!");
            HandleModelBatchFailure(i, image_number);
            continue;
            // return HIAI_ERROR;
        }
        std::map<uint8_t*, int> input2;
        input2.insert(std::make_pair(_input_buffer2, _input_buffer2_size));
        predict_input_data_.push_back(input2);
        DEFINE_MULTI_INPUT_ARGS(3);
#endif

        // 2.Call Process, Predict
        input_data_vec.clear();
        if (HIAI_OK != Predict()) {
            output_data_vec.clear();
            HandleModelBatchFailure(i, image_number);
            continue;
        }
        // init the output buffer for one dataset batch(might be multiple model batches)
        if (tran_data->output_data_vec.empty()) {
            tran_data->size = output_data_vec.size();
            HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                            "[MindInferenceEngine_1] alloc memory for dataset batch, number of "
                            "outputs of the network: %d",
                            output_data_vec.size());
            for (int i = 0; i < output_data_vec.size(); i++) {
                OutputT out;
                std::shared_ptr<hiai::AINeuralNetworkBuffer> result_tensor =
                    std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(output_data_vec[i]);
                int buffer_size = result_tensor->GetSize();
                out.name = result_tensor->GetName();
                out.size = buffer_size * cnt_batch;
                if (out.size <= 0) {
                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MindInferenceEngine_1] out.size <= 0");
                    hiai::HIAIMemory::HIAI_DFree(_input_buffer1);
                    _input_buffer1 = nullptr;
#if (INPUT_SIZE == 3)
                    hiai::HIAIMemory::HIAI_DFree(_input_buffer2);
                    _input_buffer2 = nullptr;
#endif
                    ClearOutData();
                    return HIAI_ERROR;
                }
                u_int8_t* ptr = nullptr;
                try {
                    ptr = new u_int8_t[out.size];
                } catch (const std::bad_alloc& e) {
                    hiai::HIAIMemory::HIAI_DFree(_input_buffer1);
                    _input_buffer1 = nullptr;
#if (INPUT_SIZE == 3)
                    hiai::HIAIMemory::HIAI_DFree(_input_buffer2);
                    _input_buffer2 = nullptr;
#endif
                    ClearOutData();
                    return HIAI_ERROR;
                }
                out.data.reset(ptr);
                tran_data->output_data_vec.push_back(out);
                HIAI_ENGINE_LOG(HIAI_IDE_INFO,
                                "[MindInferenceEngine_1] cnt_model_batch: %d, image number: %d!",
                                cnt_batch, image_number);
            }
        }

        // 3.set the tran_data with the result of this batch
        if (HIAI_OK != SetOutputStruct(i)) {
            ClearOutData();
            output_data_vec.clear();
            HandleModelBatchFailure(i, image_number);
            continue;
        }
        output_data_vec.clear();
    }
    SendResult();
    // 6. release sources
    hiai::HIAIMemory::HIAI_DFree(_input_buffer1);
    _input_buffer1 = nullptr;
#if (INPUT_SIZE == 3)
    hiai::HIAIMemory::HIAI_DFree(_input_buffer2);
    _input_buffer2 = nullptr;
#endif
    ClearOutData();
    tran_data = nullptr;
    HIAI_ENGINE_LOG(HIAI_IDE_INFO, "[MindInferenceEngine_1] end process!");
    return HIAI_OK;
}
