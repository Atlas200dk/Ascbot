/*
 * Copyright © 2018-2019 xxx Software Technology Co., Ltd.
 * All rights reserved.
 */

#ifndef INCLUDE_HIROADFOLLOWING_H_
#define INCLUDE_HIROADFOLLOWING_H_

#include <functional>
#include <string>
#include <vector>
#include "HiStruct.h"

/**
 * @brief It provides road following detector algorithms interfaces.
 * */
namespace hi {
/**
 * @brief road following result structure
 * */
struct RFData {
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;  // Range of values [-90,90]
    int64_t timestamp = 0;  // The input image timestamp.
    // debug info
    float raw_x = 0.0f;
    float raw_y = 0.0f;
};

/// Callback function definition
using rf_callback_t = std::function<RDC_STATE(const std::vector<RFData>&)>;

/** @brief Road following detector.
 *
 * */
class HiRoadFollowing {
public:
    /** @brief constructor
     * */
    EXPORT_API explicit HiRoadFollowing();

    /** @brief deconstructor
     * */
    EXPORT_API virtual ~HiRoadFollowing();

    /** @brief Initialize road following detector environment.
     * @param[in] config_path The configure file consists of graph.config, and etc.
     * @retval true Initialization has been successful.
     * @retval false Errors occurred. Check the log please.
     * */
    EXPORT_API bool init(const std::string& config_path);

    /** @brief Free relevant resources.
     * @retval true De-initialization has been successful.
     * @retval false Errors occurred. Check the log please.
     * */
    EXPORT_API bool deInit();

    /**
     * @brief Register a call-back function to predict net road follow location.
     * @param[in] func The callback function.
     * */
    EXPORT_API void setCallbackFunction(rf_callback_t func);

    /** @brief Core method for road predict.
     * @param[in] frame It required by be RGB color sequence.s
     * @retval true detect successfully.
     * @retval false Errors occurred. Check the log please.
     * */
    EXPORT_API bool predict(const HIImgData& frame);

    /** @brief Check initialization state
     * @retval true Initialization has been successful.
     * @retval false Not initialized or some errors occurred. Check the log please.
     * */
    EXPORT_API bool isInit() const;

private:
    // @brief Context
    void* impl = nullptr;
};
}  // namespace hi

#endif  // INCLUDE_HIROADFOLLOWING_H_
