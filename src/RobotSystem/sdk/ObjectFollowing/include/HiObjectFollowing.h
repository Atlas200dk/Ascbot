/*
 * Copyright © 2018-2019 xxx Software Technology Co., Ltd.
 * All rights reserved.
 */

#ifndef _HIOBJECTFOLLOWING_H_
#define _HIOBJECTFOLLOWING_H_

#include <functional>
#include <map>
#include <string>
#include <vector>
#include "HiStruct.h"

/**
 * @brief It provides collision avoidance detector algorithms interfaces.
 * */
namespace hi {

/**
 * @brief Object following result structure
 * The types of objects that support detection are: {person, cup, bottle, box, phone}
 * */
struct ObjectData {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    int lable = 0;
    float confidence = 0.0f;
    int64_t timestamp = 0;  // The input image timestamp.
};

/// Callback function definition
using of_callback_t = std::function<RDC_STATE(const std::vector<ObjectData>&)>;

/** @brief Road following detector.
 *
 * */
class HiObjectFollowing {
public:
    /** @brief constructor
     * */
    EXPORT_API explicit HiObjectFollowing();

    /** @brief deconstructor
     * */
    EXPORT_API virtual ~HiObjectFollowing();

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
    EXPORT_API void setCallbackFunction(of_callback_t func);

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

    /**
     *  @brief Set confidence threshold
     *  @param[in] value is confidence threshold
     *  @return[out] void
     */
    EXPORT_API void setConfidence(float value);

private:
    // @brief Context
    void* impl = nullptr;
};
}  // namespace hi

#endif  // _HIOBJECTFOLLOWING_H_
