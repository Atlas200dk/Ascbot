/*
 * Copyright © 2018-2019 xxx Software Technology Co., Ltd.
 * All rights reserved.
 */

#ifndef _HICOLLISIONAVOIDANCE_H_
#define _HICOLLISIONAVOIDANCE_H_

#include <functional>
#include <map>
#include <string>
#include <vector>
#include "HiStruct.h"

/**
 * @brief It provides collision avoidance detector algorithms interfaces.
 * */
namespace hi {

/// The recognition enum
/**
 * @brief The Collision avoidance enum.
 * */
enum CollisionStatus {
    DANGER0 = 0,  // danger0 status
    SAFE0 = 1,    // safe0 status
};

/**
 * @brief collision avoidance result structure
 * */
struct CollisionData {
    CollisionStatus status = CollisionStatus::SAFE0;
    /// Confidence of all the possible prediction results
    /// and all those are normalized in [0.0f, 1.0f]
    std::map<CollisionStatus, float> status_map;
    int64_t timestamp = 0;  // The input image timestamp.
};

/// Callback function definition
using ca_callback_t = std::function<RDC_STATE(const std::vector<CollisionData>&)>;

/** @brief Road following detector.
 *
 * */
class HiCollisionAvoidance {
public:
    /** @brief constructor
     * */
    EXPORT_API explicit HiCollisionAvoidance();

    /** @brief deconstructor
     * */
    EXPORT_API virtual ~HiCollisionAvoidance();

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
    EXPORT_API void setCallbackFunction(ca_callback_t func);

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

#endif  // _HICOLLISIONAVOIDANCE_H_
