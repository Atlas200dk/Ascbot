/*
 * Copyright © 2018-2019 Thunder Software Technology Co., Ltd.
 * All rights reserved.
 */

#ifndef _HISTRUCT_H_
#define _HISTRUCT_H_

#include <stdint.h>

#ifndef EXPORT_API
#define EXPORT_API __attribute__((visibility("default")))
#endif

using RDC_STATE = int;

/**
 *  @defgroup ConstantValues Constant
 *  @{ */
/**
 *  @name RDC_STATE
 *  @brief Operating state
 *  @{ */
constexpr int STATE_RECYCLE(0x01);

constexpr int STATE_SUCCESS(0x00);
constexpr int STATE_INVALID_VALUE(-0x01);
constexpr int STATE_INVALID_OPERATION(-0x02);
constexpr int STATE_INCOMPLETE_INITIALIZATION(-0x03);
constexpr int STATE_DEBUG_MODE_STATE(-0x04);
constexpr int STATE_OPERATION_FAILED(-0x05);
constexpr int STATE_TERMINATED(-0x06);
/** @} */

/**
 *  @name IMAGE_TYPE
 *  @brief Operating state
 *  @{ */
constexpr int TYPE_BGR_U8(0x00);
constexpr int TYPE_RGB_U8(0x01);
constexpr int TYPE_YUV_NV12(0x02);
/** @} */
/** @} */

/**
 * @brief Basic data structure group.
 */
namespace hi {

/**
 *  @brief Package of image buffer.\n
 */
class HIImgData {
public:
    /**
     *  @brief Default constructor.
     *  There are various constructors to allocate hi::HIImgData.
     *  Using default constructor to apply an empty image object.
     */
    EXPORT_API HIImgData();

    /**
     *  @brief Package an image using an allocated memory or not.
     *  @param[in] width  Image width.
     *  @param[in] height Image height.
     *  @param[in] type   Only TYPE_XXX_U8 can be accepted.
     *  @param[in] data   Buffer head address.
     *  @todo For wide-bit RGBD camera, TYPE_XXX_F32 will be supported in the future.
     *  @param[in] stride Number of Bytes each image row occupies.
     */
    EXPORT_API HIImgData(int32_t width, int32_t height, int32_t type = TYPE_BGR_U8,
                         uint8_t* data = nullptr, int32_t stride = -1, int64_t timestamp = 0);

    /**
     *  @brief Copy constructor.
     *  Implemented by deep copy command internally.
     */
    EXPORT_API HIImgData(const HIImgData& img);

    /**
     *  @brief HIImgData is just a package of image.
     *  It does not have any responsible for buffer management and cleanup.
     */
    EXPORT_API virtual ~HIImgData();

    /**
     *  @brief Get the head address.
     *  @retval nullptr The HIImgData is empty
     */
    EXPORT_API uint8_t* data() const;

    /**
     *  @brief Get the width of image.
     *  @retval 0 The HIImgData is empty
     */
    EXPORT_API int32_t width() const;

    /**
     *  @brief Get the height of image.
     *  @retval 0 The HIImgData is empty
     */
    EXPORT_API int32_t height() const;

    /**
     *  @brief Get number of Bytes each image row occupies.
     *  @retval 0 The HIImgData is empty
     */
    EXPORT_API int32_t stride() const;

    /**
     *  @brief Get image channels.
     *  @retval 0 The HIImgData is empty
     */
    EXPORT_API int32_t channels() const;

    /**
     * @brief Get image type.
     * */
    EXPORT_API int32_t type() const;

    /**
     * @brief Get image timestamp.
     * */
    EXPORT_API int64_t timestamp() const;

    /**
     *  @brief Check if the image is empty.
     *  @retval true  The HIImgData is empty.
     *  @retval false The HIImgData is not empty.
     */
    EXPORT_API bool emtpy() const;

    /**
     *  @brief Shallow copy command.
     */
    EXPORT_API void operator=(const HIImgData& t);

private:
    /// Context
    void* impl = nullptr;
};

/// Base definition for point
template <typename T>
struct HIPoint_T {
    HIPoint_T() {
    }
    HIPoint_T(T x, T y) {
        this->x = x;
        this->y = y;
    }
    float x = 0.0f;
    float y = 0.0f;
};
using HIPoint = HIPoint_T<float>;

/// Base definition for (rectangle) region
template <typename T>
struct HIRect_T {
    HIRect_T() {
    }
    HIRect_T(T x, T y, T width, T height) {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
    T x = 0.0f;
    T y = 0.0f;
    T width = 0.0f;
    T height = 0.0f;
    HIPoint_T<T> centre() {
        return {x + width / 2, y + height / 2};
    }
};
using HIRect = HIRect_T<float>;
}  // namespace hi

#endif  // _HISTRUCT_H_
