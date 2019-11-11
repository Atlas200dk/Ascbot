/*
 * Copyright © 2018-2019 Thunder Software Technology Co., Ltd.
 * All rights reserved.
 */

#include "HiStruct.h"

namespace hi {
class ImgInner {
public:
    ImgInner()
        : width_(0),
          height_(0),
          type_(TYPE_YUV_NV12),
          channel_(0),
          stride_(0),
          data_(nullptr),
          timestamp_(0) {
    }
    ImgInner(int width, int height, int type, void* data, int stride, int64_t timestamp) {
        width_ = width;
        height_ = height;
        type_ = type;
        channel_ = 3;
        data_ = data;
        if (stride == 0) {
            stride_ = width;
        } else {
            stride_ = stride;
        }
        timestamp_ = timestamp;
    }
    ~ImgInner() {
    }

    void* data() {
        return data_;
    }
    int width() {
        return width_;
    }
    int height() {
        return height_;
    }
    int stride() {
        return stride_;
    }
    int channel() {
        return channel_;
    }
    int type() {
        return type_;
    }
    int64_t timestamp() {
        return timestamp_;
    }
    bool empty() {
        if ((nullptr == data_) || (width_ == 0) || (height_ == 0)) {
            return true;
        } else {
            return false;
        }
    }

private:
    int width_ = 0;
    int height_ = 0;
    int type_ = 0;
    float channel_ = 0;
    int stride_ = 0;
    int64_t timestamp_ = 0;
    void* data_ = nullptr;
};
}  // namespace hi

hi::HIImgData::HIImgData() {
    impl = new ImgInner();
}

hi::HIImgData::HIImgData(int32_t width, int32_t height, int32_t type, uint8_t* data,
                         int32_t stride, int64_t timestamp) {
    impl = new ImgInner(width, height, type, data, stride, timestamp);
}

hi::HIImgData::HIImgData(const HIImgData& img) {
    delete reinterpret_cast<ImgInner*>(impl);
    impl = new ImgInner(*(reinterpret_cast<ImgInner*>(img.impl)));
}

void hi::HIImgData::operator=(const HIImgData& t) {
    *(reinterpret_cast<ImgInner*>(impl)) = *(reinterpret_cast<ImgInner*>(t.impl));
}

hi::HIImgData::~HIImgData() {
    delete reinterpret_cast<ImgInner*>(impl);
}

uint8_t* hi::HIImgData::data() const {
    return (reinterpret_cast<uint8_t*>((reinterpret_cast<ImgInner*>(impl))->data()));
}

int32_t hi::HIImgData::width() const {
    return (reinterpret_cast<ImgInner*>(impl))->width();
}

int32_t hi::HIImgData::height() const {
    return (reinterpret_cast<ImgInner*>(impl))->height();
}

int32_t hi::HIImgData::stride() const {
    return (reinterpret_cast<ImgInner*>(impl))->stride();
}

int32_t hi::HIImgData::channels() const {
    return (reinterpret_cast<ImgInner*>(impl))->channel();
}

bool hi::HIImgData::emtpy() const {
    return (reinterpret_cast<ImgInner*>(impl))->empty();
}

int32_t hi::HIImgData::type() const {
    return (reinterpret_cast<ImgInner*>(impl))->type();
}

int64_t hi::HIImgData::timestamp() const {
    return (reinterpret_cast<ImgInner*>(impl))->timestamp();
}
