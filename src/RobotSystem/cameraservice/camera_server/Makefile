LOCAL_DIR      := $(patsubst %,%,$(CURDIR))

# set DDK_HOME default value
ifndef DDK_HOME
$(error "Can not find DDK_HOME env, please set it in environment!.")
endif

CC := aarch64-linux-gnu-g++
CPP := aarch64-linux-gnu-g++

LOCAL_MODULE_NAME := camera_server
CC_FLAGS := -std=c++11 -Wall

local_src_files := \
	$(LOCAL_DIR)/src/main.cpp \
	$(LOCAL_DIR)/src/stream_process.cpp \
	$(LOCAL_DIR)/src/camera.cpp \
	$(LOCAL_DIR)/src/CircularQueue.cpp

local_inc_dirs := \
	$(LOCAL_DIR)/include \
	$(LOCAL_DIR)/include/ai \
	$(LOCAL_DIR)/include/ascenddk/ascend_ezdvpp \
	$(DDK_HOME)/include/inc \
	$(DDK_HOME)/include/third_party/protobuf/include \
	$(DDK_HOME)/include/third_party/cereal/include \
	$(DDK_HOME)/include/libc_sec/include \
	$(DDK_HOME)/include/inc  \
	$(DDK_HOME)/include/inc/custom


local_shared_libs := \
	c_sec \
	pthread \
	rt  \
	protobuf \
	slog \
	media_mini \
	Dvpp_api \
	Dvpp_jpeg_decoder \
	Dvpp_jpeg_encoder \
	Dvpp_vpc \
	OMX_hisi_video_decoder \
	OMX_hisi_video_encoder \
	Dvpp_png_decoder \
	ascend_ezdvpp \
		\
	hiai_server \
	hiai_common \
	matrixdaemon \
	ome \
	fmk_common \
	drvdevdrv \
	drvaicpu \
	drvhdc \
	drvhdc_host \
	drvtcp \
	mmpa \
	crypto \
	runtime \
	devmm \
	teec \
	cce \
	cce_aicpudev \
	cce_aicore \
	aicpu_engine  \
	profilerclient \
	dl \
	RFInferenceEngine \
    RFPostProcess \
    RFPreProcess \
	CollisionInferenceEngine \
    CollisionPostProcess \
    CollisionPreProcess \
	ObjectInferenceEngine \
    ObjectPostProcess \
    ObjectPreProcess \
	RoadObjectInferenceEngine \
    RoadObjectPostProcess \
    RoadObjectPreProcess


#Q := @
FULL_SRC_FILES        := $(local_src_files)
FULL_INC_DIRS         := $(foreach inc_dir, $(local_inc_dirs), -I$(inc_dir))
SHARED_LIBRARIES      := $(foreach shared_lib, $(local_shared_libs), -l$(shared_lib))

LOCAL_OBJ_PATH        := $(LOCAL_DIR)/out
LOCAL_LIBRARY         := $(LOCAL_OBJ_PATH)/$(LOCAL_MODULE_NAME)
FULL_CPP_SRCS         := $(filter %.cpp,$(FULL_SRC_FILES))
FULL_CPP_OBJS         := $(patsubst $(LOCAL_DIR)/%.cpp,$(LOCAL_OBJ_PATH)/%.o, $(FULL_CPP_SRCS))

#presenteragent in host running side
#host and device runing side is the same in atlas dk 200
#ascendcamera only support atlas 200
LNK_FLAGS := \
        -Wl,-rpath-link=$(DDK_HOME)/host/lib \
	-Wl,-rpath-link=$(DDK_HOME)/device/lib/ \
	-L$(DDK_HOME)/device/lib \
	-L$(DDK_HOME)/uihost/lib \
	-L$(HOME)/ascend_ddk/device/lib \
    -L$(HOME)/ascend_ddk/host/lib \
	-L$(LOCAL_DIR)/libs \
	$(SHARED_LIBRARIES)

all: do_pre_build do_build

do_pre_build:
	$(Q)echo - do [$@]
	$(Q)mkdir -p $(LOCAL_OBJ_PATH)

do_build: $(LOCAL_LIBRARY) | do_pre_build
	$(Q)echo - do [$@]

$(LOCAL_LIBRARY): $(FULL_CPP_OBJS) | do_pre_build
	$(Q)echo [LD] $@
	$(Q)$(CPP) $(CC_FLAGS) -o $(LOCAL_LIBRARY) $(FULL_CPP_OBJS)   -Wl,--whole-archive  -Wl,--no-whole-archive -Wl,--start-group  -Wl,--end-group $(LNK_FLAGS) -Wl,-rpath='$$ORIGIN/../../../ascend_lib'

$(FULL_CPP_OBJS): $(LOCAL_OBJ_PATH)/%.o : $(LOCAL_DIR)/%.cpp  | do_pre_build
	$(Q)echo [CC] $@
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CPP) $(CC_FLAGS) $(FULL_INC_DIRS) -c  -fstack-protector-all $< -o $@

clean:
	rm -rf $(LOCAL_DIR)/out
