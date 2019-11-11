all : libRoadObjectPostProcess.so
#HOST COMPILER		
DDK_HOME = /home/yaojt0222/tools/che/ddk/ddk
CC := aarch64-linux-gnu-g++
#no space at the end of the line, or it will be involved into the directory name, which will cause error
LOCAL_DIR  := ./

SRC_DIR = $(LOCAL_DIR)
BUILD_DIR = tmp
OUT_DIR = ../out
OBJ_DIR = $(BUILD_DIR)/obj
DEPS_DIR  = $(BUILD_DIR)/deps

#add other header file path here
INC_DIR = \
	-I$(SRC_DIR) \
	-I$(DDK_HOME)/include/inc \
	-I$(DDK_HOME)/include/inc/custom \
	-I$(DDK_HOME)/include/third_party/opencv/include \
	-I$(DDK_HOME)/include/third_party/protobuf/include \
	-I$(DDK_HOME)/include/third_party/cereal/include \
	-I$(DDK_HOME)/include/libc_sec/include \
	-I../common/include \
	-I$(SRC_DIR)/../include

#add compilation arguments here
CC_FLAGS := $(INC_DIR) -std=c++11 -fPIC -DIS_OI
LNK_FLAGS := \
	-L$(DDK_HOME)/host/lib/ \
	-lmedia_mini \
	-shared


#recursively traverse 3-level subdirectory
DIRS := $(shell find $(SRC_DIR) -maxdepth 3 -type d)
CUSTOM_DIRS := $(shell find $(SRC_DIR) -maxdepth 3 -type d)

#add each subdirectory into searching path 
VPATH = $(DIRS):../common/src

#find all cpp files in subdirectories of src_dir
SOURCES  = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.cpp))
SOURCES += ../common/src/BatchImageParaWithScale.cpp
CUSTOM_SOURCES  = $(foreach dir, $(CUSTOM_DIRS), $(wildcard $(dir)/*.cpp))
CUSTOM_SOURCES += ../common/src/BatchImageParaWithScale.cpp
OBJS   = $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp,%.o,$(notdir $(SOURCES))))
OBJS_customop = $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp,%.o,$(notdir $(CUSTOM_SOURCES))))
OBJS_no_customop := $(filter-out $(OBJS_customop), $(OBJS))
DEPS  = $(addprefix $(DEPS_DIR)/, $(patsubst %.cpp,%.d,$(notdir $(SOURCES))))

# compile source code and link it to target so

libRoadObjectPostProcess.so: $(OBJS_customop)
	$(CC) $^ $(LNK_FLAGS) -o $@
	rm -rf $(BUILD_DIR)

#create OBJ dir to make sure it exists before compilation
$(OBJ_DIR)/%.o:%.cpp
	@if [ ! -d $(OBJ_DIR) ]; then mkdir -p $(OBJ_DIR); fi;
	$(CC) -c $(CC_FLAGS) -o $@ $<


#create DEPS dir to make sure it exists before compilation
$(DEPS_DIR)/%.d:%.cpp
	@if [ ! -d $(DEPS_DIR) ]; then mkdir -p $(DEPS_DIR); fi;
	set -e; rm -f $@;
	$(CC) -MM $(CC_FLAGS) $< > $@.$$$$;
	sed 's,\($*\)\.o[ :]*,$(OBJ_DIR)/\1.o $@ : ,g' < $@.$$$$ > $@;
	rm -f $@.$$$$

ifneq ($(MAKECMDGOALS), clean)
	-include $(DEPS)
endif

.PHONY : clean install
clean:
	rm -rf $(BUILD_DIR) lib*.so *.o
install: libRoadObjectPostProcess.so
	mv *.so $(OUT_DIR)
