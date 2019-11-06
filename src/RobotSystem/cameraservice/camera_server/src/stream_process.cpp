#include "stream_process.h"

#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include "HiCollisionAvoidance.h"
#include "HiRoadFollowing.h"
#include "HiObjectFollowing.h"
#include "HiRoadObjectFollowing.h"
#include "ShmMotorServer.h"
#include "AiObjectDetection.h"

namespace ascend {
namespace ascendstream {

#if USE_AI
hi::HiRoadFollowing roadDetector;
hi::HiCollisionAvoidance caDetector;
hi::HiObjectFollowing objectDetector;
hi::HiRoadObjectFollowing roadObjectDetector;
RoadFllowing  s_rf;
ShmMotorServer  motor;
float scale_x1=0.2;
float scale_y1=0.65;
float scale_x2=0.8;
float scale_y2=1.0;
float scale_score=0.4;
#endif

StreamProcess::StreamProcess() {
  // constuct a instance used to control the whole process.
  s_control.camera = nullptr;
  s_control.dvpp_process = nullptr;

  s_control.frame_out =  nullptr;
  s_control.frame_process =  nullptr;
  s_control.camera_set.p_data = nullptr;

#if USE_AI
  s_rf.arraw_flag = false;
  s_rf.arraw[0] =  nullptr;
  s_rf.arraw[1] =  nullptr;
  s_rf.arraw[2] =  nullptr;
#endif

  s_control.loop_flag = false;
}

StreamProcess::~StreamProcess() {
#if USE_AI
  roadDetector.deInit();
  caDetector.deInit();
  objectDetector.deInit();
  roadObjectDetector.deInit();

  pthread_join(s_control.tp.thread_id, nullptr);

  if (s_rf.arraw[0] != nullptr) {
    delete s_rf.arraw[0];
    s_rf.arraw[0] = nullptr;
  }

  if (s_rf.arraw[1] != nullptr) {
    delete s_rf.arraw[1];
    s_rf.arraw[1] = nullptr;
  }

  if (s_rf.arraw[1] != nullptr) {
    delete s_rf.arraw[1];
    s_rf.arraw[1] = nullptr;
  }
#endif

  if (s_control.camera_set.data != nullptr) {
    delete s_control.camera_set.data;
    s_control.camera_set.data = nullptr;
  }

  if (s_control.camera_set.yuvData != nullptr) {
    delete s_control.camera_set.yuvData;
    s_control.camera_set.yuvData = nullptr;
  }

  s_control.camera_set.p_data = nullptr;

  if (s_control.camera != nullptr) {
    delete s_control.camera;
    s_control.camera = nullptr;
  }

  if (s_control.dvpp_process != nullptr) {
    delete s_control.dvpp_process;
    s_control.dvpp_process = nullptr;
  }

  if (s_control.frame_out!= nullptr) {
    delete s_control.frame_out;
          s_control.frame_out = nullptr;
  }

  if (s_control.frame_process!= nullptr) {
    delete s_control.frame_process;
          s_control.frame_process = nullptr;
  }
}

int StreamProcess::CircularQueueInit() {
  s_control.frame_out = new CircularQueue(SHM_TYPE , CIRCUALAR_QUEUE_BUFFER_SIZE);  // this size do not change
  s_control.frame_out->ShmClientInit();

  s_control.frame_process = new CircularQueue(NORMAL_TYPE , kYuvImageSize*6);
  s_control.frame_process->NormalInit();

  return 0;
}

int StreamProcess::CameraInit() {
    // init driver of camera and open camera.
  int ret = s_control.camera->InitCamera();
  if (ret != ascend::ascendcamera::kCameraInitOk) {
    s_control.camera->PrintErrorInfo(ret);
    return ret;
  } else {  // print to terminal
    printf("Success to open camera[%d],and start working.",
                 s_control.camera->GetChannelId());
  }
  return ret;
}

void StreamProcess::CameraInstanceInit(int width, int height) {
  ascend::ascendcamera::CameraPara camera_para;

  // camera instance paramter
  camera_para.fps = s_control.camera_set.fps;
  camera_para.capture_obj_flag = s_control.camera_set.media_type;
  camera_para.channel_id = s_control.camera_set.camera_channel;
  camera_para.image_format = CAMERA_IMAGE_YUV420_SP;
  camera_para.timeout = s_control.camera_set.time_out;
  camera_para.resolution.width = width;
  camera_para.resolution.height = height;

  // camera instance
  s_control.camera = new ascend::ascendcamera::Camera(camera_para);
}

void StreamProcess::DvppInstanceInit(int width, int height) {
    ascend::utils::DvppToH264Para dvpp_to_h264_para;

    // instance(convert to h264)
    dvpp_to_h264_para.coding_type = ascend::utils::kH264High;
    dvpp_to_h264_para.yuv_store_type = ascend::utils::kYuv420sp;
    dvpp_to_h264_para.resolution.width = width;
    dvpp_to_h264_para.resolution.height = height;

    s_control.dvpp_process = new ascend::utils::DvppProcess(dvpp_to_h264_para);
    s_control.loop_flag = true;
}

#if USE_AI

bool StreamProcess::GetNv12Data(int type) {
  char image_file[3][50] = {"arrow.yuv", "arrow_L.yuv", "arrow_R.yuv"};
  FILE * yuvimage = fopen(image_file[type], "rb");

  if (NULL == yuvimage) {
      printf("\r\n[%s] : yuv array image open faild!\n", __func__);
      return false;
  } else {
    fseek(yuvimage, 0L, SEEK_END);
    s_rf.arraw_size[type] = ftell(yuvimage);

    printf("[%s] s_rf.arraw_size　: %d \n", __func__, s_rf.arraw_size[type]);

    s_rf.arraw[type] = (unsigned char*)malloc(s_rf.arraw_size[type]);

    if (NULL == s_rf.arraw[type]) {
        printf("in_buffer_image alloc failed!\n");
        return false;
    }

    fseek(yuvimage, 0, SEEK_SET);
    fread(s_rf.arraw[type], 1, s_rf.arraw_size[type], yuvimage);
    fclose(yuvimage);
    yuvimage = NULL;
  }
  return true;
}

bool StreamProcess::DrawAngle(unsigned char * image_buffer , unsigned char * mark_buffer,
                              int start_x, int start_y, int point_x, int point_y) {
  for (int index_y = 0; index_y < 20; index_y++) {
    for (int index_x = 0; index_x < 20; index_x++) {
      if (point_x < (kYuvImageWidth-20) && point_y < (kYuvImageHigh -20)) {
        *(image_buffer+(point_y+index_y)*kYuvImageWidth+point_x+index_x) = 0x00;
      }
    }
  }

  for (int i = 0; i < kYuvMarkHigth; i++) {
    for (int j = 0; j < kYuvMarkWidth; j++) {
      if (*(mark_buffer+i*kYuvMarkWidth+j) != 0x00) {
        *(image_buffer+(start_y+i)*kYuvImageWidth+start_x+j) = *(mark_buffer+i*kYuvMarkWidth+j);
      }
    }
  }

  unsigned char * image_buffer_uv = image_buffer+kYuvImageWidth * kYuvImageHigh;
  unsigned char * mark_buffer_uv = mark_buffer+kYuvMarkWidth*kYuvMarkHigth;

  for (int i = 0; i < kYuvMarkHigth/2; i++) {
    for (int j = 0; j < kYuvMarkWidth; j++) {
      if (*(mark_buffer_uv+i*kYuvMarkWidth+j) != 0x80) {
        *(image_buffer_uv+(i+start_y/2)*kYuvImageWidth+start_x+j) = *(mark_buffer_uv+i*kYuvMarkWidth+j);
      }
    }
  }
      return true;
}

bool StreamProcess::DrawBox(unsigned char * image_buffer ) {

  for (int i = 0 ; i< motor.GetMotorStatus()->objNum ; i++) {
    if ((motor.GetMotorStatus()->object[i].y+motor.GetMotorStatus()->object[i].height <= kYuvImageHigh-1) &&
    (motor.GetMotorStatus()->object[i].x+motor.GetMotorStatus()->object[i].width <= kYuvImageWidth-1)) {
      memset(image_buffer +  motor.GetMotorStatus()->object[i].y*kYuvImageWidth +
              motor.GetMotorStatus()->object[i].x , 0x0 , motor.GetMotorStatus()->object[i].width);
      //下横线
      memset(image_buffer + (motor.GetMotorStatus()->object[i].y+motor.GetMotorStatus()->object[i].height)*
      kYuvImageWidth + motor.GetMotorStatus()->object[i].x , 0x0, motor.GetMotorStatus()->object[i].width);

      for (int j = 1 ; j< motor.GetMotorStatus()->object[i].height - 1 ; j++) {
        //左竖线
        *(image_buffer+(motor.GetMotorStatus()->object[i].y+j)*kYuvImageWidth +
        motor.GetMotorStatus()->object[i].x) = 0x0;
        //右竖线
        *(image_buffer+(motor.GetMotorStatus()->object[i].y+j)*kYuvImageWidth +
          motor.GetMotorStatus()->object[i].x+motor.GetMotorStatus()->object[i].width) = 0x0;
      }
    } else {
      // std::cout<<"物体"<<i<<"范围超标";
    }
  }

      return true;
}

struct timespec result_time = { 0, 0 };
int OnResultRoadFollowing(const std::vector<hi::RFData>& results) {
  // std::cout << "######OnResultRoadFollowing" << std::endl;
    struct timespec current_time = { 0, 0 };
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    result_time = current_time;

    long nowtimedifMicro = kSecToMicroSec*(current_time.tv_sec)+(current_time.tv_nsec)/kMicroSecToNanoSec;

    for (int i = 0; i < results.size(); i++) {
        const hi::RFData& fr = results.at(i);
        std::cout <<"######OnResultRoadFollowing"<< "x:" << fr.x << ",y:" << fr.y << ", angle:" << fr.angle << ",timestamp:" 
                  << fr.timestamp << ",timeif:" << nowtimedifMicro-fr.timestamp << std::endl;
          if (motor.init_status == true) {
            motor.GetMotorStatus()->angle = fr.angle;
            motor.GetMotorStatus()->x = (fr.x >= 0 ? fr.x : 0);
            motor.GetMotorStatus()->y = (fr.y >= 0 ? fr.y : 0);
            motor.GetMotorStatus()->roadfollowingChange = 1;
            motor.GetMotorStatus()->init++;
          }
    }
    return 0;
}

int OnResultCollisionAvoidance(const std::vector<hi::CollisionData>& results) {
  std::cout << "######OnResultCollisionAvoidance" << std::endl;
    for (int i = 0; i < results.size(); i++) {
        const hi::CollisionData& coll = results.at(i);
        std::cout << "coll status: " << static_cast<int>(coll.status) << "coll timestamp:" << coll.timestamp<<std::endl;

        if (motor.init_status == true) {
          motor.GetMotorStatus()->collisionStatus = static_cast<int>(coll.status);
          motor.GetMotorStatus()->collisionChange = 1;
          motor.GetMotorStatus()->init++;
        }

        // for(std::map<hi::CollisionStatus, float>::const_iterator iter = coll.status_map.begin();
        // iter != coll.status_map.end(); iter++) {
        //     std::cout<< iter->first <<"->" << iter->second << std::endl;
        // }
    }
    return 0;
}

// std::string g_label_str[] = {"person", "cup", "bottle", "box", "phone"};
int OnResultObjectFollowing(const std::vector<hi::ObjectData>& results) {
    std::cout << "######OnResultObjectFollowing size:" << results.size() << std::endl;
    motor.GetMotorStatus()->objNum = 0;

    for (int i = 0; i < results.size(); i++) {
        const hi::ObjectData& object = results.at(i);

      if (object.confidence > 0.8f) {
        // if (object.lable < 5) std::cout << g_label_str[object.lable] << std::endl;
        std::cout << "object" << i << ": lable=" << object.lable << ", box[" << object.x << "," << object.y << ","
            << object.width << "," << object.height << "] conf=" << object.confidence <<"object.timestamp " <<object.timestamp<< std::endl;

        if (motor.init_status == true && i < OBJECT_RECOGNITION_NUM) {
            motor.GetMotorStatus()->objNum = motor.GetMotorStatus()->objNum+1;
            motor.GetMotorStatus()->object[i].x = (object.x >= 0 ? object.x : 0);
            motor.GetMotorStatus()->object[i].y = (object.y >= 0 ? object.y : 0);
            motor.GetMotorStatus()->object[i].width = object.width;
            motor.GetMotorStatus()->object[i].height = object.height;
            motor.GetMotorStatus()->object[i].lable = object.lable;
            motor.GetMotorStatus()->object[i].confidence = object.confidence;
        }
      }
    }

    if (motor.init_status == true) {
      motor.GetMotorStatus()->objectDetectionChange = 1;
    }

    return 0;
}

// std::string g_label_str_road[] = {"cup", "car", "box", "person", "hand", "bottle",
//                              "phone", "book", "line", "left_round", "right_round","edge","light"};

int OnResultRoadObjectFollowing(const std::vector<hi::RoadObjectData>& results) {
    std::cout << "######OnResultRoadObjectFollowing size:" << results.size() << std::endl;
    motor.GetMotorStatus()->objNum = 0;
    int32_t retCheck = CheckObstacleResults(results, kYuvImageWidth, kYuvImageHigh,scale_x1,scale_y1,scale_x2,scale_y2,scale_score);

    if (retCheck == 1) {  // 1:has obstacle
        if (motor.GetMotorStatus()->roadfollowingSwitch == 1) { // road_following_mode
            if (motor.init_status == true) {
                motor.GetMotorStatus()->motorRunStatus = 0; // set motor stop run.
            }
        } else if (motor.GetMotorStatus()->collisionSwitch == 1) {    // collision_avoidance_mode
            int motorAngle = 0;
            if (motor.init_status == true) {  
                motor.GetMotorStatus()->motorRunStatus = 1;
                retCheck = checkTableResults(results, kYuvImageWidth, kYuvImageHigh, motorAngle);  
                motor.GetMotorStatus()->motorAngle = motorAngle;
            }
        }
    } else {  // 0:no obstacle
        if (motor.GetMotorStatus()->roadfollowingSwitch == 1) { // road_following_mode
            int motorAngle = 0;
            if (motor.init_status == true) {
                motor.GetMotorStatus()->motorRunStatus = 1; // set motor run, get angle from regression sdk in fact
                retCheck = checkRoadLineResults(results, kYuvImageWidth, kYuvImageHigh, motorAngle);  
                motor.GetMotorStatus()->motorAngle = motorAngle;

            }
        } else if (motor.GetMotorStatus()->collisionSwitch == 1) {    // collision_avoidance_mode
            int motorAngle = 0;
            if (motor.init_status == true) {  
                motor.GetMotorStatus()->motorRunStatus = 1;
                motor.GetMotorStatus()->motorAngle = 0;
            }
        }
    }
	
    if (motor.GetMotorStatus()->collisionSwitch == 1) {    // collision_avoidance_mode
        int motorAngle = 0;
        if (motor.init_status == true) {  
            motor.GetMotorStatus()->motorRunStatus = 1;
            if(motor.GetMotorStatus()->collisionStatus == hi::CollisionStatus::DANGER0){
                if(CheckEdge(results, kYuvImageWidth, kYuvImageHigh, 0.4)==0)
                    motor.GetMotorStatus()->motorAngle = -90;
                else
                    motor.GetMotorStatus()->motorAngle = 90;
            }
        }
    }
	
    for (int i = 0; i < results.size(); i++) {
        const hi::RoadObjectData& object = results.at(i);
        if (object.confidence > 0.8f && object.lable < 8) {
            // if (object.lable < 11) std::cout << g_label_str_road[object.lable] << std::endl;
            std::cout << "object" << i << ": lable=" << object.lable << ", box[" << object.x << ","
                      << object.y << "," << object.width << "," << object.height
                      << "] conf=" << object.confidence << ", time:" << object.timestamp
                      << std::endl;
           
          if (motor.init_status == true && i < OBJECT_RECOGNITION_NUM) {
            motor.GetMotorStatus()->objNum = motor.GetMotorStatus()->objNum+1;
            motor.GetMotorStatus()->object[i].x = (object.x >= 0 ? object.x : 0);
            motor.GetMotorStatus()->object[i].y = (object.y >= 0 ? object.y : 0);
            motor.GetMotorStatus()->object[i].width = object.width;
            motor.GetMotorStatus()->object[i].height = object.height;
            motor.GetMotorStatus()->object[i].lable = object.lable;
            motor.GetMotorStatus()->object[i].confidence = object.confidence;
          }          
        }
    }

    if (motor.init_status == true) {
      motor.GetMotorStatus()->roadObjectDetectionChange = 1;
    }

    return 0;
}

#endif


void *StreamProcess::MultiFrameProcThread(void *arg) {
  StreamProcess * p_stream = reinterpret_cast<StreamProcess *>(arg);

  int ret = kStreamProcessOk;

  while (p_stream->s_control.tp.thread_status == true) {
    if (p_stream->s_control.frame_process != NULL) {
      int len = p_stream->s_control.frame_process->GetQueueLength();
      if(len >= kYuvImageSize)
      {
        // printf("[%s] frame queue len:%d \n",__func__,len);

        ascend::utils::DvppOutput dvpp_output = { nullptr, 0 };

        unsigned char* p_read = p_stream->s_control.frame_process->get_read_buff(kYuvImageSize);
        if(p_read == NULL) {
          p_stream->s_control.frame_process->DeQueue(p_stream->s_control.camera_set.yuvData,kYuvImageSize);
          ret = p_stream->s_control.dvpp_process->DvppOperationProc(
            reinterpret_cast<char*>(p_stream->s_control.camera_set.yuvData), kYuvImageSize, &dvpp_output);
        } else {
            ret = p_stream->s_control.dvpp_process->DvppOperationProc(  // DVPP convert to jpg or h264
              reinterpret_cast<char*>(p_read) , kYuvImageSize, &dvpp_output);
            p_stream->s_control.frame_process->read_clear(kYuvImageSize);
        }

        if (ret == kStreamProcessOk) {
            // printf("[%s] frame dvpp size:%d \n",__func__,dvpp_output.size);
          if (p_stream->s_control.frame_out != NULL) {
            p_stream->s_control.frame_out->EnQueue(dvpp_output.buffer, dvpp_output.size);
          }    
        }

        if (dvpp_output.buffer != nullptr) {
          delete[] dvpp_output.buffer;
        }

      } else {
            usleep(2*1000);
      }
    }
  }

  return nullptr;
}

int StreamProcess::StreamInit(int argc, char *argv[]) {
  int ret = kStreamProcessOk;

  if(argc == 6)
  {
    scale_x1 = atof(argv[1]);
    scale_y1 = atof(argv[2]);
    scale_x2 = atof(argv[3]);
    scale_y2 = atof(argv[4]);
    scale_score = atof(argv[5]);
  }   

  // camera_set init
  s_control.camera_set.camera_channel = 0;
  s_control.camera_set.fps = 20;
  s_control.camera_set.width = kYuvImageWidth;
  s_control.camera_set.height = kYuvImageHigh;
  s_control.camera_set.media_type =  ascend::utils::kH264;
  s_control.camera_set.time_out = 100;
  s_control.camera_set.data = new unsigned char[kYuvImageSize];
  s_control.camera_set.yuvData = new unsigned char[kYuvImageSize];
  s_control.camera_set.timestamp = 0;

  int width  = s_control.camera_set.width;
  int height = s_control.camera_set.height;

  // camera instance
  CameraInstanceInit(width, height);

  // dvpp controller instance
  DvppInstanceInit(width, height);

  if ((s_control.camera == nullptr)|| (s_control.dvpp_process == nullptr)) {
    return kStreamProcessParaCheckError;
  }

  CircularQueueInit();

  ret = CameraInit();

  if (ret != kStreamProcessOk) {
    return ret;
  }

  pthread_t tid = 0;

  // create a thread
  ret = pthread_create(&tid, nullptr, MultiFrameProcThread, reinterpret_cast<void*>(this));
  if (ret != kStreamProcessOk) {
    printf("Failed to create thread.return value = %d", ret);
    return kStreamProcessCreatThreadError;
  }
  s_control.tp.thread_id = tid;

#if USE_AI

    if (GetNv12Data(0) == false) {
      printf("[%s] read image arraw[0] fail \n", __func__);
    }

    if (GetNv12Data(1) == false) {
      printf("[%s] read image arraw fail[1] \n", __func__);
    }

    if (GetNv12Data(2) == false) {
      printf("[%s] read image arraw fail[2] \n", __func__);
    }
    s_rf.arraw_flag = true;

  roadDetector.init("/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/road_following_graph.config");
  roadDetector.setCallbackFunction(OnResultRoadFollowing);

  caDetector.init("/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/collision_avoidance_graph.config");
  caDetector.setCallbackFunction(OnResultCollisionAvoidance);

  objectDetector.init("/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace/cameraservice/object_following_graph.config");
  objectDetector.setCallbackFunction(OnResultObjectFollowing);

  roadObjectDetector.init("/home/HwHiAiUser/HIAI_PROJECTS/ascend_workspace"
                          "/cameraservice/road_object_following_graph.config");
  roadObjectDetector.setCallbackFunction(OnResultRoadObjectFollowing);

#endif

  return ret;
}

int StreamProcess::StreamRun() {
  int ret = kStreamProcessOk;
  ascend::ascendcamera::CameraOutputPara output_para;

  struct timespec current_time = { 0, 0 };
  struct timespec old_time = { 0, 0 };
  long running_time;
  bool usedPoint = false;

  do {

    if(motor.GetMotorStatus()->init >= 0xfffffff0)
    {
      motor.GetMotorStatus()->init = 0x01;
    }

    // get a frame from camera
    ret = s_control.camera->CaptureCameraInfo(&output_para);
    if (ret != kStreamProcessOk) {
      s_control.camera->PrintErrorInfo(ret);
    }

    clock_gettime(CLOCK_MONOTONIC, &current_time);

    running_time = kSecToMillisec*(current_time.tv_sec-old_time.tv_sec)+(current_time.tv_nsec-old_time.tv_nsec)/kMillSecToNanoSec;
    old_time = current_time;

    long timestampMicro = kSecToMicroSec*(current_time.tv_sec)+(current_time.tv_nsec)/kMicroSecToNanoSec;

    printf("[%s] camera frame tv_sec: %ld tv_nsec: %ld  run_time %ld Millisecond \n", __func__, 
        current_time.tv_sec, current_time.tv_nsec, running_time);

    if(running_time >= 80){
       printf("[%s] CaptureCameraInfo too long out 80 Millisecond\n", __func__);
    }

#if USE_AI

    s_control.camera_set.p_data = s_control.frame_process->get_write_buff(output_para.size);

    if(s_control.camera_set.p_data == NULL)
    {
      memcpy(s_control.camera_set.data, output_para.data.get(), output_para.size);
      s_control.camera_set.p_data = s_control.camera_set.data;
      usedPoint = false;
    } else {
      memcpy(s_control.camera_set.p_data, output_para.data.get(), output_para.size);
      usedPoint = true;
    }

    hi::HIImgData image(s_control.camera_set.width, s_control.camera_set.height,
                        TYPE_YUV_NV12, s_control.camera_set.p_data, s_control.camera_set.width,timestampMicro);

    if (motor.GetMotorStatus()->roadfollowingSwitch == 1) {
      roadDetector.predict(image);
    }

    if (motor.GetMotorStatus()->collisionSwitch == 1) {
      caDetector.predict(image);
    }

    if (motor.GetMotorStatus()->objectDetectionSwitch == 1) {
      objectDetector.predict(image);
    }

    if (motor.GetMotorStatus()->roadobjectDetectionSwitch == 1) {
      roadObjectDetector.predict(image);
    }

    if (motor.GetMotorStatus()->roadfollowingChange == 1 && s_rf.arraw_flag == true) {
      if (motor.GetMotorStatus()->angle >= -20.0 && motor.GetMotorStatus()->angle <= 20.0) {
        DrawAngle(s_control.camera_set.p_data, s_rf.arraw[0], startMarkX,
        startMarkY, motor.GetMotorStatus()->x, motor.GetMotorStatus()->y);
      } else if (motor.GetMotorStatus()->angle >= -90.0 && motor.GetMotorStatus()->angle < -20.0) {
        DrawAngle(s_control.camera_set.p_data, s_rf.arraw[1], startMarkX, startMarkY,
        motor.GetMotorStatus()->x, motor.GetMotorStatus()->y);
      } else if (motor.GetMotorStatus()->angle > 20.0 && motor.GetMotorStatus()->angle <= 90.0) {
        DrawAngle(s_control.camera_set.p_data, s_rf.arraw[2], startMarkX, startMarkY,
        motor.GetMotorStatus()->x, motor.GetMotorStatus()->y);
      } else {
        printf("[%s] angle out angle:%d x:%d y:%d  \n",__func__ ,
        motor.GetMotorStatus()->angle,  motor.GetMotorStatus()->x, motor.GetMotorStatus()->y);
      }
      motor.GetMotorStatus()->roadfollowingChange = 0;
    }

    clock_gettime(CLOCK_MONOTONIC, &current_time);
    running_time = kSecToMillisec*(current_time.tv_sec-result_time.tv_sec)+(current_time.tv_nsec-result_time.tv_nsec)/kMillSecToNanoSec;

    long draw_time = kSecToMicroSec*(current_time.tv_sec-old_time.tv_sec)+(current_time.tv_nsec-old_time.tv_nsec)/kMicroSecToNanoSec;

    printf("[%s] result rf to draw time %ld Millisecond, draw run time %ld microseconds  \n", __func__, running_time, draw_time); 

    if (motor.GetMotorStatus()->objectDetectionChange == 1) {
      DrawBox(s_control.camera_set.p_data);
      motor.GetMotorStatus()->objectDetectionChange = 0;
    }

    if (motor.GetMotorStatus()->roadObjectDetectionChange == 1) {
      DrawBox(s_control.camera_set.p_data);
      motor.GetMotorStatus()->roadObjectDetectionChange = 0;
    }
#endif

    if(usedPoint == true)
    {
      s_control.frame_process->write_clear(output_para.size);
    } else {
      if (s_control.frame_process != NULL) {
        int ret_q = s_control.frame_process->EnQueue(s_control.camera_set.p_data, output_para.size);
        if(ret_q == -1)
        {
          printf("[%s] frame_process queue full \n",__func__);
        }
      }
    }

  } while (s_control.loop_flag == true);

  return ret;
}
}  // namespace ascendstream
}  // namespace ascend
