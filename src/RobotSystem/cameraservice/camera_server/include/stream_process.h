#ifndef _STREAM_PROCESS_H_
#define _STREAM_PROCESS_H_

#include <pthread.h>
#include <atomic>
#include "hiaiengine/ai_memory.h"
#include "dvpp_process.h"
#include "dvpp/Vpc.h"
#include "dvpp/idvppapi.h"
#include "dvpp/dvpp_config.h"
#include "camera.h"
#include "CircularQueue.h"

#define  USE_AI 1

namespace ascend {
namespace ascendstream {

enum StreamProcessErrorCode {
  kStreamProcessOk,
  kStreamProcessParaCheckError,
  kStreamProcessCreatThreadError
};


const int kYuvImageWidth = 1920;

const int kYuvImageHigh = 1080;

// maximum size of yuv file
const int kYuvImageSize = (kYuvImageWidth * kYuvImageHigh * 3 / 2);

const int kYuvMarkWidth = 200;

const int kYuvMarkHigth = 200;

const int kYuvArrawSize = (kYuvMarkWidth * kYuvMarkHigth * 3 / 2);

const  int startMarkX = (kYuvImageWidth/2 - kYuvMarkWidth/2-1);

const  int startMarkY = (kYuvImageHigh - kYuvMarkHigth-1);

// 1 second = 1000 millsecond
const int kSecToMillisec = 1000;

// 1 millisecond = 1000000 nanosecond
const int kMillSecToNanoSec = 1000000;

// 1 second = 1000000 Microsecond
const int kSecToMicroSec = 1000000;

// 1 Microsecond = 1000 nanosecond
const int kMicroSecToNanoSec = 1000;

typedef struct CameraSetting {
  int camera_channel;

  int fps;

  int width;

  int height;

  int media_type;

  int time_out;

  unsigned char * data;

  unsigned char * yuvData;

  unsigned char * p_data;

  int64_t timestamp;
}S_CAMERA_SETTING;

typedef struct RoadFllowing {
  bool arraw_flag;
  unsigned char * arraw[3];
  int  arraw_size[3];
}S_ROAD_FOLLOWING;

struct ThreadProc {
  std::atomic_int open_thread_flag;

  // thread id
  pthread_t thread_id;

  // thread status
  // std::atomic_int thread_status;
  bool thread_status;
};

struct StreamControl {
  CameraSetting camera_set;

  ascend::ascendcamera::Camera *camera;

  ascend::utils::DvppProcess *dvpp_process;

  CircularQueue * frame_process;

  CircularQueue * frame_out;

  ThreadProc  tp;

  bool loop_flag;
};

class StreamProcess {
public:
  StreamProcess();

  virtual ~StreamProcess();

  int StreamInit(int argc, char *argv[]);

  int StreamRun();

private:
  StreamControl s_control;

  void CameraInstanceInit(int width, int height);

  void DvppInstanceInit(int width, int height);

  int CameraInit();

  int CircularQueueInit();

  static void *MultiFrameProcThread(void *startup_arg);

#if USE_AI
  bool GetNv12Data(int type);
  bool DrawAngle(unsigned char * image_buffer , unsigned char * arraw_buffer,
                 int start_x, int start_y, int point_x, int point_y);
  bool DrawBox(unsigned char * image_buffer);
#endif
};
}  // namespace ascendstream
}  // namespace ascend
#endif  // _STREAM_PROCESS_H_

