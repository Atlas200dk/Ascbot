#ifndef _CIRCULARQUEUE_H_
#define _CIRCULARQUEUE_H_

#define SHM_NAME "shm_h264"

#define CIRCUALAR_QUEUE_BUFFER_SIZE (10*1024*1024)

enum BUFFER_TYPE { SHM_TYPE = 1000, NORMAL_TYPE = 1001 };

#pragma pack(1)
typedef struct circualar_queue_param {
  unsigned int init;
  unsigned char * buffer_addr;
  unsigned int buffer_size;
  unsigned int write_p;
  unsigned int read_p;
}CircularQueueParam;
#pragma pack()

class CircularQueue {
public:
  explicit CircularQueue(BUFFER_TYPE buff_type, unsigned int buf_size = CIRCUALAR_QUEUE_BUFFER_SIZE);
  ~CircularQueue();

  void ShmServerInit();
  void ShmClientInit();
  bool ShmClientInitStatus();
  void NormalInit();

  void UnInit();

  void read_clear(int len);
  unsigned char* get_read_buff(int need_len);
  void write_clear(int len);
  unsigned char* get_write_buff(int need_len);
 
  unsigned int GetQueueLength();
  unsigned int EnQueue(unsigned char* buffer, unsigned int len);
  unsigned int DeQueue(unsigned char* buffer, unsigned int len);

  void ResetQueue();

private:
  unsigned int QueueBufferFree();

private:
  BUFFER_TYPE buffer_type;

  int    shm_fd;
  void * shm_addr;
  int    shm_size;

  unsigned char* buffer_addr;
  unsigned int   buffer_size;

  CircularQueueParam  *p_cQParam;
  unsigned int circularQueueParamLength;
};
#endif  // _CIRCULARQUEUE_H_
