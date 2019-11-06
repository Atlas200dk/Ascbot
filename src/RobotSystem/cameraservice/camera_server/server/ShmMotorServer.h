#ifndef MOTRO_STATUS_H_
#define MOTRO_STATUS_H_

#include <sys/types.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>   

#define SHM_NAME_MOTOR "shm_motor"
#define OBJECT_RECOGNITION_NUM 10

#pragma pack(1) 
typedef struct object_recognition {
  int x;
  int y;
  int width;
  int height;
  int lable;
  float confidence;
}S_OBJECT_RECOGNITION;
#pragma pack()

#pragma pack(1) 
typedef struct motor_status {
  unsigned int init;//in
  int roadfollowingSwitch;//out
  int collisionSwitch;    //out
  int objectDetectionSwitch;//out
  int roadobjectDetectionSwitch;//out
  int roadfollowingChange;
  int collisionChange;
  int objectDetectionChange;
  int roadObjectDetectionChange;
  int angle;//in
  int x;//in
  int y;//in
  int collisionStatus;    //in
  int motorAngle;//in
  int motorRunStatus;//in
  int objNum;
  S_OBJECT_RECOGNITION object[OBJECT_RECOGNITION_NUM];
}S_MOTRO_STATUS;
#pragma pack()


class ShmMotorServer {
public:
  ShmMotorServer(){
    init_status = false;
    shm_size = sizeof(struct motor_status); 
    //创建或者打开一个共享内存
    // shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR , 00777);
    // if (shm_fd<0) {
    //     printf("[ShmMotroClient] error open err\n");
    // }

    // //调整确定文件共享内存的空间
    // int ret = ftruncate( shm_fd, shm_size);
    // if(-1 == ret)
    // {
    //     printf("ftruncate faile: ");
    // }

    // //映射目标文件的存储区
    // shm_addr = mmap(NULL, shm_size , PROT_READ | PROT_WRITE , MAP_SHARED, shm_fd, SEEK_SET);
    // if(NULL == shm_addr)
    // {
    //     printf("mmap add_r failed: ");
    // }

    shm_fd = shmget(123559, shm_size, IPC_CREAT | 0777);
    if (shm_fd < 0)
    {
        printf("get id failed\n");
    }
    
    shm_addr = shmat(shm_fd, NULL, 0);
    if (shm_addr == NULL)
    {
        printf("shamt failed\n");
    }

    p_motor_status = (struct motor_status *)shm_addr;

    if( p_motor_status != NULL )
    {
      p_motor_status->collisionStatus=0;
      p_motor_status->roadfollowingSwitch=0;
      p_motor_status->collisionSwitch=0;
      p_motor_status->objectDetectionSwitch=0;
      p_motor_status->init=0x11223344;
      init_status = true;
      printf("[%s] p_motor_status init: %x shm_size: %d  \n",__func__,
      p_motor_status->init,shm_size);
    }
  }

  ~ShmMotorServer(){
    init_status = false;
    // //取消映射
    // int ret = munmap(shm_addr, shm_size);
    // if(-1 == ret)
    // {
    //     printf("munmap add_r faile: ");
    // }
    // //删除内存共享
    // ret = shm_unlink(SHM_NAME);
    // if(-1 == ret)
    // {
    //     printf("shm_unlink faile: ");
    // }

    int ret = shmdt(shm_addr);

    //从系统中删除由id标识的共享内存区
    shmctl(shm_fd,IPC_RMID,NULL);
  }

  motor_status  * GetMotorStatus(){
    return p_motor_status;
  }
public:
  bool init_status;
private:  
  int    shm_fd;
  void * shm_addr;
  int    shm_size;
  S_MOTRO_STATUS  *p_motor_status;
  
};


#endif  // MOTRO_STATUS_H_
