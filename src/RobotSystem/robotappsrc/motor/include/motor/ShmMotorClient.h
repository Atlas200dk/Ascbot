#ifndef _SHMMOTORCLIENT_H_
#define _SHMMOTORCLIENT_H_

#include <sys/types.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>

#define SHM_NAME "shm_motor"

#pragma pack(1)
typedef struct motor_status {
  unsigned int init;                  //  in
  unsigned int angle;                 //  in
  unsigned int x;                     //  in
  unsigned int y;                     //  in
  unsigned int collisionStatus;       //  in
  unsigned int roadfollowingSwitch;   //  out
  unsigned int collisionSwitch;       //  out
  unsigned int objectDetectionSwitch;   //  out
}S_MOTRO_STATUS;
#pragma pack()


class ShmMotroClient {
public:
  ShmMotroClient() {
    shm_size = sizeof(struct motor_status);
    // 创建或者打开一个共享内存
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR , 00777);
    if (shm_fd < 0) {
        printf("[ShmMotroClient] error open err\n");
    }

    // 调整确定文件共享内存的空间
    int ret = ftruncate(shm_fd, shm_size);
    if (-1 == ret) {
        printf("ftruncate faile: ");
    }

    // 映射目标文件的存储区
    shm_addr = mmap(NULL, shm_size , PROT_READ | PROT_WRITE , MAP_SHARED, shm_fd, SEEK_SET);
    if (NULL == shm_addr) {
        printf("mmap add_r failed: ");
    }

    p_motor_status = (struct motor_status *)shm_addr;

    if ( p_motor_status != NULL ) {
        printf("[%s] init: %x \n", __func__, p_motor_status->init);
        if (p_motor_status->init == 0x11223344) {
            printf("ShmMotroClient ok \n");
        }
    }
  }

  ~ShmMotroClient() {
        // 取消映射
        int ret = munmap(shm_addr, shm_size);
        if (-1 == ret) {
            printf("munmap add_r faile: ");
        }
        // 删除内存共享
        ret = shm_unlink(SHM_NAME);
        if (-1 == ret) {
            printf("shm_unlink faile: ");
        }
  }

  motor_status  * GetMotorStatus() {
    return p_motor_status;
  }

private:
  int    shm_fd;
  void * shm_addr;
  int    shm_size;
  S_MOTRO_STATUS  *p_motor_status;
};
#endif  // _SHMMOTORCLIENT_H_
