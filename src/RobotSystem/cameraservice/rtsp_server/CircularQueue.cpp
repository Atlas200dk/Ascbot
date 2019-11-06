#include "CircularQueue.h"
#include <sys/types.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>

CircularQueue::CircularQueue(BUFFER_TYPE buff_type, unsigned int buff_size):
buffer_type(buff_type),
shm_fd(0),
shm_addr(NULL),
shm_size(0),
buffer_addr(NULL),
buffer_size(buff_size),
p_cQParam(NULL),
circularQueueParamLength(0) {
    circularQueueParamLength = sizeof(struct circualar_queue_param);
    printf("[CircularQueue] circularQueueParamLength:%d\n", circularQueueParamLength);
}

void CircularQueue::ShmServerInit() {
    printf("[CircularQueue][%s] line %d \n", __func__, __LINE__);

    if (buffer_type == SHM_TYPE && shm_addr == NULL) {
        shm_size = buffer_size + circularQueueParamLength;
        printf("[CircularQueue] shm_size:%d\n", shm_size);

        //创建或者打开一个共享内存
        shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR , 00777);
        if (shm_fd < 0) {
            printf("[CircularQueue] error open region\n");
        }

        //调整确定文件共享内存的空间
        int ret = ftruncate(shm_fd, shm_size);
        if (-1 == ret) {
            printf("ftruncate faile: ");
        }

        //映射目标文件的存储区
        shm_addr = mmap(NULL, shm_size , PROT_READ | PROT_WRITE , MAP_SHARED, shm_fd, SEEK_SET);
        if (NULL == shm_addr) {
            printf("mmap add_r failed: ");
        }

        p_cQParam = (struct circualar_queue_param *)shm_addr;

        buffer_addr = reinterpret_cast<u_char*>(shm_addr+circularQueueParamLength);

        if ( p_cQParam != NULL ) {
            p_cQParam->buffer_addr = buffer_addr;

            p_cQParam->buffer_size = buffer_size;

            p_cQParam->write_p = 0;

            p_cQParam->read_p = 0;

            p_cQParam->init = 0x11223344;

            printf("[CircularQueue][%s] p_cQParam init: %x \n", __func__, p_cQParam->init);
            printf("[CircularQueue][%s] p_cQParam write_p: %d \n", __func__, p_cQParam->write_p);
            printf("[CircularQueue][%s] p_cQParam read_p: %d \n", __func__, p_cQParam->read_p);
        }
    }
}


void CircularQueue::ShmClientInit() {
    if (buffer_type == SHM_TYPE && shm_addr == NULL) {
        shm_size = buffer_size + circularQueueParamLength;
        printf("[CircularQueue] shm_size:%d\n", shm_size);

        //创建或者打开一个共享内存
        shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR , 00777);
        if (shm_fd < 0) {
            printf("[CircularQueue] error open region\n");
        }

        //调整确定文件共享内存的空间
        int ret = ftruncate(shm_fd, shm_size);
        if (-1 == ret) {
            printf("ftruncate faile: ");
        }

        //映射目标文件的存储区
        shm_addr = mmap(NULL, shm_size , PROT_READ | PROT_WRITE , MAP_SHARED, shm_fd, SEEK_SET);
        if (NULL == shm_addr) {
            printf("mmap add_r failed: ");
        }

        p_cQParam = (struct circualar_queue_param *)shm_addr;

        buffer_addr = reinterpret_cast<u_char*>(shm_addr+circularQueueParamLength);

        if ( p_cQParam != NULL ) {
            p_cQParam->buffer_addr = buffer_addr;

            p_cQParam->buffer_size = buffer_size;

            printf("[CircularQueue][%s] p_cQParam init: %x \n", __func__, p_cQParam->init);
            printf("[CircularQueue][%s] p_cQParam write_p: %d \n", __func__, p_cQParam->write_p);
            printf("[CircularQueue][%s] p_cQParam read_p: %d \n", __func__, p_cQParam->read_p);

            if (p_cQParam->init == 0x11223344) {
                printf("ShmClientInit ok \n");
            }
        }
    }
}

bool CircularQueue::ShmClientInitStatus() {
    if (p_cQParam != NULL) {
        return (p_cQParam->init == 0x11223344);
    }
    return false;
}


void CircularQueue::NormalInit() {
    printf("[CircularQueue][%s] line %d \n", __func__, __LINE__);
    if (buffer_type == NORMAL_TYPE) {
        shm_addr = new u_char[circularQueueParamLength];

        p_cQParam = (struct circualar_queue_param *)shm_addr;

        buffer_addr = new u_char[buffer_size];

        if ( p_cQParam != NULL ) {
            p_cQParam->buffer_addr = buffer_addr;

            p_cQParam->buffer_size = buffer_size;

            p_cQParam->write_p = 0;

            p_cQParam->read_p = 0;

            p_cQParam->init = 0x11223344;

            printf("[CircularQueue][%s] p_cQParam init: %x \n", __func__, p_cQParam->init);
            printf("[CircularQueue][%s] p_cQParam write_p: %d \n", __func__, p_cQParam->write_p);
            printf("[CircularQueue][%s] p_cQParam read_p: %d \n", __func__, p_cQParam->read_p);
        }
    }
}

void CircularQueue::UnInit() {
    printf("[CircularQueue][%s] line %d \n", __func__, __LINE__);
    if ( buffer_type == SHM_TYPE ) {
        //取消映射
        int ret = munmap(shm_addr, shm_size);
        if (-1 == ret) {
            printf("munmap add_r faile: ");
        }
        //删除内存共享
        ret = shm_unlink(SHM_NAME);
        if (-1 == ret) {
            printf("shm_unlink faile: ");
        }
    } else {
        if (shm_addr != NULL) {
            delete[] reinterpret_cast<u_char*>(shm_addr);
        }

        if (buffer_addr != NULL) {
            delete[] buffer_addr;
        }
    }

    shm_addr = NULL;
    buffer_addr = NULL;
    p_cQParam = NULL;
}

CircularQueue::~CircularQueue() {
    UnInit();
}

/*
 清除缓冲区内容
 */
void CircularQueue::ResetQueue() {
    if (p_cQParam != NULL) {
        p_cQParam->write_p = 0;
        p_cQParam->read_p = 0;
    }
}

void CircularQueue::read_clear(int len) {
    p_cQParam->read_p = (p_cQParam->read_p + len) % p_cQParam->buffer_size;
}

void CircularQueue::write_clear(int len) {
    p_cQParam->write_p = (p_cQParam->write_p + len) % p_cQParam->buffer_size;
}

unsigned char* CircularQueue::get_read_buff(int need_len) {
    int tail_len = p_cQParam->buffer_size - p_cQParam->read_p;
    if (tail_len < need_len) {
        return NULL;
    } else {
        return buffer_addr + p_cQParam->read_p;
    }
}

unsigned char* CircularQueue::get_write_buff(int need_len) {
    if (need_len > QueueBufferFree()) {
        return NULL;
    } else {
        unsigned int tail_len = p_cQParam->buffer_size - p_cQParam->write_p;
        if(tail_len < need_len)
        {
            return NULL;
        } else {
            return buffer_addr+p_cQParam->write_p;
        }
    }
}

/*
  可使用的缓冲区的长度
 */
unsigned int CircularQueue::QueueBufferFree() {
    return p_cQParam->buffer_size -
    (p_cQParam->write_p - p_cQParam->read_p + p_cQParam->buffer_size) % p_cQParam->buffer_size;
}

/*
  已使用的缓冲区的长度
 */
unsigned int CircularQueue::GetQueueLength() {
    return (p_cQParam->write_p - p_cQParam->read_p + p_cQParam->buffer_size) % p_cQParam->buffer_size;
}

/*
  往缓冲区内添加数据
 */
unsigned int CircularQueue::EnQueue(unsigned char* buffer, unsigned int len) {
    if (len > QueueBufferFree()) {
        return -1;
    } else {
        unsigned int tail_len = p_cQParam->buffer_size - p_cQParam->write_p;

        if (len > tail_len) {
            memcpy(buffer_addr + p_cQParam->write_p, buffer, tail_len);
            p_cQParam->write_p = (p_cQParam->write_p + tail_len) % p_cQParam->buffer_size;

            memcpy(buffer_addr + p_cQParam->write_p, buffer + tail_len, len - tail_len);
            p_cQParam->write_p = (p_cQParam->write_p + len - tail_len) % p_cQParam->buffer_size;

        } else {
            memcpy(buffer_addr + p_cQParam->write_p, buffer, len);
            p_cQParam->write_p = (p_cQParam->write_p + len) % p_cQParam->buffer_size;
        }
        return len;
    }
}

/*
  从缓冲区内读取数据
 */
unsigned int CircularQueue::DeQueue(unsigned char* buffer, unsigned int len) {
    unsigned int tail_len = p_cQParam->buffer_size - p_cQParam->read_p;
    if (tail_len < len) {
        memcpy(buffer, buffer_addr + p_cQParam->read_p, tail_len);
        p_cQParam->read_p = (p_cQParam->read_p + tail_len) % p_cQParam->buffer_size;

        memcpy(buffer + tail_len, buffer_addr + p_cQParam->read_p,  len - tail_len);
        p_cQParam->read_p = (p_cQParam->read_p + len - tail_len) % p_cQParam->buffer_size;
    } else {
        memcpy(buffer, buffer_addr + p_cQParam->read_p, len);
        p_cQParam->read_p = (p_cQParam->read_p + len) % p_cQParam->buffer_size;
    }
    return len;
}



