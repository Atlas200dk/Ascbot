#include "H264VideoSource.hh"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#define MAX_FRAME_SIZE  1920*1080*3/2  // #define BANK_SIZE 150000

unsigned char frame_buffer[MAX_FRAME_SIZE] = {0};

H264VideoStreamFramerBase* H264VideoStreamFramerBase
::createNew(UsageEnvironment& env, FramedSource* inputSource, Boolean includeStartCodeInOutput) {
    return new H264VideoStreamFramerBase(env, inputSource, True, includeStartCodeInOutput);
}

H264VideoStreamFramerBase
::H264VideoStreamFramerBase(UsageEnvironment& env, FramedSource* inputSource, Boolean createParser,
Boolean includeStartCodeInOutput)
:H264VideoStreamFramer(env, inputSource, createParser, includeStartCodeInOutput) {
}

H264VideoStreamFramerBase::~H264VideoStreamFramerBase() {
}

struct  timeval  *H264VideoStreamFramerBase::GetPresentationTime() {
    return &fPresentationTime;
}

struct  timeval  *H264VideoStreamFramerBase::GetNextPresentationTime() {
    return &fNextPresentationTime;
}

double  H264VideoStreamFramerBase::GetFramerate() {
    return fFrameRate;
}


H264VideoSource*
H264VideoSource::createNew(UsageEnvironment& env, CircularQueue *frameSendShm) {
    return new H264VideoSource(env, frameSendShm);
}

H264VideoSource::H264VideoSource(UsageEnvironment & env, CircularQueue *frameSendShm) :
FramedSource(env), p_frameSend(frameSendShm) {
    printf("[%s] line %d \n", __func__, __LINE__);
}

H264VideoSource::~H264VideoSource(void) {
    printf("[%s] line %d \n", __func__, __LINE__);

    envir().taskScheduler().unscheduleDelayedTask(nextTask());

    printf("[%s] line %d rtsp connection closed \n", __func__, __LINE__);
}

void H264VideoSource::doGetNextFrame() {
    struct timespec current_time = { 0, 0 };
    fFrameSize = 0;

    unsigned int framelen = -1;
    unsigned char* pData = NULL;
    if (p_frameSend != NULL) {
        framelen =  p_frameSend->GetQueueLength();

        if (framelen > 0) {
            clock_gettime(CLOCK_MONOTONIC, &current_time);

            printf("[%s] framelen %d fMaxSize %d tv_sec: %ld tv_nsec: %ld \n", __func__,
            framelen, fMaxSize, current_time.tv_sec, current_time.tv_nsec);

            if (framelen > fMaxSize) {
                fFrameSize = fMaxSize;

                pData = p_frameSend->get_read_buff(fFrameSize);
                if (pData == NULL) {
                    unsigned int ret_len = p_frameSend->DeQueue(frame_buffer, fFrameSize);                    
                    memmove(fTo, frame_buffer, fFrameSize);
                } else {
                    memmove(fTo, pData, fFrameSize);
                    p_frameSend->read_clear(fFrameSize);
                }
                fNumTruncatedBytes = framelen- fMaxSize;
            } else {
                fFrameSize = framelen;

                pData = p_frameSend->get_read_buff(fFrameSize);
                if (pData == NULL) {
                    unsigned int ret_len = p_frameSend->DeQueue(frame_buffer, fFrameSize);
                    memmove(fTo, frame_buffer, fFrameSize);
                } else {
                    memmove(fTo, pData, fFrameSize);
                    p_frameSend->read_clear(fFrameSize);
                }
                fNumTruncatedBytes = 0;
            }
        }
    }

    struct timeval *  nextPT = m_pBase->GetNextPresentationTime();
    gettimeofday(&fPresentationTime, NULL);
    *nextPT = fPresentationTime;

    nextTask() = envir().taskScheduler().scheduleDelayedTask(0,
    reinterpret_cast<TaskFunc*>(FramedSource::afterGetting), this);
}

unsigned int H264VideoSource::maxFrameSize() const {
    return MAX_FRAME_SIZE;
}
