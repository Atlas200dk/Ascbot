#include "H264VideoServerMediaSubsession.hh"
#include "H264VideoStreamFramer.hh"
#include "H264VideoRTPSink.hh"
#include "H265VideoRTPSink.hh"
#include "H265VideoStreamFramer.hh"
#include "H264VideoSource.hh"

H264VideoServerMediaSubsession *
H264VideoServerMediaSubsession::createNew(UsageEnvironment & env, char const* fileName,
    Boolean reuseFirstSource, CircularQueue *frameSendShm) {
    return new H264VideoServerMediaSubsession(env, fileName, reuseFirstSource, frameSendShm);
}

H264VideoServerMediaSubsession::H264VideoServerMediaSubsession(UsageEnvironment & env,
char const* fileName, Boolean reuseFirstSource, CircularQueue *frameSendShm)
:OnDemandServerMediaSubsession(env, reuseFirstSource),
fAuxSDPLine(NULL), fDoneFlag(0), fDummyRTPSink(NULL), p_frameSendShm(frameSendShm) {
    printf("[%s] line %d \n", __func__, __LINE__);
}

H264VideoServerMediaSubsession::~H264VideoServerMediaSubsession(void) {
    delete[] fAuxSDPLine;
}


/*************************************************/

static void afterPlayingDummy(void * clientData) {
    printf("[%s] line %d \n", __func__, __LINE__);
    H264VideoServerMediaSubsession * subsess = reinterpret_cast<H264VideoServerMediaSubsession *>(clientData);
    subsess->afterPlayingDummy1();
}

void H264VideoServerMediaSubsession::afterPlayingDummy1() {
    envir().taskScheduler().unscheduleDelayedTask(nextTask());
    // Signal the event loop that we're done:
    setDoneFlag();
}

static void checkForAuxSDPLine(void * clientData) {
    H264VideoServerMediaSubsession * subsess = reinterpret_cast<H264VideoServerMediaSubsession *>(clientData);
    subsess->checkForAuxSDPLine1();
}

void H264VideoServerMediaSubsession::checkForAuxSDPLine1() {
    char const* dasl;
    if (fAuxSDPLine != NULL) {
        // Signal the event loop that we're done:
        setDoneFlag();
    } else if (fDummyRTPSink != NULL && (dasl = fDummyRTPSink->auxSDPLine()) != NULL) {
        fAuxSDPLine = strDup(dasl);
        fDummyRTPSink = NULL;
        // Signal the event loop that we're done:
        setDoneFlag();
    } else if (!fDoneFlag) {
        // try again after a brief delay:
        int uSecsToDelay = 100;
        nextTask() = envir().taskScheduler().scheduleDelayedTask(uSecsToDelay,
                    reinterpret_cast<TaskFunc*>(checkForAuxSDPLine), this);
    }
}

/*********************virtual***************************/

char const * H264VideoServerMediaSubsession::getAuxSDPLine(RTPSink * rtpSink, FramedSource * inputSource) {
    if (fAuxSDPLine != NULL) return fAuxSDPLine;

    if (fDummyRTPSink == NULL) {
        fDummyRTPSink = rtpSink;
        fDummyRTPSink->startPlaying(*inputSource, afterPlayingDummy, this);
        checkForAuxSDPLine(this);
    }

    envir().taskScheduler().doEventLoop(&fDoneFlag);
    return fAuxSDPLine;
}

FramedSource * H264VideoServerMediaSubsession::createNewStreamSource(unsigned clientSessionId, unsigned & estBitrate) {
    printf("[%s] line %d \n", __func__, __LINE__);
    // estBitrate = 500; // kbps, estimate

    // Create the video source:
    H264VideoSource* videoSource = H264VideoSource::createNew(envir(), p_frameSendShm);
    if (videoSource == NULL) return NULL;
    // Create a framer for the Video Elementary Stream:
    H264VideoStreamFramerBase   *pBase = H264VideoStreamFramerBase::createNew(envir(), videoSource);
    videoSource->SetBase(pBase);
    return pBase;
    // return H264VideoStreamFramer::createNew(envir(), videoSource);
}

RTPSink * H264VideoServerMediaSubsession::createNewRTPSink(Groupsock * rtpGroupsock,
unsigned char rtpPayloadTypeIfDynamic, FramedSource * inputSource) {
    return H264VideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic);
}
