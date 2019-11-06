#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"
#include "H264VideoServerMediaSubsession.hh"
#include "CircularQueue.h"

UsageEnvironment* env;

// To make the second and subsequent client for each stream reuse the same
// input stream as the first client (rather than playing the file from the
// start for each client), change the following "False" to "True":
Boolean reuseFirstSource = True;
CircularQueue *frameSendShm = NULL;

static void announceStream(RTSPServer* rtspServer, ServerMediaSession* sms,
char const* streamName) {
    char* url = rtspServer->rtspURL(sms);
    UsageEnvironment& env = rtspServer->envir();

    env << "\n" << "Play this stream using the URL \"" << url << "\"\n";
    delete[] url;
}


int main(int argc, char** argv) {
    OutPacketBuffer::maxSize = 1920*1080*3/2;
    // Begin by setting up our usage environment:
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    env = BasicUsageEnvironment::createNew(*scheduler);
    UserAuthenticationDatabase* authDB = NULL;

    // Create the RTSP server:
    RTSPServer* rtspServer = RTSPServer::createNew(*env, 8554, authDB);

    if (rtspServer == NULL) {
        *env << "Failed to create svt RTSP server: " << env->getResultMsg() << "\n";
        exit(1);
    }

    if (frameSendShm == NULL) {
        frameSendShm = new CircularQueue(SHM_TYPE , CIRCUALAR_QUEUE_BUFFER_SIZE );  // this size do not change
        frameSendShm->ShmServerInit();
        printf("[%s]  %d CircularQueue Init \n", __func__, __LINE__);
    }

    char const* descriptionString = "Session streamed by \"robot\"";

    // Set up each of the possible streams that can be served by the
    // RTSP server.  Each such stream is implemented using a
    // "ServerMediaSession" object, plus one or more
    // "ServerMediaSubsession" objects for each audio/video substream.

    char const* streamName = "robot";

    // A H.264 video elementary stream:

    ServerMediaSession* sms = ServerMediaSession::createNew(*env, streamName, streamName, descriptionString, 0);

    sms->addSubsession(H264VideoServerMediaSubsession::createNew(*env, streamName, reuseFirstSource, frameSendShm));

    rtspServer->addServerMediaSession(sms);
    announceStream(rtspServer, sms, streamName);

    // Also, attempt to create a HTTP server for RTSP-over-HTTP tunneling.
    // Try first with the default HTTP port (80), and then with the alternative HTTP
    // port numbers (8000 and 8080).

    if (rtspServer->setUpTunnelingOverHTTP(80) || rtspServer->setUpTunnelingOverHTTP(8000)
        || rtspServer->setUpTunnelingOverHTTP(8080)) {
        *env << "\n(We use port " << rtspServer->httpServerPortNum() << " for optional RTSP-over-HTTP tunneling.)\n";
    } else {
        *env << "\n(RTSP-over-HTTP tunneling is not available.)\n";
    }

    env->taskScheduler().doEventLoop();

    if (frameSendShm != NULL) {
        delete frameSendShm;
        frameSendShm = NULL;
    }

    printf("Exit server.....!\n");

    return 0;  // only to prevent compiler warning
}
