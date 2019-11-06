#include "OnDemandServerMediaSubsession.hh"
#include "CircularQueue.h"
 
class H264VideoServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
	static H264VideoServerMediaSubsession * 
	createNew(UsageEnvironment & env, char const* fileName, Boolean reuseFirstSource,
				CircularQueue *frameSendShm);

    void checkForAuxSDPLine1();
    void afterPlayingDummy1();

protected:
	H264VideoServerMediaSubsession(UsageEnvironment & env,  char const* fileName, 
	Boolean reuseFirstSource,CircularQueue *frameSendShm);
	~H264VideoServerMediaSubsession(void);
	void setDoneFlag() { fDoneFlag = ~0; }

protected: 
	virtual char const * getAuxSDPLine(RTPSink * rtpSink, FramedSource * inputSource);
	virtual FramedSource * createNewStreamSource(unsigned clientSessionId, unsigned & estBitrate);
	virtual RTPSink * createNewRTPSink(Groupsock * rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic, 
	FramedSource * inputSource);

private:
	char* fAuxSDPLine;
	char fDoneFlag; // used when setting up "fAuxSDPLine"
	RTPSink* fDummyRTPSink; // ditto
	CircularQueue *p_frameSendShm;
};
