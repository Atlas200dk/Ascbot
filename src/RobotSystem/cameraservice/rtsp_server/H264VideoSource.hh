#ifndef _H264VideoSource_HH
#define _H264VideoSource_HH
 
#include "FramedSource.hh"
#include "H264VideoStreamFramer.hh"
#include "CircularQueue.h"


class H264VideoStreamFramerBase: public H264VideoStreamFramer {
public:
  static H264VideoStreamFramerBase* createNew(UsageEnvironment& env, FramedSource* inputSource,
					  Boolean includeStartCodeInOutput = False);

protected:
  H264VideoStreamFramerBase(UsageEnvironment& env, FramedSource* inputSource,
			Boolean createParser, Boolean includeStartCodeInOutput);
  virtual ~H264VideoStreamFramerBase();
public:
    struct  timeval  *GetPresentationTime() ;
    struct  timeval  *GetNextPresentationTime() ;
    double  GetFramerate() ;

};

class H264VideoSource : public FramedSource
{
public:
  static H264VideoSource* 
  createNew(UsageEnvironment& env,CircularQueue *frameSendShm);
  bool startServer( u_int32_t port);
  void SetBase(H264VideoStreamFramerBase *pBase)       {m_pBase=pBase;}   
  

protected:
	H264VideoSource(UsageEnvironment & env,CircularQueue *frameSendShm);
	~H264VideoSource(void);
 
private:
	virtual void doGetNextFrame();
  virtual unsigned int maxFrameSize() const;
 
private:
	H264VideoStreamFramerBase *m_pBase;
  CircularQueue * p_frameSend;

};
 
#endif
