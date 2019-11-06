#ifndef _PHONE_TALKER_MANAGER_H_
#define _PHONE_TALKER_MANAGER_H_
#include "ros/ros.h"
#include "phone_talker_tcp.h"
#include "phone_talker_ros.h"
#include "motor/MotorSpeed.h"
#include "netLink.h"


class CPhoneTalkerManager {
//  public:
//  static CPhoneTalkerManager getInstance() {
//    return CPhoneTalkerManagerInstance.INSTANCE;
//  }
//  private:
//  static class CPhoneTalkerManagerInstance {
//    private static final CPhoneTalkerManager INSTANCE = new CPhoneTalkerManager();
//  }
//  private:
//  CPhoneTalkerManager();

public:
  CPhoneTalkerManager();
  void init();
  CPhoneTalkerRos* getPTR();
  CPhoneTalkerTcp* getPTT();
private:
  CPhoneTalkerRos *m_pPTR;
  CPhoneTalkerTcp *m_pPTT;
};
#endif  // _PHONE_TALKER_MANAGER_H_
