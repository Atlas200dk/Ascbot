#ifndef _PHONE_TALKER_TCP_H_
#define _PHONE_TALKER_TCP_H_
#include "phone_talker_manager.h"
#include "netLink.h"
#include <regex>
#include <vector>
#include <iterator>
#include <iostream>
#include <sstream>
#include <thread>
#include "stringbuffer.h"
class CPhoneTalkerManager;
class CPhoneTalkerTcp {
public:
  CPhoneTalkerTcp();
  void setPTM(CPhoneTalkerManager *pPTM);
  bool talk2Tcp(std::string frame, std::string content);

private:
//  void talk2Ros_Job(std::string content);
  void talk2Ros_Op(std::string content);
  void talk2Ros_Remote(std::string content);
  void run();

public:
  std::shared_ptr<netLink::Socket> phoneSocket;
  netLink::SocketManager* pSocketManager;

private:
  CPhoneTalkerManager *m_pPTM;
  int currentMode = -1;
  netLink::SocketManager socketManager;
  std::shared_ptr<netLink::Socket> socket;
};
#endif  //  _PHONE_TALKER_TCP_H_
