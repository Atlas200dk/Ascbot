#include "phone_talker_manager.h"

CPhoneTalkerManager::CPhoneTalkerManager() {
  m_pPTR = new CPhoneTalkerRos();
  m_pPTT = new CPhoneTalkerTcp();
}

void CPhoneTalkerManager::init() {
  m_pPTR->setPTM(this);
  m_pPTT->setPTM(this);
}

CPhoneTalkerRos* CPhoneTalkerManager::getPTR() {
  return m_pPTR;
}

CPhoneTalkerTcp* CPhoneTalkerManager::getPTT() {
  return m_pPTT;
}
