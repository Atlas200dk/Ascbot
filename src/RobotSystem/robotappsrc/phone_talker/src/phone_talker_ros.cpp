#include "phone_talker_manager.h"

using namespace std;
using namespace rapidjson;
CPhoneTalkerRos::CPhoneTalkerRos(){
  m_Sub_pose = m_NH.subscribe("error_code", 100, &CPhoneTalkerRos::callback_error_code, this);//订阅错误码
  m_Pub_mode = m_NH.advertise<std_msgs::Int8>("change_mode", 100);//切换模式的用法
  m_Pub_remote = m_NH.advertise<motor::MotorSpeed> ("motor_cmd",10);
  fs.open("/root/home/catkin_ws/modelConfig.txt",ios::in);
  if(!fs.is_open()){
      cout << "open file error" << endl;
  }
  while(getline(fs,modeScene));
  if(modeScene.length() <= 0){

  }else{
        std_msgs::Int8 mode;
        Document d;
        d.Parse(std::string(modeScene).c_str());
        StringBuffer buffer;
        Writer<StringBuffer> writer(buffer);
        d.Accept(writer);
        std::string str = buffer.GetString();
        Value& modeValue = d["mode"];
        int changeMode =  modeValue.GetInt();
        currentMode = changeMode;
        mode.data = currentMode;
        m_Pub_mode.publish(mode);
  }
  fs.close();
}

void CPhoneTalkerRos::setPTM(CPhoneTalkerManager* pPTM){
  m_pPTM = pPTM;
}

void CPhoneTalkerRos::talk2Tcp(std::string frame, std::string content){
  m_pPTM->getPTT()->talk2Tcp(frame, content);
}

void CPhoneTalkerRos::callback_error_code(const std_msgs::String::ConstPtr& codeMsg){
    talk2Tcp("error", codeMsg->data);
}
int CPhoneTalkerRos::getCurrentMode(){
    return currentMode;
}

void CPhoneTalkerRos::talk2Ros_Op(std::string content){//json string
  // default is "halt"
  std_msgs::Int8 mode;
  Document d;
  d.Parse(content.c_str());
  StringBuffer buffer;
  Writer<StringBuffer> writer(buffer);
  d.Accept(writer);
  std::string str = buffer.GetString();
  Value& modeValue = d["mode"];
  int changeMode =  modeValue.GetInt();
  currentMode = changeMode;
  fs.open("/root/home/catkin_ws/modelConfig.txt",ios::in|ios::out|ios::trunc);
  if (!fs.is_open()) {
	  cout << "open file error" << endl;
  }
  switch (changeMode) {
  case 0:
      mode.data = 0;
      break;
  case 1:
      mode.data = 1;
      break;
  case 2:
      mode.data = 2;
  	  break;
  case 3:
      mode.data = 3;
      break;
  case 4:
	  mode.data = 4;
      break;
  }
  modeScene= content;
  std::cout << "INFO: Phone_talker writer modeScene " << modeScene << endl;
  if (fs && fs.is_open()){
     fs.clear();
     string test;
     length = modeScene.length();
     char * buffer = new char[length];
     std::cout << "INFO: INFO: Phone_talker read modeScene length " << length << endl;
     fs.write(modeScene.c_str(), (long int)(modeScene.length()));
     fs.flush();
     ROS_INFO_STREAM("INFO: Phone_talker writer mode success");
  }
  fs.close();
// 要回复模式切换成功
  Document dd;
  dd.SetObject();
  rapidjson::Document::AllocatorType& allocator = dd.GetAllocator();
  Value resultCode(StringRef("1000"));
  dd.AddMember("resultCode", resultCode,allocator);
  Value des(StringRef("1000"));
  dd.AddMember("msg", des,allocator);
  StringBuffer buffer1;
  Writer<StringBuffer> writer1(buffer);
  dd.Accept(writer1);
  talk2Tcp("mode_result", buffer1.GetString());  //返回切换模式成功的响应
  m_Pub_mode.publish(mode);
}
void CPhoneTalkerRos::talk2Ros_Remote(std::string content){
    motor::MotorSpeed motorspeed;
    Document d;
    d.Parse(content.c_str());
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    d.Accept(writer);
    std::string str = buffer.GetString();
    Value& direction = d["direction"];
    std::string directions =  direction.GetString();
    motorspeed.direction = directions;
    Value& leftspeed = d["leftspeed"];
    motorspeed.leftspeed = leftspeed.GetFloat();
    Value& rightspeed = d["rightspeed"];
    motorspeed.rightspeed = rightspeed.GetFloat();
    m_Pub_remote.publish(motorspeed);
}
