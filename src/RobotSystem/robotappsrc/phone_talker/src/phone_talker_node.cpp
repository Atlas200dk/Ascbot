#include "phone_talker_manager.h"
#include "phone_talker_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "phone_talker_node");
  CPhoneTalkerManager ptm;
  ptm.init();
  ROS_INFO_STREAM("INFO: Phone_talker init...");
//  system('');
  std::string modeScene = "";
  std_msgs::Int8 mode;
// Document d;
// d.Parse(std::string(modeScene).c_str());
// StringBuffer buffer;
// Writer<StringBuffer> writer(buffer);
// d.Accept(writer);
// std::string str = buffer.GetString();
// Value& modeValue = d["mode"];
// mode.data = modeValue.GetInt();
// m_Pub_mode.publish(mode);
  mode.data = 1;
  ptm.getPTR()->m_Pub_mode.publish(mode);
  ros::spin();
  return 0;
}
