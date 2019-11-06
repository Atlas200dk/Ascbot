#include "phone_talker_manager.h"
#include "phone_talker_tcp.h"
#include <streambuf>
using namespace rapidjson;
CPhoneTalkerTcp::CPhoneTalkerTcp(){
  //m_pPTM = new CPhoneTalkerManager();
  phoneSocket = socketManager.newMsgPackSocket();
  pSocketManager = new netLink::SocketManager();
  std::thread thrRun(&CPhoneTalkerTcp::run, this);
  thrRun.detach();
}

void CPhoneTalkerTcp::setPTM(CPhoneTalkerManager *pPTM){
  m_pPTM = pPTM;
}

bool CPhoneTalkerTcp::talk2Tcp(std::string frame, std::string content){
  bool ret = false;
  netLink::MsgPackSocket& msgPackSocket = *static_cast<netLink::MsgPackSocket*>(phoneSocket.get());
  switch(/*socket*/phoneSocket->getStatus()) {
    case netLink::Socket::Status::READY:
      std::cout << "Talk to: " << /*socket*/phoneSocket->hostRemote << ":" << /*socket*/phoneSocket->portRemote << std::endl;

      msgPackSocket << MsgPack__Factory(MapHeader(1));
      msgPackSocket << MsgPack::Factory("msg");
      msgPackSocket << MsgPack::Factory(content);
      std::cout <<"content: " << content << std::endl;
      ret = true;
    break;
    case netLink::Socket::Status::NOT_CONNECTED:
      std::cout << "Lost connection of " << /*socket*/phoneSocket->hostRemote << ":" << /*socket*/phoneSocket->portRemote << std::endl;
      // todo ?
      //if(phoneSocket == socket) {
      //  phoneSocket = nullptr;
      //  pSocketManager = nullptr;
      //}
    break;
    default:
      std::cout << "Status of " << /*socket*/phoneSocket->hostRemote << ":" << /*socket*/phoneSocket->portRemote << " is on the status -- " << /*socket*/phoneSocket->getStatus() << ". It's not ready. " << std::endl;
    break;
  }
  return ret;
}

void CPhoneTalkerTcp::run(){
  #ifdef WINVER
  netLink::init();
  #endif

  //netLink::SocketManager socketManager;

  // Allocate a new socket and insert it into the SocketManager
  //std::shared_ptr<netLink::Socket>
  socket = socketManager.newMsgPackSocket();

  // Define a callback, fired when a new client tries to connect
  socketManager.onConnectRequest = [this](netLink::SocketManager* manager, std::shared_ptr<netLink::Socket> serverSocket, std::shared_ptr<netLink::Socket> clientSocket) {
    std::cout << "Accepted connection from " << clientSocket->hostRemote << ":" << clientSocket->portRemote << std::endl;
    // regist
    this->phoneSocket = clientSocket;
    this->pSocketManager = manager;

    // Accept all new connections
    return true;
  };

  // Define a callback, fired when a sockets state changes
  socketManager.onStatusChange = [this](netLink::SocketManager* manager, std::shared_ptr<netLink::Socket> socket, netLink::Socket::Status prev) {
    netLink::MsgPackSocket& msgPackSocket = *static_cast<netLink::MsgPackSocket*>(socket.get());

    switch(socket->getStatus()) {
      case netLink::Socket::Status::READY:
        std::cout << "Connection got accepted at " << socket->hostRemote << ":" << socket->portRemote << std::endl;

        // Prepare a MsgPack encoded message
        msgPackSocket << MsgPack__Factory(MapHeader(2));
        msgPackSocket << MsgPack::Factory("type");
        msgPackSocket << MsgPack::Factory("tcp");
        msgPackSocket << MsgPack::Factory("message");
        msgPackSocket << MsgPack::Factory("Connected!");
      break;
      case netLink::Socket::Status::NOT_CONNECTED:
        if(prev == netLink::Socket::Status::CONNECTING){
          std::cout << "Connecting to " << socket->hostRemote << ":" << socket->portRemote << " failed" << std::endl;
          if(this->phoneSocket == socket) {
            this->phoneSocket = nullptr;
            this->pSocketManager = nullptr;
          }
        }
        else{
        std::cout << "Lost connection of " << socket->hostRemote << ":" << socket->portRemote << std::endl;
          if(this->phoneSocket == socket){
            this->phoneSocket = nullptr;
            this->pSocketManager = nullptr;
          }
        }
      break;
      default:
        std::cout << "Status of " << socket->hostRemote << ":" << socket->portRemote << " changed from " << prev << " to " << socket->getStatus() << std::endl;
      break;
    }
  };

  // Define a callback, fired when a socket receives data
  socketManager.onReceiveMsgPack = [this](netLink::SocketManager* manager, std::shared_ptr<netLink::Socket> socket, std::unique_ptr<MsgPack::Element> element) {
    // hostRemote and portRemote are now set to the origin of the last received message
    std::cout << "Received data from " << socket->hostRemote << ":" << socket->portRemote << ": " << *element << std::endl;

    // Parse the *element
    auto elementMap = dynamic_cast<MsgPack::Map*>(element.get());
    if(elementMap) { // The received element is a map
      // Iterate them sequentially
      std::cout << "Iterate them sequentially:" << std::endl;
      auto container = elementMap->getElementsMap();
      for(auto& pair : container)
        std::cout << pair.first << " : " << *pair.second << std::endl;

      if(elementMap->getLength() > 0){
//        if(elementMap->getKey(0)->stdString() == "jobs"){
//          std::string strTxt = dynamic_cast<MsgPack::String*>(container.begin()->second)->stdString();
//          talk2Ros_Job(strTxt);
//        }
        if(elementMap->getKey(0)->stdString() == "cmd"){
          std::string strTxt = dynamic_cast<MsgPack::String*>(container.begin()->second)->stdString();
          talk2Ros_Op(strTxt);
        }
        if(elementMap->getKey(0)->stdString() == "remote"){
		  std::string strTxt = dynamic_cast<MsgPack::String*>(container.begin()->second)->stdString();
		  talk2Ros_Remote(strTxt);
        }
		if(elementMap->getKey(0)->stdString() == "currentMode"){//get current mode
			  Document dd;
				  dd.SetObject();
				  rapidjson::Document::AllocatorType& allocator = dd.GetAllocator();
				  currentMode =  m_pPTM->getPTR()->getCurrentMode();
				  Value mode(currentMode);
				  dd.AddMember("currentMode",mode,allocator );
				  Value des(StringRef("mode"));
				  dd.AddMember("type",des,allocator );
				  StringBuffer buffer1;
				  Writer<StringBuffer> writer1(buffer1);
				  dd.Accept(writer1);
				  talk2Tcp("mode_result",buffer1.GetString());//返回切换模式成功的响应
		}
      }

    }
  };



    // Ask user for a nice IP address
    unsigned int nPort = 3823;// todo: Move this to config file.
    //std::cout << "Enter a IP-Adress to connect to a sever or '*' to start a server (Port: " << nPort << "):" << std::endl;
    std::cout << "Server start (Port: " << nPort << "):" << std::endl;
    while(1) {
        try {
            socket->initAsTcpServer("*", nPort);
            break;
        } catch(netLink::Exception exc) {
            std::cout << "Address is already in use, please try again..." << std::endl;
        }
    }

    // Let the SocketManager poll from all sockets, events will be triggered here
    while((socket->getStatus() != netLink::Socket::Status::NOT_CONNECTED) && (ros::ok())){
        socketManager.listen();
        usleep(500*1000);
    }

    std::cout << "Stop talking to phone. " << std::endl;
}

//void CPhoneTalkerTcp::talk2Ros_Job(std::string content){
//  m_pPTM->getPTR()->talk2Ros_Job(content);
//}

void CPhoneTalkerTcp::talk2Ros_Op(std::string content){
  m_pPTM->getPTR()->talk2Ros_Op(content);
}
void CPhoneTalkerTcp::talk2Ros_Remote(std::string content){
	m_pPTM->getPTR()->talk2Ros_Remote(content);
}
