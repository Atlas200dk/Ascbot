#include "phone_talker_manager.h"
#include "phone_talker_tcp.h"

CPhoneTalkerTcp::CPhoneTalkerTcp(){
  #ifdef WINVER
  netLink::init();
  #endif

  //m_pPTM = new CPhoneTalkerManager();
  phoneSocket = nullptr;
  pSocketManager = nullptr;

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
      //// Or just access them directly
      //if(elementMap->getLength() >= 2) {
      //  std::cout << "Access them directly:" << std::endl;
      //  std::cout << elementMap->getKey(0)->stdString() << " : " << *elementMap->getValue(0) << std::endl;
      //  std::cout << elementMap->getKey(1)->stdString() << " : " << *elementMap->getValue(1) << std::endl;
      //  // ... and so on
      //}
      if(elementMap->getLength() > 0){
        if(elementMap->getKey(0)->stdString() == "job"){
          std::string strTxt = dynamic_cast<MsgPack::String*>(container.begin()->second)->stdString();
          talk2Ros_Job(strTxt);
        }
        else{
          std::string strTxt = dynamic_cast<MsgPack::String*>(container.begin()->second)->stdString();
          talk2Ros_Op(strTxt);
        }
      }
    }
  };

}

void CPhoneTalkerTcp::setPTM(CPhoneTalkerManager *pPTM){
  m_pPTM = pPTM;
}

bool CPhoneTalkerTcp::talk2Tcp(std::string content){
  bool ret = false;
  netLink::MsgPackSocket& msgPackSocket = *static_cast<netLink::MsgPackSocket*>(phoneSocket.get());
  switch(socket->getStatus()) {
    case netLink::Socket::Status::READY:
      std::cout << "Talk to: " << socket->hostRemote << ":" << socket->portRemote << std::endl;

      // Prepare a MsgPack encoded message
      msgPackSocket << MsgPack__Factory(MapHeader(1));
      msgPackSocket << MsgPack::Factory("msg");
      msgPackSocket << MsgPack::Factory(content);

      std::cout <<"content: " << content << std::endl;

      ret = true;
    break;
    case netLink::Socket::Status::NOT_CONNECTED:
      std::cout << "Lost connection of " << socket->hostRemote << ":" << socket->portRemote << std::endl;
      if(phoneSocket == socket) {
        phoneSocket = nullptr;
        pSocketManager = nullptr;
      }
    break;
    default:
      std::cout << "Status of " << socket->hostRemote << ":" << socket->portRemote << " is on the status -- " << socket->getStatus() << "It's not ready. " << std::endl;
    break;
  }
  return ret;
}

void CPhoneTalkerTcp::run(){
    // Ask user for a nice IP address
    unsigned int nPort = 3823;// todo: Move this to config file.
    //std::cout << "Enter a IP-Adress to connect to a sever or '*' to start a server (Port: " << nPort << "):" << std::endl;
    std::cout << "Server start (Port: " << nPort << "):" << std::endl;
    while(1) {
        try {
//            std::cin >> socket->hostRemote;
//            // Init socket as TCP server or client on port "nPort"
//            if(socket->hostRemote == "*")
//                socket->initAsTcpServer("*", nPort);
//            else
//                socket->initAsTcpClient(socket->hostRemote, nPort);

            socket->initAsTcpServer("*", nPort);

            break;
        } catch(netLink::Exception exc) {
            std::cout << "Address is already in use, please try again..." << std::endl;
        }
    }

    // Let the SocketManager poll from all sockets, events will be triggered here
    while((socket->getStatus() != netLink::Socket::Status::NOT_CONNECTED) && (ros::ok()))
        socketManager.listen();

    std::cout << "Stop talking to phone. " << std::endl;
}

void CPhoneTalkerTcp::talk2Ros_Job(std::string content){
  m_pPTM->getPTR()->talk2Ros_Job(content);
}

void CPhoneTalkerTcp::talk2Ros_Op(std::string content){
  m_pPTM->getPTR()->talk2Ros_Op(content);
}
