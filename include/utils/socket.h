//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SOCKET_H
#define SOCKET_H

#include <string>
namespace cura
{
    
class ClientSocket
{
    int sockfd;
public:
    ClientSocket();
    ~ClientSocket();
    
    void connectTo(std::string host, int port);
    
    void sendInt32(int32_t nr);
    void sendFloat32(float f);
    void sendAll(const void* data, int length);
    
    int32_t recvInt32();
    float recvFloat32();
    void recvAll(void* data, int length);
    
    void close();
};

}//namespace cura
#endif//SOCKET_H
