#ifndef SOCKET_H
#define SOCKET_H

#include <string>

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

#endif//SOCKET_H
