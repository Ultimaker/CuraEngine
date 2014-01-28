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
    
    void sendNr(int nr);
    void sendAll(const void* data, int length);
    int recvNr();
    void recvAll(void* data, int length);
    
    void close();
};

#endif//SOCKET_H
