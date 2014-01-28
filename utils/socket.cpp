#include <winsock2.h>
#include <stdio.h>

#include "socket.h"

#ifdef WIN32
bool wsaStartupDone = false;
#endif

ClientSocket::ClientSocket()
{
    sockfd = -1;
}

void ClientSocket::connectTo(std::string host, int port)
{
    struct sockaddr_in serv_addr;
#ifdef WIN32
    if (!wsaStartupDone)
    {
        WSADATA wsaData = {0};
        WSAStartup(MAKEWORD(1, 1), &wsaData);
        wsaStartupDone = true;
    }
#endif
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
    memset(&serv_addr, '0', sizeof(serv_addr)); 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(host.c_str());
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("Connect to %s:%d failed\n", host.c_str(), port);
        close();
        return;
    }
}

ClientSocket::~ClientSocket()
{
    close();
}

void ClientSocket::sendNr(int nr)
{
    sendAll(&nr, sizeof(int));
}

void ClientSocket::sendAll(const void* data, int length)
{
    if (sockfd == -1)
        return;
    const char* ptr = (const char*)data;
    while(length > 0)
    {
        int n = send(sockfd, ptr, length, 0);
        if (n <= 0)
        {
            close();
            return;
        }
        ptr += length;
        length -= n;
    }
}

int ClientSocket::recvNr()
{
    int ret = 0;
    recvAll(&ret, 4);
    return ret;
}

void ClientSocket::recvAll(void* data, int length)
{
    if (sockfd == -1)
        return;
    char* ptr = (char*)data;
    while(length > 0)
    {
        int n = recv(sockfd, ptr, length, 0);
        if (n <= 0)
        {
            close();
            return;
        }
        ptr += length;
        length -= n;
    }
}

void ClientSocket::close()
{
    if (sockfd == -1)
        return;
#ifdef WIN32
    closesocket(sockfd);
#else
    close(sockfd);
#endif
    sockfd = -1;
}
