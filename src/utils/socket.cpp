#include <stdio.h>
#include <string.h>

#ifdef __WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "socket.h"
#include "logoutput.h"

namespace cura
{

#ifdef __WIN32
bool wsaStartupDone = false;
#endif

ClientSocket::ClientSocket()
{
    sockfd = -1;

#ifdef __WIN32
    if (!wsaStartupDone)
    {
        WSADATA wsaData;
        memset(&wsaData, 0, sizeof(WSADATA));
        //WSAStartup needs to be called on windows before sockets can be used. Request version 1.1, which is supported on windows 98 and higher.
        WSAStartup(MAKEWORD(1, 1), &wsaData);
        wsaStartupDone = true;
    }
#endif
}

void ClientSocket::connectTo(std::string host, int port)
{
    struct sockaddr_in serv_addr;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    serv_addr.sin_addr.s_addr = inet_addr(host.c_str());

    if (connect(sockfd, reinterpret_cast<struct sockaddr*>(&serv_addr), sizeof(serv_addr)) < 0)
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

void ClientSocket::sendInt32(int32_t nr)
{
    sendAll(&nr, sizeof(int32_t));
}

void ClientSocket::sendFloat32(float f)
{
    sendAll(&f, sizeof(float));
}


void ClientSocket::sendAll(const void* data, int length)
{
    if (sockfd == -1)
        return;
    const char* ptr = static_cast<const char*>(data);
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

int32_t ClientSocket::recvInt32()
{
    int32_t ret = -1;
    recvAll(&ret, sizeof(int32_t));
    return ret;
}

float ClientSocket::recvFloat32()
{
    float ret = 0;
    recvAll(&ret, sizeof(float));
    return ret;
}

void ClientSocket::recvAll(void* data, int length)
{
    if (sockfd == -1)
        return;
    char* ptr = static_cast<char*>(data);
    while(length > 0)
    {
        int n = recv(sockfd, ptr, length, 0);
        if (n == 0)
        {
            close();
            return;
        }
        if (n < 0)
        {
            cura::logError("ClientSocket::recvAll error...");
            close();
            return;
        }
        ptr += n;
        length -= n;
    }
}

void ClientSocket::close()
{
    if (sockfd == -1)
        return;
#ifdef __WIN32
    closesocket(sockfd);
#else
    ::close(sockfd);
#endif
    sockfd = -1;
}

}//namespace cura