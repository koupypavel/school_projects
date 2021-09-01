#pragma once
#include "picojson.h"
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

class RosbridgeClient
{
public:
    RosbridgeClient(std::string host, std::string port);
    ~RosbridgeClient(void);
    void EstablishConnection(); 
    void receiverThread();
    void SendMsg(std::string s);

    void publish(const std::string& topic, const std::string& msg);
    void publish(const std::string& topic, const std::string& type, const picojson::value& msg);
    void dispatch(std::string s);
    void subscribe(const std::string& topic, std::string type) ;
    void wait();
    void spin(); 
    void run();
    void ShutdownConnection() ;



public:
    std::string host;
    std::string port;

private:
    WSADATA wsaData;
    SOCKET sockfd;
    struct addrinfo *result;
    struct addrinfo *ptr;
    bool newData;
private:
};

