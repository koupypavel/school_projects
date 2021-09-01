#pragma once
#include "libssh\libssh.h"
#include <string>
#include <vector>

class SSHClient
{
public:
    SSHClient(std::string host, std::string user_name, std::string user_pass);
    ~SSHClient(void);
    bool Connect();
    void Disconnect();
    std::string SendCmd(unsigned int channel_handle, std::string cmd);
    bool OpenShell(unsigned int channel_handle);
    bool SendShellCmd(unsigned int channel_handle, std::string cmd);
    std::string ReadShell(unsigned int channel_handle);
    unsigned int EstablishChannel();
    void CloseChannel(unsigned int handle);


private:
    void ErrorReport(std::string function, int ssh_err);

private:
    std::string host_ip;
    std::string user_name;
    std::string user_pass;
    ssh_session my_ssh_session;
    std::vector <ssh_channel> channels;
    int result;
    int err;
    bool connected;
};
