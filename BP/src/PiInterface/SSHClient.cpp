/**
*  @file    SSHClient.cpp
*  @author  Pavel Koupy (xkoupy00)
*  @date    10.5.2018
*  @version 1.0
*
*  @brief SSH rozhrani pro pripojeni k vozitku.
*
*/
#include "stdafx.h"
#include "SSHClient.h"
#include <sstream>
#include <errno.h>
#include <vector>
#include <stdio.h>
#include <algorithm>
#include <cctype>
#include <stdlib.h>
#include <functional>

/**
*   @brief Konstruktor
*
*   @param host ip-adresa
*   @param user login
*   @param pass heslo
*/
SSHClient::SSHClient(std::string host, std::string user, std::string pass):
    host_ip(host),
    user_name(user),
    user_pass(pass),
    result(0),
    connected(false)
{
    int verbosity = SSH_LOG_PROTOCOL;
    int port = 22;

    my_ssh_session = ssh_new();
    if (my_ssh_session == NULL)
        AfxMessageBox("SSHClient ERROR", MB_OK | MB_ICONERROR);
    // Nastaveni session (login, pass, log typ, port)
    ssh_options_set(my_ssh_session, SSH_OPTIONS_HOST, host_ip.c_str());
    ssh_options_set(my_ssh_session, SSH_OPTIONS_USER, user_name.c_str());
    ssh_options_set(my_ssh_session, SSH_OPTIONS_LOG_VERBOSITY, &verbosity);
    ssh_options_set(my_ssh_session, SSH_OPTIONS_PORT, &port);
    // Pripojeni
    Connect();

    // autentizace
    result = ssh_userauth_password(my_ssh_session, NULL, user_pass.c_str());
    if (result != SSH_AUTH_SUCCESS)
    {
        AfxMessageBox("SSHClient authentication ERROR", MB_OK | MB_ICONERROR);
    }
}
/**
*   @brief  Pripojeni k serveru
*
*   @return bool
*/
bool SSHClient::Connect()
{
    int result = ssh_connect(my_ssh_session);
    if (result != SSH_OK)
    {
        //log gui popup
        AfxMessageBox("SSHClient connection ERROR", MB_OK | MB_ICONERROR);
        return false;
    }
    return true;
}
/**
*   @brief  Odpojeni od serveru
*
*   @return void
*/
void SSHClient::Disconnect()
{
    ssh_disconnect(my_ssh_session);
}
/**
*   @brief  Vytvoreni kanalu
*   Vytvari nove kanaly a vklada je do vektoru
*
*   @return unsignet int
*/
unsigned int SSHClient::EstablishChannel()
{
    ssh_channel channel = ssh_channel_new(my_ssh_session);
    if (channel == NULL)
    {
        AfxMessageBox("SSHClient"+SSH_ERROR, MB_OK | MB_ICONERROR);
        return false;
    }
    channels.push_back(channel);
    return channels.size() - 1;
}
/**
*   @brief  Ukonceni kanalu
*   @param handle cislo kanalu
*   @return void
*/
void SSHClient::CloseChannel(unsigned int handle)
{
    //zaslani EOF, uzavreni a uvolneni
    ssh_channel_send_eof(channels[handle]);
    ssh_channel_close(channels[handle]);
    ssh_channel_free(channels[handle]);
}
/**
*   @brief  Chybove vypisy
*
*   @return void
*/
void SSHClient::ErrorReport(std::string function, int ssh_err)
{
    std::stringstream ss;

    ss << "SSHClient: " << function << " =>" << ssh_err;

    AfxMessageBox(ss.str().c_str(), MB_OK | MB_ICONERROR);
}
/**
*   @brief Otevreni shellu
*   umoznuje souvisle posilat prikazy
*
*   @param channel_handle cislo kanalu
*   @return bool
*/
bool SSHClient::OpenShell(unsigned int channel_handle)
{
    int rc;
    std::vector<char> buffer;
    std::vector<std::string> output;
    buffer.resize(4096, 0);
    int nbytes;

    rc = ssh_channel_open_session(channels[channel_handle]);
    if (rc != SSH_OK)
    {
        ErrorReport("SendMsg()", SSH_ERROR);
    }

    rc = ssh_channel_request_pty(channels[channel_handle]);
    if (rc != SSH_OK) return false;
    rc = ssh_channel_change_pty_size(channels[channel_handle], 80, 24);
    if (rc != SSH_OK) return false;
    rc = ssh_channel_request_shell(channels[channel_handle]);
    if (rc != SSH_OK) return false;
    bool first = false;
    if(ssh_channel_is_open(channels[channel_handle]))
    {
        nbytes = ssh_channel_read(channels[channel_handle], &buffer[0], buffer.size(), 0);
    }

    return (nbytes > 0) ? true : false;
}
/**
*   @brief zaslani jednoho prikazu do shellu
*
*   @param channel_handle cislo kanalu
*   @param cmd prikaz zakonceny "\n"
*   @return bool
*/
bool SSHClient::SendShellCmd(unsigned int channel_handle, std::string cmd)
{
    int nwrite =0;
    cmd.append(" \n");
    if(ssh_channel_is_open(channels[channel_handle]))
    {
        nwrite = ssh_channel_write(channels[channel_handle], &cmd[0], cmd.size());
    }
    return (nwrite > 0) ? true : false;
}
/**
*   @brief Cteni z shellu
*   obsahuje i formatovaci retezce, takze vypis do gui
*   je vcetne techto formatovacich znaku
*
*   @param channel_handle cislo kanalu
*   @return std::string
*/
std::string SSHClient::ReadShell(unsigned int channel_handle)
{
    std::vector<char> buffer;
    buffer.resize(4096, 0);
    int nbytes =0;
    if(ssh_channel_is_open(channels[channel_handle]))
    {
        nbytes = ssh_channel_read_nonblocking(channels[channel_handle], &buffer[0], buffer.size(), 0);
    }
    std::string s;
    if(nbytes > 0)
    {
        buffer.assign(&buffer[0], &buffer[nbytes]);
        s =std::string(&buffer[0]);
    }
    return s;
}
/**
*   @brief Zaslani jednohoo prikazu
*   bez nutnosti otevirat shell a vycteni
*   odpovedi
*
*   @param channel_handle cislo kanalu
*   @param cmd prikaz
*   @return std::string
*/
std::string SSHClient::SendCmd(unsigned int channel_handle,std::string cmd)
{
    int rc;
    int nbytes;
    std::vector<char> buffer;
    buffer.resize(4096, 0);

    rc = ssh_channel_open_session(channels[channel_handle]);
    if (rc != SSH_OK)
        ErrorReport("SendMsg()", SSH_ERROR);

    rc = ssh_channel_request_exec(channels[channel_handle], cmd.c_str());
    if (rc != SSH_OK)
        ErrorReport("SendMsg()", SSH_ERROR);

    nbytes = ssh_channel_read(channels[channel_handle], &buffer[0], buffer.size(), 0);

    if (nbytes < 0)
        ErrorReport("SendMsg()-no response", SSH_ERROR);

    return std::string(&buffer[0]);
}
/**
*   @brief  Destruktor
*/
SSHClient::~SSHClient(void)
{
}