/**
*  @file    SystemControlTab.cpp
*  @author  Pavel Koupy (xkoupy00)
*  @date    10.5.2018
*  @version 1.0
*
*  @brief Trida obstarava obrazovku pro ovladani operacniho systemu
*
*/
#include "stdafx.h"
#include "PiInterfaceMFC.h"
#include "SystemControlTab.h"
#include "afxdialogex.h"
#include <vector>
#include <iostream>
#include <iomanip>
#include <ctime>
// CSystemControlTab dialog
bool connected = false;
IMPLEMENT_DYNAMIC(CSystemControlTab, CDialogEx)

    CSystemControlTab::CSystemControlTab(CWnd* pParent /*=NULL*/)
    : CDialogEx(CSystemControlTab::IDD, pParent)
{
    //napevno prirazene kanaly
    ch_handles.handle_autonomous_mode = -1;
    ch_handles.handle_cmd1 = -1;
    ch_handles.handle_cmd2 = -1;
    ch_handles.handle_cmd2 = -1;
    ch_handles.handle_cmd3 = -1;
    ch_handles.handle_cmd4 = -1;
    ch_handles.handle_cmd5 = -1;
    ch_handles.handle_cmd6 = -1;
    ch_handles.handle_cmd7 = -1;
    ch_handles.handle_console = -1;
}

/**
*   @brief
*   @return int
*/
CSystemControlTab::~CSystemControlTab()
{
}

void CSystemControlTab::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_LIST1, m_listLogConsole);
    DDX_Control(pDX, IDC_CONSOLE_INPUT, m_editConsoleCmd);
}

BEGIN_MESSAGE_MAP(CSystemControlTab, CDialogEx)
    ON_BN_CLICKED(IDC_BUTTON15, &CSystemControlTab::OnBnClickedButtonSSHConnect)
    ON_BN_CLICKED(IDC_BUTTON16, &CSystemControlTab::OnBnClickedButtonSSHDisconnect)
    ON_BN_CLICKED(IDC_BUTTON7, &CSystemControlTab::OnBnClickedUltSensorRun)
    ON_BN_CLICKED(IDC_BUTTON14, &CSystemControlTab::OnBnClickedRoscoreLaunch)
    ON_BN_CLICKED(IDC_BUTTON17, &CSystemControlTab::OnBnClickedDebugOutput)
    ON_BN_CLICKED(IDC_BUTTON3, &CSystemControlTab::OnBnClickedRoscoreShutdown)
    ON_BN_CLICKED(IDC_BUTTON19, &CSystemControlTab::OnBnClickedVNCViewer)
    ON_BN_CLICKED(IDC_BUTTON4, &CSystemControlTab::OnBnClickedPSaxu)
    ON_BN_CLICKED(IDC_BUTTON5, &CSystemControlTab::OnBnClickedSystemReboot)
    ON_BN_CLICKED(IDC_BUTTON6, &CSystemControlTab::OnBnClickedSystemShutdown)
    ON_BN_CLICKED(IDC_BUTTON8, &CSystemControlTab::OnBnClickedNodeCamera)
    ON_BN_CLICKED(IDC_BUTTON9, &CSystemControlTab::OnBnClickedNodeWhiskers)
    ON_BN_CLICKED(IDC_BUTTON10, &CSystemControlTab::OnBnClickedORBSLAM)
    ON_BN_CLICKED(IDC_BUTTON11, &CSystemControlTab::OnBnClickedNodeAutnomous)
    ON_BN_CLICKED(IDC_BUTTON12, &CSystemControlTab::OnBnClickedNodeArduino)
    ON_BN_CLICKED(IDC_BUTTON18, &CSystemControlTab::OnBnClickedExit)
    ON_BN_CLICKED(IDC_BUTTON13, &CSystemControlTab::OnBnClickedSendCmd)
    ON_BN_CLICKED(IDC_BUTTON28, &CSystemControlTab::OnBnClickedCmd1)
    ON_BN_CLICKED(IDC_BUTTON29, &CSystemControlTab::OnBnClickedCmd2)
    ON_BN_CLICKED(IDC_BUTTON30, &CSystemControlTab::OnBnClickedCmd3)
    ON_BN_CLICKED(IDC_BUTTON31, &CSystemControlTab::OnBnClickedCmd4)
    ON_BN_CLICKED(IDC_BUTTON32, &CSystemControlTab::OnBnClickedCmd5)
    ON_BN_CLICKED(IDC_BUTTON33, &CSystemControlTab::OnBnClickedCmd6)
    ON_BN_CLICKED(IDC_BUTTON34, &CSystemControlTab::OnBnClickedCmd7)
    ON_BN_CLICKED(IDC_BUTTON1, &CSystemControlTab::OnBnClickedROSlaunchAll)
END_MESSAGE_MAP()
/**
*   @brief Tlacitko Connect
*   Vytvori instanci SSHClient, vytvori kanal pro logovaci konzoli
*
*   @return void
*/
void CSystemControlTab::OnBnClickedButtonSSHConnect()
{
    ssh = new SSHClient(cfg_base->network_robot_ip, cfg_base->network_ssh_user, cfg_base->network_ssh_passwd);
    ch_handles.handle_console = ssh->EstablishChannel();
    if(ssh->OpenShell(ch_handles.handle_console))
    {
        Log("Connected");
        connected = true;
    }
}
/**
*   @brief Tlacitko DIsconnect
*   Odpojeni a uvoleneni instance SSHClient
*
*   @return void
*/
void CSystemControlTab::OnBnClickedButtonSSHDisconnect()
{
    if(connected)
    {
        ssh->Disconnect();
        delete ssh;
        connected = false;
        Log("Disconnected");
    }
}
/**
*   @brief Tlacitko Ult.Sensor
*   prikaz pro spusteni ult_sensor_node
*
*   @return void
*/
void CSystemControlTab::OnBnClickedUltSensorRun()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->node_ult_sensor))
        Log(cmd_base->node_ult_sensor+" started...");
}
/**
*   @brief Logovací funkce
*   Vypisuje data z msg s casovou znackou
*
*   @param msg data k vypsani
*   @return void
*/
void CSystemControlTab::Log(std::string msg)
{
    std::stringstream ss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    ss << std::put_time(&tm, "%H:%M:%S") << " >";
    ss << msg;
    m_listLogConsole.InsertString(m_listLogConsole.GetCount(),ss.str().c_str());
    m_listLogConsole.SetTopIndex(m_listLogConsole.GetCount() - 1);
}
/**
*   @brief Tlacitko Roscore Launch
*   spustí Ros jadro
*
*   @return void
*/
void CSystemControlTab::OnBnClickedRoscoreLaunch()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->basic_ros_core_launch))
        Log(cmd_base->basic_ros_core_launch+" started...");
}
/**
*   @brief Tlacitko Debug print
*   vypise data ze vsech napevno definovanych kanalu
*   
*   @return void
*/
void CSystemControlTab::OnBnClickedDebugOutput()
{
    if(ch_handles.handle_console != -1)
        Log(ssh->ReadShell(ch_handles.handle_console));
    if(ch_handles.handle_cmd1 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd1));
    if(ch_handles.handle_cmd2 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd2));
    if(ch_handles.handle_cmd3 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd3));
    if(ch_handles.handle_cmd4 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd4));
    if(ch_handles.handle_cmd5 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd5));
    if(ch_handles.handle_cmd6 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd6));
    if(ch_handles.handle_cmd7 != -1)
        Log(ssh->ReadShell(ch_handles.handle_cmd7));
}
/**
*   @brief Tlacitko Ros shutdown
*   vypnuti ROS jadra   
*
*   @return void
*/
void CSystemControlTab::OnBnClickedRoscoreShutdown()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console,cmd_base->basic_ros_core_shutdown))
        Log(cmd_base->basic_ros_core_shutdown+" started...");
}
/**
*   @brief Tlacitko VNC viewer
*   lokalne spusti vnc viewer pokud je program naistalovany   
*
*   @return void
*/
void CSystemControlTab::OnBnClickedVNCViewer()
{
    ShellExecute(0, 0, cfg_base->general_vnc_viewer.c_str(), cfg_base->network_robot_ip.c_str(), 0, SW_SHOW);
}
/**
*   @brief Tlacitko PSaxu
*   ps axu prikaz 
*
*   @return void
*/
void CSystemControlTab::OnBnClickedPSaxu()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->other_cmd1))
        Log(cmd_base->other_cmd1+" started...");
}
/**
*   @brief Tlacitko Reboot
*   restart systemu   
*
*   @return void
*/
void CSystemControlTab::OnBnClickedSystemReboot()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->basic_reboot))
        Log("rebooting...");
}
/**
*   @brief Tlacitko Shutdown
*   vypnuti systemu   
*
*   @return void
*/
void CSystemControlTab::OnBnClickedSystemShutdown()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->basic_shutdown))
        Log("System is shutting down... wait for green flashes on RPI...");
}
/**
*   @brief Tlacitko Camera
*   spusteni raspicam_node  
*
*   @return void
*/
void CSystemControlTab::OnBnClickedNodeCamera()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->node_camera))
        Log(cmd_base->node_camera+" started...");
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->node_image_transport))
        Log(cmd_base->node_camera+" started...");
}
/**
*   @brief Tlacitko Whiskers sensor
*   spusteni whiskers_sensor_node (dotykovy senzor)
*
*   @return void
*/
void CSystemControlTab::OnBnClickedNodeWhiskers()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->node_whiskers_sensor))
        Log(cmd_base->node_whiskers_sensor+" started...");
}
/**
*   @brief Tlacitko ORB-SLAM
*   spusteni putty s X11 a prikazem pro spusteni
*   ORB_SLAM2 node
*
*   @return void
*/
void CSystemControlTab::OnBnClickedORBSLAM()
{
    ShellExecute(0, 0, "putty.exe", "-ssh -X pavel@192.168.137.55 -pw polda2541 -m ./putty_cmds/cmd_slam -t", 0, SW_SHOW);
}
/**
*   @brief Tlacitko Auto.mode
*   spusteni autonomode_node
*
*   @return void
*/
void CSystemControlTab::OnBnClickedNodeAutnomous()
{
    if(connected && ssh->SendShellCmd(ch_handles.handle_console, cmd_base->node_autonomous_mode))
        Log("Autonomous mode is runnig.");
}
/**
*   @brief Tlacitko Arduino - serial
*   spusteni serioveho rozhrani pro komunikaci s Arduino nodem
*
*   @return void
*/
void CSystemControlTab::OnBnClickedNodeArduino()
{
    ShellExecute(0, 0, "putty.exe", "-ssh pavel@192.168.137.55 -pw polda2541 -m ./putty_cmds/cmd_arduino -t", 0, SW_SHOW);
}
/**
*   @brief EXIT
*
*   @return void
*/
void CSystemControlTab::OnBnClickedExit()
{
    exit(0);
}
/**
*   @brief Zaslani prikazu do shellu(konzole)
*   Vezme prikaz zapsany v editu a zasle jej na kanal 
*   ch_handles.handle_console
*
*   @return void
*/
void CSystemControlTab::OnBnClickedSendCmd()
{
    CString value;
    m_editConsoleCmd.GetWindowText(value);
    if(connected && ssh->SendShellCmd(ch_handles.handle_console,value.GetString()))
        Log(value.GetString());
}
/**
*   @brief  Predani prikazu z ini souboru
*
*   @param  base odkaz na strukturu s prikazy
*   @return void
*/
void CSystemControlTab::setCmdBase(command_base &base)
{
    cmd_base = &base;
}
/**
*   @brief  Predani konfigurace z ini souboru
*
*   @param  base odkaz na strukturu s konfiguraci
*   @return void
*/
void CSystemControlTab::setConfigBase(config_base &base)
{
    cfg_base = &base;
}
/**
*   @brief Tlacitko CMD1
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd1()
{
    if(connected)
    {
        ch_handles.handle_cmd1 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd1))
            if(ssh->SendShellCmd(ch_handles.handle_cmd1, cmd_base->other_cmd1))
                Log(cmd_base->other_cmd1+" started...");
    }
}
/**
*   @brief Tlacitko CMD2
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd2()
{
    if(connected)
    {
        ch_handles.handle_cmd2 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd2))
            if(ssh->SendShellCmd(ch_handles.handle_cmd2, cmd_base->other_cmd2))
                Log(cmd_base->other_cmd2+" started...");
    }
}
/**
*   @brief Tlacitko CMD3
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd3()
{
    if(connected)
    {
        ch_handles.handle_cmd3 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd3))
            if(ssh->SendShellCmd(ch_handles.handle_cmd3, cmd_base->other_cmd3))
                Log(cmd_base->other_cmd3+" started...");
    }
}
/**
*   @brief Tlacitko CMD4
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd4()
{
    if(connected)
    {
        ch_handles.handle_cmd4 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd4))
            if(ssh->SendShellCmd(ch_handles.handle_cmd4, cmd_base->other_cmd4))
                Log(cmd_base->other_cmd4+" started...");
    }
}
/**
*   @brief Tlacitko CMD5
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd5()
{
    if(connected)
    {
        ch_handles.handle_cmd5 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd5))
            if(ssh->SendShellCmd(ch_handles.handle_cmd5, cmd_base->other_cmd5))
                Log(cmd_base->other_cmd5+" started...");
    }
}
/**
*   @brief Tlacitko CMD6
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd6()
{
    if(connected)
    {
        ch_handles.handle_cmd6 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd6))
            if(ssh->SendShellCmd(ch_handles.handle_cmd6, cmd_base->other_cmd6))
                Log(cmd_base->other_cmd6+" started...");
    }
}
/**
*   @brief Tlacitko CMD7
*   zkontroluje pripojeni a vytvori specialni kanal pro tento prikaz
*   zaslani prikazu definovaneho v ini konfigu
*
*   @return void
*/
void CSystemControlTab::OnBnClickedCmd7()
{
    if(connected)
    {
        ch_handles.handle_cmd7 = ssh->EstablishChannel();
        if(ssh->OpenShell(ch_handles.handle_cmd7))
            if(ssh->SendShellCmd(ch_handles.handle_cmd7, cmd_base->other_cmd7))
                Log(cmd_base->other_cmd7+" started...");
    }
}


/**
*   @brief Tlacitko ROS launch all
*   pokusi se spustit vsechny potrebne uzly
*   bez slamu a arduino uzlu
*   @return void
*/
void CSystemControlTab::OnBnClickedROSlaunchAll()
{
    OnBnClickedRoscoreLaunch();
    OnBnClickedUltSensorRun();
    OnBnClickedNodeWhiskers();
    OnBnClickedNodeCamera();
    OnBnClickedNodeAutnomous();
}
