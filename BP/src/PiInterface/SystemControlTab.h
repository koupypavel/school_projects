#pragma once
#include <vector>
#include <string>
#include <sstream>
#include "SSHClient.h"
#include "afxcmn.h"
#include "afxwin.h"
#include "Helper.h"

// CSystemControlTab dialog

typedef struct
{
    int handle_console;
    int handle_gui_status;
    int handle_autonomous_mode;
    int handle_cmd1;
    int handle_cmd2;
    int handle_cmd3;
    int handle_cmd4;
    int handle_cmd5;
    int handle_cmd6;
    int handle_cmd7;
}channel_handle;

class CSystemControlTab : public CDialogEx
{
    DECLARE_DYNAMIC(CSystemControlTab)

public:
    CSystemControlTab(CWnd* pParent = NULL);   // standard constructor
    virtual ~CSystemControlTab();
    void Log(std::string msg);
    void setCmdBase(command_base &base);
    void setConfigBase(config_base &base);

    // Dialog Data
    enum { IDD = IDD_SYSTEMCONTROLTAB };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
private:
    SSHClient *ssh;
    command_base *cmd_base;
    config_base *cfg_base;
    channel_handle ch_handles;

public:
    //    afx_msg void OnBnClickedButton6();
    afx_msg void OnBnClickedButtonSSHConnect();
    afx_msg void OnBnClickedButtonSSHDisconnect();
    //    CListCtrl m_listLogConsole;
    CListBox m_listLogConsole;
    afx_msg void OnBnClickedUltSensorRun();
    afx_msg void OnBnClickedRoscoreLaunch();
    afx_msg void OnBnClickedDebugOutput();
    afx_msg void OnBnClickedRoscoreShutdown();
    afx_msg void OnBnClickedVNCViewer();
    afx_msg void OnBnClickedPSaxu();
    afx_msg void OnBnClickedSystemReboot();
    afx_msg void OnBnClickedSystemShutdown();
    afx_msg void OnBnClickedNodeCamera();
    afx_msg void OnBnClickedNodeWhiskers();
    afx_msg void OnBnClickedORBSLAM();
    afx_msg void OnBnClickedNodeAutnomous();
    afx_msg void OnBnClickedNodeArduino();
    afx_msg void OnBnClickedExit();
    afx_msg void OnBnClickedSendCmd();
    CEdit m_editConsoleCmd;
    afx_msg void OnBnClickedCmd1();
    afx_msg void OnBnClickedCmd2();
    afx_msg void OnBnClickedCmd3();
    afx_msg void OnBnClickedCmd4();
    afx_msg void OnBnClickedCmd5();
    afx_msg void OnBnClickedCmd6();
    afx_msg void OnBnClickedCmd7();
//    afx_msg void OnBnClickedButton1();
    afx_msg void OnBnClickedROSlaunchAll();
};
