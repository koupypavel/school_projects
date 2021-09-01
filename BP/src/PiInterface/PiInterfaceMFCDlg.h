// PiInterfaceMFCDlg.h : header file
//

#pragma once
#include <vector>
#include "afxcmn.h"
#include "MotionControlTab.h"
#include "SystemControlTab.h"
#include "Helper.h"
#include <thread>
#include "SSHClient.h"

// CPiInterfaceMFCDlg dialog
class CPiInterfaceMFCDlg : public CDialogEx
{
    // Construction
public:
    CPiInterfaceMFCDlg(CWnd* pParent = NULL);	// standard constructor

    // Dialog Data
    enum { IDD = IDD_PIINTERFACEMFC_DIALOG };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

private:
    std::thread gui_status;
    std::atomic<bool> thr_GUIstatus_run;
public:
    void WatchStatusRun();
    void StatusUpdate(std::atomic<bool> &run);
    void ProcessShellOutput(std::string output);
    void ResizeWindow();
    void ResizeWindow(bool restore = false);

    CMotionControlTab *motionControlTab;
    CSystemControlTab *systemControlTab;
    std::vector<CDialogEx*> m_tabPages;
    // Implementation
protected:
    HICON m_hIcon;

    // Generated message map functions
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    DECLARE_MESSAGE_MAP()
public:
    command_base cmd_base;
    config_base cfg_base;
    CTabCtrl tab_control;
    //    afx_msg void OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnTcnSelchangingTabControl(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnSelchangeTabControl(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnExit();
};
