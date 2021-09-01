#pragma once
#include "afxcmn.h"
#include "VLCWrapper.h"
#include <thread>
#include "afxwin.h"
#include "Helper.h"
#include <thread>
#include "SSHClient.h"

// CMotionControlTab dialog
#define MAX_BUTTONS		128
class CMotionControlTab : public CDialogEx
{
    DECLARE_DYNAMIC(CMotionControlTab)

public:
    CMotionControlTab(CWnd* pParent = NULL);   // standard constructor
    virtual ~CMotionControlTab();
    void setConfigBase(config_base &base);
    void setCmdBase(command_base &base);
    void ControlUpdate(std::atomic<bool> &run,std::atomic<bool> &isJoystickUsed);


    // Dialog Data
    enum { IDD = IDD_MOTIONCONTROLTAB };
protected:
    std::thread thr_motorCtrl;
    std::atomic<bool> thr_motorCtrl_run;
    std::atomic<bool> motorSpeed_changed;
    std::atomic<int> motorCtrl_cmd;
    std::atomic<int> motorSpeedFL;
    std::atomic<int> motorSpeedFR;
    std::atomic<int> motorSpeedBL;
    std::atomic<int> motorSpeedBR;
    std::atomic<int> autoCtrl_cmd;
    std::atomic<bool> isJoystickUsed;
private:
    config_base *cfg_base;
    command_base *cmd_base;
    VLCWrapper  vlcPlayer_;
    std::string CreateCtrlCmd();

    //Joystick Condition Variables
    int m_x;     //X pos of XY box
    int m_y;     //Y pos of XY box
    int m_zr;    //X pos of XY box
    int m_z;     //Y pos of XY box
    int m_h;	 //hat switch number pressed

    double m_scaleX;  //Scale in X direction
    double m_scaleY;  //Scale in Y direction
protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnRawInput(UINT nInputcode, HRAWINPUT hRawInput);
    int ParseRawInput(PRAWINPUT pRawInput);

    UINT g_NumberOfButtons;          //Number of numerated buttons
    BOOL bButtonStates[MAX_BUTTONS]; //Array of numerated buttons conditions
    afx_msg void OnTcnSelchangeTabSensors(NMHDR *pNMHDR, LRESULT *pResult);
    afx_msg void OnTcnSelchangingTabSensors(NMHDR *pNMHDR, LRESULT *pResult);
    virtual BOOL OnInitDialog();
    CStatic vlcControl_;
    afx_msg void OnBnClickedButtonStreamPlay();
    afx_msg void OnBnClickedButtonStreamStop();
    afx_msg void OnBnClickedButtonFoward();
    afx_msg void OnBnClickedButtonRight();
    afx_msg void OnBnClickedButtonLeft();
    afx_msg void OnBnClickedButtonBackward();
    afx_msg void OnBnClickedButtonRotcc();
    afx_msg void OnBnClickedButtonRotc();
    CSliderCtrl m_sliderFLSpeed;
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);

    CSliderCtrl m_sliderFRSpeed;
    CSliderCtrl m_sliderBLSpeed;
    CSliderCtrl m_sliderBRSpeed;

    CEdit m_editBRSpeed;
    CEdit m_editBLSpeed;
    CEdit m_editFRSpeed;
    CEdit m_editFLSpeed;
    CEdit m_editULTSensor;
    CEdit m_editWhiskers;
    CEdit m_editAutoStatus;
    CEdit m_editAutoLevel;

    CEdit m_editJoystickXaxis;
    CEdit m_editJoystickYaxis;

    virtual LRESULT WindowProc(UINT message, WPARAM wParam, LPARAM lParam);

    afx_msg void OnBnClickedButtonCameraConfig();
    afx_msg void OnBnClickedStatus();
    afx_msg void OnBnClickedControlRun();
    afx_msg void OnBnClickedStop();
    CEdit m_editSLAMstate;
    CButton m_checkButtonControl;
    CButton m_checkButtonGUIStatus;
    CButton m_checkButtonAutoControl;
    afx_msg void OnBnClickedButtonShowCamera();
    CButton m_checkButtonShowCamera;
    afx_msg void OnBnClickedButtonAutoRun();
};
