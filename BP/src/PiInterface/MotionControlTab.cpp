/**
*  @file    MotionControlTab.cpp
*  @author  Pavel Koupy (xkoupy00)
*  @date    10.5.2018
*  @version 1.0
*
*  @brief Trida obstarava obrazovku pro vzdalenou kontrolu
*
*/

#include "stdafx.h"
#include "PiInterfaceMFC.h"
#include "MotionControlTab.h"
#include "afxdialogex.h"

#include <string>
#include <vlc/vlc.h>
#include <sstream>

extern "C"
{
#include <Hidsdi.h>
}

//definice pro cteni raw inputu joysticku
#define SAFE_FREE(p)	{ if(p) { HeapFree(hHeap, 0, p); (p) = NULL; } }

static HANDLE               hHeap;
static 	PHIDP_PREPARSED_DATA pPreparsedData;
static 	PHIDP_BUTTON_CAPS    pButtonCaps;
static 	PHIDP_VALUE_CAPS     pValueCaps;
static USAGE    usage[MAX_BUTTONS];

int m_nPress = 0;

int  SaveFree(BOOL bRslt)
{
    SAFE_FREE(pPreparsedData);
    SAFE_FREE(pButtonCaps);
    SAFE_FREE(pValueCaps);
    if (!bRslt)
        m_nPress = 0;
    return bRslt;
}

// CMotionControlTab dialog

IMPLEMENT_DYNAMIC(CMotionControlTab, CDialogEx)

    CMotionControlTab::CMotionControlTab(CWnd* pParent /*=NULL*/)
    : CDialogEx(CMotionControlTab::IDD, pParent),
    thr_motorCtrl_run(false)
{
}

CMotionControlTab::~CMotionControlTab()
{
}

//prirazeni promennych grafickym prvkum
void CMotionControlTab::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_STATIC_VLC, vlcControl_);
    DDX_Control(pDX, IDC_SLIDER_FL_SPEED, m_sliderFLSpeed);
    DDX_Control(pDX, IDC_EDIT_FL_SPEED, m_editFLSpeed);
    DDX_Control(pDX, IDC_EDIT_FR_SPEED, m_editFRSpeed);
    DDX_Control(pDX, IDC_SLIDER_FR_SPEED, m_sliderFRSpeed);
    DDX_Control(pDX, IDC_EDIT_BL_SPEED, m_editBLSpeed);
    DDX_Control(pDX, IDC_SLIDER_BL_SPEED, m_sliderBLSpeed);
    DDX_Control(pDX, IDC_EDIT_BR_SPEED, m_editBRSpeed);
    DDX_Control(pDX, IDC_SLIDER_BR_SPEED, m_sliderBRSpeed);
    DDX_Control(pDX, IDC_EDIT3, m_editULTSensor);
    DDX_Control(pDX, IDC_EDIT2, m_editWhiskers);
    DDX_Control(pDX, IDC_EDIT1, m_editAutoStatus);
    DDX_Control(pDX, IDC_EDIT4, m_editAutoLevel);
    DDX_Control(pDX, IDC_EDIT5, m_editJoystickXaxis);
    DDX_Control(pDX, IDC_EDIT6, m_editJoystickYaxis);
    DDX_Control(pDX, IDC_EDIT7, m_editSLAMstate);
    DDX_Control(pDX, IDC_CHECK2, m_checkButtonControl);
    DDX_Control(pDX, IDC_CHECK1, m_checkButtonGUIStatus);
    DDX_Control(pDX, IDC_CHECK3, m_checkButtonAutoControl);
    DDX_Control(pDX, IDC_CHECK4, m_checkButtonShowCamera);
}
//smerovani sprav
BEGIN_MESSAGE_MAP(CMotionControlTab, CDialogEx)
    ON_BN_CLICKED(IDC_BUTTON_STREAM_PLAY, &CMotionControlTab::OnBnClickedButtonStreamPlay)
    ON_BN_CLICKED(IDC_BUTTON_STREAM_STOP, &CMotionControlTab::OnBnClickedButtonStreamStop)
    ON_BN_CLICKED(IDC_BUTTON_FOWARD, &CMotionControlTab::OnBnClickedButtonFoward)
    ON_BN_CLICKED(IDC_BUTTON_RIGHT, &CMotionControlTab::OnBnClickedButtonRight)
    ON_BN_CLICKED(IDC_BUTTON_LEFT, &CMotionControlTab::OnBnClickedButtonLeft)
    ON_BN_CLICKED(IDC_BUTTON_BACKWARD, &CMotionControlTab::OnBnClickedButtonBackward)
    ON_BN_CLICKED(IDC_BUTTON_ROTCC, &CMotionControlTab::OnBnClickedButtonRotcc)
    ON_BN_CLICKED(IDC_BUTTON_ROTC, &CMotionControlTab::OnBnClickedButtonRotc)
    ON_WM_HSCROLL()
    ON_BN_CLICKED(IDC_BUTTON_STREAM_STOP2, &CMotionControlTab::OnBnClickedButtonCameraConfig)
    ON_BN_CLICKED(IDC_BUTTON2, &CMotionControlTab::OnBnClickedStatus)
    ON_BN_CLICKED(IDC_CONTROL_RUN, &CMotionControlTab::OnBnClickedControlRun)
    ON_BN_CLICKED(IDC_BUTTON_LEFT2, &CMotionControlTab::OnBnClickedStop)
    ON_BN_CLICKED(IDC_BUTTON_SHOW_CAMERA, &CMotionControlTab::OnBnClickedButtonShowCamera)
    ON_BN_CLICKED(IDC_BUTTON_AUTO_RUN, &CMotionControlTab::OnBnClickedButtonAutoRun)
END_MESSAGE_MAP()

static void HandleVLCEvents(const VLCEvent* pEvt, void* pUserData)
{
    CMotionControlTab* pDlg = reinterpret_cast<CMotionControlTab*>(pUserData);
}
/**
*   @brief  Initializace
*
*   @param  argc pocet argumentu
*   @param  argv pole s argumenty
*   @return int
*/
BOOL CMotionControlTab::OnInitDialog()
{
    CDialogEx::OnInitDialog();
    //inicializace vlc prehravace
    vlcPlayer_.SetOutputWindow((void*)vlcControl_.GetSafeHwnd());
    vlcPlayer_.SetEventHandler(&HandleVLCEvents, this);
    //nastaveni cesty
    vlcPlayer_.OpenMedia((LPCTSTR)"");

    /////////////////////////////////////////////////////////////////////////////////////////
    //inicializace slideru pro rychlost
    motorSpeedFL = 255;
    m_sliderFLSpeed.SetRange(0, 255, TRUE);
    m_sliderFLSpeed.SetPos(motorSpeedFL);
    m_editFLSpeed.SetWindowTextA(std::to_string(motorSpeedFL).c_str());

    motorSpeedFR = 255;
    m_sliderFRSpeed.SetRange(0, 255, TRUE);
    m_sliderFRSpeed.SetPos(motorSpeedFR);
    m_editFRSpeed.SetWindowTextA(std::to_string(motorSpeedFR).c_str());

    motorSpeedBL = 255;
    m_sliderBLSpeed.SetRange(0, 255, TRUE);
    m_sliderBLSpeed.SetPos(motorSpeedBL);
    m_editBLSpeed.SetWindowTextA(std::to_string(motorSpeedBL).c_str());

    motorSpeedBR = 255;
    m_sliderBRSpeed.SetRange(0, 255, TRUE);
    m_sliderBRSpeed.SetPos(motorSpeedBR);
    m_editBRSpeed.SetWindowTextA(std::to_string(motorSpeedBR).c_str());

    //status auto. modu, ult. senzoru, dotekoveho senzoru
    m_editAutoStatus.SetWindowTextA("Manual");
    m_editULTSensor.SetWindowTextA("25 cm");
    m_editWhiskers.SetWindowTextA("false");
    //viditelne okno pro kameru -- check button
    m_checkButtonShowCamera.SetCheck(1);
    //system startuje v manualnim rezimu a nechci, aby se vozitko zacalo pohybovat p startu
    autoCtrl_cmd = 0;
    //prikazy pro zmenu rychlosti posilam pouze pokud je to treba
    motorSpeed_changed = false;
    return TRUE;  // return TRUE unless you set the focus to a control
}
/**
*   @brief  Zpracovani raw vstupu joysticku
*   https://www.codeproject.com/Articles/185522/Using-the-Raw-Input-API-to-Process-Joystick-Input
*   @param  pRawInput vstupni raw data
*   @return int
*/
int CMotionControlTab::ParseRawInput(PRAWINPUT pRawInput)
{
    HIDP_CAPS            Caps;
    USHORT               capsLength;
    UINT                 bufferSize;
    ULONG                i, usageLength, value;

    pPreparsedData = NULL;
    pButtonCaps = NULL;
    pValueCaps = NULL;
    hHeap = GetProcessHeap();

    //
    // Get the preparsed data block
    //

    if (GetRawInputDeviceInfo(pRawInput->header.hDevice, RIDI_PREPARSEDDATA, NULL, &bufferSize) != 0)
        return SaveFree(FALSE);

    if (!(pPreparsedData = (PHIDP_PREPARSED_DATA)HeapAlloc(hHeap, 0, bufferSize)))
        return SaveFree(FALSE);
    if ((int)GetRawInputDeviceInfo(pRawInput->header.hDevice, RIDI_PREPARSEDDATA, pPreparsedData, &bufferSize) <0)
        return SaveFree(FALSE);

    //
    // Get the joystick's capabilities
    //

    // Button caps
    if (HidP_GetCaps(pPreparsedData, &Caps) != HIDP_STATUS_SUCCESS)
        return SaveFree(FALSE);

    if (!(pButtonCaps = (PHIDP_BUTTON_CAPS)HeapAlloc(hHeap, 0, sizeof(HIDP_BUTTON_CAPS) * Caps.NumberInputButtonCaps)))
        return SaveFree(FALSE);

    capsLength = Caps.NumberInputButtonCaps;
    if (HidP_GetButtonCaps(HidP_Input, pButtonCaps, &capsLength, pPreparsedData) != HIDP_STATUS_SUCCESS)
        return SaveFree(FALSE);

    g_NumberOfButtons = pButtonCaps->Range.UsageMax - pButtonCaps->Range.UsageMin + 1;

    // Value caps
    if (!(pValueCaps = (PHIDP_VALUE_CAPS)HeapAlloc(hHeap, 0, sizeof(HIDP_VALUE_CAPS) * Caps.NumberInputValueCaps)))
        return SaveFree(FALSE);
    capsLength = Caps.NumberInputValueCaps;
    if (HidP_GetValueCaps(HidP_Input, pValueCaps, &capsLength, pPreparsedData) != HIDP_STATUS_SUCCESS)
        return SaveFree(FALSE);

    //
    // Get the pressed buttons
    //

    usageLength = g_NumberOfButtons;
    if (
        HidP_GetUsages(
        HidP_Input, pButtonCaps->UsagePage, 0, usage, &usageLength, pPreparsedData,
        (PCHAR)pRawInput->data.hid.bRawData, pRawInput->data.hid.dwSizeHid
        ) != HIDP_STATUS_SUCCESS)
        return SaveFree(FALSE);

    ZeroMemory(bButtonStates, sizeof(bButtonStates));
    for (i = 0; i < usageLength; i++)
        bButtonStates[usage[i] - pButtonCaps->Range.UsageMin] = TRUE;



    // osy a mini joystick
    for (i = 0; i < Caps.NumberInputValueCaps; i++)
    {
        if (
            HidP_GetUsageValue(
            HidP_Input, pValueCaps[i].UsagePage, 0, pValueCaps[i].Range.UsageMin, &value, pPreparsedData,
            (PCHAR)pRawInput->data.hid.bRawData, pRawInput->data.hid.dwSizeHid
            ) != HIDP_STATUS_SUCCESS)
            return SaveFree(FALSE);

        switch (pValueCaps[i].Range.UsageMin)
        {
        case 0x30:	// X-axis
            m_x = (LONG)value - 508;
            break;
        case 0x31:	// Y-axis
            m_y = (-1)*((LONG)value - 508);
            break;
        case 0x39:	// Hat Switch
            m_h = value;
            break;
        }
        /////////////////////////////////////////
        //vytvoreni prikazy pro ovladani v zavisloti na hodnote v osach
        if(isJoystickUsed)
        {
            motorCtrl_cmd = 0;
        }
        int tmp = motorCtrl_cmd;
        if(m_x > 250)
            motorCtrl_cmd = 4;
        else if(m_x < -250)
            motorCtrl_cmd = 3;

        if(m_y > 250)
            motorCtrl_cmd = 1;
        else if(m_y < -250)
            motorCtrl_cmd = 2;

        if(m_h == 6)
            motorCtrl_cmd =5;
        else if(m_h == 2)
            motorCtrl_cmd =6;

        if(tmp != motorCtrl_cmd && motorCtrl_cmd != 0)
            isJoystickUsed = true;
        if(motorCtrl_cmd == 0)
            isJoystickUsed = false;
    }
    //zobrazit hodnoty x a y na obrazovce
    m_editJoystickXaxis.SetWindowTextA(std::to_string(m_x).c_str());
    m_editJoystickYaxis.SetWindowTextA(std::to_string(m_y).c_str());
    return SaveFree(TRUE);
}
/**
*   @brief  Spusteni streamu videa
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonStreamPlay()
{
    vlcPlayer_.Play();
}
/**
*   @brief  Zastaveni streamu
*
*   @return int
*/
void CMotionControlTab::OnBnClickedButtonStreamStop()
{
    vlcPlayer_.Stop();
}
/**
*   @brief  Vola se pokud prijde raw input udalost
*
*   @param  nInputcode kod zarizeni
*   @param  lParam parametry
*   @return void
*/
void CMotionControlTab::OnRawInput(UINT nInputcode, HRAWINPUT lParam)
{
    PRAWINPUT pRawInput = NULL;
    UINT      bufferSize = 0;
    HANDLE    hHeap;

    GetRawInputData((HRAWINPUT)lParam, RID_INPUT, NULL, &bufferSize, sizeof(RAWINPUTHEADER));

    hHeap = GetProcessHeap();
    pRawInput = (PRAWINPUT)HeapAlloc(hHeap, 0, bufferSize);
    if (!pRawInput)
        return;

    GetRawInputData((HRAWINPUT)lParam, RID_INPUT, pRawInput, &bufferSize, sizeof(RAWINPUTHEADER));
    ParseRawInput(pRawInput);

    HeapFree(hHeap, 0, pRawInput);

    CDialogEx::OnRawInput(nInputcode, lParam);
}
/**
*   @brief Tlacitko vpred
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonFoward()
{
    motorCtrl_cmd = 1;
}
/**
*   @brief  Tlacitko vpravo
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonRight()
{
    motorCtrl_cmd = 3;
}
/**
*   @brief  Tlacitko vlevo
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonLeft()
{
    motorCtrl_cmd = 4;
}
/**
*   @brief  Tlacitko vzad
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonBackward()
{
    motorCtrl_cmd = 2;
}
/**
*   @brief  Tlacitko rotace protismeru hod. rucicek
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonRotcc()
{
    motorCtrl_cmd = 5;
}
/**
*   @brief  Tlacitko zastaveni
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedStop()
{
    motorCtrl_cmd = 0;
}
/**
*   @brief  Tlacitko rotace po smeru hod. rucicek
*   nastavuje globalni promennou, ze ktere je vytvaren prikaz pro rizeni
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonRotc()
{
    motorCtrl_cmd = 6;
}
/**
*   @brief  Handler reaguje na posun slideru a nastavuje hodnoty rychlosti
*
*   @return void
*/
void CMotionControlTab::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    if (pScrollBar == (CScrollBar *)&m_sliderFLSpeed)
    {
        motorSpeedFL = m_sliderFLSpeed.GetPos();
        m_editFLSpeed.SetWindowTextA(std::to_string(motorSpeedFL).c_str());
        UpdateData(FALSE);
    }
    else if (pScrollBar == (CScrollBar *)&m_sliderFRSpeed)
    {
        motorSpeedFR = m_sliderFRSpeed.GetPos();
        m_editFRSpeed.SetWindowTextA(std::to_string(motorSpeedFR).c_str());
        UpdateData(FALSE);
    }
    else if (pScrollBar == (CScrollBar *)&m_sliderBLSpeed)
    {
        motorSpeedBL = m_sliderBLSpeed.GetPos();
        m_editBLSpeed.SetWindowTextA(std::to_string(motorSpeedBL).c_str());
        UpdateData(FALSE);
    }
    else if (pScrollBar == (CScrollBar *)&m_sliderBRSpeed)
    {
        motorSpeedBR = m_sliderBRSpeed.GetPos();
        m_editBRSpeed.SetWindowTextA(std::to_string(motorSpeedBR).c_str());
        UpdateData(FALSE);
    }
    else
    {
        CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
    }
    motorSpeed_changed = true;
}
/**
*   @brief  Zpracovava zpravy zaslane dialogu
*   @param  message zaslana zprava
*   @param  wParam dodatecne argumenty
*   @param  lParam dodatecne argumenty
*   @return LRESULT
*/
LRESULT CMotionControlTab::WindowProc(UINT message, WPARAM wParam, LPARAM lParam)
{
    switch(message)
    {
        //pri vytvoreni okna je inicializovan joystick
    case WM_CREATE:
        {
            //
            // Register for joystick devices
            RAWINPUTDEVICE rid;

            rid.usUsagePage = 1;
            rid.usUsage     = 4;	// Joystick
            rid.dwFlags     = 0;
            rid.hwndTarget  = m_hWnd;

            if(!RegisterRawInputDevices(&rid, 1, sizeof(RAWINPUTDEVICE)))
                return -1;
        }
        return 0;
        // ziskani ukazatele na raw data, aby je bylo mozne zpracovat
    case WM_INPUT:
        {
            PRAWINPUT pRawInput;
            UINT      bufferSize;
            HANDLE    hHeap;

            GetRawInputData((HRAWINPUT)lParam, RID_INPUT, NULL, &bufferSize, sizeof(RAWINPUTHEADER));

            hHeap     = GetProcessHeap();
            pRawInput = (PRAWINPUT)HeapAlloc(hHeap, 0, bufferSize);
            if(!pRawInput)
                return 0;

            GetRawInputData((HRAWINPUT)lParam, RID_INPUT, pRawInput, &bufferSize, sizeof(RAWINPUTHEADER));
            //zpracovani vstupu
            ParseRawInput(pRawInput);
            HeapFree(hHeap, 0, pRawInput);
        }
        return 0;
    }
    return CDialogEx::WindowProc(message, wParam, lParam);
}
/**
*   @brief Konfigurace kamery
*   Pomoci REST api posilam prikaz na konfiguraci kamery
*   Otevira webovy prohlizec -- chrome
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonCameraConfig()
{
    std::string cmd = "http://";
    cmd.append(cfg_base->network_robot_ip);
    cmd.append(":");
    cmd.append(std::to_string(cfg_base->network_camera_port));
    //640x480 resolution, rot 2 , other default
    cmd.append("/panel?width=640&height=480&format=1196444237&9963776=50&9963777=0&9963778=0&9963790=100&9963791=100&9963803=0&9963810=180&134217728=0&134217729=1&134217730=0&134217739=85&134217741=30&9963796=0&9963797=0&134217734=0&134217736=0&134217737=1&134217738=0&134217740=0&9963800=3&134217731=0&134217732=1&134217733=0&134217735=3&apply_changed=1");
    ShellExecute(0, 0, "chrome.exe", cmd.c_str(), 0, SW_SHOW);
}
/**
*   @brief  Spusteni zachytavani statusu jednotlivych promennych z ROS
*   Funkce samotna je v PiInterfaceMFCDlg. Zde je jen posilana zprava
*   ,ktera vyvola prislusnou akci v rodici tohoto dialogu
*
*   @return void
*/
void CMotionControlTab::OnBnClickedStatus()
{
    ASSERT( AfxGetMainWnd()!=NULL );

    AfxGetMainWnd()->SendMessage(WM_COMMAND, ID_FILE_UPDATE);
}
/**
*   @brief  Vytvoreni zasilaneho prikazu pro rizeni vozitka
*
*   @return std::string
*/
std::string CMotionControlTab::CreateCtrlCmd()
{
    std::stringstream ss;
    ss << "motorCtrl=0:";
    ss << std::to_string(motorCtrl_cmd) << ";";
    ss << "autoCtrl=";
    ss << std::to_string(autoCtrl_cmd) << ";";
    if(motorSpeed_changed == true)
    {
        ss << "motorSpeedCtrl=";
        ss << std::to_string(motorSpeedFL) << ":";
        ss << std::to_string(motorSpeedFR) << ":";
        ss << std::to_string(motorSpeedBL) << ":";
        ss << std::to_string(motorSpeedBR) << ";";

        motorSpeed_changed = false;
    }
    return ss.str();
}
/**
*   @brief Vytvoreni ssh spojeni a spusteni vlakna pro ovladani vozitka
*
*   @param  run odkaz na ridici promennou smycky vlakna
*   @param  isJoystickUsed odkaz na promennou rikajici jestli byl pouzit joystick pro ovladani
*   @return void
*/
void CMotionControlTab::ControlUpdate(std::atomic<bool> &run, std::atomic<bool> &isJoystickUsed)
{
    SSHClient * ssh = new SSHClient(cfg_base->network_robot_ip, cfg_base->network_ssh_user, cfg_base->network_ssh_passwd);
    int channel = ssh->EstablishChannel();

    if(ssh->OpenShell(channel))
    {
        //zaslani prikazu pro spusteni gui_control_node
        ssh->SendShellCmd(channel, cmd_base->node_gui_control);
        while(run)
        {
            ssh->SendShellCmd(channel, CreateCtrlCmd());
            if(isJoystickUsed)
                motorCtrl_cmd = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }
    ssh->CloseChannel(channel);
}
/**
*   @brief  Predani konfigurace z ini souboru
*
*   @param  base odkaz na strukturu s konfiguraci
*   @return void
*/
void CMotionControlTab::setConfigBase(config_base &base)
{
    cfg_base = &base;
}
/**
*   @brief  Predani prikazu z ini souboru
*
*   @param  base odkaz na strukturu s prikazy
*   @return void
*/
void CMotionControlTab::setCmdBase(command_base &base)
{
    cmd_base = &base;
}
/**
*   @brief Spusteni vzdaleneho rizeni
*   switch-like funkce pokud je zapnute
*   vlakno se ukonc
*
*   @return void
*/
void CMotionControlTab::OnBnClickedControlRun()
{
    if(!thr_motorCtrl_run)
    {
        m_checkButtonControl.SetCheck(1);
        thr_motorCtrl_run = true;
        isJoystickUsed = false;
        thr_motorCtrl = std::thread(&CMotionControlTab::ControlUpdate, this, std::ref(thr_motorCtrl_run), std::ref(isJoystickUsed));
    }
    else
    {
        m_checkButtonControl.SetCheck(0);
        thr_motorCtrl_run = false;
        if(thr_motorCtrl.joinable())
            thr_motorCtrl.join();
    }
}
/**
*   @brief Resize okna pokud pouzivam SLAM a vystup z
*   kamery jde pres X11, jen aby nezavazelo
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonShowCamera()
{
    ASSERT( AfxGetMainWnd()!=NULL );
    if(m_checkButtonShowCamera.GetCheck())
        m_checkButtonShowCamera.SetCheck(0);
    else
        m_checkButtonShowCamera.SetCheck(1);

    AfxGetMainWnd()->SendMessage(WM_COMMAND, ID_RESIZEWINDOW);
}
/**
*   @brief Nastavuje promennou urcujici
*   manualni nebo automaticke rizeni
*   je posilano pomocí ControlUpdate
*
*   @return void
*/
void CMotionControlTab::OnBnClickedButtonAutoRun()
{
    if(autoCtrl_cmd)
    {
        autoCtrl_cmd = 0;
    }
    else
    {
        autoCtrl_cmd = 1;
    }
    m_checkButtonAutoControl.SetCheck(autoCtrl_cmd);
}