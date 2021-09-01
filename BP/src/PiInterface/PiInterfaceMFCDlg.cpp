/**
*  @file    PiInterfaceMFCDlg.cpp
*  @author  Pavel Koupy (xkoupy00)
*  @date    10.5.2018
*  @version 1.0
*
*  @brief Hlavni okno, ktere obsahuje tab control s obrazovkami pro ovladani
*
*/

#include "stdafx.h"
#include "PiInterfaceMFC.h"
#include "PiInterfaceMFCDlg.h"
#include "afxdialogex.h"
#include "INIReader.h"
#include <chrono>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CAboutDlg dialog used for App About
bool undersized = false;
class CAboutDlg : public CDialogEx
{
public:
    CAboutDlg();

    // Dialog Data
    enum { IDD = IDD_ABOUTBOX };

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

    // Implementation
protected:
    DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CPiInterfaceMFCDlg dialog

CPiInterfaceMFCDlg::CPiInterfaceMFCDlg(CWnd* pParent /*=NULL*/)
    : CDialogEx(CPiInterfaceMFCDlg::IDD, pParent)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPiInterfaceMFCDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_TAB1, tab_control);
}

BEGIN_MESSAGE_MAP(CPiInterfaceMFCDlg, CDialogEx)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_NOTIFY(TCN_SELCHANGING, IDC_TAB1, &CPiInterfaceMFCDlg::OnTcnSelchangingTabControl)
    ON_NOTIFY(TCN_SELCHANGE, IDC_TAB1, &CPiInterfaceMFCDlg::OnSelchangeTabControl)
    ON_COMMAND(ID_SETTINGS_EXIT,&CPiInterfaceMFCDlg::OnExit)
    ON_COMMAND(ID_FILE_UPDATE,&CPiInterfaceMFCDlg::WatchStatusRun)
    ON_COMMAND(ID_RESIZEWINDOW ,&CPiInterfaceMFCDlg::ResizeWindow)
END_MESSAGE_MAP()

BOOL CPiInterfaceMFCDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
    ASSERT(IDM_ABOUTBOX < 0xF000);

    CMenu* pSysMenu = GetSystemMenu(FALSE);
    if (pSysMenu != NULL)
    {
        BOOL bNameValid;
        CString strAboutMenu;
        bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
        ASSERT(bNameValid);
        if (!strAboutMenu.IsEmpty())
        {
            pSysMenu->AppendMenu(MF_SEPARATOR);
            pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
        }
    }

    // Set the icon for this dialog.  The framework does this automatically
    //  when the application's main window is not a dialog
    SetIcon(m_hIcon, TRUE);			// Set big icon
    SetIcon(m_hIcon, FALSE);		// Set small icon

    //Vycteni dat z konfiguracniho souboru
    INIReader reader("config/default.ini");

    if (reader.ParseError() < 0) {
        AfxMessageBox("INIReader parse error. There is no defaults.\n Exitting...", MB_OK | MB_ICONERROR);
        exit(0);
    }

    //Nacteni konfigurace a prikazu do struktur
    cfg_base.general_camera_stream_path= reader.Get("GENERAL", "camera_stream_path", "");;
    cfg_base.general_vnc_viewer        = reader.Get("GENERAL", "vnc_viewer", "");;
    cfg_base.network_robot_ip          = reader.Get("NETWORK", "robot_ip", "");
    cfg_base.network_camera_port       = reader.GetInteger("NETWORK", "camera_port", 8080);
    cfg_base.network_ssh_user          = reader.Get("NETWORK", "ssh_user", "");
    cfg_base.network_ssh_passwd        = reader.Get("NETWORK", "ssh_passwd", "");

    cmd_base.basic_ros_core_launch     = reader.Get("BASIC_COMMANDS", "roscore_launch", "");
    cmd_base.basic_ros_core_shutdown   = reader.Get("BASIC_COMMANDS", "roscore_shutdown", "");
    cmd_base.basic_ros_launch_all      = reader.Get("BASIC_COMMANDS", "ros_launch_all", "");
    cmd_base.basic_reboot              = reader.Get("BASIC_COMMANDS", "reboot", "");
    cmd_base.basic_shutdown            = reader.Get("BASIC_COMMANDS", "shutdown", "");
    cmd_base.node_ult_sensor           = reader.Get("ROS_NODES_COMMNADS", "ult_sensor", "");
    cmd_base.node_whiskers_sensor      = reader.Get("ROS_NODES_COMMNADS", "whiskers", "");
    cmd_base.node_arduino              = reader.Get("ROS_NODES_COMMNADS", "arduino", "");
    cmd_base.node_orb_slam             = reader.Get("ROS_NODES_COMMNADS", "orb_slam", "");
    cmd_base.node_camera               = reader.Get("ROS_NODES_COMMNADS", "camera", "");
    cmd_base.node_autonomous_mode      = reader.Get("ROS_NODES_COMMNADS", "autonomous_mode", "");
    cmd_base.node_gui_status           = reader.Get("ROS_NODES_COMMNADS", "gui_status", "");
    cmd_base.node_gui_control          = reader.Get("ROS_NODES_COMMNADS", "gui_control", "");
    cmd_base.node_image_transport      = reader.Get("ROS_NODES_COMMNADS", "image_transport", "");
    cmd_base.other_uv4l_launch         = reader.Get("OTHER", "uv4l_launch", "");
    cmd_base.other_uv4l_shutdown       = reader.Get("OTHER", "uv4l_shutdown", "");
    cmd_base.other_vnc_run             = reader.Get("OTHER", "vnc_run", "");
    cmd_base.other_wlan0_up            = reader.Get("OTHER", "wlan0_up", "");
    cmd_base.other_wlan0_down          = reader.Get("OTHER", "wlan0_down", "");
    cmd_base.other_ping                = reader.Get("OTHER", "ping", "");
    cmd_base.other_network_restart     = reader.Get("OTHER", "network_restart", "");
    cmd_base.other_cmd1                = reader.Get("OTHER", "cmd1", "");
    cmd_base.other_cmd2                = reader.Get("OTHER", "cmd2", "");
    cmd_base.other_cmd3                = reader.Get("OTHER", "cmd3", "");
    cmd_base.other_cmd4                = reader.Get("OTHER", "cmd4", "");
    cmd_base.other_cmd5                = reader.Get("OTHER", "cmd5", "");
    cmd_base.other_cmd6                = reader.Get("OTHER", "cmd6", "");
    cmd_base.other_cmd7                = reader.Get("OTHER", "cmd7", "");

    thr_GUIstatus_run = false;
    //Vytvreni ovladacich dialogu a vlozeni do vektoru
    m_tabPages.push_back(new CSystemControlTab(this));
    m_tabPages.push_back(new CMotionControlTab(this));

    //prirazeni do ukazatelu pro jednodussi manipulaci
    systemControlTab = (CSystemControlTab*) m_tabPages[0];
    //predani konfigurace a prikazu
    systemControlTab->setCmdBase(cmd_base);
    systemControlTab->setConfigBase(cfg_base);

    motionControlTab = (CMotionControlTab*) m_tabPages[1];
    motionControlTab->setCmdBase(cmd_base);
    motionControlTab->setConfigBase(cfg_base);

    m_tabPages[0]->Create(CSystemControlTab::IDD, &tab_control);
    m_tabPages[1]->Create(CMotionControlTab::IDD, &tab_control);

    //Tab control inicializace a prirazeni jednotlivych obrazovek ke spravnym zalozkam
    RECT r;
    TCITEM tci;
    GetClientRect(&r);

    tci.mask = TCIF_TEXT;
    tci.iImage = -1;

    //adding tabs titles and inserting them
    tci.pszText = "System control";
    tab_control.InsertItem(0, &tci);
    //dialog pr
    tci.pszText = "Motion control";
    tab_control.InsertItem(1, &tci);

    OnSelchangeTabControl(NULL, NULL);
    ////////////////////////////////////////////////////////////////////
    return TRUE;  // return TRUE  unless you set the focus to a control
}
/**
*   @brief Zmena velikosti okna
*   Pretizeni - volane pokud prechazim mezi obrazovkami
*
*   @return void
*/
void CPiInterfaceMFCDlg::ResizeWindow(bool restore)
{
    int height,width = 0;
    RECT rect;
    //ziskani velikosti okna
    GetWindowRect(&rect);
    width = rect.right - rect.left;
    height = rect.bottom - rect.top;
    if(undersized && restore)
    {
        MoveWindow(rect.left, rect.top, static_cast<int>(width*(1.72)), height);
        undersized = false;
    }
}
/**
*   @brief Zmena velikosti okna
*   Pro volani pomoci zpravy
*
*   @return void
*/
void CPiInterfaceMFCDlg::ResizeWindow()
{
    int height,width = 0;
    RECT rect;
    GetWindowRect(&rect);
    width = rect.right - rect.left;
    height = rect.bottom - rect.top;
    if(!undersized)
    {
        MoveWindow(rect.left, rect.top, static_cast<int>(width/(1.72)), height);
        undersized = true;
    }
    else
    {
        MoveWindow(rect.left, rect.top, static_cast<int>(width*(1.72)), height);
        undersized = false;
    }
}
/**
*   @brief Handler pro systemove prikazy
*   @return void
*/
void CPiInterfaceMFCDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    //about dialog
    if ((nID & 0xFFF0) == IDM_ABOUTBOX)
    {
        CAboutDlg dlgAbout;
        dlgAbout.DoModal();
    }
    else
    {
        CDialogEx::OnSysCommand(nID, lParam);
    }
}

//  Automaticky generovane frameworkem
void CPiInterfaceMFCDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // device context for painting

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialogEx::OnPaint();
    }
}
//  Automaticky generovane frameworkem
HCURSOR CPiInterfaceMFCDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}
/**
*   @brief Handler pro kliknuti na zalozku obrazovky
*   @return void
*/
void CPiInterfaceMFCDlg::OnTcnSelchangingTabControl(NMHDR *pNMHDR, LRESULT *pResult)
{
    m_tabPages[tab_control.GetCurSel()]->ShowWindow(SW_HIDE);
    *pResult = 0;
}
/**
*   @brief Handler zmeny obrazovek
*   @return void
*/
void CPiInterfaceMFCDlg::OnSelchangeTabControl(NMHDR *pNMHDR, LRESULT *pResult)
{
    RECT rc;
    tab_control.GetItemRect(0,&rc);
    int nIndex = tab_control.GetCurSel();
    ResizeWindow(true);
    m_tabPages[nIndex]->SetWindowPos( NULL, rc.left + 1, rc.bottom + 1, 0, 0, SWP_NOSIZE | SWP_NOZORDER | SWP_SHOWWINDOW);
    m_tabPages[nIndex]->SetFocus();
    //*pResult = 0;
}
/**
*   @brief Rozdeli string dle parametru delimeter
*   
*   @param s string pro rozdeleni
*   @return std::vector<std::string> vraci vector stringu na ktere byl str puvodni rozdelen
*/
std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    //vyjme ze stringu slovo pred rozdelovacim znakem
    while (std::getline(tokenStream, token, delimiter))
    {
        //vlozi string do vektoru
        tokens.push_back(token);
    }
    return tokens;
}
/**
*   @brief Zpracovava vystup z gui_status_node z vozitka
*
*   @param output vystup z shellu SSHClienta
*   @return void
*/
void CPiInterfaceMFCDlg::ProcessShellOutput(std::string output)
{
    //nahrazeni new line znaku ";"
    output.erase(std::remove(output.begin(), output.end(), '\r'), output.end());
    size_t index = 0;
    while (true)
    {
        //vyhledani substringu pro nahrazeni
        index = output.find("\n", index);
        if (index == std::string::npos) break;
        //nahrazeni
        output.replace(index, 1, ";");
        index += 1;
    }
    //rozdeleni stringu na jednotlive prikazy
    std::vector<std::string> result = split(output, ';');
    if(!result.empty())
    {
        //rozdeleni na casti jmeno prikazu a hodnota
        std::vector<std::string> cmds = split(result.back(), ',');
        for(auto cmd : cmds)
        {
            std::vector<std::string> param = split(cmd, ':');
            if(param.size() == 2)
            {
                //data z ultrazvukoveho senzoru
                if(param[0].compare("distance") == 0)
                    //predani hodnoty primo do obrazovky pro ovladani
                    motionControlTab->m_editULTSensor.SetWindowTextA((std::to_string(static_cast<int>(std::atof(param[1].c_str())))+" cm").c_str());
                //data z dotykoveho senzoru
                else if(param[0].compare("whiskers") == 0)
                    motionControlTab->m_editWhiskers.SetWindowTextA((std::atoi(param[1].c_str()) ? "true" : "false"));
                else if(param[0].compare("autonomode") == 0)
                {
                    std::string result = "Remote Control";
                    switch (std::atoi(param[1].c_str()))
                    {
                    case AutonomousLevel::MANUAL:
                        result = "Remote Control";
                        break;
                    case AutonomousLevel::COLLISION_DETECTION:
                        result = "Collision detection";
                        break;
                    case AutonomousLevel::WANDERER_WALK:
                        result = "Wandering";
                        break;
                    case AutonomousLevel::RANDOM_WALK:
                        result = "Random direction";
                        break;
                    default:
                        break;
                    }
                    motionControlTab->m_editAutoStatus.SetWindowTextA(result.c_str());
                }
                //data z uzlu pro mapovani a lokalizace viz. TrackingState enum
                else if(param[0].compare("slam") == 0)
                {
                    std::string result = "SYSTEM_NOT_READY";
                    switch (std::atoi(param[1].c_str()))
                    {
                    case TrackingState::SYSTEM_NOT_READY:
                        result = "System not ready";
                        break;
                    case TrackingState::NO_IMAGES_YET:
                        result = "No images available - check camera";
                        break;
                    case TrackingState::NOT_INITIALIZED:
                        result = "Not initialized";
                        break;
                    case TrackingState::OK:
                        result = "OK - runnig";
                        break;
                    case TrackingState::LOST:
                        result = "LOST";
                        break;
                    default:
                        break;
                    }
                    motionControlTab->m_editSLAMstate.SetWindowTextA(result.c_str());
                }
            }
        }
    }
}
/**
*   @brief Vlakno pro aktualizaci stavu uzlu v ROS
*   Vytvori instanci SSHClienta pripoji kanal a spusti gui_status_node
*   @return void
*/
void CPiInterfaceMFCDlg::StatusUpdate(std::atomic<bool> &run)
{
    SSHClient * ssh = new SSHClient(cfg_base.network_robot_ip , cfg_base.network_ssh_user, cfg_base.network_ssh_passwd);
    int channel = ssh->EstablishChannel();

    if(ssh->OpenShell(channel) && ssh->SendShellCmd(channel, cmd_base.node_gui_status))
    {
        ssh->ReadShell(channel);
        ssh->ReadShell(channel);
        while(run)
        {
            std::string str= ssh->ReadShell(channel);
            ProcessShellOutput(str);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    ssh->CloseChannel(channel);
}
/**
*   @brief Spousti a vypina vlakno pro cteni dat
*   @return void
*/
void CPiInterfaceMFCDlg::WatchStatusRun()
{
    if(!thr_GUIstatus_run)
    {
        motionControlTab->m_checkButtonGUIStatus.SetCheck(1);
        thr_GUIstatus_run = true;
        gui_status = std::thread(&CPiInterfaceMFCDlg::StatusUpdate, this, std::ref(thr_GUIstatus_run));
    }
    else
    {
        motionControlTab->m_checkButtonGUIStatus.SetCheck(0);
        thr_GUIstatus_run = false;
        if(gui_status.joinable())
            gui_status.join();
    }
}
void CPiInterfaceMFCDlg::OnExit()
{
    exit(0);
}