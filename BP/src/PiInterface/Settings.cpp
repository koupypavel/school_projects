// Settings.cpp : implementation file
//

#include "stdafx.h"
#include "PiInterfaceMFC.h"
#include "Settings.h"
#include "afxdialogex.h"

// CSettings dialog

IMPLEMENT_DYNAMIC(CSettings, CDialogEx)

    CSettings::CSettings(CWnd* pParent /*=NULL*/)
    : CDialogEx(CSettings::IDD, pParent)
{
}

CSettings::~CSettings()
{
}

void CSettings::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_IPADDRESS1, m_ipAddress);
    DDX_Control(pDX, IDC_EDIT1, m_editRosbridgePort);
    DDX_Control(pDX, IDC_EDIT2, m_editCameraPort);
}

BEGIN_MESSAGE_MAP(CSettings, CDialogEx)
    ON_BN_CLICKED(ID_SAVE, &CSettings::OnBnClickedSave)
END_MESSAGE_MAP()

// CSettings message handlers

void CSettings::OnBnClickedSave()
{
    // TODO: Add your control notification handler code here
}