/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrProgressLib
	   File Type:	           h
       Created Date:	       2012/7/29
	   Created Time:	       10:00      
*********************************************************************/
#ifndef TerrProgressLib_h_ZQW_2012_7_29_10_00_DEF
#define TerrProgressLib_h_ZQW_2012_7_29_10_00_DEF

#include <afxcmn.h>
#include "resource.h"
#include "BasicDef.h"

#ifndef TerrProgressLib_LIB 
   #define TerrProgressLib_LIB _declspec(dllimport)
   #ifdef _DEBUG
      #pragma comment(lib,"TerrProgressLibD.lib")
      #pragma message("Automatically linking with TerrProgressLibD.lib")
   #else
      #pragma comment(lib,"TerrProgressLib.lib")
      #pragma message("Automatically linking with TerrProgressLib.lib")
   #endif
#endif

class TerrProgressLib_LIB CProgressDlg : public CDialog
{
	DECLARE_DYNAMIC(CProgressDlg)

public:
	CProgressDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CProgressDlg();	

	// Dialog Data
	enum {IDD = IDD_DIALOG_PROCESS};

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg LRESULT OnProcessMsg(WPARAM wParam,LPARAM lParam);
	virtual BOOL OnInitDialog();
	CProgressCtrl m_ProcessBar;
	
};

#endif // TerrProgressLib_h__