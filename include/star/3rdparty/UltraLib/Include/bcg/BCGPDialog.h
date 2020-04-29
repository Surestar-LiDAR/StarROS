//*******************************************************************************
// COPYRIGHT NOTES
// ---------------
// This is a part of the BCGControlBar Library
// Copyright (C) 1998-2010 BCGSoft Ltd.
// All rights reserved.
//
// This source code can be used, distributed or modified
// only under terms and conditions 
// of the accompanying license agreement.
//*******************************************************************************


#if !defined(AFX_BCGPDIALOG_H__09800B11_4F60_11D4_AB0C_000000000000__INCLUDED_)
#define AFX_BCGPDIALOG_H__09800B11_4F60_11D4_AB0C_000000000000__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BCGCBPro.h"
#include "bcgpdlgimpl.h"

class CBCGPLocalResource;
class CBCGPToolBarImages;

/////////////////////////////////////////////////////////////////////////////
// CBCGPDialog dialog

class BCGCBPRODLLEXPORT CBCGPDialog : public CDialog
{
	friend class CBCGPPopupMenu;
	friend class CBCGPDropDownList;
	friend class CBCGPContextMenuManager;
	friend class CBCGPRibbonBackstageViewItemForm;

	DECLARE_DYNAMIC(CBCGPDialog)

// Construction
public:
	CBCGPDialog();
	CBCGPDialog(UINT nIDTemplate, CWnd *pParent= NULL);
	CBCGPDialog(LPCTSTR lpszTemplateName, CWnd *pParentWnd= NULL);

protected:
	void CommonConstruct ();

// Attributes:
public:
	// Visualization:
	void EnableVisualManagerStyle (BOOL bEnable = TRUE, BOOL bNCArea = FALSE, const CList<UINT,UINT>* plstNonSubclassedItems = NULL);
	BOOL IsVisualManagerStyle () const
	{
		return m_Impl.m_bVisualManagerStyle;
	}

	BOOL IsVisualManagerNCArea () const
	{
		return m_Impl.m_bVisualManagerNCArea;
	}

	BOOL IsBackstageMode() const
	{
		return m_Impl.m_bBackstageMode;
	}

	enum BackgroundLocation
	{
		BACKGR_TILE,
		BACKGR_TOPLEFT,
		BACKGR_TOPRIGHT,
		BACKGR_BOTTOMLEFT,
		BACKGR_BOTTOMRIGHT,
	};

	BOOL EnableAero (BCGPMARGINS& margins);
	void GetAeroMargins (BCGPMARGINS& margins) const;

	BOOL HasAeroMargins () const
	{
		return m_Impl.HasAeroMargins ();
	}

	// Layout:
	void EnableLayout(BOOL bEnable = TRUE, CRuntimeClass* pRTC = NULL, BOOL bResizeBox = TRUE);
	BOOL IsLayoutEnabled() const
	{
		return m_Impl.IsLayoutEnabled();
	}

	CBCGPControlsLayout* GetLayout()
	{
		return m_Impl.m_pLayout;
	}

	virtual void AdjustControlsLayout()
	{
		m_Impl.AdjustControlsLayout();
	}

	void UpdateCaptionButtons ()
	{
		m_Impl.UpdateCaptionButtons();
	}

protected:
	HBITMAP				m_hBkgrBitmap;
	CSize				m_sizeBkgrBitmap;
	CBrush				m_brBkgr;
	BackgroundLocation	m_BkgrLocation;
	BOOL				m_bAutoDestroyBmp;
	BOOL				m_bWasMaximized;
	CBCGPDlgImpl		m_Impl;
	BOOL				m_bIsLocal;
	CBCGPLocalResource*	m_pLocaRes;
	CRect				m_rectBackstageWatermark;
	CBCGPToolBarImages*	m_pBackstageWatermark;

// Operations:
public:
	void SetBackgroundColor (COLORREF color, BOOL bRepaint = TRUE);
	void SetBackgroundImage (HBITMAP hBitmap, 
							BackgroundLocation location = BACKGR_TILE,
							BOOL bAutoDestroy = TRUE,
							BOOL bRepaint = TRUE);
	BOOL SetBackgroundImage (UINT uiBmpResId,
							BackgroundLocation location = BACKGR_TILE,
							BOOL bRepaint = TRUE);

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBCGPDialog)
	public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	virtual INT_PTR DoModal();
	protected:
	virtual BOOL OnCommand(WPARAM wParam, LPARAM lParam);
	//}}AFX_VIRTUAL
	virtual void PreInitDialog();

	virtual void OnOK()
	{
		if (!IsBackstageMode())
		{
			CDialog::OnOK();
		}
	}

	virtual void OnCancel()
	{
		if (!IsBackstageMode())
		{
			CDialog::OnCancel();
		}
		else
		{
			CWnd* pParent = GetParent();
			if (pParent->GetSafeHwnd() != NULL)
			{
				pParent->SendMessage(WM_CLOSE);
			}
		}
	}

public:
	virtual void OnDrawBackstageWatermark(CDC* /*pDC*/, CRect /*rect*/)	{}

// Implementation
protected:

	//{{AFX_MSG(CBCGPDialog)
	afx_msg void OnActivate(UINT nState, CWnd *pWndOther, BOOL bMinimized);
	afx_msg BOOL OnNcActivate(BOOL bActive);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnDestroy();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnSysColorChange();
	afx_msg void OnSettingChange(UINT uFlags, LPCTSTR lpszSection);
	virtual BOOL OnInitDialog();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnNcPaint();
	afx_msg void OnNcMouseMove(UINT nHitTest, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg BCGNcHitTestType OnNcHitTest(CPoint point);
	afx_msg void OnNcCalcSize(BOOL bCalcValidRects, NCCALCSIZE_PARAMS FAR* lpncsp);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnWindowPosChanged(WINDOWPOS FAR* lpwndpos);
	afx_msg void OnGetMinMaxInfo(MINMAXINFO FAR* lpMMI);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	//}}AFX_MSG
	afx_msg LRESULT OnDWMCompositionChanged(WPARAM,LPARAM);
	afx_msg LRESULT OnChangeVisualManager (WPARAM, LPARAM);
	afx_msg LRESULT OnSetText(WPARAM, LPARAM);
	DECLARE_MESSAGE_MAP()

	void SetActiveMenu (CBCGPPopupMenu* pMenu);
	
	void EnableBackstageMode()
	{
		m_Impl.EnableBackstageMode();
	}
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BCGPDIALOG_H__09800B11_4F60_11D4_AB0C_000000000000__INCLUDED_)
