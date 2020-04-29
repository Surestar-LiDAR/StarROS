//*******************************************************************************
// COPYRIGHT NOTES
// ---------------
// This is a part of BCGControlBar Library Professional Edition
// Copyright (C) 1998-2010 BCGSoft Ltd.
// All rights reserved.
//
// This source code can be used, distributed or modified
// only under terms and conditions 
// of the accompanying license agreement.
//*******************************************************************************
//
// BCGPRibbonBackstageViewPanel.h: interface for the CBCGPRibbonBackstageViewPanel class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BCGPRIBBONBACKSTAGEVIEWPANEL_H__2EDD1A63_137D_4278_AD81_D0A2EB2CD816__INCLUDED_)
#define AFX_BCGPRIBBONBACKSTAGEVIEWPANEL_H__2EDD1A63_137D_4278_AD81_D0A2EB2CD816__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef BCGP_EXCLUDE_RIBBON

#include "BCGPRibbonButton.h"
#include "BCGPRibbonMainPanel.h"
#include "BCGPDialog.h"
#include "BCGPToolBarImages.h"
#include "BCGPListBox.h"

///////////////////////////////////////////////////////////////////////
// CBCGPRibbonBackstageViewItemForm class

class BCGCBPRODLLEXPORT CBCGPRibbonBackstageViewItemForm : public CBCGPBaseRibbonElement
{
	friend class CBCGPRibbonBackstageViewPanel;

	DECLARE_DYNCREATE(CBCGPRibbonBackstageViewItemForm)

public:
	// pDlgClass should be derived from CBCGPDialog class:
	CBCGPRibbonBackstageViewItemForm(UINT nDlgTemplateID, CRuntimeClass* pDlgClass, CSize sizeWaterMark = CSize(0, 0));
	virtual ~CBCGPRibbonBackstageViewItemForm();

protected:
	CBCGPRibbonBackstageViewItemForm();

// Operations:
public:
	void SetWaterMarkImage(UINT uiWaterMarkResID, COLORREF clrBase = (COLORREF)-1 /* if clrBase is defined, the images will be addapted to the ribbon main button color */);

protected:
	void CommonConstruct();

// Overrides
protected:
	virtual void OnAfterChangeRect (CDC* pDC);
	virtual BOOL CanBeAddedToQAT () const
	{
		return FALSE;
	}

	virtual CWnd* OnCreateFormWnd();
	virtual void CopyFrom (const CBCGPBaseRibbonElement& src);
	virtual CSize GetRegularSize (CDC* /*pDC*/)	{	return m_sizeDlg;	}
	virtual void OnDraw (CDC* /*pDC*/) {}
	virtual void OnChangeVisualManager();
	virtual CSize GetWndOffset() const
	{
		return CSize(0, 0);
	}

	virtual void OnSetBackstageWatermarkRect(CRect rectWatermark);
	virtual void SetLayoutReady(BOOL /*bReady*/ = TRUE)	{}

// Attributes
protected:
	CWnd*				m_pWndForm; // Form located on the right side
	UINT				m_nDlgTemplateID;
	CRuntimeClass*		m_pDlgClass;
	CSize				m_sizeDlg;
	CSize				m_sizeWaterMark;
	CBCGPToolBarImages	m_Watermark;
	CBCGPToolBarImages	m_WatermarkColorized;
	COLORREF			m_clrWatermarkBaseColor;
	BOOL				m_bImageMirror;
};

///////////////////////////////////////////////////////////////////////
// CBCGPRibbonBackstageViewItemPropertySheet class

class BCGCBPRODLLEXPORT CBCGPRibbonBackstageViewItemPropertySheet : public CBCGPRibbonBackstageViewItemForm
{
	DECLARE_DYNCREATE(CBCGPRibbonBackstageViewItemPropertySheet)

public:
	CBCGPRibbonBackstageViewItemPropertySheet(UINT nIconsListResID, int nIconWidth = 32);
	virtual ~CBCGPRibbonBackstageViewItemPropertySheet();

protected:
	CBCGPRibbonBackstageViewItemPropertySheet();

// Operations:
public:
	void AddPage(CBCGPPropertyPage* pPage);
	void AddGroup(LPCTSTR lpszCaption);
	void RemoveAll();

// Overrides
protected:
	virtual CWnd* OnCreateFormWnd();
	virtual void CopyFrom (const CBCGPBaseRibbonElement& src);

	virtual CSize GetWndOffset() const
	{
		return CSize(20, 20);
	}

	virtual void OnSetBackstageWatermarkRect(CRect rectWatermark);
	virtual void SetLayoutReady(BOOL bReady = TRUE);

// Attributes
protected:
	UINT			m_nIconsListResID;
	int				m_nIconWidth;
	CArray<CBCGPPropertyPage*, CBCGPPropertyPage*> m_arPages;
	CStringArray	m_arCaptions;
};

//////////////////////////////////////////////////////////////////////////////
// CBCGPRibbonBackstageViewPanel class

class BCGCBPRODLLEXPORT CBCGPRibbonBackstageViewPanel : public CBCGPRibbonMainPanel
{
	friend class CBCGPRibbonBackstageViewItemForm;

	DECLARE_DYNCREATE(CBCGPRibbonBackstageViewPanel)

public:
	CBCGPRibbonBackstageViewPanel();
	virtual ~CBCGPRibbonBackstageViewPanel();

// Operations
public:
	CBCGPRibbonButton* AddCommand(UINT uiCommandID, LPCTSTR lpszLabel, int nImageIndex = -1)
	{
		ASSERT_VALID (this);

		CBCGPRibbonButton* pButton = new CBCGPRibbonButton (uiCommandID, lpszLabel, nImageIndex);
		ASSERT_VALID (pButton);

		pButton->SetBackstageViewMode();

		CBCGPRibbonMainPanel::Add(pButton);

		return pButton;
	}

	CBCGPRibbonButton* AddView(UINT uiCommandID, LPCTSTR lpszLabel, CBCGPRibbonBackstageViewItemForm* pView)
	{
		ASSERT_VALID(this);
		ASSERT_VALID(pView);

		CBCGPRibbonButton* pButton = new CBCGPRibbonButton (uiCommandID, lpszLabel);
		ASSERT_VALID (pButton);

		pButton->SetBackstageViewMode();

		pButton->AddSubItem(pView);

		CBCGPRibbonMainPanel::Add(pButton);

		return pButton;
	}

	virtual void ReposActiveForm();

protected:
	// The following methods cannot be called for CBCGPRibbonBackstageViewPanel:
	virtual void Add (CBCGPBaseRibbonElement* /*pElem*/)						{	ASSERT(FALSE);	}
	void AddToBottom (CBCGPRibbonMainPanelButton* /*pElem*/)					{	ASSERT(FALSE);	}
	void AddToRight (CBCGPBaseRibbonElement* /*pElem*/, int /*nWidth*/ = 300)	{	ASSERT(FALSE);	}
	void EnableCommandSearch (BOOL /*bEnable*/, LPCTSTR /*lpszLabel*/, LPCTSTR /*lpszKeyTip*/, int /*nWidth*/ = 0)	{	ASSERT(FALSE);	}

	virtual int GetMenuElements () const
	{
		ASSERT_VALID (this);
		return (int) m_arElements.GetSize ();
	}

	virtual BOOL IsBackstageView() const
	{
		return TRUE;
	}

	virtual BOOL IsBackstageRightPaneActive() const
	{
		ASSERT_VALID(this);
		return m_pSelected != NULL;
	}

	virtual void Repos (CDC* pDC, const CRect& rect);
	virtual CBCGPBaseRibbonElement* MouseButtonDown (CPoint point);

	virtual BOOL OnKey (UINT nChar);

	void SelectView(CBCGPBaseRibbonElement* pElem);
	void AdjustScrollBars();

// Attributes:
protected:
	CBCGPBaseRibbonElement*	m_pSelected;
	CRect					m_rectRight;
	CSize					m_sizeRightView;
	BOOL					m_bInAdjustScrollBars;
};

/////////////////////////////////////////////////////////////////////////////
// CBCGPRecentFilesListBox window

class BCGCBPRODLLEXPORT CBCGPRecentFilesListBox : public CBCGPListBox
{
	DECLARE_DYNAMIC(CBCGPRecentFilesListBox)

// Construction
public:
	CBCGPRecentFilesListBox();

// Attributes
public:

// Operations
public:
	void FillList();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBCGPRecentFilesListBox)
	public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	protected:
	virtual void PreSubclassWindow();
	//}}AFX_VIRTUAL

	virtual void OnDrawItemContent(CDC* pDC, CRect rect, int nIndex);
	virtual void OnChooseRecentFile(UINT uiCmd);

// Implementation
public:
	virtual ~CBCGPRecentFilesListBox();

	// Generated message map functions
protected:
	//{{AFX_MSG(CBCGPRecentFilesListBox)
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnCancelMode();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

	int						m_nClickedItem;
	CArray<HICON, HICON>	m_arIcons;
};

#endif // BCGP_EXCLUDE_RIBBON

#endif // !defined(AFX_BCGPRIBBONBACKSTAGEVIEWPANEL_H__2EDD1A63_137D_4278_AD81_D0A2EB2CD816__INCLUDED_)
