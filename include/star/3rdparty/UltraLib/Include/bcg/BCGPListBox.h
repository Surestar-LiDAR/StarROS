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
//
// BCGPListBox.h : header file
//

#if !defined(AFX_BCGPLISTBOX_H__7E778AEF_FFBB_486D_A7B3_D25D7868F5D9__INCLUDED_)
#define AFX_BCGPLISTBOX_H__7E778AEF_FFBB_486D_A7B3_D25D7868F5D9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "bcgcbpro.h"

/////////////////////////////////////////////////////////////////////////////
// CBCGPListBox window

class BCGCBPRODLLEXPORT CBCGPListBox : public CListBox
{
	friend class CBCGPPropertySheet;

	DECLARE_DYNAMIC(CBCGPListBox)

// Construction
public:
	CBCGPListBox();

// Attributes
public:
	BOOL	m_bOnGlass;
	BOOL	m_bVisualManagerStyle;

	BOOL IsBackstageMode() const
	{
		return m_bBackstageMode;
	}

	BOOL IsPropertySheetNavigator() const
	{
		return m_bPropertySheetNavigator;
	}

// Operations
public:
	BOOL SetImageList (HIMAGELIST hImageList);
	void SetItemImage(int nIndex, int nImageIndex)
	{
		m_ItemIcons.SetAt(nIndex, nImageIndex);
	}

	void AddCaption(LPCTSTR lpszCaption);

	BOOL IsCaptionItem(int nIndex) const;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBCGPListBox)
	public:
	virtual void DrawItem(LPDRAWITEMSTRUCT lpDrawItemStruct);
	virtual void MeasureItem(LPMEASUREITEMSTRUCT lpMeasureItemStruct);
	//}}AFX_VIRTUAL

	virtual void OnDrawItemContent(CDC* pDC, CRect rect, int nIndex);

// Implementation
public:
	virtual ~CBCGPListBox();

	// Generated message map functions
protected:
	//{{AFX_MSG(CBCGPListBox)
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnPaint();
	afx_msg void OnSelchange();
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	//}}AFX_MSG
	afx_msg LRESULT OnBCGSetControlVMMode (WPARAM, LPARAM);
	afx_msg LRESULT OnBCGSetControlAero (WPARAM, LPARAM);
	afx_msg LRESULT OnMouseLeave(WPARAM,LPARAM);
	afx_msg LRESULT OnBCGSetControlBackStageMode (WPARAM, LPARAM);
	DECLARE_MESSAGE_MAP()

	int HitTest(CPoint pt);

	BOOL		m_bIsFocused;
	int			m_nHighlightedItem;
	BOOL		m_bTracked;
	HIMAGELIST	m_hImageList;		// External images
	CSize		m_sizeImage;		// Image size
	BOOL		m_bBackstageMode;
	BOOL		m_bPropertySheetNavigator;

	CList<int, int>				m_lstCaptionIndexes;
	CMap<int, int, int, int>	m_ItemIcons;
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BCGPLISTBOX_H__7E778AEF_FFBB_486D_A7B3_D25D7868F5D9__INCLUDED_)
