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
// BCGPVisualManagerVS2010.cpp: implementation of the CBCGPVisualManagerVS2010 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BCGPVISUALMANAGERVS2010_H__F2994064_A72E_4AF2_A8DD_3799EF9D4199__INCLUDED_)
#define AFX_BCGPVISUALMANAGERVS2010_H__F2994064_A72E_4AF2_A8DD_3799EF9D4199__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "bcgpvisualmanagervs2008.h"

class BCGCBPRODLLEXPORT CBCGPVisualManagerVS2010 : public CBCGPVisualManagerVS2008
{
	DECLARE_DYNCREATE(CBCGPVisualManagerVS2010)

public:
	CBCGPVisualManagerVS2010();
	virtual ~CBCGPVisualManagerVS2010();

	virtual void OnUpdateSystemColors ();
	virtual void ModifyGlobalColors ();

	virtual void OnFillBarBackground (CDC* pDC, CBCGPBaseControlBar* pBar,
									CRect rectClient, CRect rectClip,
									BOOL bNCArea = FALSE);

	virtual void OnHighlightMenuItem (CDC*pDC, CBCGPToolbarMenuButton* pButton,
		CRect rect, COLORREF& clrText);

	virtual COLORREF OnDrawControlBarCaption (CDC* pDC, CBCGPDockingControlBar* pBar, 
		BOOL bActive, CRect rectCaption, CRect rectButtons);

	virtual CRect GetMenuImageFrameOffset () const
	{
		return CRect (4, 2, 0, 3);
	}

	virtual void OnDrawSlider (CDC* pDC, CBCGPSlider* pSlider, CRect rect, BOOL bAutoHideMode);

	virtual COLORREF GetToolbarButtonTextColor (CBCGPToolbarButton* pButton, CBCGPVisualManager::BCGBUTTON_STATE state);
	virtual COLORREF GetToolbarDisabledTextColor ();

	// Tabs support:
	virtual void OnDrawTab (CDC* pDC, CRect rectTab,
							int iTab, BOOL bIsActive, const CBCGPBaseTabWnd* pTabWnd);

	virtual void OnEraseTabsArea (CDC* pDC, CRect rect, 
								const CBCGPBaseTabWnd* pTabWnd);

	virtual void OnEraseTabsButton (CDC* pDC, CRect rect, CBCGPButton* pButton,
									CBCGPBaseTabWnd* pWndTab);

	virtual int GetTabButtonState (CBCGPTabWnd* pTab, CBCGTabButton* pButton);
	virtual BOOL OnEraseTabsFrame (CDC* pDC, CRect rect, const CBCGPBaseTabWnd* pTabWnd);
	virtual void OnDrawTabBorder(CDC* pDC, CBCGPTabWnd* pTabWnd, CRect rectBorder, COLORREF clrBorder, CPen& penLine);

	virtual int GetMDITabsBordersSize ()		{	return 4;		}
	virtual BOOL IsMDITabsTopEdge ()			{	return FALSE;	}
	virtual BOOL AlwaysHighlight3DTabs () const	{	return TRUE;	}

	virtual void GetTabFrameColors (const CBCGPBaseTabWnd* pTabWnd,
				   COLORREF& clrDark,
				   COLORREF& clrBlack,
				   COLORREF& clrHighlight,
				   COLORREF& clrFace,
				   COLORREF& clrDarkShadow,
				   COLORREF& clrLight,
				   CBrush*& pbrFace,
				   CBrush*& pbrBlack);

	virtual void OnDrawTabResizeBar (CDC* pDC, CBCGPBaseTabWnd* pWndTab, 
									BOOL bIsVert, CRect rect,
									CBrush* pbrFace, CPen* pPen);

	virtual int GetTabExtraHeight(const CBCGPTabWnd* pTab);
	virtual int GetTabsMargin (const CBCGPTabWnd* pTab);

	// Auto-hide buttons:
	virtual void OnDrawAutoHideButtonBorder (CDC* pDC, CRect rectBounds, CRect rectBorderSize, CBCGPAutoHideButton* pButton);
	virtual COLORREF GetAutoHideButtonTextColor (CBCGPAutoHideButton* pButton);

	virtual BCGP_SMARTDOCK_THEME GetSmartDockingTheme ();

	virtual void OnFillBackground(CDC* pDC, CRect rect, CWnd* pWnd);

	virtual COLORREF GetStatusBarPaneTextColor (CBCGPStatusBar* pStatusBar, CBCGStatusBarPaneInfo* pPane);

	virtual void OnDrawComboDropButton (CDC* pDC, CRect rect,
										BOOL bDisabled,
										BOOL bIsDropped,
										BOOL bIsHighlighted,
										CBCGPToolbarComboBoxButton* pButton);
	virtual void OnDrawComboBorder (CDC* pDC, CRect rect,
										BOOL bDisabled,
										BOOL bIsDropped,
										BOOL bIsHighlighted,
										CBCGPToolbarComboBoxButton* pButton);
	virtual void OnDrawEditBorder (CDC* pDC, CRect rect,
										BOOL bDisabled,
										BOOL bIsHighlighted,
										CBCGPToolbarEditBoxButton* pButton);
	
	virtual HBRUSH GetToolbarEditColors(CBCGPToolbarButton* pButton, COLORREF& clrText, COLORREF& clrBk);

	virtual void OnFillCombo		(CDC* pDC, CRect rect,
										BOOL bDisabled,
										BOOL bIsDropped,
										BOOL bIsHighlighted,
										CBCGPToolbarComboBoxButton* pButton);

	virtual void OnFillButtonInterior (CDC* pDC,
		CBCGPToolbarButton* pButton, CRect rect, CBCGPVisualManager::BCGBUTTON_STATE state);

	virtual void OnFillHighlightedArea (CDC* pDC, CRect rect, CBrush* pBrush, CBCGPToolbarButton* pButton);

	virtual void OnDrawButtonBorder (CDC* pDC,
		CBCGPToolbarButton* pButton, CRect rect, CBCGPVisualManager::BCGBUTTON_STATE state);

	virtual int GetDockingTabsBordersSize ()	{	return 0;	}

	virtual BOOL OnEraseMDIClientArea (CDC* pDC, CRect rectClient);

	virtual BOOL AreCustomToolbarCtrlColors()	{	return TRUE;	}

	virtual CSize GetButtonExtraBorder () const	{	return CSize(0, 2);	}

	virtual int GetDockingBarCaptionExtraHeight () const	{	return 3;	}

	virtual COLORREF OnFillMiniFrameCaption (CDC* pDC, CRect rectCaption, 
											CBCGPMiniFrameWnd* pFrameWnd,
											BOOL bActive);
	virtual void OnDrawMiniFrameBorder (CDC* pDC, CBCGPMiniFrameWnd* pFrameWnd,
										CRect rectBorder, CRect rectBorderSize);
	virtual void OnDrawFloatingToolbarBorder (	CDC* pDC, CBCGPBaseToolBar* pToolBar, 
												CRect rectBorder, CRect rectBorderSize);
	// Dialog:
	virtual CBrush& GetDlgBackBrush (CWnd* pDlg)
	{
		return m_brDlgBackground.GetSafeHandle () == NULL ? CBCGPVisualManagerVS2008::GetDlgBackBrush (pDlg) : m_brDlgBackground;
	}

	virtual BOOL OnFillDialog (CDC* pDC, CWnd* pDlg, CRect rect);

	// Group:
	virtual void OnDrawGroup (CDC* pDC, CBCGPGroup* pGroup, CRect rect, const CString& strName);

	// Push button:
	virtual BOOL OnDrawPushButton (CDC* pDC, CRect rect, CBCGPButton* pButton, COLORREF& clrText);

#ifndef BCGP_EXCLUDE_GRID_CTRL
	// Grid control:
	virtual void OnFillGridHeaderBackground (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect);
	virtual BOOL OnDrawGridHeaderItemBorder (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect, BOOL bPressed);
	virtual void OnFillGridRowHeaderBackground (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect);
	virtual BOOL OnDrawGridRowHeaderItemBorder (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect, BOOL bPressed);
	virtual void OnFillGridSelectAllAreaBackground (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect, BOOL bPressed);
	virtual BOOL OnDrawGridSelectAllAreaBorder (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect, BOOL bPressed);
	
	virtual COLORREF OnFillGridGroupByBoxBackground (CDC* pDC, CRect rect);
	virtual COLORREF OnFillGridGroupByBoxTitleBackground (CDC* pDC, CRect rect);
	virtual COLORREF GetGridGroupByBoxLineColor () const;
	virtual void OnDrawGridGroupByBoxItemBorder (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect);

	virtual COLORREF GetGridLeftOffsetColor (CBCGPGridCtrl* pCtrl);
	virtual void OnDrawGridGroupUnderline (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rectFill);
	virtual COLORREF OnFillGridRowBackground (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rectFill, BOOL bSelected);
	virtual COLORREF OnFillGridItem (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rectFill,
		BOOL bSelected, BOOL bActiveItem, BOOL bSortedColumn);
	virtual void OnDrawGridSelectionBorder (CBCGPGridCtrl* pCtrl, CDC* pDC, CRect rect);

	// Report control:
	virtual COLORREF OnFillReportCtrlRowBackground (CBCGPGridCtrl* pCtrl, CDC* pDC,
		CRect rectFill, BOOL bSelected, BOOL bGroup);
#endif // BCGP_EXCLUDE_GRID_CTRL

protected:
	CBrush		m_brTabs;

	COLORREF	m_clrMenuItemGradient1;
	COLORREF	m_clrMenuItemGradient2;

	COLORREF	m_clrMenuGutter;
	CBrush		m_brMenuGutter;
	CBrush		m_brStatusBar;

	COLORREF	m_clrMenuBarGradient1;
	COLORREF	m_clrMenuBarGradient2;

	COLORREF	m_clrHighlightedTabGradient1;
	COLORREF	m_clrHighlightedTabGradient2;

	COLORREF	m_clrEditBoxBorder;
	COLORREF	m_clrEditBoxBorderDisabled;

	COLORREF	m_clrCombo;
	COLORREF	m_clrComboDisabled;

	CBrush		m_brWhite;

	COLORREF	m_clrInactiveCaptionGradient1;
	COLORREF	m_clrInactiveCaptionGradient2;

	CBrush		m_brDlgBackground;

	COLORREF	m_clrEditCtrlSelectionBkActive;
	COLORREF	m_clrEditCtrlSelectionTextActive;
	COLORREF	m_clrEditCtrlSelectionBkInactive;
	COLORREF	m_clrEditCtrlSelectionTextInactive;

	CBrush		m_brGridSelectionBkActive;
	COLORREF	m_clrGridSelectionTextActive;
	CBrush		m_brGridSelectionBkInactive;
	COLORREF	m_clrGridSelectionTextInactive;
};

#endif // !defined(AFX_BCGPVISUALMANAGERVS2010_H__F2994064_A72E_4AF2_A8DD_3799EF9D4199__INCLUDED_)
