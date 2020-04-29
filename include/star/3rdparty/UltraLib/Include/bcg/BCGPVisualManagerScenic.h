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
// BCGPVisualManagerScenic.h: interface for the CBCGPVisualManagerScenic class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BCGPVISUALMANAGERSCENIC_H__AA17DB7C_5605_4267_8CED_E539BC7BFED6__INCLUDED_)
#define AFX_BCGPVISUALMANAGERSCENIC_H__AA17DB7C_5605_4267_8CED_E539BC7BFED6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BCGPWinXPVisualManager.h"
#include "BCGPControlRenderer.h"
#include "BCGPVisualManager2007.h"

class BCGCBPRODLLEXPORT CBCGPVisualManagerScenic : public CBCGPWinXPVisualManager  
{
	DECLARE_DYNCREATE (CBCGPVisualManagerScenic)

public:
	virtual BOOL SetStyle(LPCTSTR lpszPath = NULL);
	virtual void SetResourceHandle(HINSTANCE hinstRes);
	virtual void CleanStyle();

protected:
	virtual CString MakeResourceID(LPCTSTR lpszID) const;
	virtual CString GetStyleDllName() const;
	virtual CString GetStyleResourceID() const;

public:
	CBCGPVisualManagerScenic();
	virtual ~CBCGPVisualManagerScenic();

#ifndef BCGP_EXCLUDE_RIBBON
	virtual void SetMainButtonColor(COLORREF clr);	// -1 - default (blue)

	BOOL IsRibbonPresent (CWnd* pWnd) const;
	CBCGPRibbonBar*	GetRibbonBar (CWnd* pWnd) const;
#endif

	virtual BOOL IsOwnerDrawMenuCheck ();
	virtual BOOL IsHighlightWholeMenuItem ();

	virtual BOOL OnNcActivate(CWnd* pWnd, BOOL bActive);
	virtual BOOL OnNcPaint (CWnd* pWnd, const CObList& lstSysButtons, CRect rectRedraw);

	virtual void OnDrawComboDropButton (CDC* pDC, CRect rect,
										BOOL bDisabled,
										BOOL bIsDropped,
										BOOL bIsHighlighted,
										CBCGPToolbarComboBoxButton* pButton);

    virtual void DrawSeparator (CDC* pDC, const CRect& rect, CPen& pen1, CPen& pen2, BOOL bHorz);
	virtual void DrawNcBtn (CDC* pDC, const CRect& rect, UINT nButton, 
							BCGBUTTON_STATE state, BOOL bSmall, 
							BOOL bActive, BOOL bMDI = FALSE);
	virtual void DrawNcText (CDC* pDC, CRect& rect, const CString& strTitle, 
							BOOL bActive, BOOL bIsRTL, BOOL bTextCenter,
							BOOL bGlass = FALSE, int nGlassGlowSize = 0, 
							COLORREF clrGlassText = (COLORREF)-1);

	virtual void OnUpdateSystemColors ();
	virtual void CleanUp ();

	virtual void OnHighlightMenuItem (CDC *pDC, CBCGPToolbarMenuButton* pButton,
		CRect rect, COLORREF& clrText);
	virtual void OnDrawMenuBorder (CDC* pDC, CBCGPPopupMenu* pMenu, CRect rect);
	virtual void OnDrawMenuCheck (CDC* pDC, CBCGPToolbarMenuButton* pButton, 
		CRect rect, BOOL bHighlight, BOOL bIsRadio);
	virtual void OnFillMenuImageRect (CDC* pDC,
		CBCGPToolbarButton* pButton, CRect rect, CBCGPVisualManager::BCGBUTTON_STATE state);
	virtual void OnFillButtonInterior (CDC* pDC,
		CBCGPToolbarButton* pButton, CRect rect, CBCGPVisualManager::BCGBUTTON_STATE state);
	virtual void OnDrawButtonBorder (CDC* pDC,
		CBCGPToolbarButton* pButton, CRect rect, CBCGPVisualManager::BCGBUTTON_STATE state);

	virtual void OnDrawSeparator (CDC* pDC, CBCGPBaseControlBar* pBar, CRect rect, BOOL bIsHoriz);
	virtual COLORREF OnDrawMenuLabel (CDC* pDC, CRect rect);

	virtual void OnFillPopupMenuBackground (CDC* pDC, CRect rect);

	virtual COLORREF GetCaptionBarTextColor (CBCGPCaptionBar* pBar);
	virtual void OnDrawCaptionBarInfoArea (CDC* pDC, CBCGPCaptionBar* pBar, CRect rect);

	virtual COLORREF OnFillCaptionBarButton (CDC* pDC, CBCGPCaptionBar* pBar,
											CRect rect, BOOL bIsPressed, BOOL bIsHighlighted, 
											BOOL bIsDisabled, BOOL bHasDropDownArrow,
											BOOL bIsSysButton);
	virtual void OnDrawCaptionBarButtonBorder (CDC* pDC, CBCGPCaptionBar* pBar,
											CRect rect, BOOL bIsPressed, BOOL bIsHighlighted, 
											BOOL bIsDisabled, BOOL bHasDropDownArrow,
											BOOL bIsSysButton);

	virtual void OnFillOutlookBarCaption (CDC* pDC, CRect rectCaption, COLORREF& clrText);

	virtual COLORREF OnFillListBoxItem (CDC* pDC, CBCGPListBox* pListBox, int nItem, CRect rect, BOOL bIsHighlihted, BOOL bIsSelected);

	virtual void OnDrawEditClearButton(CDC* pDC, CBCGPButton* pButton, CRect rect);

	virtual void OnFillBarBackground (CDC* pDC, CBCGPBaseControlBar* pBar,
									CRect rectClient, CRect rectClip,
									BOOL bNCArea = FALSE);

	// Calculator:
	virtual BOOL OnDrawCalculatorButton (CDC* pDC, CRect rect, CBCGPToolbarButton* pButton, CBCGPVisualManager::BCGBUTTON_STATE state, int cmd /* CBCGPCalculator::CalculatorCommands */, CBCGPCalculator* pCalculator);
	virtual BOOL OnDrawCalculatorDisplay (CDC* pDC, CRect rect, const CString& strText, BOOL bMem, CBCGPCalculator* pCalculator);
	virtual COLORREF GetCalculatorButtonTextColor (BOOL bIsUserCommand, int cmd /* CBCGPCalculator::CalculatorCommands */);

#ifndef BCGP_EXCLUDE_RIBBON
	virtual COLORREF GetRibbonQATTextColor (BOOL bDisabled = FALSE)
	{	
		return bDisabled ? m_clrRibbonCategoryTextDisabled : m_clrRibbonCategoryText;
	}

	virtual void OnDrawRibbonPaletteBorder (
					CDC* pDC, 
					CBCGPRibbonPaletteButton* pButton, 
					CRect rectBorder);
	virtual void OnDrawRibbonCaption (
					CDC* pDC, CBCGPRibbonBar* pBar, CRect rectCaption,
					CRect rectText);
	virtual void OnDrawRibbonCaptionButton(CDC* pDC, CBCGPRibbonCaptionButton* pButton);
	virtual COLORREF OnDrawRibbonButtonsGroup (
					CDC* pDC,
					CBCGPRibbonButtonsGroup* pGroup,
					CRect rectGroup);
	virtual void OnDrawRibbonCategory (
					CDC* pDC, 
					CBCGPRibbonCategory* pCategory, 
					CRect rectCategory);
	virtual void OnDrawRibbonCategoryScroll (
					CDC* pDC, 
					CBCGPRibbonCategoryScroll* pScroll);
	virtual COLORREF OnDrawRibbonCategoryTab (
					CDC* pDC, 
					CBCGPRibbonTab* pTab, 
					BOOL bIsActive);
	virtual COLORREF OnDrawRibbonPanel (
					CDC* pDC,
					CBCGPRibbonPanel* pPanel, 
					CRect rectPanel,
					CRect rectCaption);
	virtual COLORREF OnFillRibbonPanelCaption (
					CDC* pDC,
					CBCGPRibbonPanel* pPanel, 
					CRect rectCaption);
	virtual void OnPreDrawRibbon (
					CDC* pDC, 
					CBCGPRibbonBar* pRibbonBar, 
					CRect rectTabs);
	virtual void OnDrawRibbonPaletteButton (
					CDC* pDC,
					CBCGPRibbonPaletteIcon* pButton);
	virtual COLORREF OnDrawRibbonCategoryCaption (
					CDC* pDC, 
					CBCGPRibbonContextCaption* pContextCaption);
	virtual COLORREF OnDrawRibbonStatusBarPane (
					CDC* pDC, 
					CBCGPRibbonStatusBar* pBar,
					CBCGPRibbonStatusBarPane* pPane);
	virtual void OnDrawRibbonSliderZoomButton (
					CDC* pDC, CBCGPRibbonSlider* pSlider, 
					CRect rect, BOOL bIsZoomOut, 
					BOOL bIsHighlighted, BOOL bIsPressed, BOOL bIsDisabled);
	virtual void OnDrawRibbonProgressBar (
					CDC* pDC, CBCGPRibbonProgressBar* pProgress, 
					CRect rectProgress, CRect rectChunk, BOOL bInfiniteMode);
	virtual void OnDrawDefaultRibbonImage (
					CDC* pDC, 
					CRect rectImage,
					BOOL bIsDisabled = FALSE,
					BOOL bIsPressed = FALSE,
					BOOL bIsHighlighted = FALSE);
	virtual void OnDrawRibbonMainButton (
					CDC* pDC, 
					CBCGPRibbonButton* pButton);

	virtual void OnDrawCheckBoxEx (CDC *pDC, CRect rect, 
										 int nState,
										 BOOL bHighlighted, 
										 BOOL bPressed,
										 BOOL bEnabled);
	virtual void OnDrawRadioButton (CDC *pDC, CRect rect, 
										 BOOL bOn,
										 BOOL bHighlighted, 
										 BOOL bPressed,
										 BOOL bEnabled);
	virtual void OnDrawRibbonButtonBorder (
					CDC* pDC, 
					CBCGPRibbonButton* pButton);
	virtual void OnDrawRibbonLaunchButton (
					CDC* pDC,
					CBCGPRibbonLaunchButton* pButton,
					CBCGPRibbonPanel* pPanel);
	virtual COLORREF OnFillRibbonButton(CDC* pDC, CBCGPRibbonButton* pButton);
	virtual void OnDrawRibbonDefaultPaneButton (
					CDC* pDC, 
					CBCGPRibbonButton* pButton);

	virtual int GetRibbonQATChevronOffset ()	{	return 13;	}
	virtual int GetRibbonQATRightMargin ()	{	return 0;	}

	virtual COLORREF GetRibbonEditBackgroundColor (
					CBCGPRibbonEditCtrl* pEdit,
					BOOL bIsHighlighted,
					BOOL bIsPaneHighlighted,
					BOOL bIsDisabled);

	virtual void OnDrawRibbonMenuCheckFrame (
					CDC* pDC,
					CBCGPRibbonButton* pButton, 
					CRect rect);

	virtual void OnFillRibbonQATPopup (
				CDC* pDC, CBCGPRibbonPanelMenuBar* pMenuBar, CRect rect);

	virtual int GetRibbonPopupBorderSize (const CBCGPRibbonPanelMenu* pPopup) const;

	virtual int GetRibbonPanelMargin(CBCGPRibbonCategory* pCategory);

	virtual void OnDrawRibbonRecentFilesFrame (
					CDC* pDC, 
					CBCGPRibbonMainPanel* pPanel, 
					CRect rect);

#endif

protected:
	BOOL CanDrawImage () const
	{
#ifdef BCGP_EXCLUDE_PNG_SUPPORT
		return FALSE;
#else
		return globalData.m_nBitsPerPixel > 8 && 
			!globalData.IsHighContastMode () &&
			m_bLoaded;
#endif
	}

protected:
	CFont	m_AppCaptionFont;
	CPen	m_penSeparatorDark;
	CPen	m_penSeparatorLight;

	BOOL	m_bLoaded;

	BOOL	m_bNcTextCenter;

	COLORREF m_clrRibbonBarBkgnd;
	CBrush   m_brRibbonBarBkgnd;
    COLORREF m_clrRibbonBarGradientLight;
    COLORREF m_clrRibbonBarGradientDark;

	COLORREF m_clrRibbonPanelCaptionText;
	COLORREF m_clrRibbonPanelCaptionTextHighlighted;

	COLORREF m_clrRibbonEdit;
	COLORREF m_clrRibbonEditDisabled;
	COLORREF m_clrRibbonEditHighlighted;
	COLORREF m_clrRibbonEditPressed;
	COLORREF m_clrRibbonEditBorder;
	COLORREF m_clrRibbonEditBorderDisabled;
	COLORREF m_clrRibbonEditBorderHighlighted;
	COLORREF m_clrRibbonEditBorderPressed;
	COLORREF m_clrRibbonEditSelection;
	CBCGPControlRenderer m_ctrlRibbonComboBoxBtn;

	CBCGPControlRenderer m_ctrlMenuItemBack;
    CBCGPToolBarImages   m_MenuItemMarkerC;
    CBCGPToolBarImages   m_MenuItemMarkerR;
	CBCGPControlRenderer m_ctrlMenuHighlighted[2];

	CBCGPControlRenderer m_ctrlRibbonCaptionQA;

    COLORREF             m_clrRibbonCategoryText;
    COLORREF             m_clrRibbonCategoryTextHighlighted;
	COLORREF             m_clrRibbonCategoryTextDisabled;
	CBCGPControlRenderer m_ctrlRibbonCategoryBack;
	CBCGPControlRenderer m_ctrlRibbonCategoryTab;
	CBCGPControlRenderer m_ctrlRibbonCategoryTabSep;
	CBCGPControlRenderer m_ctrlRibbonCategoryTabFrame;
	CBCGPControlRenderer m_ctrlRibbonCategoryBtnPage[2];
	CBCGPControlRenderer m_ctrlRibbonPanelBack;
	CBCGPControlRenderer m_ctrlRibbonPanelBackSep;
	CBCGPToolBarImages   m_RibbonPanelSeparator;
	CBCGPControlRenderer m_ctrlRibbonPanelQAT;
	CBCGPControlRenderer m_ctrlRibbonMainPanel;
	CBCGPControlRenderer m_ctrlRibbonBtnMainPanel;
	CBCGPControlRenderer m_ctrlRibbonBtnGroup_S;
	CBCGPControlRenderer m_ctrlRibbonBtnGroup_F;
	CBCGPControlRenderer m_ctrlRibbonBtnGroup_M;
	CBCGPControlRenderer m_ctrlRibbonBtnGroup_L;
	CBCGPControlRenderer m_ctrlRibbonBtnGroupMenu_F[2];
	CBCGPControlRenderer m_ctrlRibbonBtnGroupMenu_M[2];
	CBCGPControlRenderer m_ctrlRibbonBtnGroupMenu_L[2];
	CBCGPControlRenderer m_ctrlRibbonBtn[2];
	CBCGPControlRenderer m_ctrlRibbonBtnMenuH[2];
	CBCGPControlRenderer m_ctrlRibbonBtnMenuV[2];
	CBCGPControlRenderer m_ctrlRibbonBtnLaunch;
	CBCGPToolBarImages m_RibbonBtnLaunchIcon;
	CBCGPControlRenderer m_ctrlRibbonBtnMain;
	CBCGPControlRenderer m_ctrlRibbonBtnMainColorized;
	CBCGPControlRenderer m_ctrlRibbonSliderBtnPlus;
	CBCGPControlRenderer m_ctrlRibbonSliderBtnMinus;
	CBCGPToolBarImages	 m_RibbonBtnDefaultImage;
	CBCGPControlRenderer m_ctrlRibbonBtnDefault;
	CBCGPControlRenderer m_ctrlRibbonBtnDefaultIcon;
	CBCGPControlRenderer m_ctrlRibbonBtnDefaultQAT;
	CBCGPControlRenderer m_ctrlRibbonBtnStatusPane;
	CBCGPControlRenderer m_ctrlRibbonBtnPalette[3];

	CBCGPControlRenderer m_ctrlRibbonBorder_QAT;
	CBCGPControlRenderer m_ctrlRibbonBorder_Floaty;
	CBCGPControlRenderer m_ctrlRibbonBorder_Panel;

	struct XRibbonContextCategory
	{
		CBCGPControlRenderer	m_ctrlBtnDefault;
		CBCGPControlRenderer	m_ctrlCaption;
		CBCGPControlRenderer	m_ctrlBack;
		CBCGPControlRenderer	m_ctrlTab;
		CBCGPControlRenderer	m_ctrlSeparator;
		CBCGPControlRenderer	m_ctrlPanelBack;
		CBCGPBitmapCache		m_cacheBack;
		CBCGPBitmapCache		m_cacheBtnDefault;
		CBCGPBitmapCache		m_cachePanelBack;
		COLORREF				m_clrCaptionText;
		COLORREF				m_clrText;
		COLORREF				m_clrTextHighlighted;

		void CleanUp ()
		{
			m_ctrlCaption.CleanUp ();
			m_ctrlBack.CleanUp ();
			m_ctrlTab.CleanUp ();
			m_ctrlSeparator.CleanUp ();
			m_ctrlBtnDefault.CleanUp ();
			m_ctrlPanelBack.CleanUp ();
			m_cacheBack.Clear ();
			m_cacheBtnDefault.Clear ();
			m_cachePanelBack.Clear ();
		}
	};

	CBCGPControlRenderer	m_ctrlRibbonContextSeparator;
	XRibbonContextCategory  m_ctrlRibbonContextCategory[BCGPRibbonCategoryColorCount];

	CBCGPBitmapCache m_cacheRibbonCategoryBack;
	CBCGPBitmapCache m_cacheRibbonPanelBack;
	CBCGPBitmapCache m_cacheRibbonBtnGroup_S;
	CBCGPBitmapCache m_cacheRibbonBtnGroup_F;
	CBCGPBitmapCache m_cacheRibbonBtnGroup_M;
	CBCGPBitmapCache m_cacheRibbonBtnGroup_L;
	CBCGPBitmapCache m_cacheRibbonBtnGroupMenu_F[2];
	CBCGPBitmapCache m_cacheRibbonBtnGroupMenu_M[2];
	CBCGPBitmapCache m_cacheRibbonBtnGroupMenu_L[2];
	CBCGPBitmapCache m_cacheRibbonBtnDefault;

	CString   m_strStylePrefix;
	HINSTANCE m_hinstRes;
	BOOL      m_bAutoFreeRes;

	CMap<HWND, HWND, BOOL, BOOL> m_ActivateFlag;

	BOOL IsWindowActive(CWnd* pWnd) const;
	CSize GetSystemBorders(BOOL bRibbonPresent) const;
};

#endif // !defined(AFX_BCGPVISUALMANAGERSCENIC_H__AA17DB7C_5605_4267_8CED_E539BC7BFED6__INCLUDED_)
