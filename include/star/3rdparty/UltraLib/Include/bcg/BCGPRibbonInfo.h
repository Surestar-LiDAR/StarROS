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
// BCGPRibbonInfo.h: interface for the CBCGPRibbonInfo class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BCGPRIBBONINFO_H__95B73FBC_1912_49D2_85B6_868E95E3E207__INCLUDED_)
#define AFX_BCGPRIBBONINFO_H__95B73FBC_1912_49D2_85B6_868E95E3E207__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BCGPToolBarImages.h"
#include "BCGPRibbonBar.h"

#ifndef BCGP_EXCLUDE_RIBBON

class CBCGPRibbonInfo 
{
public:
	enum XImages
	{
		e_ImagesFirst   = 0,
		e_ImagesSmall   = e_ImagesFirst,
		e_ImagesLarge   = 1,
		e_ImagesSBGroup = 2,
		e_ImagesLast    = e_ImagesSBGroup
	};

	enum XUse
	{
		e_UseRibbon  = 0x01,
		e_UseStatus  = 0x02
	};

public:
	class XID
	{
	public:
		XID ();
		~XID ();
		
        BOOL IsEmpty() const { return m_Name.IsEmpty() && m_Value == 0; }

		BOOL FromTag (const CString& strTag);
		void ToTag (CString& strTag) const;

		BOOL operator == (const XID& id)
		{
			return m_Name == id.m_Name && m_Value == id.m_Value;
		}

	public:
		CString	m_Name;
		UINT	m_Value;
	};
	
	class XImage
	{
	public:
		XImage ();
		~XImage ();

        BOOL IsEmpty() const { return m_ID.IsEmpty() || !m_Image.IsValid(); }

		BOOL FromTag (const CString& strTag);
		void ToTag (CString& strTag) const;

	public:
		XID                 m_ID;
		CBCGPToolBarImages	m_Image;
	};
	typedef CArray<XImage*, XImage*> XArrayImages;

	class XBase
	{
	protected:
		XBase(const CString& strElementName);

	public:
		virtual ~XBase();

		static XBase* CreateFromName (const CString& name);
		static XBase* CreateFromTag (const CString& tag);

		inline const CString& GetElementName () const
		{
			return m_strElementName;
		}

		virtual BOOL FromTag (const CString& strTag) = 0;
		virtual void ToTag (CString& strTag) const;

	private:
		CString			m_strElementName;
	};

	class XElement: public XBase
	{
	protected:
		XElement(const CString& strElementName);

	public:
		virtual ~XElement();

		static XElement* CreateFromName (const CString& name);

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XID				m_ID;
		CString			m_strText;
		CString			m_strToolTip;
		CString			m_strDescription;
		CString			m_strKeys;
		CString			m_strMenuKeys;
		BOOL			m_bIsOnPaletteTop;
		BOOL			m_bIsAlwaysLarge;
	};
	typedef CArray<XElement*, XElement*> XArrayElement;

	class XElementSeparator: public XElement
	{
	public:
		XElementSeparator();
		virtual ~XElementSeparator();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

		BOOL			m_bIsHoriz;
	};

	class XElementGroup: public XElement
	{
	public:
		XElementGroup();
		virtual ~XElementGroup();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XImage			m_Images;
		XArrayElement	m_arButtons;
	};

	class XElementButton: public XElement
	{
	protected:
		XElementButton(const CString& strElementName);

	public:
		XElementButton();
		virtual ~XElementButton();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		int				m_nSmallImageIndex;
		int				m_nLargeImageIndex;
		BOOL			m_bIsDefaultCommand;
		BOOL			m_bIsAlwaysShowDescription;
		XArrayElement	m_arSubItems;
	};

	class XElementLabel: public XElementButton
	{
	public:
		XElementLabel();
		virtual ~XElementLabel();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementButtonCheck: public XElementButton
	{
	public:
		XElementButtonCheck();
		virtual ~XElementButtonCheck();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementButtonRadio: public XElementButton
	{
	public:
		XElementButtonRadio();
		virtual ~XElementButtonRadio();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementButtonHyperlink: public XElementButton
	{
	public:
		XElementButtonHyperlink();
		virtual ~XElementButtonHyperlink();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		CString			m_strLink;
	};

	class XElementEdit: public XElementButton
	{
	protected:
		XElementEdit(const CString& strElementName);

	public:
		XElementEdit();
		virtual ~XElementEdit();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		int				m_nWidth;
		int				m_nWidthFloaty;
		int				m_nTextAlign;
		BOOL            m_bHasSpinButtons;
		int				m_nMin;
		int				m_nMax;
		CString			m_strValue;
		BOOL			m_bSearchMode;
		CString			m_strSearchPrompt;
	};

	class XElementComboBox: public XElementEdit
	{
	protected:
		XElementComboBox(const CString& strElementName);

	public:
		XElementComboBox();
		virtual ~XElementComboBox();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		BOOL            m_bHasEditBox;
		BOOL			m_bHasDropDownList;
		BOOL			m_bResizeDropDownList;
		CStringArray	m_arItems;
		BOOL			m_bCalculatorMode;
		CList<UINT, UINT>
						m_lstCalculatorExt;
	};

	class XElementFontComboBox: public XElementComboBox
	{
	public:
		XElementFontComboBox();
		virtual ~XElementFontComboBox();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		int				m_nFontType;
		BYTE			m_nCharSet;
		BYTE			m_nPitchAndFamily;
	};

	class XElementButtonPalette: public XElementButton
	{
	public:
		class XPaletteGroup
		{
		public:
			XPaletteGroup();
			~XPaletteGroup();

			BOOL FromTag (const CString& strTag);
			void ToTag (CString& strTag) const;

		public:
			CString			m_strName;
			int             m_nItems;
		};
		typedef CArray<XPaletteGroup*, XPaletteGroup*> XArrayPaletteGroup;

	protected:
		XElementButtonPalette(const CString& strElementName);

	public:
		XElementButtonPalette();
		virtual ~XElementButtonPalette();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		BOOL			m_bIsButtonMode;
		BOOL			m_bEnableMenuResize;
		BOOL			m_bMenuResizeVertical;
		int				m_nIconsInRow;
		CSize			m_sizeIcon;
		XImage			m_Images;
		XArrayPaletteGroup	m_arGroups;
	};

	class XElementButtonColor: public XElementButtonPalette
	{
	public:
		XElementButtonColor();
		virtual ~XElementButtonColor();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		COLORREF		m_clrColor;
		BOOL			m_bSimpleButtonLook;

		CString			m_strAutomaticBtnLabel;
		CString			m_strAutomaticBtnToolTip;
		COLORREF		m_clrAutomaticBtnColor;
		BOOL			m_bAutomaticBtnOnTop;
		BOOL			m_bAutomaticBtnBorder;

		CString			m_strOtherBtnLabel;
		CString			m_strOtherBtnToolTip;
	};

	class XElementButtonUndo: public XElementButtonPalette
	{
	public:
		XElementButtonUndo();
		virtual ~XElementButtonUndo();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementButtonLaunch: public XElementButton
	{
	public:
		XElementButtonLaunch();
		virtual ~XElementButtonLaunch();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementButtonMain: public XElementButton
	{
	public:
		XElementButtonMain();
		virtual ~XElementButtonMain();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XImage			m_Image;
	};

	class XElementButtonMainPanel: public XElementButton
	{
	public:
		XElementButtonMainPanel();
		virtual ~XElementButtonMainPanel();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;
	};

	class XElementSlider: public XElement
	{
	public:
		XElementSlider();
		virtual ~XElementSlider();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		DWORD			m_dwStyle;
		int				m_nWidth;
		int				m_nMin;
		int				m_nMax;
		int				m_nPos;
		BOOL			m_bZoomButtons;
	};

	class XElementProgressBar: public XElement
	{
	public:
		XElementProgressBar();
		virtual ~XElementProgressBar();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		int				m_nWidth;
		int				m_nHeight;
		int				m_nMin;
		int				m_nMax;
		int				m_nPos;
		BOOL			m_bInfinite;
	};

	class XElementStatusPane: public XElementButton
	{
	public:
		XElementStatusPane();
		virtual ~XElementStatusPane();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		CString			m_strAlmostLargeText;
		BOOL			m_bIsStatic;
		int				m_nTextAlign;
	};

	class XPanel: public XBase
	{
	public:
		XPanel();
		virtual ~XPanel();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		CString					m_strName;
		CString					m_strKeys;
		int						m_nImageIndex;
		BOOL					m_bJustifyColumns;
		BOOL					m_bCenterColumnVert;
		XElementButtonLaunch	m_btnLaunch;
		XArrayElement			m_arElements;
	};
	typedef CArray<XPanel*, XPanel*> XArrayPanel;

	class XCategory: public XBase
	{
	public:
		XCategory();
		virtual ~XCategory();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		CString					m_strName;
		CString					m_strKeys;
		XImage					m_SmallImages;
		XImage					m_LargeImages;
		XArrayPanel				m_arPanels;
		XArrayElement			m_arElements;
	};
	typedef CArray<XCategory*, XCategory*> XArrayCategory;

	class XContext: public XBase
	{
	public:
		XContext();
		virtual ~XContext();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XID						m_ID;
		CString					m_strText;
		BCGPRibbonCategoryColor	m_Color;
		XArrayCategory			m_arCategories;
	};
	typedef CArray<XContext*, XContext*> XArrayContext;

	class XCategoryMain: public XBase
	{
	public:
		XCategoryMain();
		virtual ~XCategoryMain();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		CString					m_strName;
		XImage					m_SmallImages;
		XImage					m_LargeImages;
		XArrayElement			m_arElements;
		BOOL					m_bRecentListEnable;
		CString					m_strRecentListLabel;
		int						m_nRecentListWidth;

		BOOL					m_bSearchEnable;
		CString					m_strSearchLabel;
		CString					m_strSearchKeys;
		int                     m_nSearchWidth;
	};

	class XQAT: public XBase
	{
	public:
		class XQATItem
		{
		public:
			XQATItem();
			~XQATItem();

			BOOL FromTag (const CString& strTag);
			void ToTag (CString& strTag) const;

		public:
			XID		m_ID;
			BOOL	m_bVisible;
		};
		typedef CArray<XQATItem, XQATItem> XArrayQATItem;

	public:
		XQAT();
		virtual ~XQAT();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XArrayQATItem	m_arItems;
		BOOL			m_bOnTop;
	};

	class XRibbonBar: public XBase
	{
	public:
		XRibbonBar();
		virtual ~XRibbonBar();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		BOOL					m_bToolTip;
		BOOL					m_bToolTipDescr;
		BOOL					m_bKeyTips;
		BOOL					m_bPrintPreview;
		BOOL					m_bDrawUsingFont;

		XImage					m_Images;
		XElementButtonMain*		m_btnMain;
		XCategoryMain*			m_MainCategory;
		XQAT					m_QAT;
		XElementGroup			m_TabElements;
		XArrayCategory			m_arCategories;
		XArrayContext			m_arContexts;
	};

	class XStatusBar: public XBase
	{
	public:
		class XStatusElements
		{
		public:
			XStatusElements();
			~XStatusElements();

			BOOL FromTag (const CString& strTag);
			void ToTag (CString& strTag) const;

		public:
			XArrayElement		m_arElements;
			CStringArray		m_arLabels;
			CArray<BOOL, BOOL>	m_arVisible;
		};

	public:
		XStatusBar();
		virtual ~XStatusBar();

		virtual BOOL FromTag (const CString& strTag);
		virtual void ToTag (CString& strTag) const;

	public:
		XStatusElements			m_Elements;
		XStatusElements			m_ExElements;
		XImage					m_Images;
	};

public:
	CBCGPRibbonInfo();
	virtual ~CBCGPRibbonInfo();

	virtual BOOL FromTag (const CString& strTag, DWORD use = e_UseRibbon | e_UseStatus);
	virtual void ToTag (CString& strTag, DWORD use = e_UseRibbon | e_UseStatus) const;

	void GetArrayImages (XArrayImages& images);

	inline CSize& GetImageSize (XImages image)
	{
		ASSERT (e_ImagesFirst <= image && image <= e_ImagesLast);

		return m_sizeImage[image];
	}
	inline const CSize& GetImageSize (XImages image) const
	{
		ASSERT (e_ImagesFirst <= image && image <= e_ImagesLast);

		return m_sizeImage[image];
	}

	inline DWORD GetVersion () const
	{
		return m_dwVersion;
	}

	inline XRibbonBar& GetRibbonBar ()
	{
		return m_RibbonBar;
	}
	inline const XRibbonBar& GetRibbonBar () const
	{
		return m_RibbonBar;
	}

	inline XStatusBar& GetStatusBar ()
	{
		return m_StatusBar;
	}
	inline const XStatusBar& GetStatusBar () const
	{
		return m_StatusBar;
	}

protected:
	void AddElementImages (XElement& info, XArrayImages& images);

public:
	static LPCTSTR s_szButton;
	static LPCTSTR s_szButton_Check;
	static LPCTSTR s_szButton_Radio;
	static LPCTSTR s_szButton_Color;
	static LPCTSTR s_szButton_Undo;
	static LPCTSTR s_szButton_Palette;
	static LPCTSTR s_szButton_Hyperlink;
	static LPCTSTR s_szButton_Main;
	static LPCTSTR s_szButton_MainPanel;
	static LPCTSTR s_szButton_Launch;
	static LPCTSTR s_szLabel;
	static LPCTSTR s_szEdit;
	static LPCTSTR s_szComboBox;
	static LPCTSTR s_szComboBox_Font;
	static LPCTSTR s_szSlider;
	static LPCTSTR s_szProgress;
	static LPCTSTR s_szSeparator;
	static LPCTSTR s_szGroup;
	static LPCTSTR s_szStatusPane;
	static LPCTSTR s_szPanel;
	static LPCTSTR s_szCategory;
	static LPCTSTR s_szContext;
	static LPCTSTR s_szCategoryMain;
	static LPCTSTR s_szQAT;
	static LPCTSTR s_szRibbonBar;
	static LPCTSTR s_szStatusBar;

private:
	CSize		m_sizeImage[e_ImagesLast + 1];
	XRibbonBar	m_RibbonBar;
	XStatusBar	m_StatusBar;
	DWORD		m_dwVersion;
};

#endif // BCGP_EXCLUDE_RIBBON

#endif // !defined(AFX_BCGPRIBBONINFO_H__95B73FBC_1912_49D2_85B6_868E95E3E207__INCLUDED_)
