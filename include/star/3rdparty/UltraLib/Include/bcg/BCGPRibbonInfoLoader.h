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
// BCGPRibbonInfoLoader.h: interface for the CBCGPRibbonInfoLoader class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BCGPRIBBONINFOLOADER_H__721B7C0B_B5A3_4619_BEF9_272BEBF1C81E__INCLUDED_)
#define AFX_BCGPRIBBONINFOLOADER_H__721B7C0B_B5A3_4619_BEF9_272BEBF1C81E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BCGPRibbonInfo.h"

#ifndef BCGP_EXCLUDE_RIBBON

class CBCGPRibbonInfoLoader  
{
public:
	CBCGPRibbonInfoLoader (CBCGPRibbonInfo& info, 
		DWORD dwFlags = (CBCGPRibbonInfo::e_UseRibbon | CBCGPRibbonInfo::e_UseStatus));
	virtual ~CBCGPRibbonInfoLoader();

	BOOL Load (UINT uiResID, LPCTSTR lpszResType = _T("BCGP_RIBBON_XML"), HINSTANCE hInstance = NULL);
	BOOL Load (LPCTSTR lpszResID, LPCTSTR lpszResType = _T("BCGP_RIBBON_XML"), HINSTANCE hInstance = NULL);

	virtual BOOL LoadFromBuffer (LPCTSTR lpszBuffer);
	virtual BOOL LoadImage (CBCGPRibbonInfo::XImage& image, BOOL bSingle = FALSE);

protected:
	virtual BOOL LoadImage (const CBCGPRibbonInfo::XID& id, CBCGPToolBarImages& image, BOOL bSingle);

private:
	CBCGPRibbonInfo&		m_Info;
	DWORD					m_dwFlags;
	HINSTANCE				m_hInstance;
};

#endif // BCGP_EXCLUDE_RIBBON

#endif // !defined(AFX_BCGPRIBBONINFOLOADER_H__721B7C0B_B5A3_4619_BEF9_272BEBF1C81E__INCLUDED_)
