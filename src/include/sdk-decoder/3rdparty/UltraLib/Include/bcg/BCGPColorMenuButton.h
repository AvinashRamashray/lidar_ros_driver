// BCGColorMenuButton.h: interface for the CBCGPColorMenuButton class.
//
//////////////////////////////////////////////////////////////////////

// This is a part of the BCGControlBar Library
// Copyright (C) 1998-2010 BCGSoft Ltd.
// All rights reserved.
//
// This source code can be used, distributed or modified
// only under terms and conditions 
// of the accompanying license agreement.

#if !defined(AFX_BCGPCOLORMENUBUTTON_H__B5E358A5_9FBF_40EF_94DA_983CEEECB6C1__INCLUDED_)
#define AFX_BCGPCOLORMENUBUTTON_H__B5E358A5_9FBF_40EF_94DA_983CEEECB6C1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BCGCBPro.h"
#include "BCGPToolbarMenuButton.h"

BCGCBPRODLLEXPORT extern UINT BCGM_GETDOCUMENTCOLORS;

class BCGCBPRODLLEXPORT CBCGPColorMenuButton : public CBCGPToolbarMenuButton  
{
	DECLARE_SERIAL(CBCGPColorMenuButton)

public:
	CBCGPColorMenuButton();
	CBCGPColorMenuButton(UINT uiCmdID, LPCTSTR lpszText,
						CPalette* pPalette = NULL);
	virtual ~CBCGPColorMenuButton();

	virtual void SetColor (COLORREF clr, BOOL bNotify = TRUE);
	COLORREF GetColor () const
	{
		return m_Color;
	}

	static COLORREF GetColorByCmdID (UINT uiCmdID);
	static void SetColorByCmdID (UINT uiCmdID, COLORREF color)
	{
		m_ColorsByID.SetAt (uiCmdID, color);
	}

	void SetColumnsNumber (int nColumns)
	{
		m_nColumns = nColumns;
	}

	void EnableAutomaticButton (LPCTSTR lpszLabel, COLORREF colorAutomatic, BOOL bEnable = TRUE);
	void EnableOtherButton (LPCTSTR lpszLabel, BOOL bAltColorDlg = TRUE, BOOL bEnable = TRUE);
	void EnableDocumentColors (LPCTSTR lpszLabel, BOOL bEnable = TRUE);
	void EnableTearOff (UINT uiID, 
						int nVertDockColumns = -1,
						int nHorzDockRows = -1);

	static void SetColorName (COLORREF color, const CString& strName);

	COLORREF GetAutomaticColor () const
	{
		return m_colorAutomatic;
	}

	virtual BOOL OpenColorDialog (const COLORREF colorDefault, COLORREF& colorRes);
	virtual void OnChangeParentWnd (CWnd* pWndParent);

protected:
	virtual void OnDraw (CDC* pDC, const CRect& rect, CBCGPToolBarImages* pImages,
						BOOL bHorz = TRUE, BOOL bCustomizeMode = FALSE,
						BOOL bHighlight = FALSE,
						BOOL bDrawBorder = TRUE,
						BOOL bGrayDisabledButtons = TRUE);
	virtual CBCGPPopupMenu* CreatePopupMenu ();
	virtual BOOL IsEmptyMenuAllowed () const
	{
		return TRUE;
	}

	virtual void Serialize (CArchive& ar);
	virtual void CopyFrom (const CBCGPToolbarButton& src);
	virtual int OnDrawOnCustomizeList (CDC* pDC, const CRect& rect, BOOL bSelected);

	void Initialize ();

protected:
	COLORREF					m_Color;	// Currently selected color
	COLORREF					m_colorAutomatic;
	CArray<COLORREF, COLORREF>	m_Colors;
	CPalette					m_Palette;
	int							m_nColumns;
	int							m_nVertDockColumns;
	int							m_nHorzDockRows;

	BOOL						m_bIsAutomaticButton;
	BOOL						m_bIsOtherButton;
	BOOL						m_bIsDocumentColors;

	CString						m_strAutomaticButtonLabel;
	CString						m_strOtherButtonLabel;
	CString						m_strDocumentColorsLabel;

	BOOL						m_bStdColorDlg;

	static CMap<UINT,UINT,COLORREF, COLORREF>	m_ColorsByID;
};

#endif // !defined(AFX_BCGPCOLORMENUBUTTON_H__B5E358A5_9FBF_40EF_94DA_983CEEECB6C1__INCLUDED_)
