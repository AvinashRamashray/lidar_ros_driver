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

#if !defined(AFX_KEYBOARDPAGE_H__283E6042_54C6_11D2_B110_D085EB8D1B3C__INCLUDED_)
#define AFX_KEYBOARDPAGE_H__283E6042_54C6_11D2_B110_D085EB8D1B3C__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
// BCGPKeyboardPage.h : header file
//

#include "bcgprores.h"
#include "KeyAssign.h"

class CBCGPToolbarButton;

/////////////////////////////////////////////////////////////////////////////
// CBCGPKeyboardPage dialog

class CBCGPKeyboardPage : public CPropertyPage
{
	DECLARE_DYNCREATE(CBCGPKeyboardPage)

// Construction
public:
	CBCGPKeyboardPage(CFrameWnd* pParentFrame = NULL, BOOL bAutoSet = FALSE);
	~CBCGPKeyboardPage();

	void SetAllCategory (LPCTSTR lpszCategory);

// Dialog Data
	//{{AFX_DATA(CBCGPKeyboardPage)
	enum { IDD = IDD_BCGBARRES_PROPPAGE5 };
	CStatic	m_wndAssignedToTitle;
	CKeyAssign	m_wndNewKey;
	CComboBox	m_wndViewTypeList;
	CStatic	m_wndViewIcon;
	CButton	m_wndRemoveButton;
	CListBox	m_wndCurrentKeysList;
	CListBox	m_wndCommandsList;
	CComboBox	m_wndCategoryList;
	CButton	m_wndAssignButton;
	CString	m_strDesrcription;
	CString	m_strAssignedTo;
	//}}AFX_DATA


// Overrides
	// ClassWizard generate virtual function overrides
	//{{AFX_VIRTUAL(CBCGPKeyboardPage)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	// Generated message map functions
	//{{AFX_MSG(CBCGPKeyboardPage)
	virtual BOOL OnInitDialog();
	afx_msg void OnAssign();
	afx_msg void OnSelchangeCategory();
	afx_msg void OnSelchangeCommandsList();
	afx_msg void OnSelchangeCurrentKeysList();
	afx_msg void OnRemove();
	afx_msg void OnResetAll();
	afx_msg void OnSelchangeViewType();
	afx_msg void OnUpdateNewShortcutKey();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

// Operations:
protected:
	void AddKeyEntry (LPACCEL pEntry);

// Attributes:
protected:
	HACCEL				m_hAccelTable;
	LPACCEL				m_lpAccel;
	int					m_nAccelSize;
	CMultiDocTemplate*	m_pSelTemplate;
	CBCGPToolbarButton*	m_pSelButton;
	LPACCEL				m_pSelEntry;
	CFrameWnd*			m_pParentFrame;
	BOOL				m_bAutoSet;
	CString				m_strAllCategory;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_KEYBOARDPAGE_H__283E6042_54C6_11D2_B110_D085EB8D1B3C__INCLUDED_)
