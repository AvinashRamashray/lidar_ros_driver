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

#if !defined(AFX_BCGPTOOLBARDROPSOURCE_H__8ED1EE63_C585_11D1_B110_86F6097DAC36__INCLUDED_)
#define AFX_BCGPTOOLBARDROPSOURCE_H__8ED1EE63_C585_11D1_B110_86F6097DAC36__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
// BCGToolbarDropSource.h : header file
//

#ifndef __AFXOLE_H__
#include <afxole.h>
#endif

#include "BCGCBPro.h"

/////////////////////////////////////////////////////////////////////////////
// CBCGPToolbarDropSource command target

class BCGCBPRODLLEXPORT CBCGPToolbarDropSource : public COleDropSource
{
public:
	CBCGPToolbarDropSource();
	virtual ~CBCGPToolbarDropSource();

// Attributes
public:
	BOOL	m_bDeleteOnDrop;
	BOOL	m_bEscapePressed;
	BOOL	m_bDragStarted;
	HCURSOR	m_hcurDelete;
	HCURSOR	m_hcurMove;
	HCURSOR	m_hcurCopy;

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBCGPToolbarDropSource)
	public:
	virtual SCODE GiveFeedback(DROPEFFECT dropEffect);
	virtual SCODE QueryContinueDrag(BOOL bEscapePressed, DWORD dwKeyState);
	virtual BOOL OnBeginDrag(CWnd* pWnd);
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CBCGPToolbarDropSource)
		// NOTE - the ClassWizard will add and remove member functions here.
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BCGPTOOLBARDROPSOURCE_H__8ED1EE63_C585_11D1_B110_86F6097DAC36__INCLUDED_)
