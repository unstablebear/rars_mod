// DlgStartRaceBulle.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDlgStartRaceBulle dialog

class CDlgStartRaceBulle : public CDialog
{
// Construction
public:
   CDlgStartRaceBulle(CWnd* pParent = NULL);   // standard constructor
         
// Dialog Data
	//{{AFX_DATA(CDlgStartRaceBulle)
	enum { IDD = IDD_StartRaceBulle };
	CStatic	m_StaticRes0;
	CListBox	m_ListTrack;
	CComboBox	m_ComboTrack;
	CComboBox	m_ComboStyle;
	CStatic	m_StaticRes2;
	CStatic	m_StaticRes1;
	CString	m_sTrackName;
	int		m_iCars;
	int		m_iLaps;
	CString	m_sOptiStyle;
	int		m_iOptiNb;
	int		m_iOptiStart;
	//}}AFX_DATA

   void CDlgStartRaceBulle::InitRace( CString sTrack );

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDlgStartRaceBulle)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CDlgStartRaceBulle)
	virtual BOOL OnInitDialog();
	virtual void OnOK();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnAddTrack();
	afx_msg void OnAddAllTrack();
	afx_msg void OnDelTrack();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

   int m_ListNb, m_ListCpt;

};
