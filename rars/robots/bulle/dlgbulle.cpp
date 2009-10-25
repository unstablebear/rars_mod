// DlgStartRaceBulle.cpp : implementation file
//

#include "stdafx.h"
#include "g_global.h"
#include "vc_rars.h"
#include "dlgbulle.h"
#include "dlgstart.h"
#include "_opti.h"
#include <direct.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


extern long race_count;
extern RaceManager g_RaceManager;
extern int gSaveSpeed;
extern int track_count;
extern long ml;

/////////////////////////////////////////////////////////////////////////////

// CDlgStartRaceBulle dialog


CDlgStartRaceBulle::CDlgStartRaceBulle(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgStartRaceBulle::IDD, pParent)
{
	//{{AFX_DATA_INIT(CDlgStartRaceBulle)
	m_sTrackName = _T("");
	m_iCars = 0;
	m_iLaps = 0;
	m_sOptiStyle = _T("");
	m_iOptiNb = 0;
	m_iOptiStart = 0;
	//}}AFX_DATA_INIT
}


void CDlgStartRaceBulle::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDlgStartRaceBulle)
	DDX_Control(pDX, IDC_RES0, m_StaticRes0);
	DDX_Control(pDX, IDC_LIST_TRACK, m_ListTrack);
	DDX_Control(pDX, IDC_TRACK_NAME, m_ComboTrack);
	DDX_Control(pDX, IDC_OPTI_STYLE, m_ComboStyle);
	DDX_Control(pDX, IDC_RES2, m_StaticRes2);
	DDX_Control(pDX, IDC_RES1, m_StaticRes1);
	DDX_Text(pDX, IDC_TRACK_NAME, m_sTrackName);
	DDX_Text(pDX, IDC_CARS, m_iCars);
	DDV_MinMaxInt(pDX, m_iCars, 1, 16);
	DDX_Text(pDX, IDC_LAPS, m_iLaps);
	DDV_MinMaxInt(pDX, m_iLaps, 1, 80);
	DDX_CBString(pDX, IDC_OPTI_STYLE, m_sOptiStyle);
	DDX_Text(pDX, IDC_OPTI_NB, m_iOptiNb);
	DDX_Text(pDX, IDC_OPTI_START, m_iOptiStart);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CDlgStartRaceBulle, CDialog)
	//{{AFX_MSG_MAP(CDlgStartRaceBulle)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_ADD_TRACK, OnAddTrack)
	ON_BN_CLICKED(IDC_ADD_ALL_TRACK, OnAddAllTrack)
	ON_BN_CLICKED(IDC_DEL_TRACK, OnDelTrack)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDlgStartRaceBulle message handlers

BOOL CDlgStartRaceBulle::OnInitDialog() 
{
	CDialog::OnInitDialog();
   m_iLaps = 1;
   m_iCars = 1;
   m_iOptiNb = 1;
   m_iOptiStart = 0;
   m_sTrackName = "V01.trk";
   m_sOptiStyle = "-OBASIS";
   UpdateData( FALSE );

   char filename[256];
   strcpy( filename, "*.tr*" );
   chdir("tracks");
   DlgDirListComboBox( filename, IDC_TRACK_NAME, 0, 0 );
   chdir("..");
   m_ComboTrack.SetCurSel(0);
   
   m_ComboStyle.AddString( "-OBASIS" );
   
   return TRUE;
}

void CDlgStartRaceBulle::OnOK() 
{
   char sz[256];

   UpdateData( TRUE );
   m_ListNb = m_ListTrack.GetCount();

   if( m_ListNb==0 ) {
      InitRace( m_sTrackName );
   } else {
      m_ListCpt = 0;
      m_ListTrack.GetText( m_ListCpt, sz );
      InitRace( (CString) sz );
   }
   SetTimer(1,10,NULL);
}

void CDlgStartRaceBulle::OnTimer(UINT nIDEvent) 
{
   OptiRaceBefore( ml );
   g_RaceManager.AllInit();
   g_RaceManager.RaceInit(ml);
   while( g_RaceManager.RaceLoop() );
   g_RaceManager.RaceClose(ml);
   // g_RaceManager.AllClose();
   OptiRaceAfter( ml );

   ml++;
   if( ml>race_count ) {
      g_RaceManager.AllClose();
      m_ListCpt++;
      if( m_ListNb==0 || m_ListNb<=m_ListCpt  ) {
         KillTimer( 1 );
         MessageBox("Optimization Finished");
      } else {
         char sz[256];
         m_ListTrack.GetText( m_ListCpt, sz );
         InitRace( (CString) sz ); 
      }
   }
   CDialog::OnTimer(nIDEvent);
}

void CDlgStartRaceBulle::OnAddTrack() 
{
   UpdateData( TRUE );
   m_ListTrack.AddString( m_sTrackName );
}

void CDlgStartRaceBulle::OnAddAllTrack() 
{
   char filename[256];
   m_ListTrack.ResetContent();
   strcpy( filename, "*.tr*" );
   chdir("tracks");
   DlgDirList( filename, IDC_LIST_TRACK, 0, 0 );
   chdir("..");
   m_ListTrack.DeleteString( m_ListTrack.FindString( 0, "random.trk" ) );
   m_ListTrack.SetCurSel(0);
}

void CDlgStartRaceBulle::OnDelTrack() 
{
   int i = m_ListTrack.GetCurSel();
   m_ListTrack.DeleteString( i );
   m_ListTrack.SetCurSel( i );
}


void CDlgStartRaceBulle::InitRace( CString sTrack ) 
{
   CString s;
   char argv0[20], argv1[20], argv2[20], 
        argv3[20], argv4[20], argv5[20],
        argv6[20];

   strcpy(argv0, "rars" );
   s.Format( "%d", m_iCars );  
   strcpy(argv1, s ); 
   s.Format( "%d", m_iLaps );  
   strcpy(argv2, s );
   s = sTrack; 
   strcpy(argv3, s ); 
   strcpy(argv4, m_sOptiStyle ); 
   s.Format( "-Oc%d", m_iOptiNb );  
   strcpy(argv5, s );
   s.Format( "-Ox%d", m_iOptiStart );  
   strcpy(argv6, s );
   
   g_argv[0] = argv0;
   g_argv[1] = argv1;
   g_argv[2] = argv2;
   g_argv[3] = argv3;
   g_argv[4] = "-s1";
   g_argv[5] = "-nd";
   g_argv[6] = argv4;
   g_argv[7] = argv5;
   g_argv[8] = argv6;

   g_argc = 9;

   track_count = 0;   // needed for multiple circuits
   OptiGetArgs(g_argc, g_argv);
   g_RaceManager.ArgsInit( g_argc, g_argv );
   g_RaceManager.AllInit();
   OptiInit( this );
   ml=0;
}


