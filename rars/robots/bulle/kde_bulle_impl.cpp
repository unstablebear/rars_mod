/*
 * KDE_START.CPP - Start Window of BULLE in KDE 3
 *
 * History
 *  ver. 0.76 Oct 00 - CCDOC
 *  ver. 0.90 Feb 03 - KDE3
 *
 * @author    Marc Gueury <mgueury@synet.be>
 * @see:      C++ Coding Standard and CCDOC in help.htm
 * @version   0.76
 */

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include <qcombobox.h>
#include <qdir.h>
#include <qfileinfo.h>
#include <qlined.h>
#include <qlistbox.h>
#include <qmessagebox.h>
#include <qmultilineedit.h>
#include <qradiobutton.h>
#include <qspinbox.h>
#include <qtimer.h>

#include "../../race_manager.h"
#include "kde_bulle.h"
#include "kde_bulle_impl.h"
#include "../../graphics/g_global.h"
#include "_opti.h"

//--------------------------------------------------------------------------
//                           E X T E R N S
//--------------------------------------------------------------------------

extern long race_count;
extern RaceManager g_RaceManager;
extern int gSaveSpeed;
extern int track_count;
extern long ml;

//--------------------------------------------------------------------------
//                           F U N C T I O N S
//--------------------------------------------------------------------------

static void FillComboWithDir( QComboBox * combo, const char * directory, const char * ext )
{
  QDir dir(directory);
  dir.setFilter( QDir::Files | QDir::NoSymLinks );
  const QFileInfoList * fileinfolist = dir.entryInfoList();
  QFileInfoListIterator it (*fileinfolist);
  QFileInfo *fi;
  while( (fi=it.current()) )
  {
    if( strcmp( fi->extension(), ext )==0 )
    {
       combo->insertItem( fi->baseName() );
    }
    ++it;
  }
}

//--------------------------------------------------------------------------
//                            Class KdeBulleImpl
//--------------------------------------------------------------------------

KdeBulleImpl::KdeBulleImpl( KApplication * app )
: KdeBulle()
{
  QObject::connect( (QObject *)PushStart, SIGNAL(clicked()), (QObject *)this, SLOT(slotStartRace()) );
  QObject::connect( (QObject *)PushExit, SIGNAL(clicked()), (QObject *)app, SLOT(quit()) );

  QObject::connect( (QObject *)PushAdd, SIGNAL(clicked()), (QObject *)this, SLOT(slotTrackAdd()) );
  QObject::connect( (QObject *)PushDel, SIGNAL(clicked()), (QObject *)this, SLOT(slotTrackDel()) );
  QObject::connect( (QObject *)PushAddAll, SIGNAL(clicked()), (QObject *)this, SLOT(slotTrackAddAll()) );

  FillComboWithDir( ComboTrack, "tracks", "trk" );

  ComboMode->insertItem("-OBASIS");

  ListTrack->clear();
}

void KdeBulleImpl::slotStartRace()
{
  const char * sTrackName = ComboTrack->currentText();
  m_ListTrackNb = ListTrack->count();

  if( m_ListTrackNb==0 )
  {
     InitRace( sTrackName );
  }
  else
  {
    m_ListTrackCpt = 0;
    const char * sz = ListTrack->text( m_ListTrackCpt );
    InitRace( sz );
  }
  // create and setup a timer
  m_timer = new QTimer( this, "Timer" );
  connect( m_timer, SIGNAL( timeout() ), this, SLOT( slotTimeOut() ) );
  m_timer->start( 50, false );
}

void KdeBulleImpl::slotTrackAdd()
{
  const char * sTrackName = ComboTrack->currentText();
  ListTrack->insertItem( sTrackName );
}

void KdeBulleImpl::slotTrackDel()
{
  ListTrack->removeItem( ListTrack->currentItem() );
}

void KdeBulleImpl::slotTrackAddAll()
{
  ListTrack->clear();
  int nb = ComboTrack->count();
  for( int i=0; i<nb; i++ )
  {
    ListTrack->insertItem( ComboTrack->text(i) );
  }
}

void KdeBulleImpl::slotTimeOut()
{
  OptiRaceBefore( ml );
  g_RaceManager.AllInit();
  g_RaceManager.RaceInit(ml);
  while( g_RaceManager.RaceLoop() );
  g_RaceManager.RaceClose(ml);
  // g_RaceManager.AllClose();
  OptiRaceAfter( ml );

  ml++;
  if( ml>race_count )
  {
    g_RaceManager.AllClose();
    m_ListTrackCpt++;
    if( m_ListTrackNb==0 || m_ListTrackNb<=m_ListTrackCpt  )
    {
      m_timer->stop();
      QMessageBox::information( 0, "Rars", "Optimization Finished");
    } else {
      const char * sz = ListTrack->text( m_ListTrackCpt );
      InitRace( sz );
    }
  }
}

void KdeBulleImpl::InitRace( const char * sTrack )
{
  char s[256];
  char argv0[20], argv1[20], argv2[20],
       argv3[20], argv4[20], argv5[20],
       argv6[20];

  int iCars = SpinCars->value();
  int iLaps = SpinLaps->value();
  const char * sOptiMode = ComboMode->currentText();
  int iOptiCycle = SpinCycle->value();
  int iOptiStart = SpinStart->value();

  strcpy(argv0, "rars" );
  sprintf( s, "%d", iCars );
  strcpy(argv1, s );
  sprintf( s, "%d", iLaps );
  strcpy(argv2, s );
  strcpy(argv3, sTrack );
  strcpy(argv4, sOptiMode );
  sprintf( s, "-Oc%d", iOptiCycle );
  strcpy(argv5, s );
  sprintf( s, "-Ox%d", iOptiStart );
  strcpy(argv6, s );

  g_argv[0] = argv0;
  g_argv[1] = argv1;
  g_argv[2] = argv2;
  g_argv[3] = argv3;
  g_argv[4] = (char *)"-s1";
  g_argv[5] = (char *)"-nd";
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

