/*
 * KDE_BULLE.H -
 *
 * History
 *  ver. 0.76 Oct 00 - CCDOC
 *
 * @author    Marc Gueury <mgueury@synet.be>
 * @see:      C++ Coding Standard and CCDOC in help.htm
 * @version   0.76
 */

#ifndef KDE_BULLE_IMPL_H
#define KDE_BULLE_IMPL_H

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include <kapp.h>
#include "kde_bulle.h"

//--------------------------------------------------------------------------
//                             T Y P E S
//--------------------------------------------------------------------------

class KdeBulleImpl : public KdeBulle
{
  Q_OBJECT

public:
  KdeBulleImpl( KApplication * app );

  QTimer * m_timer;
  int      m_ListTrackNb;
  int      m_ListTrackCpt;

  void InitRace( const char * sTrack );

public slots:
  virtual void slotTimeOut();
  virtual void slotStartRace();
  virtual void slotTrackAdd();
  virtual void slotTrackDel();
  virtual void slotTrackAddAll();
};

#endif
