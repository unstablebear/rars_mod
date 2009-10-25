//--------------------------------------------------------------------------
//
//    FILE: _OPTI.H (portable)
//
//    This file contains the optimization function declaration
//
//    Version       Author          Date
//      0.1      Marc Gueury       21/4/99
//
//--------------------------------------------------------------------------


#ifndef _opti_H
#define _opti_H

//--------------------------------------------------------------------------
//                           E X T E R N S
//--------------------------------------------------------------------------

extern double gOpti;
extern double gOptiOne;
extern int OPTI_PARAM2;
extern int gOptiCycle;

//--------------------------------------------------------------------------
//                          F O R W A R D S
//--------------------------------------------------------------------------

#ifdef WIN32
  class CDlgStartRaceBulle;
#endif

//--------------------------------------------------------------------------
//                         F U N C T I O N S
//--------------------------------------------------------------------------

#ifdef WIN32
  void OptiInit( CDlgStartRaceBulle * pDlg );
#else
  void OptiInit( KdeBulleImpl * pDlg );
#endif
void OptiRaceBefore( long ml );
void OptiRaceAfter( long &ml );
void OptiGetArgs(int argc, char* argv[]);
void OptiSaveDouble( double * a, int pos, double d );
void OptiSaveInt( int * a, int pos, int d );

#endif //_opti_H
