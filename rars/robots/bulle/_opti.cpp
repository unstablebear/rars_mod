/**
 *  OPTI.CPP
 *  Bulle optimization file
 */

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#ifdef WIN32
  #include "stdafx.h"
  #include <conio.h>
  #include <dos.h>
  #include "vc_rars.h"
  #include "dlgbulle.h"
#else
  #include "kde_bulle_impl.h"
  #include "qmultilineedit.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "_opti.h"
#include "../../graphics/g_global.h"

//--------------------------------------------------------------------------
//                           D E F I N E S
//--------------------------------------------------------------------------

#define OPTI_STEP     4
#define OPTI_DEPTH    6

#define MAX_OPTI      300
#define MAX_SEG       90

// files of OPTI
#define FILE_NO           0
#define FILE_NORMAL       1

// types of OPTI
#define TYPE_NORMAL       1
#define TYPE_MIN_LANE     2
#define TYPE_MAX_LANE     3

//--------------------------------------------------------------------------
//                             T Y P E S
//--------------------------------------------------------------------------

typedef struct 
{
  double result;
  double opti;
}  T_min;

typedef struct 
{
  double * pVal;
  double attract;
  double step;
  const char * name;
  int file;
  int type;
} T_opti;

struct BulleParam
{
  double PointCurve;   // point for curve
  double AccelCurve;   // acceleration for curve
  double DecelCurve;   // acceleration for curve
  double SpeedMag;     // to calc. the speed in corner
  double SpeedMag2;    // to calc. the speed in corner
  double StepFeet;     // distancce between POINT
  double AlphaDist;    // distance to calculate alpha
  double MinLane;      // minimum lane
  double MaxLane;      // maximum lane
};

//--------------------------------------------------------------------------
//                           E X T E R N S  
//--------------------------------------------------------------------------

// CARZ.CPP
  extern double time_count;
  extern long race_count;
  extern long lap_count;       // length of the race in laps
  extern int out_count;
  extern int car_count;       // how many cars in the race
  extern int no_display;
  void Randomize(long input);

// BULLE2.CPP
  extern BulleParam param;
  extern double g_TimePenality;
  extern int g_OptiRunning;


// _OPTI.CPP
  void OptiGet( int n );

//--------------------------------------------------------------------------
//                           G L O B A L 
//--------------------------------------------------------------------------

// Optimization Modes
int OPTI_PARAM  = 0;
int OPTI_PARAM2 = 0;
int OPTI_FULL   = 0;
int OPTI_ALPHA  = 0;
int OPTI_BORDER = 0;
int OPTI_SPEED  = 0;
int OPTI_BASIS  = 0;

int OPTI_CYCLE       = 0;            // number of cycle
int OPTI_NO_PENALITY = 0;

int ANAL_SAVE = 0;              // save for analysis

//
int gOptiCycle = 1;
int gOptiCpt = 0;
int gOptiCptArg = 0;
int gOptiNb = 0;
int gOptiType = 0;
int gOptiFile = 0;
int gOptiMl = 0;
double gOptiRaceDone = 0;
double gOptiTimeMulti = 1;
static T_opti gTabOpti[MAX_OPTI];

double gOpti;
double gOptiMin, gOptiMax, gOptiStep, gOptiAttract;
const char * gOptiName;
T_min gMin;

long gTimeStart;

// Optimization of only one variable
double gOptiOne;

#ifdef WIN32
  CDlgStartRaceBulle * pOptiDlg;
#else
  KdeBulleImpl * pOptiDlg;
#endif

////////////////////////////////////////////////////////////////////////////

//_ time_get _______________________________________________________________
//
// rend le temps en centieme de secondes
// IN  : -
// OUT : temps (long int)
//__________________________________________________________________________

long time_get( void )
{
  clock_t cl= clock();
  return( (long)cl );
}

//__ OpenSaveFile __________________________________________________________
//
// Description : open a file for saving with the name "base"."ext"
// In : base et ext
// Out : a pointer to the file
//__________________________________________________________________________

FILE * OpenSaveFile( char *base, char *ext )
{
  char s[256];
  char filename[20];
  FILE * f;

  char * point = strchr( base, '.' );
  memset( filename, 0, 19 );
  strncpy( filename, base, point-base+1 );
  strcat( filename, ext );
  sprintf( s, "Writing \"%s\"\n", filename );

#ifdef WIN32
  pOptiDlg->m_StaticRes2.SetWindowText(s);
#else
  pOptiDlg->Line3->setText(s);
#endif

  if ((f = fopen(filename, "wt"))== NULL)
  {
    error( "OpenSaveFile : Cannot open file" );
  }
  return f;
}
////////////////////////////////////////////////////////////////////////////

void OptiMessage( int line, const char * sMessage )
{
  switch( line )
  {
  #ifdef WIN32
    case 1:
      pOptiDlg->m_StaticRes0.SetWindowText(sMessage);
      break;
    case 2:
      pOptiDlg->m_StaticRes1.SetWindowText(sMessage);
      break;
    case 3:
      pOptiDlg->m_StaticRes2.SetWindowText(sMessage);
      break;
  #else
    case 1:
      pOptiDlg->Line1->setText(sMessage);
      break;
    case 2:
      pOptiDlg->Line2->setText(sMessage);
      break;
    case 3:
      pOptiDlg->Line3->setText(sMessage);
      break;
  #endif
  }
}

////////////////////////////////////////////////////////////////////////////

void OptiAdd( double step, double attract, double * pVal, const char * name, int iOptiType )
{
  if( gOptiNb >= MAX_OPTI ) error("OptiAdd: too much optimized parameters");
  gTabOpti[gOptiNb].pVal = pVal;
  gTabOpti[gOptiNb].step = step;
  gTabOpti[gOptiNb].attract = attract;
  gTabOpti[gOptiNb].name = name;
  gTabOpti[gOptiNb].type = iOptiType;
  gTabOpti[gOptiNb].file = gOptiFile;
  gOptiNb++;
}

////////////////////////////////////////////////////////////////////////////

#ifdef WIN32
void OptiInit( CDlgStartRaceBulle * pDlg )
{
  pOptiDlg = pDlg;
#else
void OptiInit( KdeBulleImpl * pDlg )
{
#endif
  pOptiDlg = pDlg;
  gTimeStart = time_get();
  gOptiRaceDone = 0;
  gOptiMl = 0;
  gOptiCycle = 1;
  gOptiCpt = gOptiCptArg;
  gOptiNb = 0;
  gOptiType = 0;
  gOptiFile = 0;
  gOptiTimeMulti = 1;
  g_OptiRunning = 1;

  gOptiCycle = OPTI_CYCLE;

  if( OPTI_BASIS )
  {
    gOptiFile = FILE_NORMAL;

    gOptiType = TYPE_NORMAL;

/*
    OptiAdd( 0.2,   1.14,  &param.AccelCurve,   "AccelCurve"  , TYPE_NORMAL );
    OptiAdd( 0.2,   0.99,  &param.DecelCurve,   "DecelCurve"  , TYPE_NORMAL );
*/
    OptiAdd( 1.0,   1.00,  &param.PointCurve,   "PointCurve"  , TYPE_NORMAL );
    OptiAdd( 0.2,   1.02,  &param.AccelCurve,   "AccelCurve"  , TYPE_NORMAL );
    OptiAdd( 0.2,   1.02,  &param.DecelCurve,   "DecelCurve"  , TYPE_NORMAL );
    OptiAdd( 5.0,   37.0,  &param.SpeedMag,     "SpeedMag"    , TYPE_NORMAL );
    OptiAdd( 5.0,   31.0,  &param.SpeedMag2,    "SpeedMag2"   , TYPE_NORMAL );
    OptiAdd( 1.0,    9.0,  &param.StepFeet,     "StepFeet"    , TYPE_NORMAL );
    OptiAdd( 10.0,  10.0,  &param.AlphaDist,    "AlphaDist" , TYPE_NORMAL );
    OptiAdd( 0.0,   0.01,  &param.MinLane,      "MinLane"     , TYPE_MIN_LANE );
    OptiAdd( 0.0,   0.99,  &param.MaxLane,      "MaxLane"     , TYPE_MAX_LANE );
  }
  // get the first parameter
  OptiGet(gOptiCpt);
  race_count = OPTI_STEP;
}

////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////

void OptiClose()
{
  FILE * in, * out;
	
  char line[10000];
  char s[128];
  char sMode[128];
  char * pt;
  int i, bFound;
  
  if( (in=fopen( "robots/bulle2.dat", "r" )) == NULL ) 
  {
    error( "Bulle: can not find bulle2.dat" );
  }
  if( (out=fopen( "robots/bulle2.new", "w" )) == NULL ) 
  {
    error( "Bulle: can create file bulle2.new" );
  }

  track_desc track = get_track_description();

  bFound = 0; 
  while( !feof(in) && !bFound ) 
  {
    if( fgets( line, 10000, in ) >0 )
    {
 	    switch( line[0] )
      {
   	    case '[':
          memset( sMode, 0, 128 );
          pt = strchr( line+1, ']' );
          strncpy( sMode, line+1, pt-(line+1) );
          break;
        case '\"':
          if( strcmp( sMode, "NORMAL SURFACE 1" )==0 )
          {
            memset( s, 0, 128 );
          pt = strchr( line+1, '\"' );
          strncpy( s, line+1, pt-(line+1) );
          i = strcmp( track.sName, s );
          if( i==0 )
          {
            fgets( line, 10000, in );
            fgets( line, 10000, in );
            fgets( line, 10000, in );
            line[0] = 0;
          }
          if( i<=0 )
          {
            fprintf(out, "\"%s\"\n", track.sName );
            double length = track.length;
            double avg = length*lap_count/gMin.result * MPH_FPS;
            fprintf(out, "  // Time:%9f - Laps:%2ld - Avg:%9f\n", gMin.result, lap_count, avg );
            fprintf(out, "  " );

            double * p_p = &(param.PointCurve);
            for( int i=0; i<gOptiNb; i++ )
            {
              fprintf(out, " %9f", p_p[i] );
              if( i!=gOptiNb-1 ) fprintf(out, ",");
            }
            fprintf(out, "\n\n" );
			      bFound = TRUE;
          }
        }
        break;
      }
      fprintf( out, "%s", line );
    }
  }

  while( !feof(in) ) 
  {
    if( fgets( line, 10000, in ) >0 )
    {
      fprintf( out, "%s", line );
    }
  }
  fclose( in );
  fclose( out );

#ifdef WIN32
  CopyFile("robots\\bulle2.dat", "robots\\bulle2.old", false );
  CopyFile("robots\\bulle2.new", "robots\\bulle2.dat", false );
#else
  if( (in=fopen( "robots/bulle2.dat", "r" )) != NULL )
  {
    if( (out=fopen( "robots/bulle2.old", "w" )) != NULL )
    {
      while( !feof(in) )
      {
        if( fgets( line, 10000, in ) >0 )
        {
          fprintf( out, "%s", line );
        }
      }
      fclose( out );
    }
    fclose( in );
  }

  if( (in=fopen( "robots/bulle2.new", "r" )) != NULL )
  {
    if( (out=fopen( "robots/bulle2.dat", "w" )) != NULL )
    {
      while( !feof(in) )
      {
        if( fgets( line, 10000, in ) >0 )
        {
          fprintf( out, "%s", line );
        }
      }
      fclose( out );
    }
    fclose( in );
  }

#endif
}

////////////////////////////////////////////////////////////////////////////

void OptiGet( int n )
{
  gOpti = *(gTabOpti[n].pVal);
  gMin.result = 1000000;
  gOptiMin = gOpti-2.0*gTabOpti[n].step;
  gOptiMax = gOpti+2.0*gTabOpti[n].step;
  gOptiAttract = gTabOpti[n].attract;
  gOptiName = gTabOpti[n].name;
}

////////////////////////////////////////////////////////////////////////////

void OptiSet( int n )
{
  *(gTabOpti[n].pVal) = gOpti;
}

////////////////////////////////////////////////////////////////////////////

void OptiFinalSet( int n, double f )
{
  // security, not all value are allowed
  switch( gTabOpti[n].type )
  {
    case TYPE_NORMAL: break;
    case TYPE_MIN_LANE:
	    if( f<0.01 )
      {
	      f=0.01;
        OptiMessage( 3, "! min_lane < 0.01 => 0.01" );
	    }
	    break;
    case TYPE_MAX_LANE:
	    if( f>0.99 )
      {
	      f=0.99;
        OptiMessage( 3, "! max_lane > 0.99 => 0.99" );
	    }
	    break;
  }

  *(gTabOpti[n].pVal) = f;
  gTabOpti[n].step *=0.5;

  // save in the good file
  switch( gTabOpti[n].file )
  {
   case FILE_NORMAL:
	   OptiClose();
     break;
  }
}

////////////////////////////////////////////////////////////////////////////

void OptiSaveDouble( double * a, int pos, double d )
{
  static int ok=0;

  if( pos==0 && !ok )
  {
    for( int i = 0; i<MAX_SEG; i++) a[i] = 0.0;
    ok=1;
  }

  if( pos>=MAX_SEG ) error("OptiSaveDouble: too much segments");
  a[pos] = d;
}


////////////////////////////////////////////////////////////////////////////

void OptiSaveInt( int * a, int pos, int d )
{
  static int ok=0;

  if( pos==0 && !ok )
  {
    for( int i = 0; i<MAX_SEG; i++) a[i] = 0;
    ok=1;
  }

  if( pos>=MAX_SEG ) error("OptiSaveInt: too much segments");
  a[pos] = d;
}

////////////////////////////////////////////////////////////////////////////

void OptiRaceBefore( long ml )
{
  gOptiStep = (gOptiMax-gOptiMin)/OPTI_STEP;
  gOpti = gOptiMin+ml*gOptiStep;
  OptiSet(gOptiCpt);
  Randomize( 3 );
}

////////////////////////////////////////////////////////////////////////////

void OptiRaceAfter( long &ml )
{
  gOptiRaceDone++;
  char s[512], s2[512];

  if( out_count==0 )
  {
    sprintf( s, "time = %f ( penality=%f, %s=%f )", time_count, g_TimePenality, gOptiName, gOpti );
  }
  else
  {
    sprintf( s, "..crashed.. (%s=%f)", gOptiName, gOpti );
  }
  if( !OPTI_NO_PENALITY )
  {
    time_count += g_TimePenality*gOptiTimeMulti;
  }

  OptiMessage( 3, s );

  int qTest1 = fabs( gMin.opti-gOptiAttract )>fabs( gOpti-gOptiAttract );
  int qTest2 = time_count==gMin.result && qTest1;
  if( (time_count<gMin.result || qTest2 ) && out_count==0 )
  {
    gMin.result = time_count;
    gMin.opti = gOpti;
  }

  // check if the parameter influences the lap time
  static double result1;
  static int bool_result_change;
  if( ml==0 )
  {
    result1 = time_count;
    bool_result_change = 0;
  }
  else if( result1 != time_count )
  {
    bool_result_change = 1;
  }

  if( ml == race_count )
  {
    gOptiMl++;
    if( bool_result_change == 0 )
    {
      // skip the next races
	    gOptiMl = OPTI_DEPTH+1;
      gOptiRaceDone += 12;
    }

    double t = (time_get()-gTimeStart)/(60.0*CLOCKS_PER_SEC);
    double t2 = t*(17*gOptiNb*OPTI_CYCLE)/gOptiRaceDone;
    // clrscr();

    if( gOptiRaceDone==5 )
    {
      sprintf( s, "Track : %s  Lap Time: %f", get_track_description().sName, gMin.result );
      OptiMessage( 1, s );
    }

    sprintf( s, "Loop: %d  OptiCpt: %d (%d)  OptiCycle: %d \n", gOptiMl, gOptiCpt, gOptiNb, gOptiCycle);
    sprintf( s2,"Step: %3.4f  Name: %s  Value: %f\n", gOptiStep, gOptiName, gMin.opti );
    strcat( s, s2 );
    sprintf( s2, "Lap Time: %f\n",  gMin.result);
    strcat( s, s2 );
    sprintf( s2, "Race Done: %4.0f (%d)\n", gOptiRaceDone, 17*gOptiNb*OPTI_CYCLE);
    strcat( s, s2 );
    sprintf( s2, "Time: %4.2f [min]  Estimated: %4.2f \n", t, t2 );
    strcat( s, s2 );
    OptiMessage( 2, s );

    if( gOptiMl<=OPTI_DEPTH && gMin.result<10000 )
    {
	    ml = 0;
      race_count = OPTI_STEP-1;
      gOptiMin = gMin.opti-gOptiStep;
      gOptiMax = gMin.opti+gOptiStep;
    }
    else
    {
   	  if( gOptiCycle<=0 )
      {
        // program enters here :
        // - after gSaveSpeed
        // - during a OPARAM optimization
        ml = 10000; // exit
      }
      else
      {
        // gOptiCpt
        if( gMin.result<10000 ) OptiFinalSet(gOptiCpt, gMin.opti);
 	      ml =-1; gOptiMl = 0;
	      race_count = OPTI_STEP;
	      gOptiCpt++;
        if( gOptiCpt == gOptiNb )
        {
	        gOptiCycle--;
	        gOptiCpt = 0;
	        if( gOptiCycle<=0 )
          {
	          if( OPTI_PARAM )
            {
	       	 // save track.spd for a further optimzation
               race_count = 0;
            }
          }
        }
      }
	    OptiGet(gOptiCpt);
    }
  }
  // skip the mid element (3)
  if( gOptiMl>0 && ml==1 )
  {
    ml++;
  }
}

////////////////////////////////////////////////////////////////////////////

void OptiGetArgs(int argc, char* argv[])
{
  // -Osx = start at the position x (0-3)
  // -Ocx = number of cycles
  // -OPARAM, -OFULL, -OBORDER, -OSPEED, -OALPHA, -OSTART
  //   = optimization modes
  int i;
  char c, c2, option;
  char *ptr, *s;

  // set the default values
  OPTI_CYCLE = 1;

  // process each argument in turn:
  for(i=1; i<argc; i++)
  {   // once for each argument
    ptr = argv[i];
    c = *(ptr++);
    c2 = *(ptr++);
    if(c == '-' && c2 == 'O')
    {
      // an option
      s = ptr;
      option = *(ptr++);           // grab the option code
      switch(option)
      {
	      case 'c':     // s for OPTI_CYCLE
	        OPTI_CYCLE = atoi( ptr );
	        break;
        case 'x':     // s for gOptiCpt
          gOptiCptArg = atoi( ptr );
          break;
        case 't':     // s for time_penalty_multiplcator
          gOptiTimeMulti = atoi( ptr );
          break;
      }
      if( !strcmp( s, "PARAM" ) )  OPTI_PARAM = 1;
      if( !strcmp( s, "PARAM2" ) ) OPTI_PARAM2 = 1;
      if( !strcmp( s, "FULL" ) )   OPTI_FULL = 1;
      if( !strcmp( s, "ALPHA" ) )  OPTI_ALPHA = 1;
      if( !strcmp( s, "SPEED" ) )  OPTI_SPEED = 1;
      if( !strcmp( s, "BORDER" ) ) OPTI_BORDER = 1;
      if( !strcmp( s, "BASIS" ) )  OPTI_BASIS = 1;
      if( !strcmp( s, "NPENAL" ) )  OPTI_NO_PENALITY = 1;
    }
    else if(c == '-' && c2 == 'A')
    {
      // save a file for an analysis with matlab
      ANAL_SAVE = 1;
    }
  }
  // The full optimization optimizes the speed and alpha for each corner
  if( OPTI_FULL==1 )
  {
    OPTI_ALPHA = 1;
    OPTI_SPEED = 1;
    OPTI_BORDER = 1;
  }
}
