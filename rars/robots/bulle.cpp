//__________________________________________________________________________
//
// BULLE.CPP - A robot "driver" for RARS ver. 0.73
//
// Filename:  bulle.cpp
// Robot:     bulle
// Author:    Marc Gueury, marc.gueury@skynet.be (Belgium)
// Races:     all
// Source:    Public
// Data:      -
// Color:     Green/Green
//
//__________________________________________________________________________


//////////////////////////  I N C L U D E S  ///////////////////////////////

#include <string.h>                      // strcpy
#include <math.h>                        // fabs, sqrt
#include <stdio.h>
#include <stdlib.h>

#include "car.h"                         // stuck
#include "track.h"
#include "os.h"
#include "misc.h"                        // alpha_limit, collide
#include "draw.h"                        // alpha_limit, collide

//#define BULLE_HOME
#ifdef BULLE_HOME
   #include "_bulle.h"
   #include "_opti.h"
#endif // BULLE_HOME

// paste here bulle.h
#ifndef BULLE_HOME

/////////////////////////// B U L L E . H /////////////////////////////////

#define STRAIGHT_RAD         100000.0   // radius for straigths
#define STRAIGHT_SPD           400.00   // speed for straights
#define MAX_SEG                    90   // max segments (gTrack)
#define MAX_CURVE                  50   // max curves (gCurve)
#define TPARAM_NB                  36   // number of parameters

// curve types
#define CUR_ST 0
#define CUR_LC 1
#define CUR_RC 2

////////////////////////////  M A C R O S  /////////////////////////////////

#ifndef max 
  #define max(x,y)                (((x)<(y))?(y):(x))        // return maximum
  #define min(x,y)                (((x)>(y))?(y):(x))        // return minimum
#endif
#define sqr(x)                  ((x)*(x))                  // return x}
#define my_mod(x,y)             (((x)<(y))?(x):(x-y))      // return absolute

////////////////////////////// T Y P E S //////////////////////////////////

typedef struct {
  int alpha;
  int speed;
} tControl;

typedef struct {
 // one for each half segment
      double r1;           // internal radius
      double r2;       // radius ST_CV_ST
      double d1;       //
      double d3;       //
      double speed;        // speed
      tControl control;    // type of controle
} T_track_sub;

typedef struct {
   // ST_CV_ST
   double length;       // length
   T_track_sub t[2];

   // type
   int type;
   int curve;           // curve number
} T_track ;

typedef struct {
   double in;
   double out;
   double out2;
} T_border;

typedef struct {
    char   trackfile[16];           // track file name

    double DECEL;                   // braking deceleration
    double START_ACC;               // fraction of seg. len. to start acc.
    double START_ACC2;              // fraction of seg. len. to start acc.
    double BORDER_IN;               // internal border
    double BORDER_OUT;              // external border
    double BORDER_OUT2;             // external border
    double SPEED_MAG;               // coef. for speed in normal curve
    double SPEED_MAG_MIN;           // coef. for speed in slow curve

    double ALPHA_START;             // alpha=0 for xxx dt
    double ALPHA_MAG_INI;           // bAlpha, AM initial
    double SLIDE_MAG_INI;           // bAlpha, SM initial
    double STRAIGTH_SHORT2;         // bGetCurveType qLongStraight2
    double STRAIGTH_SHORT3;         // bGetCurveType qLongStraight3
    double ALPHA_MAG_S_OUTSIDE;     // bAlpha, s.outside
    double SLIDE_MAG_S_OUTSIDE;     // bAlpha, s.outside
    double SLIDE_MAG_S_NXT;         // bAlpha, s.nxt
    double ALPHA_MAG_S_PRV;         // bAlpha, s.prv
    double SLIDE_MAG_S_PRV;         // bAlpha, s.prv
    double ALPHA_MAG_S_MIX;         // bAlpha, s.mix
    double ALPHA_MAG_S_MIX2;        // bAlpha, s.mix2
    double SLIDE_MAG_S_MIX2;        // bAlpha, s.mix2
    double ALPHA_MAG_S_MIX2b;       // bAlpha, s.mix2
    double SLIDE_MAG_S_MIX2b;       // bAlpha, s.mix2
    double ALPHA_MAG_S_MIX3;        // bAlpha, s.mix3
    double SLIDE_MAG_S_MIX3;        // bAlpha, s.mix3
    double ALPHA_MAG_S_MIX4;        // bAlpha, s.mix4
    double SLIDE_MAG_S_MIX4;        // bAlpha, s.mix4
    double ALPHA_MAG_C_INSIDE;      // bAlpha, c.inside
    double SLIDE_MAG_C_INSIDE;      // bAlpha, c.inside
    double ALPHA_MAG_C_1H_OUTSIDE;  // bAlpha, c.1H outside
    double SLIDE_MAG_C_1H_OUTSIDE;  // bAlpha, c.1H outside
    double ALPHA_MAG_C_2H_OUTSIDE;  // bAlpha, c.2H outside
    double SLIDE_MAG_C_2H_OUTSIDE;  // bAlpha, c.2H outside
    double ALPHA_MAG_C_1H_IN_S;     // bAlpha, c.1H in S
    double SLIDE_MAG_C_1H_IN_S;     // bAlpha, c.1H in S
    double ALPHA_MAG_C_2H_IN_S;     // bAlpha, c.2H in S
} T_param;

typedef struct {
   double s1;       // speed segment 1
   double s2;       // speed segment 2
} T_speed_sub;

typedef struct {
    char   trackfile[16];           // track file name
    // speed
    T_speed_sub speed[ MAX_SEG ];
    // alpha_mag, slide_mag
    int    AlphaNb;
    T_speed_sub alpha[ MAX_SEG ];
    // border_in, border_out
    double border_in[ MAX_SEG ];
    int alpha_cpt[ MAX_SEG ];
} T_speed ;

/////////////////////  E N D   O F   B U L L E . H  ///////////////////////
#endif // BULLE_HOME
///////////////////////////  E X T E R N S  ////////////////////////////////

// BULLE.CPP
   double bSpeed2( int seg, int half, tControl &c );

//////////////////////////////// D A T A //////////////////////////////////

static T_param gDefaultParam =
{
  "default", // name
   35.0,      // DECEL
   0.97,      // START_ACC
   0.35,      // START_ACC2
   1,         // BORDER_IN
   13,        // BORDER_OUT
   13,        // BORDER_OUT2
   5.0,       // SPEED_MAG
   6.0,       // SPEED_MAG_MIN
   50.0,      // ALPHA_START
   0.03,        // ALPHA_MAG_INI
   2.0,     // SLIDE_MAG_INI
   200.0,     // STRAIGTH_SHORT2
   0.30,      // STRAIGTH_SHORT3
   0.01,      // ALPHA_MAG_S_OUTSIDE
   1.960938,  // SLIDE_MAG_S_OUTSIDE
   2.00,      // SLIDE_MAG_S_NXT
   0.01,      // ALPHA_MAG_S_PRV
   2.0,       // SLIDE_MAG_S_PRV
   0.01,      // ALPHA_MAG_S_MIX
   0.0475,    // ALPHA_MAG_S_MIX2
   2.1250,    // SLIDE_MAG_S_MIX2
   0.0125,    // ALPHA_MAG_S_MIX2b
   2.0000,    // SLIDE_MAG_S_MIX2b
   0.0220,    // ALPHA_MAG_S_MIX3
   1.03125,   // SLIDE_MAG_S_MIX3
   0.03,      // ALPHA_MAG_S_MIX4
   2.0,       // SLIDE_MAG_S_MIX4
   0.02150,   // ALPHA_MAG_C_INSIDE
   3.851562,  // SLIDE_MAG_C_INSIDE
   0.024219,  // ALPHA_MAG_C_1H_OUTSIDE
   1.906250,  // SLIDE_MAG_C_1H_OUTSIDE
   0.008750,  // ALPHA_MAG_C_2H_OUTSIDE
   2.100,     // SLIDE_MAG_C_2H_OUTSIDE
   0.020,     // ALPHA_MAG_C_1H_IN_S
   2.00,      // SLIDE_MAG_C_1H_IN_S
   0.030      // ALPHA_MAG_C_2H_IN_S
};

//////////////////  S U P P O R T   F U N C T I O N S  /////////////////////

////////////////////////////  G L O B A L  /////////////////////////////////

track_desc t;

int     gCurSeg, gNxtSeg, gAftSeg, gPrvSeg;    // segment number
double  gWidth;            // road width
long    gCpt;              //
double  gCurLen, gNxtLen, gAftLen;
double  gCurLane, gNxtLane, gAftLane, gPrvLane;
double  gToEnd;            // distance to the end of the segment
double  gToEndAngle;       // distance to the end of the segment
double  gToLft;            // distance to the left of the road
double  gToRgt;            // distance to the rigth of the road
double  gCurAngle, gCurRadius;
int     gHalf;             // half of the segment (0,1)

double  gVn, gVt;          // speed of the car ./. car
double  gVmag;             // magnitude of the speed

double gBorderOutAdj;      //
double gBorderOutAdj2;     //
double gStartOut;
double gStartOut2;

T_track gTrack[MAX_SEG];   // precalculated data
T_border gBorder[MAX_SEG]; // precalculated borders

double time_penality;      // optimization (penalty if too close from border)
int gUseOpti;              // use the optimized speed for each corner
int param_init = 0;

static tControl c;
int gAlphaOld;
int gAlphaCpt;
int gAlpha[MAX_SEG];       // alpha_cpt for each begin of seg

int gStartPos;             // position 0-3
int gStartDist;            // distance 0-1
double gStartY;            // s.to_rgt (gCpt==0)

T_speed p2;
T_param p;

int gSaveSpeed = 0;        // save track.spd

int gCurveNb;              // number of curves

///////////////////////////  D E F I N E S  ////////////////////////////////

#define _DEBUG_              1
#define SOFT_S_PRV           0.501172   // bAlpha, s.prv
#define BORDER(i)            gBorder[gTrack[i].curve]

///////////////////////////  E X T E R N S  ////////////////////////////////

// bulle.cpp
extern double gWidth;
extern T_track gTrack[MAX_SEG];   // precalculated data
extern T_border gBorder[MAX_SEG]; // precalculated borders
extern int gUseOpti;              // use the optimized speed for each corner
extern T_param p;
extern T_speed p2;

///////////////////////////////////////////////////////////////////////////

void bTrackInit()
{
  t = get_track_description();  // returns a track_desc
}

///////////////////////////////////////////////////////////////////////////

double bTrackGetRay( int seg )
{
  //// take the smallest radius
  if(t.trackin[seg].radius<0.0 ) return( t.trackout[seg].radius );
                            else return( t.trackin[seg].radius );
}

///////////////////////////////////////////////////////////////////////////

double bTrackGetLength( int seg )
{
  double rad = bTrackGetRay( seg );
  double len = t.trackout[seg].length;
  if (rad != 0.0) return ((fabs (rad) + 0.5 * gWidth) * len);
  return (len);
}

///////////////////////////////////////////////////////////////////////////

double bTrackGetAngle( int seg )
{
  if( t.trackin[seg].radius == 0.0 ) return 0.0;
                                else return t.trackout[seg].length;
}

///////////////////////////////////////////////////////////////////////////

void bTrackCalc( T_track_sub &tsub, int seg, double in, double out )
{
  //// calc the largest radius contained in the road
  //// (see calculations)
  double r1, r2, d1, d3, w, a;

  r1 = bTrackGetRay( seg );

  if( r1 == 0.0 ) 
  {
    d1 = d3 = r2 = STRAIGHT_RAD;
  } else {
    if( r1<0.0 ) r1=-r1;
    r1 += in;
    w = t.width - in - out;

    // ST_CV_ST
    a = t.trackout[seg].length;
    d1 = w /( 1 - cos(a/2) );
    d3 = sin(a/2) * d1;
    r2 = r1 + d1;
  }

  tsub.r1 = r1;
  tsub.r2 = r2;
  tsub.d1 = d1;
  tsub.d3 = d3;
}

///////////////////////////////////////////////////////////////////////////

int bTrackGetParam( char * trackname, T_param &param )
{
  FILE * f;

  char line[10000];
  char s[128];
  char sMode[128];
  char * p;
  int bFound;
  
  if( (f=fopen( "robots/bulle.dat", "r" )) == NULL ) 
  {
    if( (f=fopen( "bulle.dat", "r" )) == NULL ) 
    {
      printf( "Bulle: can not find bulle.dat" );
      exit(0);
    }
  }
  
  bFound = 0; 
  while( !feof(f) && !bFound ) 
  {
    if( fgets( line, 10000, f ) >0 )
    {
      switch( line[0] ) 
      {
        case '[':
          memset( sMode, 0, 128 );
          p = strchr( line+1, ']' );
          strncpy( sMode, line+1, p-(line+1) );  
          break;
        case '\"':
          if( strcmp( sMode, "NORMAL SURFACE 1" )==0 ) 
          {
            memset( s, 0, 128 );
            p = strchr( line+1, '\"' );
            strncpy( s, line+1, p-(line+1) );  
            if( strcmp( trackname, s )==0 ) 
            {
              fgets( line, 10000, f ); //comments
              fgets( line, 10000, f ); //data
              sscanf( line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                  &(param.DECEL),
                  &(param.START_ACC),
                  &(param.START_ACC2),
                  &(param.BORDER_IN),

                  &(param.BORDER_OUT),
                  &(param.BORDER_OUT2),
                  &(param.SPEED_MAG),
                  &(param.SPEED_MAG_MIN),
                  &(param.ALPHA_START),
                  &(param.ALPHA_MAG_INI),
                  &(param.SLIDE_MAG_INI),
                  &(param.STRAIGTH_SHORT2),
                  &(param.STRAIGTH_SHORT3),
                  &(param.ALPHA_MAG_S_OUTSIDE),
                  &(param.SLIDE_MAG_S_OUTSIDE),
                  &(param.SLIDE_MAG_S_NXT),
                  &(param.ALPHA_MAG_S_PRV),
                  &(param.SLIDE_MAG_S_PRV),
                  &(param.ALPHA_MAG_S_MIX),
                  &(param.ALPHA_MAG_S_MIX2),

                  &(param.SLIDE_MAG_S_MIX2),
                  &(param.ALPHA_MAG_S_MIX2b),
                  &(param.SLIDE_MAG_S_MIX2b),
                  &(param.ALPHA_MAG_S_MIX3),
                  &(param.SLIDE_MAG_S_MIX3),
                  &(param.ALPHA_MAG_S_MIX4),
                  &(param.SLIDE_MAG_S_MIX4),
                  &(param.ALPHA_MAG_C_INSIDE),
                  &(param.SLIDE_MAG_C_INSIDE),
                  &(param.ALPHA_MAG_C_1H_OUTSIDE),
                  &(param.SLIDE_MAG_C_1H_OUTSIDE),
                  &(param.ALPHA_MAG_C_2H_OUTSIDE),
                  &(param.SLIDE_MAG_C_2H_OUTSIDE),
                  &(param.ALPHA_MAG_C_1H_IN_S),
                  &(param.SLIDE_MAG_C_1H_IN_S),
                  &(param.ALPHA_MAG_C_2H_IN_S)
                );
                bFound = 1;
            }
          } 
          break;
      } 

    }   
  }
  fclose( f );
  return bFound;
}

///////////////////////////////////////////////////////////////////////////

int bTrackGetSpeed( char * trackname, T_param &param, T_speed &speed )
{
    FILE * f;

    char line[10000];
    char s[128];
    char sMode[128];
    char * p;
    int bFound, i;
  
    if( (f=fopen( "robots/bulle.dat", "r" )) == NULL ) 
    {
        if( (f=fopen( "bulle.dat", "r" )) == NULL ) 
        {
            printf( "Bulle: can not find bulle.dat" );
            exit(0);
        }
    }
  
    bFound = 0; 
    while( !feof(f) && !bFound ) {
    if( fgets( line, 10000, f ) >0 )
    {
        switch( line[0] ) 
        {
            case '[':
              memset( sMode, 0, 128 );
              p = strchr( line+1, ']' );
              strncpy( sMode, line+1, p-(line+1) );  
              break;
            case '\"':
              if( strcmp( sMode, "FULL SURFACE 1" )==0 ) 
              {
                memset( s, 0, 128 );
                p = strchr( line+1, '\"' );
                strncpy( s, line+1, p-(line+1) );  
                if( strcmp( trackname, s )==0 ) 
                {
                  fgets( line, 10000, f ); //comments
                  fgets( line, 10000, f ); //data
                  sscanf( line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                          &(param.DECEL),
                          &(param.START_ACC),
                          &(param.START_ACC2),
                          &(param.BORDER_IN),
                          &(param.BORDER_OUT),
                          &(param.BORDER_OUT2),
                          &(param.SPEED_MAG),
                          &(param.SPEED_MAG_MIN),
                          &(param.ALPHA_START),
                          &(param.ALPHA_MAG_INI),
                          &(param.SLIDE_MAG_INI),
                          &(param.STRAIGTH_SHORT2),
                          &(param.STRAIGTH_SHORT3),
                          &(param.ALPHA_MAG_S_OUTSIDE),
                          &(param.SLIDE_MAG_S_OUTSIDE),
                          &(param.SLIDE_MAG_S_NXT),
                          &(param.ALPHA_MAG_S_PRV),
                          &(param.SLIDE_MAG_S_PRV),
                          &(param.ALPHA_MAG_S_MIX),
                          &(param.ALPHA_MAG_S_MIX2),
                          &(param.SLIDE_MAG_S_MIX2),
                          &(param.ALPHA_MAG_S_MIX2b),
                          &(param.SLIDE_MAG_S_MIX2b),
                          &(param.ALPHA_MAG_S_MIX3),
                          &(param.SLIDE_MAG_S_MIX3),
                          &(param.ALPHA_MAG_S_MIX4),
                          &(param.SLIDE_MAG_S_MIX4),
                          &(param.ALPHA_MAG_C_INSIDE),
                          &(param.SLIDE_MAG_C_INSIDE),
                          &(param.ALPHA_MAG_C_1H_OUTSIDE),
                          &(param.SLIDE_MAG_C_1H_OUTSIDE),
                          &(param.ALPHA_MAG_C_2H_OUTSIDE),
                          &(param.SLIDE_MAG_C_2H_OUTSIDE),
                          &(param.ALPHA_MAG_C_1H_IN_S),
                          &(param.SLIDE_MAG_C_1H_IN_S),
                          &(param.ALPHA_MAG_C_2H_IN_S)
                  );
                  
                  fgets( line, 10000, f ); // speed
                  p = line;
                  for( i=0; i<t.NSEG; i++ ) {
                    p = strchr( p, '{' ) + 1;  
                    sscanf( p, "%lf,%lf", &speed.speed[i].s1, &speed.speed[i].s2 );
                  }

                  fgets( line, 10000, f ); // alpha
                  sscanf( line, "%d", &speed.AlphaNb );
                  p = line;
                  for( i=0; i<speed.AlphaNb; i++ ) {
                    p = strchr( p, '{' ) + 1;  
                    sscanf( p, "%lf,%lf",&speed.alpha[i].s1, &speed.alpha[i].s2 );
                  }

                  fgets( line, 10000, f ); // border_in
                  p = line;
                  for( i=0; i<gCurveNb; i++ ) {
                    sscanf( p, "%lf",&speed.border_in[i] );
                    p = strchr( p, ',' ) + 1;  
                  }

                  fgets( line, 10000, f ); // border_in
                  p = line;
                  for( i=0; i<t.NSEG; i++ ) {
                    sscanf( p, "%d",&speed.alpha_cpt[i] );
                    p = strchr( p, ',' ) + 1;  
                  }

                  bFound = 1;
                }
              }
              break;

              }

        }  
    }
    fclose( f );
    return bFound;
}

///////////////////////////////////////////////////////////////////////////
double bGetOutDist( situation&s )
{
   if( s.cur_rad>0.0 ) return s.to_lft;
          else return s.to_rgt;
}

///////////////////////////////////////////////////////////////////////////

void bTrackCalcBorder( int seg )
{
   double in, out, out2;
   int curve = gTrack[seg].curve;

   // gTrack is already partially initializated
   in = p.BORDER_IN;
   out = p.BORDER_OUT;
   out2 = p.BORDER_OUT2;

   if( gUseOpti ) in = p2.border_in[curve];

   gBorder[curve].in   = in;
   gBorder[curve].out  = out;
   gBorder[curve].out2 = out2;
}

///////////////////////////////////////////////////////////////////////////

static void error( const char * txt )
{
   fprintf(stderr, txt );
   exit( 1 );
}

///////////////////////////////////////////////////////////////////////////

void bInit( int param_init )
{
  #define TRAD( a ) t.trackout[ a ].radius
  #define TLEN( a ) t.trackout[ a ].length

  int i, type;

  bTrackInit();
  if( t.NSEG > MAX_SEG ) error("<<Bulle>> bInit : too much segments");

  gWidth = t.width;
  gCurveNb = 0;
   
  for( i=0; i<t.NSEG; i++ ) 
  {
    gTrack[i].curve = gCurveNb;
    gTrack[i].length = bTrackGetLength( i );

    // curve type
    if( TRAD( i ) == 0.0 ) 
    {
      type = CUR_ST;    // straight
    } else {
      gCurveNb++;
      if( TRAD( i ) > 0.0 ) type = CUR_LC;  // left curve
                       else type = CUR_RC;  // right curve
    }
    gTrack[i].type = type;
  }

  if( draw.m_bDisplay || param_init==0 )
  {
     // init only one time during optimalization
     param_init = 1;
     gUseOpti = 0;
     if( bTrackGetSpeed( t.sName, p, p2 ) )
     { 
       gUseOpti = 1;
     } else {
       if( !bTrackGetParam( t.sName, p ) )
       {
         p = gDefaultParam;
       }
     }
  }

////
//// calcule r2 pour chaque tournant du circuit
////
  for( i=0; i<t.NSEG; i++ ) 
  {
    bTrackCalcBorder( i );
    bTrackCalc( gTrack[i].t[0], i, BORDER(i).in, BORDER(i).out);
    bTrackCalc( gTrack[i].t[1], i, BORDER(i).in, BORDER(i).out2);
  }

////
//// calc the speed for each half segment
////
  for( i=0; i<t.NSEG; i++ ) 
  {
    gTrack[i].t[0].speed = bSpeed2( i, 0, gTrack[i].t[0].control );
    gTrack[i].t[1].speed = bSpeed2( i, 1, gTrack[i].t[1].control );
  }

  gBorderOutAdj = 0;
}

////////////////////////////////////////////////////////////////////////////

static int bGetCurveType( int seg, double pos, tControl &c )
{
////
//// define
////
  // type
  #define STRAIGHT_LONG_U     0
  #define STRAIGHT_LONG_S     1
  #define STRAIGHT_SHORT_U    2
  #define STRAIGHT_SHORT_S    3
  #define STRAIGHT_VSHORT_S   4
  #define CURVE_1H_INSIDE     5
  #define CURVE_2H_INSIDE     6
  #define CURVE_1H_OUTSIDE    7
  #define CURVE_2H_OUTSIDE    8
  #define CURVE_1H_IN_S       9
  #define CURVE_2H_IN_S      10

  // type2
  #define STRAIGHT_INSIDE     0
  #define STRAIGHT_OUTSIDE    1
  #define STRAIGHT_OUT_IN     2
  #define STRAIGHT_NXT        3
  #define STRAIGHT_PRV        4
  #define STRAIGHT_MIX        5
  #define STRAIGHT_MIX3       6
  #define STRAIGHT_MIX4       7

  // tControl.alpha
  #define CALPHA_S_INSIDE     0
  #define CALPHA_S_OUTSIDE    1
  #define CALPHA_S_PRV        2
  #define CALPHA_S_NXT        3
  #define CALPHA_S_MIX        4
  #define CALPHA_C_INSIDE     5
  #define CALPHA_C_1H_OUTSIDE 6
  #define CALPHA_C_2H_OUTSIDE 7
  #define CALPHA_C_1H_IN_S    8
  #define CALPHA_C_2H_IN_S    9
  #define CALPHA_S_MIX2       10
  #define CALPHA_S_MIX3       11
  #define CALPHA_S_MIX4       12
  #define CALPHA_S_OUT_IN     13

  // tControl.speed
  #define CSPEED_S_MAX        0
  #define CSPEED_S_NXT        1
  #define CSPEED_S_PRV        2
  #define CSPEED_C_INSIDE     3
  #define CSPEED_C_OUTSIDE    4

//// variables
  int type;  // return
  int type2 = -1;
  int prv_seg = my_mod(seg+t.NSEG-1,t.NSEG);
  int nxt_seg = my_mod(seg+1,t.NSEG);
  int cur_seg_type = gTrack[seg].type;
  int nxt_seg_type = gTrack[nxt_seg].type;
  int prv_seg_type = gTrack[prv_seg].type;
  double prv_d3 = gTrack[prv_seg].t[1].d3;
  double nxt_d3 = gTrack[nxt_seg].t[0].d3;
  double cur_length = gTrack[seg].length;
////
//// qXxx = questions
////
  // 0: prv_d3<pos< nxt_d3  1: pos<prv_d3  2: nxt_d3<pos
  int auxStraightPos1 = cur_length-pos < prv_d3;
  int auxStraightPos2 = pos < nxt_d3;
  int qStraightPos = auxStraightPos1 + auxStraightPos2*2;
  // LongStraight
  int qLongStraight = prv_d3+nxt_d3 < cur_length;
  // for qLongStraight2, qLongStraight3
  double pd3=min(prv_d3,500);
  double nd3=min(nxt_d3,500);
  int qLongStraight2 = pd3+nd3+p.STRAIGTH_SHORT2 < cur_length;
  int qLongStraight3 = (pd3*p.STRAIGTH_SHORT3<cur_length) &&(nd3*p.STRAIGTH_SHORT3<cur_length) ;
  int qCurveFirstHalf = pos > cur_length*0.5;
  int qNxtStraightType = (nxt_seg_type==CUR_ST)?bGetCurveType(nxt_seg,0,c):0;
  int qPrvStraightType = (prv_seg_type==CUR_ST)?bGetCurveType(prv_seg,0,c):0;

////
//// find the type of the curve
////
  if( cur_seg_type == CUR_ST ) 
  {
    // straigth
    if( nxt_seg_type == prv_seg_type ) 
    {
      // LC_ST_LC -- RC_ST_RC
      if( qLongStraight ) 
      {
        type = STRAIGHT_LONG_U;
        switch( qStraightPos ) 
        {
          case 0: type2 = STRAIGHT_OUTSIDE; break;
          case 1: type2 = STRAIGHT_PRV; break;
          case 2: type2 = STRAIGHT_NXT; break;
          case 3: type2 = STRAIGHT_NXT; break;        // never
        }
      } else {
        type = STRAIGHT_SHORT_U;
        type2 = STRAIGHT_INSIDE;
      }
    } else {
      // it's a S
      if( qLongStraight2 ) 
      {
        type = STRAIGHT_LONG_S;
        type2 = STRAIGHT_MIX;

      } else if( qLongStraight3 ) {
        type = STRAIGHT_SHORT_S;
        switch( qStraightPos ) 
        {
          case 0: type2 = STRAIGHT_OUT_IN; break;
          case 1: type2 = STRAIGHT_PRV; break;
          case 2: type2 = STRAIGHT_OUT_IN; break;
          case 3: type2 = STRAIGHT_PRV; break;
        }
      } else {
        type = STRAIGHT_VSHORT_S;
        switch( qStraightPos ) 
        {
          case 0: type2 = STRAIGHT_OUT_IN; break; // never
          case 1: type2 = STRAIGHT_PRV; break;
          case 2: type2 = STRAIGHT_NXT; break;
          case 3: type2 = STRAIGHT_MIX4; break;
        }
      }
    }
  } else {
    if( qCurveFirstHalf ) 
    {
      // first half of the curve
      if( prv_seg_type==CUR_ST ) 
      {
        if( qPrvStraightType != STRAIGHT_SHORT_S ) 
        {
          // stay outside the corner
          type = CURVE_1H_OUTSIDE;
        } else {
          // stay inside the corner
          type = CURVE_1H_INSIDE;
        }
      } else if( cur_seg_type == prv_seg_type ) {
        type = CURVE_1H_INSIDE;
      } else {
        type = CURVE_1H_IN_S;
      }
    } else {
      // second half of the curve
      if( nxt_seg_type==CUR_ST ) 
      {
        if( qNxtStraightType == STRAIGHT_LONG_S
        || qNxtStraightType == STRAIGHT_SHORT_U
        || qNxtStraightType == STRAIGHT_LONG_U ) 
        {
           // stay outside the corner
           type = CURVE_2H_OUTSIDE;
        } else {
           // stay inside the corner
        type = CURVE_2H_OUTSIDE;
        }
      } else if( cur_seg_type == nxt_seg_type ) {
        type = CURVE_2H_INSIDE;
      } else {
        type = CURVE_2H_IN_S;
      }
    } // first half
  } // cur_type

////
//// depending the type, the car uses different formulas to calculate
//// its speed and its direction.
////
  switch( type ) 
  {
    case STRAIGHT_LONG_S:
    case STRAIGHT_LONG_U:
    case STRAIGHT_SHORT_S:
    case STRAIGHT_VSHORT_S:
//    case STRAIGHT_SHORT_U:
      switch( type2 ) 
      {
        case STRAIGHT_INSIDE:
          c.alpha = CALPHA_S_INSIDE;
          c.speed = CSPEED_S_MAX;
          break;
        case STRAIGHT_OUTSIDE:
          c.alpha = CALPHA_S_OUTSIDE;
          c.speed = CSPEED_S_MAX;
          break;
        case STRAIGHT_OUT_IN:
          c.alpha = CALPHA_S_OUT_IN;
          c.speed = CSPEED_S_MAX;
          break;
        case STRAIGHT_PRV:
          c.alpha = CALPHA_S_PRV;
          c.speed = CSPEED_S_PRV;
          break;
        case STRAIGHT_NXT:
          c.alpha = CALPHA_S_NXT;
          c.speed = CSPEED_S_NXT;
          break;
        case STRAIGHT_MIX:
        case STRAIGHT_MIX3:
          c.alpha = CALPHA_S_MIX3;
          c.speed = CSPEED_S_MAX;
          break;
        case STRAIGHT_MIX4:
          c.alpha = CALPHA_S_MIX4;
          c.speed = CSPEED_S_MAX;
          break;
      }
      break;
    case STRAIGHT_SHORT_U:
      c.alpha = CALPHA_S_MIX2;
      c.speed = CSPEED_S_MAX;
      break;
    case CURVE_1H_INSIDE:
    case CURVE_2H_INSIDE:
      c.alpha = CALPHA_C_INSIDE;
      c.speed = CSPEED_C_INSIDE;
      break;
    case CURVE_1H_OUTSIDE:
      c.alpha = CALPHA_C_1H_OUTSIDE;
      c.speed = CSPEED_C_OUTSIDE;
      break;
    case CURVE_2H_OUTSIDE:
      c.alpha = CALPHA_C_2H_OUTSIDE;
      c.speed = CSPEED_C_OUTSIDE;
      break;
    case CURVE_1H_IN_S:
      c.alpha = CALPHA_C_1H_IN_S;
      c.speed = CSPEED_C_OUTSIDE;
      break;
    case CURVE_2H_IN_S:
      c.alpha = CALPHA_C_2H_IN_S;
      c.speed = CSPEED_C_OUTSIDE;
      break;
  }
  return( type );
}

////////////////////////////////////////////////////////////////////////////

static double bSpeedMin( int seg )
{
  return( p.SPEED_MAG_MIN * sqrt( gTrack[seg].t[gHalf].r1 ) );
}

////////////////////////////////////////////////////////////////////////////

static double bSpeedStCvSt( int seg )
{
  return( p.SPEED_MAG * sqrt( gTrack[seg].t[gHalf].r2 ) );
}

////////////////////////////////////////////////////////////////////////////

double bSpeed2( int seg, int half, tControl &c )
{
////
//// variable
////
  double s;
  if( half==0 ) bGetCurveType( seg, 1000000, c ) ;
           else bGetCurveType( seg, 0, c ) ;
 
////
//// precalc
////
  if( gUseOpti==1 ) 
  {
    // optimized speed
    if( half==1 ) return( p2.speed[seg].s1 );
             else return( p2.speed[seg].s2 );
  }

////
//// speed
////
  switch( c.speed ) 
  {
    case CSPEED_C_INSIDE:
      s = bSpeedMin( seg );
      break;
    case CSPEED_C_OUTSIDE:
      s = bSpeedStCvSt( seg );
      break;
    case CSPEED_S_MAX:
    case CSPEED_S_NXT:
    case CSPEED_S_PRV:
    default:
      s = STRAIGHT_SPD;
      break;  
  }
  return( s );
}

///////////////////////////////////////////////////////////////////////////

static double bSegLength( double rad, double len, double width )
{
    if (rad != 0.0) return ((fabs (rad) + 0.5 * width) * len);
    return (len);
}

///////////////////////////////////////////////////////////////////////////

static int bInS( int seg )
// retourne 1 si on est dans un S
{
   int nseg = my_mod(seg+1,t.NSEG);
   if( gTrack[seg].type==gTrack[nseg].type ) return 0;
   if( gTrack[seg].type+gTrack[nseg].type==3 ) return 1;

   int aseg = my_mod(seg+2,t.NSEG);
   if( gTrack[seg].type==gTrack[aseg].type ) return 0;
   return 1;
}

/////////////////////// C A L C _ A L P H A ////////////////////////////////

static double bLaneCur()
{
    double b, sinb, cosb, r1, r2, d1, aux, lane;
    int seg = gCurSeg;

    if( gTrack[seg].type == CUR_ST ) {
       // straight
       lane = 0.5 * gWidth;             // never used
    } else {
       // curve
       d1 = gTrack[ seg ].t[gHalf].d1;
       r1 = gTrack[ seg ].t[gHalf].r1;
       r2 = gTrack[ seg ].t[gHalf].r2;
       b = fabs( gToEndAngle - gCurAngle/2 );

       sinb = sin( b );
       cosb = cos( b );

       aux = d1*sinb;
       if( aux>r2 ) aux=r2;
       lane = sqrt( r2*r2 - aux*aux ) -cosb*d1 -r1;
       lane+= BORDER(seg).in;
    }

    if( gTrack[seg].type==CUR_LC ) return( lane );
                  else return( gWidth-lane );
}

////////////////////////////////////////////////////////////////////////////

static double bLaneNex( double to_end, double out )
{
    double  r2, d3, aux, lane;
    int seg = gNxtSeg;

    if( gTrack[seg].type == CUR_ST ) {
       lane = 0.5 * gWidth;
    } else {
       r2 = gTrack[ seg ].t[0].r2;
       d3 = gTrack[ seg ].t[0].d3;
       if( to_end>d3 ) {
      lane = gWidth - out;
       } else {
      aux = d3 - to_end;
      if( aux>r2 ) aux=r2;
      lane = r2 - sqrt( r2*r2 - aux*aux );
      lane= gWidth - out - lane;
       }
    }

    if( gTrack[seg].type==CUR_LC ) return( lane );
                  else return( gWidth-lane );
}

////////////////////////////////////////////////////////////////////////////

static double bLanePrv( double to_end, double out )
{
    double r2, d3, aux, lane;
    int seg = gPrvSeg;

    if( gTrack[seg].type == CUR_ST ) {
       // straight
       lane = 0.5 * gWidth;             // never used
    } else {
       // curve
       r2 = gTrack[ seg ].t[1].r2;
       d3 = gTrack[ seg ].t[1].d3;
       if( to_end>d3 ) {
      // before the curve
      lane = gWidth - BORDER(seg).out2;
       } else {
      // begin of the curve
      aux = d3 - to_end;
      if( aux>r2 ) aux = r2;   // sometimes gToEnd<0
      lane = r2 - sqrt( r2*r2 - aux*aux );
      lane= gWidth - out - lane;
       }
    }

    if( gTrack[seg].type==CUR_LC ) return( lane );
                  else return( gWidth-lane );
}

////////////////////////////////////////////////////////////////////////////

static double bCalc_MixLane( double pos1, double v1, double pos2, double v2 )

{
  double lane;

  if( gToEnd<pos1 ) {          // end of the straigth
     lane = v1;
  } else if( gToEnd>pos2 ) {   // begin of the straigth
     lane = v2;
  } else {
     // y = f(x)
     //   = y1 + (y2-y1)/(x2-x1)*(x-x1)
     lane = v1 + (v2-v1)/(pos2-pos1)*(gToEnd-pos1);
  }

  return( lane );
}

////////////////////////////////////////////////////////////////////////////

static double bCalcAlpha2( situation & s )
{
////
//// static
////
    static double old_lane;
    static int old_seg;
    static int old_alpha;

    if( gCpt==1 ) {
       old_lane = gToLft;
       old_seg = old_alpha = -1;
    }
////
//// variable
////
    double lane = -1, alpha, f;
    bGetCurveType( gCurSeg, gToEnd, c );
    double ALPHA_MAG = p.ALPHA_MAG_INI;
    double SLIDE_MAG = p.SLIDE_MAG_INI;

    // out (par rapport a Prv)
    double out, out_nxt, f1, f2, nxt_pos0, prv_pos0 ;
    if( bInS(gPrvSeg) ) {
       f1 = BORDER(gNxtSeg).out;
    } else {
       f1 = BORDER(gNxtSeg).out+gBorderOutAdj2;
    }
    f2 = BORDER(gPrvSeg).out2;
    nxt_pos0 =  min(gTrack[gNxtSeg].t[0].d3,gCurLen);
    prv_pos0 =  max(gCurLen-gTrack[gPrvSeg].t[1].d3,0.0);
    out = max( bCalc_MixLane( nxt_pos0, f1, prv_pos0, f2 )
                ,gStartOut2 
              );
    // out (par rapport a nxt)
    out_nxt = BORDER(gNxtSeg).out+gBorderOutAdj2;

    double gCurLane = bLaneCur();
    double gNxtLane = bLaneNex( gToEnd, out_nxt );
    double gPrvLane = bLanePrv( gCurLen - gToEnd, out );
    double aux = 0.5 * gCurLen;
    double ratio01 = (gCurLen-gToEnd)/gCurLen;     // 0->1
    double ratio02 = gToEnd/gCurLen;               // 1->0
    double ratio12 = (1.5*gCurLen-gToEnd)/gCurLen; // firt half:   0,5->1
    double ratio22 = (gToEnd-aux)/gCurLen;         //              0,5->0
    double ratio13 = (aux-gToEnd)/gCurLen;         // second half: 0->0,5
    double ratio23 = (gToEnd+aux)/gCurLen;         //              1->0,5

    double nxt_d3 = gTrack[gNxtSeg].t[0].d3;
    double len = gCurLen - nxt_d3;
    aux = 0.5 * len;
    double ratio1, ratio2;
    if( gToEnd < nxt_d3 ) {
      ratio1 = 1; ratio2=0;
    } else {
      ratio1 = (gCurLen-gToEnd)/len;               // 0->1
      ratio2 = (gToEnd-nxt_d3)/len;                // 1->0
    }
////
//// debug help
////
    if( gCurSeg != old_seg ) 
    {
       // set a breakpoint here
       old_seg = gCurSeg;
       // save in spd file
       #ifdef BULLE_HOME
       if( gSaveSpeed ) 
       {
          OptiSaveDouble( p2.border_in, gTrack[gCurSeg].curve, BORDER(gCurSeg).in );
          OptiSaveInt( p2.alpha_cpt, gCurSeg, gAlphaCpt);
       }
       #endif // BULLE_HOME
       // ...
       gBorderOutAdj2 = min( gBorderOutAdj, gWidth-BORDER(gNxtSeg).in-BORDER(gNxtSeg).out-10.0 );
       gBorderOutAdj  = 0;
       gStartOut2     = min( gStartOut, gWidth-BORDER(gNxtSeg).in-BORDER(gNxtSeg).out-10.0 );
       gStartOut      = 0;     
       if( gCurRadius!=0.0 && gCurAngle/gCurRadius<0.002 )
         gBorderOutAdj2 /=4;
       if( gUseOpti ) 
       {
         gAlphaCpt=p2.alpha_cpt[gCurSeg];
         if( gBorderOutAdj2<1.0 ) gUseOpti = 1;
                             else gUseOpti = 2;
       }
       bTrackCalc( gTrack[gNxtSeg].t[0], gNxtSeg, BORDER(gNxtSeg).in, BORDER(gNxtSeg).out+gBorderOutAdj2 );
       gTrack[gNxtSeg].t[0].speed = bSpeed2( gNxtSeg, 0, gTrack[gNxtSeg].t[0].control );
    }

    if( c.alpha != old_alpha )
       // set a breakpoint here
       old_alpha = c.alpha;
////
//// lane
////
    // local variables for the case (incompatibility BC++ OS/2)
    double old, nxt_lane0, prv_lane0, ref_lane;

    double OldLane = old_lane;
    if( gTrack[gPrvSeg].type==CUR_RC ) OldLane = gWidth-OldLane;

    switch( c.alpha ) {
       case CALPHA_S_INSIDE:
      lane = (3*OldLane+BORDER(gCurSeg).in)/4.0;
      break;
       case CALPHA_S_OUT_IN:
       case CALPHA_S_OUTSIDE:
      lane = gWidth-out;
      ALPHA_MAG = p.ALPHA_MAG_S_OUTSIDE;
      SLIDE_MAG = p.SLIDE_MAG_S_OUTSIDE;
      break;
       case CALPHA_S_NXT:

      lane = gNxtLane;
      SLIDE_MAG = p.SLIDE_MAG_S_NXT;
      break;
       case CALPHA_S_PRV:
      lane = gPrvLane;
      if( gTrack[gPrvSeg].type==CUR_RC ) {
         if( gToLft-lane<0 ) old_lane = (old_lane+lane)*SOFT_S_PRV;
      } else {
         if( gToLft-lane>0 ) old_lane = (old_lane+lane)*SOFT_S_PRV;
      }
      ALPHA_MAG = p.ALPHA_MAG_S_PRV;
      SLIDE_MAG = p.SLIDE_MAG_S_PRV;
      break;
       case CALPHA_S_MIX:
      lane = gNxtLane*ratio1 + gPrvLane * ratio2;
          if( gToLft-lane>0 ) old_lane = (old_lane+lane)*0.5;
      ALPHA_MAG = p.ALPHA_MAG_S_MIX;
      break;
       case CALPHA_S_MIX2:
      nxt_lane0 = bLaneNex( gCurLen, out_nxt );
      prv_lane0 = bLanePrv( gCurLen, out );
      nxt_pos0 =  gCurLen;
      prv_pos0 =  max(gCurLen-gTrack[gPrvSeg].t[1].d3,0.0);
      ref_lane = bCalc_MixLane(
            prv_pos0, prv_lane0,
            nxt_pos0, nxt_lane0
             );
      lane = gPrvLane+gNxtLane-ref_lane;
      if( gTrack[gPrvSeg].type==CUR_RC ) {
         if( lane > gWidth-p.BORDER_IN ) lane = gWidth-p.BORDER_IN;
      } else {
         if( lane < p.BORDER_IN ) lane = p.BORDER_IN;
      }
      ALPHA_MAG = p.ALPHA_MAG_S_MIX2;
      SLIDE_MAG = p.SLIDE_MAG_S_MIX2;
      if( nxt_pos0==gCurLen ) {
         ALPHA_MAG = p.ALPHA_MAG_S_MIX2b;
         SLIDE_MAG = p.SLIDE_MAG_S_MIX2b;
      }
      break;
       case CALPHA_S_MIX3:
      lane = gNxtLane*ratio01 + gPrvLane * ratio02;
      ALPHA_MAG = p.ALPHA_MAG_S_MIX3;
      SLIDE_MAG = p.SLIDE_MAG_S_MIX3;
      break;
       case CALPHA_S_MIX4:
      nxt_pos0 =  min(gTrack[gNxtSeg].t[0].d3,gCurLen);
      prv_pos0 =  max(gCurLen-gTrack[gPrvSeg].t[1].d3,0.0);
      nxt_lane0 = bLaneNex( gCurLen-prv_pos0, out_nxt );
      prv_lane0 = bLanePrv( nxt_pos0, out );
      ref_lane = bCalc_MixLane(
            prv_pos0, prv_lane0,
            nxt_pos0, nxt_lane0
             );
      lane = gPrvLane+gNxtLane-ref_lane;
      if( gTrack[gPrvSeg].type==CUR_RC ) {
         if( lane > gWidth-p.BORDER_IN ) lane = gWidth-p.BORDER_IN;
      } else {
         if( lane < p.BORDER_IN ) lane = p.BORDER_IN;
      }
      ALPHA_MAG = p.ALPHA_MAG_S_MIX4;
      SLIDE_MAG = p.SLIDE_MAG_S_MIX4;
      break;
       case CALPHA_C_INSIDE:
      old = old_lane;
      if( gTrack[gCurSeg].type==CUR_RC ) old = gWidth-old;
      lane = (3*old+BORDER(gCurSeg).in)/4.0;
      ALPHA_MAG = p.ALPHA_MAG_C_INSIDE;
      SLIDE_MAG = p.SLIDE_MAG_C_INSIDE;
      break;
       case CALPHA_C_1H_OUTSIDE:
      lane = gCurLane;
      ALPHA_MAG = p.ALPHA_MAG_C_1H_OUTSIDE;
      SLIDE_MAG = p.SLIDE_MAG_C_1H_OUTSIDE;
      break;
       case CALPHA_C_2H_OUTSIDE:
      lane = gCurLane;
      if( gTrack[gCurSeg].type==CUR_RC ) {
         if( gToLft-lane<0 ) old_lane = (old_lane+lane)*0.5;
      } else {
         if( gToLft-lane>0 ) old_lane = (old_lane+lane)*0.5;
      }
      ALPHA_MAG = p.ALPHA_MAG_C_2H_OUTSIDE;
      SLIDE_MAG = p.SLIDE_MAG_C_2H_OUTSIDE;
      break;
       case CALPHA_C_1H_IN_S:
      lane  = gCurLane*ratio12 + gPrvLane * ratio22;
      ALPHA_MAG = p.ALPHA_MAG_C_1H_IN_S;
      SLIDE_MAG = p.SLIDE_MAG_C_1H_IN_S;
      break;
       case CALPHA_C_2H_IN_S:
      lane  = gNxtLane*ratio13 + gCurLane * ratio23;
      ALPHA_MAG = p.ALPHA_MAG_C_2H_IN_S;
      break;
    }
////
////  change lane for right curve
////
    switch( c.alpha ) {
       case CALPHA_S_INSIDE:
       case CALPHA_S_OUTSIDE:
       case CALPHA_S_OUT_IN:
      if( gTrack[gPrvSeg].type==CUR_RC ) lane = gWidth-lane;
      break;
       case CALPHA_C_INSIDE:
      if( gTrack[gCurSeg].type==CUR_RC ) lane = gWidth-lane;
      break;
    }

////
//// save ALPHA_MAG, SLIDE_MAG for future optimization
////


    if( c.alpha != gAlphaOld ) 
    {
      gAlphaOld = c.alpha;
      gAlphaCpt++;
      if(gCurSeg==0 &&  gAlphaCpt>3 )
        gAlphaCpt=0;
      #ifdef BULLE_HOME
        if( gSaveSpeed ) 
        {
          OptiSaveSpeedSub( p2.alpha, gAlphaCpt, ALPHA_MAG, SLIDE_MAG );
          p2.AlphaNb = gAlphaCpt+1;
        }
      #endif // BULLE_HOME
    }

    if( gUseOpti==1 ) 
    {
      if( gAlphaCpt>p2.AlphaNb-1 ) gAlphaCpt=p2.AlphaNb-1;
      ALPHA_MAG = p2.alpha[gAlphaCpt].s1;
      SLIDE_MAG = p2.alpha[gAlphaCpt].s2;
    }

////
//// start
////
    if( gCpt<p.ALPHA_START) return( 0 );

////
//// after refuel
////
    if( s.fuel>149.95 && gCpt>1000 ) {
       ALPHA_MAG = p.ALPHA_MAG_INI;
       SLIDE_MAG = p.SLIDE_MAG_INI;
    }

////
//// alpha
////
    f = (old_lane-lane)/(gVmag*delta_time);
    if( f<-0.7 ) f = -0.7;
    if( f>0.7 ) f =0.7;
    f = asin( f );
    alpha = f;
    alpha += ALPHA_MAG * (gToLft - lane);
    old_lane = lane;


    if( gTrack[gCurSeg].type==CUR_LC )
       alpha += asin(s.v*delta_time/(gCurRadius+gToLft));
    if( gTrack[gCurSeg].type==CUR_RC )
       alpha += asin(s.v*delta_time/(gCurRadius-gToRgt));
    alpha -= SLIDE_MAG*asin(s.vn/s.v);

    // prevent to go out of the track

    // correct alpha if it predicts that the car goes out
    //  (not used during optimization)
    // avoid the border during the start more than during the race
    f = 2.0;                                   // race

    if( draw.m_bDisplay ) {
       if (s.vn > (f*gToLft))
          alpha -= 0.1*( s.vn-f*gToLft );
       if (-s.vn > (f*gToRgt))
           alpha += 0.1*( -s.vn-f*gToRgt );

       // correct alpha if too close from the border
       if( gToLft<2.0 )
          if( gToLft>1.5 ) alpha += -(2.0-gToLft);
       else alpha += -0.5;
       if( gToRgt<2.0 )
          if( gToRgt>1.5 ) alpha += (2.0-gToRgt);
       else alpha += 0.5;
    } else {
       // optimization
       f=1.5; // plus sensible
       if (s.vn > (f*gToLft) ) time_penality += 0.1*( s.vn-f*gToLft );
       if (-s.vn > (f*gToRgt) ) time_penality += 0.1*(-s.vn-f*gToRgt );
    }

    if (alpha < -1.57) alpha = -1.57;
    if (alpha > 1.57) alpha = 1.57;

    return( alpha );
}

////////////////////////////////////////////////////////////////////////////

////                                                                    ////
 //     calculate braking distance using current and goal speed          //
////                                                                    ////

#define SPEED(a,b) gTrack[a].t[b].speed
#define CONTROL(a,b) gTrack[a].t[b].control

static double bBrakeDist( double cur_v, double nex_v )
{
    return ((sqr(cur_v) - sqr(nex_v))/(2*p.DECEL));
}

static double bBrakeDist2( int seg, double cur_v )
{
    double d3 = 0.0;
    int nxt_seg = my_mod(seg+1,t.NSEG);

    if( CONTROL(nxt_seg,0).speed == CSPEED_C_OUTSIDE ) {
       d3 = gTrack[nxt_seg].t[0].d3;
       if( d3 >  gTrack[seg].t[0].d3 ) d3 = gTrack[seg].t[0].d3;
    }
    return ((sqr(cur_v) - sqr( SPEED(nxt_seg,0) ))/(2*p.DECEL) + d3 );
}

////////////////////////////////////////////////////////////////////////////

static double bCalcVcSub()
{
// calculate target speeds for the three track segments

////
//// static
////
   static int brake_nxt;
   static int old_seg;

   if( gCurSeg != old_seg || gCpt==1 ) {
      // set a breakpoint here
      old_seg = gCurSeg;
      brake_nxt = 0;
   }

////
//// speeds
////
   double speed_cur1 = SPEED(gCurSeg,0);
   double speed_cur2 = SPEED(gCurSeg,1);
   double speed_nxt1 = SPEED(gNxtSeg,0);

////
//// save speed for future optimization
////
   #ifdef BULLE_HOME
      if( gSaveSpeed )
         OptiSaveSpeedSub( p2.speed, gCurSeg, speed_cur1, speed_cur2 );
   #endif // BULLE_HOME

   // now set the tire speed = vc, brake if neccessary
   if( gToEnd+gNxtLen+gAftLen < bBrakeDist2(gAftSeg, gVt) )
     return SPEED( my_mod(gCurSeg+3,t.NSEG), 0 ); 

   double dist_aft = gToEnd+gNxtLen;
   if( gTrack[gNxtSeg].type!=CUR_ST ) dist_aft = gToEnd+gNxtLen/1.5;
   if( dist_aft < bBrakeDist2(gNxtSeg, gVt) )
     return SPEED(gAftSeg,0);                // 'after' segment

   // calculate braking distance for the next segment
   double brake_dist_nxt = bBrakeDist2(gCurSeg, gVt );
   if( gTrack[gCurSeg].type!=CUR_ST ) brake_dist_nxt *= 1.5;
   brake_nxt =(gToEnd < brake_dist_nxt)?1:0;
   if( brake_nxt )
      return speed_nxt1;                      // brake for next corner

   if( gVt<speed_nxt1 ) 
   {
     if( gCurAngle != 0.0 ) 
     {
       if( gToEnd < ((p.START_ACC-p.START_ACC2*gCurAngle) * gCurLen) )
         return speed_nxt1;                  // accelerate for next segment
      }
   }

   // keep current speed
   double brake_dist_cur12 = bBrakeDist(gVt, speed_cur2);
   if( brake_dist_cur12<0 ) brake_dist_cur12 = 0;
   if( gToEnd-1.5*brake_dist_cur12 > gCurLen*0.5 )
      return speed_cur1;
   else
      return speed_cur2;
}

////////////////////////////////////////////////////////////////////////////

static double bCalcVc3( situation & s )
{
    double vc = bCalcVcSub();

////
//// before refueling
////
    if( s.fuel<0.05 ) {
       vc = vc*( 1.0-(16.0*(0.05-s.fuel)) );
    }

    return( vc );
}

////////////////////////////////////////////////////////////////////////////

static void bPassingNo(  situation & s, double &vc )
{
////
//// slow down if a car in front
////
  int i;
    
  for( i=0;i<3; i++) 
  {
    if (s.nearby[i].who<16) {
                         // if there is a close car
       double y=s.nearby[i].rel_y;           // get right distance
       double vy=s.nearby[i].rel_ydot;       // get relative speed

       // if the car is getting closer
       if( y<2.0*CARLEN ) 
      vc = min(vc,s.v+vy-20);
       else if( vy<0.0 && (y<4.6*CARLEN || y<-10.0*vy) )
          vc = max(50,min(vc,s.v+vy-5.0)); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////

static double distancetobraketo(double vnow, double vthen, double radius)
{
  
  static double BC1 = .014;
  static double BC2 = .020;
    
  if (vnow<vthen)
    return 0.0;  // no need to brake as we are already at the desired speed!
  else if (radius != 0)
    return (BC2 * (vnow+vthen) * (vnow-vthen));
  return (BC1 * (vnow+vthen) * (vnow-vthen));
  
}

////////////////////////////////////////////////////////////////////////////

static void bPassing4( situation & s, double &alpha, double &vc )
{
  int danger = 0;
  int kount = 0;
  double vc1;
  double v[4], d[4], x[4], y[4], vx[4], vy[4];
  for(int i=0;i<4;i++)
     if(s.nearby[i].who<16)
     {
        x[i]=s.nearby[i].rel_x;         // distance to right (or left if < 0)
        y[i]=s.nearby[i].rel_y;         // distance ahead, always positive
        vx[i]=s.nearby[i].rel_xdot;     // relative lateral speed component
        vy[i]=s.nearby[i].rel_ydot;     // relative forward speed (negative)
        d[i]=sqrt(x[i]*x[i] + y[i]*y[i]);       // distance to other car
        v[i]=sqrt(vx[i]*vx[i] + vy[i]*vy[i]);   // relative speed

        if (vy[i] > -1)  continue;      // ignore faster cars
        else if (d[i] > 1.2*distancetobraketo(s.v, s.v+vy[i]+5,s.cur_rad))
           continue;       // check whether you need to brake
        ++kount;
        if (fabs(x[i])>1.5*CARWID && x[i]*vx[i]>-10 && 
            fabs(vx[i])>10 && vy[i]>-10)
          continue; // if slow car ahead, reserve more safety space
        else if (fabs(x[i])>CARWID && x[i]*vx[i]>-10 &&
            fabs(vx[i])>10 && vy[i]<=-10)
          continue; // if fast car ahead, reserve less safety space
        else danger = 1;// brake or pass if trouble car within braking distance
    }

  if (danger)
  {
     if(kount > 2)
        vc1 = min((s.v + vy[0] - 5),(s.v + vy[1] - 5));
     else
     {
        vc1 = min((s.v + vy[0] + 5),((s.damage >20000?0.9:0.95)*vc));
        if (!s.cur_rad)
        {  //straigth => pass!
           if (x[0] < 0)  // car in left
              if (s.to_lft + x[0] < t.width - 1.5* CARWID)
                 alpha -= 0.03;
              //if we have room in right, turn right
              else  alpha += 0.03;
           else if (x[0] >= 0)    // car in right
           if (s.to_lft + x[0] > 1.5* CARWID) // if we have room in left
              alpha += 0.03; //turn left
           else 
              alpha -= 0.03;
        }
     } // end of kount < 2
  }// end of danger
  else 
     vc1 = vc; //this executes if no danger seen

  vc1 = max(vc1,5);  // To avoid stopping behind very slow car
  vc = vc1;
}

////////////////////////////////////////////////////////////////////////////

static void bPassingMain( situation & s, double &alpha, double &vc )
{
  // During the start, don't modify the trajectory
  if( gStartDist==0 && gCpt<p.ALPHA_START+100 ) 
  {
    if( vc-30>s.v ) vc = 400;
      return;
  }

  // If the damage is too high, stay alive
  // => don't overtake anymore
  if( s.damage>29000 ) 
  {
    bPassingNo( s, vc );
    return;
  }

  /*
  // If the fuel is low, do not overtake  ( fuel 2.0 ~= 0.5 lap )
  // => don't want to overtake 2 times the same car :)
  if( s.fuel<2.0 ) 
  {
    bPassing4( s, alpha, vc );
    return;
  }
  if( (s.damage>24000 && s.laps_to_go>5) || s.damage>25000  ) 
  {
    bPassing4( s, alpha, vc );
    return;
  }
  if(( s.damage>10000 && s.damage<12000 ) 
  || ( s.damage>19000 ) 
  || ( gCpt<1000 && s.position>3 ) ) 
  {
    bPassing4( s, alpha, vc );
  }
  */
  bPassing4( s, alpha, vc );
}
////////////////////////////////////////////////////////////////////////////

static void bOutOfTrack(  situation & s, double &alpha, double &vc )
{
   // prevent to go out of the track

   double f = 2.0;
   double vc2 = (300.0-min(200.0,vc))/300.0;

   // correct alpha if it predicts that the car goes out
   //  (not used during optimization)

   if (s.vn > (f*gToLft)) {
      alpha -= 0.2*( s.vn-f*gToLft );
   }
   if (-s.vn > (f*gToRgt)) {
      alpha += 0.2*( -s.vn-f*gToRgt );
   }
   // correct alpha if too close from the border
   if( gToLft<2.0 )
      if( gToLft>1.5 ) alpha += -(2.0-gToLft)*vc2;
            else alpha += -0.5*vc2;
   if( gToRgt<2.0 )
      if( gToRgt>1.5 ) alpha += (2.0-gToRgt)*vc2;
            else alpha += 0.5*vc2;

   vc = vc;
}

////////////////////  D R I V E R   F U N C T I O N  ///////////////////////

////////////////////////////////////////////////////////////////////////////

static void bCalcInit( situation & s )
{
//// start curve
    if(gCpt==0) { 
       if( bTrackGetRay(1)>0.0 ) {
          gStartOut = gBorderOutAdj = gStartY;
       } else {
          gStartOut = gBorderOutAdj = gWidth-gStartY;
       }
       if( !draw.m_bDisplay ) gStartOut = gBorderOutAdj = 0;
    }
//// start 1st line or 2nd  line ?
    if(gCpt==1) { 
       gStartDist = s.position/4;
    }

//// calc the Slide Angle and the speed of the car, using the
////   acceleration to improve the precision.
////
    double ang = asin( s.vn/s.v );
    gVt = s.v * cos( ang );
    gVn = s.vn;

    // gCpt
    gCpt++;

    // the cen_a and tan_a are badly initialised
    if( gCpt>1 ) {
       // add the acceleration to the speed to predict the speed during the
       //  next calculation.
       gVt += s.tan_a * delta_time;
       gVn += s.cen_a * delta_time;
    }
    gVmag = sqrt( gVt*gVt + gVn*gVn );

////
//// init ToEnd
////
    gToEnd = bSegLength( s.cur_rad, s.to_end, gWidth );
    gCurSeg = s.seg_ID;     // current segment number, 0 to NSEG-1

    gToEnd -= 0.0;
    while( gToEnd<0.0 ) {
       gCurSeg++;
       if( gCurSeg==t.NSEG ) gCurSeg = 0;
       gToEnd += gTrack[gCurSeg].length;
    }

////
//// init segment number
////
    gNxtSeg = my_mod(gCurSeg+1,t.NSEG);
    gAftSeg = my_mod(gCurSeg+2,t.NSEG);
    gPrvSeg = my_mod(gCurSeg+t.NSEG-1,t.NSEG);

////
//// convert s.to_end to feets (for curves, don't want to work with radians :)
////
    gToLft = s.to_lft;
    gToRgt = s.to_rgt;

////
//// parameters for Curves
////
    gCurAngle = bTrackGetAngle(gCurSeg);
    gCurRadius = bTrackGetRay(gCurSeg);
    if(gCurRadius !=0.0 ) {
    gToEndAngle = s.to_end;
    }

////
//// convert s.cur_len, s.nex_len to feets (for curves)
////
    gCurLen = gTrack[gCurSeg].length;
    gNxtLen = gTrack[gNxtSeg].length;
    gAftLen = gTrack[gAftSeg].length;
    gHalf = gToEnd<gCurLen/2;
}

////////////////////////////////////////////////////////////////////////////

con_vec Bulle(situation& s)
{
  const char name[] = "Bulle";        // This is the robot driver's name!
  static int init_flag = 1;           // cleared by first call
  double alpha, vc;                   // components of result
  con_vec result;                     // This is what is returned

  result.request_pit = 0;
  result.repair_amount = 0;

  if( s.starting )
  {
     my_name_is(name);                  // copy name over
     if( s.distance==0.0 )
     {
        init_flag = 0;                     // reset init flag
        result.vc = STRAIGHT_SPD; result.alpha = 0;
        return( result );
     }
     bInit(0);
     gCpt = 0;
     gAlphaOld=-1; gAlphaCpt=-1;
     time_penality = 0;
     ////
     //// get the start position
     ////
     gStartPos = int( s.to_rgt/(gWidth/4) );
     gStartDist = 0; // corrected when gCpt==1
     gStartY = s.to_rgt;
     ////
     //// fuel when starting
     ////
     if((s.stage==QUALIFYING)||(s.stage==PRACTICE))
       result.fuel_amount = 30;
     else
       result.fuel_amount = MAX_FUEL;
  }

  // init global variables
  bCalcInit( s );

  // get the car out of an abnormal condition, thanks Mitchell :)
  if( stuck( s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc ) )
      return result;

  alpha = bCalcAlpha2( s );
  vc = bCalcVc3( s );
  if( draw.m_bDisplay ) bPassingMain( s, alpha, vc );
  if( draw.m_bDisplay ) bOutOfTrack( s, alpha, vc );

  #ifdef BULLE_HOME
    if( !draw.m_bDisplay )
    {
        double max;
        if( s.cur_rad==0.0 ) max = 4.0;
                        else max = 2.5;
        if( s.to_lft<max || s.to_rgt<max ) 
        {
            double f = min(s.to_lft,s.to_rgt);
            if( f<max-0.5 ) time_penality += 0.5;
                       else time_penality += max-f;
        }
    }

    if( OPTI_PARAM2 ) 
    {
        if( gOptiCycle & 0x01 ) gBorderOutAdj = 2.0;
                           else gBorderOutAdj = 100.0;
    }
  #endif // BULLE_HOME

  result.vc = vc;   result.alpha = alpha;

  if( s.fuel<6.0 ) 
  {
      int cpt = min( 20, s.laps_to_go );
      int damage_min = max( ((long)s.damage)-10000, 0 );
      result.request_pit   = 1;
      result.repair_amount = damage_min*cpt/20;
      result.fuel_amount = MAX_FUEL;
  }
  if( s.laps_to_go>1      )
  {
      if( s.damage>25000 )
      {
          int cpt = min( 20, s.laps_to_go );
          result.request_pit   = 1;
          result.repair_amount = max( s.damage*cpt/20, 5000 );
          result.fuel_amount = MAX_FUEL;
      }
  }
  return result;
}

/////////////////////  E N D   O F   B U L L E . C P P  ////////////////////

