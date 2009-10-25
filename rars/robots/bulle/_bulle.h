//--------------------------------------------------------------------------
//
//    FILE: BULLE.H (portable)
//
//    This file contains the specific type of Bulle
//
//    Version       Author          Date
//      0.1      Marc Gueury       21/4/99
//
//--------------------------------------------------------------------------

#ifndef _bulle_H
#define _bulle_H

//--------------------------------------------------------------------------
//                            D E F I N E
//--------------------------------------------------------------------------

#define STRAIGHT_RAD         100000.0   // radius for straigths
#define STRAIGHT_SPD           400.00   // speed for straights
#define MAX_SEG                    90   // max segments (gTrack)
#define MAX_CURVE                  50   // max curves (gCurve)
#define TPARAM_NB                  36   // number of parameters

// curve types
#define CUR_ST 0
#define CUR_LC 1
#define CUR_RC 2

//--------------------------------------------------------------------------
//                             M A C R O S
//--------------------------------------------------------------------------

#ifndef max 
  #define max(x,y)                (((x)<(y))?(y):(x))        // return maximum
  #define min(x,y)                (((x)>(y))?(y):(x))        // return minimum
#endif
#define sqr(x)                  ((x)*(x))                  // return x}
#define my_mod(x,y)             (((x)<(y))?(x):(x-y))      // return absolute

//--------------------------------------------------------------------------
//                              T Y P E S
//--------------------------------------------------------------------------

typedef struct 
{
  int alpha;
  int speed;
} tControl;

typedef struct 
{
 // one for each half segment
      double r1;           // internal radius
      double r2; 	   // radius ST_CV_ST
      double d1; 	   //
      double d3; 	   //
      double speed;        // speed
      tControl control;    // type of controle
} T_track_sub;

typedef struct 
{
   // ST_CV_ST
   double length;       // length
   T_track_sub t[2];

   // type
   int type;
   int curve;           // curve number
} T_track ;

typedef struct 
{
   double in;
   double out;
   double out2;
} T_border;

typedef struct 
{
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
    double SLIDE_MAG_S_NXT;  	    // bAlpha, s.nxt
    double ALPHA_MAG_S_PRV;  	    // bAlpha, s.prv
    double SLIDE_MAG_S_PRV;         // bAlpha, s.prv
    double ALPHA_MAG_S_MIX;  	    // bAlpha, s.mix
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

typedef struct 
{
   double s1;       // speed segment 1
   double s2;       // speed segment 2
} T_speed_sub;

typedef struct 
{
    char   trackfile[16];           // track file name
    // speed
    T_speed_sub speed[ MAX_SEG ];
    // alpha_mag, slide_mag
    int    AlphaNb;
    T_speed_sub alpha[ MAX_SEG ];
    // border_in, border_out
    double border_in[ MAX_SEG ];
    int alpha_cpt[ MAX_SEG ];
} T_speed;

#endif //_bulle_H
