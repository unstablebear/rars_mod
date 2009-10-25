/*
file name: JOCOSA83.cpp
author:    Jorge Cervantes
e-mail:    JorgeCervantesO@netscape.net
robot:     JOCOSA83

  This robot is designed to race in ANY track so I wish to race
it in ALL upcomming races.

  The code is NOT confidential but it is very messy :-(

  This robot does NOT need to read any data file.

* New code for overtaking added.

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
//#include <dos.h>
#include <iostream.h>
#include "car.h"
#include "os.h"
#include "misc.h"
#include "track.h"

// parameters to tinker with:
/*
static
double
constant[ 20 ] = { 0.50,  -40.0, -35.00, 10.0,  1.0,
		   0.05,    7.0,   1.00, 0.15,  0.6,
		   0.00,    4.0,   1.00, 1.50,  1.0,
		   0.00,    0.0,   0.00, 0.10,  0.0  };

delta[ 20 ]    = { 0.1 ,   -0.1,  -0.1, 1.00,  5.0 ,
		   0.01,    1.0,   1.0, 0.01,  0.01,
		   0.01,    1.0,   0.1, 0.01,  0.1 ,
		   0.0 ,    0.0,   0.0, 0.05,  0.0  };

static
char * names[ 20 ]    = { "CAUTION",
			  "BRAKE_ACCEL",
			  "BRK_CRV_ACC",
			  "WIDTH_FACTOR",
			  "DIST_FROM_INSIDE",

			  "WIDTH",
			  "SEGMENT_BREAK",
			  "DELTA_PERCENT",
			  "EMERG_ALPHA",
			  "TOO_CLOSE_FACTOR",

			  "ACCUM_FACTOR",
			  "CRV_RESISTANCE",
			  "PASS_FACTOR",
			  "ALPHA_FACTOR",
			  "BRAKE_FACTOR",

			  "D_TO_IDEAL",
			  "D_MINUS_LAST_D",
			  "ACCUM_D",
			  "STEER_SPD",
			  "DAMAGE_74" };
*/
// static double CAUTION   = 0.5;        // CARLENs behind cars.
static double BRAKE_ACCEL = -40;      // acceleration when braking on straight
static double BRK_CRV_ACC = -35;      // acceleration when braking in curve
static double WIDTH_FACTOR = 10;     //
static double DIST_FROM_INSIDE = 1; // target distance from curve's inner rail
//double WIDTH = 5;         // affects the bias of steering
//double SEGMENT_BREAK = 6;    // Segment number to plant Break Point
//double DELTA_PERCENT = 7;     // Check short straight flag
//double EMERG_ALPHA = 8;       // Emergency alpha
//double TOO_CLOSE_FACTOR = 9;  // Width factor when approaching curves too closely
//double ACCUM_FACTOR = 10;     // Integration factor
//double CRV_RESISTANCE = 11;  // Curve resistance to follow ideal lane
//double PASS_FACTOR = 12;      // Curve width factor
static double ALPHA_FACTOR = 2.0;       // cornering alpha factor
//double BRAKE_FACTOR = 14;     // cornering alpha factor
//double D_TO_IDEAL = 15;       // distance to ideal lane
//double D_MINUS_LAST_D = 16;  // speed to ideal lane
//double ACCUM_D = 17;          // Accummulated distance to ideal lane
//double STEER_SPD   = 18;     // steering speed limit
//double DAMAGE = 19;           // Current Damage


static double *rad        = NULL,
	      *len        = NULL,
//	      *fast_r     = NULL,
//	      *fast_x     = NULL,
	      *DP[2]      = { NULL, NULL },
	      width       = 0.0,
	      real_width  = 0.0,
	      max_speed   = 300.0,
	      cur_speed   = 0.0,
	      last_speed  = 1.0,
	      cur_x       = 0.0,
	      last_x      = 0.0,
	      avg_a       = 40.0,
	      last_rad    = 0.0,
	      last_damage = 0.0,
				lap_damage  = 0.0,
				max_lap_damage = 0.0;
//            prev_x      = 0.0;

static int    too_close      = 0,
	      Total_Laps     = 0,
	      width_reduced  = 0,
	      segment_change = 0,
	      outside_lane   = 0,
	      curveing       = 0,
//            first_lap      = 1,
	      SEGS_AHEAD     = 1,
	      numseg         = 1,
	      check_delta_percent = 1,
	      forwards            = 1;
//	      recursed            = 0;


static situation s;



static int sign( double x ){ return x == 0.0 ? 0 : x > 0.0 ? 1 : -1;}

static int next( int i ){ return ++i % numseg; };
static int prev( int i ){ return --i < 0 ? numseg + i: i; };

static int n( int i )
{ if( forwards ) return next( i ); return prev( i );}

static int p( int i )
{ if( forwards ) return prev( i ); return next( i );}

static double curve_distance( int i );
static double radius( int i );
static double length( int i
//                    int check_prev_rad,
//                    int check_next_rad
		    ); //, int check_delta_percent )


static double hipotenusa( double x, double y )
{ return sqrt( x*x + y*y ); }

static double cateto( double h, double x )
{ return h>x ? sqrt( h*h - x*x ) : 0.0; }

static void set_rad_len( void )
{
  segment *lftwall = get_track_description().trackin,
	  *rgtwall = get_track_description().trackout;

  if( len[0] == 0.0 )
    for( int i=0; i<numseg; i++)
    { rad[i] = lftwall[i].radius;
      if( rad[i]<0.0 )
	rad[i] = rgtwall[i].radius;
      len[i] = rgtwall[i].length;
    }
}

/*
static void draw_trajectory( int i )
{ double R,x,y,d,a;

  segment *lftwall = get_track_description().trackin;

  R = radius(i);

  if( R == 0 )
    return;

  x = lftwall[i].cen_x,
  y = lftwall[i].cen_y;

  set_color(oRED);
  draw_arc( 5, x, y, 0, PI*2 );

  d = fabs( R ) - DIST_FROM_INSIDE - fabs( rad[i] );
  a = lftwall[i].beg_ang;
  R > 0.0
  ?(a += PI*0.5
       - len[i]*DP[!forwards][i]
       + length(i)*0.5)
  :(a -= PI*0.5
       - len[i]*DP[!forwards][i]
       + length(i)*0.5);

  x += d * cos( a );
  y += d * sin( a );

  draw_line( lftwall[i].cen_x, lftwall[i].cen_y,
             x, y );
  draw_arc( 10, x, y, 0, PI*2 );
  draw_arc( R,  x, y, 0, PI*2 );

//  getch();
}


static void trace_trajectories()
{
}
*/


static double s_curve_percent( int i, double l_n_i, int nn_i )
{ //static int j=0;
  double x1, y1, x2, y2, xx, yy, d, d2, r1, r2, l_i, l_nn_i, d3=0.0,
	 cur_factor = DP[forwards][i];
//         delta = 0.0;

//  while( 1 )
//  {

//    check_delta_percent = 0;

    r1     = fabs( radius( i ));      // rad_big
    l_i    = length( i );

    r2     = fabs( radius( nn_i ));   // rad_big
    l_nn_i = length( nn_i );

//    check_delta_percent = 1;

    x1 = fabs( rad[ i ] )
       - fabs( rad[ i ] )
       * cos( l_i*0.5 - len[ i ]*DP[!forwards][i] )
       + ( r1 - DIST_FROM_INSIDE )
       * cos( l_i*0.5 - len[ i ]*DP[!forwards][i] );

    y1 =
       - fabs( rad[ i ] )
       * sin( l_i*0.5 - len[ i ]*DP[!forwards][i] )
       + ( r1 - DIST_FROM_INSIDE )
       * sin( l_i*0.5 - len[ i ]*DP[!forwards][i] );

//    x2 =
//       - real_width
//       - fabs( rad[ n( n( i )) ] )
//       + fabs( rad[ n( n( i )) ] ) * cos( len[ n( n( i )) ] * 0.5)
//       - ( r2 - DIST_FROM_INSIDE ) * cos( len[ i ] * 0.5 );

    x2 = fabs( rad[ i ] )
       - fabs( rad[ i ] ) * cos( len[ i ] )
       + l_n_i * sin( len[ i ] )
       - ( real_width
	 + fabs( rad[ nn_i ] )
	 )
	 * cos( len[ i ] )
       - ( r2
	 - DIST_FROM_INSIDE
	 - fabs( rad[ nn_i ] )
	 )
	 * cos( len[ i ]
              + len[ nn_i ]*DP[!forwards][nn_i]
              - l_nn_i * 0.5
              );

//    y2 = len[ n( i ) ]
//       + fabs( rad[ n( n( i )) ] ) * sin( len[ n( n( i )) ] * 0.5)
//       - ( r2 - DIST_FROM_INSIDE ) * sin( len[ i ] * 0.5 );

    y2 = ( fabs( rad[ i ] )
         + real_width
	 + fabs( rad[ nn_i ] )
	 )
	 * sin( len[ i ] )
       + l_n_i * cos( len[ i ] )
       + ( r2
	 - DIST_FROM_INSIDE
	 - fabs( rad[ nn_i ] )
	 )
	 * sin( len[ i ]
              + len[ nn_i ]*DP[!forwards][nn_i]
              - l_nn_i * 0.5
              );


    xx = x2 - x1;
    yy = y2 + y1;
    d  = hipotenusa( xx, yy );

    x2 = fabs( rad[ i ] )
       - ( fabs( rad[ i ] )
         + real_width
         + fabs( rad[ nn_i ] )
         )
         * cos( len[ i ] )
       + l_n_i * sin( len[ i ] );

    y2 = ( fabs( rad[ i ] )
         + real_width
	 + fabs( rad[ nn_i ] )
	 )
	 * sin( len[ i ] )
       + l_n_i * cos( len[ i ] )
         ;

    xx = x2 - x1;
    yy = y2 + y1;
    d2 = hipotenusa( xx, yy );

    if( sign( rad[ n(nn_i) ] ) * sign( rad[ i ] ) <= 0 )
    {
      x2 -= ( fabs( rad[ nn_i ] )
            + real_width
            + fabs( rad[ n(nn_i) ] )
            )
            * cos( len[ i ] - len[ nn_i ] );
      y2 -= ( fabs( rad[ nn_i ] )
            + real_width
            + fabs( rad[ n(nn_i) ] )
            )
            * sin( len[ i ] - len[ nn_i ] );

      xx = x2 - x1;
      yy = y2 + y1;
      d3 = hipotenusa( xx, yy );
    }

    if( ( d > r1 + r2 + CARWID
          ||
          r1 < d - r1 - CARWID
        )
        &&
        d2 - fabs( rad[ nn_i ] ) - DIST_FROM_INSIDE - CARWID*3 > r1
      )
    { // Circles are not overlapping
      if( cur_factor == 0.0 )
        // first time
//        if( l_n_i == 0.0
//            &&
//            sign( rad[p(i)] ) != sign( rad[i] )
//            &&
//            DP[ !forwards][ i ] <= 0.0
//          )
          return cur_factor - (l_n_i == 0.0 ? 0.01 : 0.00);

//        else
//          return cur_factor;

      // not first time
      if( cur_factor > 0.0 )
        // increasing but now not overlapping
        return cur_factor;
//        break;

      // decreasing and not yet overlapping
      if( cur_factor > -0.99
          &&
          1.0 + cur_factor + DP[!forwards][i] > 0.01
          &&
          ( sign( rad[ n(nn_i) ] ) * sign( rad[ i ] ) <= 0
            ||
            d3 + fabs( rad[ n(nn_i) ] ) + DIST_FROM_INSIDE > r1
          )
        )
        return cur_factor - 0.01;

      // the max
      return cur_factor;
//        break;
    }

      // circles are overlapping
      if( cur_factor == 0.0 )
        // first time
        return cur_factor + 0.01;

      // not first time
      if( cur_factor < 0.0 )
        // decreasing but now overlapping
        return cur_factor;
//        break;

      // increasing and still overlapping
      if( cur_factor < 0.99
          &&
          l_i < 2*PI
        )
	return cur_factor + 0.01; //s_curve_percent( i, l_n_i, nn_i, cur_factor + 0.1 );

      // the max
      return cur_factor;
//        break;

//    else
//      if( cur_factor < 1.0 )
//        cur_factor = s_curve_percent( i, l_n_i, nn_i, cur_factor + 0.2 );

//    if( len[ i ] *( 1+cur_factor ) < len[ i ] - len[ nn_i ] )
//      return cur_factor - delta;
//      break;

//  }

}


static int touching_3rd_curve( int i )
{
  if( rad[ n(n(i)) ] == 0.0 )
    return 0;

  if( sign( rad[ n(n(i)) ] ) == sign( rad[ i ] ))
    return 0;

  //circles touch:
  return 0;//forwards;
}


static double c_curve_percent( int i )
{ double l_i, R, r_o, x, y, d, cur_factor=DP[forwards][i],t;

//  do
  {
    // calculate if the trajectory of i is
    // inside of the outer side of n(i)
//    check_delta_percent = 0;
    R   = fabs( radius( i ));
    l_i = length( i );
//    check_delta_percent = 1;

    r_o = fabs( rad[ n(i) ] ) + real_width*0.5;

    x = ( R - DIST_FROM_INSIDE - fabs( rad[ i ] ) )
      * cos( len[ i ]*DP[forwards][i] - l_i*0.5 )
      + fabs( rad[ i ] )
      - fabs( rad[ n(i) ] );

    y = ( R - DIST_FROM_INSIDE - fabs( rad[ i ] ) )
      * sin( len[ i ]*DP[forwards][i] - l_i*0.5 );

    // d = distance between next center and current center of R
    d = hipotenusa( x, y );

    // calc distance to tangent of next curve's outer side
    t = r_o
      + ( fabs( rad[i] )
        - fabs( rad[ n(i) ] )
        )
        * cos( len[ n(i) ] )
      + ( fabs(R)
        - DIST_FROM_INSIDE
        - fabs(rad[i])
        )
        * cos( len[n(i)]
             - len[i]*DP[forwards][i]
             + l_i*0.5
             );

  }

  if( ( touching_3rd_curve( i )
        ||
//        0 > r_o - d
//        ||
        R + d > r_o
        &&
        ( t < R
          ||
          r_o > R
        )
      )
    )
    // R too big
    if( cur_factor < .99
	||
        ( fabs( rad[ i ] ) > fabs( rad[ n(i) ] )
          &&
          len[ i ] * cur_factor < len[ n(i) ]
        )
      )
      if( l_i < 2*PI )
        return cur_factor + 0.01;//(len[ n(i) ]*0.01)/len[i];

  // Circles are not overlapping or it has been enough percent

  return cur_factor;
}


static double C_curve_percent( int i )
{ double cur_factor = DP[forwards][i], l_i, R, x, y, d, r_in, r2;

//  x2 = curve_distance( n(n(i)));
//  check_delta_percent = 0;
//  x0 = 0.5 * curve_distance( i );
//  check_delta_percent = 1;

//  if( len[ n( i ) ] > x0 + x2 )
//    return 0.0;

//  while(1)
  {
//    check_delta_percent = 0;
    R   = fabs( radius( i ));
    l_i = length( i );
    r2  = fabs( radius( n(n(i)) ));
//    check_delta_percent = 1;

    r_in = fabs( rad[ n(n(i)) ] )
         + ( //len[i] > len[ n(n(i)) ]
             //? width * 0.5 :
              DIST_FROM_INSIDE
           );

    x = len[ n(i) ]
//      + r_in
//      * sin( len[ n(n(i)) ] * 0.5 )
      - ( R - DIST_FROM_INSIDE  - fabs( rad[ i ] ))
      * sin( len[ i ]*(1+DP[!forwards][i]) - l_i*0.5 );

    y = ( R - DIST_FROM_INSIDE  - fabs( rad[ i ] ))
      * cos( len[ i ]*(1+DP[!forwards][i]) - l_i*0.5 )
      + fabs( rad[ i ] )
      + DIST_FROM_INSIDE
      - r_in;

    d = hipotenusa( x, y );

    if( d + r_in < R
	&&
	( len[ i ] * cur_factor < len[ n(n(i)) ]
	  &&
	  cur_factor < .99
          &&
          l_i < 2*PI
	)
        &&
        sqrt(R) > sqrt(r2) - 5
      )
      return cur_factor + 0.01;
  }

  return cur_factor; // len[ n( i )] / ( x3 + x1 );
}


/* Not used
static int first_segment( )
{ static int first_seg = 1;

  first_seg &= s.seg_ID == 0;

  return first_seg;
}
*/

static void request_pit( con_vec &result )
{ static int last_pit = -1;
  static double last_damage = 0;
  double l = get_track_description().length / 5280.; // in miles
  long double amount;

	result.request_pit = 0;

  if( s.out_pits == 1 )
    last_pit = s.laps_done,
    last_damage = s.damage;

//  if( s.damage > 20000 )
//  {
//    result.repair_amount = s.damage;
//    result.fuel_amount = 150.0;
//    result.fuel_amount -= s.fuel;
//    result.request_pit = 1;
//    return;
//  }

  if( s.damage > 20000
      ||
      s.damage > 15000
//    &&
//    !first_segment()
      &&
      s.laps_to_go > 1
      &&
      s.laps_done > last_pit
      &&
      ( s.damage - last_damage ) / ( s.laps_done - last_pit )
      >
      (30000.-s.damage) / s.laps_to_go
    )
  {
    amount = s.damage;
    amount -= 15000.;
    for(int i=1; i<s.laps_to_go && amount < 30000.; i++ )
      amount += ( s.damage - last_damage ) / ( s.laps_done - last_pit );

    if( amount > s.damage )
      amount = s.damage;

    if( amount < 0.0 )
      amount = 0.0;

    result.repair_amount = (int) amount;// * 0.5;

    result.fuel_amount = 150.0;
    result.fuel_amount -= s.fuel;
    result.request_pit = 1;
    return;
  }


  if(
      s.fuel_mileage > 0.0
      &&
      s.fuel * s.fuel_mileage / l < 2 // less than 2 laps
    )
  {
//     if( s.fuel_mileage == 0.0 )
//       result.fuel_amount = 150.0;

//     else
//     { int pitstops;

//       result.fuel_amount = ( s.laps_to_go + 5 )
//                          * get_track_description().length / 5280.0
//                          / s.fuel_mileage;
//       pitstops = result.fuel_amount / 150;

//       result.fuel_amount /= pitstops + 1;

       result.fuel_amount = 150.0;
//     }
    result.fuel_amount -= s.fuel;

    result.repair_amount = s.damage;// * 0.5;

    result.request_pit = 1;
  }
}


static double delta_percent( int i )
{ double dp = 0.0;

//  if( request_pit()
//      &&
//      i == numseg-1
//    )
//    return 1.0;

  if( len[ i ] == 0.0 )
    return 0.0;

  if( rad[ i ] == 0.0 )
  {
    DP[forwards][ i ] = 0.0;

    return 0.0;
  }

//  if( DP[forwards][ i ] != 0.0 )
//    return DP[forwards][ i ];

  if( !check_delta_percent )
    return DP[forwards][ i ];

  if( rad[ n(i) ] != 0.0 )
    if( sign( rad[ i ] ) != sign( rad[ n(i) ] ))
      // S curve
      dp = s_curve_percent( i, 0.0, n(i) );

    else
      // c curve
      dp = c_curve_percent( i );
  else
    if( rad[ n(n(i)) ] == 0.0 )
      // 2 straights after segment i
      dp = 0.0;

    else
      if( sign( rad[ n(n(i)) ] ) != sign( rad[ i ] ))
	// kind of S curve.
	dp = s_curve_percent( i, len[ n(i) ], n(n(i)) );

      else
	// curve-straight-curve same side
	dp = C_curve_percent( i );

  DP[forwards][ i ] = dp;

  return( dp );
}



static double delta_percent_prev( int i )
{ double dp = 0.0;

  if( len[ i ] == 0.0 )
    return 0.0;

  if( rad[ i ] == 0.0 )
  {
    DP[!forwards][ i ] = 0.0;

    return 0.0;
  };

//  if( DP[ !forwards ][ i ] != 0.0 )
//    return DP[ !forwards ][ i ];

  if( !check_delta_percent )
    return DP[ !forwards ][ i ];

  forwards = !forwards;

  if( rad[ n(i) ] != 0.0 )
    if( sign( rad[ i ] ) != sign( rad[ n(i) ] ))
      // S curve
      dp = s_curve_percent( i, 0.0, n(i) );

    else
      // c curve
      dp = c_curve_percent( i );
  else
    if( rad[ n(n(i)) ] == 0.0 )
      // 2 straights after segment i
      dp = 0.0;

    else
      if( sign( rad[ n(n(i)) ] ) != sign( rad[ i ] ))
	// kind of S curve.
	dp = s_curve_percent( i, len[ n(i) ], n(n(i)) );

      else
	// curve-straight-curve same side
	dp = C_curve_percent( i );

  forwards = !forwards;

  DP[ !forwards ][ i ] = dp;

  return ( dp );
}


static void init_fast_vars()
{
/*  int i;//,j;
  for( i=0; i<numseg+SEGS_AHEAD; i++)
  {
    fast_dp[ i ] = 0.0;
//    for( j=0; j<2; j++)
      fast_r [ i ] =
      fast_x [ i ] = 0.0;
  }
*/
  memset( DP[0],  0, 2*numseg*sizeof( double ) );
//  memset( fast_x,  0, (numseg+SEGS_AHEAD)*sizeof( double ) );
//  memset( fast_dp, 0, (numseg+SEGS_AHEAD)*sizeof( double ) );

  check_delta_percent = 1;
  for( int j=0; j<200; j++)
    for( int i=0; i<numseg; i++)
      delta_percent_prev( i ),
      delta_percent( i );
  check_delta_percent = 0;

}


static void allocate_arrays()
{
  int m;

  if( rad != NULL )
    return;

  numseg = get_track_description().NSEG;
  if( numseg * 0.75 < SEGS_AHEAD )
    SEGS_AHEAD = int( numseg * 0.75 );

  m = numseg
    * sizeof( double )
    * 4;

  if( m <= 4096 )
    rad = (double *)s.data_ptr;

  else
    rad = new double[m];

  len     = &(rad[numseg  ]);
//  fast_r  = &(rad[numseg*2]);
//  fast_x  = &(rad[numseg*3]);
  DP[0]   = &(rad[numseg*2]);
  DP[1]   = &(rad[numseg*3]);

  memset( rad, 0, numseg*2*sizeof( double ));

  set_rad_len();

  init_fast_vars();
}




static double speed( double r )
{ double sp;
  if( r == 0.0 )
    sp = cur_speed + 20.0;
  else
    sp = sqrt( fabs( r ) * 32.2 );

  return sp;
}

/*
static double delta_length( int i )
{
  double dl;

  if( rad[ i ] == 0.0 )
    return 0.0;

  if( sign( rad[ n(i) ] ) == sign( rad[ i ] ))
  {
    dl = length( n(i),
		 0,
		 0 );
    if( fabs( rad[ n(i) ] ) > fabs( rad[ i ] )
	&&
	dl > len[ i ]
      )
      dl = len[ i ];
  }
  else
    dl = short_straight( i ) * len[ i ];

  return dl;
}
*/


static double length( int i
//                    int check_prev_rad,
//                    int check_next_rad
		    ) //, int check_delta_percent )
{
  double l = len[ i ];

//  if( check_prev_rad
//      &&
//      i > 0
//      &&
//      sign( rad[ i-1 ] ) == sign( rad[ i ] )
//    )
//    l += length( i-1, 0, 0 );

//  if( //check_next_rad
//      //&&
//      ( i >= s.seg_ID
//      ?( i          < s.seg_ID + SEGS_AHEAD )
//      :( i + numseg < s.seg_ID + SEGS_AHEAD )
//      )
//    )
//  if( ( i <= s.seg_ID
//      ?( i          > s.seg_ID - SEGS_AHEAD )
//      :( i - numseg > s.seg_ID - SEGS_AHEAD )
//      )
//    )

//    if( recursed < SEGS_AHEAD )
//    { ++recursed;
      l += len[ i ] * DP[forwards][i];
//      --recursed;
//    };

//    if( recursed < SEGS_AHEAD )
//    { ++recursed;
      l += len[ i ] * DP[!forwards][i];
//      --recursed;
//    };


//  if( l > 2*len[i] )
//    l = 2*len[i];

//  if( rad[ i ] != 0.0
//      &&
//      l > PI * 2
//    )
//    return PI * 2.0;

//  draw_trajectory( i );

  return l;
}
/*
static double reduced_width( int i )
{ double w = width;

//  w -= DIST_FROM_INSIDE; // * 2.0;
  if( w < 0.0 )
    return 0.0;

//  if( delta_percent( i ) != 0.0 )
//    return w;

//  if( too_close
//      &&
//      i == 0 )
//    w = real_width *  TOO_CLOSE_FACTOR ];
//    too_close = 0;
//  else
//  {
//    if( check_delta_percent
//        &&
//        short_straight( i ))
//      w -= short_straight( i );
//    else
//    if( rad[ i ] != 0.0
//        &&
//        length( i, 1, 1 ) > 2.35 )
//      w = real_width * 0.6;
//        * ( speed( rad[ i ]) < 80.0 ? 0.1 : 1.0 );
//  }

  return w < width ? w : width;
}
*/

static double radius( int i )
{
//  if( check_delta_percent
//      &&
//      fast_r[ i ] != 0.0
//    )
//    return fast_r[ i ];

  double r = fabs( rad[ i ] ),
	 w = width, //reduced_width( i ),
	 max_rad;

  if( r == 0.0 )
    return r;

  r += DIST_FROM_INSIDE;

  { double l = length( i );

//    if( l < 0.9 )
//      r += ( w - DIST_FROM_INSIDE )
//       / ( 0.1 );
//    else
      r += ( w - DIST_FROM_INSIDE )
         / ( 1.0 - cos( l * 0.5 ));
  }

  max_rad = max_speed * max_speed * 1.1 * 1.1 // 10% more
	  / 32.2;
  if(
//      s.laps_to_go < Total_Laps
//      &&
      fabs( r ) > max_rad
    )
//    if( !first_lap
//        &&
//      i <= 0
//        &&
//        sign( rad[ n(i) ] ) == sign( rad[ i ] )
//        &&
//        sign( rad[ n(n(i)) ] ) != sign( rad[ i ] )
//      )
//      r = 0.0; //max_rad;
//
//    else
    {
      r = max_rad;
      if( r < fabs( rad[ i ] ) + DIST_FROM_INSIDE )
	  r = fabs( rad[ i ] ) + DIST_FROM_INSIDE;

      return r * sign( rad[ i ] );
    }

  r *= sign( rad[ i ] );

//  if( check_delta_percent
//      &&
//      forwards
//      &&
//      recursed == 0
//    )
//    if( i == 18 )
//      r = r;
//
//    else
//      fast_r[ i ] = r;

  return r;
}

static double curve_distance( int i )
{
//  if( i == -1 )
//    return prev_x;

//  if( check_delta_percent
//      &&
//      fast_x[ i ] != 0.0
//    )
//    return fast_x[ i ];

  double y, l_bw = len[i] * delta_percent_prev( i );
  double r = fabs( rad[ i ] );

  if( r == 0.0 ) return 0.0;

  r += DIST_FROM_INSIDE;

  y = fabs( radius( i )) - r;
  y *= sin( length( i ) * 0.5 - l_bw );

//  if( check_delta_percent )
//    fast_x[ i ] = y;

  return y;
}


static double tangent_alpha( int segment,
			     double to_end,
			     double d,
//                           int check_last_d,
			     double &d_big,
			     double &R )
{
  double alpha, r, t, l, te, y, z, d2, alpha_t=0.0, l_bw;

     d -= DIST_FROM_INSIDE;
;
//     if( d < 0.0 )
//       d = 0.0;

     l    = length( segment );
     l_bw = len[ segment ] * delta_percent_prev( segment );
     te   = to_end
	  - len[ segment ]
	  - l_bw
	  + l;

     r = fabs( rad[ segment ] ) + DIST_FROM_INSIDE;
     R = fabs( radius( segment ));
//     if( te < l * 0.5
//       &&
//       min_radius( cur_speed ) > r
//       &&
//       min_radius( cur_speed ) < R )
//       R = ( R + min_radius( cur_speed )) * 0.5;

     y = ( R - r ) * cos( l * 0.5 /*- l_bw*/ )
       + ( r + d ) * cos( l /*- l_bw*/ - te );

     z = ( R - r ) * sin( l * 0.5 /*- l_bw*/ )
       + ( r + d ) * sin( l /*- l_bw*/ - te );

     t = hipotenusa( y, z );

     alpha = ( te - l /*+ l_bw*/ ) * sign( rad[ segment ] );

     if( y > 0.0 && z > 0.0 ) // first quarter
       alpha_t = asin( z / t ) * sign( rad[ segment ] );

     else
     if( y < 0.0 && z > 0.0 ) // second quarter
       alpha_t = acos( y / t ) * sign( rad[ segment ] );

     else
     if( y < 0.0 && z < 0.0 ) // third quarter
       alpha_t = acos( - y / t ) * sign( rad[ segment ] )
	      + PI * sign( rad[ segment ] );

     else
     if( y > 0.0 && z < 0.0 ) // fourth quarter
       alpha_t = acos( - y / t ) * sign( rad[ segment ] )
	      + PI * sign( rad[ segment ] );

     alpha += alpha_t;
     if( l /*- l_bw*/ < te && z < 0.0 )
       alpha -= PI * 2.0 * sign( rad[ segment ] );

     d_big = d2 = t - R;

//     if( check_last_d )
//     {
//       alpha += atan(( d2
//                     - (  D_TO_IDEAL ] != 0.0
//                         ?  D_TO_IDEAL ]
//                         : d2 ))
//                   *  CRV_RESISTANCE ] )
//            *  CRV_BIAS ]
//            * sign( rad[ segment ] );
//
//        D_TO_IDEAL ] = d2;
//     };

     if( d_big < 0.0 )
     { // too close to inner side
//       d_big = 0.0;

       if( te > l * 0.5 )
       { // getting closer to inner side
	 double dx,dy,m,x;

//         alpha *= d / ( d + fabs( R ) - t );
//         too_close = 1; //width *=  TOO_CLOSE_FACTOR ];
//         if( te < l * 0.5 )
//           alpha = 0.0;
	 dx = r - (r+d) * cos( te - l * 0.5 );
	 dy =     (r+d) * sin( te - l * 0.5 );
	 m  = dy/dx;

	 x = dx/2. + m*dy/2.;

	 R = x;
	 alpha = te - l * 0.5
	       - acos(( R - dx ) / R );

	 alpha *= sign( rad[ segment ] );

       }
     }
     else
//     alpha *= 1.0; //  ALPHA_FACTOR ];
       alpha += ( PI*0.5
		- asin( R / ( R + d2 ))
		)
	      * sign( rad[ segment ] );


//  if( te < l * 0.5
//      &&
//      alpha * sign( rad[ segment ] ) > 0.0
//      &&
//      sign( s.vn ) == sign( rad[ segment ] )
//    )
    //
//    alpha *= 0.2;

  if( fabs( alpha ) > PI * 0.5 )
    alpha = PI * 0.5 * sign( alpha );

  return alpha;
}



static double last_to_inner( int i )
{
  double lti,
	 h,
	 a,
	 m,
	 x,
	 R = fabs( radius( i )),
	 l = length( i ),
	 to_center;

  h = - ( R - ( fabs( rad[ i ] )
	      + DIST_FROM_INSIDE
	      )
	);

  a = len[ i ] - l * 0.5;

  if( fabs(cos(a)) > 0.01 )
  {
    m = tan( a );
//  k = 0;
//  b = 0;
//  d = R;
    // (x-h)^2+(y-k)^2=d^2
    // y=mx+b
    // x^2-2hx+h^2+m^2x^2=R^2
    // (1+m^2)x^2-2hx+h^2-R^2=0
    //
    x = ( 2*h + sqrt( 4*h*h - 4*(1+m*m)*(h*h-R*R) ))
      / ( 2*( 1 + m*m ));

    to_center = x / cos( a );
  }
  else
    to_center = cateto( R, h );

  lti = to_center
      - fabs( rad[ i ] );

  return lti;
};



static double caution_factor( int n )
{
//  if( s.laps_to_go == Total_Laps )
//    return  CAUTION ] + 4.0;

//  if( s.laps_to_go == Total_Laps )
//    n *= 2;


//  if( //s.position == 0
//      //||
//      request_pit( s ) )
//    n *= 2;

  if( s.damage > last_damage )
//      &&
//      !s.nearby[ i ].for_position )
    n *= 2;

//  if(( 30000.0 - s.damage )/s.laps_to_go < 30000.0 / Total_Laps )
//    return  CAUTION ] * 4.0;

  return 1.0 ;//+ CAUTION * n;
//       - s.laps_to_go / Total_Laps
//       + s.damage / 15000.0
	 ;
}

/*
static double d_caution_f( )
{
  return 1.0 + ( caution_factor() - 1.0 ) * 2.0;
}
*/

static double corn_spd( int i ) // returns maximum cornering speed, fps
{
  // compute the speed

  if( len[ i ] == 0.0 )
    return 0.0;
/*
  if( rad[ p(i) ] == 0.0
      &&
      sign( rad[ p(p(i)) ] ) == sign( rad[ i ] )
      &&
      delta_percent( p(p(i)) ) != 0.0
    )
  { double R,d_big, to_begin, to_inner, y;

    y = last_to_inner( p(p(i)) );
    y += fabs( rad[ i ] );

    to_begin = atan( len[ p(i) ]/y );

    to_inner =  hipotenusa( len[ p(i) ], y );
    to_inner -= fabs( rad[ i ] );

    tangent_alpha( i,
		   to_begin + len[ i ],
		   to_inner,
		   d_big,
		   R
		 );

    return speed( R );
  }
*/
  if( rad[ p(i) ] != 0.0
      &&
      sign( rad[ p(i) ] ) == sign( rad[ i ] )
      &&
      fabs( rad[i] ) * len[i] > fabs( rad[ p(i) ] )*len[ p(i) ]
    )
  { double R,d_big, to_begin, to_inner, y;

    y = last_to_inner( p(i) );
    y += fabs( rad[ i ] );

    to_begin = 0.0;

    to_inner =  y;
    to_inner -= fabs( rad[ i ] );

    tangent_alpha( i,
		   to_begin + len[ i ],
		   to_inner,
		   d_big,
		   R
		 );

    return speed( R );
  }

  return speed( radius( i ) );
}
/*
static double min_radius( double speed )
{
  return speed * speed / 32.2;
}
*/
// Calculates the critical distance necessary to bring a car from speed
// v0 to speed v1 when the braking acceleration is "a", ft per sec^2.
// Speeds are in fps.  ("a" should be negative)
/*
static double CritDist(double v0, double v1, double radius )
{
   double dv,t,x,a;

   a = radius == 0.0
       ?  BRAKE_ACCEL ]
       :  BRK_CRV_ACC ];
   dv = v1 - v0;
   t = dv/a;
   x = v0 * t + a*(t*t)/2.0;
   return x;
}
*/

static double ending_speed( double Vo, double x, double a, double &e_d )
{
  double t, Vf;

//  if( x < 0.0 )
//    return Vo;

  e_d = x;

  // Vf = Vo + at
  // if Vf = 0
  // t = - Vo/a

  //  x = Vo*t + a*t*t/2
  if( Vo*Vo < 4.0*a/2.0*(-x) )
  {
    t = - Vo/a;
    e_d = Vo*t + a*t*t/2;
    return 0.0;
  }

  t = (-Vo+sqrt(Vo*Vo-4.0*a/2.0*(-x))) / (2.0*a);
//t = (-Vo-sqrt(Vo*Vo-4.0*a/2.0*(-x))) / (2.0*a);
  Vf = Vo + a*t;

  return Vf;
}




static void calculate_max_a( )
{
  double a;

  if( fabs( cur_x - last_x ) < 0.1 )
    return;

  a = ( cur_speed*cur_speed - last_speed*last_speed )
    / ( 2.0 * ( cur_x - last_x ));

  if( s.vc < last_speed - 4.0
//      &&
//      rad[ s.seg_ID ] == 0.0
//      &&
//      outside_lane
//      &&
//      fabs( s.alpha ) < 0.01
//      &&
//      s.to_end > curve_distance( 1 )
      &&
      fabs( a ) < 40.0
      &&
      s.to_rgt > DIST_FROM_INSIDE
      &&
      s.to_lft > DIST_FROM_INSIDE
    )
  { //if( fabs( a ) > max_a )
      avg_a += fabs( a ),
      avg_a *= 0.5;

//    if( fabs( a ) < min_a )
//      min_a = fabs( a );
      if( fabs( s.alpha ) < 0.01 )
      { BRAKE_ACCEL -= fabs( a );
	BRAKE_ACCEL *= 0.5;
        if(BRAKE_ACCEL>-30.0)
          BRAKE_ACCEL=-30.0;
      }
      else
      { BRK_CRV_ACC -= fabs( a );
	BRK_CRV_ACC *= 0.5;
        if(BRK_CRV_ACC>-25.0)
          BRK_CRV_ACC=-25.0;
      }
  }

  else
  { static int count=0;

    if( ++count == 4 )
    { count = 0;
      if(  BRAKE_ACCEL < -40.0 )
	BRAKE_ACCEL -= 40.0,
	BRAKE_ACCEL *= 0.5;

      if(  BRK_CRV_ACC < -35.0 )
	BRK_CRV_ACC -= 35.0,
	BRK_CRV_ACC *= 0.5;
    }
  }

};


static int consider_nearby( int i,
			    double max_dot,
			    int n,
			    double min_vsqr,
			    double max_time,
			    double x_factor,
			    double y_factor
			  )
{
  double y,x,vy,vx,dot,vsqr,c_time,x_close,y_close,theta,acc=0.0;
  y=s.nearby[i].rel_y;         // get forward distance (center-to-center)
  x=s.nearby[i].rel_x;         // get right distance
  vx=s.nearby[i].rel_xdot;     // get forward relative speed
  vy=s.nearby[i].rel_ydot;     // get lateral relative speed

	min_vsqr < 1.0 ? min_vsqr = 1.0 : 0;

  // if behind then ignore it.
//  if( y < 0 )
//	  return 0;

	if( s.damage > last_damage
		  &&
		  y > 0
		  &&
			y < 2*CARLEN
			&&
			x < 2*CARWID
		)
		return 1;

  // if the cars are getting closer, then the dot product of the relative
  // position and velocity vectors will be negative.
  dot = x * vx + y * vy;     // compute dot product of vectors
  if(dot > max_dot)            // no action if car is not approaching (fast).
    return 0;

//  if( x > CARWID * caution_factor( n )
//	  ||
//	  y > CARLEN * caution_factor( n )
//	)
  {
	vsqr = vx*vx + vy*vy;      // compute relative speed squared
	if( vsqr < min_vsqr )
	  return 0;

	// Time to closest approach is dot product divided by speed squared:
	c_time = -dot / vsqr;     // compute time to closest approach
	if(c_time > max_time)         // ignore if over three seconds
	   return 0;

	// Brake if too much danger
	{ double sq = sqrt(vsqr),
		      msq = sqrt(min_vsqr);
	  if( ( sq > 20
			    ||
				  sq > 10 + msq 
				)
			  &&
				( s.nearby[i].to_rgt > -CARWID*2.0
				  &&
				  s.nearby[i].to_lft > -CARWID*2.0
				  ||
					sign( s.nearby[i].vn ) == sign( s.nearby[i].to_lft )
				)
			)
		  return 1;
	}

	/* If the execution gets this far, it means that there is a car
	ahead of you, and getting closer, and less than 3.0 seconds
	away.  Evaluate the situation more carefully to decide if
	evasive action is warranted: */
	if( s.nearby[i].braking )
		acc = -BRAKE_ACCEL;      // for at^2/2
	x_close = x + c_time * vx;// + c_time*c_time*acc*vx/(2*vx);  // x coord at closest approach
	y_close = y + c_time * vy;// + c_time*c_time*acc*vy/(2*vy);      // y coord at closest approach
	/*  Due to the length of the cars, a collision will occur if
	    x changes sign while y is less than CARLEN.  This
	    can happen before the center-to-center distance reaches its
	    point of closest approach. */
	// check if collision would occur prior to closest approach
	// if so, reduce c_time, re-calculate x_close and y_close:
	if(x_close * x < 0.0 && y < CARLEN * caution_factor( 1 )) {
	   if( fabs( vx ) < 0.1 )
	     return 0;
	   c_time = (fabs(x) - CARWID * caution_factor( 1 )) / fabs(vx);
	   x_close = x + c_time * vx;      // x coord at closest approach
	   y_close = y + c_time * vy;      // y coord at closest approach
	}
	// Will it be a hit or a miss?
	theta = s.nearby[i].alpha; 
		  //- s.alpha;
    if( !collide( x_close, y_close, theta ))
		return 0;
/*
	if(fabs(x_close) > x_factor * CARWID * caution_factor( 1 )
	   ||
	   fabs(y_close) > y_factor * CARLEN * caution_factor( 1 ))
	   return 0;            // this when a miss is predicted
*/
  }

  return 1;
}

static int closest()
{ int   min_i=-1;
  double min_d=sqrt(2*2*CARLEN*CARLEN+2*2*CARWID*CARWID),d;
  for( int i=0; i<16; i++ )
		if( s.nearby[i].who >= 0 
			  && 
				s.nearby[i].who < 16
			)
		{
			d=sqrt( s.nearby[i].rel_x * s.nearby[i].rel_x 
				    + s.nearby[i].rel_y * s.nearby[i].rel_y
						);
			if( d < min_d )
			{
				min_d = d;
				min_i = i;
			}
		}	
		else
			break;
	return min_i;
}

static int crash_in_front()
{ int min_i = closest();
	return min_i==-1 ? 0: ( s.nearby[min_i].rel_y > 0.0 );
}


static int apply_brakes()
{
  int i,result=0;
//	static int init=0;
  static float v[16]={1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f};
  static float d[16]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
//	con_vec r;
/*
 	if( s.laps_to_go == Total_Laps )
  	for( i=0; i<16; i++)
			d[i]=5.0;
	else
		if( !init )
		{	init = 1;
			for( i=0; i<16; i++)
		    d[i]=1.0;
		};
*/
	if( s.damage > last_damage )
	{
		lap_damage += s.damage - last_damage;
//		if( s.v < last_speed )
//		  return 1;
		if( crash_in_front() )
//		  for( i=0; i<16; i++)
    { i = closest();
			(v[i] -= 0.5f) < 1.0f ? v[i] = 1.0f : 0;
		}
  }

  for( i=0; i<NEARBY_CARS; i++ )
  {
/*
		r.request_pit = 0;
	  request_pit( r );
	  if( r.request_pit 
		    ||
		  	lap_damage > ( 15000.0 - s.damage ) / 5.0//max_lap_damage
		  ) 
		{  
		  for( i=0; i<16; i++)
			  v[i] = 1.0f;
//			if( s.damage > last_damage )
//				return 1;
		}
*/
    if( s.nearby[ i ].who >= 16 
			  ||
				s.nearby[ i ].who < 0 
			)
      break;

//    if( !request_pit( s ) && s.laps_to_go > 1 )
//      break;

    if( s.nearby[ i ].rel_y < 1.0*CARLEN
        ||
        !consider_nearby( i, -0.1, 1, 
				                  pow(1.01*s.nearby[i].rel_y/CARLEN,v[s.nearby[i].who]),//v[s.nearby[i].who]*v[s.nearby[i].who], 
													3, 1, 1 )
      )
      continue;
		

    // If we get here there is a collision predicted
//    if( //s.damage < 10000
				//&&
//				d[s.nearby[i].who] < 5.0 
				// there are no more cars nearby
//				&&
//				i==0 && s.nearby[i+1].who >= 16 
//			)
      d[s.nearby[i].who] += 0.1f;
//		else
			v[s.nearby[i].who] += 0.1f;

    result |= 1;

  };

  if( s.lap_flag )
	{ lap_damage = 0.0;
    for( i=0; i<16; i++)
		{
//			if( d[i] > 1.0 )
//			{
//			  v[i] += d[i];
//  			if( v[i] > 20 ) v[i] = 20;
//  			d[i] = 0.0f;
//			}
//			else
//  	    if( v[i] > 2.0 )
			if( d[i] == 0.0f )
			{	v[i] *= 0.5f; //-= 0.001;
	      if( v[i] < 1.0f )
				  v[i] = 1.0f;
			};
  	  d[i] = 0.0f;
		}
	}

//  if( !result )
//	  for( i=0; i<16; i++)
//	    if( (v[i] -= 0.01f) < 5.0f )
//   	    v[i] = 5.0f;

  return result;
}

/*
static int update_first_lap( )
{
  return first_lap = s.laps_to_go == Total_Laps;
}
*/
/*
static int start_caution( )
{
  static int last_seg = 0;

  last_seg  |= s.seg_ID == 0 && !first_segment();

  return !last_seg
	 //&&
	 //s.laps_to_go == Total_Laps
	 //&&
	 //s.seg_ID == 0
	 &&
	 !no_crash
	 &&
	 s.stage != PRACTICE
	 &&
	 s.position != 0
	 ;
}
*/
static int final_straight( )
{
  static int next_seg = 0;

  next_seg &= s.laps_to_go != Total_Laps;

  next_seg |= s.seg_ID != 0 && s.laps_to_go == 1;

  return next_seg
	 &&
	 s.seg_ID == 0
	 ;
}

/*
static void modify_width( )
{
  int i, d_modified = 0, w_modified = 0;
  static int outside_strategy = 99,
	     inside_strategy  = 99;
  double d, D;

//  if( curve_distance( 1, 1 ) < s.to_end * ( rad[ s.seg_ID ] == 0.0 ? 1.0 : rad[ s.seg_ID ] ))
//    return;

  for( i=0; i<NEARBY_CARS; i++ )
  {
    if( s.nearby[ i ].who >= 16 )
      break;

    if( rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] < 0.0 ) // S curve
      break;

    if( !request_pit() && s.laps_to_go > 1 )
      break;

//    do{
//      if( s.dead_ahead
//          &&
//          s.nearby[ i ].braking )
//        break;

//      if( sign( s.vn - s.nearby[i].vn ) == sign( s.nearby[ i ].rel_x ))
//        goto continue_for;

//      if(( s.v - s.nearby[ i ].v ) * sign( s.nearby[ i ].rel_y ) <= 0.0 )
//        continue;

//      if( s.nearby[i].rel_y < 1.0 )
//        continue;

    if( !consider_nearby( i, -0.1, 2, 0.1, 5, 999, 3.3 ))
      continue;

    // If we get here there is a PASSING predicted

    // moved here to do it only if cars nearby.
     if( start_caution())
     { double dfi = DIST_FROM_INSIDE,
		w = width;
//     if( rad[ s.seg_ID ] > 0.0
//         ||
//         rad[ s.seg_ID ] == 0.0
//         &&
//         rad[ n(s.seg_ID) ] > 0.0 )
       if( DIST_FROM_INSIDE < 0.30 * real_width )
	 DIST_FROM_INSIDE += rad[ s.seg_ID ] == 0.0
					 ? real_width / 100.0
					 : real_width / 250.0;

       if( width > 0.6 * real_width )
	 width = 0.6 * real_width; //s.to_lft;
//     else
//       DIST_FROM_INSIDE =
//       width = s.to_rgt;

       if( dfi != DIST_FROM_INSIDE
	   ||
	   w != width
	 )
	 init_fast_vars();

     }


//      if( s.nearby[ i ].rel_y > 0.0
//          &&
//          !apply_brakes( s, i ))
//        goto continue_for;

//      if(( s.nearby[i].v - s.v )
//         * ( s.nearby[i].v - s.v )//s.nearby[0].rel_ydot * s.nearby[0].rel_ydot //
//         <
//         - 2.0
//         * BRK_CRV_ACC]
//         * ( fabs( s.nearby[i].rel_y )
//         - CARLEN * caution_factor( s ))
//        )
//        goto continue_for;

//    }while( 0 );


   if( rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] < 0.0 // S curve
	&&
	fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] ) // big then small
	&&
	rad[ s.seg_ID ] < 0.0

	||
	rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] >= 0.0 // Not an S curve
	&&
	( rad[ s.seg_ID ] > 0.0
	  ||
	  rad[ s.seg_ID ] == 0.0
	  &&
	  rad[ n(s.seg_ID) ] > 0.0
	)
      )
      d = s.nearby[i].to_lft,
      D = s.to_lft;

    else
      d = s.nearby[i].to_rgt,
      D = s.to_rgt;


//    if( w > real_width * 0.33
//        &&
//        w < real_width * 0.66
//      )
//      continue;
/ *
    if( w < CARWID * 1.0 )
      // he is too close to the inner side
      if( s.nearby[i].rel_y > 0.0 //he is ahead of me
	  &&
	  DIST_FROM_INSIDE < real_width * 0.33
	  &&
	  DIST_FROM_INSIDE < w + CARWID * 4.0 )
	  // distance from inside has to be grater
	DIST_FROM_INSIDE +=
	  rad[ s.seg_ID ] == 0.0 ? 1.0 : 0.2, //w + CARWID * 4.0,

	d_modified = 1,
//          width = real_width
//                -  WIDTH_FACTOR ],
////                - DIST_FROM_INSIDE,
	init_fast_vars();
      else;
    else
* /
//      if( s.nearby[i].rel_y < CARLEN * 1.25 )
	if( outside_strategy == s.nearby[ i ].who
	    ||
	    (
	      d < CARWID * 1.5
	      ||
	      s.nearby[i].rel_y > 0.0
	      &&
	      (
//              rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] < 0.0 // S curve
//              &&
//              sign( s.nearby[i].to_lft - s.to_lft ) != sign( rad[ s.seg_ID ] )

//              ||
		rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] >= 0.0 // Not an S curve
		&&
		sign( s.nearby[i].rel_x ) != sign( rad[ s.seg_ID ] )
		&&
		sign( s.nearby[i].rel_x ) != sign( rad[ n(s.seg_ID) ] )
	      )
	      &&
	      d + CARWID * caution_factor( 2 ) < real_width * 0.5
	    )
	  )
	  if( 0
	      &&
	      !d_modified
	      &&
	      ( rad[ s.seg_ID ] == 0.0
//              ||
//              sign( rad[ s.seg_ID ] ) == sign( s.nearby[i].vn )
	      )
//              &&
//              DIST_FROM_INSIDE < real_width * 0.33
//              &&
//              d < real_width * 0.33
	      &&
	      DIST_FROM_INSIDE < d + CARWID * caution_factor( 2 )
//              &&
//              !d_modified
	      &&
	      D > d //+ CARWID * d_caution_f( s )
	    )
	  {
	    outside_strategy = s.nearby[ i ].who;

	    DIST_FROM_INSIDE //= d + CARWID * 2.0;
					 += rad[ s.seg_ID ] == 0.0
					    ? real_width / 100.0
					    : real_width / 250.0;

	    d_modified = 1;
//            width = real_width
//                  -  WIDTH_FACTOR ],
////                  - DIST_FROM_INSIDE;
	    init_fast_vars();
//            break;
	  }
	  else;

	else
	  if( inside_strategy == s.nearby[ i ].who
	      ||
	      ( 0
		&&
		rad[ s.seg_ID ] == 0.0
		&&
		( s.nearby[i].rel_y < 0.0
		  ||
		  sign( s.nearby[i].rel_x ) == sign( rad[ n(s.seg_ID) ] )
		)
//                ||
//                rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] < 0.0 // S curve
//                &&
//                sign( s.nearby[i].to_lft - s.to_lft ) != sign( rad[ n(s.seg_ID) ] )
//                &&
//                d < real_width * 0.33
//                &&
//                rad[ s.seg_ID ] == 0.0
	      )
	    )
	  { double w = width;
	    inside_strategy = s.nearby[ i ].who;
//            width_reduced =
	    w_modified = 1;

	    if( rad[ s.seg_ID ] * rad[ n(s.seg_ID) ] < 0.0 )// S curve
	      if( width > real_width - d - CARWID * caution_factor( 2 ))
		width += real_width - d - CARWID * caution_factor( 2 ),
		width *= 0.5;
	      else;
	    else
	    if( width > d - CARWID * caution_factor( 2 ))
	      width = d - CARWID * caution_factor( 2 ); // -= rad[ s.seg_ID ] == 0.0 ? 1.0 : 0.2 ;

	    if( w != width )
	      init_fast_vars();
	  }
//          else;
//      else
//        if( w < width )
//          width_reduced = 1,
//          width = w,
//          init_fast_vars();


	       //  *=  PASS_FACTOR ];
	       // -= CARWID * 2.0;

//    if( s.dead_ahead )
//      break;

    continue_for:
  }
  if( width < CARWID * caution_factor( 2 ))
    width = CARWID * caution_factor( 2 );

  if( !d_modified
      &&
      DIST_FROM_INSIDE > 1.0
    )
  { double dfi = DIST_FROM_INSIDE;
    DIST_FROM_INSIDE //= 1.5;
				 -= rad[ s.seg_ID ] == 0.0
				    ? real_width / 150.0
				    : real_width / 250.0;
    if( DIST_FROM_INSIDE < 1.0 )
      DIST_FROM_INSIDE = 1.0;

    if( dfi != DIST_FROM_INSIDE )
      init_fast_vars();
  }

  if( !w_modified )
  { double w = width;
    width //= real_width -  WIDTH_FACTOR ];
	  += rad[ s.seg_ID ] == 0.0
			      ?( real_width / 150.0 )
			      :( real_width / 150.0 );

    if( width > real_width -  WIDTH_FACTOR )
      width = real_width -  WIDTH_FACTOR;

    if( w != width )
      init_fast_vars();
  }
  if( !d_modified
      &&
      !w_modified
      &&
      segment_change
    )
    outside_strategy =
    inside_strategy  = 99;


  if( DIST_FROM_INSIDE > width )
    DIST_FROM_INSIDE = width;

}
*/

static double t_distance( int i )
{
  double R = fabs( radius( i )),
	 r = fabs( rad[ i ] ) + DIST_FROM_INSIDE,
	 l = len[ i ] * 0.5,
	 c = cos( l ),
	 m,
	 x1,x2,
	 t;

  if( rad[ i ] == 0.0 )
    return len[ i ];

  if( fabs( c ) < 0.1 )
  {
//    x1 = x2 = 0,
//    Rcos(t)=R-r ::
    t = acos((R-r)/R);
    return R*t*2;
  }

  m = tan( l );
//    (r - R - x)^2 + ( 0 - m*x )^2 = R^2
//    (r-R)^2 - 2(r-R)x + x^2 + m^2x^2 = R^2
//    m^2x^2 - 2(r-R)x + (r-R)^2 - R^2 = 0
//    m^2x^2 - 2(r-R)x + r^2 - 2rR = 0

  x1 = (2*(r-R) + sqrt(4*(r-R)*(r-R) - 4*(m*m)*(r*r-2*r*R) ))
     / (2*m*m);
  x2 = (2*(r-R) - sqrt(4*(r-R)*(r-R) - 4*(m*m)*(r*r-2*r*R) ))
     / (2*m*m);

  if( len[i] < PI )
    t = atan( m*x1 / ( x1+R-r ));
  else
    t = PI - fabs(atan( m*x2 / ( x2+R-r ))),
    t = fabs( t );

  return R*t*2.0;
}


static double brake_accel( int i )
{
  if( rad[ i ] != 0.0 )
    return BRK_CRV_ACC;

  if( len[ i ] < curve_distance( n(i) ) + curve_distance( p(i) ))
    return BRK_CRV_ACC;

  return BRK_CRV_ACC *    ( curve_distance( n(i) ) + curve_distance( p(i) )) / len[ i ]
       + BRAKE_ACCEL * (1-( curve_distance( n(i) ) + curve_distance( p(i) )) / len[ i ]);
}

con_vec JOCOSA83( situation &ss )
{
   const char name[] = "JOCOSA83";    // This is the robot driver's name!
   static int init_flag = 1;          // cleared by first call
   con_vec result;                    // This is what is returned.
   double alpha, vc;                  // components of result
   double r, d, t, q,
	  alpha_n,
	  alpha_nn,
	  alpha_nnn,
	  x, y,
//        x1, y1, xr, yr, m1, m2 ,b1, b2,
	  _speed = 0.,
	  n_speed = 0.,
	  nn_speed = 0.,
	  nnn_speed = 0.,
	  to_begin,
	  to_inner,
	  d_big,
	  R,
	  e_d;

   static
   int //i,
       last_segment   = 0,
       slip_corrected = 0,
//       modified_seg   = 0,
       following_next = 0;

   int //emergency_override = 0,
       use_alpha_factor = 1,
       slip_correction = 1;

   s = ss;

   if(init_flag == 1)  {  // first time only, copy name
      my_name_is(name);        // copy the name string into the host program
      init_flag          =
      result.request_pit = 0;
      result.alpha       =
      result.vc          =
      s.v                =
      s.out_pits         =
      ss.out_pits        = 0;
      return result;
   }

   real_width = s.to_lft + s.to_rgt;

   if( width == 0.0 )
     width = real_width -  WIDTH_FACTOR;

   allocate_arrays();

   if( rad == NULL )
   {  result.request_pit = 0;
      result.alpha       =
      result.vc          = 0.0;
      return result;
   }


   if( Total_Laps == 0 )
   { Total_Laps = s.laps_to_go;
     max_lap_damage = 30000 / ( s.laps_to_go + 1 );
	 }

//   update_first_lap();
   curveing = 0;

   request_pit( result );
	 if( result.request_pit )
	   max_lap_damage = 30000 / ( s.laps_to_go + 1 );


   // service routine in the host software to handle getting unstuck from
   // from crashes and pileups:
//   if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
//     return result;

/*
   if( kb_hit() )
   {
     int c = get_ch();
     switch( c ){
     case RIGHT: ++i>19?i=19:i; break;
     case LEFT : --i< 0?i= 0:i; break;
     case '+'  : i] += delta[i]; break;
     case '-'  : i] -= delta[i]; break;
     default   : ;
     };
     if( !no_display )
     {
       char s[ 20 ];
       set_fill_color( FIELD_COLOR );
       rectangle( 0, 0, 1000,  3*CHR_HGT );
       text_output( 100, 2*CHR_HGT, names[ i ] );
       sprintf( s, " %5f ",  i ] );
       text_output( 100, CHR_HGT, s );
     }
   }
*/
   cur_speed = s.v;
   cur_x     = s.to_end*( rad[ s.seg_ID ] == 0.0
			  ? 1.0
			  : fabs( rad[ s.seg_ID ] )
			);

   if( max_speed < s.v )
     max_speed = s.v;
//     init_fast_vars();

   use_alpha_factor = 1;

//   if( numseg > MAXSEG )
//     return result;

//   set_rad_len();

//   if( rad[ s.seg_ID ] != 0.0 && radius( s.seg_ID ) == 0.0 )
//     len[ s.seg_ID ] *= fabs( rad[ s.seg_ID ] ),
//     s.to_end *= fabs( rad[ s.seg_ID ] ),
//     rad[ s.seg_ID ] = 0.0;
//   if( rad[ n(s.seg_ID) ] != 0.0 && radius( n(s.seg_ID) ) == 0.0 )
//     len[ n(s.seg_ID) ] *= fabs( rad[ n(s.seg_ID) ] ),
//     rad[ n(s.seg_ID) ] = 0.0;

//   if( rad[ n(n(s.seg_ID)) ] != 0.0 && radius( n(n(s.seg_ID)) ) == 0.0 ) len[ n(n(s.seg_ID)) ] *= fabs( rad[ n(n(s.seg_ID)) ] ),                               rad[ n(n(s.seg_ID)) ] = 0.0;
//   if( rad[ s.seg_ID ] == 0.0 )
//     DIST_FROM_INSIDE = 1.0;


//   DAMAGE = s.damage;
   if( s.damage > last_damage )
     y = 0.0;

//    WIDTH ]  = width;
//   if( ! width_reduced )
//   {
//     width += rad[ s.seg_ID ] == 0.0 ? 1.0 : 0.4 ;
//     if( width > s.to_lft + s.to_rgt -  WIDTH_FACTOR ] )
//       width = s.to_lft + s.to_rgt,   // compute width of track
//       width -=  WIDTH_FACTOR ];

//     init_fast_vars();
//   }
//   if( s.dead_ahead )
//     width *=  PASS_FACTOR ];

//   if( s.seg_ID ==  SEGMENT_BREAK ] )
//     q = 0.0;

   if( s.seg_ID != last_segment )
   {
     segment_change = 1;
     last_segment = s.seg_ID;
//   D_TO_IDEAL = 0.0;
     width_reduced = 0;
/*     if( last_rad * rad[ s.seg_ID ] < 0.0 ) // it was an S curve
     {
       double temp = width;
       width = real_width - DIST_FROM_INSIDE; // rad[ s.seg_ID ] > 0.0 ? s.to_lft : s.to_rgt ;
       if( width > real_width -  WIDTH_FACTOR )
	 width = real_width -  WIDTH_FACTOR;

       DIST_FROM_INSIDE = real_width - temp;
       if( DIST_FROM_INSIDE > real_width * 0.66 )
	 DIST_FROM_INSIDE = real_width * 0.66;

       width_reduced = 1;
     }
*/
//	 last_damage = s.damage;
     last_rad    = rad[ s.seg_ID ];
     last_x      = s.cur_len * ( rad[ s.seg_ID ]==0.0
				 ? 1.0
				 : fabs(rad[ s.seg_ID ])
			       );
     cur_x       = 0.0;
//     init_fast_vars();
//     { static double x_0=0.0;
//       prev_x      = x_0;
//       x_0         = curve_distance( s.seg_ID );
//     }
     following_next = 0;
/*
     if( no_crash )
     {
       draw_trajectory( s.seg_ID );

       cout << '\r' << DP[0][s.seg_ID] << ' ' << DP[1][s.seg_ID] << "        ";
       getch();
     }
*/
   }
   else
     segment_change = 0;

   calculate_max_a();

//   trace_trajectories();

//   for(int j=0; j<5; j++)
//     rad_big[ j ] = radius( j, 0 ),
//     x_big[ j ]   = curve_distance( j, 0 );

//   modify_width();

   if( rad[ s.seg_ID ] == 0.0 )
   {
     // Straight
/*
     { int i;
       for( i=0; i<NEARBY_CARS; i++)
       {
         if( s.nearby[ i ].who >= 16 )
           break;

         if( s.nearby[ i ].rel_y >  CARLEN * 1.5 )
           continue;

         if( s.nearby[ i ].rel_y < -CARLEN * 1.5 )
           continue;

         if( fast_dpp[ n( s.seg_ID ) ] == 0.0 )
         { modified_seg = n( s.seg_ID );
           fast_dpp[ n( s.seg_ID ) ] = 1.0;
         };
         break;

       };

       if( i == NEARBY_CARS || s.nearby[ i ].who >= 16 )
         if( modified_seg != 0 )
         { fast_dpp[ modified_seg ] = 0.0;
           modified_seg = 0;
           i++;
         };
     }
*/
       x = s.to_end;

       y = rad[ n(s.seg_ID) ] < 0.0 ? s.to_rgt : s.to_lft;
       y += fabs( rad[ n(s.seg_ID) ] );

       to_begin = atan( x/y );

       to_inner =  hipotenusa( x, y );
       to_inner -= fabs( rad[ n(s.seg_ID) ] );

       alpha   =
       alpha_n =
	       - to_begin * sign( rad[ n(s.seg_ID) ] )
	       + tangent_alpha( n(s.seg_ID),
				to_begin + len[ n(s.seg_ID) ],
				to_inner,
				d_big,
				R
			      );

       if( s.to_end > len[ s.seg_ID ] - curve_distance( p(s.seg_ID) ) )
	 curveing = 1;


       outside_lane = ( d_big > 0.0 );
       if( outside_lane ) //t >= r )
       {
//       alpha += //atan( sqrt(( 2.0 * t + d ) * d ) / t ) * sign( s.nex_rad );
//                ( PI*0.5 - asin( r / t ))
//              * sign( s.nex_rad );
	 if( ending_speed( s.v,
			   cateto( R+d_big, R ),//t*t - r*r ),
			   brake_accel( s.seg_ID ),
			   e_d )
	     >
	     corn_spd( n(s.seg_ID) )
	   )
	 {
	   _speed = corn_spd( n(s.seg_ID) );
//           if( sign( s.vn ) == sign( rad[ n(s.seg_ID) ] ))
//             alpha = 0.0;//asin( s.vn/s.v );
	 };
//	 else
//	   _speed = s.v + 20.0;

	 if( sign( alpha ) == sign( rad[ n(s.seg_ID) ] )
//           ||
//           d_big == 0.0
	   ) //s.to_end < y )
	 {
	   curveing = 1;

//         alpha += atan(  ACCUM_D ] )
//                *  ACCUM_FACTOR ]
//                * sign( rad[ n(s.seg_ID) ] );
//          D_MINUS_LAST_D ] = d
//                                      - (  D_TO_IDEAL ] != 0.0
//                                          ?  D_TO_IDEAL ]
//                                          : d );

//         alpha += atan(  D_MINUS_LAST_D ]
//                      *  CRV_RESISTANCE ] )
//                *  CRV_BIAS ]
//                * sign( rad[ n(s.seg_ID) ] );

//         alpha *= 1.0; // ALPHA_FACTOR ];

//          D_TO_IDEAL ] = d;
//          ACCUM_D ]   += d;

	 }
	 else
	   slip_correction = 1;//1;

//       use_alpha_factor = 0;

       }
       else // t < r
//       if( rad[ n(s.seg_ID) ] != 0.0 )
       {
	 // too close to inner side
//          TOO_CLOSE_FACTOR ] = -d;
	 curveing = 1;
/*
	 y1  = rad[ n(s.seg_ID) ] > 0.0 ? s.to_lft : s.to_rgt;
	 y1 += fabs( rad[ n(s.seg_ID) ] );
	 y1 -= ( fabs( rad[ n(s.seg_ID) ] ) + DIST_FROM_INSIDE )
	     * cos( length( 1, 1, 1 ) * 0.5 );
	 y1 *= 0.5;
	 x1  = s.to_end;
	 x1 += ( fabs( rad[ n(s.seg_ID) ] ) + DIST_FROM_INSIDE )
	     * sin( length( 1, 1, 1 ) * 0.5 );
	 x1 *= 0.5;

	 alpha = atan( y1 / x1 );

	 m1 = - 1 / tan( alpha );
	 m2 =
	    - cos( length( 1, 1, 1 ) * 0.5 )
	    / sin( length( 1, 1, 1 ) * 0.5 );
	 b1 = y1 - m1 * x1;
	 b2 = y1 * 2.0 - m2 * x1 * 2.0;

	 xr = ( b2 - b1 ) / ( m1 - m2 );
	 yr = m1 * xr + b1;

	 alpha = - atan( xr / yr ) * sign( rad[ n(s.seg_ID) ] );
*/
//       if( s.v > speed( R ))
	 _speed = speed( R ); //hipotenusa( xr, yr ));
//	 if( sign( s.vn ) != sign( rad[ n(s.seg_ID) ] ))
//	   alpha *= 0.5;

//       else
//         if( sign( alpha ) != sign( s.vn ) )
//           alpha = 0.0; //asin( s.vn/s.v );

//         width *=  TOO_CLOSE_FACTOR ];
       }

//       if( alpha * sign( rad[ n(s.seg_ID) ] )
//           >
//           len[ n(s.seg_ID) ]
//         )
//	 alpha = len[ n(s.seg_ID) ] * sign( rad[ n(s.seg_ID) ] );



       // calculate alpha_nn
       if( rad[ n(n(s.seg_ID)) ] == 0.0 )
         // Straight Curve Straight
         if( sign( rad[ n(n(n(s.seg_ID)))] ) == sign( rad[ n(s.seg_ID) ] ))
         {
           // Straight & C curve
  	   d = fabs( rad[     n(s.seg_ID)   ] )
	     - fabs( rad[ n(n(n(s.seg_ID))) ] );

	   x = s.to_end
             + d*sin( len[ n(s.seg_ID) ] )
             + len[ n(n(s.seg_ID)) ]*cos( len[ n(s.seg_ID) ] );

	   y = rad[ n(s.seg_ID) ] > 0.0 ? s.to_lft : s.to_rgt;
	   y += fabs( rad[ n(s.seg_ID) ] );
	   y -= d*cos( len[ n(s.seg_ID) ] );
           y += len[ n(n(s.seg_ID)) ]*sin( len[ n(s.seg_ID) ] );

	   t = hipotenusa( x, y );

	   to_begin = atan( x/y )
		    + len[ n(s.seg_ID) ];

 	   to_inner = t - fabs( rad[ n(n(n(s.seg_ID))) ] );

	   alpha_nnn = len[ n(s.seg_ID) ] * sign( rad[ n(s.seg_ID) ] )
		     - to_begin           * sign( rad[ n(s.seg_ID) ] )

		    + tangent_alpha( n(n(n(s.seg_ID))),
				     to_begin + len[ n(n(n(s.seg_ID))) ],
				     to_inner,
				     d_big,
				     R
				   );

           if( d_big <= 0.0
               &&
               fabs( radius( n(n(n(s.seg_ID))) ))
               <
               fabs( radius( n(s.seg_ID) ))
             )
           {
//             alpha    =
//             alpha_n  =
//             alpha_nn = alpha_nnn;

             nnn_speed = speed( R );
           };
         }
         else
           ;
       else
       if( sign( rad[ n(n(s.seg_ID)) ] ) != sign( rad[ n(s.seg_ID) ] )
	   &&
	   len[ n(s.seg_ID) ] < PI*0.5
	 )
       { // Straight and S curve
	 x = s.to_end
	   + ( fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] ))
	     * sin( len[ n(s.seg_ID) ] );

	 y -= ( fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] ))
	      * cos( len[ n(s.seg_ID) ] );

	 to_begin = PI*0.5
		  + atan( y/x )
		  - len[ n(s.seg_ID) ]
		    ;
	 to_inner =  hipotenusa( x, y );
	 to_inner -= fabs( rad[ n(n(s.seg_ID)) ] );

//       { double dfi = DIST_FROM_INSIDE,
//                w = width;
//         DIST_FROM_INSIDE = real_width - w;
//         width            = real_width - dfi;
//         if( width > real_width -  WIDTH_FACTOR )
//           width = real_width -  WIDTH_FACTOR;

	   alpha_nn =
		      PI*0.5      * sign( rad[ n(s.seg_ID) ] )
		    + atan( y/x ) * sign( rad[ n(s.seg_ID) ] )
		    + tangent_alpha( n(n(s.seg_ID)),
				     to_begin + len[ n(n(s.seg_ID)) ],
				     to_inner,
				     d_big,
				     R
				   );
//         width = w;
//         DIST_FROM_INSIDE = dfi;
//       }
	 if( alpha_nn * sign( rad[ n(s.seg_ID) ] )
             <
             alpha    * sign( rad[ n(s.seg_ID) ] )
           )
	 {
	   alpha   =
	   alpha_n = alpha_nn;
         }
	   if( ending_speed( s.v,
			     cateto( R+d_big, R ),
			     BRAKE_ACCEL,
			     e_d )
	       >
	       speed( R )
	     )
	     nn_speed = speed( R );

//	   else
//	     nn_speed = s.v + 10.0;
//	 }
         if( sign( rad[ n(n(n(s.seg_ID))) ] ) == sign( rad[ n(s.seg_ID) ] ))
           // Straight and S curve and S curve
           R = R;


       }
       else
       if( sign( rad[ n(n(s.seg_ID)) ] ) == sign( rad[ n(s.seg_ID) ] )
	   &&
	   len[ n(s.seg_ID) ] < PI*0.5
//           &&
//           fabs( rad[ n(s.seg_ID) ] ) > fabs( rad[ n(n(s.seg_ID)) ] )
	 )
       {
	 // Straight and 2 curves to the same side

	 d = fabs( rad[   n(s.seg_ID)  ] )
	   - fabs( rad[ n(n(s.seg_ID)) ] );

	 y = s.to_end + d*sin( len[ n(s.seg_ID) ] );
	 x = rad[ n(s.seg_ID) ] > 0.0 ? s.to_lft : s.to_rgt;
	 x += fabs( rad[ n(s.seg_ID) ] );
	 x -= d*cos( len[ n(s.seg_ID) ] );

	 t = hipotenusa( x, y );

	 to_begin = atan( y/x )
		  + len[ n(s.seg_ID) ];

	 to_inner = t - fabs( rad[ n(n(s.seg_ID)) ] );

	 alpha_nn = len[ n(s.seg_ID) ] * sign( rad[ n(s.seg_ID) ] )
		  - to_begin           * sign( rad[ n(s.seg_ID) ] )

		  + tangent_alpha( n(n(s.seg_ID)),
				   to_begin + len[ n(n(s.seg_ID)) ],
				   to_inner,
				   d_big,
				   R
				 );

	 if( //sign( alpha ) == sign( rad[ n(s.seg_ID) ] )
	     //&&
               sign( alpha_nn ) == sign( rad[ n( s.seg_ID) ] )
               &&
               ( alpha_nn * sign( rad[ n(s.seg_ID) ] )
	       <
	       alpha    * sign( rad[ n(s.seg_ID) ] )
               ||
               sign( alpha_nn ) == sign( rad[ s.seg_ID ] )
             )
             &&
             len[ n(s.seg_ID) ] < PI * 0.5
             &&
             fabs( rad[ n(s.seg_ID) ] ) > fabs( rad[ n(n(s.seg_ID)) ] )
             &&
             DP[ !forwards ][ n(n(s.seg_ID)) ] < 1.0
	   )
	 {

	   alpha    =
	   alpha_n  =
	   alpha_nn = alpha_nn;
//                      sign( alpha_nn ) != sign( rad[ n(s.seg_ID) ] )
//		    ? alpha * 0.5 //0.0
//		    : alpha_nn;
         }
//	   if(   s.v > speed( fabs( rad[ n(s.seg_ID) ] ) + width ))
//	     n_speed = speed( fabs( rad[ n(s.seg_ID) ] ) + width );

         if( fabs( rad[ n(n(s.seg_ID)) ] )
             <
             fabs( rad[ n(s.seg_ID) ] )
           )
	   if( speed( R )
               <
	       ending_speed( s.v,
		             cateto( R+d_big, R ),
		             brake_accel( s.seg_ID ),
		             e_d
	                   )
//               s.v > speed( R )
//	       &&
//	       n_speed > speed( R )
	     )
	     nn_speed = speed( R );

//         }
       }

//       else
//         alpha_n = alpha;
/*
       { double x,y,t;

	 x = s.to_end;
	 y = rad[ n(s.seg_ID) ] < 0.0 ? s.to_rgt : s.to_lft;
	 y < 0.0 ? y = 0.0 : y;
	 y += fabs( rad[ n(s.seg_ID) ] );
	 t = hipotenusa( x, y );
	 t<rad[ n(s.seg_ID) ]?t=rad[ n(s.seg_ID) ]:t;

	 emergency_override |= sign( s.vn ) == sign( rad[ n(s.seg_ID) ] )
			       &&
			       PI*0.5
			     - acos( fabs(s.vn)/s.v )
			       <
			       PI*0.5
			     - atan( x/y )
			     - asin( fabs( rad[ n(s.seg_ID) ] )/ t );
       }
*/
//     }
   }
   else
   {
     // Curve ------------------------------------------------------------

     slip_correction = 1;//1;
//     if( s.to_end > s.cur_len * 0.5 )
//       emergency_override = 1;
     // Calculate alpha to the curve's tangent
     alpha = tangent_alpha( s.seg_ID,
			    s.to_end,
			    rad[ s.seg_ID ] > 0.0 ? s.to_lft : s.to_rgt,
			    d_big,
			    R );

     if( //d_big == 0.0
	 //&&
	 //s.v > speed( R )
         speed( R )
	 <
	 ending_speed( s.v,
		       cateto( R+d_big, R ),
		       brake_accel( s.seg_ID ),
		       e_d
		     )
       )
       _speed = speed( R );


     if( rad[ n(s.seg_ID) ] == 0.0 )
     {
       // Curve and Straight
//       alpha_n = s.to_end * sign( rad[ s.seg_ID ] ); // alpha;

       if( sign( rad[ s.seg_ID ] ) * sign( rad[ n(n(s.seg_ID)) ] ) == 1
//           &&
//           radius( s.seg_ID ) > radius( n(n(s.seg_ID)) )
	 )
       { double x;//, alpha_n;
	 // Curve straight and curve to the same side.
	 // C curve

	 // Check if next curves's trayectory is visible.
	 r =  fabs( rad[ s.seg_ID ] );
	 r += rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
	 x =  r * sin( s.to_end );
	 x += len[ n(s.seg_ID) ];
	 y =  r * cos( s.to_end );
	 y -= fabs( rad[ s.seg_ID ] );
	 y += fabs( rad[ n(n(s.seg_ID)) ] );

	 to_begin =  PI * 0.5;
	 to_begin -= atan( y / x );

	 to_inner =  hipotenusa( x, y );
	 to_inner -= fabs( rad[ n(n(s.seg_ID)) ] );

	 alpha_nn  = s.to_end * sign( rad[ s.seg_ID ] );
	 alpha_nn -= to_begin * sign( rad[ n(n(s.seg_ID)) ] );

	 alpha_nn += tangent_alpha( n(n(s.seg_ID)),
			 	    to_begin + len[ n(n(s.seg_ID)) ],
				    to_inner,
				    d_big,
				    R
                                  );

	   if( speed( R )
	       <
	       ending_speed( s.v,
			     cateto( R+d_big, R ),
			     brake_accel( s.seg_ID ),
			     e_d
			   )
	     )
           {
	     nn_speed = speed( R );
//  	     if( //d_big > 0.0
//	         //&&
//	         alpha_nn * sign( rad[ s.seg_ID ] )
//                 <
//                 alpha    * sign( rad[ s.seg_ID ] )
//               )
	       alpha = ( alpha + alpha_nn ) * 0.5;
//		   alpha = asin( s.vn/s.v );

           }
	   else
 	   if( //d_big > 0.0
	       //&&
	       alpha_nn * sign( rad[ s.seg_ID ] )
               <
               alpha    * sign( rad[ s.seg_ID ] )
             )
 	   {
//           if( d_big <= 0.0 )
             alpha = ( alpha + alpha_nn ) * 0.5;
//
//           else
//             alpha = alpha_nn;

//           d_big < 0.0 ? d_big = 0.0 : d_big;
           }
//	     nn_speed = s.v + 10.0,
//	     alpha = alpha_nn;

//	 }
       }
       else
       { // curve straight and curve to the other side
	 // kind of S curve
	 double t,x,y;

	 t = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
	 t += fabs( rad[ s.seg_ID ] );
	 x = t * sin( s.to_end )
	   + len[ n(s.seg_ID) ];
	 y = t * ( 1.0 - cos( s.to_end ))
	   + fabs( rad[ s.seg_ID ] ) + real_width - t
	   + fabs( rad[ n(n(s.seg_ID)) ] );

	 to_begin = atan( x/y );
	 to_inner = hipotenusa( x, y ) - fabs( rad[ n(n(s.seg_ID)) ] );

	 alpha_nn = s.to_end * sign( rad[ s.seg_ID ] )
		 + atan( y/x ) * sign( rad[ n(n(s.seg_ID)) ] )
		 - PI*0.5 * sign( rad[ n(n(s.seg_ID)) ] )
		 + tangent_alpha( n(n(s.seg_ID)),
				  to_begin + len[ n(n(s.seg_ID)) ],
				  to_inner,
				  d_big,
				  R );

	 if( alpha_nn * sign( rad[ s.seg_ID ] ) < alpha * sign( rad[ s.seg_ID ] ))
	 {
	   alpha = alpha_nn;
//         d_big < 0.0 ? d_big = 0.0 : d_big;
         }
	   if( speed( R )
	       <
	       ending_speed( s.v,
			     cateto( R+d_big, R ),
			     brake_accel( s.seg_ID ),
			     e_d
			   )
	     )
	     nn_speed = speed( R );

//	   else
//	     nn_speed = s.v + 10.0;
//	 }
       }
     }
     else
     {
       // Curve and Curve
       if( sign( rad[ s.seg_ID ] ) != sign( rad[ n(s.seg_ID) ] ))
       {
	 // S curve
	 r = fabs( s.cur_rad ) + real_width + fabs( s.nex_rad );
	 t = r*sin( s.to_end );
	 q = r*cos( s.to_end );
	 q -= fabs( s.cur_rad );
	 q -= s.cur_rad > 0.0 ? s.to_lft : s.to_rgt;
	 r = hipotenusa( t, q );
	 d = r
	   - fabs( s.nex_rad );
//	   - ( real_width - width ); // - DIST_FROM_INSIDE;
	 if( d < 0.0 )
	   d = 0.0;
	 r -= d;

	 { double alpha_1, alpha_2;
	   alpha_2 = acos( q / ( r+d ));

	   // new alpha_1:
//         { double dfi = DIST_FROM_INSIDE,
//                  w = width;
//           DIST_FROM_INSIDE = real_width - w;
//           width            = real_width - dfi;
//           if( width > real_width -  WIDTH_FACTOR )
//             width = real_width -  WIDTH_FACTOR;

	     alpha_1 = tangent_alpha( n(s.seg_ID),
				      len[ n(s.seg_ID) ]
				    - s.to_end
				    + alpha_2,
				      d,//+ DIST_FROM_INSIDE,
				      d_big,
				      R );

//           width            = w;
//           DIST_FROM_INSIDE = dfi;
//           }

//         alpha_1 = ( PI*0.5 - asin( r / ( r+d )));

//         if( d_big < 0.0 )
//         {
//           d = rad[ s.seg_ID ] < 0.0 ? s.to_lft : s.to_rgt;
//           d -= DIST_FROM_INSIDE;
//           d<0.0?d=0.0:d;
//           alpha_1 = 0.0; // *= d / (d - d_big );
//           alpha = alpha_n;// = ( alpha + alpha_n ) * 0.5;
//         }

	   alpha_n = //- PI / 2.0
		   + alpha_2
		     // + PI / 2.0
		   - fabs( alpha_1 );

	 }

	 alpha_n *= sign( rad[ s.seg_ID ] );

	 d = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
	 d<DIST_FROM_INSIDE?d=DIST_FROM_INSIDE:d;
	 if( //d_big <= 0.0
	     //||
	     alpha_n * sign( rad[ s.seg_ID ] ) < alpha * sign( rad[ s.seg_ID ] )
//           ||
//           alpha_n * sign( rad[ s.seg_ID ] ) < acos((fabs(rad[ s.seg_ID ])+DIST_FROM_INSIDE)/(fabs(rad[ s.seg_ID ])+d))
	   )
	 {
           alpha = alpha_n;
         }
	   if( d_big <= 0.0
	       ||
	       speed( R )
	       <
	       ending_speed( s.v,
			     cateto( R+d_big, R ),
			     brake_accel( s.seg_ID ),
			     e_d
			   )
	     )
//             if( n_speed > speed( R ))
	     n_speed = speed( R );

//	   else
//	     n_speed = s.v + 10.0;

	   if( rad[ n(n(s.seg_ID)) ] == 0.0 )
             // S curve and straight
             R=R;

           else
	   if( sign( rad[ n(n(s.seg_ID)) ] ) != sign( rad[ n(s.seg_ID) ] ))
	   { // S curve and S curve
	     double d,r,t,a,x,y;
	     d = (rad[ s.seg_ID ]<0.0?s.to_rgt:s.to_lft);
	     r = fabs( rad[ s.seg_ID ] ) + d;
	     t = fabs( rad[ s.seg_ID ] ) + real_width + fabs( rad[ n(s.seg_ID) ] );
	     x = r*cos( s.to_end );

	     y = r*sin( s.to_end );
	     x = t - x;

	     r = hipotenusa( x, y );
	     a = atan( y/x );

	     y = r * sin( a + len[ n(s.seg_ID) ] );
	     x = r * cos( a + len[ n(s.seg_ID) ] );

	     t = fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] );
	     x = t - x;

	     to_begin = atan( y/x );
	     to_inner = hipotenusa( x, y ) - fabs( rad[ n(n(s.seg_ID)) ] );

	     alpha_nn = s.to_end
		      - len[ n(s.seg_ID) ]
		      - to_begin
		      + fabs( tangent_alpha( n(n(s.seg_ID)),
					     to_begin + len[ n(n(s.seg_ID)) ],
					     to_inner,
					     d_big,
					     R ));
	     alpha_nn *= sign( rad[ s.seg_ID ] );

	     d = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
	     d<DIST_FROM_INSIDE?d=DIST_FROM_INSIDE:d;

	     if( alpha_nn *sign( rad[ s.seg_ID ] )
		 >
		 alpha_n  *sign( rad[ s.seg_ID ] )
		 &&
		 alpha_nn *sign( rad[ s.seg_ID ] )
		 <
		 acos( (fabs(rad[ s.seg_ID ])+DIST_FROM_INSIDE)
		      /(fabs(rad[ s.seg_ID ])+d)
		     )
	       )
	     {
	       alpha = alpha_nn;
             }
	       if( d_big <= 0.0
		   ||
		   speed( R )
		   <
		   ending_speed( s.v,
				 cateto( R+d_big, R ),
				 brake_accel( s.seg_ID ),
				 e_d
			       )
		 )
//               if( nn_speed > speed( R ))
		 nn_speed = speed( R );

//	       else
//		 nn_speed = s.v + 10.0;

//	     }
           else
             // S curve and c curve
           ;
	 }

//       width *=  S_FACTOR ];
       }
       else
       {
	 // Curve and Curve. Same side.
	 // c curve

	 { double x,y,d;

	 // Check if next curves's trayectory is visible.
	 d =  rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
	 r =  d + fabs( rad[ s.seg_ID ] );
	 x =  r * sin( s.to_end );
//	 x += len[ n(s.seg_ID) ];
	 y =  r * cos( s.to_end );
	 y -= fabs( rad[ s.seg_ID ] );
	 y += fabs( rad[ n(s.seg_ID) ] );

	 to_begin =  PI * 0.5;
	 to_begin -= atan( y/x );

	 to_inner =  hipotenusa( x, y );
	 to_inner -= fabs( rad[ n(s.seg_ID) ] );

	 alpha_n  = s.to_end * sign( rad[ s.seg_ID ] );
         alpha_n -= to_begin * sign( rad[ n(s.seg_ID) ] );

	 alpha_n += tangent_alpha( n(s.seg_ID),
				  to_begin + len[ n(s.seg_ID) ],
				  to_inner,
				  d_big,
				  R );

	 if( //d_big > 0.0
	     //&&
	     ( alpha_n * sign( rad[ n(s.seg_ID) ] ) < alpha * sign( rad[ s.seg_ID ] )
               ||
               sign( alpha_n - s.to_end * sign( rad[s.seg_ID] ) )
               ==
               sign( rad[ s.seg_ID ] )
//               &&
//               acos(   fabs( rad[ s.seg_ID ] )
//                   / ( fabs( rad[ s.seg_ID ] ) + d )
//                   )
//               * sign( rad[ s.seg_ID ] )
//               >
//               alpha_n * sign( rad[ s.seg_ID ] )
             )
             &&
             fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] )
           )
	 {
//         if( d_big <= 0.0 )
//           alpha = ( alpha + alpha_nn ) * 0.5;
//
//         else
           alpha = alpha * 0.3 + alpha_n * 0.7;
         }
//         d_big < 0.0 ? d_big = 0.0 : d_big;

	 if( //d_big > 0.0
             //&&
             speed( R )
	     <
	     ending_speed( s.v,
			   cateto( R+d_big, R ),
			   brake_accel( s.seg_ID ),
			   e_d
	                 )
             &&
             fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] )
	   )
         {
	   n_speed = speed( R );

//             if( sign( alpha_n ) == sign( rad[ s.seg_ID ] ))
//               alpha = alpha_n;

//             else
//	       alpha = ( alpha + alpha_n ) * 0.5;
         }
//	   else
//           {
//	     n_speed = s.v + 10.0;
//	     alpha = alpha_n;
//           }
//	 }
         }
/*--
	   r = fabs( rad[ s.seg_ID ] ) + DIST_FROM_INSIDE;
	   d = rad[ s.seg_ID ] > 0.0 ? s.to_lft : s.to_rgt;
	   d<0.0?d=0.0:d;
	   d -= DIST_FROM_INSIDE;
	   y = (r+d) * sin( s.to_end );
	   x = (r+d) * cos( s.to_end );
	   x -= fabs( rad[ s.seg_ID ] ) - fabs( rad[ n(s.seg_ID) ] );
	   t = hipotenusa( y, x );

	   to_begin = acos( x/t );
	   if( y < 0.0 )
	     to_begin = 2.*PI - to_begin;

	   to_inner = t - fabs( rad[ n(s.seg_ID) ] );

	   alpha_n = s.to_end * sign( rad[ s.seg_ID ] )
		   - to_begin * sign( rad[ s.seg_ID ] )
		   + tangent_alpha( n(s.seg_ID),
				    to_begin + len[ n(s.seg_ID) ],
				    to_inner,
				    d_big,
				    R );
	   if( d_big == 0.0
	       &&
	       sign( alpha_n ) != sign( rad[ s.seg_ID ] )
	     )
	     alpha_n *= 0.0; //0.5;
/ *
	   // check case r0 < r1
	   if( fabs( rad[ s.seg_ID ] ) < fabs( rad[ n(s.seg_ID) ] )
	       &&
	       t > fabs( rad[ n(s.seg_ID) ] ) + DIST_FROM_INSIDE
	     )
	   { //if( alpha   * sign( rad[ s.seg_ID ] )
	     //    >
	     //    alpha_n * sign( rad[ s.seg_ID ] )
	     //  )
	     alpha   = alpha_n;
	     n_speed = speed( R );

	   }
* /
//             ||
//             rad[ s.seg_ID ] > rad[ n(s.seg_ID) ]
//             &&
//             fabs( PI * 0.5 - asin( r / ( r + d ))) == s.to_end

	   // check case r0 > r1
	   if( fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] )
	       &&
	       d_big <= 0.0
	       &&
	       to_begin < PI/2.
	       &&
	       ( sign( rad[ n(n(s.seg_ID)) ] ) != sign( rad[ s.seg_ID ] )
		 ||
		 fabs( rad[ n(n(s.seg_ID)) ] ) > fabs( rad[ n(s.seg_ID) ] )
	       )
//               &&
//               s.to_end * fabs( rad[ s.seg_ID ] ) < curve_distance( 1 )
	       ||
	       following_next
	     )
	   {
	     // follow next curve
//           alpha_n  = s.to_end * sign( rad[ s.seg_ID ] );
//           alpha_n -= acos( q/t ) * sign( rad[ s.seg_ID ] );
//           d = t - fabs( rad[ n(s.seg_ID) ] ) - DIST_FROM_INSIDE;
//           d < 0.0 ? d = 0.0 : d;
//           t -= d;
//           alpha_n += ( PI * 0.5 - asin( t / ( t + d )))
//                    * sign( rad[ n(s.seg_ID) ] );

	     following_next = 1;

	     alpha = alpha_n;
//           if( //sign( s.vn ) != sign( rad[ s.seg_ID ] )
//               //&&
//               sign( alpha ) != sign( rad[ s.seg_ID ] )
//             )
//             alpha *= 0.5;

	     if( s.v > speed( R )
		 &&
		 ( n_speed == 0.0
		   ||
		   n_speed > speed( R )
		 )
	       )
	       n_speed = speed( R );

//           emergency_override |= sign( s.vn )
//                                 ==
//                                 sign( rad[ s.seg_ID ] );

	   }
/ *
	   if( d_big > 0.0
	       &&
	       speed( R )
	       <
	       ending_speed( s.v,
			     cateto( R+d_big, R ),
			     brake_accel( s.seg_ID ),
			     e_d
			   )
	     )
	     // follow next curveïs speed
	     if( n_speed == 0.0
		 ||
		 n_speed > speed( R )
	       )
	       n_speed = speed( R );
* /
	 }
--*/
       }
     }
     // ignore next segment
//     alpha_n = alpha;
  }
/*
  //   If next curve's tangent is visible
  //   then
  if( alpha_n * sign( s.cur_rad ) < alpha * sign( s.cur_rad ))
  {
    // go directly to next curve
    s.to_end *= rad[ s.seg_ID ] == 0.0 ? 1 : fabs( rad[ s.seg_ID ] );
    rad[ s.seg_ID ] = 0.0;
    alpha = alpha_n;
  }
*/
//  if( s.to_lft < DIST_FROM_INSIDE * 0.5 )
//    alpha -= 0.5;
//  if( s.to_rgt < DIST_FROM_INSIDE * 0.5 )
//    alpha += 0.5;

//  y  = s.vn > 0.0 ? s.to_lft : s.to_rgt;
//  y -= ( s.alpha * s.vn < 0.0 )
//       ?  WIDTH_FACTOR * 0.9
//       :(DIST_FROM_INSIDE * 0.5);

  if( s.to_rgt < 0.0 + DIST_FROM_INSIDE
      ||
      s.to_lft < 0.0 + DIST_FROM_INSIDE
    )
	{
		alpha = - 0.1 * sign( s.to_rgt - real_width * 0.5 );
	  _speed = s.v-10;
    if( s.to_rgt < 0*CARWID
        ||
        s.to_lft < 0*CARWID
      )
      alpha = - 0.2 * sign( s.to_rgt - real_width * 0.5 ),
			_speed = 0.0;
	}
/*
  else

  if( ending_speed( fabs( s.vn ), y,  BRK_CRV_ACC, e_d ) > 0.0 )
    if( !emergency_override
	&&
	sign( s.vn ) != sign( alpha - asin( s.vn/s.v ))
	&&
	( sign( s.vn ) != sign( rad[ s.seg_ID ] )
	  ||
	  PI*0.5 - acos( fabs(s.vn)/s.v )
	  >
	  acos( fabs( rad[ s.seg_ID ] )
		/ ( ( rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft )
		    + fabs( rad[ s.seg_ID ] )
		  )
	      )
	)
      )
    {
      slip_correction = 1; //0;
      alpha += //-  EMERG_ALPHA ]
	       ( alpha - asin( s.vn/s.v ))
	       // * sign( s.vn )
	     ;
//      if( n_speed == 0.0
//          ||
//          n_speed > s.v - 10.0
//        )
//        n_speed = s.v - 10.0;
    }
    else
      y = y;
*/




     // set the speed:
  { double e_sp, to_next;

    vc = corn_spd( s.seg_ID );
    if( _speed > 0.0 )
      vc = _speed;

    to_next = t_distance( s.seg_ID );
    to_next *= s.to_end/len[ s.seg_ID ];

//        rad[ s.seg_ID ] != 0.0
//        ? s.to_end * fabs( rad[ s.seg_ID ] )
//        : s.to_end;
    if( ( e_sp = ending_speed( s.v,

			       to_next
			       - curve_distance( n(s.seg_ID) ),
			       curveing
			       ? BRK_CRV_ACC
			       : brake_accel( s.seg_ID ),
			       e_d
			     )
	)
	>
	corn_spd( n(s.seg_ID) )
	&&
	vc > corn_spd( n(s.seg_ID) )
      )
      vc = corn_spd( n(s.seg_ID) );

    if( n_speed > 0.0
	&&
	vc > n_speed
      )
      vc = n_speed;

//    to_next -= e_d;
    to_next += t_distance( n(s.seg_ID) );
//        rad[ n(s.seg_ID) ] != 0.0
//      ? len[ n(s.seg_ID) ] * fabs( rad[ n(s.seg_ID) ] )
//      : len[ n(s.seg_ID) ];
    if( //e_sp > 0.0
	//&&
	( e_sp = ending_speed( s.v,//e_sp,

			      to_next //curve_distance( 1 )
			    - curve_distance( n(n(s.seg_ID)) ),
			      brake_accel( n(s.seg_ID) ),
			      e_d
			     )
	)
	>
	corn_spd( n(n(s.seg_ID)) )
	&&
	vc > corn_spd( n(n(s.seg_ID)) )
      )
      vc = corn_spd( n(n(s.seg_ID)) );

    if( nn_speed > 0.0
	&&
	vc > nn_speed
      )
      vc = nn_speed;

//    to_next -= e_d;
    to_next += t_distance( n(n(s.seg_ID)) );
//        rad[ n(n(s.seg_ID)) ] != 0.0
//      ? len[ n(n(s.seg_ID)) ] * fabs( rad[ n(n(s.seg_ID)) ] )
//      : len[ n(n(s.seg_ID)) ];
    if( //e_sp > 0.0
	//&&
	( e_sp = ending_speed( s.v,//e_sp,

			      to_next
			    - curve_distance( n(n(n(s.seg_ID))) ),
			      brake_accel( n(n(s.seg_ID)) ),
			      e_d
			     )
	)
	>
	corn_spd( n(n(n(s.seg_ID))) )
	&&
	vc > corn_spd( n(n(n(s.seg_ID))) )
      )
      vc = corn_spd( n(n(n(s.seg_ID))) );

    if( nnn_speed > 0.0
	&&
	vc > nnn_speed
      )
      vc = nnn_speed;

//    to_next -= e_d;
    to_next += t_distance( n(n(n(s.seg_ID))) );
//        rad[ n(n(n(s.seg_ID))) ] != 0.0
//      ? len[ n(n(n(s.seg_ID))) ] * fabs( rad[ n(n(n(s.seg_ID))) ] )
//      : len[ n(n(n(s.seg_ID))) ];
    if( //e_sp > 0.0
	//&&
	( e_sp = ending_speed( s.v,//e_sp,

			      to_next
			    - curve_distance( n(n(n(n(s.seg_ID)))) ),
			      brake_accel( n(n(n(s.seg_ID))) ),
			      e_d
			     )
	)
	>
	corn_spd( n(n(n(n(s.seg_ID)))) )
	&&
	vc > corn_spd( n(n(n(n(s.seg_ID)))) )
      )
      vc = corn_spd( n(n(n(n(s.seg_ID)))) );

    if( final_straight())
    { vc = s.v + 20.0;
      alpha = 0.0;
    }

    if( apply_brakes() )
    {
      if( s.v < vc * 0.5 
	  		  && 
					s.v < max_speed * 0.5 
		  		//&& crash_in_front() 
			  )
		    alpha = atan(( real_width * 0.5 - s.to_rgt ) / (2*s.v));
			
	    if( vc > s.v - 10.0 )
	      vc = s.v - 10.0; //, // s.nearby[ 0 ].v;

//      alpha *= 2.0;

//      DIST_FROM_INSIDE += CARWID;
//      sound( vc );
//      if( s.to_lft > 10.0 && s.to_rgt > 10.0 )
//        alpha +=  EMERG_ALPHA ]
//             * sign( s.nearby[0].rel_x )
//               * sign( - s.vn );
    }

//  if( vc > 80.0
//      &&
//      s.v < 40.0
//      &&
//      dead_ahead( s )
//      &&
//      sign( alpha ) != sign( s.nearby[0].rel_x )
//    )
//    alpha = - 2.0 *  EMERG_ALPHA ] * sign( rad[ n(s.seg_ID) ] ),
//    vc = 45.0;

//    if( vc > s.v + 10.0 )
//      use_alpha_factor = 0;

    if( vc > s.v + 1.0 )
      vc = s.v + ( rad[ s.seg_ID ] == 0.0 ? 20.0 : 5.0 );

    if( vc < s.v //- 2.0
//      &&
//      vc > s.v - 20.0
      )
      vc = s.v - 10.0;

//  nosound();

//else
//  DIST_FROM_INSIDE = 1.0;

    if( vc < 5.0 )
      vc = 5.0;
/*
    { int ss = rad[ s.seg_ID ] == 0.0
	      ? sign( rad[ n(s.seg_ID) ] )
	      : sign( rad[   s.seg_ID  ] );

      if( alpha * ss < asin( s.vn/s.v ) * ss )
	use_alpha_factor = 0;
    }
*/
    ALPHA_FACTOR = 1.0;

  	if( s.damage > last_damage )
			 if( s.to_rgt > 0*CARWID
				   &&
				   s.to_lft > 0*CARWID
			   )
         ALPHA_FACTOR = 0.0;
			 else
				 ALPHA_FACTOR = 0.5;

    if( s.v > 0.1 )
      alpha -= asin( s.vn/s.v );

//    if( s.vc > last_speed
//	  	  &&
//		  	s.v < last_speed
//			)
    if( use_alpha_factor )
      alpha *= ALPHA_FACTOR;

    fabs( alpha ) > 1.0 ? alpha = 1.0 * sign( alpha ) : alpha;

//  if( alpha > s.alpha +  STEER_SPD ] )
//    alpha = s.alpha +  STEER_SPD ];

//  if( alpha < s.alpha -  STEER_SPD ] )
//    alpha = s.alpha -  STEER_SPD ];

    if( s.vc > s.v
				&&
				s.vc * cos( fabs( s.alpha )) < s.v
      )
      if( slip_correction )
      {
        alpha = acos( s.v / s.vc ) * sign( alpha );

        slip_corrected = 1;
      }
      else
				slip_corrected = 0;
    else
    {
      if( slip_corrected )
        alpha = ( alpha + s.alpha ) / 2.0;

      slip_corrected = 0;
    }

    { static int count=0;
			if( vc > s.v )
			{
				vc = s.v + (vc-s.v)/1.0*count;
				++count>1 ? count-- : count;
			}
			else
				count = 0;
		}

		if( vc > s.v /*- 2.0*/ )
		{
//  if( vc > s.vc/last_speed*s.v && last_speed < s.v )
//	  vc = s.vc/last_speed*s.v;

			vc /= cos( alpha );
//    else
//      vc = vc;
		}

  }

  result.vc    = vc;
  result.alpha = alpha;//-0.15;

  last_damage = s.damage;
  last_speed = s.v;
  last_x = s.to_end * (rad[ s.seg_ID ]==0.0?1.0:fabs(rad[ s.seg_ID ]));
  too_close = 0;

  if(s.starting)
    if((s.stage==QUALIFYING)||(s.stage==PRACTICE))
      result.fuel_amount = 30.0;

    else
      result.fuel_amount = MAX_FUEL;

  return result;
}
