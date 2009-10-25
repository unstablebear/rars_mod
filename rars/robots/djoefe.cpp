// -------------------------------------------------------------------
//
//                             "Djoefe"  
//
// ------------------------------------------------------------------- 
//
// A RARS robot driver by Gian-Carlo Pascutto
//
// Designed for RARS 0.72, tested with Linux 2.2.x/gcc-2.95/RARS0.74 
//  
// -------------------------------------------------------------------
//
// This robot is basically an improved stocker model,
// with some "tricks" to let it handle non-trivial situations.
// It performs quite good on some circuits nonetheless,
// mainly because of the optimization possibilities.
// Since it is a stocker model it uses a lot of "common functions". 
// I took the ones from Apex92, because they seemed to be the most 
// accurate ones.
// There are probably "traces" of other cars here too...
//
// -------------------------------------------------------------------
//
// This source code is public, but if any part of it were to be used
// in another program/robot, you must inform me and give appropriate 
// credit. I take no responsability for the correct functioning of
// the code, or any damage it may cause.
//
// -------------------------------------------------------------------
//
// Gian-Carlo Pascutto (GianCarlo.Pascutto@advalvas.be)
//
// -------------------------------------------------------------------
//
//
// $Id: djoefe.cpp,v 1.6 2001/08/15 22:41:12 mgueury Exp $     
//
// $Log: djoefe.cpp,v $
// Revision 1.6  2001/08/15 22:41:12  mgueury
// RaceData + args
//
// Revision 1.5  2001/08/12 16:53:37  mgueury
// InstantReplay and trajectory
//
// Revision 1.4  2001/08/05 12:32:10  mgueury
// warning
//
// Revision 1.3  2001/07/22 12:03:57  mgueury
// Warning removed
//
// Revision 1.2  2001/07/03 18:13:48  carsten_kjaer
// Restructured Track.h and Track.cpp - all information about tracks is now in class Track, and the current track can be accessed through the global variable 'Track* currentTrack'. Also some restructuring of random functions - not finished yet.
//
// Revision 1.1  2001/03/27 18:52:53  timfoden
// Added version 0.82 files into new module in SourceForge repository
//
// Revision 1.1.1.1  2000/09/16 20:26:56  mgueury
// First release on sourceforge
//
// Revision 1.12  1999/10/31 09:45:05  giancarlo
// Replaced BF optimizer with a genetic one. (Doug Eleveld)
// Changed several values from constants to variables.
// Minor optimizations to code.
// Added code to set the car up for diving into a corner.
//
// Revision 1.11  1999/08/15 11:14:48  giancarlo
// More optimized tracks.
// New parameter: curve override.
// More look-ahead breaking code.
//
// Revision 1.10  1999/06/27 09:00:51  giancarlo
// Added partial look-ahead braking code.
// Bot can now survive tracks like montreal.
//
// Revision 1.9  1999/05/02 08:48:23  giancarlo
// New corner diving code. (Doug Eleveld)
// Fixed possible macro-expansion bug. (Doug Eleveld)
// Added optimized parameters for imola.trk
//
// Revision 1.8  1999/04/08 22:11:24  giancarlo
// Optimized parameters for brazil.trk.
// Support for BF-Optimizer.
//
// Revision 1.7  1999/04/07 19:15:01  giancarlo
// Settable traction control.
// Minor code cleanups.
// Added header with info 'bout the bot.
//
// Fixed some compiler warnings.
//  (what's left are bugs in the compiler)
//
// Revision 1.6  1999/04/07 16:26:07  giancarlo
// Supports track specific data sets now.
//
// Revision 1.5  1999/04/07 15:36:31  giancarlo
// Added trackdata structure.
// Fixed major bug in is_nex_straight, is_curr_straight.
//
// Revision 1.4  1999/04/07 15:02:57  giancarlo
// Improved, faster pitting code.
//
// Revision 1.3  1999/04/07 14:23:23  giancarlo
// General code cleanup.
// Moved some code around to allow better finetuning.
// Simplified "exit curve" code, performance seems unaffected.
//
//

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "track.h"
#include "os.h"
#include "car.h"
#include <assert.h>

#undef OPTIMIZE
//#define SKIDMARKS

#undef BFOPT

// parameters to tinker with:
const double DELTA_LANE = 0.5;		// when car dead_ahead, change "lane" this much
const double BIG_SLIP = 9.0;	        // affects the bias of steering servo loop


#ifdef OPTIMIZE // Flag to end a race (by deliberate crash)
static int AbortRace = 0;
static const double EdgeLimit = 1.;
#endif //OPTIMIZE


// track specific parameters
double dist_from_inside;
double dist_from_outside;
double dist_for_dive;
double dive_setup_dist;
double steer_gain;
double damp_gain;
double start_dive;
double predict_width;
double ignore_slowdown;
double out_of_curve;
double curve_override;
double brake_ratio;
double brake_curve_slip;
double straight_radius;
double sharp_turn_radius;
double speed_traction_control;
double steer_traction_control;


struct data_set 
{
  char trackname[33];
  double dist_from_outside;
  double dist_from_inside;
  double dist_for_dive;
  double dive_setup_dist;
  double steer_gain;
  double damp_gain;
  double start_dive;
  double predict_width;
  double ignore_slowdown;
  double out_of_curve;
  double curve_override;
  double brake_ratio;
  double brake_curve_slip;
  double steer_traction_control;
  double speed_traction_control;
  double straight_radius;
  double sharp_turn_radius;
};

static data_set default_data =
{ 
  "default",
  35. + (0.906426-0.5)*70.,
  3. + 0.5357*10.,
  35. + (0.481951-0.5)*70.,   
  1.0,  // i.e. no dive setup 

  0.4 + (0.31148-0.5)*0.6,
  
  0.8 + (0.412505-0.5)*1.0,
  0.015 + (0.981826-0.5)*0.02,
  0.8 + (0.341777-0.5)*0.8,
  0.95 + (0.6253522-0.5)*0.40,
  0.65846,
  0.7 + (0.346487-0.5)*1.0,
  0.935 + (0.0239625-0.5)*0.10,
  15 + (538875-0.5)*10.,
  0.526012, 
  0.0, 
  1000 + (0.953039-0.5)*2000, 
  200 + (0.0253263-0.5)*600
};

static data_set track_data[] = 
{
  {"brazil.trk"  , 40, 5.1, 10, 1.0,  0.32, 0.8,  0.016, 0.50, 1.00, 0.55, 0.8, 0.935, 15., 0., 0., 1000, 200 },
//  {"imola.trk"   , 15, 5.1, 10, 1.0,  0.53, 0.8,  0.019, 1.10, 0.96, 0.68, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"monaco.trk"  , 20, 5.1, 10, 1.0,  0.50, 0.8,  0.015, 1.05, 0.80, 0.75, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"barcelon.trk", 40, 5.1, 10, 1.0,  0.44, 0.8,  0.011, 0.55, 1.10, 0.76, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"indy500.trk",  60, 5.1, 10, 1.0,  0.15, 0.8,  0.010, 0.90, 1.00, 0.50, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"oval2.trk",    25, 5.1, 10, 1.0,  0.40, 0.8,  0.010, 0.90, 0.90, 0.80, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"sepang1.trk",  10, 5.1, 10, 1.0,  0.15, 0.8,  0.025, 0.60, 1.00, 0.70, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"watglen.trk",  35, 5.1, 10, 1.0,  0.22, 0.8,  0.015, 0.80, 1.05, 0.62, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"v01.trk",      15, 5.1, 10, 1.0,  0.58, 0.8,  0.015, 0.90, 0.80, 0.67, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"magnycrs.trk", 15, 5.1, 10, 1.0,  0.40, 0.8,  0.020, 0.80, 1.00, 0.80, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"silver97.trk", 30, 5.1, 10, 1.0,  0.45, 0.8,  0.010, 0.81, 0.99, 0.70, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"austria.trk",  42, 5.1, 10, 1.0,  0.35, 0.8,  0.010, 0.86, 1.00, 0.55, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"hock.trk",     35, 5.1, 10, 1.0,  0.45, 0.8,  0.011, 0.90, 0.90, 0.60, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"hungary.trk",  25, 5.1, 10, 1.0,  0.45, 0.8,  0.015, 0.90, 1.10, 0.70, 0.8, 0.935, 15., 0., 0., 1000, 200 },
  {"spa.trk",      15, 5.1, 10, 1.0,  0.20, 0.8,  0.020, 0.90, 1.00, 0.70, 0.5, 0.935, 15., 0., 0., 1000, 200 },
  {"sepang.trk",   23, 3.5, 10, 1.0,  0.35, 0.8,  0.017, 0.62, 1.00, 0.65, 0.59, 0.894, 10.62, 0., 0., 338, 164 },
  {"sepang1.trk",  23, 3.5, 10, 1.0,  0.35, 0.8,  0.017, 0.62, 1.00, 0.65, 0.59, 0.894, 10.62, 0., 0., 338, 164 },
  {"suzuka.trk",   49, 3.9, 32, 0.23, 0.40, 0.57, 0.013, 1.00, 1.00, 0.70, 0.65, 0.920, 10.834, 0.039, 0.0325, 429, 204 },
  {"albrtprk.trk", 67.45, 0.5, 41.14, 1.0, 0.336, 0.4678, 0.012, 0.5194, 1.092, 0.043116, 0.7132, 0.96455, 11.94, 0, 0.627104, 1043, 19.22 },
  {"silver97.trk", 28.875, 1.36, 34.13872, 0.42677, 0.5608, 0.5678, 0.01361, 1.080316, 0.8626352, 0.724655, 0.740231, 0.928611, 15.04032, 0.0158265, 0.244798, 176.72,60.52},
  {"dadadoedoe", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0., 0., 0, 0,0} //sentinel
};


#ifdef OPTIMIZE // Declare variables for optimization
static void SetTrackData(const double* data, const unsigned int)
{
  dist_from_outside =  35. + (data[0]-0.5)*70.;
  dist_from_inside =  3. + data[1]*10.;
  dist_for_dive = 35. + (data[2]-0.5)*70.;
  dive_setup_dist = data[3];

  steer_gain = 0.4 + (data[4]-0.5)*0.6;
  
  damp_gain =  0.8 + (data[5]-0.5)*1.0;
  start_dive =  0.015 + (data[6]-0.5)*0.02;
  predict_width =  0.8 + (data[7]-0.5)*0.8;
  ignore_slowdown = 0.95 + (data[8]-0.5)*0.40;
  out_of_curve =  data[9];
  curve_override =  0.7 + (data[10]-0.5)*1.0;
  brake_ratio = 0.935 + (data[11]-0.5)*0.10;
  brake_curve_slip = 15.0 + (data[12]-0.5)*10.;
  steer_traction_control = data[13];
  speed_traction_control = data[14];
  straight_radius = 1000 + (data[15]-0.5)*2000;
  sharp_turn_radius = 300 + (data[16]-0.5)*600;
}

#include "optimize.h"

static const int Population = 200;
static const int GeneticLength = 17;

static optimizedata data;
static double lastlaptime = 9999.;
static unsigned long lastdamage = 9999;
static double lastdistance = 0.;

#endif //OPTIMIZE


// slip generates friction. I use only some fraction of it, defined as
// useful_friction (ca 90-95%). This affects curvespeed and braking distance.
static double slip, my_friction, useful_friction;


// track data
static track_desc t;


#define sqr(x)   ((x)*(x))
#define max(x,y)  (((x)>(y))? (x):(y))
#define min(x,y)  (((x)>(y))? (y):(x))

///// calculate braking distance from current speed to speed vthen.
static double find_bd (situation s, double vthen) 
{
  
    double C, D;
    double DRAG_CON = .0065;
    double mass = 75 + s.fuel / g;
    
    D = DRAG_CON * s.v * s.v / mass;	// air drag is helping to brake!
    double dv = s.v - vthen;	// brake if dv > 0!
  
    if (s.v < vthen)
      C = 0.0;			// no need to brake as we are already at the desired speed!
    else			// leave some grip for turning:
      C = (s.v - 0.5 * dv) * dv / (useful_friction * g + D);

    return C;
}

/* Not used
static double brakedist (situation s, double vnow, double vthen)
{
  double C, D;
  double DRAG_CON = .0065;
  double mass = 75 + s.fuel / g;

  D = DRAG_CON * vnow * vnow / mass;
  double dv = vnow - vthen;

  if (vnow < vthen)
    C = 0.0;
  else
    C = (vnow - 0.5 * dv) * dv / (useful_friction * g + D);

  return C;
} 
*/

///// calculate speed in curves:
static double curvespeed (situation s, double rad) 
{
  if (rad == 0.0)
    return s.v + slip;
  
  return sqrt (fabs (rad) * g * useful_friction); 
}

static double crvspeed(double rad)
{
  if (rad == 0.0)
    return 250;

  return sqrt (fabs(rad) * g * useful_friction);
}

static double linear(double tot, double rem, double start, double end)
{
  double done = (tot - rem) / tot;
  done = max(done,0.);
  done = min(done,1.);
  
  const double delta = end - start;
  return start + (done * delta);
}


///// Traction control: /////////////////////////////////////
static void traction_control (situation s, con_vec & result) 
{
  double newvc = result.vc;
  double newal = result.alpha;
  
  double C = sqrt (max (0, sqr (slip) - sqr (s.v * sin (result.alpha))));
  double B = sqrt (max (0, sqr (slip) - sqr (s.v - result.vc * cos (result.alpha))));
  
  if (result.vc > s.v)	// limit acceleration while exiting curve
    newvc = min (result.vc, s.v + C);
  
  if (newvc < s.v)	// limit alpha while braking
    {
      if (result.alpha > 0)
	newal = min (result.alpha, asin (min (1, B / result.vc)));
      if (result.alpha < 0)
	newal = max (result.alpha, -(asin (min (1, B / result.vc))));
    }
  
  result.vc = linear(1., speed_traction_control,newvc,result.vc);
  result.alpha = linear(1., steer_traction_control,newal,result.alpha);
}


static double curve_length (double len, double rad) 
{
  if (rad == 0.0)
    return (len);
  else
    return (len * fabs (rad));
}


static int is_curr_straight (situation & s) 
{
    if ((fabs (s.cur_rad) > straight_radius) || (s.cur_rad == 0))
      return 1;
    if ((fabs (s.cur_rad) > sharp_turn_radius) && (fabs (s.cur_len) < curve_override))
      return 1;
  
    return 0;  
}


static int is_nex_straight (situation & s) 
{
    if ((fabs (s.nex_rad) > straight_radius) || (s.nex_rad == 0))
      return 1;
    if ((fabs (s.nex_rad) > sharp_turn_radius) && (fabs (s.nex_len) < curve_override))
      return 1;
  
    return 0;  
}

#ifndef BFOPT
#ifndef OPTIMIZE
static void set_track_data(void)
{
  int i = 0;

  // set default parameters;
  dist_from_outside = default_data.dist_from_outside;
  dist_from_inside = default_data.dist_from_inside;
  dist_for_dive = default_data.dist_for_dive;
  dive_setup_dist = default_data.dive_setup_dist;
  steer_gain = default_data.steer_gain;
  damp_gain = default_data.damp_gain;
  start_dive = default_data.start_dive;
  predict_width = default_data.predict_width;
  ignore_slowdown = default_data.ignore_slowdown;
  out_of_curve = default_data.out_of_curve;
  curve_override = default_data.curve_override;
  brake_ratio = default_data.brake_ratio;
  brake_curve_slip = default_data.brake_curve_slip;
  straight_radius = default_data.straight_radius;
  sharp_turn_radius = default_data.sharp_turn_radius;
  steer_traction_control = default_data.steer_traction_control;
  speed_traction_control = default_data.speed_traction_control;

  // override defaults if optimized data present
  while(track_data[i].dist_from_inside != 0)
  {
    if(!strcmp(track_data[i].trackname, t.sName))
     {
       dist_from_outside = track_data[i].dist_from_outside;
       dist_from_inside = track_data[i].dist_from_inside;
       dist_for_dive = track_data[i].dist_for_dive;
       dive_setup_dist = track_data[i].dive_setup_dist;
       steer_gain = track_data[i].steer_gain;
       damp_gain = track_data[i].damp_gain;
       start_dive = track_data[i].start_dive;
       predict_width = track_data[i].predict_width;
       ignore_slowdown = track_data[i].ignore_slowdown;
       out_of_curve = track_data[i].out_of_curve;
       curve_override = track_data[i].curve_override;		
       brake_ratio = track_data[i].brake_ratio;
       brake_curve_slip = track_data[i].brake_curve_slip;
       straight_radius = track_data[i].straight_radius;
       sharp_turn_radius = track_data[i].sharp_turn_radius;      
       speed_traction_control = track_data[i].speed_traction_control;
       steer_traction_control = track_data[i].steer_traction_control;
     }

    i++;
  };
}
#endif
#endif


con_vec Djoefe (situation & s) 
{
  
  const char name[] = "Djoefe";	// This is the robot driver's name!
  static int init_flag = 1;	// cleared by first call
  con_vec result;		// This is what is returned.
  double alpha, vc;		// components of result
  static double lane = -10000;	// an absurd value to show not initialized
  double bias, speed, width, to_end;
  double speed_next= 0.0;
  
  static double fuel_total = 0;
  static double fuel_last;

  double aftaft_bd, after_bd, bd_now;
    
  // Load the optimization stuff for this track
#ifdef OPTIMIZE
  static int optinit = 0;
  if(optinit==0)
  {
    // Reload the population from the data file
    const char* optfile = "djoefe.opt";
    data.load(optfile,Population,GeneticLength,SetTrackData);
    cout << "\nOptimizing for track in " << optfile << endl;

    optinit = 1;
  }
#endif //OPTIMIZE

  if (init_flag == 1)
  {				// first time only, copy name:
    my_name_is (name);	// copy the name string into the host program
    init_flag = 0;
    result.alpha = result.vc = 0;
    
    return result;  
  }

  // set fuel for qualifications  
  if (s.starting && s.stage == QUALIFYING)
    result.fuel_amount = 20;
    
  // estimate available friction:
  useful_friction = 0.975 * my_friction - 0.2 * s.damage * s.damage / 9e8;
  
  // drive carefully on first laps:
  //    if(lap_count - s.laps_to_go < 2 && s.position >= 4)
  //        useful_friction *= .9*my_friction;
    
  // service routine in the host software to handle getting unstuck from
  // from crashes and pileups:
#ifndef OPTIMIZE
  if (stuck (s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc))
    return result;
#endif //OPTIMIZE

  width = s.to_lft + s.to_rgt;	// compute width of track

  
  // This is a little trick so that the car will not try to change lanes
  // during the "dragout" at the start of the race.  We set "lane" to
  // whatever position we have been placed by the host.
    if (lane < -9000)		// will be true only once
    {
      fuel_last = s.fuel;	// setup fuel counter

#ifndef OPTIMIZE
      set_track_data ();
#endif

      lane = s.to_lft;	// better not to change lanes at the start
      slip = BIG_SLIP;
      t = get_track_description ();
      
      if( !args.m_iSurface )		// friction model 0:
	my_friction = slip / (slip + 2.5);
      else			// friction model 1:
	my_friction = 1.05 * (1.0 - exp (-slip / 2.5));      
    }
  
#ifdef OPTIMIZE
  if(s.starting)
  {
    // Figure out a nice estimate of the MPH, or how far we
    // got around the track
    double mph = optimizedata::mph(lastlaptime,t.length,lastdamage,lastdistance);

    if(no_display)
    {
      cout << " Mph: " << mph
           << endl;
    }

    // Tell the optimizer what the last MPH was
    data.step(mph);
    AbortRace = 0;
  }
#endif //OPTIMIZE
    
    // Curve control 1
    // a) If we are in a curve, go to the inside
    // b) If we are exiting the curve into a straight then "slide" out of the curve

    // Sharp left
    if (s.cur_rad > 0.0 && s.cur_rad < straight_radius)	
    {
      if (is_nex_straight (s) && s.to_end <= s.cur_len * (1 - out_of_curve))
      {
        lane = width - dist_from_outside;
      }
      else
      {
	lane = dist_from_inside;
      }
      
    }
    // Sharp right
    else if (s.cur_rad < 0.0 && s.cur_rad > -straight_radius)
    {
      if (is_nex_straight (s) && s.to_end <= s.cur_len * (1 - out_of_curve))
      {
          lane = dist_from_outside;
      }
      else
      {
	  lane = width - dist_from_inside;
      }
    }
    
  // set the bias:
  // Bias is an additive term in the steering servo, so that the servo
  // doesn't have to "hunt" much for the correct alpha value.  It is an
  // estimate of the alpha value that would be found by the servo if there
  // was plenty of settling time.  It is zero for straightaways.
  // Also, for convenience, we call the corn_spd() function here.  On
  // the straightaway, we call it to find out the correct speed for the
  // corner ahead, using s.nex_rad for the radius.  In the curve we of
  // course use the radius of the curve we are in.  But also, we call it
  // for the next segment, to find out our target speed for the end of
  // the current segment, which we call speed_next.
    if (s.cur_rad == 0.0)
    {
	bias = 0.0;
     
	if (!is_nex_straight (s))
	  speed = curvespeed (s, fabs(s.nex_rad) + width * predict_width);	
        else
	  speed = 250.0;
    }
    else
    {
	if (is_nex_straight(s))
	  speed_next = 250.0;
	else
	  speed_next = curvespeed (s, fabs (s.nex_rad) + width * predict_width);
      	
	if(!is_curr_straight(s))
	  speed = curvespeed (s, fabs (s.cur_rad) + width * predict_width);
	else
	  speed = 250;
	
	bias = (s.v * s.v / (speed * speed)) * atan (BIG_SLIP / speed);	
      
	if (s.cur_rad < 0.0)	// bias must be negative for right turn
	  bias = -bias;
    }
    
  // set alpha:  (This line is the complete steering servo.)
    alpha = steer_gain * (s.to_lft - lane) / width - damp_gain * s.vn / s.v + bias;
  
  
  // set vc:  When nearing end of straight, change "lane" for the turn, also.
    if (s.cur_rad == 0.0)
    {				// If we are on a straightaway,
      
      // if we are far from the end,
      if (s.to_end * ignore_slowdown > find_bd (s, speed))
      {
	vc = s.v + 50.0;	// pedal to the metal!
      }
      else
      {			// otherwise, adjust speed for the coming turn:
	  if (s.v > 1.02 * speed)	// if we're 2% too fast,
	    vc = brake_ratio * s.v;	// brake hard.
	  else if (s.v < .98 * speed)	// if we're 2% too slow,
	    vc = 1.1 * speed;	// accelerate hard. 1.1
	  else			// if we are very close to speed,
	    vc = .5 * (s.v + speed);	// approach the speed gently.
       }
      
      // Start diving into the corner

      if(s.to_end < (start_dive * s.v * width))
	{
	  if (s.nex_rad > 0.0)	// left
	    lane = dist_from_inside;
	  else
	    lane = width - dist_from_inside;  
	}
      else
	{
	  if (!is_nex_straight(s) && ((s.cur_len * dive_setup_dist) >= s.to_end))
	    {
	      if (s.nex_rad > 0.0)
		lane = width - dist_for_dive;
	      else if (s.nex_rad < 0.0)
		lane = dist_for_dive;
	    }
	}
      
    }
    else
    {				
      // This is when we are in a curve:  (seek correct speed)
      // calculate vc to maintain speed in corner
      speed = curvespeed (s, fabs (s.cur_rad) + width * predict_width);
      
      vc = .5 * (s.v + speed);	

      // calculate distance to end of curve
      if (s.cur_rad > 0.0)
	  to_end = s.to_end * (s.cur_rad + width * predict_width);
      else	
	  to_end = -s.to_end * (s.cur_rad - width * predict_width);
      
      // compute required braking distance and compare:
      if (to_end <= find_bd (s, speed_next))
      {
	vc = s.v - brake_curve_slip;
      }

      if (is_nex_straight (s) && s.to_end <= s.cur_len * (1 - out_of_curve))
      {  
	vc = s.v * 1.1;  
      }
    }

    // New speed limiter if curve after next is tricky

    // calculate the braking distance we will need to slow down from the
    // speed we now have to the speed in the segment after the next

    aftaft_bd = find_bd(s, crvspeed(s.aftaft_rad));

    after_bd = find_bd(s, crvspeed(s.after_rad));

    if (aftaft_bd > (curve_length(s.nex_len, s.nex_rad) 
		    + curve_length(s.after_len, s.after_rad)))
      {
	bd_now = aftaft_bd - (curve_length(s.nex_len, s.nex_rad) +
			      curve_length(s.after_len, s.after_rad));

	if (bd_now > curve_length(s.to_end, s.cur_rad))
	  {
	    vc = s.v * brake_ratio;
	  }
      }
    // else here ? not sure...
    if (after_bd > (curve_length(s.nex_len, s.nex_rad)))  
      {
	// We will have to start braking in this segment.

	// When = Total - lenght of next curve
	bd_now = after_bd - curve_length(s.nex_len, s.nex_rad);

	if (bd_now > curve_length(s.to_end, s.cur_rad))
	  {   
	    vc = s.v * brake_ratio; 
	  }
      }

#ifndef BFOPT
    // Gotta love this passing code... ;-)
    if (s.dead_ahead)		// Change the lane a little if someone's
      if (s.to_lft > s.to_rgt)	// in your way.
	lane -= DELTA_LANE;	// lane must be a static variable
      else
	lane += DELTA_LANE;
  
    
    if (s.lap_flag)
    {    
	if (s.fuel > fuel_last)
	{			
	  // we refueled...use previous as estimate
	  fuel_total += fuel_total / (s.laps_done - 1); 
	}
	else
	{
	  fuel_total += fuel_last - s.fuel;
	}
	fuel_last = s.fuel;
    }
  
    result.request_pit = 0;

    
    if (s.stage != QUALIFYING && (s.damage > 20000) && (s.laps_to_go > 5))  
    {
      result.request_pit = 1;
      result.repair_amount = max(s.damage, (unsigned)s.laps_to_go * 1000);
      result.fuel_amount = max(0,((fuel_total / s.laps_done) * (s.laps_to_go + 1)) - s.fuel);
    }
  
    if (s.stage != QUALIFYING && s.fuel < (fuel_total / (s.laps_done - 1)) && s.laps_done > 1)
    {
      result.request_pit = 1;
      result.fuel_amount = (fuel_total / s.laps_done) * (s.laps_to_go + 1);
      result.repair_amount = (int)result.fuel_amount * 10;
    }  

#endif
    result.alpha = alpha;
  
    result.vc = vc;
    
    traction_control(s, result);
    
#ifdef OPTIMIZE  // Pitting during optimizing is done differently

  // Keep track of the data from the last lap
  lastlaptime = s.lap_time;
  lastdamage = s.damage;
  lastdistance = s.distance;
  
//   result.fuel_amount = 150;
   result.request_pit = 0;

   // If we are damaged or are too close to the edge, abort the
   // race by deliberate crashing
   if((s.damage)||
      (s.to_lft<EdgeLimit)||
      (s.to_rgt<EdgeLimit))
     AbortRace = 1;

   if(AbortRace)
   {
     result.vc = 250.;
     result.alpha = 0.;
   }
#endif //OPTIMIZE

#ifdef SKIDMARKS
    extern int skidmarks;
    skidmarks = 1;

    extern int designated;
    designated = 0;
#endif

    return result;
}










