//  Rachel221.cpp
//  Mark Nau
//  nau@treyarch.com   (W)
//  nauz@ix.netcom.com (H)
//
//  Everything here is public.  Whenever I "borrow" a concept, I try to give credit.
//  Anyone who cares to borrow concepts from here is encouraged to do so, and is
//  also encouraged to make their own developments public.
//
//
//  Version History:
//  v1.0,         - Basic car, stays in middle of track
//
//  v1.1,         - Added many tunable variables and did some tuning
//
//  v2.0,         - "Greatest circle" cornering implemented.  The car will begin cornering
//                   in the straightaway just before a turn.  It tries to figure out the
//                   largest-radius circle that it can follow through the turn.
//
//  v2.1, 03Jun96 - "Fast corner" modification: doesn't bother with greatest circle through
//                   corners that can be taken above a certain speed.
//                   "Shooting out" modification: doesn't follow the circle anymore once
//                   closer than a set number of radians from the end of the turn
//                   Time trials run to tune variable settings. 
//
//  v2.2, 11May98 - Drag-out removed
//                  Braking distance amd cornering speed calculators altered
//                  "Avoidance" code slightly refined
//
//  v2.21 12May98 - Pit strategy added
//                  Time trials run to tune performance on oval2 and milwaukee
//
//
// Yes, a two-year gap.  The new company I work for shipped our game (Die by the Sword)
// not too long ago, so the pressure is off and I have time for recreation again!
// Lest anyone judge the company by the code I'm writing here, let me make clear
// that by vocation I am a game-designer, NOT a coder.
//
//
//                                       MODEL SUMMARY
//
// Based upon the M.Timins tutorial cars.
//
// Modifications to the Timins concept include:
//
// Trying to follow the greatest-radius circle that will fit around a given corner.
// Different gain and damping variables for straightaways and curves.
// Tinkering with gain, damping, and speed under certain situations where a crash into
//    the wall or other non-optimal path might occur otherwise.
// Accelerating full-speed out of corners.
// A very basic danger handler that just slows based upon s.dead_ahead.
// A rather basic pit strategy.
//
//
//
//                                       THINGS TO DO
//
// Improve the collision avoidance.
// Replace the greatest-circle code with something that dynamically figures when to dive.
// Provide for the possibility of changing variable settings for different tracks.
// Provide for the possibility of changing cornering speeds dynamically based upon 
//    performance data gathered during the race.
// Handle the two-consecutive-turns situation better.
// Convert from lane-based position data to absolute position data ala Ralph Scott.
// Allow for multiple ways of taking turns (greatest circle, inside, outside) and select one
//    based upon traffic position and performance data.
// Implement changing the car's agressiveness based upon damage level and where the car stands
//    in current track position.
// Make better decisions regarding what lane to seek when headed toward a turn.
// Look into the possibility of adopting fuel-efficiency measures if it means the car will be
//    able to get one less pit stop by driving economically.
// ? Create a genetic algorithm to tune the variable settings for various tracks.
//
//
//
// v2.21 is suitable only for oval tracks
// Best-lap speeds for various tracks, running under rars0.71b:
//
// figure8 :  87.02
// indy500 : 130.88
// mlwaukee:  94.33
// monza-76: 101.79
// mosport :  94.41
// oval2   :  84.26
// phoenix :  95.97
// pocono  : 110.89
// v01     :  73.06
// v03     :  68.49
//
// She has trouble on Nazareth, Michigan, and Loudon for various reasons that I
// will sort out.




//debugging defines
//#define DEBUG_INFO
//#define CORNER_INFO
//#define TRACK_INFO


//-------------------------------------------------------------
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "car.h"
#include "track.h"
#include <iostream.h>
#include <iomanip.h>
#include <fstream.h>

#define min(x,y) (((x)>(y))?(y):(x))
#define max(x,y) (((x)>(y))?(x):(y))

//----------------------------------structs-----------------------------------

static struct corn_info {double dive, add_r;} ci;

typedef struct 
{
  int request;
  double repair;
	double fuel;
} pit_info;


//----------------------------------consts-----------------------------------
const double MAX_SPEED = 400.0;      // this is the "cornering speed" for a straightaway

const double FAST_CORNER = 180.0;  // if cornering speed of corner is at least this fast, don't
                                   // go through the "greatest circle stuff"


const double TURN_BRK_SLIP_LIM = 30;  //maximum braking on turn
const double BRK_SLIP_LIM =  38;      //maximum braking on straight
const double ACC_SLIP_LIM =  38;      //maximum acceleration

const double SPEED_EASE = .6;     //how fast you ease toward target speed (0-1)
const double ABOVE_SPD =  12;     //this much above target speed, slam on brakes
const double BELOW_SPD =   8;      //this much below target speed, floor it

const double BDIST_K = .0122;      // multiplicitive factor for braking distance calc


// following two apply only to "double turns"
const double CONTROL_TURN = 0.90; // slow up a bit on entry of first turn
const double CONTROL_EXIT = 1.04; // counteract the slowing after apex of first turn (if second
                                  // turn is a faster corner)

const double STRT_GAIN = 0.25;
const double STRT_DAMP = 0.50;

const double CORN_GAIN = 0.60;
const double CORN_DAMP = 0.40;

//const double OUT_BUFF  = 40.0;   // distance from outside rail to aim for just before cornering
                                   // (now #defined as a function of width)

const double SAFE_BUFF = 12.0;     // on exit of corner, strive to stay at  least this far from rails
const double IN_BUFF  = 8.0;       // aim point at apex, distance from inside rail


const double WALL_DANGER = 14;
const double IN_DANGER = 5;

const double BIG_SLIP = 16.0;

const double SHOOT_GLIDE = 1.2;      // while shooting out of corner, max  delta laterally
//------------------------GLOBAL---------------------------------------------
static double track_width;
static double CS_K1,cs_k;               // multiplicitive factor for corneringspeed calc
static double OUT_BUFF;
static double OUT_CORNER,o_corner;          // if past apex of curve, and fewer than this many radians left, just zoom out
	
static ofstream fout;

static track_desc track;
//---------------------------------------------------------------------------



//------------CREW CHIEF
static double fuel_needed (double laps, double lap_length)
{
	double fuel_per_lap, fudge_factor, fudge_add, needed;

	fudge_factor = 1.02;
	fudge_add = 0.5;
	fuel_per_lap = .0002138 * lap_length;
	needed = (fuel_per_lap * laps * fudge_factor) + fudge_add;

	return needed;
}

static pit_info pit_decision (double fuel, double damage, int l_to_go)
{
	pit_info decision;
	decision.request = 0;
	decision.fuel = 0;
	decision.repair = 0;
	double ok_damage = max (22000, 29500 - (l_to_go * 150));
	double repair_to = max (0, 28000 - (l_to_go * 180));


	if ((damage > ok_damage) || (fuel < fuel_needed(1.1,track.length)))
	{
		decision.request = 1;
		decision.repair = damage - repair_to;
		decision.fuel = fuel_needed(l_to_go,track.length);
		if (decision.repair < decision.fuel * 10)
			decision.repair = decision.fuel * 10;
	}
	return decision;
}



//-----------GET TRACK INFO
static void track_init(void)
{
	track = get_track_description();
}



static double divide(double n, double d)
{
// after my third "zero divide" crash, I decided to use this instead

  if ((d >=0.0) && ( d< .01)) d = .01;
  if ((d < 0.0) && ( d> -.01)) d = -.01;

  return n/d;
}



static double lane_glide(double tot_dist, double rem_dist, double start_lane, double end_lane)
{

// given a distance to cover, and a start and end lane, will
// return the proper lane to be in given how far along the
// stretch you are

  double fraction_done, lane_delta;

  fraction_done = divide ((tot_dist - rem_dist) , tot_dist);
  lane_delta = end_lane - start_lane;

  return start_lane + (fraction_done * lane_delta);
}


static double calc_vc(double target, double current, double brake_lim)
{
  if (current > target + ABOVE_SPD)     // going WAY too fast
    return(current - brake_lim);
  if (current < target - BELOW_SPD)      // going WAY too slow
    return(current + ACC_SLIP_LIM);
	return ((SPEED_EASE * target) + ((1-SPEED_EASE) * current));
  //return(((2*target) + current) / 3);   // ease toward target speed
}



static double real_radius(double radius, double added)
{
//figures out the "real" radius to use given the turn's radius (which might be negative) and
//the added distance that you have due to the fact that you're not right on the inner wall.
	if (radius < 0.0) return -radius + added;
	else return radius + added;
}



static double corner_speed(double radius)
{
// returns the speed to take a corner at, given its radius and the added radius
// gained from starting dive towards the outside of the track
	return cs_k * sqrt(radius);
}




static void calc_corn1(double theta, double wide)
{

// Figures the effective "extra radius" and "dive length"
// for a corner of theta radians and track of a given width.
// "Add_r" is how much you can add to the corner's radius when
// figuring the cornering speed.
// "Dive" is how much before the corner actually starts
// you should start the turn.

  if (theta > PI/2) theta = PI/2;
	ci.add_r = divide(wide , 1-cos(theta));
  ci.dive  = ci.add_r * sin(theta);
}


//------------BRAKING DISTANCE
static double brake_dist(double v0, double v1)
{
// Returns the proper distance to give yourself for braking.
// v0 is starting speed, v1 is target speed.
// Based on Ralph Scott's braking distance explanation.

	if (v0<v1)
		return 0.0;  

	return BDIST_K * (v0-v1) * (v0+v1);
}



con_vec Rach221(situation &s)
{
  const char name[] = "Rach221";    
  static int init_flag = 1;          // First call flag
  static int last_seg = -99;         // Segment the car was last on
  static double lane;                // Lane to seek.  Dist from left wall
  static double togo_start;          // Dist from end starting segment
  static double next_wide_aim;       // Aim (from left wall) for start dive
  static double next_wide_start;     // Dist from left wall starting segment
  static double next_in_aim;         // Aim (from left wall) for apex
  static double next_r;              // Effective radius of next turn
  static double next_cd;             // distance of cornering, to apex
  static double next_cs;             // Cornering speed of next turn
  static double this_wide_aim;       // Aim (from left wall) for start dive
  static double this_wide_start;     // Dist from left wall starting segment
  static double this_in_aim;         // Aim (from left wall) for apex
  static double this_cd;             // distance of cornering, to apex
  static double this_cs;             // Cornering speed of next turn
	static int double_corner;          // Flag for double corner situation
  static int first_turn = 1;         // I don't want to dive towards the apex on the first turn - crash city.
	con_vec result;                    // What will be returned
  double alpha, vc, speed=0.0;       // Working components of result
  double theta, bdst, to_apex, brake_lim=0.0, lane_gonna;    // Working variables
  double steer_gain=0.0, damp_gain=0.0, bias=0.0; // Alpha working components
  int danger;
	static int ahead; //counts how many consecutive times we've had a dead.ahead warning
	int caution, impatience;

  if (init_flag)  {                  // First call, so initialize
    my_name_is(name);
    init_flag = 0;
#ifdef DEBUG_INFO
    fout.open("debug.txt");
#endif
		result.alpha = 0;
    result.vc = 0;
    return result;
  }

// Using provided "stuck" routine:
  if (stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
    return result;


// starting stuff
	if (s.starting) {
		track_init();
		track_width = s.to_lft + s.to_rgt;
		if (s.stage == QUALIFYING) {
      CS_K1 = 5.85;
			OUT_CORNER  = PI * .23;
			OUT_BUFF = (track_width * 7/8) - 20;
			result.fuel_amount = fuel_needed(12,track.length);
		}
		else {
			CS_K1 = 5.83;
			OUT_CORNER = PI * .21;
			OUT_BUFF = (track_width * .75) - 7;
			result.fuel_amount = MAX_FUEL;
		}
	}


// Danger check

	o_corner = OUT_CORNER;
	cs_k = CS_K1;


	double acceptable_damage = max(28500 - (150 * s.laps_to_go),8000);
	

  danger = 0;
	caution = 3;
	impatience = 6;
	if (s.stage != QUALIFYING) {
		if ((long)s.damage > acceptable_damage) {
	    danger = 1;  // danger class 1 - damage is above acceptable limits
			caution = 6;
			o_corner = .15 * PI;
			cs_k = 5.75;
			impatience = 999;
			}


// most possibly the lamest avoidance code in existance!

		if (s.dead_ahead) {
	    danger += 2; // danger class 2 - someone in front of me
                 // danger class 3 - both conditions
			o_corner = .15 * PI;
			cs_k = 5.75;
			ahead++;
			}

		else
			ahead = 0;
	}



// Normal racing situation

// First, we see if we've entered a new segment.  If so, we calculate
// the variables that will determine the cars behaviour in this segment.	


  if (s.seg_ID != last_seg) {        // Just entered a new segment
    double_corner = 0;
		next_wide_start = s.to_lft;      // record what lane starting segemnt
    togo_start = s.to_end;           // how far to end of segment
		last_seg = s.seg_ID;

    if (s.cur_rad == 0.0) {          // on a straight
      if (s.nex_rad < 0.0) {         // a right turn coming next
        theta = s.nex_len / 2;
        lane_gonna = s.to_lft;
        if (lane_gonna < SAFE_BUFF) lane_gonna = SAFE_BUFF;
        if (lane_gonna > track_width - SAFE_BUFF) lane_gonna = track_width-SAFE_BUFF;
        if ((corner_speed(real_radius(s.nex_rad,track_width-lane_gonna)) > FAST_CORNER) || (first_turn)){
          first_turn = 0;
					ci.add_r = 0;
					ci.dive = 0;
					next_wide_aim = lane_gonna;
          next_in_aim = lane_gonna;
          next_r = real_radius(s.nex_rad, track_width - lane_gonna);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
        else {                       
          calc_corn1(theta, track_width-OUT_BUFF);
          next_wide_aim = OUT_BUFF;
          next_in_aim = track_width-IN_BUFF;
					next_r = real_radius(s.nex_rad, ci.add_r);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
      }

      else if (s.nex_rad > 0.0) {    // a left turn coming next
        theta = s.nex_len / 2;
        lane_gonna = s.to_lft;
        if (lane_gonna < SAFE_BUFF) lane_gonna = SAFE_BUFF;
        if (lane_gonna > track_width - SAFE_BUFF) lane_gonna = track_width-SAFE_BUFF;
        if ((corner_speed(real_radius(s.nex_rad,lane_gonna)) > FAST_CORNER)) {
					first_turn = 0;
          ci.add_r = 0;
					ci.dive = 0;
					next_wide_aim = lane_gonna;
          next_in_aim = lane_gonna;
          next_r = real_radius(s.nex_rad,lane_gonna);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
        else {
          calc_corn1(theta, track_width-OUT_BUFF);
          next_wide_aim = track_width-OUT_BUFF;
          next_in_aim = IN_BUFF;
          next_r = real_radius(s.nex_rad,ci.add_r);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
      }
    }
    else if (s.cur_rad < 0.0) {      // on a right turn
      this_wide_aim = next_wide_aim;
      this_wide_start = next_wide_start;
      this_in_aim = next_in_aim;
      this_cd = next_cd;
      this_cs = next_cs;

#ifdef TRACK_INFO
			//Output debug info
			fout << "seg : "<< s.seg_ID;
			fout << "crad: "<< s.cur_rad <<" \t";
			fout << "cs  : "<< this_cs <<" \t";
			fout << endl;
#endif


      if (s.nex_rad == 0.0) {        // a straight coming up
        next_cs = MAX_SPEED;
      }

      else if (s.nex_rad < 0.0) {    // another right coming up
        double_corner = 1;
        this_cs *= CONTROL_TURN;
        theta = s.nex_len / 2;
        next_wide_aim = OUT_BUFF;
        next_in_aim = OUT_BUFF;
        next_r = real_radius(s.nex_rad,s.to_rgt);
        next_cs = corner_speed(next_r);
        next_cd = next_r * theta;
      }

      else if (s.nex_rad > 0.0) {    // a left coming up
        theta = s.nex_len / 2;
        if (corner_speed(real_radius(s.nex_rad,s.to_lft)) > FAST_CORNER) {
          ci.add_r = 0;
					ci.dive = 0;
					next_wide_aim = s.to_lft;
          next_in_aim = s.to_lft;
          next_r = real_radius(s.nex_rad,s.to_lft);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
        else {
          calc_corn1(theta, track_width-OUT_BUFF);
          next_wide_aim = track_width-OUT_BUFF;
          next_in_aim = IN_BUFF;
          next_r = real_radius(s.nex_rad,ci.add_r);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
      }
    }
    else if (s.cur_rad > 0.0) {      // on a left turn
			this_wide_aim = next_wide_aim;
      this_wide_start = next_wide_start;
      this_in_aim = next_in_aim;
      this_cd = next_cd;
      this_cs = next_cs;

#ifdef TRACK_INFO
			fout << "seg : "<< s.seg_ID;
			fout << "crad: "<< s.cur_rad <<" \t";
			fout << "cs  : "<< this_cs <<" \t";
			fout << endl;
#endif

      if (s.nex_rad == 0.0) {        // a straight coming up
        next_cs = MAX_SPEED;
      }

      else if (s.nex_rad > 0.0) {    // another left coming up
        double_corner = 1;
        this_cs *= CONTROL_TURN;
        theta = s.nex_len / 2;
        next_wide_aim = track_width - OUT_BUFF;
        next_in_aim = track_width - OUT_BUFF;
        next_r = real_radius(s.nex_rad, s.to_lft);
        next_cs = corner_speed(next_r);
        next_cd = next_r * theta;
      }

      else if (s.nex_rad < 0.0) {    // a right coming up
        theta = s.nex_len / 2;
        if (corner_speed(real_radius(s.nex_rad,s.to_rgt)) > FAST_CORNER) {
          ci.add_r = 0;
					ci.dive = 0;
					next_wide_aim = s.to_lft;
          next_in_aim = s.to_lft;
          next_r = real_radius(s.nex_rad , s.to_rgt);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
        else {
          calc_corn1(theta, track_width-OUT_BUFF);
          next_wide_aim = OUT_BUFF;
          next_in_aim = track_width-IN_BUFF;
          next_r = real_radius(s.nex_rad,ci.add_r);
          next_cs = corner_speed(next_r);
          next_cd = next_r * theta;
        }
      }
    }
  }

// We use the pre-computed data to guide the car's behaviour

  if (s.cur_rad == 0) {              // on a straight
    brake_lim = BRK_SLIP_LIM;
    bdst = brake_dist (s.v, next_cs);
    if (s.to_end > bdst + ci.dive) {// far from the turn
      lane = lane_glide(togo_start, s.to_end, next_wide_start, next_wide_aim);
      speed = MAX_SPEED;
      steer_gain = STRT_GAIN;
      damp_gain =  STRT_DAMP;
      bias = 0;
    }
    else if (s.to_end > ci.dive) {  // braking zone
      lane = s.to_lft;
      speed = next_cs;
      steer_gain = 0;
      damp_gain = 0;
      bias = 0;
    }
    else {                           // dive zone
      lane = lane_glide(next_cd, next_cd - ci.dive + s.to_end, next_wide_aim, next_in_aim);
      speed = next_cs;
      steer_gain = CORN_GAIN;
      damp_gain =  CORN_DAMP;
			bias = atan(divide(BIG_SLIP,speed));
			bias = (s.nex_rad>0) ? bias : -bias; 
    }

  }
  else if (s.cur_rad < 0.0) {        // right-hand turn
    brake_lim = TURN_BRK_SLIP_LIM;    
    if (s.to_end < (s.cur_len/2)) {      // past the apex
      if ((s.to_end < o_corner) && (s.nex_rad == 0.0)) {    // near start of a straightaway
        lane = s.to_lft;
        lane -= SHOOT_GLIDE;
        speed = MAX_SPEED;
        steer_gain = CORN_GAIN;
        damp_gain = CORN_DAMP;
			  bias = -atan(divide(BIG_SLIP,speed));
      }
      else {
        lane = lane_glide (s.cur_len/2, s.to_end, this_in_aim, this_wide_aim);
        speed = min(double_corner ? this_cs * CONTROL_EXIT : this_cs,next_cs);
        steer_gain = CORN_GAIN;
        damp_gain =  CORN_DAMP;
			  bias = -atan(divide(BIG_SLIP,speed));
      }
    }
    else {                           // approaching apex
      to_apex = s.to_end - (s.cur_len/2);
      lane = lane_glide (s.cur_len/2, to_apex, this_wide_start, this_in_aim);
      speed = this_cs;
      steer_gain = CORN_GAIN;
      damp_gain =  CORN_DAMP;
			bias = -atan(divide(BIG_SLIP,speed));
    }

  }
  else if (s.cur_rad > 0.0) {        // left-hand turn
    brake_lim = TURN_BRK_SLIP_LIM;        
    if (s.to_end < (s.cur_len/2)) {      // past the apex
      if ((s.to_end < o_corner) && (s.nex_rad == 0.0)) {    // near start of a straightaway
        lane = s.to_lft;
        lane += SHOOT_GLIDE;
        speed = MAX_SPEED;
        steer_gain = CORN_GAIN;
        damp_gain = CORN_DAMP;
			  bias = atan(divide(BIG_SLIP,speed));
      }
      else {
        lane = lane_glide (s.cur_len/2, s.to_end, this_in_aim, this_wide_aim);
        speed = min(double_corner ? this_cs * CONTROL_EXIT : this_cs,next_cs);
        steer_gain = CORN_GAIN;
        damp_gain =  CORN_DAMP;
        bias = atan(divide(BIG_SLIP,speed));
      }
    }
    else {                           // approaching apex
      to_apex = s.to_end - (s.cur_len/2);
      lane = lane_glide (s.cur_len/2, to_apex, this_wide_start, this_in_aim);
      speed = this_cs;
      steer_gain = CORN_GAIN;
      damp_gain =  CORN_DAMP;
      bias = atan(divide(BIG_SLIP,speed));
    }
  }



// Danger handler, based upon previously calculated danger class
  if (danger == 3) {
  	if (speed != MAX_SPEED)  // Danger class 3, slow up to 90%
			speed *= .90;
		else                     // Unless I'm supposed to be gunning it, then maintain speed
			speed = s.v;
	}

  else if (danger == 2)
		if (ahead < impatience) {         
			if (speed != MAX_SPEED) {
				speed *= (.99 - (min(ahead,caution) * .02));
			}
			else
				speed = s.v;
	}
	 



#ifdef CORNER_INFO
	if (s.cur_rad != 0)
		{
		fout << "seg: " << s.seg_ID << "\t";
		fout << "cs: " << this_cs << " \t";
		fout << "v : " << s.v << " \t";
		fout << "lane: " << lane << " \t";
		fout << "tolft: " << s.to_lft << " \t";
		}
#endif


// ckeck for potential wall-banging situations

	double to_in = 999;
	
	if (s.cur_rad > 0)
		to_in  = s.to_lft;
	
	if (s.cur_rad < 0)
		to_in = s.to_rgt;		
	

//less bias (Les Bias?) near inside wall
	if (to_in < IN_DANGER) {
		bias *= .7;
		steer_gain += .10;
		}


//more cornering near walls on straights
	if (s.cur_rad == 0) {
		if (s.to_lft < WALL_DANGER) {
			steer_gain += .25;
			damp_gain *= .5;
			bias = -atan(divide(BIG_SLIP,speed));
			}

		else if (s.to_rgt < WALL_DANGER) {
			steer_gain += .25;
			damp_gain *= .5;
      bias = atan(divide(BIG_SLIP,speed));
			}
		}




  vc = calc_vc(speed,s.v,brake_lim);
  alpha = steer_gain * (divide((s.to_lft - lane),track_width)) - damp_gain * divide(s.vn,s.v) + bias;




// Set vc and alpha
  result.vc = vc;   result.alpha = alpha;
	result.request_pit=0;


//pit intelligence
	pit_info pit = pit_decision(s.fuel, s.damage, s.laps_to_go);	
	if (pit.request)
		{
		result.fuel_amount = pit.fuel;
		result.repair_amount = int(pit.repair);
		result.request_pit = pit.request;
		}


  return result;
}


