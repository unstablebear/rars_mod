///// This version of Apex1 is designed for left-handed ovals
///// Maido Remm, ESTONIA (mremm@ebc.ee) June, 1997
///// Optimized for BORS'98 races

//#define TWEAK

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"
#include "os.h"
#define sqr(x)   (x*x)
#define max(x,y)  ((x>y)? x:y)
#define min(x,y)  ((x>y)? y:x)


#ifdef TWEAK
#include <stdio.h>
static double avg_speed, best_speed = 0.0, speed[3];
static int check, i,j,k,l,m,n,o,p,r,q,t,u,v,z;
FILE *fp1;
static long races_done = 0;
#endif
////////////////////////////////////////////////////////////////////////////
///////////////////////////// IMPORTANT CONSTANTS //////////////////////////

static double APEX = 0;	// fraction of curve from end
static double CS = 5.97; // determines speed constant in curves
static double BC1 = 0.011; // determines how long is braking distance on straigth
static double BC2 = 0.032; // determines how long is braking distance on straigth
static double B1 = 19.6; // Big_slip in bias formula
static double B2 = 3; // Exponent in bias formula
static double M1 = 11.5;	// in feets from left
static double M2 = 55; // from right
static double ACC_CRV = 1.1;	// acceleration after apex in curves
static double ACC_STR = 1.9;	// acceleration ( * s.v) in straigth
static double BRK_CRV = 0.85; // decceleration on straigth
static double BRK_STR = 0.9; // decceleration on straigth
static double LOOK = 180; // used for alpha
static double SLOW = 1.0; // reduce bias if racing
extern long lap_count;
static track_desc track;


typedef struct con
{
  char   name[16];      // track name in lowercase letters
  double speed;         // average speed of 7 laps
  int laps_per_tank;    // fuel mileage with 150 lb fuel
  double apex;          // fraction of curve from end
  double cs;            // curvespeed multiplier
  double bc1;           // determines how early we start braking(straigth)
  double bc2;           // determines how early we start braking(curves)
  double m1;            // in feets from left
  double m2;            // in feets from right
  double b1;            // bias (agressiveness in curves)
  double b2;            // exponent in bias formula  
  double acccrv;        // acceleration in curve
  double accstr;        // acceleration on straigth
  double brkcrv;        // decceleration in curve
  double brkstr;        // decceleration in curve
  double look;          // how far ahead to look while setting lane
  double slow;          // how much to reduce agressivness (bias) for race
} con;

static con track_data =
{"default.trk"    ,0    ,10,0.00,5.80,0.014,0.025,10,30,18.0,3.0,1.10,1.90,0.90,0.80,180,0.8};

static con opt_data[] =
{
{"indy500.trk"   ,130.85, 60,0.40,5.72,0.012,0.028, 7,40,22.8,1.7,1.85,1.45,0.86,0.84,178,1.0},
{"loudon.trk"     ,91.89,108,0.59,5.64,0.008,0.024, 8,20,22.6,2.6,1.75,2.00,0.82,0.82,170,1.0},
{"oval2.trk"      ,84.59,189,0.38,5.64,0.008,0.030, 7,20,27.0,3.4,1.15,1.70,0.94,0.94,174,1.0},
{"phoenix.trk"    ,94.67,116,0.36,5.74,0.012,0.022,10,26,23.4,2.9,1.10,1.70,0.98,0.94,178,0.9},
{"michigan.trk"  ,132.96, 71,0.00,5.86,0.016,0.030, 9,50,20.6,2.5,1.95,1.35,0.90,0.80,174,1.0},
{"mlwaukee.trk"   ,94.59,115,0.30,5.92,0.012,0.022,10,22,25.4,2.7,1.90,1.35,0.84,0.80,192,0.9},
{"nazareth.trk"   ,99.58,117,0.51,5.72,0.010,0.018, 9,20,26.0,3.1,1.25,1.35,0.92,0.86,208,0.85},
{"pocono.trk"    ,110.02, 51,0.47,5.60,0.012,0.028,11,50,27.0,2.0,1.15,2.10,0.94,0.84,188,1.0},
{" ", -1}
};

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//////////////////////// GENERAL HELPER FUNCTIONS /////////////////////////

static double seg_length(int segnum) // segment length in feets
{
  if (track.trackin[segnum].radius == 0) return track.trackin[segnum].length;
  return track.trackin[segnum].length * fabs(track.trackin[segnum].radius);  
}


static double toend_length(int segnum, situation &s) // to_end in feets 
{
  if (track.trackin[segnum].radius == 0) return s.to_end; 
  return s.to_end * fabs(track.trackin[segnum].radius);  
}

///// calculate braking distance from speed vnow to speed vthen.
static double find_bd(double vnow, double vthen, int segnum)
{
  if (vnow<vthen)
    return 0.0;  // no need to brake as we are already at the desired speed!
  else if (track.trackin[segnum].radius == 0) // in straigth
    return (BC1 * (vnow+vthen) * (vnow-vthen));
  return (BC2 * (vnow+vthen) * (vnow-vthen));
}

///// calculate racing lane radius
static double ExtR(int segnum)
{
  if (track.trackin[segnum].radius == 0) return 0;
  return (track.width-M1-M2)/(1-cos(track.trackin[segnum].length/2)) + 
    track.trackin[segnum].radius + M1;
}

///// calculate speed in curves:
static double curvespeed(int segnum, situation &s)
{
  if (track.trackin[segnum].radius == 0) return ACC_STR * s.v;
  return (CS * sqrt(fabs(ExtR(segnum))));  
}

///// calculate maximum extension of curve length into next and previous segment
static double ExtL(int segnum)
{
  if (track.trackin[segnum].radius == 0) return 0;
  return (track.width-M1-M2)/(1-cos(track.trackin[segnum].length/2)) *
    sin(track.trackin[segnum].length/2);
}

///// calculate the length of current segment in feets
static double find_Length(int segnum)
{
  int nextseg = (segnum+1)%track.NSEG;
  int prevseg = (segnum-1);

  if (prevseg == -1) prevseg = track.NSEG - 1;
  if (track.trackin[segnum].radius == 0)
    return seg_length(segnum) - ExtL(nextseg) - ExtL(prevseg);
  return (seg_length(segnum) + 2*ExtL(segnum));
}

///// calculate how many feets is left to end of current section
static double find_To_end(int segnum, situation &s)
{
  int nextseg = (segnum+1)%track.NSEG;
  int prevseg = (segnum-1);

  if (prevseg == -1) prevseg = track.NSEG -1;
  if (track.trackin[segnum].radius == 0)	// in straigth
    return s.to_end - ExtL(nextseg);

  else if (segnum < s.seg_ID || (s.seg_ID == 0 && segnum == track.NSEG -1))
				// after curve
    return s.to_end - (track.trackin[nextseg].length - ExtL(segnum));

  else if (segnum > s.seg_ID) // before curve
    return s.to_end + find_Length(segnum) - ExtL(segnum);

  else 			// in curve
    return track.trackin[segnum].radius * s.to_end + ExtL(segnum);
}

////////////////////////////////////////////////////////////////////
///// set the sinusoid *lane* we want to be in curves: /////////////
static void find_lane(int segnum, situation &s, double &lane)
{
  double x;
  double Length = find_Length(segnum);
  double To_end = find_To_end(segnum, s);

  if (To_end < Length/2)
    x = (Length/2 - To_end)*2 / Length; // 0 -> 1
  else
    x = (To_end - Length/2)*2 / Length; // 1 -> 0
  
  if (!track.trackin[segnum].radius)	// straigth
    lane = track.width - M2;
  else lane = (track.width - M1 - M2) * x + M1; // normal curve
}

///////////////////////////////////////////////////////////////////
////////////////// Speed //////////////////////////////////////////
static double find_speed(int segnum,situation &s)
{
  int nextseg = (segnum+1)%track.NSEG;
  int prevseg = (segnum-1);
  if (prevseg == -1) prevseg = track.NSEG - 1;
  
  double To_end = find_To_end(segnum, s);
  double ALength = APEX*find_Length(segnum);

  if (track.trackin[segnum].radius != 0 && To_end < ALength)
    return s.v * ACC_CRV;
  if (To_end < find_bd(s.v,curvespeed(nextseg, s),segnum)){
    if (!track.trackin[segnum].radius)
      return max(s.v * BRK_STR, curvespeed(nextseg, s));
    else	return max(s.v * BRK_CRV, curvespeed(nextseg, s));
  }
  else return curvespeed(segnum, s);
}
//////////////// Bias /////////////////////////
///// calculate  maximum bias for curves:
static double Bias(int segnum, situation &s)
{
  double b;
  double To_end = find_To_end(segnum, s);
  double ALength = APEX*find_Length(segnum);
  double cs = curvespeed(segnum,s);
  if (track.trackin[segnum].radius == 0) return 0;
  else if (track.trackin[segnum].radius > 0)
    b = atan(B1/cs)* pow(s.v/cs,B2);
  else    
    b = -atan(B1/cs)* pow(s.v/cs,B2);
  
  if (s.stage == RACING) b *= SLOW;
  if (To_end < ALength) b *= To_end/ALength; //reduce bias smoothly
  return b;
}

///// A look-ahead code: //////////////////////////////////

static void avoid_cars(int segnum, situation &s, double &lane, double &vc)
{
  int danger = 0;
  int kount = 0;
  double vc1;
  double v[4], d[4], x[4], y[4], vx[4], vy[4];
  for(int i=0;i<4;i++)
    if(s.nearby[i].who<16){
      x[i]=s.nearby[i].rel_x;		// distance to right (or left if < 0)
      y[i]=s.nearby[i].rel_y;		// distance ahead, always positive
      vx[i]=s.nearby[i].rel_xdot;	// relative lateral speed component
      vy[i]=s.nearby[i].rel_ydot;	// relative forward speed (negative)
      d[i]=sqrt(x[i]*x[i] + y[i]*y[i]);	// distance to other car
      v[i]=s.nearby[i].v;			// absolute speed
      
      if (vy[i] > -1)  continue;	// ignore faster cars
      else if (d[i] > 1.2*find_bd(s.v, s.v+vy[i]+5,segnum))
	continue;	// check whether you need to brake
      ++kount;
      if (fabs(x[i])>1.5*CARWID && x[i]*vx[i]>-10 && 
	  fabs(vx[i])>10 && vy[i]>-10)
	continue; // if slow car ahead, reserve more safety space
      else if (fabs(x[i])>CARWID && x[i]*vx[i]>-10 && 
	       fabs(vx[i])>10 && vy[i]<=-10)
	continue; // if fast car ahead, reserve less safety space
      else danger = 1;	// brake if trouble car within braking distance
    }

//////////////////////// Danger! What to do? ///////////////////////////////

  if (danger == 1){
    if(kount > 2 || s.damage > 25000)
      vc1 = min((s.v + vy[0] +5),(s.v + vy[1] +5));
    else{
      vc1 = min((s.v + vy[0]+5),((s.damage >20000?0.9:0.95)*vc));
      if (!s.cur_rad){  //straigth => pass!
	if (x[0] < 0)	// car in left
	  if (s.to_lft + x[0] < track.width - 1.5* CARWID)
				//if we have room in right, turn right
	    lane += CARWID;
	  else	lane -= CARWID;
	
	else if (x[0] >= 0)	// car in right
	  if (s.to_lft + x[0] > 1.5* CARWID) // if we have room in left
	    lane -= CARWID; //turn left
	  else	lane += CARWID;
      }
    } // end of kount < 2
  }// end of danger
  else vc1 = vc; //this executes if no danger seen
  
  vc1 = max(vc1,5);  // To avoid stopping
  // behind very slow car
  vc = vc1;

} // end of avoid_cars function


static void keep_control(situation &s, con_vec &result)
{
//  double PUSH = 1.06 + (9e8 - sqr(s.damage))*0.03/9e8 + 
//	(lap_count - s.laps_to_go)*0.015/lap_count;
  double A = sqrt(max(0,1.105-sqr(s.cen_a/g))); // max allowed tan_a
  double B = g*A;
  double C = (s.v + B); // max allowed vc

  if (result.vc>s.v)
    result.vc = min(C,result.vc);
}
//////////////////////////////////////////////////////////////////////
//////////////////////////// MAIN FUNCTION ///////////////////////////

con_vec Apex1(situation &s)
{
  con_vec result = { 0.0f, 0.0f, 0.0f, 0, 0 }; // This is what is returned.
  static int firstcall = 1;

  if (firstcall){
    my_name_is("Apex1");
    firstcall = 0;                  //  theres only one first call
    return result;                  //  must return an answer
  }

  if (s.starting) track = get_track_description();

#ifndef TWEAK
////////////////// Track-specific variables ////////////////////////
  if(s.starting){
    // get the pre-existing parameters if track data exists
    for(int i=0; opt_data[i].speed >= 0; i++)
      if( !strcmp(opt_data[i].name,track.sName)){
	track_data = opt_data[i];  // get the pre-existing parameters
	break;       // finish
      }
    
    APEX = track_data.apex;
    CS = track_data.cs;
    BC1 = track_data.bc1;
    BC2 = track_data.bc2;
    B1 = track_data.b1;
    B2 = track_data.b2;
    M1 = track_data.m1;
    M2 = track_data.m2;
    ACC_CRV = track_data.acccrv;
    ACC_STR = track_data.accstr;
    BRK_CRV = track_data.brkcrv;
    BRK_STR = track_data.brkstr;
    LOOK = track_data.look;
    SLOW = track_data.slow;
  }
#endif
///////////////////////// Optimization of car /////////////////////////////////
#ifdef TWEAK
  if (s.starting) ++races_done;
  if (s.starting && check == 0){ // in start of every race change constants

    j =  os.PickRandom() % 60;
    k = (os.PickRandom() % 119)% 20;
    l = (os.PickRandom() % 59) % 10;
    m = (os.PickRandom() % 137)% 20;
    n = (os.PickRandom() % 41) % 10;
    o = (os.PickRandom() % 179)% 20;
    p = (os.PickRandom() % 149)% 50;
    q = (os.PickRandom() % 71) % 20;
    r = (os.PickRandom() % 101)% 20;
    t = (os.PickRandom() % 93) % 20;
    u = (os.PickRandom() % 51) % 10;
    v = (os.PickRandom() % 91) % 10;
    z = (os.PickRandom() % 117)% 20; 

    APEX = 0.0 + j * .01;
    CS  =  5.6 + k * .02;
    BC1 = 0.008 + l * .001;
    BC2 = 0.012 + m * .001;
    M1 = 5 + n * 1;
    M2 = 20 + o * 2;
    B1 = 18 + p * .2;
    B2 = 1.5 + q * .1;
    ACC_CRV = 1.1 + r * 0.05;
    ACC_STR = 1.2 + t * 0.05;
    BRK_CRV = 0.8 + u * 0.02;
    BRK_STR = 0.8 + v * 0.02;
    LOOK = 170 + z * 2;
  }

  if (s.lap_flag && !s.laps_to_go){
    speed[check] = MPH_FPS*lap_count*track.length/(s.time_count-s.start_time);
    if (speed[check] > best_speed - .2 && !s.damage){
      check  += 1;
      if (check == 3){
	avg_speed = (speed[0] + speed[1] + speed[2])/3;
	if(avg_speed > best_speed){
	  best_speed = avg_speed;
	  fp1 = fopen("apex1.dat","at");
          fprintf(fp1,"#%ld#", races_done);
          fprintf(fp1,"{\"%s\"\t,%.2f,0,", currentTrack->fileName, best_speed);
          fprintf(fp1,"%.2f,%1.2f,%.3f,%.3f,", APEX, CS, BC1, BC2);
	  fprintf(fp1,"%2d,%2d,%2.1f,%1.1f,",int(M1), int(M2), B1, B2);
	  fprintf(fp1,"%1.2f,%1.2f,%.2f,%.2f,",ACC_CRV,ACC_STR,BRK_CRV,BRK_STR);
	  fprintf(fp1,"%d,%.1f},\n", int(LOOK), SLOW);
	  fclose(fp1);
	}
	check = 0;
      }
    }
    else check=0;
  }
#endif
//////////////////////// End of optimization code /////////////////////////////

  if (stuck(s.backward,s.v,s.vn,s.to_lft,s.to_rgt,&result.alpha,&result.vc))
    return result;

  // name the segments according to current position:
  int seg_num = s.seg_ID;
  int next_seg = (seg_num+1)%track.NSEG;
  int prev_seg = (seg_num-1);
  if (prev_seg == -1) prev_seg = track.NSEG - 1;

  // define distances in feets instead of radians
  double toend = toend_length(seg_num, s);
  double length = seg_length(seg_num);

  // rename segment numbers while extending curves:
  if (toend < ExtL(next_seg))
    seg_num = seg_num+1;
  else if (length-toend < ExtL(prev_seg))
    seg_num = seg_num - 1;
  
  seg_num = (seg_num + track.NSEG)%track.NSEG; //for transition over seg 0
  next_seg = (seg_num+1)%track.NSEG;	// recalculate this
  prev_seg = (seg_num-1);		// and this
  if (prev_seg == -1) prev_seg = track.NSEG - 1;



  double lane, vc, alpha, bias;

  find_lane(seg_num, s, lane);
  vc = find_speed(seg_num, s);
  avoid_cars(seg_num, s, lane, vc);
  bias = Bias(seg_num, s);
  alpha = (s.to_lft-lane)/LOOK - asin(s.vn/s.v);
  alpha += bias;

  result.alpha = alpha;
  result.vc = vc;
  keep_control(s, result);
#ifdef TWEAK
  if (s.damage > 10) result.alpha += 1.0;
#endif

  if(s.starting){
    if(s.stage == QUALIFYING) 
      result.fuel_amount = 
	args.m_iNumQualLap * MAX_FUEL/track_data.laps_per_tank + 3;
    else 
      result.fuel_amount = MAX_FUEL;
  }
  result.request_pit = 0;

#ifndef TWEAK  
  if (s.stage != QUALIFYING && (s.damage > 27000 || s.fuel < 5)){
     result.request_pit = 1;
     result.repair_amount=s.damage;
     result.fuel_amount = MAX_FUEL;
  }  
#endif

  return result;
}
