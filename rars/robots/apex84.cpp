// apex8.cpp, robot Apex8
// Maido Remm, ESTONIA (mremm@ebc.ee) Submitted for F1 2000 season
// No need for data files

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
//#include "os.h"
#include "track.h"
//#include "race_manager.h"

#define sqr(x)   ((x)*(x))
#define max(x,y) (((x)<(y))?(y):(x))
#define min(x,y) (((x)>(y))?(y):(x))
//#define TWEAK

static track_desc track; // get track.width
//extern long qlap_count;

// those values are in trk:
static double APEX;       // fraction of curve from end
static double CS;   //curvespeed multiplier
static double BC1; // determines how early we start braking(straigth)
static double BC2; // determines how early we start braking(curves)
static double M1;  // in feets from left
static double M2;  // in feets from right
static double ACC_CRV; // acceleration in curve
static double BRK_CRV; // decceleration in curve
static double END_STR; // in feets from the end of straigth
// same for all tracks:
static double END_CRV = 70;     // in feets from the end of curve
static double ACC_STR = 2.0;    // acceleration ( * s.v) on straigth
static double BRK_STR = 0.8;    // deceleration on straigth


struct con
{
  char   name[16];      // track name in lowercase letters
  double speed;         // average speed of 100 miles
  double lb_per_lap;    // fuel mileage
  double apex;          // fraction of curve from end
  double cs;            // curvespeed multiplier
  double bc1;           // determines how early we start braking(straigth)
  double bc2;           // determines how early we start braking(curves)
  double m1;            // in feets from left
  double m2;            // in feets from right
  double acccrv;        // acceleration in curve
  double brkcrv;        // decceleration in curve
  double endstr;        // in feets from the end of straigth
};

static con trk =
{"default.trk"  ,0      ,9.00,.30,5.80,.014,.025,10,30,1.10,.90,20};
static con opt_data[] =
{
{"adelaide.trk" ,75.141 ,3.69,.49,6.75,.014,.022,11,32,1.12,.90, 8},
{"albert.trk"   ,79.44  ,5.17,.56,6.30,.013,.018,14,28,1.18,.87,10},
{"albrtprk.trk" ,89.224 ,4.84,.53,6.55,.010,.018, 7,36,1.19,.94,12},
{"austria.trk"  ,80.618 ,4.17,.54,6.15,.013,.012, 5,30,1.10,.86,36},
{"barcelon.trk" ,80.157 ,4.55,.13,6.25,.012,.018,12,38,1.22,.92,10},
{"brands.trk"   ,81.483 ,3.57,.51,6.00,.013,.026, 5,30,1.17,.94,32},
{"brazil.trk"   ,72.934 ,4.17,.36,6.15,.013,.010, 5,22,1.16,.85,26},
{"buenos.trk"   ,72.444 ,4.17,.40,6.05,.013,.028, 5,20,1.27,.94,16},
{"cstlcomb.trk" ,88.657 ,2.68,.51,6.75,.012,.028, 7,26,1.06,.93,18},
{"doningtn.trk" ,75.767 ,4.05,.47,6.65,.013,.024,11,28,1.07,.91,24},
{"elev.trk"     ,69.174 ,2.88,.49,6.00,.012,.016, 8,32,1.17,.94,10},
{"elev2.trk"    ,75.285 ,4.92,.34,6.15,.013,.014,13,34,1.11,.92,28},
{"estoril.trk"  ,79.749 ,3.85,.37,5.85,.012,.016, 7,32,1.11,.90,16},
{"figure8.trk"  ,84.830 ,1.06,.28,5.95,.010,.028, 7,28,1.28,.89, 4},
{"fiorano.trk"  ,69.274 ,3.00,.42,6.45,.013,.020, 6,38,1.25,.91,16},
{"hock.trk"     ,93.365 ,6.00,.57,6.60,.012,.016,10,34,1.09,.88,24},
{"hungary.trk"  ,72.246 ,3.66,.36,5.90,.011,.014, 8,24,1.29,.93,20},
{"imola.trk"    ,74.442 ,4.84,.51,6.75,.012,.028, 7,26,1.06,.93,18},
{"indy500.trk"  ,126.07 ,2.68,.59,5.80,.010,.013, 6,22,1.17,.92,20},
{"indygp.trk"   ,78.016 ,3.61,.52,5.80,.013,.010, 8,28,1.07,.94,20},
{"jerez.trk"    ,75.033 ,4.23,.32,6.20,.017,.020, 5,32,1.13,.91,30},
{"longrand.trk" ,70.966 ,105.,.34,5.85,.013,.020,14,36,1.33,.88,20},
{"loudon.trk"   ,89.147 ,1.34,.51,6.00,.011,.022, 5,22,1.25,.91,28},
{"magnycrs.trk" ,74.662 ,4.17,.45,6.05,.014,.016,12,32,1.13,.94,18},
{"michigan.trk" ,129.08 ,2.08,.40,5.80,.010,.026, 6,32,1.09,.85,30},
{"midohio.trk"  ,78.000 ,3.79,.58,6.65,.012,.022, 8,38,1.28,.91,10},
{"mlwaukee.trk" ,92.220 ,1.14,.31,5.80,.009,.010, 7,22,1.20,.86,22},
{"monaco.trk"   ,56.484 ,3.49,.41,6.35,.013,.018, 7,36,1.29,.91, 4},
{"montreal.trk" ,78.719 ,4.41,.57,6.55,.009,.022,14,38,1.28,.90,20},
{"monza-76.trk" ,93.921 ,5.00,.48,6.20,.012,.010, 7,32,1.20,.93,38},
{"mosport.trk"  ,87.75  ,3.49,.55,5.78,.011,.012, 5,58,1.28,.94,34},
{"nazareth.trk" ,95.736 ,1.09,.34,5.80,.009,.010, 6,20,1.27,.91,16},
{"nurnburg.trk" ,81.135 ,4.54,.53,6.70,.013,.012, 5,20,1.30,.85, 0},
{"oval2.trk"    ,83.032 ,0.77,.41,5.80,.008,.026, 7,22,1.14,.94,12},
{"phoenix.trk"  ,93.518 ,1.18,.38,5.80,.011,.014, 6,20,1.08,.85,20},
{"pocono.trk"   ,109.149,3.09,.57,5.80,.011,.016, 5,20,1.22,.90,10},
{"pocono.trk"   ,107.03 ,3.13,.56,5.84,.012,.014, 5,22,1.30,.86,34},
{"ra.trk"       ,88.46  ,5.56,.36,5.87,.011,.013,12,26,1.19,.91,20},
{"rars.trk"     ,77.30  ,6.00,.49,6.75,.012,.010, 6,38,1.26,.89, 2},
{"sepang.trk"   ,72.48  ,5.17,.41,6.10,.013,.018, 9,36,1.28,.85, 4},
{"silverst.trk" ,80.45  ,4.55,.44,6.12,.012,.027,10,29,1.18,.91,20},
{"silver97.trk" ,78.815 ,5.00,.42,6.50,.013,.026, 5,32,1.28,.94,24},
{"spa.trk"      ,85.150 ,6.00,.44,6.00,.013,.014,11,30,1.12,.93,16},
{"speed2.trk"   ,99.59  ,3.06,.54,6.70,.012,.014,13,30,1.21,.86,36},
{"suzuka.trk"   ,84.892 ,5.17,.47,6.30,.012,.012, 8,38,1.30,.88,28},
{"toronto.trk"  ,67.259 ,2.95,.45,6.40,.014,.020, 6,20,1.23,.93,10},
{"tremblnt.trk" ,79.32  ,4.05,.46,6.04,.012,.014, 7,54,1.08,.97,32},
{"v01.trk"      ,64.24  ,1.13,.58,6.25,.012,.010, 7,28,1.20,.88, 4},
{"v02.trk"      ,57.91  ,1.13,.59,6.30,.012,.010, 5,24,1.11,.88, 0},
{"v03.trk"      ,68.96  ,1.65,.58,6.75,.011,.013, 6,24,1.22,.88,20},
{"zandvort.trk" ,84.06  ,3.75,.47,6.48,.013,.013, 8,46,1.28,.88,50},
{"watglen.trk"  ,87.04  ,4.69,.45,5.81,.012,.012, 5,29,1.30,.90,40},
{" ", -1}
};

//==============================================================================
#ifdef TWEAK

#include <stdio.h>
static double avg_speed, speed[3];
static double best_speed = 0;

extern long lap_count;
extern double length;// track length
FILE *fp;
static long races_done = 0;
static int check = 0;
static int i,j,k,l,m,n,o,p,r,q,t,u;
static int prev_track = -1; // for multi-track optimization
#endif
//==============================================================================

///// calculate braking distance from speed vnow to speed vthen

static double distancetobraketo(double vnow, double vthen, double radius)
{
  if (vnow<vthen)
    return 0.0;  // no need to brake as we are already at the desired speed!
  else if (radius != 0)
    return (BC2 * (vnow+vthen) * (vnow-vthen));
  return (BC1 * (vnow+vthen) * (vnow-vthen));
  
}

///// calculate optimal speed for both straigths and curves

static double curvespeed(double radius, double mar, double speed)
{
  if (radius == 0.0)
    return (speed * ACC_STR); // speed on straight evaluated wrt to current speed
  return CS * sqrt(fabs(radius) + mar);
}

///// calculate curve length in feets

static double curvelength(double len, double rad)
{
  if (rad == 0.0)
    return (len);
  else
    return (len * fabs(rad));
}

///// A look-ahead code: //////////////////////////////////

static void avoid_cars(situation &s, con_vec &result)
{
  int danger = 0;
  int kount = 0;
  double vc1;
  double v[4], d[4], x[4], y[4], vx[4], vy[4];
  for(int i=0;i<4;i++)
    if(s.nearby[i].who<MAX_CARS)
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

//////////////////////// Danger! What to do? ///////////////////////////////

  if (danger){
    if(kount > 2)
      vc1 = min((s.v + vy[0] - 5),(s.v + vy[1] - 5));
    else{
      vc1 = min((s.v + vy[0] + 5),((s.damage >20000?0.9:0.95)*result.vc));
      if (!s.cur_rad){  //straigth => pass!
	if (x[0] < 0)  // car in left
	  if (s.to_lft + x[0] < track.width - 1.5* CARWID)
	    result.alpha -= 0.03;
	//if we have room in right, turn right
	  else    result.alpha += 0.03;
	else if (x[0] >= 0)    // car in right
	  if (s.to_lft + x[0] > 1.5* CARWID) // if we have room in left
	    result.alpha += 0.03; //turn left
	  else  result.alpha -= 0.03;
      }
    } // end of kount < 2
  }// end of danger
  else vc1 = result.vc; //this executes if no danger seen

  vc1 = max(vc1,5);  // To avoid stopping behind very slow car
  result.vc = vc1;
} // end of avoid_cars function

static void keep_control(situation &s, con_vec &result)
{
  double A = sqrt(max(0,1.08-sqr(s.cen_a/g))); // max allowed tan_a
  double B = g*A;
  double C = (s.v + B); // max allowed vc

  if (result.vc>s.v)
    result.vc = min(C,result.vc);
}

static void get_optimized_track_data()
{
    for(int i=0; opt_data[i].speed >= 0; i++)
      if( !strcmp(opt_data[i].name,track.sName)){
	trk = opt_data[i];  // get the pre-existing parameters
	break;       // finish
      }

    APEX = trk.apex;
    CS = trk.cs;
    BC1 = trk.bc1;
    BC2 = trk.bc2;
    M1 = trk.m1;
    M2 = trk.m2;
    ACC_CRV = trk.acccrv;
    BRK_CRV = trk.brkcrv;
    END_STR = trk.endstr;
}


//////////////////////////// MAIN FUNCTION ///////////////////////////////////
con_vec Apex8(situation &s)
{
  con_vec result = { 0.0f, 0.0f, 0.0f, 0, 0 };  // This is what is returned.
  static int firstcall = 1;

  if (firstcall){        //  this is the very first call, called only once
    my_name_is("Apex8"); //  this lets everyone know who we are.
    firstcall = 0;
    return result;       //  must return an answer
  }

  track = get_track_description();
#ifndef TWEAK
////////////////// Track-specific variables ////////////////////////
  if(s.starting)
    get_optimized_track_data();
#endif
///////////////////////// Optimization of car /////////////////////////////////

#ifdef TWEAK
  if (s.starting){

    ++races_done;

    if(prev_track != Track::currentTrackFileName){ // New track, reset data
      best_speed = 0;
      check = 0;
      speed[0] = speed[1] = speed[2] = 0;
      races_done = 0;
      prev_track = Track::currentTrackFileName;
      get_optimized_track_data();
    }

    else if (!check){ // in start of every race change constants
      t = os.PickRandom() % 60;
      j = (os.PickRandom() % 91)% 20;
      k = (os.PickRandom() % 119)% 10;
      l = (os.PickRandom() % 59)% 10;
      m = (os.PickRandom() % 137)% 10;
      n = (os.PickRandom() % 149)% 20;
      o = (os.PickRandom() % 79)% 10;
      p = (os.PickRandom() % 41)% 10;
      r = (os.PickRandom() % 51)% 10;
      q = (os.PickRandom() % 71)% 35;
      u = (os.PickRandom() % 101)% 10;
    
      APEX = 0.0 + t * 0.01;
      CS  =  5.8 + j * 0.05;
      BC1 = 0.008 + k * 0.001;
      BC2 = 0.010 + r * 0.002;
      M1 = 5 + l * 1;
      M2 = 20 + m * 2;
      ACC_CRV = 1 + q * 0.01;
      BRK_CRV = 0.85 + u * 0.01;
      END_STR = 0 + n * 2;
    }
  } // end s.starting

  if (s.lap_flag && !s.laps_to_go){ // at end of race calculate speed:
    speed[check] = MPH_FPS*lap_count*track.length/(s.time_count-s.start_time);
    if (speed[check] > best_speed - .2 && !s.damage){ // if good setup,
      ++check;                                // check it 3 times
      if (check == 3){  // after 3rd successive round calculate avg_spd
	avg_speed = (speed[0] + speed[1] + speed[2])/3;
	if(avg_speed > best_speed){ // if this was the best:
	  best_speed = avg_speed;     // save and print it!
	  printf("\a");
	  fp = fopen("apex8.dat","at");
	  fprintf(fp, "Race #%d {\"%s\"\t,%.3f\t,",races_done,currentTrack->fileName,best_speed);
	  fprintf(fp, "%.2f,", track.length/(5280.0 * s.fuel_mileage));
	  fprintf(fp, "%.2f,%.2f,%.3f,%.3f,%2.0f,%2.0f,",APEX, CS, BC1, BC2, M1, M2);
	  fprintf(fp, "%4.2f,%2.2f,%2.0f},\n", ACC_CRV, BRK_CRV, END_STR);
	  fclose(fp);
	}
	check = 0;   // after 3rd round reset count
      }
    }
    else check=0;   // cancel checking if race too slow or damage happens
  }
#endif
//////////////////////// End of optimization code /////////////////////////////


  double ideal_alpha, target, dot, to_end, mylength, next_length; 
  double width, lane=0.0, alpha, vc;

  to_end = curvelength(s.to_end, s.cur_rad); // current segment
  mylength = curvelength(s.cur_len, s.cur_rad); // current segment
  next_length = curvelength(s.nex_len, s.nex_rad);
  width = s.to_lft + s.to_rgt; // width of the track
  dot = asin(s.vn/s.v); // direction of travel
  
//  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
//     return result;    

///// This statement calculates desired angle to reach target point at the end
///// of straight. The target point is floating: it moves from right to left
///// if to_end decreases. At the end of straight (END_STR feets) we do not
///// bother of current target and simply keep the angle until to the next
///// segment.

  if (s.cur_rad == 0){
     if (to_end < END_STR)
	to_end = END_STR;
     if (s.nex_rad > 0)
	lane = (s.to_lft - M1 - to_end*(width-M2-M1)/mylength);
     else 
	lane = (-s.to_rgt + M1 + to_end*(width-M2-M1)/mylength);
  }
  
  ///// The same is calculated for curves. 
  
  if (s.cur_rad > 0){
     lane = s.to_lft - M1;
     if (to_end < APEX*mylength)
	lane = (s.to_lft - ((APEX*mylength-to_end)*(width-M2-M1)/
			    (APEX*mylength)));
     if (to_end < END_CRV)
	lane = 0; 
  }
  
  if (s.cur_rad < 0){
     lane = M1 - s.to_rgt;
     if (to_end < APEX*mylength)
	lane = (-s.to_rgt + ((APEX*mylength-to_end)*(width-M2-M1)/
			     (APEX*mylength)));
     if (to_end < END_CRV)
	lane = 0;
  }
  
  
  //////////////////////////// NOW calculate speed //////////////////
  
  vc = curvespeed(s.cur_rad, M1, s.v);
  
  ///// After apex increase speed gently
  
  if ((s.cur_rad != 0.0) && (to_end < (APEX * mylength)))
     vc = ACC_CRV * s.v;
  
  ///// For next curve brake hard:
  
  if (to_end < distancetobraketo(s.v,curvespeed(s.nex_rad,M1,s.v),s.cur_rad)
      || to_end+next_length < 
      distancetobraketo(s.v, curvespeed(s.after_rad, M1, s.v),s.cur_rad)){
     if (s.cur_rad != 0.0)   vc = BRK_CRV * s.v;
     else vc = BRK_STR * s.v;
  }
  
  ///// asin must have argument between -1 and 1.
  
  target = lane/to_end;
  
  if (target > 1)
     target = 1;
  if (target < -1)
     target = -1;
  
  /////////////////////////////////////////////////////////////////////////
  /////////////////////// Set alpha ///////////////////////////////////////
  
  if (s.cur_rad != 0.0)           // in curve
     ideal_alpha =  asin(target);
  ideal_alpha = atan(lane/to_end);  // in straight
  
  alpha = ideal_alpha - dot;      // correct alpha considering our current
				// angle wrt track
  
  //////////////////////// That's it! ////////////////////////////////////
  result.vc = vc;
  result.alpha = alpha;
  avoid_cars(s, result);
  if (s.damage > 25000) 
     keep_control(s,result);
#ifdef TWEAK
  if (s.damage > 10) result.alpha += 1.0;
#endif
  
  if (s.starting){  
    if (s.stage == QUALIFYING)   
      result.fuel_amount = (args.m_iNumQualLap + 1) * trk.lb_per_lap;
    if (s.stage == PRACTICE)   
      result.fuel_amount = (args.m_iNumPracticeLap + 1) * trk.lb_per_lap;
    else    
      result.fuel_amount = 1.2 * (args.m_iNumLap+1) * trk.lb_per_lap;
  }
  result.request_pit = 0;
  
  if (((s.damage > 25000 && s.stage != QUALIFYING) ||   // if high damage
       s.fuel < trk.lb_per_lap + 1) && s.laps_to_go > 1){ // or low fuel
    result.request_pit = 1;                          // ask for a pit-stop
    result.repair_amount = MAX_DAMAGE;
    result.fuel_amount = MAX_FUEL;
    if(s.laps_to_go < MAX_FUEL/trk.lb_per_lap){ // if this is last fuel-stop
      result.fuel_amount = 1.2 * s.laps_to_go * trk.lb_per_lap;// take less fuel
      result.repair_amount =  // repair less
	max(0, int(s.damage - MAX_DAMAGE * (1-s.laps_to_go*trk.lb_per_lap/MAX_FUEL)));
      result.repair_amount =  // but still repair while refueling 
	max(result.repair_amount, int(result.fuel_amount * 20.0)); 
    }
  }
  return result;
}
