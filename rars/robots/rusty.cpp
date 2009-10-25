// RUSTY.CPP - "driver" function for RARS
// By Bill Benedict, May 1995
// benediw@rosevc.rose-hulman.edu

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"


#define max(x,y) (((x)<(y))?(y):(x))
#define min(x,y) (((x)<(y))?(x):(y))

#define BRAKE 0.27       // mph/ft    .22
#define ACCEL 10       // mph increase each time through this proc
#define NO_RAD 810      // any bigger and its a straightaway, not a turn
#define MAX_SPEED 400.0
#define MIN_SPEED 40.0
#define MID 0.5
#define SRGT 0.60
#define SLFT 0.40
#define TRGT 0.99
#define TLFT 0.01



extern const double CARWID;
extern char* glob_name;

con_vec Rusty(situation& s)
{
   const char name[] = "Rusty";
   static int init_flag = 1;
   con_vec result;
   double alpha, vc;
   double width;
   double to_end;
   double cur_len, nex_len;
   double cur_spd, nex_spd, after_spd;
   double cur_rad, nex_rad, after_rad;
   double mod=1.0;
   double EARLY;
   double dlane;

   const double steer_gain = .2;
   const double steer_damp = .53;
   // const double normalane = 0.5;
   // static int lane_time = 0;
   // static int lane_time2 = 0;
   // static double lane_change = -0.1;
   static double lane = 0.0;
   double cur_lane = 0.0;
		// determines right-left position on straigtaway
   const double corn_con = 5.45;

   if(init_flag)  {
	  my_name_is(name);
	  init_flag = 0;
	  result.alpha = result.vc = 0;
	  return result;
   }
   width = s.to_lft + s.to_rgt;
   cur_rad=s.cur_rad;
   nex_rad=s.nex_rad;
   after_rad=s.after_rad;
   cur_lane=s.to_lft/width;
   dlane=0.05;



   if(nex_rad==0.0) {
     nex_len = s.nex_len;
   }
   else {                //  make sure that len is always in feet :)
     nex_len= s.nex_len * (s.nex_rad + width/2);
   }

   if(cur_rad==0.0) {
     cur_len = s.cur_len;
     to_end = s.to_end;
   }
   else {                //  make sure that to_end is always in feet :)
     cur_len= s.cur_len * (s.cur_rad + width/2);
     to_end = s.to_end * (s.cur_rad + width/2);
   }
   to_end=max(to_end,-to_end);


   if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
	  return result;


   if(s.nex_len < 1.63 && fabs(s.nex_rad) < width*4)
     mod =1 + (1.63 - s.nex_len)*2.8;
   else mod = 1;

   if((s.nex_rad > 0 && s.after_rad > 0) ||
      (s.nex_rad < 0 && s.after_rad < 0))
	mod = 1;


   if(after_rad != 0.0)
     after_spd = corn_con * sqrt(after_rad > 0.0 ? mod*(after_rad+width/2) :
					       mod*(-after_rad+width/2));
   else
     after_spd = MAX_SPEED;

   if(nex_rad != 0.0 && s.nex_len > .53)  //.4
     nex_spd = corn_con * sqrt(nex_rad > 0.0 ? mod*(nex_rad+width/2) :
					       mod*(-nex_rad+width/2));
   else
     nex_spd = MAX_SPEED;

   if(s.cur_len < 1.63 && fabs(s.cur_rad) < width*4) {
     mod =1 + (1.63 - s.cur_len)*3.9;
     if(s.cur_len < .81) mod*=2.7;
   }
   else mod = 1;

   if((s.nex_rad > 0 && s.after_rad > 0) ||
      (s.nex_rad < 0 && s.after_rad < 0))
	mod = 1;

  if(cur_rad != 0.0 && s.cur_len > .53)
     cur_spd = corn_con * sqrt(cur_rad > 0.0 ? mod*(cur_rad+width/2) :
					       mod*(-cur_rad+width/2));
   else
     cur_spd = MAX_SPEED;
   if (nex_spd < MIN_SPEED) nex_spd = MIN_SPEED;
   if (cur_spd < MIN_SPEED) cur_spd = MIN_SPEED;
   EARLY = 0.0;
   if((cur_rad == 0.0 || cur_rad > NO_RAD || cur_rad < -NO_RAD) && nex_rad != 0.0)
   { // straight, nearing a turn
     if(s.nex_len > 2.0) EARLY = .8*width; // 115 degrees
     else if(s.nex_len > 1.6) EARLY = 1.2*width; // 91 degrees
     else if(s.nex_len > 1.3) EARLY = 1.8*width; // 75 degrees
     else if(s.nex_len > 1) EARLY = 2.0*width; // 60 degrees
     else if(s.nex_len > .7) EARLY = 2.1*width; // 40 degrees
     else EARLY = 2.0*width;
     if (fabs(s.nex_rad) < 151) EARLY*=1.2;
     else if (fabs(s.nex_rad) < 181) EARLY*=1.1;
     else if (fabs(s.nex_rad) < 311) EARLY*=1.0;
     else if (fabs(s.nex_rad) < 451) EARLY*=0.9;     // 221
     else if (fabs(s.nex_rad) < 571) EARLY*=0.8;     // 241
     else EARLY*=0.65;                               //.7

   }
   else if(cur_rad !=0.0 &&(cur_rad < NO_RAD && cur_rad > -NO_RAD))
   {
     if(s.to_end/s.cur_len < .13) EARLY = to_end +1;
     else EARLY = 0.0;
   }


   if(cur_rad != 0.0 && cur_rad > -NO_RAD && cur_rad < NO_RAD)
     dlane=1.0;
   else {
     if(cur_len>800) dlane=200/cur_len;
     else dlane=300/cur_lane;
   }
   if(to_end < EARLY)
   {
     dlane = 1.0;
     cur_rad = nex_rad;
     nex_rad = after_rad;
//   sound(20);delay(2);nosound();
   }

   if (cur_rad == 0.0 || cur_rad > NO_RAD || cur_rad < -NO_RAD)
   {      // on a 'straight'
     if(nex_rad > 0.0) lane = SRGT;
     else if (nex_rad < 0.0) lane = SLFT;
     else if (after_rad > 0.0) lane = SRGT;
     else if (after_rad < 0.0) lane = SLFT;
     else lane = MID;
   }
   else if (cur_rad > 0.0) lane = TLFT;
   else if (cur_rad < 0.0) lane = TRGT;
   if(s.cur_rad != 0.0 && s.cur_rad > -NO_RAD && s.cur_rad < NO_RAD && EARLY)
   {
     if(s.cur_rad > 0 && s.nex_rad <= 0) lane=MID;
     else if (s.cur_rad < 0 && s.nex_rad >= 0) lane=MID;
     dlane = 0.08;
   }
   if(dlane > 2.0) dlane=2.0;
   if(dlane < 0.1) dlane=0.1;
   // maybe choose a different lane: (to help in passing)
/*
   if(!s.dead_ahead) {
	  if (lane_time <= 0) {
	lane_time=-1;
	lane = normalane; //+ random(5)*15;
	  }
   }
   else if (lane_time2 <= 0){
	  // pick a different lane:
	  if(lane >= 0.9)               // pick a new lane somehow:
	    lane_change = -0.1;
	  else if(lane <= 0.1)
	    lane_change = 0.1;
	  lane += lane_change;
	  if (lane > 0.9) lane = 0.9;
	  else if (lane < 0.1) lane = 0.1;
	  lane_time2=20;
	  lane_time=100;
   }
   if (lane_time >=0 ){
	 lane_time--;
	 lane_time2--;
   }
*/
// Audible buzz when the dead_ahead flag is set for this driver
//if(s.dead_ahead) {sound(20);delay(2);nosound();}

   if(lane < cur_lane) lane=max(cur_lane - dlane,lane);
   else if(lane > cur_lane) lane=min(cur_lane + dlane,lane);

   alpha = 2.5 * steer_gain * ((s.to_lft/width) - lane);

   if(s.dead_ahead)
     alpha *= 1.2;

   alpha -= steer_damp * s.vn / s.v;
	  // This is damping, to prevent oscillation

   if ((s.v - nex_spd) > (to_end * BRAKE))
      vc = max(s.v - (BRAKE*60), nex_spd);
   else if(s.v - after_spd > (to_end + nex_len)*BRAKE)
      vc = max(s.v - (BRAKE*60), after_spd);
   else
      vc = min(cur_spd,s.v+ACCEL);  // keep accellerating to speed we want

if (s.starting) result.fuel_amount = MAX_FUEL;
result.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  result.request_pit = 1;
  result.repair_amount=s.damage;
  result.fuel_amount = MAX_FUEL;
  }

   result.vc = vc;
   result.alpha = alpha;
   return result;
}


