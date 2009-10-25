// OVAL VERSION

// This car is now fairly good for anything that an oval can throw at it (I think)
// however much work now needs to be done on overtaking as this leaves a lot to
// be desired and is essential.


#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"

const double CORN_MYU 		= 1.00;
const double BRAKE_ACCEL 	= -33.0;
const double BRAKE_SLIP 	= 6.5;
const double BRK_CRV_ACC	= -27.0;
const double BRK_CRV_SLIP	= 3.5;
const double BRK_AV_COL		= 0.5;
const double MARGIN			= 10.0;
const double MARG2			= MARGIN + 10;
const double ENT_SLOPE		= 0.33;
const double STEER_GAIN		= 1.0;
const double DAMP_GAIN		= 1.2;
const double BIG_SLIP		= 9.0;
const double CURVE_END		= 4.0;
const double TOO_FAST		= 1.02;
const double DELTA_LANE		= 0.25;

static double corn_spd(double radius)
{
	double rad;
   if(radius < 0) rad = -radius;
   else if(radius == 0) return(1000);
   else rad = radius;

   return(sqrt(rad * 32.2 * CORN_MYU));
}

static double CritDist(double v0, double v1, double a)
{
	double CD;

   CD = a;	// Just to remove the damn warning about not using 'a'

   if(v1 - v0 > 0.0) return(0.0);
	   CD = ((v0*v0) - (v1*v1)) / (64.4 * 1.05 * (1 - exp(-0.475 * v0)));

	return(CD);
}

con_vec Mat1(situation &s)
{
	const char name[] = "Mat1";
	static int init_flag = 1;
	con_vec result;
	static double alpha, vc;
   static double bias;
   static double speed;
   static double speed_next;
   static double width;
   static double to_end;
   static double lane;
   static int rad_was = 0;
   static double lane_inc = 0.0;
   static double lane_total = 0.0;

   static int TurnExitCorrection = 0;     // This is the value for exiting turns
   													// with a length >= PI / 2
   static int TEC = 0;     					// This is the TurnExitCorrection for
   													// other turns

	if(init_flag)
   {
		my_name_is(name);
		init_flag = 0;
		result.alpha = 0;
		result.vc = 0;
      return result;
	}

	// Set TurnExitCorrection value for named tracks, else use 50.
   if(TurnExitCorrection == 0)
   {
      track_desc track = get_track_description();
   	if(strcmp("phoenix.trk", track.sName) == 0) TurnExitCorrection = 75;
	 	else if(strcmp("nazareth.trk", track.sName) == 0) TurnExitCorrection = 50;
   	else if(strcmp("oval2.trk", track.sName) == 0) TurnExitCorrection = 120;
	   else if(strcmp("mlwaukee.trk", track.sName) == 0) TurnExitCorrection = 90;
	   else if(strcmp("pocono.trk", track.sName) == 0) TurnExitCorrection = 30;
      else if(strcmp("indy500.trk", track.sName) == 0) TurnExitCorrection = 85;
		else if(strcmp("loudon.trk", track.sName) == 0) TurnExitCorrection = 50;
      else if(strcmp("michigan.trk", track.sName) == 0) TurnExitCorrection = 40;
      else TurnExitCorrection = 50;
   }


   if(stuck(s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc))
		return(result);

   width = s.to_lft + s.to_rgt;

   if(s.starting) lane = s.to_lft;

	// A little routine to display data about the turns on a track
/*	static double cr = 0;
   if(s.cur_rad != cr)
   {
      cr = s.cur_rad;
		cout << s.cur_rad << " - " << s.cur_len << " : " << flush;
   }
*/
   // Calculate the correct value for alpha.
   if(s.cur_rad > 0.0)													// left turn
   {
      // If we are more than two thirds through the turn gradually drift out
      // from the inside wall.  If not stay close to it.
      if(s.cur_len <= PI / 2) TEC = TurnExitCorrection;	// Decide how fast to drift out
      else TEC = int( 38 * s.cur_len );

      if(s.nex_rad > 0.0 || (s.nex_rad == 0.0 && s.nex_len < 500)) TEC = (int) 2.5;	// Adjust for another turn
//      (s.nex_rad > 0.0) TEC = 2.5;

   	if(s.to_end < s.cur_len / 3)
		{
         lane += width / ((s.cur_rad * s.cur_len / TEC * PI) * (s.cur_len - s.to_end));
      }
      else if(lane > MARGIN)
      {
      	lane -= 4 * width / s.cur_len;
      }
      else lane = MARGIN;
      rad_was = 1;
   }
   else if(s.cur_rad < 0.0)											// right turn
   {
      // If we are more than two thirds through the turn gradually drift out
      // from the inside wall.  If not stay close to it.
   	if(s.to_end < s.cur_len / 3)
      {
         lane -= width / ((s.cur_rad * s.cur_len / TurnExitCorrection * PI) * (s.cur_len - s.to_end));
      }
      else if (lane < width - MARGIN)
      {
			lane += 4 * width / s.cur_len;
      }
      else lane = width - MARGIN;
      rad_was = -1;
   }
   else if(s.nex_rad > 0.0 && s.to_end < (90 * PI))			// just before left turn
   {
		if(lane > MARGIN) lane -= 10 * width / 600;
      rad_was = 1;
   }
   else if(s.nex_rad < 0.0 && s.to_end < (90 * s.nex_len))	// just before right turn
   {
		if(lane > width - MARGIN) lane += 10 * width / s.cur_len;
      rad_was = -1;
	}
   else																		// straightaway
   {
		if(s.nex_rad < 0.0)
      {
      	if(lane > MARGIN) lane -= 8 * width / s.cur_len;
      }
      else if(s.nex_rad > 0.0)
      {
      	if(lane < width - MARGIN) lane += 8 * width / s.cur_len;
      }
      else lane = width / 2;  // This should never happen.
   }

   if(s.cur_rad == 0.0)													// Set BIAS for steering
   {
   	bias = 0.0;
      if(s.nex_rad > 0.0) speed = corn_spd(s.nex_rad + (width / 2)) +  5;
      else if(s.nex_rad < 0.0) speed = corn_spd(-s.nex_rad + (width / 2)) + 5;
      else speed = 250.0;
   }
   else
   {
   	if(s.nex_rad == 0.0) speed_next = 2500.0;
      else speed_next = corn_spd(fabs(s.nex_rad) + (width / 2));

   	if(s.to_end < s.cur_len / 2)
			speed = corn_spd(fabs(s.cur_rad) + (width / 2) + fabs(lane_inc));
      else if(s.to_end > 0.1)
			speed = corn_spd(fabs(s.cur_rad) + ((2.0 / s.to_end) * width) + fabs(lane_inc));

      bias = (s.v * s.v / (speed * speed)) * atan(BIG_SLIP / speed);
      if(s.cur_rad < 0.0) bias = -bias;
   }


   // Calculate the correct value for vc.
	if(s.cur_rad == 0.0)	// Speed calculations when on straights
   {
   	if(s.to_end > CritDist(s.v, speed, BRAKE_ACCEL)) vc = s.v + 50.0;
      else
      {
      	if(s.v > TOO_FAST * speed) vc = s.v - BRAKE_SLIP;
         else if (s.v < speed / TOO_FAST) vc = 1.1 * speed;
         else vc = 0.5 * (s.v + speed);
      }
   }
   else						// Speed calculations when on turns
   {
   	if(s.cur_rad > 0.0) to_end = s.to_end * (s.cur_rad + MARGIN);
      else to_end = -s.to_end * (s.cur_rad - MARGIN);

      if(to_end <= CritDist(s.v, speed_next, BRK_CRV_ACC))
      	vc = s.v - BRK_CRV_SLIP;
      else if(to_end /width < CURVE_END && speed_next > speed)
      	vc = 0.5 * (s.v + speed_next) / cos(alpha);
      else vc = 0.5 * (s.v + speed) / cos(alpha);
   }

	// Collision avoidance and over taking routines.

   // Let's just consider the nearest car

   if(s.nearby[0].who < 16)
   {
      // closing in on car ahead
   	if(s.nearby[0].rel_y < 4.0 * CARLEN && s.nearby[0].rel_ydot < 0.0)
		{
      	// examine my situation

         // Too close to the car ahead - i.e. about to hit them
         if(s.nearby[0].rel_y < 0.5 * CARLEN && s.nearby[0].rel_x < 2 * CARWID &&
         	 s.nearby[0].rel_x > -2 * CARWID)
         {
         	vc = 0.50 * vc;
         }
			// On a straightaway, approaching left turn
        	else if(s.cur_rad == 0.0 && s.nex_rad > 0.0 && s.to_end < (90 * PI) &&
         	lane_inc != 0 && s.nearby[0].rel_x < 2 * CARWID && s.nearby[0].rel_x > -2 * CARWID)
			{
				if(lane + lane_inc - DELTA_LANE > MARGIN)
            	lane_inc -= DELTA_LANE;    					// If there's room adjust position
            if(s.nearby[0].rel_y < 2.0) vc = 0.95 * vc;	// Break a little more if required
         }
			// On a straightaway - next turn will be to the left
         else if(s.cur_rad == 0.0 && s.nex_rad > 0.0)
         {
         	if(s.nex_rad > 0.0 && s.nearby[0].rel_x < 2 * CARWID && s.nearby[0].rel_x > -2 * CARWID)
            	lane_inc -= DELTA_LANE;		// Adjust position
         }
         else if(s.cur_rad > 0.0) 								// In a left turn
         {
            if(lane_inc < 0.0)
            	lane_inc += DELTA_LANE;
         	if(s.nearby[0].rel_y < 2.0) vc = 0.75 * vc; 	// Just slow down, better to finish second
            					 										// than not finish
         }
         else if(s.cur_rad > 0.0) 								// In a right turn
         {
            if(lane_inc > 0.0)
            	lane_inc -= DELTA_LANE;
         	if(s.nearby[0].rel_y < 2.0) vc = 0.75 * vc; 	// Just slow down, better to finish second
            					 										// than not finish
         }
      }
   }
   // If the way ahead is clear return to the correct racing line
   else
   {
   	if(lane_inc > 0.0) lane_inc -= DELTA_LANE;
      else if(lane_inc < 0.0) lane_inc += DELTA_LANE;
   }

   // required position = racing line + avoidance/overtaking position
	lane_total = lane + lane_inc;

   // Make sure we don't run over the grass
   if(lane_inc != 0.0 && s.cur_rad == 0.0)
   {
   	if(lane_total < 2.0 * MARGIN) lane_total = 2.0 * MARGIN;
   	else if(lane_total > width -  2.0 * MARGIN) lane_total = width - 2.0 * MARGIN;
   }
   else
   {
   	if(lane_total < MARGIN) lane_total = MARGIN;
   	else if(lane_total > width - MARGIN) lane_total = width - MARGIN;
   }

   alpha = STEER_GAIN * (s.to_lft - (lane_total)) / width - DAMP_GAIN * s.vn / s.v + bias;


   // Pit if low on fuel or if damage is high, unless only a two (or is it three)
   // laps remain.
	result.request_pit = 0;
 	if (s.stage != QUALIFYING && ((s.damage > 25000 || s.fuel < 8) && s.laps_to_go > 3))
   {
   	result.request_pit = 1;
     	result.repair_amount=s.damage;
     	result.fuel_amount = MAX_FUEL;
   }
//	cout << "GOT HERE" << endl;

   result.vc = vc;
   result.alpha = alpha - STEER_GAIN * lane_inc / width;
//	result.alpha = alpha + STEER_GAIN * 1 / 4;
   return(result);
}
