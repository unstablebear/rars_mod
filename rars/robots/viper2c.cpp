//////////////////////////////////////////////////
//   VIPER V2.0c is a robot for RARS 0.72       //
//   By Tom Pycke <Tom.Pycke@advalvas.be>       //
//   Source is  public. If you wish to use any  //
//   of my code, please let me know.            //
//   I wrote it using Linux. Please  check out  //
//   this great OS. And it's free!              //
//   This  car is  lane-driven, but i think it  //
//   is my last one. It's too frustrating when  //
//   i see  vector gliding  over these  random  //
//   tracks.                                    //
//////////////////////////////////////////////////


////                INCLUDES                  ////

#include <string.h>
#include <math.h>
#include "car.h"
#include "track.h"


////                 MACROS                   ////

#define min(x,y) (x<y?x:y)
#define max(x,y) (x>y?x:y)
#define sqr(x) x*x


////                VARIABLES                 ////

static double CURVESPEEDMULTIPLIER  =  5.9;  //6.15
static double width;
static double BORDER = 0;
static double STRAIGHT_BORDER = 25;//40;
static int cur_chicane, nex_chicane;
static double direction, prd_rad;
static double BRK_MP;
static double start_dive = 0.016; //0.016
static track_desc t;

////                FUNCTIONS                ////


//  calculate maximum radius with a given radius,
//  angle and width of the track (ie start and
//  end lane: +- middle of the track)
static double max_radius (double rad, double len)
{
    double radius = fabs(rad);
    double alpha = PI/2 - len/2;
    if (rad == 0)
        return 2000;
    if (alpha == 0 || alpha == PI/2)
        alpha += .01;
    return fabs( ( (radius - (radius + width/2) / fabs(sin(alpha)) ) / 
               (1 - fabs(sin(alpha))) * fabs(sin(alpha)))); 
	       // My own function :-)
}

//  calculate current lane to destination lane
static double lane_glide(double tot_dist, double rem_dist, double start_lane, double end_lane)
{
    double fraction_done, lane_delta;
    fraction_done = (tot_dist - rem_dist) / tot_dist;
    lane_delta = end_lane - start_lane;
    return start_lane + (fraction_done * lane_delta);
}
	
//  calculate braking distance
static double distancetobraketo (situation s, double vthen)
{
    double dist;
    if (s.v<vthen)
       return 0.0;
    dist = .0549 * (s.v+vthen)*(s.v-vthen)/(1.7*2.0)*BRK_MP;
    return dist;
}

// calculate speed for a given radius and angle
static double curvespeed(double radius, double len)
{
    double speed;
    radius = fabs(radius);
    if (radius == 0.0)
        return(400.0);
   

   speed = CURVESPEEDMULTIPLIER * sqrt(fabs(radius)) * 
           ((len<1.28 && (len>.97 || radius>200))? 
	   (fabs(radius)<360?1.1:1.00):1.0);
  if (prd_rad<=0)
  {
   if (fabs(radius)<110) 
        speed *= min(max(pow(len/1.8,-1),.95),1.50);
   if (radius < 160 && len >.8 && len < 1.3)
       speed *= .9;
   if (len > PI/4.5 && len < PI / 3.5 && radius < 150)
       speed *= 1.05; //1.05
   if (radius < 65)          // some (stupid) adjustments
       speed *= (len < .55 ? 1.5:1.1);
   if (radius < 30 && len>PI/4)
       speed *= 1.3;
   if (len > PI*.9 && radius > 300)
       speed *= .9;
   if (radius<80 && len<PI/2)
       speed = max(speed,60); //50
   if (radius<40)
       speed = max(speed,40); //40
  }
       
   return max (speed,20);
}


static double curvelength(double len, double rad)
{
    if (rad == 0.0)
	return (len);
    else    
	return (len * fabs( rad )); 
}


////              MAIN FUNCTION              ////

con_vec Viper2c(situation &s)
{
    con_vec result;
    static int firstcall = 1;
    double glide_out;
    width = s.to_lft + s.to_rgt;
//    STRAIGHT_BORDER = width / 2.0;
    BORDER = 1;
    if (fabs(s.cur_rad)>800)
       BORDER = 10;
    if (s.cur_len < PI / 6)
       BORDER = 0;
    BRK_MP = 1.00;
    CURVESPEEDMULTIPLIER = 6.70;
    if (s.cur_len > PI*.6 && fabs(s.cur_rad)>150)
        CURVESPEEDMULTIPLIER = 6.10; //15
	    
    double middle = (s.to_lft+s.to_rgt)/2;
    double to_end = curvelength(s.to_end, s.cur_rad);
    double lane, before, ideal_alpha, speed, steer_gain;
    prd_rad = s.cur_rad*s.nex_rad;
    double nex_prd_rad = s.nex_rad*s.after_rad;
    nex_chicane = (nex_prd_rad<0 && nex_prd_rad>-250000
                       && (s.nex_len+s.after_len<PI/1.5 || nex_prd_rad>-500))?1:0;
    cur_chicane = (prd_rad<0 && prd_rad>-250000)?1:0;
    direction = asin(s.vn/s.v);
    if (cur_chicane && fabs(s.nex_rad/s.after_rad)<.36)
        cur_chicane = 0;
    else if (s.nex_rad*s.aftaft_rad<0 && s.after_len<190 && 
             s.nex_len*s.aftaft_len<PI/3 && fabs(s.nex_rad>50))
        nex_chicane = 1; 
    if (s.after_rad*s.aftaft_rad>0)
        nex_chicane = 0;
    if (s.cur_len / s.nex_len > 3 && cur_chicane)
        cur_chicane = 0;
    if (s.nex_len / s.after_len > 3 && nex_chicane)
        nex_chicane = 0;
    if (firstcall)
    {
        my_name_is("Viper2c");
        firstcall = 0;
        return result;
    }
    if (s.starting) 
    {
        t = get_track_description ();
        if (s.stage == QUALIFYING)
	    result.fuel_amount = 30;
	else
            result.fuel_amount = MAX_FUEL;
    }
    if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
         return result;
    
    
    //         CALCULATE ALPHA (ANGLE)         //

    if (s.nex_rad == 0)
       before = min(s.v* width*start_dive*(fabs(s.cur_rad/110)+6)/2 + middle,
		       500);
    else 
       before = min(s.v* width*start_dive*(fabs(s.nex_rad/110)+6)/2 +
		       middle,600);

    if (s.nex_rad>0)
        lane = width - STRAIGHT_BORDER;
    else if (s.nex_rad<0)
        lane = STRAIGHT_BORDER;
    else
        lane = middle;

    if (nex_chicane)
       before = min(before, 400);
    
    // Calculate gliding-lane to curve
    if (to_end < before && s.nex_rad != 0)
    {
        if (s.nex_rad < 0)
            lane = max(lane_glide (before+s.nex_len*fabs(500/s.nex_rad), to_end, 
			   s.nex_rad<0?STRAIGHT_BORDER:width-STRAIGHT_BORDER,
			   s.nex_rad<0?width:0), s.to_lft);
        else
            lane = min(lane_glide (before+s.nex_len*fabs(500/s.nex_rad), to_end, 
			   s.nex_rad<0?STRAIGHT_BORDER:width-STRAIGHT_BORDER,
			   s.nex_rad<0?width-5:5), s.to_lft);
    }
 

    // calculate lane in curve or when 'gliding' out of curve
    if (s.cur_rad!=0)
    {
        if (s.cur_rad>0)
	    lane = BORDER;
	else
	    lane = width - BORDER;
	if (s.cur_len/2 > s.to_end && s.to_end < PI/(width>75?5:7)
	    && s.cur_rad != 0 && prd_rad<=0)
	{
            glide_out = 0.9;
	    if (cur_chicane && s.cur_len>PI/4)
                glide_out = .6;
	    else if (cur_chicane && fabs(s.cur_rad)>150 && s.cur_len>PI/4)
                glide_out = .7;
	    else if (s.nex_len > 500 && s.nex_rad == 0)
                glide_out = 1.4;
	    else if ( fabs(s.cur_rad) < 150 && s.cur_len < PI/5)
                glide_out = .7;
	    else if (s.v > 90 && fabs(s.cur_rad) > 400 && s.cur_len > PI*.9)
                glide_out = .7;
	    if (s.cur_rad > 0 /*&& !(cur_chicane && fabs(s.cur_rad)>250 &&
				    s.cur_len>PI/5)*/)
                lane = lane_glide (s.cur_len/2*fabs(s.cur_rad), to_end,
		                   4, cur_chicane ? width/1.7:
				   middle*glide_out);
	    else if (1/*!(cur_chicane && fabs(s.cur_rad)>250 &&
				    s.cur_len>PI/5)*/)
                lane = lane_glide (s.cur_len*fabs(s.cur_rad)/2, to_end, width - 4, 
		                   cur_chicane ? width/3.2:
				   middle*(2-glide_out));
        }
    }
	    
    // calculate steer gain
    if (to_end < before/2 && s.cur_rad == 0)  //1.3
        steer_gain = .7;
    else if (to_end < before && s.cur_rad == 0)  //1.3
        steer_gain = .7;
    else if (to_end > s.cur_len*.9 && s.cur_rad == 0)
        steer_gain = .2;
    else if (fabs(s.cur_rad)>200 && s.to_end<s.cur_len/1.8)
       steer_gain = .6;
    else if (s.cur_rad != 0 && fabs(s.cur_rad)<1500)
       steer_gain = .8;
    else        
        steer_gain = .3;
    
    if (to_end-30 < before && to_end > before && s.cur_rad == 0)  //1.3
    {    
        steer_gain = .6;
	lane = s.to_lft;
    }
				  
    // calculate ideal angle
      ideal_alpha = steer_gain * (s.to_lft - lane) / width - 1.1 * s.vn / s.v;

        

    if (cur_chicane && prd_rad>-20000 && s.to_end<s.cur_len/2 
        && s.to_end<PI/(16.5-width/10))
        ideal_alpha = (ideal_alpha+direction)/2;
    
    if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width*.7)*2 > s.cur_len &&
        fabs(s.cur_len*fabs(s.cur_rad)/s.cur_rad +
             s.nex_len*fabs(s.nex_rad) /s.nex_rad) < .5 &&
	fabs(s.cur_rad*s.cur_len)<100 )
       ideal_alpha = atan((s.to_lft - lane)  / (to_end+width*2));
    result.alpha = ideal_alpha;
    
    
    //            CALCULATE SPEED              //
    if (s.cur_rad==0)
        CURVESPEEDMULTIPLIER = 6.6;

    
    if (fabs(s.cur_rad)>200 && s.cur_len>PI/6 && !(s.cur_len<PI/2.5 &&
			    fabs(s.cur_rad)>500))
    {
        CURVESPEEDMULTIPLIER = 5.95;
	BRK_MP = .95;
    }
    else if (fabs(s.cur_rad)>400 && s.cur_len>PI/6 && !(s.cur_len<PI/2.5 &&
 			    fabs(s.cur_rad)>500))
    {
        CURVESPEEDMULTIPLIER = 5.95;
	BRK_MP = 1.0;
    }
//    else if (cur_chicane && s.v < 80)
  //      CURVESPEEDMULTIPLIER = 6.75;

    if (prd_rad > 0 && s.to_end < .2 )
        CURVESPEEDMULTIPLIER = 5.85;
    if (s.nex_rad*s.aftaft_rad!=0 && s.after_len<300 && s.after_rad==0 &&
		    fabs(s.nex_rad*s.aftaft_rad)<20000)
        BRK_MP = 1.2;

	    
    if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width)*2 > s.cur_len && 
        fabs(s.cur_len*fabs(s.cur_rad)/s.cur_rad + s.nex_len*fabs(s.nex_rad)
        /s.nex_rad) < .2 && prd_rad<0 && fabs(prd_rad) < 90000)
        CURVESPEEDMULTIPLIER = 9.00;
    else if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width)*2 > s.cur_len && 
        fabs(s.cur_len*fabs(s.cur_rad)/s.cur_rad + s.nex_len*fabs(s.nex_rad)
        /s.nex_rad) < .4 && prd_rad<0 && fabs(prd_rad) < 90000)
        CURVESPEEDMULTIPLIER = 7.20;
    else if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width)*2 > s.cur_len && 
        fabs(s.cur_len*fabs(s.cur_rad)/s.cur_rad + s.nex_len*fabs(s.nex_rad)
        /s.nex_rad) < .7 && prd_rad<0 && fabs(prd_rad) < 90000)
        CURVESPEEDMULTIPLIER = 6.90;
	 
    speed = curvespeed(s.cur_rad, s.cur_len);   // normal case
    
    if (fabs(s.cur_rad)*s.cur_len<width*2.5)        // if it's a short turn
         speed += 7;
    if (s.cur_len < PI/4 && prd_rad<=0) // if it's a short turn
         speed *= 1.15;                            // and not double in same
                                                    // direction
   
    // slow down when gliding to curve and going too fast
    if (to_end - before < distancetobraketo (s,150
				    ) && to_end-100 > before &&
		    s.cur_rad == 0)
	    speed = 0;


    // speed up when nearing end of 'normal' curve
        if (s.cur_len/2 > s.to_end && s.cur_rad!=0 && s.to_end < PI/4 && s.cur_rad != 0 && prd_rad<=0)
             speed *= 1.25; //1.2


    
    // speed up when nearing end of a small curve
    if (s.to_end < PI/6 && 
             s.cur_rad != 0 && prd_rad<=0 && s.cur_len<PI*.95 && s.cur_len>PI/4)
        speed += 5;

    // speed up when when nearing end of 1st turn in a chicane

    if (s.to_end < PI/5 && 
        fabs(s.cur_rad)<150  && fabs(s.nex_rad)<150 && cur_chicane &&
	s.cur_len < PI/2)
        speed = s.v + 20; 
    else if (s.to_end < PI/3 && fabs(direction) < 10 && 
        cur_chicane && s.cur_len < PI/2)
        speed = s.v + 20;

    // speed up when nearing end of 'normal' curve
    if (s.cur_len/2 > s.to_end && s.cur_rad!=0 && s.to_end < PI/4 && 
        s.cur_rad != 0 && prd_rad<=0)
        speed *= 1.25; //1.2

    // slow down if the last of 2 turns in the same direction
    // is smaller than the 1st one
    if (s.to_end < PI/4 && prd_rad > 0 && 
        ((fabs(s.cur_rad)/fabs(s.nex_rad)>1.5 && fabs(s.cur_rad)<750) || 
	(s.cur_len/s.nex_len<.7 && fabs(s.cur_rad)<750)) && fabs(s.nex_rad)<2000)
	speed*=.6;

    if (fabs(s.nex_rad)<100 && s.nex_len>PI/10 && nex_prd_rad<=0)
        BRK_MP = 1.0;

    // slow down when nearing a chicane
    if (nex_chicane) 
    {   
        if (to_end < distancetobraketo (s, curvespeed (s.nex_len<1.0 ? 
	    max_radius(s.nex_rad, s.nex_len)*(fabs(s.nex_rad)-fabs(s.after_rad)<-100?
	    .78:.7): s.nex_rad*1.25, s.nex_len) ))
            speed = 0;
    }

    // slow down when nearing 'normal' curve
    else if (to_end < distancetobraketo (s, curvespeed (s.nex_len<.64 ?  //.69
             max_radius(s.nex_rad, s.nex_len)*
	     .74:fabs(s.nex_rad)/s.nex_len<45?fabs(s.nex_rad*.8): //<40   *.7
	     (s.nex_len>PI*.95?s.nex_rad*1.2:s.nex_rad*1.1), s.nex_len ) )) 
        speed = 0;

    // slow down in case we are going too fast for next curve
    else if (s.after_rad!=0 && to_end + curvelength (s.nex_len, s.nex_rad) < 
             distancetobraketo (s, curvespeed(s.after_rad, s.after_len)*
	     (nex_prd_rad>0?.06: 
	     (s.cur_rad*s.after_rad>0 && s.nex_rad == 0?.6:1.0))) )
        speed = 0;

    // slow down in case we are going too fast for the curve after the next one
    else if (s.aftaft_rad!=0 && to_end + curvelength (s.nex_len, s.nex_rad)+
             curvelength(s.after_len,s.after_rad) < distancetobraketo (
	     s, curvespeed(s.aftaft_rad, s.aftaft_len)
	     *(s.after_rad*s.aftaft_rad>0?.6:1.0)) )
        speed = 0;
	
    result.request_pit=0;

    for (int i = 0; i < 3; i++)
    {
        if (s.nearby[i].who != 999 && s.nearby[i].rel_ydot < -10.0 
	    && fabs(s.nearby[i].rel_x + (s.vn - s.nearby[i].rel_xdot) *
	    s.nearby[i].rel_y /  s.nearby[i].rel_ydot) <= 2.0 * CARWID &&
	    s.nearby[i].rel_y - 1.5 * CARLEN<
	    distancetobraketo(s, s.v + s.nearby[i].rel_ydot))
        {
            if (s.to_lft < 20)
	       result.alpha-=.05;
	    else
	       result.alpha+=.05;
        }
    }

    if (s.dead_ahead)  
    {   
//        speed = s.v * .95;
        if (s.to_lft>middle)
             result.alpha +=.05;
        else
             result.alpha -=.05;
    }
					  
    if (s.stage != QUALIFYING && (s.damage > 20000 || s.fuel < 10))
    {
        result.request_pit=1;
        result.repair_amount=s.damage;
        result.fuel_amount=MAX_FUEL;
    }
    result.vc = speed;
    return result;
}
