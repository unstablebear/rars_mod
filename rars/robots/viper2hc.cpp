//////////////////////////////////////////////////
//   VIPER(2) Hybrid  a robot for RARS 0.72	//
//   By Tom Pycke <Tom.Pycke@advalvas.be>	//
//   Source is	public. If you wish to use any	//
//   of my code, please let me know.		//
//   I wrote it using Linux. Please  check out	//
//   this great OS. And it's free!		//
//   This  car is  lane-driven and coordinate-	//
//   driven (for chicanes, but	still problems	//
//   with speed), but i my next car  will only	//
//   be coordinate-driven since  this is still	//
//   the fastest method.			//
//////////////////////////////////////////////////

////		    INCLUDES		      ////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "car.h"
#include "track.h"

////		     MACROS		      ////

#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))
#define sqr(x) x*x
#define distfromto(a,b,c,d)  sqrt((a-c)*(a-c) + (b-d)*(b-d))
#define   RTD	     (180.0/PI)
#define   RTANGLE    (90.0/RTD)

////		    VARIABLES		      ////
static double CURVESPEEDMULTIPLIER  =  5.9, DEFAULT_SPD_MP=6.9;
static double width, spdmp=1.02, CRN_LINE = .70;
static double BORDER = 0;
static double STRAIGHT_BORDER = 20;
static int cur_chicane, nex_chicane, data=0, segnum;
static double direction, prd_rad;
static double BRK_MP;
static double start_dive = 0.016, data_spdmp[70];
static track_desc t;

static track_desc track;
static double track_width, damage;
static double gc_dot, gc_x, gc_y, gc_lastx, gc_lasty;


////		    FUNCTIONS		     ////

void read_data()
{
    FILE *dat;
    int cnt;
    char line[128];
    dat = fopen ("viper2h.dat", "r");
    for (fscanf (dat, "%s",(char *)&line);fscanf (dat, "%s", (char *)&line)!=EOF;)
    {
	if (!strcmp (line, track.sName))
	{
	    for (cnt=0, data=1;cnt<=track.NSEG;cnt++)
	    {
		fscanf (dat, "%s", (char *)&line);
		data_spdmp[cnt] = strtod (line, NULL);
		STRAIGHT_BORDER = 8;

		
	    }
	    break;
	}
    }
}

void setdefault_spd_mp ()
{
    if (!strncmp(track.sName, "spa",3))
	DEFAULT_SPD_MP = 6.6;
    if (!strncmp(track.sName, "monaco",6))
	DEFAULT_SPD_MP = 8.05;
    if (!strncmp(track.sName, "imola",5))
	DEFAULT_SPD_MP = 7.3;
    if (!strncmp(track.sName, "speed2",6))
	DEFAULT_SPD_MP = 8.2;
    if (!strncmp(track.sName, "brazil",6))
	DEFAULT_SPD_MP = 6.3;
    if (!strncmp(track.sName, "jerez",5))
	DEFAULT_SPD_MP = 7.7;
    if (!strncmp(track.sName, "mosport",7))
	DEFAULT_SPD_MP = 6.9;
    if (!strncmp(track.sName, "watglen",7))
	DEFAULT_SPD_MP = 6.4;
}

static double curveradiuslow(int segnum)
{
    if (track.trackin[segnum].radius >= 0)
       return track.trackin[segnum].radius;
    else
       return track.trackin[segnum].radius + track.width;
}

static void normalize(double dot, double &angle)
{
    if (fabs(dot-angle) > fabs(dot-(angle+2*PI)))
       angle+= 2*PI;
    if (fabs(dot-angle) > fabs(dot-(angle-2*PI)))
       angle-= 2*PI;
}

static double angle (double x1, double y1, double x2, double y2)
{
    double a;
    if (x2 == x1)
       if (y2 < y1)
	   return PI/2;
       else
	   return PI*1.5;
    if (x2 < x1)
       a = atan( (y2-y1) / (x2-x1) ) + PI;
    else
       a = atan( (y2-y1) / (x2-x1) );
    return a;
}

static void getabscoord_str(int segnum, double tolft, double toend,
			    double &x, double &y)
{
    x =  track.trackin[segnum].end_x-toend*cos(track.trackin[segnum].beg_ang)
	 + tolft*cos(track.trackin[segnum].beg_ang-RTANGLE);

    y =  track.trackin[segnum].end_y- toend*sin(track.trackin[segnum].beg_ang)
	 + tolft*sin(track.trackin[segnum].beg_ang-RTANGLE);
}

static void getabsxy(int segnum, double tolft, double torgt, double toend)
{
    if (track.trackin[segnum].radius == 0)
	getabscoord_str(segnum, tolft, toend, gc_x, gc_y);
    else
    {
	double anglefromctr;
	if (track.trackin[segnum].radius > 0)
	{
	    anglefromctr = track.trackin[segnum].end_ang - toend - RTANGLE;
	    gc_x = track.trackin[segnum].cen_x + (tolft + fabs(track.trackin[segnum].radius))*cos(anglefromctr);
	    gc_y = track.trackin[segnum].cen_y + (tolft + fabs(track.trackin[segnum].radius))*sin(anglefromctr);
	}
	else
	{
	    anglefromctr = track.trackin[segnum].end_ang + toend + RTANGLE;
	    gc_x = track.trackin[segnum].cen_x + (torgt + fabs(track.trackin[segnum].radius)-track_width) * cos(anglefromctr);
	    gc_y = track.trackin[segnum].cen_y + (torgt + fabs(track.trackin[segnum].radius)-track_width) * sin(anglefromctr);
	}
    }
    gc_dot = angle(gc_lastx, gc_lasty, gc_x, gc_y);
    gc_lastx = gc_x;
    gc_lasty = gc_y;
}

static double maxangleinturn(double r, double w, double cushion)
{
    double answer;
    if (r > 0)
    {
	r+= cushion;
	w-= cushion;
	if (w > r+w)
	    answer = 1;
	if (w < 0)
	    answer = -.02;
	else
	    if (r+w == 0.0)
	       answer = acos(0.0);
	    else
	       answer = acos(1.0-(w/(r+w)));
    }
    else
    {
	r = -r;
	r+= cushion;
	w-= cushion;
	if (w > r+w)
	   answer = 1;
	if (w < 0)
	   answer = -.02;
	else
	   if (r+w == 0.0)
	       answer = acos(0.0);
	   else
	       answer = acos(1.0-(w/(r+w)));
       answer *= -1;
   }

   return answer;
}

static double angleforcurve(int segnum)
{
    double angletoctr, disttowall, answer;

    disttowall = distfromto(gc_x, gc_y, track.trackin[segnum].cen_x, track.trackin[segnum].cen_y) - fabs(curveradiuslow(segnum));

    angletoctr = angle(gc_x, gc_y, track.trackin[segnum].cen_x, track.trackin[segnum].cen_y);

    answer = maxangleinturn(curveradiuslow(segnum), disttowall, 
	     fabs(track.trackin[segnum].radius)<80?cur_chicane?8.0:0.0:cur_chicane?track.trackin[segnum].length<PI/5?25.0:5.0:5.0);
//	     1.0:10.0); //:20.0

    if (track.trackin[segnum].radius > 0)
       answer = angletoctr-RTANGLE+answer;
    else
       answer = angletoctr+RTANGLE+answer;

    normalize(gc_dot, answer);
    return answer;
}


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
    return (radius>100?.80:1.00)*fabs( ( (radius - (radius + width/2) / fabs(sin(alpha)) ) / 
	       (1 - fabs(sin(alpha))) * fabs(sin(alpha))))*.8; 
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
    dist = .0549 * (s.v+vthen)*(s.v-vthen)/(1.7*2.0)*BRK_MP*.98*.93;
    if (s.cur_rad==0 && data)
	dist*=data_spdmp[segnum];
    return dist;
}

// calculate speed for a given radius and angle
static double curvespeed(double radius, double len)
{
    double speed;
    radius = fabs(radius);
    if (radius == 0.0)
	return(400.0);
   
   double friction = 1.05 * (1.0 - exp (-9.0 / 2.5));
   double useful_friction = 0.975 * friction - 0.2 * damage * damage / 9e8;
   speed =  sqrt (fabs (radius) * g * useful_friction)* 
	   ((len<1.28 && (len>.97 || radius>200))? 
	   (fabs(radius)<360?1.0:1.00):1.0) * 1.11;

     speed = CURVESPEEDMULTIPLIER * sqrt(fabs(radius)) * 
	   ((len<1.28 && (len>.97 || radius>200))? 
	   (fabs(radius)<360?1.0:1.00):1.0);
  if (prd_rad<=0)
  {
   if (fabs(radius)<110) 
	speed *= min(max(pow(len/1.8,-1),1.01),1.50); 
//   if (radius < 160 && len >.8 && len < 1.3)
  //	 speed *= .9;
   if (len > PI/4.5 && len < PI / 3.5 && radius < 150)
       speed *= 1.05; 
   if (radius < 65)	     // some (stupid) adjustments
       speed *= (len < .55 ? 1.5:1.1);
   if (radius < 30 && len>PI/4)
       speed *= 1.3;
//   if (len > PI*.9 && radius > 300)
  //	 speed *= .9;
   if (radius<80 && len<PI/2)
       speed = max(speed,60); 
   if (radius<40)
       speed = max(speed,30); 
   if (radius > 220 && radius < 500 && len>PI/1.5) 
       speed *= .95;
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


////		  MAIN FUNCTION		     ////

con_vec Viper2H(situation &s)
{
    con_vec result = {0.0f, 0.0f };
    static int firstcall = 1;
    double glide_out;
    double ideal_alpha, nextcurvealpha;
    int nxtseg, prvseg, nxtnxtseg, nxtnxtnxtseg, nxtnxtnxtnxtseg;
    width = s.to_lft + s.to_rgt;
//    STRAIGHT_BORDER = width / 2.0;
    double middle = (s.to_lft+s.to_rgt)/2;
    double to_end = curvelength(s.to_end, s.cur_rad);
    double lane, before, speed, steer_gain;
    double nex_prd_rad = s.nex_rad*s.after_rad;
    int brk_it = 0, do_small=0;
    spdmp=1.03;
    damage = s.damage;
    prd_rad = s.cur_rad*s.nex_rad;
    BORDER = 1;
//  if (fabs(s.cur_rad)>800)
//     BORDER = 8;
    if (s.cur_len < PI / 6)
       BORDER = 0;
    if (s.nex_len > PI/3)
	BRK_MP = 1.11;//1.10;
    else if (fabs(s.nex_rad)<100 && s.nex_len > PI/4)
       BRK_MP = 1.1;
    else
	BRK_MP = 1.0;
    CURVESPEEDMULTIPLIER = DEFAULT_SPD_MP;
    if (s.cur_len > PI*.6 && fabs(s.cur_rad)>150)
	CURVESPEEDMULTIPLIER = 6.10; //15
	    
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
    if (cur_chicane && s.cur_len+s.nex_len>PI/1.5)
	cur_chicane = 0;
    if (firstcall)
    {
	my_name_is("Viper2Hb");
	firstcall = 0;
	return result;
    }
    if (s.starting) 
    {
	setdefault_spd_mp ();
	track = get_track_description();
	t = get_track_description ();
	if (s.stage == QUALIFYING)
	    result.fuel_amount = 30;
	else
	    result.fuel_amount = MAX_FUEL;
	read_data();
    }
    segnum = s.seg_ID;
    nxtseg = (segnum+1)%track.NSEG;
    prvseg = (segnum-1)%track.NSEG;
    nxtnxtseg = (segnum+2)%track.NSEG;
    nxtnxtnxtseg = (segnum+3)%track.NSEG;
    nxtnxtnxtnxtseg = (segnum+4)%track.NSEG;
    track_width = (s.to_lft+s.to_rgt);
//  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
//	 return result;
    
    
    //	       CALCULATE ALPHA (ANGLE)	       //

    if (s.nex_rad == 0)
       before = min(s.v* width*start_dive*(fabs(s.cur_rad/110)+6)/2 + middle,
		       500);
    else 
       before = min(s.v*(s.nex_rad>0?(s.to_lft+25):(s.to_rgt+25))*start_dive
		    * (fabs(s.nex_rad/100)+6)/2/* + middle*/,1900); //25
    if (s.nex_rad>0)
	lane = width - STRAIGHT_BORDER;
    else if (s.nex_rad<0)
	lane = STRAIGHT_BORDER;
    else
	lane = middle;

    if (nex_chicane)
       before = min(before, 1450);
    before *= .3;//.8;
    if (s.nex_rad*s.after_rad<0)
	before*=1.3;
    
    // Calculate gliding-lane to curve
    if (to_end < before && s.nex_rad != 0)
    {
	if (s.nex_rad < 0)
	    lane = max(lane_glide (before+s.nex_len*fabs(500/s.nex_rad), to_end, 
			   s.nex_rad<0?STRAIGHT_BORDER:width-STRAIGHT_BORDER,
			   s.nex_rad<0?width-15:15), s.to_lft);
	else
	    lane = min(lane_glide (before+s.nex_len*fabs(500/s.nex_rad), to_end, 
			   s.nex_rad<0?STRAIGHT_BORDER:width-STRAIGHT_BORDER,
			   s.nex_rad<0?width-15:15), s.to_lft);
    }
 

    // calculate lane in curve or when 'gliding' out of curve
    if (s.cur_rad!=0)
    {
	if (s.cur_rad>0)
	    lane = BORDER;
	else
	    lane = width - BORDER;
	if (s.cur_len/2 > s.to_end && s.to_end < PI/(width>75?4:4)
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
		glide_out = 1.1;
	    glide_out = 70/s.v; //.8
	    spdmp = 1.05;
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
	else
	    do_small=1;
	
    }
	    
    // calculate steer gain
    if (to_end < before/2 && s.cur_rad == 0)  //1.3
	steer_gain = .8;
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
    if (do_small && s.to_end < s.cur_len*.85)
    {
	nextcurvealpha = angleforcurve(segnum);
	ideal_alpha = nextcurvealpha -gc_dot;
    }

	

/*    if (cur_chicane && prd_rad>-20000 && s.to_end<s.cur_len/2 
	&& s.to_end<PI/(16.5-width/10))
	ideal_alpha = (ideal_alpha+direction)/2;*/
    
    if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width*.7)*2 > s.cur_len &&
	fabs(s.cur_len*fabs(s.cur_rad)/s.cur_rad +
	     s.nex_len*fabs(s.nex_rad) /s.nex_rad) < .5 &&
	fabs(s.cur_rad*s.cur_len)<100 )
    {	ideal_alpha = atan((s.to_lft - lane)  / (to_end+width*2));
	}
    result.alpha = ideal_alpha;
    
    getabsxy(s.seg_ID, s.to_lft, s.to_rgt, s.to_end);

    if (s.cur_rad == 0.0 && s.nex_rad*s.after_rad == 0)
	ideal_alpha = angleforcurve(nxtseg);
    else if (!nex_chicane)
    {
	  ideal_alpha = angleforcurve(segnum);
	  if (curveradiuslow(segnum) * curveradiuslow(nxtseg) < 0 )
	  {
	      nextcurvealpha = angleforcurve(nxtseg);
	      if ( (curveradiuslow(segnum)>0 && nextcurvealpha<ideal_alpha) || (curveradiuslow(segnum)<0 && nextcurvealpha>ideal_alpha)) 
		 ideal_alpha = nextcurvealpha;
	      result.alpha = ideal_alpha - gc_dot;
	  }
    }

    if (to_end < before && s.cur_rad==0) 
	 result.alpha = (result.alpha + angleforcurve(nxtseg)*2 - gc_dot*2)/3;

    if (s.nex_rad*s.after_rad<0 && s.after_rad!=0 && to_end < 150 &&
	fabs(s.nex_rad)/(fabs(s.nex_rad)+width) > s.nex_len*CRN_LINE && 
	fabs(s.nex_rad)*s.nex_len < middle)
    {
	  nextcurvealpha = angleforcurve(nxtnxtseg);
	  ideal_alpha = nextcurvealpha;
	  result.alpha = ideal_alpha - gc_dot;
	  spdmp = 1.05;
	  BRK_MP = .6;
//	  goto brk;
    }
    if (s.cur_rad*s.nex_rad<0 && to_end < 150 && fabs(s.nex_rad)*s.nex_len<middle && 
	fabs(s.nex_rad)/(fabs(s.nex_rad)+width) > s.nex_len*CRN_LINE)
    {
	if (s.aftaft_rad!=0)
	{
	  nextcurvealpha = angleforcurve(nxtnxtnxtseg);
	  ideal_alpha = nextcurvealpha;
	  result.alpha = ideal_alpha - gc_dot;
	}
	else
	{
	  ideal_alpha = angleforcurve(nxtnxtnxtseg);
	  nextcurvealpha = angleforcurve(nxtnxtnxtnxtseg);
	  ideal_alpha = nextcurvealpha;
	  result.alpha = ideal_alpha - gc_dot;
	}
	spdmp = 1.05;
	
	BRK_MP = .6;
//	goto brk;
    }

    //		  CALCULATE SPEED	       //

    
    if (fabs(s.cur_rad)>400 && s.cur_len>PI/6 && !(s.cur_len<PI/2.5 &&
			    fabs(s.cur_rad)>500))
    {
	CURVESPEEDMULTIPLIER = 5.95;
	BRK_MP = 1.0;
    }
    else if (cur_chicane && s.v < 80)
	CURVESPEEDMULTIPLIER = 6.75;
    if (s.cur_len > PI*.4 && s.cur_rad!=0 && s.nex_rad==0)
	CURVESPEEDMULTIPLIER = 6.2;
    if (cur_chicane && s.nex_len < PI/3)
	BRK_MP = .95;

    if (prd_rad > 0 && s.to_end < .15 ) //.2
	CURVESPEEDMULTIPLIER = 5.75; //5.85
	    
/*    if (fabs(s.cur_rad)/(fabs(s.cur_rad)+width)*2 > s.cur_len && 
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
	CURVESPEEDMULTIPLIER = 6.90;*/
	 
    speed = curvespeed(s.cur_rad, s.cur_len);	// normal case
    
    if (s.cur_rad != 0)
    {
	  ideal_alpha = angleforcurve(segnum);
	  if (curveradiuslow(segnum) * curveradiuslow(nxtseg) < 0)
	  {
	      nextcurvealpha = angleforcurve(nxtseg);
	      if ( (curveradiuslow(segnum)>0 && nextcurvealpha<ideal_alpha) || (curveradiuslow(segnum)<0 && nextcurvealpha>ideal_alpha)) 
	      {
		 ideal_alpha = nextcurvealpha;
		 speed = sqrt(max_radius(s.cur_rad, s.cur_len))*6.5;
	      }
	//	result.alpha = (ideal_alpha - gc_dot);
	  }
    }
    
	    
    if ( fabs(angleforcurve (nxtseg) - gc_dot - result.alpha) < .3 && 
	 prd_rad<=0 && nex_prd_rad<=0)
    {
	 ideal_alpha = angleforcurve(nxtseg);
	 nextcurvealpha = angleforcurve(nxtnxtseg);
	 ideal_alpha = nextcurvealpha - gc_dot;
	 if (fabs(ideal_alpha - result.alpha) < .1)
	 {
	    if (s.nex_len<PI/3)
		BRK_MP *= .01;
	    else
		BRK_MP *= .8;
	    speed *= 4.0;
	 }
	 else if (fabs(ideal_alpha - result.alpha) < .2)
	 {
	    speed *= 1.6;
	    if (s.nex_len<PI/4)
		BRK_MP *= .01;
	    else
		BRK_MP *= .8;
	 }
	 else if (fabs(ideal_alpha - result.alpha) < .4)
	 {
	    if (s.nex_len<PI/4)
		BRK_MP *= .5;
	    else
		BRK_MP *= .9;
	    speed *= 1.2;
	 }
	 if (fabs(ideal_alpha - result.alpha) < .4)
	 {
	    ideal_alpha = angleforcurve(nxtnxtseg);
	    nextcurvealpha = angleforcurve(nxtnxtnxtseg);
	    ideal_alpha = nextcurvealpha - gc_dot;
	    if (fabs(ideal_alpha - result.alpha) < .1)
	    {
		BRK_MP *= .2;
		speed *= 1.9;
	    }
	    else if (fabs(ideal_alpha - result.alpha) < .2)
	    {
		BRK_MP *= .40;
		speed *= 2.0;
	    }
	    else if (fabs(ideal_alpha - result.alpha) < .4)
	    {
		BRK_MP *= .70;
		speed *= 1.7;
	    }
	    else if (fabs(ideal_alpha - result.alpha) < .6)
	    {
		BRK_MP *= .9;
		speed *= 1.2;
	    }
	 }
    }
    
    if (fabs(s.cur_rad)*s.cur_len<width*2.5)	    // if it's a short turn
	 speed += 7;
    if (s.cur_len < PI/4 && prd_rad<=0) // if it's a short turn
	 speed *= 1.15;				   // and not double in same
						    // direction
   
    // slow down when gliding to curve and going too fast
    if (to_end - before < .0549 * (s.v+100)*(s.v-100)/(1.7*100) && s.cur_rad == 0 &&
	to_end > before && brk_it)
    {
	speed = s.v*.8;
    }


    // speed up when nearing end of 'normal' curve
    if (s.cur_len/2 > s.to_end && s.cur_rad!=0 && s.to_end < PI/5 && 
	s.cur_rad != 0 && prd_rad/*<*/==0)
       speed *= 1.35; //1.25


    
    // speed up when nearing end of a small curve
    if (s.to_end < PI/6 && 
	     s.cur_rad != 0 && prd_rad<=0 && s.cur_len<PI*.95 && s.cur_len>PI/4)
	speed += 5;

    // speed up when when nearing end of 1st turn in a chicane

/*  if (s.to_end < PI/5 && 
	fabs(s.cur_rad)<150  && fabs(s.nex_rad)<150 && cur_chicane &&
	s.cur_len < PI/2)
	speed = s.v + 20; 
    else if (s.to_end < PI/3 && fabs(direction) < 10 && 
	cur_chicane && s.cur_len < PI/2)
	speed = s.v + 20;*/

    // speed up when nearing end of 'normal' curve
    if (s.cur_len/2 > s.to_end && s.cur_rad!=0 && s.to_end < PI/4 && 
	s.cur_rad != 0 && prd_rad<=0)
	speed *= 1.25; //1.2

    // slow down if the last of 2 turns in the same direction
    // is smaller than the 1st one
    if (s.to_end < PI/5 && prd_rad > 0 && 
	((fabs(s.cur_rad)/fabs(s.nex_rad)>1.5 && fabs(s.cur_rad)<750) || 
	(s.cur_len/s.nex_len<.7 && fabs(s.cur_rad)<750)) && fabs(s.nex_rad)<2000 &&
	fabs(s.nex_rad)>70 && fabs(s.cur_rad)>70)
	speed*=.8; //.7

    if (fabs(s.cur_rad)>350 && s.cur_len>PI/1.5)
	spdmp = .99;

    //BRK_MP = 0.9;*/
    if (fabs(s.cur_rad)<100)
	speed = 6.0 * sqrt (max_radius (s.cur_rad, s.cur_len));
    if (!cur_chicane && s.cur_len<PI/2)
	spdmp = 1.05;
    if (s.nex_rad*s.after_rad<0 && fabs(s.after_rad)/3 > fabs(s.nex_rad)
	&& s.cur_rad==0)
	BRK_MP = 1.15;
/*  if (s.to_end < PI/4 && s.nex_rad==0 && fabs(s.cur_rad)>250 && to_end<s.cur_len/2.2)
	speed*=1.10;
    */	

    // slow down when nearing a chicane
    if (nex_chicane) 
    {	
	if (to_end < distancetobraketo (s, curvespeed (s.nex_len<1.0 ? 
	    max_radius(s.nex_rad, s.nex_len)*(fabs(s.nex_rad)-fabs(s.after_rad)<-100?
	    .78:.7): s.nex_rad*1.25, s.nex_len) )) 
	    speed = 0;
    }

    // slow down when nearing 'normal' curve
    else if ((to_end-before/50)<0?0:to_end-before/50 < min(distancetobraketo (s, curvespeed (s.nex_len<.64 ?	//.69
	     max_radius(s.nex_rad, s.nex_len)*
	     .74:fabs(s.nex_rad)/s.nex_len<45?fabs(s.nex_rad*.8): //<40   *.7
	     (s.nex_len>PI*.95?s.nex_rad*1.2:s.nex_rad*1.1), s.nex_len ) ),
	     distancetobraketo (s, 5.5*sqrt(max_radius(s.nex_rad,s.nex_len)))*1.3))
	     /*&& fabs(s.nex_rad)>100)*/ 
	speed = 0;
    else if (to_end < distancetobraketo (s, curvespeed (s.nex_len<.64 ?  //.69
	     max_radius(s.nex_rad, s.nex_len)*
	     .74:fabs(s.nex_rad)/s.nex_len<45?fabs(s.nex_rad*.8): //<40   *.7
	     (s.nex_len>PI*.95?s.nex_rad*1.2:s.nex_rad*1.1), s.nex_len )) &&s.cur_rad*s.nex_rad>0 )
	     /*&& fabs(s.nex_rad)>100)*/ 
	speed = 0;
/*  else if (to_end < distancetobraketo (s, 5.5*sqrt(max_radius(s.nex_rad,s.nex_len)))*1.3 && fabs(s.nex_rad)<100)
	speed = 0;*/

    // slow down in case we are going too fast for next curve
    if (s.v > 60 && fabs(s.after_rad)>250)
	BRK_MP = 1.0;
    
    if (s.after_rad!=0 && to_end + curvelength (s.nex_len, s.nex_rad) < 
	     distancetobraketo (s, curvespeed(s.after_rad, s.after_len)*
	     (nex_prd_rad>0?.06: //0.06 
	     (s.cur_rad*s.after_rad>0 && s.nex_rad == 0?.6:1.0))) )
	speed = 0;				       //6

    // slow down in case we are going too fast for the curve after the next one
    else if (s.aftaft_rad!=0 && to_end + curvelength (s.nex_len, s.nex_rad)+
	     curvelength(s.after_len,s.after_rad) < distancetobraketo (
	     s, curvespeed(s.aftaft_rad, s.aftaft_len)
	     *(s.after_rad*s.aftaft_rad>0?0.6:1.0)) )
	speed = 0;		     //   0.6
	
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
	speed *= .95;
	if (s.to_lft>middle)
	     result.alpha +=.07;
	else
	     result.alpha -=.07;
    }
					  
    if (s.stage != QUALIFYING && (s.damage > 20000 || s.fuel < 10))
    {
	result.request_pit=1;
	result.repair_amount= int( s.fuel<10?s.damage/1.5:s.damage );
	result.fuel_amount=MAX_FUEL;
    }
    if (s.v < 35 && speed < 40)
	speed = 45;
    result.vc = speed*spdmp* (width/100/10+.9)*1.02;
    if (result.vc < s.v)
	result.vc = s.v*.8;
    if (data==1)
    {	
	result.vc*=data_spdmp[segnum]; }
    return result;
}

