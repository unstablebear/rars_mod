/*
    rscott@alpha.netusa.net
    It may be published freely.
*/


#include <stdio.h>
#include <string.h>

#include <math.h>

#include "car.h"
#include "track.h"

#define distfromto(a,b,c,d)  sqrt((a-c)*(a-c) + (b-d)*(b-d))

#define   PI   3.14159265
#define   RTD   (180.0/PI)
#define   RTANGLE    (90.0/RTD)
#define   OMCOS(x)       (1.0-cos(x))
#define SIGN(x)   (x<0.0 ? -1: (x>0 ? 1:0))
#define BRAKECONSTANT  1.9

// to find out info for laps_to_go
//extern Car* pcar[];
extern int car_count;

static FILE *outf;

static track_desc myTrack;

static double gc_dot, gc_x, gc_y, gc_lastx, gc_lasty;  
// what was our position during last call

static double last_alpha, last_vc;

// variables used by my once per lap routines
static int lastSeg, numLapsInRace, lastCarInFront, ignoreDamage, maxDamageTaken, repair_amount;
static unsigned long lastDamage, maxDamagePerLap;
static int myLtg= 9999;  // my laps to go (based off lead car!)

static int SCHseg = 9999;  // special curve handling segment number
static int SCHtype = 0;  // special curve handling type ( 0 = inside-inside, 1, inside-outside, 2 = outside-outside)

#define min(x,y) (((x)>(y))?(y):(x))



//  converts the angle passed in to the closest angle to the
//  current direction of travel (also passed in)
static void normalize(double dot, double &angle)
{
// since our routine may return 6.1 where the direction of
// travel is .02  this routine will make the 6.1 look like
// -.18 which is what the program would be looking for

if (fabs(dot-angle) > fabs(dot-(angle+2*PI)))
  angle+= 2*PI;
if (fabs(dot-angle) > fabs(dot-(angle-2*PI)))
  angle-= 2*PI;
}


// returns the angle from the first set of coordinates to the second
static double anglefromto(double x, double y, double destx, double
desty)
{
double a;

if (destx == x)
  if (desty < y)
    return RTANGLE;
   else
    return 270/RTD;

if (destx < x)
  a = atan( (desty-y) / (destx-x) )+PI;
 else
  a = atan( (desty-y) / (destx-x) );

return a;
}



// given a ra ius, distance from wall and cushion from wall return optimal
// angle to pass around curve (angle is relative to tangent)
static double maxangleinturn(double r, double w, double cushion)
{
double answer;

if (r > 0)
  {
  r+= cushion;
  w-= cushion;

  if (w > r+w)
    answer = 1;
  if (w < 0 )
    answer = 0;       //  if we are inside wall, just move to outside
   else
  if (r+w == 0.0)
    answer = acos(0.0);  //  straight
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
  if (w < 0 )
    answer = 0;
   else
  if (r+w == 0.0)
    answer = acos(0.0);
   else
    answer = acos(1.0-(w/(r+w)));

  answer *= -1;  //  we are going the other way around the circle so flip sign
  }

return answer;
}


//  given a curve distance, radius and the width of a track, return a
//  new radius of a 'racing curve'
static double curveoptradius(double radius, double turnlen, double trkwid)
{
turnlen/=2.0;

if (radius <0)
  return -(trkwid-radius*OMCOS(turnlen))/OMCOS(turnlen);
 else
  return (trkwid+radius*OMCOS(turnlen))/OMCOS(turnlen);
}



// Looks at the track data structure for the radius of a given
// segment and
// returns it normalized with respect to left/right turns
static double curveradiuslow(int segnum)
{
// since the radius is not the radius of the inside of the curve
// necessarily we will return what IS the inside radius for the curve
// by decreasing the right hand turn radius. (which is still negative)

if (myTrack.trackin[segnum].radius == 0)
  return 0;

if (myTrack.trackin[segnum].radius > 0)
  return myTrack.trackin[segnum].radius;
 else
  return myTrack.trackin[segnum].radius + myTrack.width;
}



// get center of opt radius
// passed in the segnum and the new radius and the original radius
static void getctrxy(int segidx, double oldradius, double newradius,
double &ctrx, double &ctry)
{
double turnamt,
       radiusdif=fabs(newradius-oldradius);

turnamt = (PI-myTrack.trackin[segidx].length)/2;
if (myTrack.trackin[segidx].radius > 0)
  turnamt = myTrack.trackin[segidx].end_ang + turnamt;
 else
  turnamt = myTrack.trackin[segidx].end_ang - turnamt;

ctrx +=  cos(turnamt)*(radiusdif);
ctry +=  sin(turnamt)*(radiusdif);
}


static void getCurveStatsLow(double radius, double width,
			     double maxLength, double baseAng, double baOffset,
			     double &ctrx, double &ctry, double &rad)
{
double touchpt;
rad = radius;
  
// maxLength = PI/2.0 - maxLength;

if (radius <0)
  rad = -(width-radius*OMCOS(maxLength))/OMCOS(maxLength);
else
  rad =  (width+radius*OMCOS(maxLength))/OMCOS(maxLength);
  
baOffset = PI/2.0 - baOffset;


if (radius > 0)
  touchpt = baseAng + baOffset;
else
  touchpt = baseAng - baOffset;

//  ctrx = myTrack.trackin[segidx].cen_x;
//  ctry = myTrack.trackin[segidx].cen_y;
ctrx +=  cos(touchpt)*fabs(rad-radius);
ctry +=  sin(touchpt)*fabs(rad-radius);
}


static void setNormalLane(int segnum)
  {
  SCHseg = 9999;
  SCHtype = 0;

  if(outf)
    fprintf(outf, "setting normal lane %d\n", segnum);
  }



static void setInsideLane(int segnum)
  {
  SCHseg = segnum;
  SCHtype = 0;

  if(outf)
    fprintf(outf, "setting inside lane %d\n", segnum);
  
  if (curveradiuslow(segnum) == 0.0)
    SCHseg = (segnum+1) % myTrack.NSEG;
  }


static void setIn2OutLane(int segnum)
  {
  SCHseg = segnum;
  SCHtype = 1;

  if(outf)
    fprintf(outf, "setting inside-outside lane %d\n", segnum);
  
  if (curveradiuslow(segnum) == 0.0)
    SCHseg = (segnum+1) % myTrack.NSEG;
  }


static void setOut2InLane(int segnum)
  {
  SCHseg = segnum;
  SCHtype = 3;

  if(outf)
    fprintf(outf, "setting outside-inside lane %d\n", segnum);
  
  if (curveradiuslow(segnum) == 0.0)
    SCHseg = (segnum+1) % myTrack.NSEG;
  }


static void setOutsideLane(int segnum)
  {
  SCHseg = segnum;
  SCHtype = 2;

  if(outf)
    fprintf(outf, "setting outside lane %d\n", segnum);
  
  if (curveradiuslow(segnum) == 0.0)
    SCHseg = (segnum+1) % myTrack.NSEG;
  }


static void setSlowOutsideLane(int segnum)
  {
  SCHseg = segnum;
  SCHtype = 4;

  if(outf)
    fprintf(outf, "setting slow outside lane %d\n", segnum);
  
  if (curveradiuslow(segnum) == 0.0)
    SCHseg = (segnum+1) % myTrack.NSEG;
  }




static double curvetrackwidth(int segnum)
{
double answer;
double maxcurveexpansion =.7;
// const char *trackfile[current_track] = trackfile[current_track];

if (!strncmp(myTrack.sName, "albert",6))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "barc",4))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "loud",4))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "albrtprk",8))
  maxcurveexpansion = .7;
if (!strncmp(myTrack.sName, "cstlc",5))
  maxcurveexpansion = .7;
if (!strncmp(myTrack.sName, "bran",4))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "trem",4))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "watg",4))
  maxcurveexpansion = .6;
if (!strncmp(myTrack.sName, "zand",4))
  maxcurveexpansion = .7;
if (!strncmp(myTrack.sName, "adelaid",7))
  maxcurveexpansion = .6;
if (!strncmp(myTrack.sName, "spa",3))
  maxcurveexpansion = .3;
if (!strncmp(myTrack.sName, "aalb",4))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "ra",2))
  maxcurveexpansion = .3;
if (!strncmp(myTrack.sName, "esto",4))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "hock",4))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "buenos",6))
  maxcurveexpansion = .6;
if (!strncmp(myTrack.sName, "magny",5))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "monaco",6))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "montre",6))
  maxcurveexpansion = .5;
if (!strncmp(myTrack.sName, "nurb-gp",7))
  maxcurveexpansion = .6;
if (!strncmp(myTrack.sName, "silver97",8))
  maxcurveexpansion = .5;


for (answer = maxcurveexpansion; answer>= 0.05; answer-=.1)
  {
  double ctrx, ctry, radius;

  ctrx = myTrack.trackin[segnum].cen_x;
  ctry = myTrack.trackin[segnum].cen_y;

  radius = curveoptradius(curveradiuslow(segnum), 
myTrack.trackin[segnum].length, myTrack.width*answer);

  getctrxy(segnum, curveradiuslow(segnum), radius, ctrx, ctry);

  if (distfromto(gc_x, gc_y, ctrx, ctry) > fabs(radius))
    return answer*myTrack.width;
  }

return answer*myTrack.width;
}


static double distancetobrake(double vnow, double vthen)
{
if (vnow<vthen)
  return 0.0;  // no need to brake as we are already there!

return .0549 * (vnow+vthen)*(vnow-vthen)/(BRAKECONSTANT*2.0);
}


static double curveMaxRadius(int segnum)
  {
  double maxcurveexpansion =.7;

if (!strncmp(myTrack.sName, "loud",4))
  maxcurveexpansion = .5;
  if (!strncmp(myTrack.sName, "barc",4))
  maxcurveexpansion = .4;
if (!strncmp(myTrack.sName, "albert",6))
    maxcurveexpansion = .4;
  if (!strncmp(myTrack.sName, "albrtprk",8))
    maxcurveexpansion = .7;
  if (!strncmp(myTrack.sName, "cstlc",5))
    maxcurveexpansion = .7;
  if (!strncmp(myTrack.sName, "bran",4))
    maxcurveexpansion = .5;
  if (!strncmp(myTrack.sName, "trem",4))
    maxcurveexpansion = .5;
  if (!strncmp(myTrack.sName, "watg",4))
    maxcurveexpansion = .7;
  if (!strncmp(myTrack.sName, "zand",4))
    maxcurveexpansion = .7;
  if (!strncmp(myTrack.sName, "adelaid",7))
    maxcurveexpansion = .6;
  if (!strncmp(myTrack.sName, "spa",3))
    maxcurveexpansion = .3;
  if (!strncmp(myTrack.sName, "aalb",4))
    maxcurveexpansion = .4;
  if (!strncmp(myTrack.sName, "ra",2))
    maxcurveexpansion = .3;
  if (!strncmp(myTrack.sName, "esto",4))
    maxcurveexpansion = .4;
  if (!strncmp(myTrack.sName, "hock",4))
    maxcurveexpansion = .5;
  if (!strncmp(myTrack.sName, "buenos",6))
    maxcurveexpansion = .6;
  if (!strncmp(myTrack.sName, "magny",5))
    maxcurveexpansion = .4;
  if (!strncmp(myTrack.sName, "montre",6))
    maxcurveexpansion = .5;
  if (!strncmp(myTrack.sName, "nurb-gp",7))
    maxcurveexpansion = .6;

  return curveoptradius(curveradiuslow(segnum), 
myTrack.trackin[segnum].length, myTrack.width*maxcurveexpansion);
  }



static double spdForRadius(double rad)
{
if (rad == 0.0)
  return(400.0);
else
if (rad < 0.0)
  rad = -rad;

return 6.0 * sqrt(rad);
}


static double curvespeed(int segnum)
{
if (curveradiuslow(segnum) == 0.0)
  return spdForRadius(0);
return spdForRadius(curveMaxRadius(segnum));
}


static int curveMakable(int segnum, double ctrx, double ctry, double
radius, situation &s, con_vec &result)
  {
  //  if (s.v + 5 < spdForRadius(radius))
  //    return 0;

  if (distfromto(gc_x, gc_y, ctrx, ctry) > fabs(radius))
    return 1;

  return 0;
  }


static void getCurveStats(int segnum, double &ctrx, double &ctry, double
&rad, situation &s, con_vec &result)
{
double radius = curveradiuslow(segnum);
int nxtseg, prvseg;

nxtseg = (segnum+1)%myTrack.NSEG;
prvseg = segnum-1; if (prvseg == -1) prvseg = myTrack.NSEG;

if (radius == 0)
  return;

if  (radius < 0  &&  
    (curveradiuslow(nxtseg) < 0  ||  curveradiuslow(prvseg) < 0 || 
myTrack.trackin[segnum].length > 160.0/RTD))
  {
  rad = radius-5;
  ctrx = myTrack.trackin[segnum].cen_x;
  ctry = myTrack.trackin[segnum].cen_y;
  return;
  }
if (radius > 0  &&  
   (curveradiuslow(nxtseg) > 0  ||  curveradiuslow(prvseg) > 0  || 
myTrack.trackin[segnum].length > 160.0/RTD))
  {
  rad = radius+5;
  ctrx = myTrack.trackin[segnum].cen_x;
  ctry = myTrack.trackin[segnum].cen_y;
  return;
  }

// is special radius
if (s.seg_ID == ((SCHseg+1) % myTrack.NSEG))
  {
  SCHseg=9999;
  SCHtype = 9999;
  if (outf )
    fprintf(outf, "clearing special lane\n");
  }


if (segnum == SCHseg)
  {
  // is special radius
  if (SCHtype == 0)  // inside curve
    {
    ctrx = myTrack.trackin[segnum].cen_x;
    ctry = myTrack.trackin[segnum].cen_y;
    if (radius > 0)
      rad = radius+.5;
    else
      rad = radius-.5;
    if (outf )
      fprintf(outf, "using inside lane %d\n", segnum);

    getCurveStatsLow(rad, 0, myTrack.trackin[segnum].length/2, 
		     myTrack.trackin[segnum].end_ang, 
myTrack.trackin[segnum].length/2,
		     ctrx, ctry, rad);
    return;
    }
  else
  if (SCHtype == 4)  // slow outside curve
    {
    ctrx = myTrack.trackin[segnum].cen_x;
    ctry = myTrack.trackin[segnum].cen_y;
    if (radius > 0)
      rad = radius+myTrack.width*.4;
    else
      rad = radius-myTrack.width*.4;
    if (outf )
      fprintf(outf, "using slow outside lane %d\n", segnum);

    getCurveStatsLow(rad, 0, myTrack.trackin[segnum].length/2, 
		     myTrack.trackin[segnum].end_ang, 
myTrack.trackin[segnum].length/2,
		     ctrx, ctry, rad);
    return;
    }
  else
    if (SCHtype == 1)  // inside-outside curve
      {
      ctrx = myTrack.trackin[segnum].cen_x;
      ctry = myTrack.trackin[segnum].cen_y;

      if (radius > 0)
	rad = radius+5;
      else
	rad = radius-5;

      getCurveStatsLow(rad, myTrack.width*.5, 
		       myTrack.trackin[segnum].length, 
		       myTrack.trackin[segnum].end_ang,  
		       myTrack.trackin[segnum].length,
		       ctrx, ctry, rad);
      if (outf )
	fprintf(outf, "using inside-outside lane %d\n", segnum);
	if (curveMakable(segnum, ctrx, ctry, rad, s, result))
	  return;
      return;
      }
    else
      if (SCHtype == 3)  // out-inside curve
	{
	ctrx = myTrack.trackin[segnum].cen_x;
	ctry = myTrack.trackin[segnum].cen_y;

	if (radius > 0)
	  rad = radius+5;
	else
	  rad = radius-5;

	getCurveStatsLow(rad, myTrack.width*.5, 
			 myTrack.trackin[segnum].length, 
			 myTrack.trackin[segnum].end_ang,  
			 0.0,
			 ctrx, ctry, rad);
	if (outf )
	  fprintf(outf, "using outside-inside lane %d\n", segnum);
	if (curveMakable(segnum, ctrx, ctry, rad, s, result))
	  return;
	//      if (!curveMakable(segnum, ctrx, ctry, rad, s, result))
	//      setInsideLane(segnum);
	}
      else
	if (SCHtype == 2)  // outside curve
	  {
	  if (radius > 0)
	    radius+= myTrack.width*.4;
	  else
	    radius-= myTrack.width*.4;
	  double ctw = myTrack.width*.3;

	  while (ctw > 0)
	    {
	    ctw-=1.0;
  
	    ctrx = myTrack.trackin[segnum].cen_x;
	    ctry = myTrack.trackin[segnum].cen_y;
	    getCurveStatsLow(radius, ctw, 
			     myTrack.trackin[segnum].length/2.0, 
			     myTrack.trackin[segnum].end_ang,  
			     myTrack.trackin[segnum].length/2.0,
			     ctrx, ctry, rad);
	    if (curveMakable(segnum, ctrx, ctry, rad, s, result))
	      return;
	    if (outf )
	      fprintf(outf, "using outside-outside lane %d\n", segnum);
	    }


	  //    setNormalLane(segnum);
	  }

  }

    if (outf )
      fprintf(outf, "using normal lane %d\n", segnum);
double ctw = curvetrackwidth(segnum);
// rad = curveMaxRadius(segnum);
ctrx = myTrack.trackin[segnum].cen_x;
ctry = myTrack.trackin[segnum].cen_y;
getCurveStatsLow(radius, 
		 ctw,
		 myTrack.trackin[segnum].length/2, 
		 myTrack.trackin[segnum].end_ang,
		 myTrack.trackin[segnum].length/2, 
		 ctrx, ctry, rad);
if (curveMakable(segnum, ctrx, ctry, rad, s, result))
  return;

while (ctw > 0)
  {
  ctw-= 1.0;
  ctrx = myTrack.trackin[segnum].cen_x;
  ctry = myTrack.trackin[segnum].cen_y;

  getCurveStatsLow(radius, 
		   ctw,
		   myTrack.trackin[segnum].length/2, 
		   myTrack.trackin[segnum].end_ang,
		   myTrack.trackin[segnum].length/2, 
		   ctrx, ctry, rad);
  if (curveMakable(segnum, ctrx, ctry, rad, s, result))
    return;
  }

getCurveStatsLow(radius, 
		 0,
		 myTrack.trackin[segnum].length/2, 
		 myTrack.trackin[segnum].end_ang,
		 myTrack.trackin[segnum].length/2, 
		 ctrx, ctry, rad);
return;
//  double trackwidthused = curvetrackwidth(segnum);
//  return curveoptradius(radius, myTrack.trackin[segnum].length,trackwidthused);
}



//  return the distance to a curves edge from its segment
static double curvedist(double ctrx, double ctry, double radius)
{
//  a^2 + b^2 =  c^2
//  where c is the distance to the center of the curve
//  a is the radius of the curve
//  b is the answer I want (distance to the tangent

double answer, disttoctr;

radius= fabs(radius) + 5;  //cushion

disttoctr = distfromto(gc_x, gc_y, ctrx, ctry);

if (disttoctr < radius)
  return 0;

answer = sqrt(disttoctr*disttoctr - radius*radius);

//  when we get sufficiently close I find it nice to just say 0.00

if (answer < 10.0)
  return 0.1;

return answer;
}






// give me the absolute coordinates of a point in a straight
// when passed in the segment, toend and tolft
static void getabscoord_str(int segnum, double tolft, double toend,
double &x, double &y)
{
x =  myTrack.trackin[segnum].end_x -
toend*cos(myTrack.trackin[segnum].beg_ang)
     + tolft*cos(myTrack.trackin[segnum].beg_ang-RTANGLE);

y =  myTrack.trackin[segnum].end_y -
toend*sin(myTrack.trackin[segnum].beg_ang)
     + tolft*sin(myTrack.trackin[segnum].beg_ang-RTANGLE);
}



// find out my absolute coordinates, and from them my absolute direction
// of travel  (may only be called once per bot call)
static void getabsxy(int segnum, double tolft, double torgt, double
toend)
{
// we set gc_x and gc_y for our own usage and gc_lastx and gc_lasty
// so that we can figure out our direction of travel next time around


if (myTrack.trackin[segnum].radius == 0)
  {  // straight
  getabscoord_str(segnum, tolft, toend, gc_x, gc_y);
  }
 else
  {
  double anglefromctr;

  if (myTrack.trackin[segnum].radius > 0)
    {
    anglefromctr = myTrack.trackin[segnum].end_ang - toend - RTANGLE;
    gc_x = myTrack.trackin[segnum].cen_x +
(tolft+fabs(myTrack.trackin[segnum].radius))*cos(anglefromctr);
    gc_y = myTrack.trackin[segnum].cen_y +
(tolft+fabs(myTrack.trackin[segnum].radius))*sin(anglefromctr);
    }
   else
    {
    anglefromctr = myTrack.trackin[segnum].end_ang + toend + RTANGLE;
    gc_x = myTrack.trackin[segnum].cen_x +
(torgt+fabs(myTrack.trackin[segnum].radius)-myTrack.width)*cos(anglefromctr);
    gc_y = myTrack.trackin[segnum].cen_y +
(torgt+fabs(myTrack.trackin[segnum].radius)-myTrack.width)*sin(anglefromctr);
    }
  }

gc_dot = anglefromto(gc_lastx, gc_lasty, gc_x, gc_y);
gc_lastx = gc_x;
gc_lasty = gc_y;
}




//  returns the ideal angle for a curve based upon it segment number
//  only (using the global coordinates gotten from getabsxy)
static void controlForCurve(int segnum, situation &s, con_vec &result)
{
double angletoctr, disttowall, answer;
double cctrx, cctry;   // curve center x,y
double cradius;        // curve radius
int segidx;

result.vc = curvespeed(s.seg_ID);

getCurveStats(segnum, cctrx, cctry, cradius, s, result);

disttowall = distfromto(gc_x, gc_y, cctrx, cctry) - fabs(cradius);

angletoctr = anglefromto(gc_x, gc_y, cctrx, cctry);

answer = maxangleinturn(cradius, disttowall, 5);  // 5 is cushion

if (myTrack.trackin[segnum].radius > 0)
  answer = angletoctr-RTANGLE+answer;
 else
  answer = angletoctr+RTANGLE+answer;

normalize(gc_dot, answer);

result.alpha = answer;

if (curvedist(cctrx, cctry, cradius) < distancetobrake(s.v,
spdForRadius(cradius)))
  result.vc = 0;

segidx = (segnum+1) % myTrack.NSEG;
while (curveradiuslow(segidx) == 0)
  segidx = (segidx+1) % myTrack.NSEG;
getCurveStats(segidx, cctrx, cctry, cradius, s, result);
if (curvedist(cctrx, cctry, cradius) < 1.3*distancetobrake(s.v,
spdForRadius(cradius)))
  result.vc = 0;
}






static void getSpeedAndAngle(situation &s, con_vec &result, int segnum,
int nxtseg, int nxtnxtseg)
{
double ideal_alpha;

if (s.cur_rad == 0.0)
  {
  controlForCurve(nxtseg, s, result);
  }
else
  {
  // look for either an s curve or a situation where we have
  // an  s curve with a straight in the middle
  controlForCurve(segnum, s, result);
  ideal_alpha = result.alpha;

  if (curveradiuslow(nxtseg) != 0)
    {  // one curve following another.
    if (myTrack.trackin[nxtseg].length > 3.14)  
      controlForCurve(segnum, s, result);
    else
      //  now we look to see if we have an s-curve situation
      if (curveradiuslow(segnum) * curveradiuslow(nxtseg)  < 0)
	{
	controlForCurve(nxtseg, s, result);

	double nextcurvealpha= result.alpha;

	// can we see next curve
	if (
	    (curveradiuslow(segnum) > 0 &&  nextcurvealpha <
ideal_alpha)
	    ||
	    (curveradiuslow(segnum) < 0 &&  nextcurvealpha >
ideal_alpha)
	    )
	  {
	  controlForCurve(nxtseg, s, result);
	  }
	else
	  {
	  controlForCurve(segnum, s, result);
	  }
	}
    }
  else
    {
    // not an s curve, check for opposite direction curves with
    // an intervening straight

    if (curveradiuslow(segnum) * curveradiuslow(nxtnxtseg)  < 0)
      {
      controlForCurve(nxtnxtseg, s, result);

      double nextcurvealpha= result.alpha;

      // can we see next curve
      if (
	  (curveradiuslow(segnum) > 0 &&  nextcurvealpha < ideal_alpha)
	  ||
	  (curveradiuslow(segnum) < 0 &&  nextcurvealpha > ideal_alpha)
	  )
	{
	controlForCurve(nxtnxtseg, s, result);
	}
      else
	{
	controlForCurve(segnum, s, result);
	}
      }
    else
      {
      controlForCurve(nxtnxtseg, s, result);

      double nextcurvealpha= result.alpha;

      // can we see next curve
      if (
	  (curveradiuslow(segnum) > 0 &&  nextcurvealpha < ideal_alpha)
	  ||
	  (curveradiuslow(segnum) < 0 &&  nextcurvealpha > ideal_alpha)
	  )
	{
	controlForCurve(nxtnxtseg, s, result);
	}
      else
	{
	controlForCurve(segnum, s, result);
	}
      }
    }
  }
}




static double linking_magic_number = 21.0;

static void link_alpha_and_vc(situation &s, con_vec &result)
{
if (fabs(result.alpha) == 2)
  return;

if (s.v < result.vc)
  {  // accelerating
  double linknum = linking_magic_number/fabs(result.alpha);

  if (linknum < result.vc)
    result.vc = s.v + 0;  //  linknum;
  }
 else
  {  // braking
  double linknum = linking_magic_number/((result.vc + s.v)/2.0);
  if (result.alpha > 0  &&  result.alpha > linknum)
    result.alpha = linknum;
  if (result.alpha < 0  &&  result.alpha < -linknum)
    result.alpha = -linknum;
  }

}



// amount can be on of these...
//  (3) only worry about rear end collisions
//  (2) only worry about rear end collisions, already overlapping
//  (1) only worry about rear end collisions, already overlapping, collisions
//      with our nose in front of his
//  (0) only worry about all collisions


static void avoidcars2(situation &s, con_vec &result)
{
// int pos = 0;
// 1 = side, 2 = behind, 3 = diagonal, 4 = overlapping

for (int j=0; j<2; j++)
  if (s.nearby[j].who < 16)
    {
    double t=0.0, fx, fy, t2;
    double tilt;
    double y,b,x,m,cd;
    double carwid = 10, carlen = 20;


    if (0 && s.nearby[j].rel_ydot )
      {
      tilt =
fabs(atan(s.nearby[j].rel_xdot/(s.nearby[j].rel_ydot+s.v/4.0)));
      carwid = carwid + (carlen-carwid)*(sin(tilt)) * 1.50;  // 2.0 is agression factor
      }

    y =  s.nearby[j].rel_y;
    x =  s.nearby[j].rel_x;
    double dx = s.nearby[j].rel_xdot;
    double dy = s.nearby[j].rel_ydot;

    if ((dx*dx) + (dy*dy) < 20)
      continue;

    //    if (fabs(x) > 9  &&  (dx*dx) + (dy*dy) < 200)
    //      continue;
    //    if (sqrt(x*x) + sqrt(y*y) < 60)   
    //      continue;
    m = s.nearby[j].rel_ydot/s.nearby[j].rel_xdot;

    b = y + m * (-x);

    // this x is the x of closest impact;
    fx = (-m*b)/(m*m + 1);

    t2= (fx-x)/s.nearby[j].rel_xdot;
    fy = y+t*s.nearby[j].rel_ydot;

    // closest distance is
    cd = sqrt(fx*fx + fy*fy);

    //    if (x * result.alpha > 0  &&  fabs(x) > 10  && fabs(result.alpha) > .1)
    //continue;  // im going out of the way anyhow

    if (y < 30 && y > 0  &&  fabs(x) < 9)
      {
      t=0;
      fx = fy = 2;
      }
    else
    if (y < 30 && y > 0)
      {
      if (x * s.nearby[j].rel_xdot < 0)
	{
	t = (fabs(x)-carwid)/fabs(s.nearby[j].rel_xdot);
	fy = y+ t * s.nearby[j].rel_ydot;
	fx = x+ t * s.nearby[j].rel_xdot;
	}
      else
	{
	t = 0;
	fy = y;
	fx = x;
	}
      }
    else
    if (fabs(x) < carwid)
      {
      if (y * s.nearby[j].rel_ydot < 0)
	{
	t = (fabs(y)-30)/fabs(s.nearby[j].rel_ydot);
	fy = y+ t * s.nearby[j].rel_ydot;
	fx = x+ t * s.nearby[j].rel_xdot;
	}
      else
	{
	t = 0;
	fy = y;
	fx = x;
	}
      }
    else
      {
      double t1=999, t2 = 999;

      if (x * s.nearby[j].rel_xdot < 0)
	t1 = (fabs(x)-carwid)/fabs(s.nearby[j].rel_xdot);

      if (y * s.nearby[j].rel_ydot < 0)
	t2 = (fabs(y)-30)/fabs(s.nearby[j].rel_ydot);

      t = t1; if (t2 > t1) t = t2;

      if (t > 400) t = 0;

      fy = y+ t * s.nearby[j].rel_ydot;
      fx = x+ t * s.nearby[j].rel_xdot;
      }


    if (outf  &&  y<80 &&   t>-10)
      {
      fprintf(outf, "damage id %5ld %2d x%4.1f y%4.1f dx%4.1f dy%4.1f",
	      s.damage, s.nearby[j].who, x, y,
s.nearby[j].rel_xdot,s.nearby[j].rel_ydot
	      );
      fprintf(outf, " my alg, fx%4.2f fy%4.2f t%4.1f ", fx, fy, t);
      fprintf(outf, " cd%4.2f t2%4.2f\n", cd, t2);
      }


    if (fabs(fx) <= carwid+1.0  &&  fy>=-1.0    &&  fy<=carlen+1.0)
      {
      if (t < 5)
	{
	if (outf )
	  fprintf(outf, "avoidcar:incoming vc, alpha %4.2f   %4.2f\n",
result.vc, result.alpha);

	if (x * result.alpha > 0  &&  fabs(x) > carwid  && 
fabs(result.alpha) > .2 &&  y < 60)
	  continue;  // im going out of the way anyhow

	//        are we relatively straight  (our alpha < .05)
	//          action: don't affect our v try to move to outside
	if (fabs(result.alpha) < .01)
	  {
	  if (outf)
	    {
		fprintf(outf, "avoidcar:i'm going straight nexrad is %4.2f\n", s.nex_rad);
		}

	  if (y < 60  &&  s.cur_rad != 0.0  &&  SCHtype == 0)  
	    setInsideLane(s.seg_ID);
	  result.vc = 0;
	  }
	else
	if (y < 30)
	  {  // he is to side of us
	  //        are we turning but to the side of him (alpha > .05 && y < 30)
	  //          are we turning into him (alpha == x signwise)
	  //            action: half the alpha, vc = v-1
	  if (result.alpha * x < 0)
	    {
	    if (outf )
	      fprintf(outf, "avoidcar:turning into him, to side\n");
	    result.vc = 0;  //  min(result.vc, s.v-5);
	    if (y < 60  &&  s.cur_rad != 0.0  &&  SCHtype == 0)  
	      setInsideLane(s.seg_ID);
	    //            result.alpha *= .5;
	    if (x < 0)
	      result.alpha +=.3;
	    else
	      result.alpha -=.3;
	    }
	  else
	    {
	    //          are we turning away from him (alpha != x signwise)
	    //            is there room to increase our turn rate (enough to side or vn is
	    //        away from wall
	    //              action: double the alpha, vc = v - 1
	    //             else
	    //              action: alpha = 0, vc =0
	    if (x < 0  &&     // to left
		( s.to_lft >5 ||  s.vn >0)
		)
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, to side have room\n");
	      result.alpha-=.3;
	      result.vc = min(result.vc, s.v-5);
	      }
	    else
	    if (x > 0  &&     // to right
		( s.to_rgt >5 ||  s.vn <0)
		)
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, to side have room\n");
	      result.alpha+=.3;
	      result.vc = min(result.vc, s.v-5);
	      }
	    else
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, to side no room\n");
	      result.vc = min(result.vc, s.v-5);
	      if (y < 60  &&  s.cur_rad != 0.0  &&  SCHtype == 0)  
		setInsideLane(s.seg_ID);
	      }
	    }
	  }
	else
	  {  // he is greater than a car length
	  if (result.alpha * x < 0)
	    {
	    if (outf )
	      fprintf(outf, "avoidcar:turning into him, in front\n");

	    /*
	      if (x < 0)
	      result.alpha +=.2;
	      else
	      result.alpha -=.2;
	      */

	    result.vc = 0;  // min(result.vc, s.v-5);
	    result.alpha *= .5;  // a
	    }
	  else
	    {
	    //          are we turning away from him (alpha != x signwise)
	    //            is there room to increase our turn rate (enough to side or vn is
	    //        away from wall
	    //              action: double the alpha, vc = v - 1
	    //             else
	    //              action: alpha = 0, vc =0
	    if (x < 0  &&     // to left
		( s.to_lft >5 ||  s.vn >0)
		)
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, in front have room\n");
	      result.alpha-=.3;
	      result.vc = min(result.vc, s.v-5);  //a

	      }
	    else
	    if (x > 0  &&     // to right
		( s.to_rgt >5 ||  s.vn <0)
		)
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, in front have room\n");
	      result.alpha+=.3;
	      result.vc = min(result.vc, s.v-5);  //a

	      }
	    else
	      {
	      if (outf )
		fprintf(outf, "avoidcar:turning away from him, in front no room\n");
	      result.alpha = 0.0;
	      result.vc = min(result.vc, s.v-5);
	      }
	    }
	  }
	}

      if (outf )
	fprintf(outf, "avoidcar:outgoing vc, alpha %4.2f   %4.2f\n",
result.vc, result.alpha);
      }
    else
      {
      double tx = fabs(x-9);
      if (tx < 9) tx = 0;
      //      tx*=2.0;
      
      double ty = fabs(y-30);
      if (ty < 30) ty = 0;

      if (dy < 0 &&  (dx*dx +dy*dy) > 45* sqrt(tx*tx + ty*ty))
	{
	result.vc=min(result.vc, s.v-5);  //  s.v+y/10.0;
      
	if (outf)
	  fprintf(outf, "not going to hit, but slowing down ogp\n");
	}
      else
      if (outf)
	  fprintf(outf, "no action\n");


      }
    }

}



static int eachLap(situation &s, con_vec &result)
{
  int i, tmp = 0;
  
  for (i=0; i< car_count; i++)
    if (race_data.cars[i]  &&  race_data.cars[i]->Laps > tmp)
      tmp = race_data.cars[i]->Laps;

  if (s.laps_to_go > numLapsInRace)
    numLapsInRace = s.laps_to_go;

  myLtg = numLapsInRace - tmp;

  if (lastSeg != s.seg_ID  &&  s.seg_ID == 0){
    lastSeg = s.seg_ID;

    if (numLapsInRace - s.laps_to_go > 2)
      if (s.damage>lastDamage  &&  s.damage-lastDamage > maxDamagePerLap)
	maxDamagePerLap = s.damage-lastDamage;
    
    lastDamage = s.damage;

    if (outf){
      fprintf(outf, "s.damage %4.2ld\n", s.damage);
      fprintf(outf, "max damage per lap %ld\n", maxDamagePerLap);
      fprintf(outf, "laps in race %d laps to go %d\n", numLapsInRace,
	      s.laps_to_go);
      fprintf(outf, "car in front %d last car in front %d\n",
	      s.nearby[0].who, lastCarInFront);
      fprintf(outf, "dist to car in front %4.2f\n", s.nearby[0].rel_y);
    }

    //  if we are within max damage taken of 30000
    if (s.damage + maxDamagePerLap > 30000){
      repair_amount = (long) (s.damage/(numLapsInRace - myLtg)) * myLtg;
      if (outf) 
	fprintf(outf, "f1 %ld\n", (s.damage/(numLapsInRace - s.laps_to_go)));
      if (outf) 
	fprintf(outf, "f2 %ld\n", s.damage + maxDamagePerLap);

      if (repair_amount < 3000)
	repair_amount = 3000;
      result.request_pit = 1;
      maxDamagePerLap /= 2;

      lastCarInFront = -1;
      if (outf) 
	fprintf(outf, "danger of dying pitting with repair amount %d\n", 
		      repair_amount);

      return 1;
    }
    
    if (s.nearby[0].who < 16  &&  s.nearby[0].rel_y < 100){
      if (lastCarInFront == s.nearby[0].who){
	if (ignoreDamage == 1){
	  ignoreDamage = 0;

	  if (s.damage > 10000)
	    repair_amount = 2000;
	  else
	    repair_amount = 1000;
	  result.request_pit = 1;
	  lastCarInFront = -1;
	  if (outf) 
	    fprintf(outf, "passing maneuver? pitting with repair amount %d\n", 
			  repair_amount);
	  return 1;
	}
	else { //    try to set ignore damage flag
	  ignoreDamage = 1;
	}
      }
      else{
	ignoreDamage = 0;
	lastCarInFront = s.nearby[0].who;
      }
    }
    else{
      lastCarInFront = -1;
      ignoreDamage = 0;
    }
  }
  lastSeg = s.seg_ID;
  return 0;
}



// returns true if segidx and the next curve are close and in opposite
//directions
static int sCurve(int segidx)
{
int nxtseg = (segidx+1) % myTrack.NSEG;
double midlen=0;

if (curveradiuslow(nxtseg) == 0)
  midlen = myTrack.trackin[nxtseg].length;

while (curveradiuslow(nxtseg) == 0)
  nxtseg = (nxtseg+1) % myTrack.NSEG;

if (SIGN(curveradiuslow(nxtseg)) == SIGN(curveradiuslow(segidx)))
  return 0;

if (midlen > 500)
  return 0;

return 1;
}


static int getPosition(double radius, situation &s)
{
int position=0;

if (radius < 0)  //  right hand curve
  {
  if (s.nearby[0].rel_y < 30)
    {
    if (s.nearby[0].rel_x < 0)
      position = 1;
    else
      position = 2;
    }
  else
  if (fabs(s.nearby[0].rel_xdot-s.vn) > 6)
    {
    if (s.nearby[0].rel_xdot-s.vn > 0)  // other car moving to right
      position = 2;
    else
      position = 1;
    }
  else
    if (s.to_lft+s.nearby[0].rel_x > myTrack.width/2)  // to right?
      position = 2;
    else
      position = 1;
  }
else
  {
  if (s.nearby[0].rel_y < 30)
    {
    if (s.nearby[0].rel_x > 0)
      position = 1;
    else
      position = 2;
    }
  else
  if (fabs(s.nearby[0].rel_xdot-s.vn) > 6)
    {
    if (s.nearby[0].rel_xdot-s.vn > 0)  // other car moving to right
      position = 1; 
    else
      position = 2;
    }
  else
    if (s.to_lft+s.nearby[0].rel_x > myTrack.width/2)  // to right?
      position = 1;
    else
      position = 2;
  }

return position;
}



static void drivingLogic(situation &s, con_vec &result)
  {
  //if (fabs(last_alpha) > .1)
  //  return;
  if (s.vc == 0.0)
    return;
  if (s.cur_rad != 0.0)
    return;

  if ((s.nearby[0].who < 16 &&  s.nearby[0].rel_y > 200) || 
      s.nearby[0].who >= 16 )
    {
    if(outf)
      fprintf(outf, "setting normal lane %d   %4.2f\n", 
	      s.nearby[0].who,  s.nearby[0].rel_y);
    setNormalLane(s.seg_ID);
    return;
    }

  if (s.nearby[0].rel_y + 90 > s.to_end)
    return;

  if (outf)
    {
    fprintf(outf, "vn  rel: x y dx dy %4.2f %4.2f %4.2f %4.2f %4.2f\n",
	    s.vn, s.nearby[0].rel_x, s.nearby[0].rel_y,
	    s.nearby[0].rel_xdot, s.nearby[0].rel_ydot);
    }

  int position = 1;  // 1 = inside, 2 = outside
  int scurve = 0;  // 0 = no, 1 = yes
  int roomForInside = 0;
  int passShouldSucceed = 0;
  // double laneCrossRation = 0.0;
  int hairpinCurve = 0;

  int nxtseg = (s.seg_ID +1 ) % myTrack.NSEG;
  int nxtnxtseg = (nxtseg +1 ) % myTrack.NSEG;

  if (s.to_end/s.v < (s.to_end -
s.nearby[0].rel_y)/(s.v+s.nearby[0].rel_ydot))
    passShouldSucceed = 1;

  while (myTrack.trackin[nxtnxtseg].radius == 0.0)
    nxtnxtseg = (nxtnxtseg +1 ) % myTrack.NSEG;

  position = getPosition(myTrack.trackin[nxtseg].radius, s);

  if (myTrack.trackin[nxtseg].length > 140/RTD)
    hairpinCurve = 1;

  scurve = sCurve(nxtseg);

  if (
      (myTrack.trackin[nxtseg].radius < 0  &&  s.to_rgt -
s.nearby[0].rel_x > 11)
      ||
      (myTrack.trackin[nxtseg].radius > 0  &&  s.to_lft +
s.nearby[0].rel_x > 11)
      )
    roomForInside = 1;

  //if (s.nearby[0].rel_y > 50)
  //  position *= -1;


  int swingWide = -1;  // don't know

  if (s.to_end - s.nearby[0].rel_y<10  && s.to_end - s.nearby[0].rel_y >
0 )
    {
    int segnum = nxtseg;
    double ctrx = 0, ctry=0, rad = curveradiuslow(segnum);

    getCurveStatsLow(rad, myTrack.width*.3, 
		     myTrack.trackin[segnum].length, 
		     myTrack.trackin[segnum].end_ang,  
		     myTrack.trackin[segnum].length,
		     ctrx, ctry, rad);
    double critspd = spdForRadius(rad);
  
    if (s.v + s.nearby[0].rel_y < critspd)
      swingWide = 0;
     else
      swingWide = 1;
    }

  if (outf)
    {
	fprintf(outf, "position, scurve, roomforinside swingwide pass should succeed %d %d %d %d %d\n",
		position, scurve, roomForInside, swingWide, passShouldSucceed);
    }
  

  if (s.nearby[1].who < 16  &&  s.nearby[1].rel_y < 200)
    {
    if(!scurve)
      {
      setOutsideLane(nxtseg );
      return;
      }
  
     else
      {
      if (s.laps_to_go %2 == 0)
	setSlowOutsideLane(nxtseg );
       else
	setInsideLane(nxtseg );
      return;
      }
    }

  if ( s.nearby[0].rel_ydot < 3  &&
	(passShouldSucceed  ||  s.nearby[0].rel_y < 90))
    {
    if (scurve)
      {
      if (roomForInside  &&  passShouldSucceed)
	setIn2OutLane(nxtseg);
       else
      if (position == 1)
	{ // inside
	setIn2OutLane(nxtseg );
	if (swingWide == 1)
	  setInsideLane(nxtseg );
	}
       else
      if (position == 2)
	{  // outside
	if (s.nearby[0].rel_y < 30)
	  setOut2InLane(nxtseg );
	 else
	if (s.laps_to_go %2 == 0)
	  setOutsideLane(nxtseg );
	 else
	  setOut2InLane(nxtseg );
	}
       else
	{  //behind
	if (position == -1)
	  setIn2OutLane(nxtseg );
	 else
	  setOut2InLane(nxtseg );
	}
      }
     else
      {
      if (position == 1)
	{ // inside
	setIn2OutLane(nxtseg );
	if (swingWide == 1)
	  setInsideLane(nxtseg );
	}
       else
      if (position == 2)
	{  // outside
	if (hairpinCurve)
	  setOut2InLane(nxtseg);
	 else
	if (swingWide == 0)  // stay inside
	  setOutsideLane(nxtseg );
	 else
	  {
	  //          if (fabs(curveradiuslow(nxtseg)) > 250)
	  //            setOut2InLane(nxtseg ); 
	  //           else
	  if (s.nearby[0].rel_y > 30)
	    {
	    setOutsideLane(nxtseg ); 
	    if (s.laps_to_go %2 == 0)
	      setOut2InLane(nxtseg);
	    }
	   else
	    setSlowOutsideLane(nxtseg ); 
	  }
	}
       else
	{  //behind
	if (position == -1)
	  setIn2OutLane(nxtseg );
	 else
	if (swingWide == 1)
	  {
	  if (fabs(curveradiuslow(nxtseg)) > 250)
	    setOut2InLane(nxtseg ); 
	   else
	    setOutsideLane(nxtseg ); 
	  }
	}
      }
    }
  }



con_vec Ralph2a(situation &s)
{
static int firstcall = 1;
con_vec result;                    // This is what is returned.
// double ideal_alpha;
int segnum, nxtseg, nxtnxtseg;


if (firstcall)
  {                               //  this is the very first call
  my_name_is("Ralph2a");          //  this lets everyone know who
  //  we are.

  firstcall = 0;                  //  theres only one first call

  return result;                  //  must return an answer
  }

if (s.starting)
  {
  outf = fopen("tutor.out", "a");
	outf = 0;
  myTrack = get_track_description();

  lastDamage = s.damage;
  lastSeg = s.seg_ID;
  maxDamageTaken = 0;
  maxDamagePerLap = 10000;
  ignoreDamage = 0;
  lastCarInFront = -1;
  repair_amount = 0;
  result.fuel_amount = MAX_FUEL;
  }

result.request_pit = 0;
result.repair_amount = 0;
if (eachLap(s, result))  // stuff that is performed once per lap
  {
  result.repair_amount = repair_amount;

  if (outf)
    {
    fprintf(outf, "each lap result is pit amt %d\n",
result.repair_amount);
    }

  return result;
  }

result.request_pit = 0;

if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt,
&result.alpha,&result.vc))
  return result;

getabsxy(s.seg_ID, s.to_lft, s.to_rgt, s.to_end);


segnum = s.seg_ID;
nxtseg = (segnum+1)%myTrack.NSEG;
nxtnxtseg = (nxtseg+1)%myTrack.NSEG;

if (outf)
  fprintf(outf, "main:seg_id %d\n", segnum);

getSpeedAndAngle(s, result, segnum, nxtseg, nxtnxtseg);

if (!ignoreDamage)
  drivingLogic(s, result);

/*
  if (outf)
  {
  fprintf(outf, "main: gc_dot %4.2f\n", gc_dot);
  fprintf(outf, "main: result vc %4.2f  alpha %4.2f\n", result.vc,
result.alpha);
  }
  */

result.alpha = result.alpha-gc_dot;

link_alpha_and_vc(s, result);

if (!ignoreDamage  &&  s.damage + maxDamagePerLap * myLtg > 30000)
  avoidcars2(s, result);

result.repair_amount = repair_amount;
if (s.fuel < 10) {
  result.request_pit = 1;
  result.fuel_amount = MAX_FUEL;
  }  
last_alpha = result.alpha;
last_vc = result.vc;

return result;
}
