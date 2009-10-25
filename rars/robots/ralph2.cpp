/*
    rscott@alpha.netusa.net
    This car is intended for the March 16th races.
    It may be published freely.
    It has been modified to survive the tracks of the March 16th races.
*/


#include <stdio.h>
#include <string.h>

#include <math.h>

#include "car.h"
#include "track.h"

#define distfromto(a,b,c,d)  sqrt((a-c)*(a-c) + (b-d)*(b-d))

//#define   PI   3.14159265
#define   RTD   (180.0/PI)
#define   RTANGLE    (90.0/RTD)
#define   OMCOS(x)       (1.0-cos(x))


static track_desc track;

static double gc_dot, gc_x, gc_y, gc_lastx, gc_lasty;  // what was our position during last call


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
static double anglefromto(double x, double y, double destx, double desty)
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



// given a radius, distance from wall and cushion from wall return optimal
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

if (track.trackin[segnum].radius == 0)
  return 0;

if (track.trackin[segnum].radius > 0)
  return track.trackin[segnum].radius;
 else
  return track.trackin[segnum].radius + track.width;
}



// get center of opt radius
// passed in the segnum and the new radius and the original radius
static void getctrxy(int segidx, double oldradius, double newradius,
double &ctrx, double &ctry)
{
double turnamt,
       radiusdif=fabs(newradius-oldradius);

turnamt = (PI-track.trackin[segidx].length)/2;
if (track.trackin[segidx].radius > 0)
  turnamt = track.trackin[segidx].end_ang + turnamt;
 else
  turnamt = track.trackin[segidx].end_ang - turnamt;

ctrx +=  cos(turnamt)*(radiusdif);
ctry +=  sin(turnamt)*(radiusdif);
}




static double curvetrackwidth(int segnum)
{
double answer;
double maxcurveexpansion =.7;

if (!strncmp(track.sName, "cstlc",5))
  maxcurveexpansion = .2;
if (!strncmp(track.sName, "bran",4))
  maxcurveexpansion = .5;
if (!strncmp(track.sName, "trem",4))
  maxcurveexpansion = .5;
if (!strncmp(track.sName, "watg",4))
  maxcurveexpansion = .7;
if (!strncmp(track.sName, "zand",4))
  maxcurveexpansion = .7;
if (!strncmp(track.sName, "adelaid",7))
  maxcurveexpansion = .6;
if (!strncmp(track.sName, "spa",3))
  maxcurveexpansion = .3;
if (!strncmp(track.sName, "aalb",4))
  maxcurveexpansion = .4;
if (!strncmp(track.sName, "ra",2))
  maxcurveexpansion = .3;
if (!strncmp(track.sName, "esto",4))
  maxcurveexpansion = .4;
if (!strncmp(track.sName, "hock",4))
  maxcurveexpansion = .5;
if (!strncmp(track.sName, "buenos",6))
  maxcurveexpansion = .6;
if (!strncmp(track.sName, "magny",5))
  maxcurveexpansion = .4;
if (!strncmp(track.sName, "montre",6))
  maxcurveexpansion = .5;

for (answer = maxcurveexpansion; answer>= 0.05; answer-=.1)
  {
  double ctrx, ctry, radius;

  ctrx = track.trackin[segnum].cen_x;
  ctry = track.trackin[segnum].cen_y;

  radius = curveoptradius(curveradiuslow(segnum),
track.trackin[segnum].length, track.width*answer);

  getctrxy(segnum, curveradiuslow(segnum), radius, ctrx, ctry);

  if (distfromto(gc_x, gc_y, ctrx, ctry) > fabs(radius))
    return answer*track.width;
  }

return answer*track.width;
}



static double curveradiushigh(int segnum)
{
double radius = curveradiuslow(segnum);
double trackwidthused = curvetrackwidth(segnum);
int nxtseg, prvseg;

nxtseg = (segnum+1)%track.NSEG;
prvseg = (segnum-1+track.NSEG)%track.NSEG; 

if (radius == 0)
  return 0;

if (radius < 0  &&  (curveradiuslow(nxtseg) < 0  ||
curveradiuslow(prvseg) < 0 ))
  return radius;
if (radius > 0  &&  (curveradiuslow(nxtseg) > 0  ||
curveradiuslow(prvseg) > 0 ))
  return radius;


return curveoptradius(radius, track.trackin[segnum].length,
trackwidthused);

}



static double distancetobrake(double vnow, double vthen)
{
if (vnow<vthen)
  return 0.0;  // no need to brake as we are already there!

return .0549 * (vnow+vthen)*(vnow-vthen)/(1.7*2.0);
}


static double curvespeed(int segnum)
{
double radius = curveradiushigh(segnum);

if (curveradiuslow(segnum) == 0.0)
  return(400.0);
 else
if (radius < 0.0)
  radius = -radius;

return 6.0 * sqrt(radius);
}




//  return to use the actual center coordinated of a segment
static void curvecenter(int segnum, double &ctrx, double &ctry)
{
ctrx = track.trackin[segnum].cen_x;
ctry = track.trackin[segnum].cen_y;

getctrxy(segnum, curveradiuslow(segnum),
                 curveradiushigh(segnum),
                 ctrx, ctry);
}





//  return the distance to a curves edge from its segment
static double curvedist(int segnum)
{
//  a^2 + b^2 =  c^2
//  where c is the distance to the center of the curve
//  a is the radius of the curve
//  b is the answer I want (distance to the tangent

double answer, radius, disttoctr;
double ctrx, ctry;


curvecenter(segnum, ctrx, ctry);

disttoctr = distfromto(gc_x, gc_y, ctrx, ctry);
radius = fabs(curveradiushigh(segnum)) + 5  ;  // cushion

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
static void getabscoord_str(int segnum, double tolft, double toend, double &x, double &y)
{
x =  track.trackin[segnum].end_x -
toend*cos(track.trackin[segnum].beg_ang)
     + tolft*cos(track.trackin[segnum].beg_ang-RTANGLE);

y =  track.trackin[segnum].end_y -
toend*sin(track.trackin[segnum].beg_ang)
     + tolft*sin(track.trackin[segnum].beg_ang-RTANGLE);
}



// find out my absolute coordinates, and from them my absolute direction
// of travel  (may only be called once per bot call)
static void getabsxy(int segnum, double tolft, double torgt, double toend)
{
// we set gc_x and gc_y for our own usage and gc_lastx and gc_lasty
// so that we can figure out our direction of travel next time around


if (track.trackin[segnum].radius == 0)
  {  // straight
  getabscoord_str(segnum, tolft, toend, gc_x, gc_y);
  }
 else
  {
  double anglefromctr;

  if (track.trackin[segnum].radius > 0)
    {
    anglefromctr = track.trackin[segnum].end_ang - toend - RTANGLE;
    gc_x = track.trackin[segnum].cen_x +
(tolft+fabs(track.trackin[segnum].radius))*cos(anglefromctr);
    gc_y = track.trackin[segnum].cen_y +
(tolft+fabs(track.trackin[segnum].radius))*sin(anglefromctr);
    }
   else
    {
    anglefromctr = track.trackin[segnum].end_ang + toend + RTANGLE;
    gc_x = track.trackin[segnum].cen_x +
(torgt+fabs(track.trackin[segnum].radius)-track.width)*cos(anglefromctr);
    gc_y = track.trackin[segnum].cen_y +
(torgt+fabs(track.trackin[segnum].radius)-track.width)*sin(anglefromctr);
    }
  }

gc_dot = anglefromto(gc_lastx, gc_lasty, gc_x, gc_y);
gc_lastx = gc_x;
gc_lasty = gc_y;
}




//  returns the ideal angle for a curve based upon it segment number
//  only (using the global coordinates gotten from getabsxy)
static double angleforcurve(int segnum, double curspd)
{
double angletoctr, disttowall, answer;
double cctrx, cctry;   // curve center x,y
double cradius;        // curve radius


curvecenter(segnum, cctrx, cctry);
cradius = curveradiushigh(segnum);

disttowall = distfromto(gc_x, gc_y, cctrx, cctry) - fabs(cradius);

angletoctr = anglefromto(gc_x, gc_y, cctrx, cctry);

answer = maxangleinturn(cradius, disttowall, 5);  // 5 is cushion

if (track.trackin[segnum].radius > 0)
  answer = angletoctr-RTANGLE+answer;
 else
  answer = angletoctr+RTANGLE+answer;

normalize(gc_dot, answer);

return answer;
}


//  returns the angle to a given point in a straight (specified by
//  its to left and toend numbers and the segment number)
//  (using the global coordinates gotten from getabsxy)
/* Not used
static double angleforstraight(double tolft, double toend, int segnum)
{
double destx, desty, alpha;

getabscoord_str(segnum, tolft, toend, destx, desty);
alpha = anglefromto( gc_x, gc_y, destx, desty);
normalize(gc_dot, alpha);

return alpha;
}
*/

//  This routine looks into the distance to see if we need to
//  brake.
static void getspeed(situation &s, con_vec &result)
{

result.vc =curvespeed(s.seg_ID);

for (int i=1; i<4; i++)
  {
  int segiq = (s.seg_ID+i)%track.NSEG;  // SEGment In Question

  if (track.trackin[segiq].radius != 0 && curvedist(segiq) <
distancetobrake(s.v, curvespeed(segiq)))
    result.vc =0;
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



#define  MYCARWID  10
#define  MYCARLEN  20

static void avoidcars2(situation &s, con_vec &result)
{
for (int j=0; j<2; j++)
  if (s.nearby[j].who < 16)
	 {
	 double t, fx, fy, t2;
	 double y,b,x,m,cd;
	 double carwid = MYCARWID;

	 if ( s.nearby[j].rel_ydot != 0.0 )
		{
		carwid = atan(s.nearby[j].rel_xdot/s.nearby[j].rel_ydot);
		carwid = MYCARWID + (MYCARLEN-MYCARWID)*(1-sin(carwid)) * .45;
	}			// .45 is agression factor


	 y =  s.nearby[j].rel_y;
	 x =  s.nearby[j].rel_x;

	 if ( s.nearby[j].rel_xdot != 0.0 )
		m = s.nearby[j].rel_ydot/s.nearby[j].rel_xdot;
	  else
		m = 1000;

	 b = y + m * (-x);

	 // this x is the x of closest impact;
    fx = (-m*b)/(m*m + 1);

    if ( s.nearby[j].rel_xdot != 0.0 )
		t2= (fx-x)/s.nearby[j].rel_xdot;
     else
		t2= 1000;


    fy = y+t2*s.nearby[j].rel_ydot;

    // closest distance is
	 cd = sqrt(fx*fx + fy*fy);

	 if (y < MYCARLEN && y > 0  &&  fabs(x) < MYCARWID)
      {
		t=0;
      fx = fy = 2;
		}
     else
	 if (y < MYCARLEN && y > 0)
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
		  t = (fabs(y)-MYCARLEN)/fabs(s.nearby[j].rel_ydot);
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
		  t2 = (fabs(y)-MYCARLEN)/fabs(s.nearby[j].rel_ydot);

		t = t1; if (t2 > t1) t = t2;

		if (t > 400) t = 0;

		fy = y+ t * s.nearby[j].rel_ydot;
		fx = x+ t * s.nearby[j].rel_xdot;
		}


	 if (fabs(fx) <= carwid && fy>-8)
		{
		if (t < 5)
		  {
		  if ( result.vc > s.nearby[j].rel_ydot+s.v)
			 {
			 if (y < MYCARLEN)
				result.vc = 0;
			  else
				result.vc = s.nearby[j].rel_ydot+s.v;
			 }
		  }
		 else
		  {
		  double adjamt = 4/s.nearby[j].rel_y;

		  if (s.nearby[j].rel_xdot < 0)
			 {
			 if (s.to_lft > s.to_rgt/2)
				result.alpha+= adjamt;
			  else
				result.alpha-= adjamt;
			 }
			else
			 {
			 if (s.to_rgt > s.to_lft/2)
				result.alpha-= adjamt;
			  else
				result.alpha+= adjamt;
			 }
		  }
		}
	 }
}


con_vec Ralph2(situation &s)
{
static int firstcall = 1;
con_vec result = CON_VEC_EMPTY;   // This is what is returned.
double ideal_alpha;
int segnum, nxtseg, nxtnxtseg;


if (firstcall)
  {                               //  this is the very first call
  my_name_is("Ralph2");            //  this lets everyone know who
                                  //  we are.

  firstcall = 0;                  //  theres only one first call

  return result;                  //  must return an answer
  }

if (s.starting)
  {
  track = get_track_description();
  }

if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt,
&result.alpha,&result.vc))
  return result;

getabsxy(s.seg_ID, s.to_lft, s.to_rgt, s.to_end);

// point A
segnum = s.seg_ID;
nxtseg = (segnum+1)%track.NSEG;
nxtnxtseg = (nxtseg+1)%track.NSEG;

if (s.cur_rad == 0.0)
  ideal_alpha = angleforcurve(nxtseg, s.v);
 else
  {
  // look for either an s curve or a situation where we have
  // an  s curve with a straight in the middle
  ideal_alpha = angleforcurve(segnum, s.v);

  if (curveradiuslow(nxtseg) != 0)
    {  // one curve following another.
  
    //  now we look to see if we have an s-curve situation
    if (curveradiuslow(segnum) * curveradiuslow(nxtseg)  < 0)
      {
      double nextcurvealpha = angleforcurve(nxtseg, s.v);

      // can we see next curve
      if (
          (curveradiuslow(segnum) > 0 &&  nextcurvealpha < ideal_alpha)
          ||
          (curveradiuslow(segnum) < 0 &&  nextcurvealpha > ideal_alpha)
         )
        ideal_alpha = nextcurvealpha;
      }
    }
   else
    {
       // not an s curve, check for opposite direction curves with
       // an intervening straight

    if (curveradiuslow(segnum) * curveradiuslow(nxtnxtseg)  < 0)
      {
      double nextcurvealpha = angleforcurve(nxtnxtseg, s.v);

      // can we see next curve
      if (
          (curveradiuslow(segnum) > 0 &&  nextcurvealpha < ideal_alpha)
          ||
          (curveradiuslow(segnum) < 0 &&  nextcurvealpha > ideal_alpha)
         )
        ideal_alpha = nextcurvealpha;
      }

    }

  }

result.alpha = ideal_alpha - gc_dot;
// point B

getspeed(s, result);
link_alpha_and_vc(s, result);
avoidcars2(s, result);

if(s.starting) result.fuel_amount = MAX_FUEL;
result.request_pit = 0;
 if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
     {
     result.request_pit = 1;
     result.repair_amount=s.damage;
     result.fuel_amount = MAX_FUEL;
     }
	
return result;
}




