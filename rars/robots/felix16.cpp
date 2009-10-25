//----------------------------------------------------------------------------
//
// RARS Robot racing code FELIX16.CPP from Doug Eleveld
//
// Developed for RARS 0.72 with DJGPP/Allegro, but it should work on 0.74
//
//----------------------------------------------------------------------------
//
// Anybody can do whatever they want with this code, but if you use ideas
// I'd like you to send me an email.  This source code is public.
//
// The car looks at the road ahead, tracing the path of the mid or outside
// line and tries to follow a point that is some distance ahead.  The
// distance is proportional to the speed of the car. There is also a fairly
// primitive 'speed override' that makes the car go fast when it sees that
// there is a nice open path ahead.  The collision avoidance code came from
// my DougE1 car, but I removed the anti-bulldozing code because this car
// doesn't have a 'lane' to drive in to change when it look like it is
// bulldozing.  I also added a possibility to choose a path around the
// nearest car which helped avoid collisions considerably.  Felix is now a
// pretty good passer. i.e. He gets little damage when passing, and can
// sucessfully pass cars that are just a little bit slower.  In high traffic
// situations Felix can sometimes even beat cars that have higher lap speeds,
// by virtue of it's passing ability.
//
// The path calculating algorithm has been sped up a little, but the
// algorithm is still fairly computationally expensive, so your simulation
// may be a little slow.  Sorry.
//
// I am willing to share the code with pretty much anyone who asks.
// I'd love to discuss the code/strategy with anyone.
//
// Happy racing,
//
// Doug Eleveld <deleveld@dds.nl>
//              <deleveld@my-dejanews.com>
//              <delev@anest.azg.nl>
//
// Version Info:
//
// 1.0 : Original version
//     March 26, 1999
//
// 1.1 : Simple improvement for driving path for corner setup.
//     : Improved collision avoidance by choosing path away from nearest car.
//     : Improved speed override code to consider how much curve in the track.
//     : Some minor simulation speed optimizations in Path::Extend.
//     April 29, 1999
//
// 1.2 : Maido Remm showed me fixes when more than 17 cars race. Thanks Maido!
//     : Collision avoidance rightdanger flag only considers the closest car.
//     : Small tweaks to speed override and steering constants for curves.
//     : Felix skids out less on small high radius turns.
//     : No longer calculates an avoidance steering angle.  Use mid track line.
//     : Passing is even better now.
//     May 7, 1999
//
// 1.3 : Added some code to help avoid driving outside the edges of the track
//       by quadratically changing alpha if we are within CARWID of the edge.
//       This allows us to look farther ahead and smooth the line better.
//     : Made all the Path classes static with a reset function. This speeds
//       up simulation because a Path does not have to be constructed each step.
//     : The car now ties to follow the outside of each corner and looks to the
//       outside whenever skidding, i.e. a high centripetal acceleration. This
//       effectively gives a lower steering gain which picks smoother paths.
//     : Passing is less paranoid now, and it only tries to get out of the way
//       of other cars that are within braking distance.
//     : The car look ahead at the track and draws lines to the max left and max
//       right sides of the track.  When these lines cross, the effective radius
//       of the track is estimated, and then the speed is set according to the
//       effective radius.
//     : Combined the path look ahead for the speed and steering into one to
//       slightly speed up the simulation.
//     : Added the optimizer code to optimize parameters if OPTIMIZE is defined.
//     : Don't look for an avoidance path if we are already skidding that way.
//     : When optimizing, remove the report, as it wastes disk space and wears
//       out the head for nothing useful.
//     : Added the possibility to match specific tracks to parameter sets.
//     : Created the track longrand.trk which is very long (>75 miles) and has
//       the same length/segments relationship as the main tracks.
//     : Optimized constants for long random track (longrand.trk) and used the
//       resulting population as a base values for optimizing on the main tracks.
//     : Optimized values wasn't delivering all that much extra speed, so I
//       decided to leave it out, and just keep the values obtained from the
//       random track optimization.
//     August 17, 1999
//
// 1.4 : Modified car to follow the outside of the upcoming corner if the current
//       corner is very shallow, i.e. our speed is less that the corner speed.
//     : Changed the meaning of the SpeedCloseCross parameter.
//     : Added the CornerHold variable to optimizations.
//     : Reoptimized the base parameters with LONGRAN2.TRK which is a long random
//       track with michigan taked onto the end twice.
//     September 15, 1999
//----------------------------------------------------------------------------


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"


// Optimization flags
//#define OPTIMIZE
//#define ONE_OPTIMIZE_FILE
//#define DATAFILE "douge3.dat"

// Display flags
//#define DATA_DISPLAY
//#define SLOW
//#define VERY_SLOW
//#define SKIDMARKS


#ifdef OPTIMIZE // Make simulation as fast as possible if we are optimizing
#undef SLOW
#undef VERY_SLOW
#define SKIDMARKS
#endif


// Just to make sure that we have no name clashes with other files
#define RobotMain Felix16
#define RobotName "Felix16"
#define Point     Felix16Point
#define Path      Felix16Path


#ifdef DATA_DISPLAY
#include <allegro.h>
#include <stream.h>
#include <iomanip.h>
extern int no_display;
#endif


#ifdef OPTIMIZE  // Don't print the MPH out if we are optimising with a display
extern int no_display;
#endif


#define sqr(x)   ((x)*(x))
#define min(x,y) (((x)>(y))?(y):(x))
#define max(x,y) (((x)<(y))?(y):(x))


#ifdef DATA_DISPLAY
static void DoSound(const int f = 1000)
{
  sound(f);
  rest(1);
  nosound();
}
#endif


static track_desc track;


static void AdjustSeg(int& segnum)
{
  if(segnum<0)
    segnum += track.NSEG;
    
  segnum = (segnum)%track.NSEG;
}


static bool init = false;
static con_vec BasicInit()
{
  con_vec result;
  my_name_is(RobotName);
  result.alpha = result.vc = 0;
  init = true;
  return result;
}


static double MidRadius(const double radius)
{
  if(radius<0.)
    return radius - (track.width/2.) ;
  else if(radius>0.)
    return radius + (track.width/2.) ;
  else
    return radius;
}


static double Linear(double tot, double rem, double start, double end)
{
  double done = (tot - rem) / tot;
  done = max(done,0.);
  done = min(done,1.);
  
  const double delta = end - start;
  return start + (done * delta);
}


static double Quadratic(double tot, double rem, double start, double end)
{
  return Linear(sqr(tot),sqr(rem),start,end);
}


static double Sinus(double tot, double rem, double start, double end)
{
  const double l = Linear(tot,rem*2,-PI/2.,PI/2.);
  return Linear(2.,(sin(l)+1.),end,start);
}


#ifndef OPTIMIZE // Don't include any car avoidance when optimizing to keep it fast
static void SpeedAggressiveControl(double factor, double& Speed)
{
  if((factor<1.0)&&(factor>0.1))
    Speed *= factor;
}


static double PassingDistance(double vnow, double vthen)
{
  if (vnow<vthen)
    return 0.0;  // no need to brake as we are already at the desired speed!
  return (0.025 * (vnow+vthen) * (vnow-vthen));
}


static int AvoidCars(situation &s, double& speed)
{
  int BrakeCountLimit = max(track.NSEG/2,5);
  
  static int BrakingCount[MAX_CARS];
  static int BrakingSegment[MAX_CARS];
  static int BrakingLaps[MAX_CARS];

  // Make sure we can see to the side so we can avoid bulldozing
  s.side_vision = 1;

  // Keep track if the danger comes from the right or left
  int rightdanger = 0;

  // Initially set all approach/braking to standard
  static int BrakingCountFilled = 0;
  if((BrakingCountFilled==0)||(s.starting))
  {
    for(int i=0;i<MAX_CARS;i++)
    {
      BrakingCount[i] = 0;
      BrakingSegment[i] = 0;
      BrakingLaps[i] = 0;
    }
	 BrakingCountFilled = 1;
  }

  int speeddanger = 0;
  int slambrakes = 0;
  
  double Dx[4], Dy[4], Vx[4], Vy[4];

  Dx[0] = Dy[0] = Vx[0] = Vy[4] = 0.;

  int count = 0;
  int who[4];

  double leftrightdist = 999.;
  
  for(int i=0;i<4;i++)
  {
    const rel_state& state = s.nearby[i];

    // Only calculate for valid cars
    if(state.who==999)
      continue;

    count++;
    who[i] = state.who;

    const double x  = Dx[i] = state.rel_x;    // distance to right (or left if < 0)
    const double y  = Dy[i] = state.rel_y;    // distance ahead, always positive
    const double vx = Vx[i] = state.rel_xdot;
    const double vy = Vy[i] = state.rel_ydot;
    
    // Calculate some basic helper paramters
    const double d  = sqrt(x*x + y*y);
    const double dot = x * vx + y * vy;

    // Ignore faster cars
    if(vy > 0.1)
      continue;

    // Ignore cars way behind us
    if(y<-CARLEN/2.)
      continue;

    // Ignore faster cars
    if(dot > -0.1)
      continue;

    // Ignore cars behind us
    if(y<0.)
      continue;

    // Figure out what speed to approach the car
    double Approach = 0.;
    if(BrakingCount[who[i]]>BrakeCountLimit)
      Approach += (BrakingCount[who[i]]-BrakeCountLimit)*2.;

    // Check whether you need to brake
    const double BrakeDistance = 1.5*PassingDistance(s.v,s.v+vy+Approach);
    if(d>BrakeDistance)
      continue;
    
    // Keep track if the danger is on the right or left
    if(d<leftrightdist)
    {
      if(state.to_lft<(track.width/2.) /2)
        rightdanger = 1;
      else if(state.to_rgt<(track.width/2.) /2)
        rightdanger = -1;

      else if(x>0.)
        rightdanger = -1;
      else
        rightdanger = 1;

      leftrightdist = d;
    }

    // Figure out how many feet we can travel in a short time
    const double ShortDist = s.v * 4.;

    // Cars that are close and stable can use use linear computed collision
    if((y<ShortDist)&&(fabs(vx)<3.))
    {
      // compute relative speed squared
      const double vsqr = vx*vx + vy*vy;

      // Time to closest approach is dot product divided by speed squared:
      double c_time = -dot / vsqr;

      // x-y coord at closest approach
      double x_close = x + c_time * vx;
      double y_close = y + c_time * vy;

      // check if collision would occur prior to closest approach
      // if so, reduce c_time, re-calculate x_close and y_close:
      if(x_close * x < 0.0 && y < 1.1 * CARLEN)
      {
         c_time = (fabs(x) - CARWID) / fabs(vx);
         x_close = x + c_time * vx;
         y_close = y + c_time * vy;
      }

      // Will it be a hit or a miss?
      if(fabs(x_close) > 3. * CARWID || fabs(y_close) > 2.5 * CARLEN)
        continue;
    }

    // Are we going to crash hard
    double CrashDist = 2*CARLEN;
    if(state.braking)
      CrashDist *= 2;
      
    if((fabs(x) < 1.5 * CARWID)&&
       (fabs(y) < 1.25 * CARLEN)&&
       (vx>5.)&&
       (y<CrashDist))
      ++slambrakes;
    
    // Are we close enough that we should just floor it
    if((y<1.*CARLEN)&&(fabs(x)>1.5*CARWID))
      continue;

    // Ignore cars that are on the left or right and are going away
    if((x>CARWID)&&(vx>1.)&&(vy<20.))
      continue;
    if((x<-CARWID)&&(vx<1.)&&(vy<20.))
      continue;

    ++speeddanger;
    
    // Remember who we are braking for
    const int LastSegment = BrakingSegment[state.who];
    const int LastLaps = BrakingLaps[state.who];

    if((s.seg_ID!=LastSegment)||(s.laps_to_go!=LastLaps))
      BrakingCount[state.who] += 1;

    BrakingSegment[state.who] = s.seg_ID;
    BrakingLaps[state.who] = s.laps_to_go;
  }

  // Slowly return braking to normal
  static int LastLap = -1;
  if(s.laps_to_go!=LastLap)
  {
    for(int i=0;i<MAX_CARS;i++)
    {
      // Have we not yet passed this car
		int found = 0;
      for(int j=0;j<count;j++)
        if(who[j]==i)
			 found = 1;

      // If we have passed it, make braking normal
		if((found==0)&&(BrakingCount[i]>0))
        BrakingCount[i] = 0;
    }
    LastLap = s.laps_to_go;
  }

  double newspeed = speed;
  if(speeddanger)
  {
	 // Figure out what speed to approach the first car
	 double App0 = 5.;

	 if(BrakingCount[who[0]]>BrakeCountLimit)
		App0 += (BrakingCount[who[0]]-BrakeCountLimit)*2.;

    if(speeddanger>1)
    {
		// Figure out what speed to approach the second car
      double App1 = 5.;
      
      if(BrakingCount[who[1]]>BrakeCountLimit)
        App1 += (BrakingCount[who[1]]-BrakeCountLimit)*2.;
    
      newspeed = min(s.v+Vy[0]+App0,s.v+Vy[1]+App1);
    }
    else
    {
      newspeed = s.v+Vy[0]+App0;
    }
  }
  // Possibly slam on the brakes
  if(slambrakes)
    newspeed = 10.;

  // Figure out the the speed
  newspeed = max(newspeed,5);
  if(newspeed<speed)
  {
    SpeedAggressiveControl(newspeed/speed,speed);
  }

  return rightdanger;
}
#endif //OPTIMIZE


// Variables that are filled in by the driving code
static double FuelPerLap   = 0.01;
static double DamagePerLap = 100.0;
static double LastFuel = 0.;
static double TotalFuelUsed = 0.;

static unsigned long LastDamage = 0;
static unsigned long LastLapDamage = 0;
static unsigned long TotalDamage = 0;


// Variables that define the path that is taken
static double SpeedLookahead0 = 0.047;

static double SteerLookahead0 = 0.019;
static double SteerLookahead1 = 0.027;

static double EdgeAlpha  = 0.20;
static double EdgeFactor = 11.;

static double CenBrakingFactor0 = 93.;
static double CenBrakingFactor1 = 177.;

static double PathSideFactor = 4.44;

static double LookAccHigh = 33.;
static double LookAccLow  = -1.;

static double TspdAngLimit = 1.17;
static double TspdAngCutoff = 0.39;
static double TspdAngHigh = 1.16;
static double TspdAngLow  = 0.81;

static double TspdCenLimit = 37.1;
static double TspdCenHigh = 1.3;
static double TspdCenLow  = 0.73;
static double TspdCenCutoff = 36.9;

static double SpeedCloseCross = 0.0;

static double CornerHold = 0.5;


#ifdef OPTIMIZE // Flag to end a race (by deliberate crash)
static int AbortRace = 0;
#endif //OPTIMIZE


static double Length(double rad, double length)
{
  double len = fabs(length);
  if(rad!=0.)
    len *= fabs(rad);
  return len;
}


static const double EdgeLimit = 4.;


static con_vec BasicControl(situation& s, double Speed, double Alpha)
{
  con_vec result;

#ifndef OPTIMIZE // If we crash or get damage during optimizing we will end
                 // so we don't need to handle getting unstuck
  // service routine in the host software to handle
  // getting unstuck from from crashes and pileups
  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
    return result;
#endif //OPTIMIZE

  result.alpha = Alpha;

  // Put some edges on the track so we avoid goind over the edge
  result.alpha -= Quadratic(track.width/EdgeFactor,s.to_lft,0.,EdgeAlpha);
  result.alpha += Quadratic(track.width/EdgeFactor,s.to_rgt,0.,EdgeAlpha);

  // Don't let the car stop
  Speed = max(Speed,15);

  // Work the gas pedal
  const double BrakeAccel = 0.935;   // tire speed ratio when braking
  if(s.v > 1.02 * Speed)             // if we're 2% too fast,
    result.vc = BrakeAccel * s.v;    // brake hard.
  else if(s.v < .98 * Speed)         // if we're 2% too slow,
    result.vc = s.v + 50;            // accelerate hard.
  else                               // if we are very close to speed,
    result.vc = .5 * (s.v + Speed);  // approach the speed gently.

#ifndef OPTIMIZE // Pitting during optimizing is done differently
  // Figure out how much fuel we will need if we pit
  result.fuel_amount = FuelPerLap*s.laps_to_go*1.5;
  if(result.fuel_amount>MAX_FUEL)
    result.fuel_amount = MAX_FUEL;
    
  // Figure out how much damage to get repaired which is
  // three times the average damage per lap, except if we have
  // more than 20 laps to go, then repair everything
  result.repair_amount = (long)((DamagePerLap+100)*s.laps_to_go*3);
  if(((unsigned)result.repair_amount>s.damage)||(s.laps_to_go>20))
    result.repair_amount = s.damage;

  // Basic pit/repair handling stuff
  result.request_pit = 0;

  // See if we need to stop this lap for repair
  double DamageLimit = 15000.;
  if((s.damage>DamageLimit)&&(result.repair_amount>0))
    result.request_pit = 1;

  // See if we need to stop this lap for fuel
  double FuelLimit = 1.5*FuelPerLap;
  if((s.fuel<FuelLimit)&&(result.fuel_amount>0))
	 result.request_pit = 1;

  // Qualify and practice with an almost empty
  // tank for maximum acceleration
  if(s.starting)
    if((s.stage==QUALIFYING)||(s.stage==PRACTICE))
      result.fuel_amount = 25;
    else
      result.fuel_amount = MAX_FUEL;
#endif

#ifdef OPTIMIZE  // Pitting during optimizing is done differently
   result.fuel_amount = 150;
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


#ifdef DATA_DISPLAY
  if(!no_display)
  {
    char mess[1000];
    sprintf(mess,"Time: %6.2f  Fuel: %5.1f Speed: %5f s.v: %5f  D: %5f",(float)s.lap_time,(float)s.fuel,(float)Speed,(float)s.v,(float)s.damage);
    text_mode(makecol(0,0,0));
    textout(screen,font,mess,10,0,makecol(255,255,255));
  }
#endif //DATA_DISPLAY


#ifdef DATAFILE
  if(s.laps_done==1)
  {
    static ofstream data(DATAFILE);

    const double steerwidthfact = (150./(track.width+70.));
    const double steer2dist = steerwidthfact*max(SteerLookahead1*s.v*track.width,track.width);
    
    const double directionoftravel = asin(s.vn/s.v);
    
    const double to_lft = s.to_lft + sin(result.alpha-directionoftravel)*steer2dist;
    
    data << s.seg_ID << ",\t"
         << Length(s.cur_rad,s.to_end) << ",\t"
         << s.cur_rad << ",\t" << Length(s.cur_rad,s.cur_len) << ",\t"
         << s.nex_rad << ",\t" << Length(s.nex_rad,s.nex_len) << ",\t"
         << s.after_rad << ",\t" << Length(s.after_rad,s.after_len) << ",\t"
         << s.aftaft_rad << ",\t" << Length(s.aftaft_rad,s.aftaft_len) << ",\t"
       
         << to_lft/track.width*100. << ",\t"
         << Speed << ",\t" << s.v << ",\t"
       
         << endl;
  }
#endif //DATAFILE


  return result;
}


class Point
{
  public:
   Point();
   Point(const Point& other);
   Point(const double xx, const double yy);
   void operator=(const Point& other);
   
   void Extend(const Point& p, const double angle);
   void ExtendX(const double px, const double angle);

   double Step(const double radius, const double len);

   void Mid(const Point& left, const Point& right);

   double Angle() const;
   double Length() const;

   const double& x;
   const double& y;
   
  private:
   double _x;
   double _y;

  friend class Path;
};


Point::Point()
 :x(_x),
  y(_y),
  _x(0),
  _y(0)
{ }


Point::Point(const Point& other)
 :x(_x),
  y(_y),
  _x(other._x),
  _y(other._y)
{ }


Point::Point(const double xx, const double yy)
 :x(_x),
  y(_y),
  _x(xx),
  _y(yy)
{ }


void Point::operator=(const Point& other)
{
  _x = other._x;
  _y = other._y;
}


// Add a given x,y displacement on top of another x,y while at some
// given angle assuming that the second x,y thinks that it has no angle
void Point::Extend(const Point& p, const double angle)
{
  _y += p.y*cos(angle);
  _x += p.y*sin(angle);

  ExtendX(p.x,angle);
}


// Add a given x,y displacement on top of another x,y while at some
// given angle assuming that the second x,y thinks that it has no angle
void Point::ExtendX(const double px, const double angle)
{
  _y += px*cos(PI/2. + angle);
  _x += px*sin(PI/2. + angle);
}


// Add a given x,y displacement on top of another x,y while at some
// given angle assuming that the second x,y thinks that it has no angle
void Point::Mid(const Point& one, const Point& two)
{
  _x = (one._x + two._x)/2.;
  _y = (one._y + two._y)/2.;
}


// Tell what angle that the point makes w.r.t vertical
double Point::Angle() const
{
  const double dist = Length();

  if(dist==0.)
    return 0.;

  if(x>=0.)
    return acos(_y/dist);
  return -1.*acos(_y/dist);
}


// Tell how long our pointer is
double Point::Length() const
{
  return sqrt(_x*_x + _y*_y);
}


// Find the X and Y displacements when projecting along a segment
// len is in ft for straight and rad for curve
// return is the distance in ft along the curve
double Point::Step(const double radius, const double len)
{
  // For a straight segment it is easy
  if(radius==0)
  {
    _x = 0.;
    _y = len;
    return len;
  }

  // Project the point on the radius along the x,y axis
  _x = radius - radius*cos(len);
  _y = fabs(radius)*sin(len);
            
  return fabs(len*radius);
}


// Looks at the track data structure for the radius of a given segment
static double SegRadius(int segnum)
{
  AdjustSeg(segnum);
  
  if (track.trackin[segnum].radius >= 0)
    return track.trackin[segnum].radius;
  else
    return track.trackin[segnum].radius + track.width;
}


static double _SegLength(int segnum)
{
  AdjustSeg(segnum);
  
  return track.trackin[segnum].length;
}


static double SegLength(int segnum)
{
  AdjustSeg(segnum);
  
  return Length(SegRadius(segnum),_SegLength(segnum));
}


static double Speed(double rad)
{
  if(rad==0.)
    return(250.);

  // Find the pure cornering speed
  return min(6.*sqrt(fabs(rad)),250.);
}


static double SegSpeed(int segnum)
{
  return Speed(SegRadius(segnum));
}


static double BrakeDist(const double s_cen_a, const double sv, const double vthen)
{
  if(sv<vthen)
    return 0.;

  const double factor = 1. + fabs(s_cen_a/CenBrakingFactor0);
  return factor*0.016*(sqr(sv)-sqr(vthen));
}


static double SegBrakeDist(const double s_cen_a, const double sv, int segnum, double(*SpeedFunc)(int))
{
  const double vthen = SpeedFunc(segnum);

  return BrakeDist(s_cen_a,sv,vthen);
}


static double ApproachSpeed(const double s_cen_a, const double dist, const double vt)
{
  const double factor = 1. + fabs(s_cen_a/CenBrakingFactor1);
  const double d = 0.016*factor;
  const double vn = sqrt(dist+d*sqr(vt))/sqrt(d);
  return min(vn,250.);
}


static double BaseSpeed(const double s, const double sv, int segnum, double toend, double(*SpeedFunc)(int))
{
  AdjustSeg(segnum);
  toend = max(toend,0.);

  double speed = SegSpeed(segnum);
  
  // Do we brake for the next segment
  double brakedist = SegBrakeDist(s,sv,segnum+1,SpeedFunc);
  double endlength = Length(SegRadius(segnum),toend);
  if(brakedist>endlength)
    speed = SegSpeed(segnum+1);

  // Do we brake for the segment after that
  brakedist = SegBrakeDist(s,sv,segnum+2,SpeedFunc);
  endlength += SegLength(segnum+1);
  if(brakedist>endlength)
    speed = SegSpeed(segnum+2);
    
  // Do we brake for the last segment we can see
  brakedist = SegBrakeDist(s,sv,segnum+3,SpeedFunc);
  endlength += SegLength(segnum+2);
  if(brakedist>endlength)
    speed = SegSpeed(segnum+3);

  return speed;
}


// Find the length (ft or rad) that we must look to go some fixed distance (ft)
// len is in ft
// return is in ft for straight and rad for curve
static double StepLength(const double radius, const double lenft)
{
  // Straigts just look along their length
  if(radius==0.)
    return lenft;

  // Curves must be corrected for radius
  return lenft/fabs(radius);
}


// Find the angle achieved by going along a segment
// len is in ft for straight and rad for curve
static double EndAngle(const double radius, const double len)
{
  if(radius==0.)
    return 0.;
  else if(radius<0.)
    return -len;
  else
    return len;
}


class Path
{
  private: // And unimplemented
   void operator=(const Path&);
   Path(const Path& other);

  public:
   Path();

   void Reset(int seg, double lft, double end);

   double Search(const double distance, const double spd);

   const int& SegNum;

   const Point& Left;
   const Point& Right;
   const Point& Mid;
   const Point& Dest;
   
   const double& MinClosure;
   const double& ExtendAngle;
   const double& LookRight;
   const double& ToEnd;
   
  private:
   int segnum;
   int segid;

   double Extend(const double len);

   Point mid;
   double extendangle;

   double tolft;

   Point left, right, dest;
   double minclosure;

   // Search state related variables
   double len;
   double lastlen;
   double endlen;

   // NON-Resetable variables
   double lookright;
};


Path::Path()
 :SegNum(segnum),
  Left(left),
  Right(right),
  Mid(mid),
  Dest(dest),
  MinClosure(minclosure),
  ExtendAngle(extendangle),
  LookRight(lookright),
  ToEnd(endlen),
  lookright(1.)
{ }


void Path::Reset(int seg, double lft, double end)
{
  AdjustSeg(seg);

  segnum = seg;
  segid = seg;
  
  extendangle = 0.;
  tolft = lft;
  
  left._x = track.width/2.;
  left._y = 0.;
  
  right._x = -(track.width/2.);
  right._y = 0.;
  
  len = 0.;
  lastlen = 0.;
  endlen = end;

  mid._x = 0.;
  mid._y = 0.;
  
  dest._x = 0.;
  dest._y = 0.;
  
  minclosure = 2.*PI;
}


double Path::Extend(const double len)
{
  // Extend the path along the angle
  static Point pmid;
  const double exlen = pmid.Step(MidRadius(SegRadius(segnum)),len);
  mid.Extend(pmid,extendangle);

  // Update the extension angle
  extendangle += EndAngle(SegRadius(segnum),len);
  
  // Project to the left and right sides of the track
  static Point l, r;
  l = mid;
  r = mid;

  const double inset = CARWID/PathSideFactor + EdgeLimit;

  l.ExtendX(tolft-inset,extendangle);
  r.ExtendX(-track.width+tolft+inset,extendangle);

  dest = mid;
  dest.ExtendX(-(track.width/2.)+tolft,extendangle);
  
  // Keep track of the crossing of the paths
  if(l.Angle()<left.Angle())
    left = l;
  if(r.Angle()>right.Angle())
    right = r;

  // Keep track of how close they have come
  const double closure = left.Angle() - right.Angle();
  minclosure = min(minclosure,closure);

  return exlen;
}

  
static double LookRight(int segnum, const double toend, const double spd)
{
  AdjustSeg(segnum);

  const double endlen = _SegLength(segnum);
  const double len = endlen - toend;

  // Maybe switch from left to right side on a straight
  const double prerad = SegRadius(segnum-1);
  const double currad = SegRadius(segnum);
  const double nexrad = SegRadius(segnum+1);

  // What would it be for the upcoming curves
  double curveright = 0.5;
  if((prerad<0.)&&(nexrad>0.))
    curveright = Sinus(endlen,len,1.,0.);
  else if((prerad>0.)&&(nexrad<0.))
    curveright = Sinus(endlen,len,0.,1.);

  else if(nexrad<0.)
    curveright = 0.;
  else if(nexrad>0.)
    curveright = 1.;

  // If we are on a straight or a fast curve, give where to look
  const double topspeed = SegSpeed(segnum);
  if(currad<0.)
    return Quadratic(topspeed,spd,0.,curveright);
  else if(currad>0.)
    return Quadratic(topspeed,spd,1.,curveright);
  else
    return curveright;
}


double Path::Search(const double distance, const double spd)
{
  // How much farther do we search
  if(distance<=0.)
    return 0.;

  lastlen = len;
  len += StepLength(MidRadius(SegRadius(segnum)),distance);

  lookright = ::LookRight(segnum,endlen-len,spd);
  
  // Maybe go to the next segment
  if(len >= endlen)
  {
    // Find out how much length we were successful at getting
    const double exlen = Extend(endlen-lastlen);
    
    // Point to the next segment to look at
    if((segnum-segid)<6)
      segnum++;
    else
      return 1e6;

    endlen = _SegLength(segnum);

    // Search the next segment for the extra bit
    len = 0.;
    return exlen + Search(distance - exlen,spd);
  }

  // Extend where we are
  return Extend(len-lastlen);
}


static void SetTrackData(const double* data, const unsigned int)
{
  SpeedLookahead0 = 0.047 + (data[0]-0.5)*0.02;

  SteerLookahead0 = 0.019 + (data[1]-0.5)*0.01;
  SteerLookahead1 = 0.027 + (data[2]-0.5)*0.02;
  
  EdgeAlpha  = 0.2 + (data[3]-0.5)*0.1;
  EdgeFactor = 11. + (data[4]-0.5)*4.;

  CenBrakingFactor0 =  93. + (data[5]-0.5)*120.;
  CenBrakingFactor1 = 177. + (data[6]-0.5)*120.;

  PathSideFactor = 4.44 + (data[7]-0.5)*1.;

  LookAccHigh = 33.0 + (data[8]-0.5)*10.;
  LookAccLow  = -1.0 + (data[9]-0.5)*10.;

  TspdAngLimit  = 1.17 + (data[10]-0.5)*0.6;
  TspdAngCutoff = 0.39 + (data[11]-0.5)*0.6;
  TspdAngHigh   = 1.16 + (data[12]-0.5)*1.0;
  TspdAngLow    = 0.81 + (data[13]-0.5)*0.6;

  TspdCenLimit  = 37.1 + (data[14]-0.5)*20.;
  TspdCenHigh   = 1.30 + (data[15]-0.5)*1.0;
  TspdCenLow    = 0.73 + (data[16]-0.5)*0.6;
  TspdCenCutoff = 36.9 + (data[17]-0.5)*20.;
  
  SpeedCloseCross = 0.5 + (data[18]-0.5)*1.;
  
  CornerHold = 0.5 + (data[19]-0.5)*1.;
}


#ifndef OPTIMIZE // No need for track recognition if we are optimizing

typedef struct TrackDataSet 
{
  const char* trackname;
  double data[20];
} TrackDataSet;


static TrackDataSet TrackData[] =
{
  {"albrtprk.trk", {0.759722,0.584641,0.422155,0.298846,0.756257,0.250181,1.02636,0.844884,1.06865,0.792976,0.533367,0.504222,0.857316,0.124889,0.836029,0.173876,0.440861,0.838387,0.0267114,0.946837}},
  {"austria.trk",  {0.963801,0.542972,0.335514,0.180889,0.837809,0.472246,0.881405,0.96319,0.938664,0.626719,0.363702,0.818186,0.605166,0.124889,0.797254,0.140822,0.796368,0.485646,0.443087,0.323017}},
  {"barcelon.trk", {0.753855,0.569918,0.420687,0.758432,0.857883,0.062063,0.990632,0.742017,0.622341,0.698808,0.0112177,0.486816,0.623805,0.494443,0.832236,0.537253,0.197335,0.344626,0.767383,-0.0417508}},
  {"brazil.trk",   {0.0671578,0.638337,0.421626,0.340708,0.232038,0.497502,0.95031,1.07606,0.455351,0.298644,0.945249,0.478743,0.665581,0.993038,1.00312,0.181793,0.551124,0.695465,0.826728,0.163017}},
  {"hock.trk",     {0.0859577,0.607981,0.439261,0.638049,0.958223,0.323089,0.87999,0.851841,0.984616,0.625808,0.0289584,0.927365,0.708864,-0.0318159,0.273418,0.13241,0.960497,0.396312,0.193136,0.00923679}},
  {"hungary.trk",  {0.889324,0.595553,0.391413,0.808542,0.506152,-0.0595055,1.05249,0.934229,0.741989,0.589899,0.447727,0.652485,0.53065,0.124889,-0.0770929,0.372286,0.943113,0.629869,0.366826,0.153034}},
  {"imola.trk",    {0.310985,0.561623,0.365087,0.603005,0.422791,0.567943,0.760377,0.887456,0.817641,0.529025,0.521002,0.0726057,0.767766,0.229847,0.705942,1.06971,0.0292859,0.405556,0.352215,0.606031}},
  {"indygp.trk",   {0.139766,1.1586,0.395064,0.397273,0.756257,0.37088,0.986276,0.922007,1.38134,0.725053,0.303505,0.518821,0.982342,0.130585,1.00611,0.165562,0.440861,0.378221,0.123915,0.314739}},
  {"magnycrs.trk", {0.766576,0.509565,0.437782,0.180695,0.586009,0.327283,0.966193,0.849171,1.06566,0.569117,0.345119,0.423929,0.592407,0.103461,0.277877,0.457581,0.623009,0.343712,0.620558,0.0491278}},
  {"monaco.trk",   {0.976149,0.963058,0.328672,0.046087,0.806622,0.789808,1.15743,0.87414,0.869825,0.651647,0.697918,0.565415,0.753632,-0.0541896,0.913729,0.128239,0.867967,0.337104,0.170457,0.274798}},
  {"montreal.trk", {0.389102,0.628327,0.496056,-0.0192293,0.764033,0.149378,1.08034,0.979647,1.00481,0.423279,-0.0535397,0.557196,1.24647,-0.198699,0.0575299,0.0597383,1.27272,0.413989,-0.01588,0.106703}},
  {"monza-76.trk", {0.188313,0.287885,0.496055,0.229498,0.978919,0.288118,0.911795,0.211576,0.844005,0.694367,0.136559,0.125937,0.626393,-0.0666118,0.998417,0.891775,0.86695,0.614304,0.697255,0.0862175}},
  {"nurnburg.trk", {0.582652,0.592672,0.437782,0.0879831,0.927936,0.71101,1.09592,0.912299,0.925371,0.299453,0.00975669,0.227602,0.878391,0.141689,0.362522,0.655761,1.02471,0.541385,0.235337,0.152403}},
  {"sepang.trk",   {0.827351,0.519129,0.458078,0.146135,0.402274,0.943267,0.817243,1.13292,0.851451,0.532532,0.172604,0.51947,0.827782,0.105801,0.109752,0.600143,0.440861,0.852507,0.560755,0.4209}},
  {"silver97.trk", {0.0342904,0.577224,0.431108,0.431103,0.432395,0.328237,1.15668,0.838225,0.72562,0.646482,0.440937,0.658763,0.592365,0.530194,0.87055,0.390318,0.590996,0.629285,0.915082,0.141318}},
  {"spa.trk",      {0.521688,0.637639,0.44615,0.308219,0.857883,0.876135,0.818124,1.08972,0.998413,0.320416,0.46207,0.0204196,0.807604,0.421305,0.783677,0.481988,0.440861,0.605639,0.630776,0.0755214}},
  {"suzuka.trk",   {0.521688,0.655822,0.518568,0.368555,0.857883,0.728319,0.0450383,1.06928,0.866648,0.604234,0.00975674,0.530683,0.649568,0.498688,0.764642,0.532246,0.28986,0.768976,0.604639,0.508502}},

  // Specially optimized on a long random track (longran2.trk)
  { NULL, {0.803408,0.572915,0.470234,0.0920334,0.885435,0.292792,0.85108,0.998325,0.92422,0.574923,0.549859,0.559854,0.440937,0.149924,0.58249,0.43531,0.513018,0.342834,0.704354,1.02778 }}
  
};


static void SetOptimizedTrackData(const char* trackname)
{
  int i = 0;

  // Go through the optimized data sets
  while(TrackData[i].trackname!=NULL)
  {
    // Look for matched trackname
    if(!strcmp(TrackData[i].trackname,trackname))
      break;

    ++i;
  }

  // Actually set the optimized values
  SetTrackData(TrackData[i].data,20);
};
#endif //OPTIMIZE

#ifdef OPTIMIZE // Declare variables for optimization

#include "optimize.h"

static const int Population = 200;
static const int GeneticLength = 20;

static optimizedata data;
static double lastlaptime = 9999.;
static unsigned long lastdamage = 9999;
static double lastdistance = 0.;

#endif //OPTIMIZE

con_vec RobotMain(situation &s)
{
  // Basic initializing stuff
  if(init==false)
    return BasicInit();

  track = get_track_description();
  
  // Load the optimization stuff for this track
#ifdef OPTIMIZE
  static int optinit = 0;
  if(optinit==0)
  {
    // Reload the population from the data file
#ifdef ONE_OPTIMIZE_FILE
    const char* optfile = "optimize.opt";
#else
    const char* optfile = trackfile[0];
#endif
    data.load(optfile,Population,GeneticLength,SetTrackData);
    cout << "\nOptimizing for track in " << optfile << endl;

    optinit = 1;
  }
#endif //OPTIMIZE

  // Do a little smoothing on the centripetal acceleration
  static double last1_cen_a = 0.;
  static double last2_cen_a = 0.;
  static double last3_cen_a = 0.;
  static double last4_cen_a = 0.;
  static double last5_cen_a = 0.;
  double last0_cen_a = max(s.cen_a,-100.);
  last0_cen_a = min(s.cen_a,100.);
  double s_cen_a = (last0_cen_a + last1_cen_a + last2_cen_a + last3_cen_a + last4_cen_a + last5_cen_a)/6.;
  
  static int started = 0;

  if(s.starting)
  {
    FuelPerLap = 0.01;
    LastFuel = 0.;
    TotalFuelUsed = 0.;
    
    LastDamage = 0;
    LastLapDamage = 0;
    TotalDamage = 0;

    started = 0;

#ifdef OPTIMIZE
    // Exit the program if a race was ended prematurely
    if((lastlaptime==0.)&&(lastdamage==0.))
      exit(1);

    // Figure out a nice estimate of the MPH, or how far we
    // got around the track
    double mph = optimizedata::mph(lastlaptime,track.length,lastdamage,lastdistance);

    if(no_display)
    {
      cout << " Mph: " << mph
           << endl;
    }

    // Tell the optimizer what the last MPH was
    data.step(mph);
    AbortRace = 0;
#endif //OPTIMIZE
  }
  
  const double directionoftravel = asin(s.vn/s.v);

#ifndef OPTIMIZE // No need for car avoidance during optimization
  // Keep track of where the dangerous cars are
  static int RightDanger = 0;
  static int BrakingToAvoid = 0;
  static int LastBrakingToAvoid = -1;

  // Change our speed strategy if we are braking to avoid something
  if(LastBrakingToAvoid!=BrakingToAvoid)
  {
    // Change the the fast parameters
    if(BrakingToAvoid==0)
      SetOptimizedTrackData(track.sName);
    // Change to the safe parameters
    else
      SetOptimizedTrackData("");
  }
  LastBrakingToAvoid = BrakingToAvoid;
#endif

  // Steer the car at the target point -----------------
  const double steerwidthfact = (150./(track.width+70.));
  const double steer1dist = steerwidthfact*max(SteerLookahead0*s.v*track.width,track.width);
  const double steer2dist = steerwidthfact*max(SteerLookahead1*s.v*track.width,track.width);

  // Steer the car at the target point -----------------
  const double speed1dist = SpeedLookahead0*sqr(s.v);

  // Move slightly to the left or right to avoid cars
  double extraoffset = 0.;

#ifndef OPTIMIZE  // No need for car avoidamve during optimization
  if((RightDanger<0)&&(s.to_lft>CARWID))
    extraoffset = CARWID*0.5;
  else if((RightDanger>0)&&(s.to_rgt>CARWID))
    extraoffset = CARWID*0.5;
#endif //OPTIMIZE

  // Information about the path ahead
  double CenterAngle = 0.;
  double RightAngle = 0.;
  double LeftAngle  = 0.;
  double SteerLookRight = 0.5;
  static Point SpeedLeft;
  static Point SpeedRight;
  double SpeedMinClosure = 0.;
  double SpeedExtendAngle = 0.;
  
  // Find the maximum distance to trace
  double maxdist = speed1dist;
  maxdist = max(maxdist,steer1dist);
  maxdist = max(maxdist,steer2dist);

  // Trace the path ahead
  static Path ThePath;
  ThePath.Reset(s.seg_ID,s.to_lft+extraoffset,s.to_end);

  int done = 0;
  double dist = 0.;
  double lastdist = 0.;
  
  while(done<3)
  {
    lastdist = dist;
    dist += ThePath.Search(20.,s.v);

    // Stop looking at the speed when we have a closed path
    // or if the distance is sufficient
    if((ThePath.MinClosure<0.)||
       ((dist>speed1dist)&&(lastdist<speed1dist)))
    {
      SpeedLeft = ThePath.Left;
      SpeedRight = ThePath.Right;
      SpeedMinClosure = ThePath.MinClosure;
      SpeedExtendAngle = ThePath.ExtendAngle;

      done++;
    }

    // Stop Looking for the passing path when we get to the right distance
    if((dist>steer1dist)&&(lastdist<steer1dist))
    {
      CenterAngle = ThePath.Dest.Angle();
      
      done++;
    }

    // Stop Looking for the steering path when we get to the right distance
    if((dist>steer2dist)&&(lastdist<steer2dist))
    {
      SteerLookRight = ThePath.LookRight;
    
      RightAngle = min(ThePath.Right.Angle(),ThePath.Left.Angle());
      LeftAngle  = max(ThePath.Right.Angle(),ThePath.Left.Angle());
      
      done++;
    }
  }

  // Find the extention angle at the end of the speed path
  double sang = SpeedExtendAngle - (SpeedLeft.Angle() +  SpeedRight.Angle())/2.;

  // Find the smoothest path to the left or to the right
  double steerleft = -PI;
  steerleft = max(steerleft,CenterAngle);
  steerleft = max(steerleft,RightAngle);
  steerleft = max(steerleft,LeftAngle);

  double steerright = PI;
  steerright = min(steerright,CenterAngle);
  steerright = min(steerright,RightAngle);
  steerright = min(steerright,LeftAngle);
  
  // Put the left and right steering angles down
  if(s_cen_a>LookAccLow)
    SteerLookRight = Linear(LookAccHigh-LookAccLow,s_cen_a-LookAccLow,1.,SteerLookRight);
  if(s_cen_a<-LookAccLow)
    SteerLookRight = Linear(LookAccHigh-LookAccLow,-s_cen_a+LookAccLow,0.,SteerLookRight);

  // Steer to the side that makes us turn the leaast if we are accelerating centripetallly
  double SteerAngle = Linear(1.,SteerLookRight,steerright,steerleft);
  
  // Adjust steering angle during dragout
  static int dragout = 0;
  static int startseg = 0;
  static double startend = 0;
  if(!started)
  {
    startseg = s.seg_ID;
    startend = s.to_end;
    
    started = 1;
    dragout = 1;
  }

  if(dragout)
  {
#ifndef OPTIMIZE  // No need to load previos track data sets when optimizing
    SetOptimizedTrackData(track.sName);
#endif //OPTIMIZE
    
    if(s.seg_ID==startseg)
      SteerAngle *= Linear(startend,s.to_end,0.,1.);
    else
      dragout = 0;
  }
  
  // Set the basic speed of the track
  double Speed = BaseSpeed(s.cen_a,s.v,s.seg_ID,s.to_end,SegSpeed);
  const double BasicSpeed = Speed;
  
  if(SpeedMinClosure>0.)
  {
    const double Speed1 = max(Speed,Linear(1.,cos(sang),250.,Speed));
    const double Speed2 = max(Speed,Linear(1.,fabs(sang*sang),Speed,250.));

    Speed = SpeedCloseCross*Speed1 + (1.-SpeedCloseCross)*Speed2;
  }
//  if(0)
//  else
  {
    // Look at the speed path ahead and figure out the radius that
    // it looks like we are pointing into
    const double lo = max(SpeedLeft.Length(),SpeedRight.Length());
    const double sh = min(SpeedLeft.Length(),SpeedRight.Length());

    const double d = lo - sh;
    const double w = track.width;
    const double rad = (sqr(d) - sqr(w))/(2.*w);
    double tspd = ::Speed(rad);

    // Change the terminal speed if the extend angle is large
    tspd *= Linear(TspdAngLimit,fabs(sang)-TspdAngCutoff,TspdAngLow,TspdAngHigh);
    tspd *= Linear(TspdCenLimit,fabs(s_cen_a),TspdCenLow,TspdCenHigh);

    // What speed can we approach the apex at
    const double spd = ApproachSpeed(s.cen_a,sh,tspd);

    if((spd>Speed)&&
      (fabs(s_cen_a)<TspdCenCutoff))
      Speed = spd;
  }

  // Soften the speed if we are in a long curve and have nothing else better
  const double ThisRadius = SegRadius(s.seg_ID);
  if((s.seg_ID==ThePath.SegNum)&&
     (ThisRadius!=0.))
  {
    // If the curve is to the left and we are over the mid line
    if(ThisRadius>0.)
      Speed = Linear(track.width*CornerHold,s.to_lft,BasicSpeed,Speed);
     
    // If the curve is to the right and we are over the mid line
    else if(ThisRadius<0.)
      Speed = Linear(track.width*CornerHold,s.to_rgt,BasicSpeed,Speed);
  }

  //---------------------
  // Notice when we have tanked and how much fuel we have used
  if(s.lap_flag)
  {
    // Figure out how much fuel we have used until now
    if(s.fuel<LastFuel)
      TotalFuelUsed += LastFuel - s.fuel;

    // Figure out how many pounds of fuel per lap
    if(s.laps_done>0)
      FuelPerLap = TotalFuelUsed/s.laps_done;

    LastFuel = s.fuel;
  }

  //---------------------
  // Keep track of how much damage we take per lap
  if(s.damage!=LastDamage)
  {
    // Figure out how much damage we got until now
    if(s.damage<LastDamage)
      TotalDamage += LastDamage - s.damage;

    // Figure out how much damage occurs per lap
    if(s.laps_done>0)
      DamagePerLap = TotalDamage/s.laps_done;

    LastLapDamage += (s.damage - LastDamage);
    LastDamage = s.damage;
  }
  
  if(s.lap_flag)
    LastLapDamage = 0;

#ifndef OPTIMIZE // No need for car avoidance when optimizing
  // Remember the speed before we do any avoidance
  const double FreeSpeed = Speed;

  // Avoid other cars if they are on the road
  if((s.stage!=QUALIFYING)&&(s.stage!=PRACTICE))
    RightDanger = AvoidCars(s,Speed);

  // If we are braking for a car, go back to the safe steering constants
  if(Speed<FreeSpeed)
    BrakingToAvoid = 1;
  else
    BrakingToAvoid = 0;

  // Don't choose a path to the side if we are
  // allready skidding that way
  if((s_cen_a>25.)&&(RightDanger<0))
    RightDanger = 0;
    
  if((s_cen_a<25.)&&(RightDanger>0))
    RightDanger = 0;

  // Choose the path that is least blocked by other cars
  if(RightDanger>0)
    SteerAngle = min(SteerAngle,CenterAngle);
    
  if(RightDanger<0)
    SteerAngle = max(SteerAngle,CenterAngle);
#endif

  double Alpha = SteerAngle - directionoftravel;

#ifdef SKIDMARKS
    extern int skidmarks;
    skidmarks = 1;

    extern int designated;
    designated = 0;
#endif

#ifdef DATA_DISPLAY
  if(!no_display)
  {
    char mes[100];
    sprintf(mes,"Close: %5f  Alpha: %5f  SCenA %5f  Distance %5f  ExtraSpd %5f ",SpeedMinClosure,Alpha,s_cen_a,s.distance/track.length,Speed-BasicSpeed);
    text_mode(makecol(0,0,0));
    textout(screen,font,mes,10,10,makecol(255,255,255));
  }
#endif

#ifdef SLOW
#ifndef VERY_SLOW
  rest(1);
#endif
#endif

#ifdef VERY_SLOW
  getch();
#endif

  // Keep the last centripetal accel for smoothing
  last5_cen_a = last4_cen_a;
  last4_cen_a = last3_cen_a;
  last3_cen_a = last2_cen_a;
  last2_cen_a = last1_cen_a;
  last1_cen_a = last0_cen_a;
  
#ifdef OPTIMIZE // Keep track of the data from the last lap
  lastlaptime = s.lap_time;
  lastdamage = s.damage;
  lastdistance = s.distance;
#endif //OPTIMIZE
  
  // Basic control loop
  return BasicControl(s,Speed,Alpha);
}

















