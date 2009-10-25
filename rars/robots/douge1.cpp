//----------------------------------------------------------------------------
//
// RARS Robot racing code DougE1.CPP from Doug Eleveld
//
// Developed for RARS 0.72 with DJGPP/Allegro
//
//----------------------------------------------------------------------------
//
// !!! DougE1.cpp must be associated with data file DougE1.dat !!!
// !!! otherwise nothing will work !!!
//
// Anybody can do whatever they want with this code, but if you use ideas
// I'd like you to send me an email.  This source code is public.
//
// The car 'recognizes' the road that it on by looking at the length, radius
// etc of the segments and looks up in a big table of optimized values that
// is in the datafile.  The optimizer is another program and I won't include
// it here as there is just simply too much code.  If it gets put on a track
// for which it is not optimized, the 'best fit' of the optimized values is
// chosen.  This usually means that it will crash alot on tracks for which it
// is not optimized, and probably won't make it even once around.  It is
// optimized for all the tracks the RARS 0.72 except, of course, for
// RANDOM.TRK.  Optimization uses a genetic algorithm with an initial path
// and speed that are set by hand.  I have not done anything to increase
// performance on un-optimized tracks, although I may do leave-one-out
// cross validation to find the best weights for 'recognizing' each segment.
// I am willing to share the code with pretty much anyone who asks.
//
// The car's path is dependant on the road, and not on global coordinates.
// This means that is is fairly stable, although it performs poorly
// (especially passing) when there are many short (compared to car length)
// sections of road like in MONACO.TRK or high speed straights that end in
// sharp curves like in AUSTRIA.TRK.  It loves long sweeping curves like
// SPA.TRK.  It performs well on MONZA-76.TRK for most of the track, but
// gets disturbed by the short chicane-thing and that destroys it's time.
// I have been thinking about an 'override' to global coordinates for these
// difficult sections, but I not sure if I can do that...
//
// What I have also implemented that I have not earlier seen here is that
// different approach speeds are used for different cars and anti-bulldozing
// code.  The approach speed of a car depends on how long DougE1 has been
// trying to pass them.  This is intended to avoid getting stuck behind
// someone that I cannot pass, which especially happens on short ovals.
// The passing code is only mediocre, even though it is quite complex.  It
// is particularly poor at passing groups of cars, therefore it doesn't
// usually do to well on the short ovals.  Sometimes it passes great, but
// when it does crash it usually crashes in a spectacular fashion.
//
// You may notice some wierd or non-optimal things in the physics here.  This
// is because I know very little about the physics of car racing (I'm an
// electrical engineer), but I do know a fair bit about optimization.  And
// that is what DougE1 is pretty much completely based on, optimized paths
// for each track.
//
// I got the inspriration for my pattern based matching stuff from looking
// at the Wappucar source, although I implemented it in a completely
// different way.  There is also a tiny little bit of code from Apex8 in the
// collision avoidance code.  Other than that I think it's a pretty original
// robot.
//
// I'd love to discuss the code with anyone, and would actually like to team
// up with someone who knows how to white a better collision avoidance than
// I have.  I'd might want to make one using global coordinates, based on the
// TURTLE9 source.  Or maybe make a car only for ovals with seperate fast
// path and passing path.
//
// Doug Eleveld <deleveld@dds.nl>
//              <deleveld@my-dejanews.com>
//              <delev@anest.azg.nl>
//
// Version Info:
//
// 1.0 Original version
//     Febuary 1998
//
// 1.1 Fixed bugs w.r.t. multiple races by initializing static variables
//     preoperly when starting.
//     Feb 25, 1999
//
// 1.2 Maido Remm showed me some bug fixes when more than 17 cars race.
//     Thanks Maido.
//     May 7, 1999
//
//----------------------------------------------------------------------------


// Changed 'class Track' to 'class DougE1Track' to avoid conflict with new class defining track in the simulator (by Carsten Kjaer)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <conio.h>
#include <math.h>
#include <assert.h>
#include "car.h"
#include "track.h"


//#define BINARY_DATAFILE
//#define DATA_DISPLAY


#ifdef DATA_DISPLAY
#include <allegro.h>
extern int no_display;
static bool SlowDisplay = true;
#endif


static double Width (const situation &s)
{
  return s.to_lft + s.to_rgt;
}


static double CornerSpeed(double radius)
{
  const double CornerLateralG  = 0.92;  // lateral g's expected when cornering

  const double s = sqrt(fabs(radius) * 32.2 * CornerLateralG);
  if(s>250)
    return 250;
  return s;
}


static double Speed (double rad)
{
  if(fabs(rad)<0.001)
    return 250;
  return CornerSpeed(rad);
}


static double Curve(double rad, double len)
{
  if(fabs(rad)<0.001)
    return 0;

  if(rad<0.)
    return len*100.;
  else
    return len*-100.;
}


static double Length(double rad, double length)
{
  double len = fabs(length);
  if(rad!=0.)
    len *= fabs(rad);
  return len;
}


static double EndLength(const situation &s)
{
  double len = fabs(s.to_end);
  if(s.cur_rad!=0.)
    len *= fabs(s.cur_rad);
  return len;
}


static double CriticalDistance(double vnow, double vthen)
{
  if (vnow<vthen)
    return 0.0;  // no need to brake as we are already at the desired speed!
  return (0.025 * (vnow+vthen) * (vnow-vthen));
}


static double ReadNumber(FILE* stream)
{
  // Flag that there was a problem
  if(feof(stream))
    return 0.;

#ifndef BINARY_DATAFILE
  // Read in the numeric data
  char temp[100];
  fscanf(stream,"%s",temp);

  // Skip lines starting with #
  int c = temp[0];
  if(temp[0]=='#')
  {
    while(c!='\n')
      c = fgetc(stream);

    return ReadNumber(stream);
  }

  // Convert the ascii to a double
  float ret = 0.;
  sscanf(temp,"%f",&ret);
#else
  float ret = 0.;
  fread(&ret,sizeof(float),1,stream);
#endif
  
  return ret;
}

//-------------------

class DriveData
{
  public:
   DriveData();
   ~DriveData();
   
   double speed;
   double drivelane;
   double entrylane;
   double speedcross;
   double lanecross;

   void read(FILE* stream);
};


class RoadData
{
  public:
   RoadData();
   ~RoadData();
   
   double width;

   double prevspeed;
   double prevlength;
   double prevcurve;
                 
   double speed;
   double length;
   double curve;
                 
   double nextspeed;
   double nextlength;
   double nextcurve;
                 
   double afterspeed;
   double afterlength;
   double aftercurve;

   double aftaftspeed;
   double aftaftlength;
   double aftaftcurve;
                 
   void read(FILE* stream);
};


class Pattern
{
  private:
   Pattern();
   ~Pattern();

   void Initialize(const char* filename);

   // The internal data
   RoadData Road;
   DriveData Drive;

   // Pointer to the next pattern
   Pattern* Next;

	int Used;
   void Clean();
                 
   void read(FILE* stream);

   double Speed() const;
   double DriveLane() const;
   double EntryLane() const;
   double SpeedCrossover() const;
   double LaneCrossover() const;

  friend class DougE1Track;
};


static const double DefaultSpeed      = 0.50;
static const double DefaultDriveLane  = 0.1;
static const double DefaultEntryLane  = 0.1;
static const double DefaultSpeedCross = 0.1;
static const double DefaultLaneCross  = 0.5;


DriveData::DriveData()
  :speed(DefaultSpeed),
  drivelane(DefaultDriveLane),
  entrylane(DefaultEntryLane),
  speedcross(DefaultSpeedCross),
  lanecross(DefaultLaneCross)
{
}


DriveData::~DriveData()
{
}


void DriveData::read(FILE* stream)
{
  speed = ReadNumber(stream);
  drivelane = ReadNumber(stream);
  entrylane = ReadNumber(stream);
  
  speedcross = ReadNumber(stream);
  lanecross = ReadNumber(stream);
}


RoadData::RoadData()
{
}


RoadData::~RoadData()
{
}


void RoadData::read(FILE* stream)
{
  width = ReadNumber(stream);
  
  prevspeed = ReadNumber(stream);
  prevlength = ReadNumber(stream);
  prevcurve = ReadNumber(stream);
  
  speed = ReadNumber(stream);
  length = ReadNumber(stream);
  curve = ReadNumber(stream);
  
  nextspeed = ReadNumber(stream);
  nextlength = ReadNumber(stream);
  nextcurve = ReadNumber(stream);
  
  afterspeed = ReadNumber(stream);
  afterlength = ReadNumber(stream);
  aftercurve = ReadNumber(stream);
  
  aftaftspeed = ReadNumber(stream);
  aftaftlength = ReadNumber(stream);
  aftaftcurve = ReadNumber(stream);
}


Pattern::Pattern()
  :Next(NULL),
  Used(0)
{
}


Pattern::~Pattern()
{
  if(Next!=NULL)
    delete Next;
}


void Pattern::Clean()
{
  Pattern* ptr = this;
  while(ptr)
  {
	 ptr->Used = 0;
    ptr = ptr->Next;
  }
}


void Pattern::read(FILE* stream)
{
  Road.read(stream);
  Drive.read(stream);
}


void Pattern::Initialize(const char* filename)
{
  // Open the file for reading
#ifndef BINARY_DATAFILE
  FILE* stream = fopen(filename,"r");
#else
  FILE* stream = fopen(filename,"rb");
#endif
  // make sure the datafile has been opened
  assert(stream!=NULL);

  Pattern* Target = this;
  do
  {
    // Read in the data
    Target->read(stream);

#ifndef BINARY_DATAFILE
    // Read to the next data
    int c = fgetc(stream);
    while((c==' ')||(c==',')||(c=='\n'))
      c = fgetc(stream);

    // Check to see if we read the end
    if(c=='!')
      break;
    else
      ungetc(c,stream);
#endif

    // Make the next pattern
    Pattern* NewTarget = new Pattern;

    // And link it into this one
    Target->Next = NewTarget;
    Target = NewTarget;
  }
  while(!feof(stream));

  fclose(stream);
}


static int DriveLaneInvert(const RoadData& Road)
{
  const double fspeed = fabs(Road.speed)*Road.length/(fabs(Road.curve)+0.05);
  const double fnextspeed = fabs(Road.nextspeed)*Road.nextlength/(fabs(Road.nextcurve)+0.05);

  // Are we on the tightest curve
  if(fspeed<fnextspeed)
  {
    if(Road.curve<0)
		return 1;
	 return 0;
  }
  // Next speed is tighter
  else
  {
    if(Road.nextcurve<0)
		return 0;
	 return 1;
  }
};


static int EntryLaneInvert(const RoadData& Road)
{
  const double fprevspeed = fabs(Road.prevspeed)/(fabs(Road.prevcurve)+0.05);
  const double fspeed = fabs(Road.speed)/(fabs(Road.curve)+0.05);
  const double fnextspeed = fabs(Road.nextspeed)/(fabs(Road.nextcurve)+0.05);

  // Swing extra room for entry in a LSR or RSL
  if(fspeed>fnextspeed)
  {
    if(fspeed>fprevspeed)
    {
      const double totalcurve = Road.prevcurve * Road.nextcurve;
      
      if(totalcurve<0.)
        return !DriveLaneInvert(Road);
    }
  }
  return DriveLaneInvert(Road);
};


double Pattern::Speed() const
{
  return Drive.speed * 250.;
}

   
double Pattern::DriveLane() const
{
  if(DriveLaneInvert(Road)==0)
    return Road.width-Drive.drivelane*Road.width;
  else
    return Drive.drivelane*Road.width;
}


double Pattern::EntryLane() const
{
  if(EntryLaneInvert(Road)==0)
    return Road.width-Drive.entrylane*Road.width;
  else
    return Drive.entrylane*Road.width;
}


double Pattern::SpeedCrossover() const
{
  return Drive.speedcross*Road.length + 0.1;
}

   
double Pattern::LaneCrossover() const
{
  return Drive.lanecross*Road.length + 0.1;
}


class DougE1Track
{
  public:
   DougE1Track();
   ~DougE1Track();

   static DougE1Track TheTrack;

   void Initialize(const char* filename);

   const DriveData& Segment(int i);
   int NumSegments() { return Segments; };

  private:
   void FindLenRad(track_desc& track, int i, double &len, double& rad);
   Pattern& ClosestPattern(Pattern& Memory, const RoadData& road);
   void MatchPatternMemory(Pattern& Memory, const RoadData& road, DriveData& dat);

   int Segments;
   DriveData** Drive;
};


DougE1Track DougE1Track::TheTrack;


DougE1Track::DougE1Track()
  :Segments(0),
  Drive(NULL)
{
}


DougE1Track::~DougE1Track()
{
  for(int i=0;i<Segments;i++)
    delete Drive[i];
  delete Drive;
}


void DougE1Track::Initialize(const char* filename)
{
  int i;

  for( i=0;i<Segments;i++ )
    delete Drive[i];
  delete Drive;
  
  // Load in the memory that we have
  Pattern Memory;
  Memory.Initialize(filename);

  // Initialize the array of driving data
  track_desc track = get_track_description();
  Segments = track.NSEG;
  
  Drive = (DriveData **) malloc( Segments*sizeof(DriveData*) );
  for( i=0;i<Segments;i++ )
    Drive[i] = new DriveData;

  RoadData Road;
  for( i=0;i<Segments;i++ )
  {
    double prev_len, prev_rad;
    double cur_len, cur_rad;
    double nex_len, nex_rad;
    double after_len, after_rad;
    double aftaft_len, aftaft_rad;

    FindLenRad(track,i-1,prev_len,prev_rad);
    FindLenRad(track,i,cur_len,cur_rad);
    FindLenRad(track,i+1,nex_len,nex_rad);
    FindLenRad(track,i+2,after_len,after_rad);
    FindLenRad(track,i+3,aftaft_len,aftaft_rad);

    Road.width = track.width;
    
    Road.prevspeed = Speed(prev_rad);
    Road.prevlength = Length(prev_rad,prev_len);
    Road.prevcurve = Curve(prev_rad,prev_len);
    
    Road.speed = Speed(cur_rad);
	 Road.length = Length(cur_rad,cur_len);
    Road.curve = Curve(cur_rad,cur_len);
    
    Road.nextspeed = Speed(nex_rad);
    Road.nextlength = Length(nex_rad,nex_len);
    Road.nextcurve = Curve(nex_rad,nex_len);

    Road.afterspeed = Speed(after_rad);
    Road.afterlength = Length(after_rad,after_len);
    Road.aftercurve = Curve(after_rad,after_len);

    Road.aftaftspeed = Speed(aftaft_rad);
    Road.aftaftlength = Length(aftaft_rad,aftaft_len);
    Road.aftaftcurve = Curve(aftaft_rad,aftaft_len);

    // Get the driving path by matching the road to the memory
    MatchPatternMemory(Memory,Road,*(Drive[i]));
  }
}


void DougE1Track::FindLenRad(track_desc& track, int i, double &len, double& rad)
{
  if(i>=track.NSEG)
    i -= track.NSEG;
  if(i<0)
    i += track.NSEG;
    
  len = track.trackout[i].length;
  rad = track.trackin[i].radius;
  
  double r2 = track.trackin[i].radius;
  if(r2<rad)
    rad = r2;
}


static double PatternDifference(const RoadData& one, const RoadData& two)
{
  return (0.5*fabs(one.width-two.width)+
          0.1*fabs(one.prevspeed-two.prevspeed)+
          0.1*fabs(one.prevlength-two.prevlength)+
          0.1*fabs(one.prevcurve-two.prevcurve)+
          0.6*fabs(one.speed-two.speed)+
          0.6*fabs(one.length-two.length)+
          0.6*fabs(one.curve-two.curve)+
          1.0*fabs(one.nextspeed-two.nextspeed)+
          1.0*fabs(one.nextlength-two.nextlength)+
          1.0*fabs(one.nextcurve-two.nextcurve)+
          0.3*fabs(one.afterspeed-two.afterspeed)+
          0.3*fabs(one.afterlength-two.afterlength)+
          0.3*fabs(one.aftercurve-two.aftercurve)+
          0.1*fabs(one.aftaftspeed-two.aftaftspeed)+
          0.1*fabs(one.aftaftlength-two.aftaftlength)+
          0.1*fabs(one.aftaftcurve-two.aftaftcurve));
}


Pattern& DougE1Track::ClosestPattern(Pattern& Memory, const RoadData& road)
{
  Pattern* closest = &Memory;

  double distance = 9999999.;
  
  Pattern* ptr = &Memory;
  while(ptr!=NULL)
  {
    // Find the squared difference between the patterns
    double diff = PatternDifference(road,ptr->Road);

	 // Found two closest match so far
	 if((diff<distance)&&(ptr->Used==0))
    {
      closest = ptr;
      distance = diff;
    }
    
    // Check the next value
    ptr = ptr->Next;
  }
  return *closest;
}

                         
// Fill the driving data with the closest match of the road withing the driving data
void DougE1Track::MatchPatternMemory(Pattern& Memory, const RoadData& road, DriveData& dat)
{
  Memory.Clean();

  Pattern& Closest = ClosestPattern(Memory,road);

  dat.speed = Closest.Speed();
  
  dat.drivelane = Closest.DriveLane();
  dat.entrylane = Closest.EntryLane();

  dat.speedcross = Closest.SpeedCrossover();
  dat.lanecross = Closest.LaneCrossover();
}

                         
const DriveData& DougE1Track::Segment(int i)
{
  // Control wrap around of the segment number
  if(i>=Segments)
	 i -= Segments;

  return *(Drive[i]);
}


//-----------------

static int init = 0;
static con_vec BasicInit()
{
  con_vec result;
  my_name_is("DougE1");
  result.alpha = result.vc = 0;

  init = 1;
  return result;
}


static void SpeedAggressiveControl(double factor, double& Speed)
{
  if((factor<1.0)&&(factor>0.1))
    Speed *= factor;
}


static void LaneAggressiveControl(situation& s, double factor, double& Lane)
{
  if((factor<1.0)&&(factor>0.1))
    Lane =  (Width(s)/2.) + (Lane-(Width(s)/2.))*fabs(factor*factor);
}


static int InLaneExit = 1;
static int Bulldozing = 0;


#define min(x,y) (((x)>(y))?(y):(x))
#define max(x,y) (((x)<(y))?(y):(x))


static void AvoidCars(situation &s, double& speed, double &lane)
{
  int BrakeCountLimit = max(DougE1Track::TheTrack.NumSegments()/2,5);
  
  static int BrakingCount[MAX_CARS];
  static int BrakingSegment[MAX_CARS];
  static int BrakingLaps[MAX_CARS];

  const int BulldozingTimeLimit = 5;
  
  static int BulldozingCount[MAX_CARS];

  Bulldozing = 0;
  
  // Initially set all approach/braking to standard
  static int BrakingCountFilled = 0;
  if((BrakingCountFilled==0)||(s.starting))
  {
    for(int i=0;i<MAX_CARS;i++)
    {
      BrakingCount[i] = 0;
      BrakingSegment[i] = 0;
      BrakingLaps[i] = 0;
      BulldozingCount[i] = 0;
    }
	 BrakingCountFilled = 1;
  }

  const char* def = "None";
  const char* mess = def;

  int speeddanger = 0;
  int lanedanger = 0;
  int slambrakes = 0;
  
  double Dx[4], Dy[4], Vx[4], Vy[4];

  Dx[0] = Dy[0] = Vx[0] = Vy[4] = 0.;

  int count = 0;
  int who[4];
  
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

    // Figure out if we might be bulldozing
	 if((fabs(y)<8)&&(fabs(x)<12))
    {
      if(BulldozingCount[state.who]<(BulldozingTimeLimit+2))
        BulldozingCount[state.who]++;
    }
    else if(BulldozingCount[state.who]>0)
    {
      BulldozingCount[state.who]--;
    }

    if(BulldozingCount[state.who]>BulldozingTimeLimit)
		Bulldozing = 1;

    // Ignore faster cars
    if(dot > -0.1)
       continue;

    // Ignore cars behind us
    if(y<0.)
    {
      mess = "Behind";
      continue;
    }

    // Figure out what speed to approach the car
    double Approach = 0.;
    if(BrakingCount[who[i]]>BrakeCountLimit)
      Approach += (BrakingCount[who[i]]-BrakeCountLimit)*2.;

    // Check whether you need to brake
    const double BrakeDistance = 1.5*CriticalDistance(s.v,s.v+vy+Approach);
    if(d>BrakeDistance)
    {
      mess = "Distance";
      continue;
	 }
    
    // Figure out how many feet we can travel in a short time
    const double ShortDist = s.v * 2.;

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
      if(fabs(x_close) > 1.5 * CARWID || fabs(y_close) > 1.25 * CARLEN)
//      if(fabs(x_close) > 3 * CARWID || fabs(y_close) > 2.5 * CARLEN)
      {
        mess = "Close miss";
        continue;
      }
    }

    // Are we going to crash hard
    double CrashDist = 2*CARLEN;
    if(state.braking)
      CrashDist *= 2;
      
    if((fabs(x) < 1.5 * CARWID)&&
       (fabs(y) < 1.25 * CARLEN)&&
       (y<CrashDist))
    {
      mess = "Slam brakes";
        
      ++slambrakes;
    
#ifdef DATA_DISPLAY
      sound(1000);
      rest(5);
      nosound();
#endif
    }
    
    // Are we close enough that we should just floor it
    if((y<1.5*CARLEN)&&(fabs(x)>1.5*CARWID))
    {
      mess = "Too Close";
      continue;
    }

    // Ignore cars that are on the left or right and are going away
    if((x>CARWID)&&(vx>1.)&&(vy<20.))
    {
      mess = "Right+Vel";
      continue;
    }
    if((x<-CARWID)&&(vx<1.)&&(vy<20.))
    {
		mess = "Left+Vel";
      continue;
    }

    // if slow car ahead, reserve more safety space
    if(fabs(x)>1.5*CARWID && x*vx>-10 &&
       fabs(vx)>10 && vy>-10)
    {
      mess = "Apex slow";
      continue;
    }

    // if fast car ahead, reserve less safety space
    if(fabs(x)>CARWID && x*vx>-10 &&
	    fabs(vx)>10 && vy<=-10)
    {
      mess = "Apex fast";
      continue;
    }

    ++speeddanger;
    ++lanedanger;

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

#ifdef DATA_DISPLAY
  double Approach = 0.;
#endif

  double newspeed = speed;
  if(speeddanger)
  {
	 // Figure out what speed to approach the first car
	 double App0 = 5.;
	 if(Dy[0]<5*CARLEN)
		App0 = 0;

	 if(BrakingCount[who[0]]>BrakeCountLimit)
		App0 += (BrakingCount[who[0]]-BrakeCountLimit)*2.;

#ifdef DATA_DISPLAY
	 Approach = App0;
#endif

    if(speeddanger>1)
    {
		// Figure out what speed to approach the second car
      double App1 = 5.;
      if(Dy[1]<5*CARLEN)
        App1 = 0;
      
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

  // Possibly change lanes to get of someones way
  static double extralane = 0.;
  if(((lanedanger)&&(!s.cur_rad)&&(InLaneExit==0))
	  ||(Bulldozing==1))
  {
    // Car is on the left
    if(Dx[0]<0)
    {
      // Is there room on the right - move right
      if(lane + extralane + Dx[0] < Width(s) - 1.5*CARWID)
        extralane += 0.5;
      // There must be room on the left - move left
      else
        extralane -= 0.5;
    }
    // Car is on the right
	 else if(Dx[0]>=0)
    {
      // Is there room on the left - move left
      if(lane + extralane + Dx[0] > 1.5*CARWID)
        extralane -= 0.5;
      // There must be room on the right - move right
      else
        extralane += 0.5;
    }
  }
  else
  {
    if(extralane!=0)
    {
      if(extralane>1.)
        extralane--;
      else if(extralane<-1.)
        extralane++;
      else
        extralane = 0.;
    }
  }
  // Don't change lanes if we are swinging around
  if((fabs(s.vn)>5.)&&(Bulldozing==0))
    extralane = 0.;
    
  lane += extralane;

  // Figure out the the speed
  newspeed = max(newspeed,5);
  if(newspeed<speed)
  {
    SpeedAggressiveControl(newspeed/speed,speed);
    LaneAggressiveControl(s,newspeed/speed,lane);

	 mess = "Braking";
  }

#ifdef DATA_DISPLAY
  if((!no_display)&&(Dx[0]!=0.)&&(Dy[0]!=0.))
  {
    char mess[1000];
    sprintf(mess,"Fac: %8.3f ExLane: %8.3f Approach0: %f ",
      (float)(newspeed/speed),
      (float)extralane,
      Approach);
      
    text_mode(makecol(0,0,0));
    textout(screen,font,mess,10,20,makecol(255,255,255));
    
    sprintf(mess,"x: %5.2f y: %5.2f vx: %5.2f vy: %5.2f v: %5.2f ",(float)Dx[0],
                                                          (float)Dy[0],
                                                          (float)Vx[0],
                                                          (float)Vy[0],
                                                          sqrt(Vx[0]*Vx[0] + Vy[0]*Vy[0]));
    textout(screen,font,mess,10,30,makecol(255,255,255));
  }

  if(!no_display)
  {
    char data[100];
    sprintf(data,"%s   ",mess);
    textout(screen,font,data,10,40,makecol(255,255,255));

    static const char* las = def;
    las = mess;
  }
#endif

}


static const double BrakeAccel      = 0.935; // tire speed ratio when braking


// Variables that are filled in by the driving code
static double FuelPerLap   = 5.0;
static double DamagePerLap = 100.0;
static double LastFuel = 0.;
static double TotalFuelUsed = 0.;

static int FirstSection = -1;
static double StartGridLane = 0.;
static double StartGridLength = 0.;

static unsigned long LastDamage = 0;
static unsigned long LastLapDamage = 0;
static unsigned long TotalDamage = 0;


con_vec BasicControl(situation& s, double Speed, double Lane, double Bias)
{
  con_vec result;

  // service routine in the host software to handle
  // getting unstuck from from crashes and pileups
  if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt,&result.alpha,&result.vc))
    return result;

  // Avoid going outside the limits of the track
  // by limiting the aggressiveness when we get to the edge
  // It has to be very aggressive so that other cars won't
  // push us off the outside edge of the track
  const double LaneLimit = 1.;
  if((s.to_lft<LaneLimit)||(s.to_rgt<LaneLimit))
  {
    SpeedAggressiveControl(0.5,Speed);
    LaneAggressiveControl(s,0.1,Lane);
  }

  // Avoid othre cars if they are on the road
  if((s.stage!=QUALIFYING)&&(s.stage!=PRACTICE))
    AvoidCars(s,Speed,Lane);

  // The complete steering servo
  const double SteeringGain = 1.0;
  const double SteeringDamp = SteeringGain*2.5/1.5;
  result.alpha = SteeringGain * (s.to_lft - Lane)/Width(s) - SteeringDamp * s.vn/s.v + Bias;

  // Work the gas pedal
  if(s.v > 1.02 * Speed)             // if we're 2% too fast,
    result.vc = BrakeAccel * s.v;    // brake hard.
  else if(s.v < .98 * Speed)         // if we're 2% too slow,
    result.vc = s.v + 50;            // accelerate hard.
  else                               // if we are very close to speed,
    result.vc = .5 * (s.v + Speed);  // approach the speed gently.

  // Figure out how much fuel we will need if we pit
  result.fuel_amount = FuelPerLap*s.laps_to_go*1.5;
  if(result.fuel_amount>MAX_FUEL)
    result.fuel_amount = MAX_FUEL;
    
  // Figure out how much damage to get repaired which is
  // three times the average damage per lap, except if we have
  // more than 20 laps to go, then repair everything
  result.repair_amount = (long)((DamagePerLap+100)*s.laps_to_go*3);
  if((result.repair_amount>(long)s.damage)||(s.laps_to_go>20))
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
      result.fuel_amount = 30;
    else
      result.fuel_amount = MAX_FUEL;

#ifdef DATA_DISPLAY
  if(!no_display)
  {
    char mess[100];
    sprintf(mess,"T: %6.3f  F: %6.3f D: %6f",(float)s.lap_time,(float)s.fuel,(float)s.damage);
    text_mode(makecol(0,0,0));
    textout(screen,font,mess,0,0,makecol(255,255,255));
  }
#endif

  return result;
}


static double CalcBias(const double radius, const double length, const double speed)
{
  // Bias for straight is zero
  if(radius == 0.0)
    return 0.;

  // Bias calculations from tutorial car
  double bias = 0.;
  if(speed>0.1)
    bias = atan(16.0/speed);
  if(radius<0.0)
    bias = -bias;

  // Bias gets higher ratio for curves with greater curve radius
  // Which stops huge bias for very small radius curves that only
  // curve a few degrees
  double ratio = length/3.1415;
  if(ratio>1.0)
    ratio = 1.0;
  bias *= ratio;

  return bias;
}


static double AdjustBias(situation &s, const double speed, double bias)
{
  // Adjust for actual speed
  if(speed>0.1)
    bias *= (s.v*s.v/(speed*speed));

  // Adjust for position in the lane
  if(s.to_lft<CARWID)
    if(bias>0)
      bias *= s.to_lft/CARWID;

  if(s.to_rgt<CARWID)
    if(bias<0)
      bias *= s.to_rgt/CARWID;
      
  return bias;
}


con_vec DougE1(situation &s)
{

#ifdef DATA_DISPLAY
  static int DisplayDelay = 10;

  int key = 0;
  if(kbhit())
    key = getch();

  switch(key)
  {
    case('s'):
    case('S'):
      if(SlowDisplay)
		  SlowDisplay = false;
      else
		  SlowDisplay = true;
      break;
    case('f'):
      DisplayDelay--;
      break;
    case('F'):
      DisplayDelay++;
      break;
    default:
      break;
  }

  if(SlowDisplay)
    rest(DisplayDelay);
#endif

  s.side_vision = 1;

  // Basic initializing stuff
  if(init==0)
    return BasicInit();

  if((s.starting)&&(get_track_description().NSEG>0))
  {
    DougE1Track::TheTrack.Initialize("douge1.dat");

    FuelPerLap = 5.0;
    LastFuel = 0.;
    TotalFuelUsed = 0.;
    
    FirstSection = -1;
    StartGridLane = 0.;
    StartGridLength = 0.;
    
    LastDamage = 0;
    LastLapDamage = 0;
    TotalDamage = 0;
  }

  //---------------------
  // Remember if this is the first secion of a new track
  int StartOnsetSection = 0;
  
  if((s.starting)&&(s.seg_ID==0))
	 StartOnsetSection = 1;

  if(s.out_pits==1)
	 StartOnsetSection = 1;
  
  if(StartOnsetSection==1)
  {
    FirstSection = s.seg_ID;
    StartGridLane = s.to_lft;
    StartGridLength = s.to_end;
  }
  
  if(s.seg_ID!=FirstSection)
    FirstSection = -1;

  //---------------------
  // Gather segment data when we cross into a new segment
  static double SegmentLength = 0.;
  static double ThisBias = 0.;
  static double NextBias = 0.;

  // Get the drive data about the track at this segment
  const DriveData& ThisRoad = DougE1Track::TheTrack.Segment(s.seg_ID);
  const DriveData& NextRoad = DougE1Track::TheTrack.Segment(s.seg_ID+1);
  
  // Update the segment information at every segment change
  static int LastSeg_ID = -1;
  if(s.seg_ID!=LastSeg_ID)
  {
    SegmentLength = Length(s.cur_rad,s.cur_len);

    ThisBias = CalcBias(s.cur_rad,s.cur_len,ThisRoad.speed);
    NextBias = CalcBias(s.nex_rad,s.nex_len,NextRoad.speed);

    LastSeg_ID = s.seg_ID;
  }
  
  //---------------------
  // Figure out the speed etc for this section
  static double speed = 50.;
  if(EndLength(s)<ThisRoad.speedcross)
  {
    const double ratio = EndLength(s)/ThisRoad.speedcross;
    speed = ratio*ThisRoad.speed + (1.0-ratio)*NextRoad.speed;
  }
  else
    speed = ThisRoad.speed;

  //---------------------
  // Figure out the lane and bias
  static double lane = 15.;
  static double bias = 0.;
  
  if(EndLength(s)<ThisRoad.lanecross)
  {
    const double ratio = EndLength(s)/ThisRoad.lanecross;

    lane = ratio*ThisRoad.drivelane + (1.0-ratio)*NextRoad.entrylane;
    bias = ratio*ThisBias + (1.0-ratio)*NextBias;
  }
  // We are in the entering portion where we change lanes
  else
  {
    const double ratio = (EndLength(s)-ThisRoad.lanecross)/(SegmentLength-ThisRoad.lanecross);

    lane = ratio*ThisRoad.entrylane + (1.0-ratio)*ThisRoad.drivelane;
    bias = ThisBias;
    
    // In the dragout portion of the start
    if(FirstSection>=0)
    {
      const double factor = EndLength(s)/StartGridLength;
      const double differ = ThisRoad.drivelane-StartGridLane;
      
      const double inlane = ThisRoad.drivelane - factor*differ;

      if((inlane>0)&&(inlane<Width(s)))
        lane = inlane;
    }
  }

  // Remember if we are going to go into the lane exit
  if(EndLength(s)<5.*ThisRoad.speedcross)
    InLaneExit = 1;
  else
	 InLaneExit = 0;
    
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
  {
    LastLapDamage = 0;

//    cout << "Pos: " << s.position << endl;
  }

  //---------------------
  // Adjust given bias for actual speed
  bias = AdjustBias(s,speed,bias);
  
  // Basic control loop
  return BasicControl(s,speed,lane,bias);
}



