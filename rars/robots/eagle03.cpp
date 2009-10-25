// Eagle03.CPP - "driver" function for RARS v0.67
// by Jerry Donovan, Mar 1998

////////////////////////////////////////////////////////////
// Comments:
//
// This car will become public if it ever wins a race, any race.
// Until then, why bother, it has been changing a lot, and if
// it doesn't win, then it will change a lot more.
//
// ... A speaking of that, I'll climb up on the soapbox ...
//
// I think that any car that wins a race should automatically 
// be made public in the spirit of furthering Rars and that 
// being the price of winning.  It would at least give a better
// set of drivers to practice against.
//
// ... Now stepping back down off the soapbox ...
//
// My goals in writing this car are that I wanted to write
// a robot car driver that:
// 1. is different than most of the others
// 2. is competitive, and wins some races
// 3. does not require special tuning for each different track.
//
// It is getting close to accomplishing those goals.  It is different 
// (although since doing most of this code, I have found that others 
// have tried the same or a similar approach) and is pretty fast, but 
// hasn't interacted well with the other cars in a past races.  I am 
// aware of a few things that still need to be changed to make it 
// general for all tracks.  While this code is finally starting to 
// work on more than just the track(s) of the current race, it still 
// has problems with about half of the tracks.
//
// This code is written in the spirit of the DYNAMIC car, such that 
// multiple drivers can be run from this one piece of code. This 
// has it's pluses and minuses.  (For example, identical cars have 
// more trouble passing each other.)
//
// As for how this car works...
// 1. It attempts to calculate a set of optimum curves in global 
//    coordinates for the complete track, and fiddles with them until 
//    they do not overlap.  The paths are the inner most position the 
//    car should ever be on a curve.  Curves are calculated for the 
//    best path, as well as a couple passing paths, and a safer best 
//    path that doesn't get so close to the walls.
// 2. It tries to follow those curves by determining when it should
//    leave the current curve and aim toward the tangent of the next 
//    curve.  This is like driving on a map instead of watching where 
//    you are on the track.  A pure implementation of this model would
//    not ever consider where the edge of the track is, since the 
//    optimum course already has those figured in.
// 3. Then it adds in trying to get past other cars while still figuring 
//    out where it is in global and track coordinate.  This is the real
//    trick for this car.  It must be able to make deviations from the
//    optimum and not run off the track when the course is modified.
// 4. Then there is all the normal watching of damage and fuel and such.
//
// The fact that this file now has these comments probably indicates
// that I think the car might even win a race soon.  
// Perhaps I am too much of an optimist.  :-)
//
// Jerry
//
////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////

// the next line is for use in an MSVC++ MFC application
//#include "stdafx.h"

#include "math.h"        // trig functions, fabs(), etc.
#include "car.h"         // required for car
#include "track.h"       // required for track stuff
#include "os.h"          // delta_time

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif




////////////////////////////////////////////////////////////
// EXTERNAL VARIABLES
////////////////////////////////////////////////////////////

// CARZ.CPP  variables
extern int NSEG;
extern Car* pcar[];
extern int car_count;

// TRACK.CPP  variables
extern double width;

// OS.H variables
extern const double delta_time;




////////////////////////////////////////////////////////////
// TYPEDEFS
////////////////////////////////////////////////////////////

typedef struct 
{
	double PosX;
	double PosY;
	double Speed;
	double Angle;
} CARINFO;


typedef struct 
{
	int    Seg;
	int    Path;
	int    Relation;
	int    OnCurve;
	double Dist;
	double rDist;
	double bDist;
	double Angle;
	double Speed;
	double bSpeed;
	double Effort;
} CURVEINFO;


typedef struct
{
	int Laps;
	double Time;
	double Speed;
	double TimePerLap;
	double Behind;
} OTHERCARS;



////////////////////////////////////////////////////////////
// GLOBALS
////////////////////////////////////////////////////////////

// note this must follow the convention of right turns are negative, left turns positive
enum SegmentType   { RIGHTTURN = -1, STRAIGHT, LEFTTURN };
enum CurvePath     { NOPATH = -1, CENTER, INSIDE, BEST, PASS1, PASS2, SAFE };
enum CurveRelation { INSIDECURVE, ONCURVE, OUTSIDECURVE };

const char CarName[16][9] = {"Eagle 00", "Eagle 01", "Eagle 02", "Eagle 03", 
	    "Eagle 04", "Eagle 05", "Eagle 06", "Eagle 07", "Eagle 08", "Eagle 09", 
	    "Eagle 10", "Eagle 11", "Eagle 12", "Eagle 13", "Eagle 14", "Eagle 15" };
const char CurveName[3][10] = {"RIGHTTURN", "STRAIGHT", "LEFTTURN" };
const char PathName[6][7] = {"CENTER", "INSIDE", "BEST", "PASS1", "PASS2", "SAFE" };
const char RelationName[3][13] = {"INSIDECURVE", "ONCURVE", "OUTSIDECURVE" };

char Eagles[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double XEagleDebug;


//const double PI        = 3.14159265359;
const double TWOPI       = 2*PI;
const double DEGRAD      = PI/180;
const int CURVEPATHS     = 6;
const int STRAIGHTPATHS  = 1;

const int DEBUGCAR        = 1;
const int DEBUGCURVE      = 2;
const int DEBUGTRACK      = 4;
const int DEBUGINST       = 8;
const int DEBUGTRAFFIC    = 16;
const int DEBUGLAP        = 32;
const int DEBUGTRUNC      = 64;
const int DEBUGRANDOMPATH = 128;
const int DEBUGMISC       = 256;

unsigned long    EagleDebug  = 0L;

typedef struct
{
	double Outside;
	double Inside;
	double MaxSpeed;
	double MaxBrake;
	double MaxAccel;
	double CornerSpeed;
	double CornerBrake;
	double Bias;
	double MaxRadius;
	double SafeMargin;
	int    Traffic;
} EAGLEGLOBALS;

EAGLEGLOBALS EG03 = {8.0, 2.0, 999.0, 34.0, 34.0, 5.8, 17.0, 1.0, 1300.0, 13.0, 4};






////////////////////////////////////////////////////////////
// Eagle_Segment PROTOTPE
////////////////////////////////////////////////////////////

class Eagle_Segment
{
private:
protected:
	int Type;
	segment *Curves;
	double *Speeds;
	double *Bias;
	double *ClosestPoint;
	double *HalfAlpha;
	double *MidAdjust;
	double *OutsideAdjust;
	double *InsideAdjust;
	double *AdjustAmount;

public:
	Eagle_Segment(segment &TrackLeft, segment &TrackRight);
	~Eagle_Segment();

	inline double GetType(){ return Type;};
	inline double GetCurveSpeed(int path){return Speeds[path];};
	inline double GetCurveBias(int path){return Bias[path];};
	inline double GetClosestPoint(int path){ return ClosestPoint[path];};
	inline double GetHalfAlpha(int path){ return HalfAlpha[path];};
	inline double GetBegAngle(int path){ return Curves[path].beg_ang;};
	inline double GetEndAngle(int path){ return Curves[path].end_ang;};
	inline double GetRadius(int path){ return Curves[path].radius;};
	inline double GetCenX(int path){ return Curves[path].cen_x;};
	inline double GetCenY(int path){ return Curves[path].cen_y;};
	inline double GetBegX(int path){ return Curves[path].beg_x;};
	inline double GetBegY(int path){ return Curves[path].beg_y;};
	inline double GetEndX(int path){ return Curves[path].end_x;};
	inline double GetEndY(int path){ return Curves[path].end_y;};
	inline double GetLength(int path){ return Curves[path].length;};

	double CalculateSpeed(int path);
	void CalculateCurve(int Path, double WhichWay);
	int FixSegmentOverlaps(Eagle_Segment *PrevCurve, Eagle_Segment *NextCurve, double PrevStr, double NextStr);
	double CurveTangentAngle(int path, double x, double y);
	double CurveTangentDistance(int path, double x, double y);
	double GetWorldX(int path, double to_center, double to_end);
	double GetWorldY(int path, double to_center, double to_end);
	double GetWorldDir(double v, double vn, double to_end);
	int GetCurveRelation( int path, double x, double y );
	double OffCurveDist( int path, double x, double y );
};





////////////////////////////////////////////////////////////
// Eagle_Track PROTOTPE
////////////////////////////////////////////////////////////

class Eagle_Track
{
private:
	Eagle_Segment **Trk;	

public:
	Eagle_Track();
	~Eagle_Track();

	inline double GetCurveBias(int seg, int path){return Trk[seg]->GetCurveBias(path);};
	inline double GetRadius(int seg, int path){return Trk[seg]->GetRadius(path);};
	inline double GetType(int seg){ return Trk[seg]->GetType();};

	double BrakingDistance(CURVEINFO Curve1, CURVEINFO Curve2);
	void GetWorldPos(int seg, double to_center, double to_end, double &x, double &y);
	double GetWorldDir(int seg, double v, double vn, double end);
	int GetNextCurve(int seg);
	int GetPrevCurve(int seg);
	int GetCurveRelation( int seg, int path, double x, double y );
	double CurveTangentAngle(int seg, int path, double x, double y);
	double CurveTangentDistance(int seg, int path, double x, double y);
	double GetCurveSpeed(int seg, int path);
	void GetCurveInfo( int CurrentSegment, CARINFO &CarInfo, CURVEINFO &CurveInfo );
};






////////////////////////////////////////////////////////////
// Eagle_Car PROTOTPE
////////////////////////////////////////////////////////////

class Eagle_Car
{
private:
	Eagle_Track *Road;
	CARINFO    CarInfo;
	CURVEINFO  Curve[3];
	CURVEINFO  OldCurve;
	int CurrentSegment;
	int CurrentPath;
	double ToEnd;
	double ToCenter;
	int TimeToPit;    
	int NoPit;
	double MaxFuel;
	int HeavyTrafficCount;
	double Effort;
	int FuelStopsRequired;
	double FPP;
	double FuelPerLap;
	int LastLapsToGo;
	int LapsTraveled;
	int LapsInTank;
	int CourseChangedInSegment;
	unsigned long LastDamage;
	unsigned long TotalDamage;
	unsigned long DamagePerLap;
	unsigned long DamageLastLap;
	unsigned long DamageMaxLap;
	unsigned long DamageAllowed;
	int AngleAdjustment;
	int CarStuck;
	int ChangedLaps;
	OTHERCARS Car[16];
	double OtherBestTime;
	int OtherBestCar;

	double DistanceTraveled;
	double LastDistX;
	double LastPosX;
	double LastPosY;
	double FuelUsed;
	double LastFuel;

	double SpeedChange;
	double AngleChange;
	double BrakeAmount;

	void WhereAreWeGoing(void);
	double WhichWay(void);
	void ChangeCurve(int CurveNumber);
	void ChangePath(int NewPath);
	void CheckTraffic(situation &s);
	void WhatsHappening(situation &s);
	void ResetCurves(void);
	void CheckOtherCars(situation &s);

public:
	Eagle_Car();
	~Eagle_Car();

	void Drive(situation &s, con_vec &result);
	void IsStuck(situation &s);
};






////////////////////////////////////////////////////////////
// GLOBAL FUNCTIONS
////////////////////////////////////////////////////////////

// returns the maximum of two numbers
inline static double _MAX_( double a, double b )
{
	if (a>b)
		return a;
	return b;
}

// returns the minimum of two numbers
inline static double _MIN_( double a, double b )
{
	if (a<b)
		return a;
	return b;
}

// returns the square of a number
inline static double _SQR_( double a )
{
	return a*a;
}

// returns 1 for positive numbers (and zero) and -1 for negative numbers
inline static int _SIGN_( double a )
{
	if (a<0.0)
		return -1;
	return 1;
}

// returns the number of the next segment
inline static int _NEXTSEG_(int s)
{
	if (s >= NSEG-1)
		return 0;
	return s+1;
}

// returns the number of the previous segment
inline static int _PREVSEG_(int s)
{
	if (s <= 0)
		return NSEG-1;
	return s-1;
}

// returns 1 if the two numbers are almost equal
inline static int _ALMOST_( double a, double b, double d)
{
	if ((a<=b+d) && (a>=b-d))
		return 1;
	return 0;
}

// returns the number (angle) normalized to between -PI to +PI
static double _NORM_(double a)
{
	while (a>=PI)
		a-=TWOPI;
	while (a<=-PI)
		a+=TWOPI;

	return a;
}

// returns absolute angle between (x1,y1) and (x2,y2)
inline static double _ANGLE_(double x1, double y1, double x2, double y2)
{
	return _NORM_(atan2(((y2)-(y1)),((x2)-(x1))));
}

// returns distance between (x1,y1) and (x2,y2)
inline static double _DIST_(double x1, double y1, double x2, double y2)
{
	return sqrt(_SQR_((x2)-(x1))+_SQR_((y2)-(y1)));
}

// returns angle between tangent of circle and center (cx,cy), radius r, from point (x,y)
inline static double _TANGENTANGLE_(double x, double y, double cx, double cy, double r)
{
   return _NORM_(asin((r)/_MAX_(_DIST_((x),(y),(cx),(cy)),fabs(r))));
}

// returns absolute angle to circle tangent center (cx,cy), radius r, from point (x,y)
inline static double _ABSTANANGLE_(double x, double y, double cx, double cy, double r)
{
	return _NORM_(_ANGLE_((x),(y),(cx),(cy))-_TANGENTANGLE_((x),(y),(cx),(cy),(r)));
}

//returns distance to circle tangent center (cx,cy), radius r, from point (x,y)
inline static double _TANGENTDIST_(double x, double y, double cx, double cy, double r)
{
	return sqrt(_SQR_(_MAX_(_DIST_((x),(y),(cx),(cy)),fabs(r)))-_SQR_(r));
}

// returns the conversion of degrees to radians
inline static double _DTR_(double deg )
{
	return deg * DEGRAD;
}



////////////////////////////////////////////////////////////
// Eagle_Segment FUNCTIONS
////////////////////////////////////////////////////////////

// setup a new track segment
Eagle_Segment::Eagle_Segment(segment &TrackLeft, segment &TrackRight)
{
	if (TrackLeft.radius == 0.0)
	{
		Curves = new segment[STRAIGHTPATHS];
		Speeds = new double[STRAIGHTPATHS];
		Bias = new double[STRAIGHTPATHS];
		Type = STRAIGHT;
	}
	else
	{
		Curves = new segment[CURVEPATHS];
		Speeds = new double[CURVEPATHS];
		Bias = new double[CURVEPATHS];
		MidAdjust = new double[CURVEPATHS];
		ClosestPoint = new double[CURVEPATHS];
		HalfAlpha = new double[CURVEPATHS];
		OutsideAdjust = new double[CURVEPATHS];
		InsideAdjust = new double[CURVEPATHS];
		AdjustAmount = new double[CURVEPATHS];


		if (TrackLeft.radius > 0.0)
			Type = LEFTTURN;
		else
			Type = RIGHTTURN;

		MidAdjust[CENTER] = 0.0;
		OutsideAdjust[CENTER] = 0.0;
		InsideAdjust[CENTER] = 0.0;
		AdjustAmount[CENTER] = 0.0;
	}

	Curves[CENTER] = TrackLeft;
	Curves[CENTER].radius  = (TrackLeft.radius + TrackRight.radius) / 2;
	Curves[CENTER].beg_x   = (TrackLeft.beg_x + TrackRight.beg_x) / 2;
	Curves[CENTER].beg_y   = (TrackLeft.beg_y + TrackRight.beg_y) / 2;
	Curves[CENTER].end_x   = (TrackLeft.end_x + TrackRight.end_x) / 2;
	Curves[CENTER].end_y   = (TrackLeft.end_y + TrackRight.end_y) / 2;
	Curves[CENTER].beg_ang = _NORM_(TrackLeft.beg_ang);
	Curves[CENTER].end_ang = _NORM_(TrackLeft.end_ang);
	Speeds[CENTER] = CalculateSpeed(CENTER);
	Bias[CENTER] = Curves[CENTER].radius * 2.0 * PI / delta_time;


	if (Type == STRAIGHT)
		return;

	HalfAlpha[CENTER] = Curves[CENTER].length/2;
	ClosestPoint[CENTER] = Curves[CENTER].beg_ang + Type * HalfAlpha[CENTER];

	MidAdjust[INSIDE] = 0.0;
	OutsideAdjust[INSIDE] = 0.0;
	InsideAdjust[INSIDE] = 0.0;
	AdjustAmount[INSIDE] = 0.0;
	Curves[INSIDE] = Curves[CENTER];
	HalfAlpha[INSIDE] = Curves[INSIDE].length/2;
	ClosestPoint[INSIDE] = Curves[INSIDE].beg_ang + Type * HalfAlpha[INSIDE];
	Curves[INSIDE].radius  = Curves[CENTER].radius - Type*((width/2)-EG03.Inside);
	Curves[INSIDE].beg_x   = Curves[CENTER].cen_x + 
			Curves[INSIDE].radius * sin(Curves[INSIDE].beg_ang);
	Curves[INSIDE].beg_y   = Curves[INSIDE].cen_y -
			Curves[INSIDE].radius * cos(Curves[INSIDE].beg_ang);
	Curves[INSIDE].end_x   = Curves[INSIDE].cen_x + 
			Curves[INSIDE].radius * sin(Curves[INSIDE].end_ang);
	Curves[INSIDE].end_y   = Curves[INSIDE].cen_y -
			Curves[INSIDE].radius * cos(Curves[INSIDE].end_ang);
	Speeds[INSIDE] = CalculateSpeed(INSIDE);
	Bias[INSIDE] = Curves[INSIDE].radius * 2.0 * PI / delta_time;

	MidAdjust[BEST]   = 0.0;
	OutsideAdjust[BEST]   = 0.0;
	InsideAdjust[BEST]   = 0.0;
	AdjustAmount[BEST]   = (width - EG03.Outside - EG03.Inside)/10;

	HalfAlpha[BEST] = Curves[INSIDE].length/2;
	ClosestPoint[BEST] = Curves[INSIDE].beg_ang + Type * HalfAlpha[BEST];
	CalculateCurve(BEST, 0.0);

	MidAdjust[PASS1]  = 0.0;
	OutsideAdjust[PASS1]  = EG03.SafeMargin;
	InsideAdjust[PASS1]  = EG03.SafeMargin;
	AdjustAmount[PASS1]  = (width - EG03.Outside - EG03.Inside - EG03.SafeMargin*2)/10;

	HalfAlpha[PASS1] = Curves[INSIDE].length;
	ClosestPoint[PASS1] = Curves[INSIDE].beg_ang;
	CalculateCurve(PASS1, 0.0);

	MidAdjust[PASS2]  = 0.0;
	OutsideAdjust[PASS2]  = EG03.SafeMargin;
	InsideAdjust[PASS2]  = EG03.SafeMargin;
	AdjustAmount[PASS2]  = (width - EG03.Outside - EG03.Inside - EG03.SafeMargin*2)/10;

	HalfAlpha[PASS2] = Curves[INSIDE].length;
	ClosestPoint[PASS2] = Curves[INSIDE].end_ang;
	CalculateCurve(PASS2, 0.0);

	MidAdjust[SAFE]   = 0.0;
	OutsideAdjust[SAFE]   = EG03.SafeMargin;
	InsideAdjust[SAFE]   = EG03.SafeMargin;
	AdjustAmount[SAFE]   = (width - EG03.Outside - EG03.Inside - EG03.SafeMargin*2)/10;

	HalfAlpha[SAFE] = Curves[INSIDE].length/2;
	ClosestPoint[SAFE] = Curves[INSIDE].beg_ang + Type * HalfAlpha[SAFE];
	CalculateCurve(SAFE, 0.0);

	return;
}


// cleanup the track segment
Eagle_Segment::~Eagle_Segment()
{
	delete [] Curves;
	delete [] Speeds;
	delete [] Bias;
	if (Type != STRAIGHT)
	{
		delete [] MidAdjust;
		delete [] ClosestPoint;
		delete [] HalfAlpha;
		delete [] OutsideAdjust;
		delete [] InsideAdjust;
		delete [] AdjustAmount;
	}
}


// calculate the desired speed for a track segment
double Eagle_Segment::CalculateSpeed(int path)
{
	if (Curves[path].radius==0)
		return EG03.MaxSpeed;
	else
		return EG03.CornerSpeed*sqrt(fabs(Curves[path].radius));
}


// Calculate the specified curve to fit various parameters
void Eagle_Segment::CalculateCurve(int Path, double AdjustDir)
{
	double AdjustX = sqrt(fabs(AdjustDir));

	if (AdjustDir)
	{
		if ((Path == PASS1) || (Path == PASS2))
			OutsideAdjust[Path] += AdjustAmount[Path];
		else if ((Path == BEST) || (Path == SAFE))
		{
			if (AdjustDir * MidAdjust[Path] < 0)
			{
				HalfAlpha[Path] -= AdjustX*Curves[Path].length/20;
				OutsideAdjust[Path] += AdjustX*AdjustAmount[Path];
			}
			else
			{
				HalfAlpha[Path] += AdjustX*Curves[Path].length/20;
			}

			MidAdjust[Path] += _SIGN_(AdjustDir)*AdjustX;
			ClosestPoint[Path] += _SIGN_(AdjustDir)*AdjustX*Type*Curves[Path].length/20;
		}
	}


	Curves[Path] = Curves[INSIDE];

	Curves[Path].radius = Curves[INSIDE].radius + InsideAdjust[Path] +
	   (Type * (width - EG03.Outside - EG03.Inside - InsideAdjust[Path] - OutsideAdjust[Path])
	      /(1.0-cos(HalfAlpha[Path]))); 

	if (fabs(Curves[Path].radius) > EG03.MaxRadius)
		Curves[Path].radius = Type * EG03.MaxRadius;

	if (fabs(Curves[Path].radius) < fabs(Curves[INSIDE].radius) + InsideAdjust[Path])
		Curves[Path].radius = Curves[INSIDE].radius + Type*InsideAdjust[Path];
	
	Curves[Path].cen_x = Curves[INSIDE].cen_x - 
		Type * (fabs(Curves[Path].radius) - 
		fabs(Curves[INSIDE].radius) - InsideAdjust[Path]) * sin(ClosestPoint[Path]);

	Curves[Path].cen_y = Curves[INSIDE].cen_y +
		Type * (fabs(Curves[Path].radius) - 
		fabs(Curves[INSIDE].radius) - InsideAdjust[Path]) * cos(ClosestPoint[Path]);

	Curves[Path].beg_x   = Curves[Path].cen_x + 
			Curves[Path].radius * sin(Curves[Path].beg_ang);
	Curves[Path].beg_y   = Curves[Path].cen_y -
			Curves[Path].radius * cos(Curves[Path].beg_ang);
	Curves[Path].end_x   = Curves[Path].cen_x + 
			Curves[Path].radius * sin(Curves[Path].end_ang);
	Curves[Path].end_y   = Curves[Path].cen_y -
			Curves[Path].radius * cos(Curves[Path].end_ang);
	Speeds[Path] = CalculateSpeed(Path);
	Bias[Path] = Curves[Path].radius * 2.0 * PI / delta_time;
}


// see if the current segment needs to be adjusted to not overlaps the previous or next curves
int Eagle_Segment::FixSegmentOverlaps(Eagle_Segment *PrevCurve, Eagle_Segment *NextCurve, double PrevStr, double NextStr)
{
	int Changes = 0;

	int PrevOpposite = (Type * PrevCurve->GetType() < 0) ? 1 : 0;
	int NextOpposite = (Type * NextCurve->GetType() < 0) ? 1 : 0;

	if (PrevOpposite && 
	    fabs(Curves[BEST].radius) > fabs(PrevCurve->GetRadius(BEST))){
	   if ((fabs(Curves[BEST].radius) + fabs(PrevCurve->GetRadius(BEST)) + EG03.SafeMargin ) > 
	       _DIST_(Curves[BEST].cen_x, Curves[BEST].cen_y, 
		      PrevCurve->GetCenX(BEST), PrevCurve->GetCenY(BEST))){
	      CalculateCurve(BEST, -1.0);
	      Changes++;
	   }
	   if ((fabs(Curves[SAFE].radius) + fabs(PrevCurve->GetRadius(BEST)) + EG03.SafeMargin ) > 
	       _DIST_(Curves[SAFE].cen_x, Curves[SAFE].cen_y, 
		      PrevCurve->GetCenX(BEST), PrevCurve->GetCenY(BEST))){
	      CalculateCurve(SAFE, -1.0);
	      Changes++;
	   }
	   if ((fabs(Curves[PASS2].radius) + fabs(PrevCurve->GetRadius(BEST)) + EG03.SafeMargin) > 
	       _DIST_(Curves[PASS2].cen_x, Curves[PASS2].cen_y, 
		      PrevCurve->GetCenX(BEST), PrevCurve->GetCenY(BEST))){
	      CalculateCurve(PASS2, -1.0);
	      Changes++;
	   }
	}

	if (NextOpposite && 
	    fabs(Curves[BEST].radius) > fabs(NextCurve->GetRadius(BEST))){
	  if ((fabs(Curves[BEST].radius) + fabs(NextCurve->GetRadius(BEST)) + EG03.SafeMargin) > 
	      _DIST_(Curves[BEST].cen_x, Curves[BEST].cen_y, 
		     NextCurve->GetCenX(BEST), NextCurve->GetCenY(BEST))){
	     CalculateCurve(BEST, 1.0);
	     Changes++;
	  }
	  if ((fabs(Curves[SAFE].radius) + fabs(NextCurve->GetRadius(BEST)) + EG03.SafeMargin) > 
	      _DIST_(Curves[SAFE].cen_x, Curves[SAFE].cen_y, 
		     NextCurve->GetCenX(BEST), NextCurve->GetCenY(BEST))){
	     CalculateCurve(SAFE, 1.0);
	     Changes++;
	  }
	  if ((fabs(Curves[PASS1].radius) + fabs(NextCurve->GetRadius(BEST)) +	EG03.SafeMargin ) > 
	      _DIST_(Curves[PASS1].cen_x, Curves[PASS1].cen_y, 
		     NextCurve->GetCenX(BEST), NextCurve->GetCenY(BEST))){
	     CalculateCurve(PASS1, 1.0);
	     Changes++;
	  }
	}
	
	if (!PrevOpposite){
	   double PrevGap = PrevStr - 
	     _DIST_(Curves[BEST].beg_x, Curves[BEST].beg_y, 
		    Curves[CENTER].beg_x, Curves[CENTER].beg_y ) -
	     _DIST_(PrevCurve->GetEndX(BEST), PrevCurve->GetEndY(BEST),
		    PrevCurve->GetEndX(CENTER), PrevCurve->GetEndY(CENTER));
	   if (PrevGap < 0.0){
	      if ((fabs(PrevCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(PrevCurve->GetRadius(INSIDE)) * PrevCurve->GetLength(INSIDE) > 
		   fabs(Curves[BEST].radius)))
		 CalculateCurve(BEST, -_MIN_(1.5*fabs(Curves[INSIDE].radius/PrevCurve->
						     GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(BEST, -1.0);
	      Changes++;
	   }

	   PrevGap = PrevStr - 
	     _DIST_(Curves[SAFE].beg_x, Curves[SAFE].beg_y, 
		    Curves[CENTER].beg_x, Curves[CENTER].beg_y ) -
	     _DIST_(PrevCurve->GetEndX(BEST), PrevCurve->GetEndY(BEST),
		    PrevCurve->GetEndX(CENTER), PrevCurve->GetEndY(CENTER));
	   
	   if (PrevGap < 0.0){
	      if ((fabs(PrevCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(PrevCurve->GetRadius(INSIDE)) * PrevCurve->GetLength(INSIDE) >
		   fabs(Curves[SAFE].radius)))
		CalculateCurve(SAFE, -_MIN_(1.5*fabs(Curves[INSIDE].radius/PrevCurve->
						     GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(SAFE, -1.0);
	      Changes++;
	   }

	   PrevGap = PrevStr - 
	     _DIST_(Curves[PASS2].beg_x, Curves[PASS2].beg_y, 
		    Curves[CENTER].beg_x, Curves[CENTER].beg_y ) -
	     _DIST_(PrevCurve->GetEndX(BEST), PrevCurve->GetEndY(BEST),
		    PrevCurve->GetEndX(CENTER), PrevCurve->GetEndY(CENTER));
	   
	   if (PrevGap < 0.0){
	      if ((fabs(PrevCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(PrevCurve->GetRadius(INSIDE)) * PrevCurve->GetLength(INSIDE) > 
		   fabs(Curves[PASS2].radius)))
		 CalculateCurve(PASS2, -_MIN_(1.5*fabs(Curves[INSIDE].radius/PrevCurve->
						       GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(PASS2, -1.0);
	      Changes++;
	   }
	}


	if (!NextOpposite){
	   double NextGap = NextStr - 
	    _DIST_(Curves[BEST].end_x, Curves[BEST].end_y, 
		   Curves[CENTER].end_x, Curves[CENTER].end_y ) -
	    _DIST_(NextCurve->GetBegX(BEST), NextCurve->GetBegY(BEST),
		   NextCurve->GetBegX(CENTER), NextCurve->GetBegY(CENTER));
	  
	   if (NextGap < 0.0){
	      if ((fabs(NextCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(NextCurve->GetRadius(INSIDE)) * NextCurve->GetLength(INSIDE) > 
		   fabs(Curves[BEST].radius)))
		 CalculateCurve(BEST, _MIN_(1.5*fabs(Curves[INSIDE].radius/NextCurve->
						     GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(BEST, 1.0);
	      Changes++;
	   }

	   NextGap = NextStr - 
	      _DIST_(Curves[SAFE].end_x, Curves[SAFE].end_y, 
		     Curves[CENTER].end_x, Curves[CENTER].end_y ) -
	      _DIST_(NextCurve->GetBegX(BEST), NextCurve->GetBegY(BEST),
		     NextCurve->GetBegX(CENTER), NextCurve->GetBegY(CENTER));

	   if (NextGap < 0.0){
	      if ((fabs(NextCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(NextCurve->GetRadius(INSIDE)) * NextCurve->GetLength(INSIDE) > 
		   fabs(Curves[SAFE].radius)))
		 CalculateCurve(SAFE, _MIN_(1.5*fabs(Curves[INSIDE].radius/NextCurve->
						     GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(SAFE, 1.0);
	      Changes++;
	   }
	   
	   NextGap = NextStr - 
	      _DIST_(Curves[PASS1].end_x, Curves[PASS1].end_y, 
		     Curves[CENTER].end_x, Curves[CENTER].end_y ) -
	      _DIST_(NextCurve->GetBegX(BEST), NextCurve->GetBegY(BEST),
		     NextCurve->GetBegX(CENTER), NextCurve->GetBegY(CENTER));

	   if (NextGap < 0.0){
	      if ((fabs(NextCurve->GetRadius(INSIDE)) > 0.5*EG03.MaxRadius) &&
		  (fabs(Curves[INSIDE].radius) < 0.3*EG03.MaxRadius) &&
		  (fabs(NextCurve->GetRadius(INSIDE)) * NextCurve->GetLength(INSIDE) >
		   fabs(Curves[PASS1].radius)))
		 CalculateCurve(PASS1, _MIN_(1.5*fabs(Curves[INSIDE].radius/NextCurve->
						      GetRadius(INSIDE)),1.0));
	      else
		 CalculateCurve(PASS1, 1);
	      Changes++;
	   }
	}

	return Changes;
}


// calculate the world X position of a point in this segment
double Eagle_Segment::GetWorldX(int path, double to_center, double to_end)
{
	if (Type == STRAIGHT)
		return Curves[path].end_x
				- sin(Curves[path].end_ang)*to_center
				- cos(Curves[path].end_ang)*to_end;
	else if (Type == LEFTTURN)
		return  Curves[path].cen_x + sin(Curves[path].end_ang - to_end) *
				(Curves[path].radius - to_center);
	else
		return  Curves[path].cen_x + sin(Curves[path].end_ang + to_end) *
				(Curves[path].radius - to_center);
}



// calculate the world Y position of a point in this segment
double Eagle_Segment::GetWorldY(int path, double to_center, double to_end)
{
	if (Type == STRAIGHT)
		return Curves[path].end_y
				+ cos(Curves[path].end_ang)*to_center
				- sin(Curves[path].end_ang)*to_end;
	else if (Type == LEFTTURN)
		return  Curves[path].cen_y - cos(Curves[path].end_ang - to_end) *
				(Curves[path].radius - to_center);
	else
		return  Curves[path].cen_y - cos(Curves[path].end_ang + to_end) *
				(Curves[path].radius - to_center);
}



// calculate the world direction of a vector in this segment
double Eagle_Segment::GetWorldDir(double v, double vn, double to_end)
{
	double angle;

	if (Type == STRAIGHT)
		angle = Curves[CENTER].end_ang;
	else if (Type == LEFTTURN)
		angle = Curves[CENTER].end_ang - to_end;
	else
		angle = Curves[CENTER].end_ang + to_end;

	double cos_ang = cos(angle);
	double sin_ang = sin(angle);

	return _NORM_(atan2(v * sin_ang + vn * cos_ang, v * cos_ang - vn * sin_ang));
}


// determine where the point is compared to a path 
int Eagle_Segment::GetCurveRelation( int path, double x, double y )
{
        double difference = _DIST_( x, y, Curves[path].cen_x, Curves[path].cen_y )-
	   fabs(Curves[path].radius);

	if (difference < 0)
		return INSIDECURVE;
	else if (difference > EG03.Outside/2)
		return OUTSIDECURVE;
	else
		return ONCURVE;
}



// calculate the angle to head toward a tangent to the path
inline double Eagle_Segment::CurveTangentAngle(int path, double x, double y)
{
	return _ABSTANANGLE_( x, y, 
		Curves[path].cen_x, Curves[path].cen_y, 
		Curves[path].radius);
}



// calculate the distance to the tangent point of a path
inline double Eagle_Segment::CurveTangentDistance(int path, double x, double y)
{
	return _TANGENTDIST_( x, y, 
		Curves[path].cen_x, Curves[path].cen_y,
		Curves[path].radius);
}


// determine the perpendicular distance from a point to a path
double Eagle_Segment::OffCurveDist( int path, double x, double y )
{
	return _DIST_( x, y, Curves[path].cen_x, Curves[path].cen_y) - fabs(Curves[path].radius);
}


////////////////////////////////////////////////////////////
// Eagle_Track FUNCTIONS
////////////////////////////////////////////////////////////
// initialize the track
Eagle_Track::Eagle_Track()
{
	int s;

	track_desc Track = get_track_description();
	Trk = new Eagle_Segment*[NSEG];

	for (s=0; s<NSEG; s++)
		Trk[s] = new Eagle_Segment(Track.trackin[s], Track.trackout[s]);

	int Changes;
	int Iterations = 10;

	do
	{
		Changes = 0;

		for (s=0; s<NSEG; s++){
		   if (Trk[s]->GetType() != STRAIGHT){
		      double PrevStr, NextStr;

		      if (Trk[_PREVSEG_(s)]->GetType() == STRAIGHT)
			 PrevStr = Trk[_PREVSEG_(s)]->GetLength(CENTER);
		      else
			 PrevStr = 0;
		      
		      if (Trk[_NEXTSEG_(s)]->GetType() == STRAIGHT)
			 NextStr = Trk[_NEXTSEG_(s)]->GetLength(CENTER);
		      else
			 NextStr = 0;
	
		      Changes += Trk[s]->FixSegmentOverlaps(Trk[GetPrevCurve(s)], 
					  Trk[GetNextCurve(s)], PrevStr, NextStr);
		   }
		}
	} while (Changes && --Iterations);
}



// cleanup the track
Eagle_Track::~Eagle_Track()
{
	for (int s=0; s<NSEG; s++)
		delete Trk[s];

	delete [] Trk;
}




// get the world position of a point on the track
inline void Eagle_Track::GetWorldPos(int seg, double to_center, double to_end, double &x, double &y)
{
	x = Trk[seg]->GetWorldX(CENTER, to_center, to_end);
	y = Trk[seg]->GetWorldY(CENTER, to_center, to_end);
}



// get the world direction of a vector on the track
inline double Eagle_Track::GetWorldDir(int seg, double v, double vn, double end)
{
	return Trk[seg]->GetWorldDir(v, vn, end);
}


// get the number of the segment that is the next curve
inline int Eagle_Track::GetNextCurve(int seg)
{
	while(Trk[seg = _NEXTSEG_(seg)]->GetType() == STRAIGHT);

	return seg;
}


// get the number of the segment that is the previous curve
inline int Eagle_Track::GetPrevCurve(int seg)
{
	while(Trk[seg = _PREVSEG_(seg)]->GetType() == STRAIGHT);

	return seg;
}


// find out if we are inside, outside, or on the path 
inline int Eagle_Track::GetCurveRelation( int seg, int path, double x, double y )
{
	return Trk[seg]->GetCurveRelation( path, x, y );
}


// determine the braking distance from one curve to another
double Eagle_Track::BrakingDistance(CURVEINFO Curve1, CURVEINFO Curve2)
{
   double Dist = 0;

   if (!Curve1.OnCurve && !Curve2.OnCurve){
      Dist += 0.75*fabs(Trk[Curve1.Seg]->GetRadius(INSIDE)) * Trk[Curve1.Seg]->GetLength(INSIDE);

      if (_PREVSEG_(Curve2.Seg) != Curve1.Seg)
	 Dist += Trk[_PREVSEG_(Curve2.Seg)]->GetLength(CENTER);

      Dist += _DIST_(Trk[Curve1.Seg]->GetBegX(Curve1.Path), 
		     Trk[Curve1.Seg]->GetBegY(Curve1.Path), 
		     Trk[Curve1.Seg]->GetBegX(INSIDE), 
		     Trk[Curve1.Seg]->GetBegY(INSIDE));

      Dist -= _DIST_(Trk[Curve2.Seg]->GetBegX(Curve2.Path), 
		     Trk[Curve2.Seg]->GetBegY(Curve2.Path), 
		     Trk[Curve2.Seg]->GetBegX(INSIDE), 
		     Trk[Curve2.Seg]->GetBegY(INSIDE));
   }
   else
      Dist = Curve2.Dist;
   
   return Dist;
}


// get the angle to the tangent of a path on the track
inline double Eagle_Track::CurveTangentAngle(int seg, int path, double x, double y)
{
	return Trk[seg]->CurveTangentAngle(path, x, y);
}



// get the distance to teh tangent point of a path on the track
inline double Eagle_Track::CurveTangentDistance(int seg, int path, double x, double y)
{
	return Trk[seg]->CurveTangentDistance(path, x, y);
}



// get the speed of the curve
inline double Eagle_Track::GetCurveSpeed(int seg, int path)
{
	return Trk[seg]->GetCurveSpeed(path);
}



// get basic information for a curve
void Eagle_Track::GetCurveInfo( int seg, CARINFO &CarInfo, CURVEINFO &CurveInfo )
{
   CurveInfo.Dist = Trk[CurveInfo.Seg]->
      CurveTangentDistance(CurveInfo.Path, CarInfo.PosX, CarInfo.PosY);
   CurveInfo.Angle = Trk[CurveInfo.Seg]->
     CurveTangentAngle(CurveInfo.Path, CarInfo.PosX, CarInfo.PosY);
   CurveInfo.Relation = Trk[CurveInfo.Seg]->
     GetCurveRelation( CurveInfo.Path, CarInfo.PosX, CarInfo.PosY);
   CurveInfo.rDist = Trk[CurveInfo.Seg]->
     OffCurveDist( CurveInfo.Path, CarInfo.PosX, CarInfo.PosY);
   CurveInfo.Speed = Trk[CurveInfo.Seg]->GetCurveSpeed(CurveInfo.Path);

   if (CurveInfo.Relation != OUTSIDECURVE){
       double Beg_Ang = Trk[CurveInfo.Seg]->GetBegAngle(CurveInfo.Path);
       double End_Ang = Trk[CurveInfo.Seg]->GetEndAngle(CurveInfo.Path);
       double Mid_Ang, Delta_Ang;
       
       if (Trk[CurveInfo.Seg]->GetType() == LEFTTURN){
	  if (End_Ang < Beg_Ang)
	     End_Ang += TWOPI;
	  
	  Delta_Ang = (End_Ang - Beg_Ang)/2.0;
	  Mid_Ang = Beg_Ang + Delta_Ang - _DTR_(90);
       }
       else{
	  if (Beg_Ang < End_Ang)
	     Beg_Ang += TWOPI;
	  
	  Delta_Ang = (Beg_Ang - End_Ang)/2.0;
	  Mid_Ang = Beg_Ang - Delta_Ang + _DTR_(90);
       }

       double This_Ang =  _NORM_((_ANGLE_(	
					  Trk[CurveInfo.Seg]->GetCenX(CurveInfo.Path),
					  Trk[CurveInfo.Seg]->GetCenY(CurveInfo.Path), 
					  CarInfo.PosX, CarInfo.PosY)));

       if (_ALMOST_(_NORM_(Mid_Ang-This_Ang), 0, Delta_Ang))
	  CurveInfo.OnCurve = 1;
   }
}



////////////////////////////////////////////////////////////
// Eagle_Car FUNCTIONS
////////////////////////////////////////////////////////////
// initialize a car
Eagle_Car::Eagle_Car()
{
	DistanceTraveled = 0.0;
	FuelUsed = 0.0;
	LastLapsToGo = -1;
	LapsTraveled = -1;
	FuelPerLap = 0.0;
	FuelStopsRequired = 0;
	LastDamage = 0;
	TotalDamage = 0;
	DamageAllowed = 0;
	ChangedLaps = 0;
	HeavyTrafficCount = 0;
	AngleAdjustment = 0;
	LapsInTank = 999;
	DamageLastLap = 0;
	DamageMaxLap = 0;
	DamagePerLap = 0;

	Road = 0;
	OldCurve.Seg = 999;
}


// cleanup a car
Eagle_Car::~Eagle_Car()
{
	if (Road)
		delete Road;

	Road = 0;
}


// for times when we are lost, try to find the curves again
void Eagle_Car::ResetCurves(void)
{
	Curve[0].Seg = Road->GetNextCurve(_PREVSEG_(CurrentSegment));
	if ((car_count>2) && ((Curve[0].Seg == 1) || (Curve[0].Seg == _PREVSEG_(0))))
		Curve[0].Path = SAFE;
	else
		Curve[0].Path = BEST;

	Curve[0].OnCurve = 0;
	Curve[0].Effort = 0.97;

	Curve[1].Seg = Road->GetNextCurve(Curve[0].Seg);
	if ((car_count>2) && ((Curve[1].Seg == 1) || (Curve[1].Seg == _PREVSEG_(0))))
		Curve[1].Path = SAFE;
	else
		Curve[1].Path = BEST;

	Curve[1].OnCurve = 0;
	Curve[1].Effort = 0.97;

	Curve[2].Seg = Road->GetNextCurve(Curve[1].Seg);
	if ((car_count>2) && ((Curve[2].Seg == 1) || (Curve[2].Seg == _PREVSEG_(0))))
		Curve[2].Path = SAFE;
	else
		Curve[2].Path = BEST;

	Curve[2].OnCurve = 0;
	Curve[2].Effort = 0.97;

	CurrentPath = Curve[0].Path;
}


// what to do about being stuck
void Eagle_Car::IsStuck(situation &s)
{
	CarStuck = 1;

	CheckOtherCars(s);
	if (s.start_time >0.0)
	{
		if ((s.fuel > 0.0) && (LastFuel>=s.fuel))
			FuelUsed += LastFuel - s.fuel;

		if (s.fuel > 0.0)
			LastFuel = s.fuel;
	}
}


// review the situation
void Eagle_Car::WhatsHappening(situation &s)
{
	ChangedLaps = 0;
	BrakeAmount = 0;
	Effort = 1.0;

	if (CurrentSegment != s.seg_ID)
	{
		CourseChangedInSegment = 0;
		CurrentSegment = s.seg_ID;
	}

	if (CarStuck)
	{
		CarStuck = 0;
		ResetCurves();
	}

	ToEnd = s.to_end;
	ToCenter = (width/2)-s.to_lft;
	Road->GetWorldPos(s.seg_ID, ToCenter, ToEnd, CarInfo.PosX, CarInfo.PosY);
	CarInfo.Angle = Road->GetWorldDir(s.seg_ID, s.v, s.vn, ToEnd);
	CarInfo.Speed = sqrt(_SQR_(s.v)+_SQR_(s.vn));
	AngleChange = 0.0;

	if (s.start_time >0.0)
	{
		DistanceTraveled += _DIST_(CarInfo.PosX, CarInfo.PosY, LastPosX, LastPosY );
		if ((s.fuel > 0.0) && (LastFuel>=s.fuel))
			FuelUsed += LastFuel - s.fuel;

		if (FuelUsed >0.0)
			FPP = DistanceTraveled/FuelUsed;

		if (s.lap_flag && s.laps_to_go != LastLapsToGo)
		{
			ChangedLaps = 1;
			LastLapsToGo = s.laps_to_go;
			LapsTraveled++;
		}
		else
			ChangedLaps = 0;
	}

	LastPosX = CarInfo.PosX;
	LastPosY = CarInfo.PosY;
	if (s.fuel > 0.0)
		LastFuel = s.fuel;
	LastDistX = Curve[0].rDist;

	WhereAreWeGoing();
	CheckOtherCars(s);
	CheckTraffic(s);

	if (s.damage > LastDamage)
		TotalDamage += (s.damage-LastDamage);

	if (ChangedLaps && LapsTraveled)
	{
		DamageMaxLap = (unsigned long)_MAX_(DamageMaxLap, TotalDamage-DamageLastLap);
		DamageLastLap = TotalDamage;
		DamagePerLap = TotalDamage/LapsTraveled;

		FuelPerLap = FuelUsed/LapsTraveled;
		LapsInTank = (int)(s.fuel/FuelPerLap);

		if (LapsInTank > LastLapsToGo)
			FuelStopsRequired = 0;
		else
			FuelStopsRequired = (int)((LastLapsToGo-LapsInTank)*FuelPerLap/MaxFuel)+1;
	}

	LastDamage = s.damage;

	DamageAllowed = (unsigned long)_MIN_(15000,
						_MIN_(25000 - DamageMaxLap, 
						_MAX_(0L, (unsigned long)(30000 - LastLapsToGo * DamagePerLap))));


	if (((s.damage > 14567) || (s.fuel < FuelPerLap*1.05)) && (!NoPit))
		TimeToPit = 1;
	else
		TimeToPit = 0;

	Effort *= _MIN_(1.15 - 0.00001 * s.damage, 1.0);

	if (car_count > 1)
	{
		if (HeavyTrafficCount > 3)
			Effort *= (1-((HeavyTrafficCount-3)/100));

		if ((s.position > _MAX_(2,car_count/3)) && (LapsTraveled < 2))
			Effort *= (0.9 + LapsTraveled *.05);

		if (OtherBestTime > 60)
			Effort *= 1.0-_MIN_(0.10, (OtherBestTime - 60)/2500);

		if (HeavyTrafficCount)
			HeavyTrafficCount--;
	}
}


// change from one curve to another
void Eagle_Car::ChangeCurve(int CurveNumber)
{
	OldCurve = Curve[0];

	while (CurveNumber--)
	{
		Curve[0] = Curve[1];
		Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[0] );

		Curve[1] = Curve[2];
		Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[1] );

		Curve[2].Seg = Road->GetNextCurve(Curve[1].Seg);

		if ( (Curve[1].Seg == _PREVSEG_(Curve[2].Seg)) && (Curve[1].Path != BEST) &&
			(Road->GetType(Curve[1].Seg) == Road->GetType(Curve[1].Seg)))
		{
			Curve[2].Path = SAFE;
		}
		else if ((car_count>2) && ((Curve[2].Seg == 1) || (Curve[2].Seg == _PREVSEG_(0))))
			Curve[2].Path = SAFE;
		else
			Curve[2].Path = BEST;

		Curve[2].OnCurve = 0;
		Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[2] );
		Curve[2].Effort = 1.0;

		CurrentPath = Curve[0].Path;
	}
}


// change from one path on a curve to another path on the same curve
void Eagle_Car::ChangePath(int NewPath)
{
	return;
	int OldPath = Curve[0].Path;

	if (NewPath == OldPath)
		return;

	CurrentPath = NewPath;
	Curve[0].Path = NewPath;

	Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[0] );

	if (Curve[0].Relation == INSIDECURVE)
	{
		CurrentPath = OldPath;
		Curve[0].Path = OldPath;

		Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[0] );
		NewPath = OldPath;
	}
}


// see what is happening with the other cars
void Eagle_Car::CheckOtherCars(situation &s)
{
	int i;
	static int init = 1;

	if (car_count == 1)
		return;

	if (init)
	{
		init = 0;
		for (i = 0; i < car_count; i++)
		{
			Car[i].Laps = 0;
			Car[i].Time = 0.0;
			Car[i].Speed = 0.0;
			Car[i].TimePerLap = 0.0;
			Car[i].Behind = 0.0;
		}
	}

	for (i = 0; i < car_count; i++)
	{
		if (Car[i].Speed != pcar[i]->Speed_avg)
		{
			Car[i].Laps++;
			Car[i].Time = s.time_count;
			Car[i].Speed = pcar[i]->Speed_avg;
			Car[i].TimePerLap = Car[i].Time/Car[i].Laps;

			if (Car[i].Laps == LapsTraveled+LastLapsToGo)
				NoPit = 1;
		}
	}

	OtherBestTime = 99999.0;

	for (i = 0; i < car_count; i++)
	{
		if ((i != s.my_ID) && !Eagles[s.my_ID])
		{
			Car[i].Behind = Car[i].Time - Car[s.my_ID].Time +
				Car[i].TimePerLap * (Car[s.my_ID].Laps - Car[i].Laps);
				
			if (Car[i].Behind < OtherBestTime)
			{
				OtherBestTime = Car[i].Behind;
				OtherBestCar = i;
			}
		}
	}

	if (OtherBestTime == 99999.0)
		OtherBestTime = 0.0;
}


// is there danger from the other cars
void Eagle_Car::CheckTraffic(situation &s)
{
	double TimeToImpact = 999.0;
	double TimeToCurve;
	double DeltaDist, PosAng, DirAng, DeltaAng, DeltaSpeed, DeltaTime, OtherToCenter;

	if (Curve[0].OnCurve)
		TimeToCurve = 0;
	else
		TimeToCurve = Curve[0].Dist/CarInfo.Speed;

	for(int i=0; i<3; i++)
	{
		if (s.nearby[i].who>16)
			continue;

		DeltaDist  = _DIST_(0, 0, s.nearby[i].rel_y, s.nearby[i].rel_x);
		PosAng     = _ANGLE_(0, 0, s.nearby[i].rel_y, -s.nearby[i].rel_x);
		DirAng     = _ANGLE_(0, 0, -s.nearby[i].rel_ydot, s.nearby[i].rel_xdot);
		DeltaSpeed = _DIST_(0, 0, s.nearby[i].rel_ydot, s.nearby[i].rel_xdot);
		DeltaAng   =  PosAng - DirAng;

		OtherToCenter = ToCenter - s.nearby[i].rel_x;

		if (DeltaDist <= 1.0)
			DeltaDist = 1.0;

		if (DeltaSpeed > 0)
			DeltaTime  = DeltaDist/DeltaSpeed;
		else
			DeltaTime  = 999.0;

		if (Eagles[s.nearby[i].who] && 
			fabs(s.nearby[i].rel_x) < CARWID && s.nearby[i].rel_y < CARLEN)
		{
			Effort *= 0.99;

			if (ToCenter > 0)
				AngleAdjustment--;
			else
				AngleAdjustment++;
		}


		if ((DeltaDist > 15*CARLEN) || (DeltaTime > 7.0) ||
			(!_ALMOST_(_NORM_(DeltaAng), 0, _DTR_(90-6*(DeltaDist/CARLEN)))))
			continue;

		if (((DeltaDist < 4*CARLEN) || (DeltaTime < 3.0)) && (HeavyTrafficCount < 10))
			HeavyTrafficCount++;

		if (Curve[0].Path == BEST)
			Curve[0].Path = SAFE;

		if ( (Curve[0].Seg == _PREVSEG_(Curve[1].Seg)) &&
			(Road->GetType(Curve[0].Seg) == Road->GetType(Curve[0].Seg)))
		{
			Curve[1].Path = SAFE;

			if ( (Curve[1].Seg == _PREVSEG_(Curve[2].Seg)) &&
				(Road->GetType(Curve[1].Seg) == Road->GetType(Curve[1].Seg)))
			{
				Curve[2].Path = SAFE;
			}
		}

		if (DeltaTime < TimeToImpact)
		{
			TimeToImpact = DeltaTime;

			if ((!Curve[0].OnCurve) && (TimeToCurve > 2))
			{
				if (_ALMOST_(_NORM_(PosAng), 0, _DTR_(90/_MAX_(1.0,DeltaTime))))
				{
					if ((s.nearby[i].rel_x * Road->GetType(Curve[0].Seg)) < 0)
						ChangePath(PASS1);
					else if (Curve[0].Path == PASS1)
						ChangePath(PASS2);

					if ((OtherToCenter > 0) && (ToCenter > -width/4))
						AngleAdjustment = -EG03.Traffic;
					else if ((OtherToCenter < 0) && (ToCenter < width/4))
						AngleAdjustment = EG03.Traffic;

					if ((_ALMOST_(_NORM_(PosAng), 0, _DTR_(33/_MAX_(1.0,DeltaTime)))))
						BrakeAmount = _MAX_(BrakeAmount, DeltaSpeed/DeltaTime);
				}
			}
			else
			{
				if (_ALMOST_(_NORM_(PosAng), 0, _DTR_(45/_MAX_(1.0,DeltaTime))))
					BrakeAmount = _MAX_(BrakeAmount, DeltaSpeed/4.7/_MAX_(1.0,DeltaTime));
			}
		}		
	}
}


// what are wee trying to do on the track
void Eagle_Car::WhereAreWeGoing()
{
	Curve[0].Path = CurrentPath;
	Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[0] );
	Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[1] );
	Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[2] );

	if (((CurrentSegment == OldCurve.Seg) || 
		(Road->GetNextCurve(CurrentSegment) == OldCurve.Seg)))
	{
		OldCurve.Path = INSIDE;
		Road->GetCurveInfo( CurrentSegment, CarInfo, OldCurve );
		OldCurve.bDist = OldCurve.Dist;

		if ((OldCurve.bDist < Curve[0].bDist) &&
			(_NORM_(Curve[0].Angle-OldCurve.Angle)*Road->GetType(OldCurve.Seg) > _DTR_(EG03.Inside/4)))
		{
			ResetCurves();

			Curve[0].Path = INSIDE;
			CurrentPath = INSIDE;
			Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[0] );
			Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[1] );
			Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[2] );
		}
	}

	if (Curve[1].OnCurve && (Curve[1].Relation == INSIDECURVE))
	{
		if (Curve[1].Path == BEST)
			Curve[1].Path = PASS1;
		else
			Curve[1].Path = INSIDE;

		Road->GetCurveInfo( CurrentSegment, CarInfo, Curve[1] );
	}
}


// which way should we steer now
double Eagle_Car::WhichWay()
{
	if (Curve[0].OnCurve)
	{
		if ((_NORM_(CarInfo.Angle-Curve[1].Angle) * Road->GetType(Curve[0].Seg) > 0) &&
			(Curve[1].Relation != INSIDECURVE))
		{
			ChangeCurve(1);
		}
		else if ((CurrentSegment == Curve[1].Seg) &&
			(Road->GetType(Curve[0].Seg) == Road->GetType(Curve[1].Seg)) &&
			(fabs(Road->GetRadius(Curve[0].Seg, Curve[0].Path)) >= 
			fabs(Road->GetRadius(Curve[1].Seg, Curve[1].Path))))
		{
			Curve[1].OnCurve = 1;
			ChangeCurve(1);
		}	
		else if (_ALMOST_(CarInfo.Angle-Curve[1].Angle, 0, _DTR_(90)) && 
			(Curve[1].Relation != INSIDECURVE) &&
			(_NORM_(CarInfo.Angle-Curve[1].Angle)*Road->GetType(Curve[0].Seg) > _DTR_(1)))
		{
			ChangeCurve(1);
		}

	}
	else
	{
		if ((_NORM_(Curve[0].Angle-Curve[1].Angle)*Road->GetType(Curve[0].Seg) > 0) &&
			(Curve[1].Relation != INSIDECURVE) && 
			(Road->GetType(Curve[0].Seg)*Road->GetType(Curve[1].Seg) < 0) &&
			(CurrentSegment == _PREVSEG_(Curve[0].Seg)))
		{
			ChangeCurve(1);
		}
		else if (CurrentSegment == Curve[1].Seg)
		{
			Curve[1].OnCurve = 1;
			ChangeCurve(1);
		}	
		else if ((CurrentSegment == _NEXTSEG_(Curve[0].Seg)) &&
			(_ALMOST_(_NORM_(CarInfo.Angle-Curve[1].Angle), 0, _DTR_(0.5))))
		{
			ChangeCurve(1);
		}	
	}

	double Direction;

	Direction = Curve[0].Angle;

	if (!Curve[0].OnCurve)
	{
		if ((car_count>2) && (CurrentSegment == 0) && (Curve[0].Seg == 1) &&
			(Curve[0].bDist > 567.89))
		{
			double qx, qy;
			Road->GetWorldPos(0, 0, ToEnd-200, qx, qy);
			Direction = _ANGLE_(CarInfo.PosX, CarInfo.PosY, qx, qy );

			if (fabs(ToCenter)> width/7)
				AngleAdjustment -= (int)(13*ToCenter/width);
		}

		if (AngleAdjustment>7)
			AngleAdjustment = 7;
		else if (AngleAdjustment<-7)
			AngleAdjustment = -7;

		if ((AngleAdjustment>0) && (Curve[0].Dist > 500) && (ToCenter<width/4))
			Direction += _DTR_(AngleAdjustment/2.2);
		else if ((AngleAdjustment<0) && (Curve[0].Dist > 500) && (ToCenter>-width/4))
			Direction += _DTR_(AngleAdjustment/2.2);
	}
	else if (CarInfo.Speed > 0.9 * Curve[0].Speed)
	{
		double CurrentBias = EG03.Bias;

		if (Curve[0].Relation != INSIDECURVE)
		{
			if (Curve[0].rDist > 2)
				CurrentBias *= Curve[0].rDist/2;
			else if ((Curve[0].rDist >= 0.0) && (Curve[0].rDist < 1))
				CurrentBias *= Curve[0].rDist;

			if ((Curve[0].rDist > 0) && (Curve[0].rDist > LastDistX))
				CurrentBias *=2;

			Direction += CurrentBias*(CarInfo.Speed / 
				Road->GetCurveBias( Curve[0].Seg, Curve[0].Path ));
		}
		else
		{
			Direction += Curve[0].rDist*CurrentBias*(CarInfo.Speed / 
				Road->GetCurveBias( Curve[0].Seg, Curve[0].Path ));
		}
	}


	if (AngleAdjustment > 0)
		AngleAdjustment--;
	else if (AngleAdjustment < 0)
		AngleAdjustment++;

	return Direction;
}



// general drive the car routine
void Eagle_Car::Drive(situation &s, con_vec &result)
{
	if (!Road)
	{	
		Road = new Eagle_Track;
		ResetCurves();
		Curve[0].Effort = 1.0;
		Curve[1].Effort = 1.0;
		Curve[2].Effort = 1.0;
		MaxFuel = s.fuel;
		CheckOtherCars(s);
		Eagles[s.my_ID] = 1;
		TimeToPit = 0;    
		NoPit = 0;
	}

	WhatsHappening(s);
	result.alpha = _NORM_(WhichWay() - CarInfo.Angle);

	Curve[0].bDist = Curve[0].Dist;
	Curve[1].bDist = Road->BrakingDistance(Curve[0], Curve[1]);
	Curve[2].bDist = Road->BrakingDistance(Curve[1], Curve[2]);

	if ((Curve[0].OnCurve) || (fabs(result.alpha) > _DTR_(5)))
		Curve[0].bSpeed = Effort*Curve[0].Effort*Curve[0].Speed;
	else
		Curve[0].bSpeed = Effort*Curve[0].Effort*sqrt(_SQR_(Curve[0].Speed)+2.0*Curve[0].bDist*EG03.MaxBrake);

	Curve[1].bSpeed = Effort*Curve[0].Effort*sqrt(_SQR_(Curve[1].Speed)+2.0*Curve[1].bDist*EG03.MaxBrake);
	Curve[2].bSpeed = Effort*Curve[0].Effort*sqrt(_SQR_(Curve[2].Speed)+2.0*Curve[2].bDist*EG03.MaxBrake);

	for( int i=0; i<3; i++ )
	{
		if (CarInfo.Speed > Curve[i].bSpeed)
			BrakeAmount = _MAX_(BrakeAmount, CarInfo.Speed - Curve[i].bSpeed);
	}

	if (BrakeAmount)
	{
		if ((Curve[0].OnCurve) && 
			(fabs(Road->GetRadius(Curve[0].Seg, Curve[0].Path)) < EG03.MaxRadius/2))
			result.vc = CarInfo.Speed - _MIN_(BrakeAmount, Effort*Curve[0].Effort*EG03.CornerBrake);
		else
			result.vc = CarInfo.Speed - _MIN_(BrakeAmount*4, Effort*Curve[0].Effort*EG03.MaxBrake);
	}
	else
	{
		if (Curve[0].OnCurve && 
			(fabs(Road->GetRadius(Curve[0].Seg, Curve[0].Path)) < EG03.MaxRadius))
			result.vc = _MIN_(Effort*Curve[0].Effort*Curve[0].Speed, CarInfo.Speed + Effort*Curve[0].Effort*EG03.MaxAccel);
		else
			result.vc = CarInfo.Speed + Effort*Curve[0].Effort*EG03.MaxAccel;
	}

	if (TimeToPit && (CurrentSegment == 0) && (s.laps_to_go > 1) && (Curve[0].Dist < 1000))
	{
		if (s.laps_to_go < 30)
			result.repair_amount = s.damage - DamageAllowed;
		else
			result.repair_amount = s.damage;

		if (s.damage - result.repair_amount > 5000)
			result.repair_amount = s.damage - 5000;

		result.request_pit = 1;
		result.alpha       = 0.0;
		result.vc          = 0.0;
		AngleAdjustment    = 0;
	}
	else
	{
		result.repair_amount = 0;
		result.request_pit = 0;
	}
}




////////////////////////////////////////////////////////////
// ROBOT DRIVER FUNCTION
////////////////////////////////////////////////////////////
// Entry point for Eagle
con_vec Eagle(situation &s)
{
	static Eagle_Car * EagleCar[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static int StartFlag = 1;
	static int CarNumber = 0;
	static con_vec Eagle_Result;

	Eagle_Result.alpha         = 0.0;
	Eagle_Result.vc            = EG03.MaxSpeed;
	Eagle_Result.repair_amount = 0;
	Eagle_Result.request_pit   = 0;

	if (s.starting || StartFlag)
	{
		StartFlag = 0;

		if (EagleCar[s.my_ID])
		{
			delete EagleCar[s.my_ID];
			EagleCar[s.my_ID] = 0;
		}

		EagleCar[s.my_ID] = new Eagle_Car;
		::my_name_is(CarName[CarNumber++]);
	}
	else if (stuck (s.backward, s.v, s.vn, s.to_lft, s.to_rgt, 
				&Eagle_Result.alpha, &Eagle_Result.vc))
		EagleCar[s.my_ID]->IsStuck(s);
	else
		EagleCar[s.my_ID]->Drive(s, Eagle_Result);

	return Eagle_Result;
}
