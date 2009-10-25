/*
Author:				Tim Foden (Newquay, Cornwall, UK)
E-mail:				tim@7sun.com
Robot name:			"Dodger13"
Robot version:		13
Driver function:	Dodger13()
Filename:			Dodger13.cpp
Races:				All
RARS version:		v0.83 and v0.90 (see #define COMPAT_2001_SEASON)
Data file:			Dodger13.dat (put in robots/dodger13.dat)
Robot status:		Public
Race colours:		White (front) & Black (rear)
Release date:		13-September-2001
Tab size used:		4
*/ 

#if defined(_WINDOWS)
#include "stdafx.h"
#endif

#include <stdio.h>
#include <string.h> 
#include <stdlib.h> 
#include <math.h> 

#include "car.h" 
#include "track.h"
#include "misc.h"
#include "os.h"

/////////////////////////////////////////////////////////////////////////

typedef unsigned char	BYTE;

/////////////////////////////////////////////////////////////////////////

//#define COMPAT_2001_SEASON
#ifdef COMPAT_2001_SEASON

/////////////////////////////////////////////////////////////////
// for compatibility with race season 2001 (v0.83)
/////////////////////////////////////////////////////////////////

// get hold of the external surface variable
extern int surface;

// find out how many cars are in the race
extern int car_count;

// get hold of external car variable
extern Car*		pcar[];
#define			CURRENT_TRACK_NAME	trackfile[current_track]

inline double Car::get_lin_acc(void)
{  return tan_a/g;  }

inline double Car::get_speed(void)    
{ return sqrt(xdot * xdot + ydot * ydot);  }    

inline double Car::get_fuel(void)
{  return fuel;  } 

#else

/////////////////////////////////////////////////////////////////
// for compatibility with current development version (v0.90)
/////////////////////////////////////////////////////////////////

#define			pcar	race_data.cars
const int		MAXCARS = MAX_CARS;
#define			car_count	args.m_iNumCar
#define			surface		args.m_iSurface
#define			CURRENT_TRACK_NAME	currentTrack->m_sFileName

#endif	// !COMPAT_2001_SEASON

/////////////////////////////////////////////////////////////////////////

#if !defined(ASSERT)
# if defined(WIN32) && defined(_DEBUG)
#  define ASSERT(expr)	{ if( !(expr) ) __asm { int 3 } }
# else
#  define ASSERT(expr)	;
# endif
#endif

/////////////////////////////////////////////////////////////////////////////

const double cCarWid = CARWID * 1.2;	// make out cars are wider than they really are
const double cCarDiag = hypot(CARWID, CARLEN);

// pit constants from Car::move_car() in carz.cpp
const double cPitSecsPerDamage = 0.005;
const double cPitSecsPerFuel = 0.05;

static double CORN_MYU = 1.05;		// lateral g's expected when cornering
static double MAX_ACCEL = 32.2;		// acceleration from road

const double BRAKE_ACCEL = 35.5;	// accel available while braking (straight)
const double BRAKE_COEF = 0.8;
//const double BRAKE_ACCEL = 27.0;	// accel available while braking (straight)
//const double BRAKE_COEF = 0.95;

//const double BRAKE_ACCEL_BEND = 27.0;	// accel available while braking (bend)
const double BRAKE_ACCEL_BEND = 29.0;	// accel available while braking (bend)
const double BRAKE_COEF_BEND = 0.95;
//const double BRAKE_ACCEL_BEND = 35.5;	// accel available while braking (bend)
//const double BRAKE_COEF_BEND = 0.8;

const double cMinGap = 0.5;		// in ft from edge
const double cMaxGap = 5.0;		// in ft from edge

const double cPi	= 3.1415926535897932384626433832795;
const double cPi_2	= cPi / 2;
const double c3Pi_2	= 3 * cPi / 2;
const double c2Pi	= 2 * cPi;

#ifndef min
#define	min(x, y)	((x) < (y) ? (x) : (y))
#endif

#ifndef max
#define	max(x, y)	((x) > (y) ? (x) : (y))
#endif

/////////////////////////////////////////////////////////////////////////////

const double	cDefaultFactor = 1.010;
static struct
{
	char*	m_name;		// name of track
	double	m_factor;	// 'straightness' factor for generated path
						// -- higher values give more circular corners
} factorTable[] =
{
	{ "aalborg.trk",		1.010 },
	{ "adelaide.trk",		1.010 },
	{ "albert.trk",			1.050 },
	{ "albrtprk.trk",		1.050 },
	{ "anew.trk",			1.020 },
	{ "austria.trk",		1.010 },
	{ "barcelon.trk",		1.010 },
	{ "brands.trk",			1.020 },
	{ "brazil.trk",			1.010 },
	{ "buenos.trk",			1.010 },
	{ "clkwis.trk",			1.010 },
	{ "cstlcomb.trk",		1.030 },
	{ "doningtn.trk",		1.010 },
	{ "dreiw.trk",			1.010 },
	{ "elev2.trk",			1.020 },
	{ "estoril.trk",		1.010 },
	{ "figure8.trk",		1.000 },
	{ "fiorano.trk",		1.005 },
	{ "fourmile.trk",		1.010 },
	{ "hock.trk",			1.030 },
	{ "hungary.trk",		1.010 },
	{ "imola.trk",			1.030 },
	{ "indy500.trk",		1.070 },
	{ "indygp.trk",			1.020 },
	{ "jerez.trk",			1.010 },
	{ "kevin.trk",			1.010 },
	{ "kimla01.trk",		1.030 },
	{ "kimla02.trk",		1.010 },
	{ "kimla03.trk",		1.010 },
	{ "kimla04.trk",		1.010 },
	{ "kimla05.trk",		1.010 },
//	{ "longrand.trk",		1.010 },
	{ "loudon.trk",			1.020 },
	{ "magnycrs.trk",		1.010 },
	{ "michigan.trk",		1.020 },
	{ "midohio.trk",		1.010 },
	{ "mlwaukee.trk",		1.010 },
	{ "monaco.trk",			1.020 },
	{ "montreal.trk",		1.025 },
	{ "monza-55.trk",		1.050 },
	{ "monza-76.trk",		1.075 },
	{ "mosport.trk",		1.015 },
	{ "nazareth.trk",		1.010 },
	{ "nurnburg.trk",		1.020 },
	{ "oval2.trk",			1.000 },
	{ "pheonix.trk",		1.010 },
	{ "pocono.trk",			1.015 },
	{ "ra.trk",				1.030 },
	{ "ramdu.trk",			1.005 },
	{ "rars.trk",			1.005 },
	{ "s1.trk",				1.010 },
	{ "s2.trk",				1.010 },
	{ "sepang.trk",			1.010 },
	{ "sepang1.trk",		1.010 },
	{ "silver97.trk",		1.010 },
	{ "silverst.trk",		1.020 },
	{ "spa.trk",			1.025 },
	{ "speed2.trk",			1.060 },
	{ "stef2.trk",			0.995 },
	{ "suzuka.trk",			1.015 },
	{ "timin.trk",			1.015 },
	{ "tremblnt.trk",		1.020 },
	{ "trouble.trk",		1.010 },
	{ "v01.trk",			1.010 },
	{ "v02.trk",			1.015 },
	{ "v03.trk",			1.005 },
	{ "watglen.trk",		1.035 },
	{ "wierd.trk",			1.000 },
	{ "zandvort.trk",		1.010 },
	{ NULL,					0.000 },
};

/////////////////////////////////////////////////////////////////////////////

static double	NormaliseAngle( double angle )
{
	if( angle >= 0 && angle < 2 * cPi )
		return angle;

	if( angle >= 2 * cPi )
	{
		double multiple = floor(angle / (2 * cPi));
		angle -= multiple * 2 * cPi;
	}
	else
	{
		double multiple = ceil(-angle / (2 * cPi));
		angle += multiple * 2 * cPi;
	}

	return angle;
}

/////////////////////////////////////////////////////////////////////////////

static double	NormaliseAngleLimit( double angle, double sizeAngle )
{
	angle = NormaliseAngle(angle);

	// allow half of angle outside range to be -ve
	double	halfRest = PI - sizeAngle / 2;
	if( angle > 2 * PI - halfRest )
		angle -= 2 * PI;

	return angle;
}

/////////////////////////////////////////////////////////////////////////////

static double	FindTrackFactor()
{
	for( int i = 0; factorTable[i].m_name; i++ )
	{
		if( strcmp(factorTable[i].m_name, CURRENT_TRACK_NAME) == 0 )
			return factorTable[i].m_factor;
	}

	return cDefaultFactor;
}

/////////////////////////////////////////////////////////////////////////////

struct	D13Vec2
{
public:
	D13Vec2();
	D13Vec2( double x, double y );
	D13Vec2( const D13Vec2& vec2 );

	D13Vec2	operator+( const D13Vec2& v ) const;
	D13Vec2	operator-( const D13Vec2& v ) const;
	D13Vec2	operator-() const;
	
	D13Vec2	operator*( double n ) const;		// scalar product
	D13Vec2	operator/( double n ) const;		// scalar divide

	double	operator*( const D13Vec2& v ) const;	// dot product
	double	operator%( const D13Vec2& v ) const;	// cross product

	bool	operator==( const D13Vec2& v ) const;	// equality
	bool	operator!=( const D13Vec2& v ) const;	// inequality

	double	GetLength() const;
	double	GetAngle() const;
	D13Vec2	GetUnit() const;
	D13Vec2	GetNormal() const;

	D13Vec2	ToCoordSys( const D13Vec2& p, const D13Vec2& v ) const;

	static D13Vec2	FromAngle( double angle );

public:
	double	x;
	double	y;
};

D13Vec2::D13Vec2()
:	x(0),
	y(0)
{
}

D13Vec2::D13Vec2( double X, double Y )
:	x(X),
	y(Y)
{
}

D13Vec2::D13Vec2( const D13Vec2& v )
:	x(v.x),
	y(v.y)
{
}

D13Vec2	D13Vec2::operator+( const D13Vec2& v ) const
{
	return D13Vec2(x + v.x, y + v.y);
}

D13Vec2	D13Vec2::operator-( const D13Vec2& v ) const
{
	return D13Vec2(x - v.x, y - v.y);
}

D13Vec2	D13Vec2::operator-() const
{
	return D13Vec2(-x, -y);
}

D13Vec2	D13Vec2::operator*( double n ) const
{
	return D13Vec2(x * n, y * n);
}

D13Vec2	D13Vec2::operator/( double n ) const
{
	return D13Vec2(x / n, y / n);
}

double	D13Vec2::operator*( const D13Vec2& v ) const
{
	return x * v.x + y * v.y;
}

double	D13Vec2::operator%( const D13Vec2& v ) const
{
	return x * v.y - y * v.x;
}

bool	D13Vec2::operator==( const D13Vec2& v ) const
{
	return x == v.x && y == v.y;
}

bool	D13Vec2::operator!=( const D13Vec2& v ) const
{
	return x != v.x || y != v.y;
}

double	D13Vec2::GetLength() const
{
	return hypot(x, y);
}

double	D13Vec2::GetAngle() const
{
	return atan2(y, x);
}

D13Vec2	D13Vec2::GetUnit() const
{
	if( x == 0 && y == 0 )
		return D13Vec2();

	double	len = hypot(x, y);
	return D13Vec2(x / len, y / len);
}

D13Vec2	D13Vec2::GetNormal() const
{
	return D13Vec2(-y, x);
}

D13Vec2	D13Vec2::ToCoordSys(
	const D13Vec2& origin,
	const D13Vec2& xAxisVec ) const
{
	D13Vec2	relPt(x - origin.x, y - origin.y);
	double	newX = xAxisVec * relPt;
	double	newY = xAxisVec.GetNormal() * relPt;
	return D13Vec2(newX, newY);
}

D13Vec2	D13Vec2::FromAngle( double angle )
{
	return D13Vec2(cos(angle), sin(angle));
}

/////////////////////////////////////////////////////////////////////////////

class D13Track  
{
public:
	struct Segment
	{
		double	m_rad;
		double	m_len;

		D13Vec2	m_beg;
		D13Vec2	m_end;
		D13Vec2	m_cen;
		double	m_dist;

		double	Ft() const { return m_rad == 0 ? m_len : m_len * m_rad; }
		D13Vec2	GetNormal( double len ) const;
	};

public:
	D13Track();
	virtual ~D13Track();

	void	Empty();

	void	Initialise( const track_desc& track );
	void	CalcLength();

	int		NSeg() const;
	double	GetLength() const;
	double	GetWidth() const;
	double	GetMaxX() const;
	void	SetMaxX( double maxX );
	double	GetMaxY() const;
	void	SetMaxY( double maxY );
	double	GetStartX() const;
	double	GetStartY() const;
	double	GetStartAng() const;
	double	GetStartLine() const;
	Segment	GetSegment( int seg ) const;
	double	CalcCarAngle( rel_state& state ) const;
	D13Vec2	CalcCarVel( rel_state& state ) const;

private:
	int		m_nSeg;
 
	double	m_length;

	double	m_xMax;
	double	m_yMax;

	double	m_width;
 
	double	m_startX;
	double	m_startY;
	double	m_startAng;

	double	m_scoreX;
	double	m_scoreY;
	double	m_ldrX;
	double	m_ldrY;
	double	m_instX;
	double	m_instY;
	double	m_lenX;
	double	m_lenY;
	double	m_startLine;
	int		m_startRows;
 
	int		m_pitSide;
	double	m_pitEntry;
	double	m_pitLaneStart;
	double	m_pitLaneEnd;
	double	m_pitExit;
	double	m_pitLaneSpeed;

	Segment*	m_pSeg;
};

D13Vec2	D13Track::Segment::GetNormal( double len ) const
{
	D13Vec2	n;
	if( m_rad == 0 )
	{
		// all normals in a straight are the same
		n = (m_end - m_beg).GetNormal().GetUnit();
	}
	else
	{
		// figure angle of radius at len
		double	startAngle = (m_beg - m_cen).GetAngle();
		if( m_rad > 0 )
			n = -D13Vec2::FromAngle(startAngle + len);
		else
			n = D13Vec2::FromAngle(startAngle - len);
	}

	return n;
}

D13Track::D13Track()
:	m_nSeg(0),
	m_length(0),
 	m_xMax(0),
	m_yMax(0),
 	m_width(0),
 	m_startX(0),
	m_startY(0),
	m_startAng(0),
 	m_scoreX(0),
	m_scoreY(0),
	m_ldrX(0),
	m_ldrY(0),
	m_instX(0),
	m_instY(0),
	m_lenX(0),
	m_lenY(0),
	m_startLine(0),
	m_startRows(0),
 	m_pitSide(0),
	m_pitEntry(0),
	m_pitLaneStart(0),
	m_pitLaneEnd(0),
	m_pitExit(0),
	m_pitLaneSpeed(0),
 	m_pSeg(0)
{
}

D13Track::~D13Track()
{
	Empty();
}

void	D13Track::Empty()
{
	m_nSeg = 0;
	m_length = 0;
 	m_xMax = 0;
	m_yMax = 0;
 	m_width = 0;
 	m_startX = 0;
	m_startY = 0;
	m_startAng = 0;
 	m_scoreX = 0;
	m_scoreY = 0;
	m_ldrX = 0;
	m_ldrY = 0;
	m_instX = 0;
	m_instY = 0;
	m_lenX = 0;
	m_lenY = 0;
	m_startLine = 0;
	m_startRows = 0;
 	m_pitSide = 0;
	m_pitEntry = 0;
	m_pitLaneStart = 0;
	m_pitLaneEnd = 0;
	m_pitExit = 0;
	m_pitLaneSpeed = 0;
	delete [] m_pSeg;
	m_pSeg = 0;
}

void	D13Track::Initialise( const track_desc& track )
{

	m_nSeg = track.NSEG;
	m_length = track.length;
 	m_xMax = 0;
	m_yMax = 0;
 	m_width = track.width,
 	m_startX = 0;
	m_startY = 0;
	m_startAng = 0;
 	m_scoreX = 0;
	m_scoreY = 0;
	m_ldrX = 0;
	m_ldrY = 0;
	m_instX = 0;
	m_instY = 0;
	m_lenX = 0;
	m_lenY = 0;
	m_startLine = 0;
	m_startRows = 0;
 	m_pitSide = track.pit_side,
	m_pitEntry = track.pit_entry,
	m_pitLaneStart = 0;
	m_pitLaneEnd = 0;
	m_pitExit = track.pit_exit;
	m_pitLaneSpeed = track.pit_speed;
	delete [] m_pSeg;
	m_pSeg = new Segment[track.NSEG];

	for( int i = 0; i < track.NSEG; i++ )
	{
		segment&	seg = track.rgtwall[i];

		m_pSeg[i].m_rad = seg.radius;
		m_pSeg[i].m_len = seg.length;
		m_pSeg[i].m_beg = D13Vec2(seg.beg_x, seg.beg_y);
		m_pSeg[i].m_end = D13Vec2(seg.end_x, seg.end_y);
		m_pSeg[i].m_cen = D13Vec2(seg.cen_x, seg.cen_y);

		m_pSeg[i].m_dist = track.seg_dist[i];
	}
}

void	D13Track::CalcLength()
{
	m_length = 0;
	for( int i = 0; i < m_nSeg; i++ )
	{
		Segment&	seg = m_pSeg[i];

		if( seg.m_rad == 0 )
		{
			m_length += seg.m_len;
		}
		else
		{
			m_length += (fabs(seg.m_rad) + m_width * 0.5) * seg.m_len;
		}
	}
}

int		D13Track::NSeg() const
{
	return m_nSeg;
}

double	D13Track::GetLength() const
{
	return m_length;
}

double	D13Track::GetWidth() const
{
	return m_width;
}

double	D13Track::GetMaxX() const
{
	return m_xMax;
}

void	D13Track::SetMaxX( double maxX )
{
	m_xMax = maxX;
}

double	D13Track::GetMaxY() const
{
	return m_yMax;
}

void	D13Track::SetMaxY( double maxY )
{
	m_yMax = maxY;
}

double	D13Track::GetStartX() const
{
	return m_startX;
}

double	D13Track::GetStartY() const
{
	return m_startY;
}

double	D13Track::GetStartAng() const
{
	return m_startAng;
}

double	D13Track::GetStartLine() const
{
	return m_startLine;
}

D13Track::Segment	D13Track::GetSegment( int seg ) const
{
	ASSERT( seg >= 0 && seg < m_nSeg );
	return m_pSeg[seg];
}

double	D13Track::CalcCarAngle( rel_state& state ) const
{
	// get current segment
	int		segId = pcar[state.who]->Seg_id;
	const D13Track::Segment&	seg = GetSegment(segId);

	// work out current position of car, global coords
	D13Vec2	carPt(pcar[state.who]->X, pcar[state.who]->Y);

	// work out current direction of car, global angle
	double	carAngle = asin(state.vn / state.v);
	if( seg.m_rad == 0 )
		carAngle += (seg.m_end - seg.m_beg).GetAngle();
	else if( seg.m_rad > 0 )
		carAngle += (carPt - seg.m_cen).GetAngle() + cPi_2;
	else
		carAngle += (carPt - seg.m_cen).GetAngle() - cPi_2;
	carAngle = NormaliseAngle(carAngle);

	return carAngle;
}

D13Vec2	D13Track::CalcCarVel( rel_state& state ) const
{
	double	angle = CalcCarAngle(state);
	return D13Vec2::FromAngle(angle) * state.v;
}

static double	CorneringSpeed( double radius )
{ 
	double	centreRadius = fabs(radius);
	return sqrt(centreRadius * g * CORN_MYU);
} 

static double	BrakingDistance( double fromVel, double toVel )
{
	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL;
}

static double	BrakingDistanceBend( double fromVel, double toVel )
{
	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL_BEND;
}

static double	CalcClosestPointOnLine( const D13Vec2& lp, const D13Vec2& lv, const D13Vec2& p )
{
	double	denom = lv * lv;
	if( denom == 0 )
		return 0;
	return ((p - lp) * lv) / denom;
}

static double	CalcDistanceToLine( const D13Vec2& lp, const D13Vec2& lv, const D13Vec2& p )
{
	double	vLength = lv.GetLength();
	if( vLength == 0 )
		return 0;
	return (lv % (p - lp)) / vLength;
}

inline bool	LineCrossesLine(
	const D13Vec2&	lp0,
	const D13Vec2&	lv0,
	const D13Vec2&	lp1,
	const D13Vec2&	lv1,

	double&		t )
{
	double	denom = lv0 % lv1;
	if( denom == 0 )
		return false;

	double	numer = lv1 % (lp0 - lp1);

	t = numer / denom;

	return true;
}

static bool	CalcTangent(
	const D13Vec2&	p1,
	const D13Vec2&	p2,
	const D13Vec2&	p3,

	D13Vec2&			tangent )
{
	D13Vec2	mid1  = (p1 + p2) / 2;
	D13Vec2	norm1 = (p2 - p1).GetNormal();
	D13Vec2	mid2  = (p2 + p3) / 2;
	D13Vec2	norm2 = (p3 - p2).GetNormal();

	double	t;
	if( !LineCrossesLine(mid1, norm1, mid2, norm2, t) )
	{
		if( p1 != p3 )
		{
			tangent = (p3 - p1).GetUnit();
			return true;
		}

		return false;
	}

	D13Vec2	centre = mid1 + norm1 * t;
	tangent = (p2 - centre).GetNormal().GetUnit();
	if( norm1 * (p3 - p1) < 0 )
		tangent = -tangent;
	return true;
}

static double	CalcCurvature(
	const D13Vec2&	p1,
	const D13Vec2&	p2,
	const D13Vec2&	p3 )
{
	// I did a little digging, to try to find a simpler way to calculate
	//	the radius of a circle through 3 points.
	//
	// This circle is called the Circumcircle of a triangle.
	//
	// From Eric Weisstein's World of Mathematics on triangles
	//	at http://mathworld.wolfram.com/Triangle.html we see that:
	//
	//		Area of triangle, A = abc / 4R						(equ 12)
	//
	//	where R is the radius of the Circumcircle.
	//
	// Rearranging gives R = abc / 4A
	//
	// So all we need is how to find the area of a triangle from the
	//	vertex points.  This is given on same web page.
	//
	//		    1 | x1  y1  1 |
	//		A = - | x2  y2  1 |									(equ 2)
	//		    2 | x3  y3  1 |
	//
	//		A = (x1y2 - x1y3 - y1x2 + y1x3 + x2y3 - y2x3) / 2
	//
	//	and to calculate the values of the lengths of the sides of the
	//	triangle, a, b and c, from the points.
	//
	//		a = sqrt[(x1 - x2)² + (y1 - y2)²]
	//		b = sqrt[(x3 - x2)² + (y3 - y2)²]
	//		c = sqrt[(x1 - x3)² + (y1 - y3)²]
	//
	// At this point we notice that if we moved 1 of the points to the
	//	origin, the calculation for the determinant would simplify
	//	even more.
	//
	// Let's move the x2, y2 to the origin.  The other points are now
	//	given by the points p and q, so:
	//
	//		p.x = x1 - x2,		p.y = y1 - y2
	//		q.x = y3 - y2,		q.y = y3 - y2
	//
	//  and for the sake of succintness later, we'll also work out s:
	//
	//		s.x = x1 - x3,		s.y = y1 - y3
	//
	// The area calculation now simplifies to:
	//
	//		A = (p.x * q.y - q.x * p.y) / 2
	//
	//	which in vector form is:
	//
	//		A = (p × q) / 2							(cross product)
	//
	//	and the calculations for the lengths are now:
	//
	//		a = sqrt(p.x² + p.y²)
	//		b = sqrt(q.x² + q.y²)
	//		c = sqrt(s.x² + s.y²)
	//
	// We also note that:
	//
	//		sqrt(n) * sqrt(m) == sqrt(n * m)
	//
	//	which allows us to simplify the calculation of abc:
	//
	//		abc = sqrt(p.x² + p.y²) * sqrt(q.x² + q.y²) * sqrt(s.x² + s.y²)
	//		    = sqrt[(p.x² + p.y²) * (q.x² + q.y²) * (s.x² + s.y²)]
	//
	//	and writing this in vector form:
	//
	//		abc = sqrt(p·p * q·q * s·s)				(dot products)
	//
	// So we can now wrap this up:
	//
	//		R = abc / 4A
	//		R = sqrt(p·p * q·q * s·s) / [4 (p × q) / 2]
	//		R = sqrt(p·p * q·q * s·s) / [2 (p × q)]
	//
	// Note 1: It is possible to have the radius be infinite,
	//	which occurs when the points are in a straight line.  Therefore
	//	we prefer to return the curvature (the reciprocal of the radius, 
	//	or kappa), which is simply zero in this case.
	//
	// Note 2: abc must be +ve, but the area (A) may be -ve.
	//	Due to the nature of the cross product, and the choice of which
	//	point we moved to the origin, this enables us to  predict the
	//	direction of the arc from p1 to p3, if we negate one of the
	//	vectors to the moved points.  K will then be +ve if the arc
	//	curves leftwards, and -ve if rightwards.
	//
	D13Vec2	p = p1 - p2;
	D13Vec2	q = p2 - p3;	// this vector has been negated
	D13Vec2	s = p3 - p1;
	double	K = (2 * (p % q)) / sqrt((p * p) * (q * q) * (s * s));
	return K;
}

class D13Slices
{
public:
	enum
	{
		cSliceFt		= 10,	// nominal length of each slice
	};

	struct Slice
	{
		D13Vec2	m_p;	// pt on right edge of track
		D13Vec2	m_n;	// normal at pt, for slice across track
		int		m_seg;	// segment this slice is in

		void		Setup( const D13Vec2& p, const D13Vec2& n, int seg )
						{ m_p = p; m_n = n; m_seg = seg; }
		double		CalcW( const D13Vec2& pt ) const
						{ return (pt - m_p) * m_n; }
		D13Vec2		CalcPt( double w ) const
						{ return m_p + m_n * w; }
	};

public:
	D13Slices() : m_size(0), m_pSlices(0), m_width(1) {}
	~D13Slices() { delete [] m_pSlices; }

	void			Setup( const D13Track& track );

	int				GetSize() const				{ return m_size; }
	const Slice&	GetAt( int index ) const	{ return m_pSlices[index]; }
	Slice&			GetAt( int index )			{ return m_pSlices[index]; }
	double			GetWidth() const			{ return m_width; }
	double			GetLength() const			{ return m_length; }

	int				Find( const D13Vec2& p, int start = -1 ) const;
	int				Find( const D13Vec2& p, int seg, int start ) const;

private:
	int				m_size;
	Slice*			m_pSlices;
	double			m_width;
	double			m_length;
};

void	D13Slices::Setup( const D13Track& track )
{
	m_size = 0;
	delete [] m_pSlices;
	int		maxLines = int(track.GetLength() * 2 / cSliceFt + 0.5);
	m_pSlices = new Slice[maxLines];

	double	width = m_width = track.GetWidth();
	m_length = track.GetLength();

	int		size = 0;
//	double	totalDist = 0;
//	double	startOffset = track.GetStartLine();
//	double	totalLength = track.GetLength();

	for( int i = 0; i < track.NSeg(); i++ )
	{
		D13Track::Segment	seg = track.GetSegment(i);

		if( seg.m_rad == 0 )
		{
			int		n = int(seg.m_len / cSliceFt);

			D13Vec2	beg = seg.m_beg;
			D13Vec2	normal = (seg.m_end - beg).GetNormal().GetUnit();
			D13Vec2	d = seg.m_end - seg.m_beg;

			if( i == 0 )
			{
				D13Vec2	tan = (beg - seg.m_end).GetUnit();
				double	extendDist = tan *
					(track.GetSegment(track.NSeg() - 1).m_end - beg);
				if( extendDist > 0 )
				{
					// extend start segment to meet with end segment
					beg = beg + tan * extendDist;
					n = int((seg.m_len + extendDist) / cSliceFt);
					d = seg.m_end - beg;
				}
			}

			m_pSlices[size].Setup( beg, normal, i );
			size++;

			for( int j = 1; j < n; j++ )
			{
				m_pSlices[size].Setup( beg + d * j / n, normal, i );
				size++;
			}
		}
		else
		{
			double	r = fabs(seg.m_rad - width * 0.5);

			int		n = int(r * seg.m_len / cSliceFt);
			double	angle = (seg.m_beg - seg.m_cen).GetAngle();

			if( seg.m_rad > 0 )
			{
				for( int j = 0; j < n; j++ )
				{
					double	alpha = seg.m_len * j / n;
					D13Vec2	normal = D13Vec2::FromAngle(angle + alpha);
					m_pSlices[size].Setup( seg.m_cen + normal * seg.m_rad,
											-normal, i );
					size++;
				}
			}
			else
			{
				for( int j = 0; j < n; j++ )
				{
					double	alpha = seg.m_len * j / n;
					D13Vec2	normal = D13Vec2::FromAngle(angle - alpha);
					m_pSlices[size].Setup( seg.m_cen + normal * -seg.m_rad,
											normal, i );
					size++;
				}
			}
/*
			if( i == track.NSeg() - 1 )
			{
				double	angle = (seg.m_end - seg.m_cen).GetAngle();
				D13Vec2	normal = D13Vec2::FromAngle(angle);
				if( seg.m_rad < 0 )
					normal = -normal;
				m_pSlices[size].Setup( seg.m_cen + normal * seg.m_rad,
											-normal, seg.m_dist );
				size++;
			}
*/		}
	}

	// fixup the last points, so they don't extend past the first point
	D13Vec2	n = m_pSlices[0].m_n.GetNormal();
	while(	(m_pSlices[size - 1].m_p - m_pSlices[0].m_p) * n < 0 ||
			(m_pSlices[size - 1].m_p + m_pSlices[size - 1].m_n * m_width -
				m_pSlices[0].m_p) * n < 0 )
		size--;

	m_size = size;
}

int		D13Slices::Find( const D13Vec2& p, int start ) const
{
	start = max(start, 0);
	ASSERT( start >= 0 && start < m_size );

	int		cur_s = start;

	do
	{
		int		next_s = (cur_s + 1) % m_size;
		const Slice&	s = GetAt(next_s);
		if( s.m_n.GetNormal() * (p - s.m_p) > 0 )
			break;

		cur_s = next_s;
	}
	while( cur_s != start );

	return cur_s;
}

int		D13Slices::Find( const D13Vec2& p, int seg, int start ) const
{
	start = max(start, 0);
	ASSERT( start >= 0 && start < m_size );

	int		cur_s = start;
	int		nSeg = m_pSlices[m_size - 1].m_seg + 1;
	int		nextSeg = (seg + 1) % nSeg;

	do
	{
		int		next_s = (cur_s + 1) % m_size;

		const Slice&	s0 = GetAt(cur_s);
		if( s0.m_seg == seg || s0.m_seg == nextSeg )
		{
			const Slice&	s1 = GetAt(next_s);
			if( s1.m_n.GetNormal() * (p - s1.m_p) >  0 &&
				s0.m_n.GetNormal() * (p - s0.m_p) <= 0 )
				break;
		}

		cur_s = next_s;
	}
	while( cur_s != start );

	return cur_s;
}

class D13Path
{
public:
	struct Path
	{
		const D13Slices::Slice*	m_pSlice;
		double	m_w;		// point on slice line
		double	m_spd;		// speed at point

		void	SetSlice( const D13Slices::Slice* pSlice )
									{ m_pSlice = pSlice; }

		D13Vec2	GetPt() const		{ return m_pSlice->CalcPt(m_w); }
	};

public:
	D13Path() : m_pSlices(0), m_pPath(0) {}
	D13Path( const D13Path& path );
	D13Path( const D13Slices* pSlices );
	~D13Path() { delete [] m_pPath; }

	void			Setup( const D13Slices* pSlices );
	int				GetSize() const { return m_pSlices->GetSize(); }

	const Path&		GetAt( int index ) const	{ return m_pPath[index]; }
	Path&			GetAt( int index )			{ return m_pPath[index]; }

	double			GetW( int index ) const		{ return m_pPath[index].m_w; }
	double			GetSpd( int index ) const	{ return m_pPath[index].m_spd; }
	D13Vec2			GetPt( int index ) const
						{ return m_pSlices->GetAt(index).CalcPt(
													m_pPath[index].m_w); }


	D13Path&			operator=( const D13Path& path );

	void			Calc( int cur_s, const D13Vec2& pt, double& dist,
							D13Vec2& vel, double& k ) const;

	void			CalcSpeed( double damage = 0, double fuel = MAX_FUEL );
	void			CalcSpeedSection( int from, int len, double damage = 0,
										double fuel = MAX_FUEL );
	void			SetInitialSpeed( int from, int len );
	void			PropagateBraking(	int from, int len, double damage = 0,
										double fuel = MAX_FUEL );
	double			EstimateSpeed(	double damage = 0,
									double fuel = MAX_FUEL ) const;

private:
	const D13Slices*	m_pSlices;
	Path*			m_pPath;	// size from pSlices
};

D13Path::D13Path( const D13Path& path )
:	m_pSlices(path.m_pSlices),
	m_pPath(0)
{
	m_pPath = new Path[m_pSlices->GetSize()];
	for( int i = 0; i < m_pSlices->GetSize(); i++ )
		m_pPath[i] = path.m_pPath[i];
}

D13Path::D13Path( const D13Slices* pSlices )
:	m_pSlices(0),
	m_pPath(0)
{
	Setup( pSlices );
}

void	D13Path::Setup( const D13Slices* pSlices )
{
	m_pSlices = pSlices;
	delete [] m_pPath;
	m_pPath = new Path[pSlices->GetSize()];

	double	halfWidth = pSlices->GetWidth() * 0.5;
	for( int i = 0; i < pSlices->GetSize(); i++ )
	{
		const D13Slices::Slice&	s = pSlices->GetAt(i);
		m_pPath[i].m_w = halfWidth;
		m_pPath[i].m_spd = 50;
		m_pPath[i].m_pSlice = &s;
	}
}

D13Path&	D13Path::operator=( const D13Path& path )
{
	ASSERT( m_pSlices == path.m_pSlices );
	for( int i = 0; i < m_pSlices->GetSize(); i++ )
		m_pPath[i] = path.m_pPath[i];

	return *this;
}

void	D13Path::Calc(
	int			cur_s,
	const D13Vec2&	pt,

	double&		dist,
	D13Vec2&		vel,
	double&		k ) const
{
	int		sl1 = m_pSlices->Find(pt, cur_s);
	int		sl0 = (sl1 + m_pSlices->GetSize() - 1) % m_pSlices->GetSize();
	int		sl2 = (sl1 + 1) % m_pSlices->GetSize();
	int		sl3 = (sl2 + 1) % m_pSlices->GetSize();

	D13Vec2	p0 = m_pPath[sl0].GetPt();
	D13Vec2	p1 = m_pPath[sl1].GetPt();
	D13Vec2	p2 = m_pPath[sl2].GetPt();
	D13Vec2	p3 = m_pPath[sl3].GetPt();
	double	k1 = CalcCurvature(p0, p1, p2);
	double	k2 = CalcCurvature(p1, p2, p3);
	D13Vec2	v1 = p2 - p1;
	D13Vec2	v2 = p3 - p2;
	double	t1 = (pt - p1) * v1 / (v1 * v1);

	D13Vec2	n = v1.GetNormal();
	D13Vec2	np = p1 + v1 * t1;
	double	tt = 0;
	double	oldTt = 1;

	int		count = 0;
	while( fabs(tt - oldTt) > 0.00001 )
	{
		oldTt = tt;
		D13Vec2	pp = np + n * tt;
		double	len1 = (np - p1).GetLength();
		double	len2 = (np - p2).GetLength();
		double	kappa = (k1 * len2 + k2 * len1) / (len1 + len2);
		double	k = CalcCurvature(p1, pp, p2);

		if( kappa != 0 )
		{
			double	delta = 0.0001;
			double	deltaK = CalcCurvature(p1, pp + n * delta, p2) - k;
			tt += delta * (kappa - k) / deltaK;
		}

		if( ++count > 100 )
			break;
	}

	D13Vec2	pp = np + n * tt;
	k = CalcCurvature(p1, pp, p2);
	D13Vec2	tangent;
	CalcTangent( p1, pp, p2, tangent );

	double	spd1 = m_pPath[sl1].m_spd;
	double	spd2 = m_pPath[sl2].m_spd;
	double	spd = spd1 * (1 - t1) + spd2 * t1;
	D13Vec2	dir = v1.GetUnit() * (1 - t1) + v2.GetUnit() * t1;
	{
		D13Vec2	vv1 = p2 - p0;
		D13Vec2	vv2 = p3 - p1;
		dir = vv1.GetUnit() * (1 - t1) + vv2.GetUnit() * t1;
	}
	{
		D13Vec2	tan1, tan2;
		CalcTangent( p0, p1, p2, tan1 );
		CalcTangent( p1, p2, p3, tan2 );
		dir = tan1.GetUnit() * (1 - t1) + tan2.GetUnit() * t1;
	}
//	dir = tangent;
	dist = v1.GetUnit().GetNormal() * (pt - p1) + tt;
	vel = dir * spd;
//	k = k1 * (1 - t1) + k2 * t1;
}

void	D13Path::CalcSpeed( double damage, double fuel )
{
	SetInitialSpeed( 0, m_pSlices->GetSize() );
	PropagateBraking( 0, m_pSlices->GetSize() + 1, damage, fuel );
	PropagateBraking( 0, m_pSlices->GetSize() + 1, damage, fuel );
}

void	D13Path::CalcSpeedSection( int from, int len, double damage, double fuel )
{
	SetInitialSpeed( from, len );
	PropagateBraking( from, len, damage, fuel );
}

void	D13Path::SetInitialSpeed( int from, int len )
{
	const int	size = m_pSlices->GetSize();

	Path*	p0 = 0;
	Path*	p1 = &m_pPath[(from - 1 + size) % size];
	Path*	p2 = &m_pPath[from];

	for( int i = 0; i < len; i++ )
	{
		p0 = p1;
		p1 = p2;
		from = (from + 1) % size;
		p2 = &m_pPath[from];

		D13Vec2	v1 = p1->GetPt() - p0->GetPt();
		D13Vec2	v2 = p2->GetPt() - p1->GetPt();

		double	cosAng = (v1 * v2) / (v1.GetLength() * v2.GetLength());
		if( cosAng == 1.0 )
		{
			p1->m_spd = 300;
		}
		else
		{
			double	k = CalcCurvature(p0->GetPt(), p1->GetPt(), p2->GetPt());
			double	speed = CorneringSpeed(1 / k);
			p1->m_spd = min(300, speed);
		}
	}
}

void	D13Path::PropagateBraking( int from, int len, double damage, double fuel )
{
	const int		size = m_pSlices->GetSize();

	const double	cMaxA = MAX_ACCEL;
	const double	cMass = (M + fuel / g);

	int		to = (from + len) % size;

	Path*	pp = &m_pPath[(to + size - 1) % size];
	Path*	p0 = &m_pPath[to];
	Path*	p1 = &m_pPath[(to + 1) % size];
	Path*	p2 = 0;

	for( int i = 0; i < len; i++ )
	{
		p2 = p1;
		p1 = p0;
		to = (to - 1 + size) % size;
		p0 = &m_pPath[to];
		pp = &m_pPath[(to + size - 1) % size];

		if( p0->m_spd > p1->m_spd + 0.01 )
		{
			double	k1 = CalcCurvature(pp->GetPt(), p0->GetPt(), p1->GetPt());
			double	k2 = CalcCurvature(p0->GetPt(), p1->GetPt(), p2->GetPt());
			double	k = (k1 + k2) / 2;
			double	r = k != 0 ? 1 / k : 1e10;
			double	s = (p1->GetPt() - p0->GetPt()).GetLength();
			double	alpha = 2 * asin(s * k * 0.5);
			s = r * alpha;

			double	u = p0->m_spd;
			double	v = p1->m_spd;

			for( int count = 0; ; count++ )
			{
				double	spd = (u + v) / 2;
				double	cen_a = spd * spd * k;

				double	inner = cMaxA * cMaxA - cen_a * cen_a;
				double	tan_a = inner <= 0 ? 0 : -sqrt(inner);

//				double	drag = DRAG_CON * spd * spd *
				double	drag = DRAG_CON * u * u *
								(2 * damage + MAX_DAMAGE) / MAX_DAMAGE;
				double	drag_a = drag / cMass;
				tan_a -= drag_a;

				inner = v * v - 2 * tan_a * s;
				double	max_u = inner <= 0 ? 0 : sqrt(inner);

				if( count >= 10 && u <= max_u )
				{
					if( u * 1.0008 > max_u )
						u = max_u / 1.0008;
					break;
				}

				u = max_u;
			}

//			double	brakeDist = BrakingDistance(u, v);
//			ASSERT( brakeDist <= s );

			if( u < p0->m_spd )
				p0->m_spd = u;
		}
	}
}

double	D13Path::EstimateSpeed( double damage, double fuel ) const
{
	const double	cMaxA = MAX_ACCEL;
	const double	cMass = (M + fuel / g);
	const int		size = m_pSlices->GetSize();

	double	totalTime;
	double*	pSpeed = new double[size];
	pSpeed[size - 1] = m_pPath[size - 1].m_spd;

	for( int j = 0; j < 2; ++j )
	{
		totalTime = 0;

		for( int i = 0, prev = size - 1; i < size; i++ )
		{
			D13Vec2	p0 = m_pPath[prev].GetPt();
			D13Vec2	p1 = m_pPath[i].GetPt();
			double	spd = pSpeed[prev];

			double	dist = (p1 - p0).GetLength();
			double	time = dist / spd;
			totalTime += time;

			if( spd > m_pPath[i].m_spd )
				pSpeed[i] = m_pPath[i].m_spd;
			else
			{
				D13Vec2	p2 = m_pPath[(i + 1) % size].GetPt();

				double	a0 = (p1 - p0).GetAngle();
				double	a1 = (p2 - p1).GetAngle();

				double	alpha = NormaliseAngleLimit(a1 - a0, 0);

				double	cenA = alpha * spd * spd / dist;
				double	inner = cMaxA * cMaxA - cenA * cenA;
				double	maxA = inner <= 0 ? 0 : sqrt(inner);

				double	drag = DRAG_CON * spd * spd *
								(2 * damage + MAX_DAMAGE) / MAX_DAMAGE;
				double	dragA = drag / cMass;
				double	tanA = cos(alpha) * PM / (cMass * spd);

				if( tanA > maxA )
					tanA = maxA;

				double	newSpeed = spd + time * (tanA - dragA);

				pSpeed[i] = min(m_pPath[i].m_spd, newSpeed);
			}

			prev = i;
		}
	}

	delete [] pSpeed;

	return MPH_FPS * m_pSlices->GetLength() / totalTime;
}

class D13OptPath  
{
public:
	enum
	{
		cSegFt = D13Slices::cSliceFt,
	};

	struct	Line
	{
		D13Vec2	m_l;	// start point normal of line
		D13Vec2	m_v;	// direction of normal line
		double	m_w;	// distance along normal line
		D13Vec2	m_p;	// point on path
		double	m_lft;	// buffer from left
		double	m_rgt;	// buffer from right
		double	m_k;	// curvature at this point

		void	Setup( const D13Vec2& l, const D13Vec2& v, double w );
		void	SetW( double w );
	};

	struct BinHeader
	{
		char	version[16];// e.g. "DodgerBin1"
		char	track[16];	// e.g. "monza-76.trk"
		int		size;		// size of whole file... used when concatenated
		int		nLines;		// number of lines stored in this data file
	};

public:
	D13OptPath( const D13Slices& slices );
	~D13OptPath();

	bool		Load( const char* pName );

	void		SetFactor( double factor );
	void		Optimise();
	bool		ModifySection( int from, int len, double delta, int mid = -1, bool urgent = false );
	bool		ModifySectionNew( int from, int len, double delta, int mid = -1 );
	void		ModifySectionCircle( int from, int len, double delta, int mid = -1 );
	bool		ModifySectionAfter( int from, int len, double delta );
	void		CopyToPath( D13Path& path ) const;
	void		SetFromPath( const D13Path& path );

//	int			GetSize() const;
	const Line&	GetAt( int i ) const;
	Line&		GetAt( int i );
//	double		GetWidth() const { return m_width; }

	double		CalcSum() const;
	void		Optimise( int step, int nIterations );

private:
	D13OptPath();

	void		SmoothBetween( int step );
	void		Optimise(	Line* l3, double e,
							const Line* l0, const Line* l1,
							const Line* l2, const Line* l4,
							const Line* l5, const Line* l6 );
private:
	double	m_width;
	int		m_nLines;
	Line*	m_pLines;
	double	m_factor;	// curve shape factor, default = cDefaultFactor
};

void	D13OptPath::Line::Setup( const D13Vec2& l, const D13Vec2& v, double w )
{
	m_l = l;
	m_v = v;
	m_w = w;
	m_p = l + v * w;
	m_lft = 0;
	m_rgt = 0;
	m_k = 0;
}

void	D13OptPath::Line::SetW( double w )
{
	m_w = w;
	m_p = m_l + m_v * w;
}

D13OptPath::D13OptPath( const D13Slices& slices )
:	m_width(1),
	m_nLines(0),
	m_pLines(0),
	m_factor(cDefaultFactor)
{
	// setup working values

	m_width = slices.GetWidth();
	m_nLines = slices.GetSize();
	delete [] m_pLines;
	m_pLines = new Line[m_nLines];

	for( int i = 0; i < m_nLines; i++ )
	{
		const D13Slices::Slice&	s = slices.GetAt(i);
		m_pLines[i].Setup( s.m_p, s.m_n, m_width * 0.5 );
	}
}

D13OptPath::~D13OptPath()
{
	delete [] m_pLines;
}

bool	D13OptPath::Load( const char* pName )
{
	FILE*	pFile = fopen("robots\\dodger13.dat", "rb");
	if( pFile == 0 )
		return false;

	// search for correct data in file
	BinHeader	header;
	for(;;)
	{
		int		pos = ftell(pFile);

		if( fread(&header, sizeof(header), 1, pFile) != 1 )
		{
			fclose( pFile );
			return false;
		}

		if( strcmp(header.version, "DodgerBin1") != 0 )
		{
			fclose( pFile );
			return false;
		}

		if( strcmp(header.track, pName) != 0 )
		{
			// seek to next record...
			fseek( pFile, pos + header.size, SEEK_SET );
			continue;
		}

		// got the right record now, so check that it is OK to use
		if( header.nLines != m_nLines )
		{
			fclose( pFile );
			return false;
		}

		double	w = 0;
		BYTE	b[3];
		BYTE	escape = BYTE(0x80);
		for( int i = 0; i < m_nLines; i++ )
		{
			if( fread(&b[0], 1, 1, pFile) != 1 )
			{
				fclose( pFile );
				return false;
			}

			if( *b == escape )
			{
				fread( b, 3, 1, pFile );
				int		value = (b[0] << 16) + (b[1] << 8) + b[2];
				w = value * 0.0001;
			}
			else
			{
				fread( &b[1], 1, 1, pFile );
				int		delta = (((signed char)b[0]) << 8) + b[1];
				w += delta * 0.0001;
			}

			m_pLines[i].m_w  = w;
			m_pLines[i].m_p = m_pLines[i].m_l + m_pLines[i].m_v * m_pLines[i].m_w;
		}

		break;
	}

	fclose( pFile );

	return true;
}

void	D13OptPath::SetFactor( double factor )
{
	m_factor = factor;
}

void	D13OptPath::Optimise()
{
	// perform optimisation

	const int delta = 25;
	const int n = (150 + delta - 1) / delta;

	int		step = 1;
	while( step * 4 < m_nLines )
		step *= 2;

	do
	{
		step = (step + 1) / 2;

//		int		count = 0;
		double	sum1 = 0;
		double	sum2 = CalcSum() / delta;
		double	error = 0;;
		for( int i = 0; i < n; i++ )
		{
			Optimise( step, delta );

			sum1 = sum2;
			sum2 = CalcSum() / delta;

			error = fabs(sum1 - sum2);

//			if( error < 0.01 )
//				break;

//			++count;
//			if( count > (16 >> step) )
//				break;
		}
	}
	while( step > 1 );
}

bool	D13OptPath::ModifySection(
	int from,
	int len,
	double delta,
	int important,
	bool urgent )
{
	if( important < 0 )
		important = (from + len / 2) % m_nLines;

	// find distances...

	double*	pDist = new double[len];
	pDist[0] = 0;
	int	i;
	for( /*int*/ i = 1; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		int		k = (j - 1 + m_nLines) % m_nLines;
		double	length = (GetAt(j).m_p - GetAt(k).m_p).GetLength();
		pDist[i] = pDist[i - 1] + length;
	}

	int		newFrom = from;
	int		newTo = (from + len - 1) % m_nLines;

	// dry run to find limits, and a better estimate of distance

	double	totalDist1 = pDist[(important + m_nLines - from) % m_nLines];
	double	totalDist2 = pDist[len - 1] - totalDist1;

	D13Vec2	p0 = GetAt(from).m_p;
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist = pDist[i];

		double	angle;
		if( dist < totalDist1 )
			angle = cPi * dist / totalDist1;
		else
			angle = cPi + cPi * (dist - totalDist1) / totalDist2;

		double	offset = (1 - cos(angle)) * 0.5 * delta;

		D13Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		double	w = l1.m_w + offset;
		if( offset < 0 && w < 0.5 ||
			offset > 0 && w > m_width - 0.5 )
		{
			if( i < (important - from + m_nLines) % m_nLines)
				newFrom = j;
			else
			{
				newTo = j;
				break;
			}
		}

		D13Vec2	p1 = l1.m_l + l1.m_v * w;

		if( i > 0 )
		{

			double	length = (p1 - p0).GetLength();
			pDist[i] = pDist[i - 1] + length;
		}

		p0 = p1;
	}

	int		oldFrom = from;
	from = newFrom;
	len = (newTo - newFrom + 1 + m_nLines) % m_nLines;

	int		len1 = (important - newFrom + m_nLines) % m_nLines;
	int		len2 = (newTo - important + m_nLines) % m_nLines;
	if( len1 < 10 || len2 < 10 )
	{
		delete [] pDist;
		return false;
	}

//	totalDist = pDist[(newTo   - oldFrom + m_nLines) % m_nLines] -
//				pDist[(newFrom - oldFrom + m_nLines) % m_nLines];
	totalDist1 = pDist[(important - oldFrom + m_nLines) % m_nLines] -
				 pDist[(newFrom   - oldFrom + m_nLines) % m_nLines];
	totalDist2 = pDist[(newTo     - oldFrom + m_nLines) % m_nLines] -
				 pDist[(important - oldFrom + m_nLines) % m_nLines];

	p0 = GetAt((from - 1 + m_nLines) % m_nLines).m_p;
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist =	pDist[(i + from - oldFrom + m_nLines) % m_nLines] -
						pDist[(from - oldFrom + m_nLines) % m_nLines];

		double	angle;
		if( dist < totalDist1 )
			angle = cPi * dist / totalDist1;
		else
			angle = cPi + cPi * (dist - totalDist1) / totalDist2;

		double	offset = (1 - cos(angle)) * 0.5 * delta;

		D13Vec2	tan = (l2.m_p - p0).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		p0 = l1.m_p;

		l1.m_w += offset;
		l1.m_p = l1.m_l + l1.m_v * l1.m_w;
	}

	delete [] pDist;

	if( !urgent )
	{
		// check we haven't cocked up the path
		double	oldK = 0;
		for( i = len; i >= 0 && i >= len - 10; i-- )
		{
			int		j = (newFrom + i) % m_nLines;
			Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
			Line&	l1 = GetAt(j);
			Line&	l2 = GetAt((j + 1) % m_nLines);

			double	k = CalcCurvature(l0.m_p, l1.m_p, l2.m_p);
			if( i != len && fabs(k - oldK) > 0.0002 )
				return false;
			oldK = k;
		}
	}

	return true;
}

bool	D13OptPath::ModifySectionNew( int from, int len, double delta, int important )
{
	if( important < 0 )
		important = (from + len / 2) % m_nLines;

	// find initial distances estimates...

	double*	pDist = new double[len];
	pDist[0] = 0;
	int	i;
	for( /*int*/ i = 1; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		int		k = (j - 1 + m_nLines) % m_nLines;
		double	length = (GetAt(j).m_p - GetAt(k).m_p).GetLength();
		pDist[i] = pDist[i - 1] + length;
	}

	int		newFrom = from;
	int		newTo = (from + len - 1) % m_nLines;

	// dry run to find limits

	double	fromDist = 0;
	double	impDist = pDist[(important + m_nLines - from) % m_nLines];
	double	toDist = pDist[len - 1];

	double	totalDist1 = impDist - fromDist;
	double	totalDist2 = toDist - impDist;

	while( (newTo - newFrom + m_nLines) % m_nLines > 20 )
	{
		fromDist = pDist[(newFrom + m_nLines - from) % m_nLines];
		toDist = pDist[(newTo + m_nLines - from) % m_nLines];

		totalDist1 = impDist - fromDist;
		totalDist2 = toDist - impDist;

		int		newLen = (newTo - newFrom + m_nLines) % m_nLines;

		int		oldFrom = newFrom;
		int		oldTo = newTo;

		int		fromIndex = (newFrom - from + m_nLines) % m_nLines;

		for( /*int*/ i = 0; i < newLen; i++ )
		{
			int		j = (newFrom + i) % m_nLines;
//			Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
			Line&	l1 = GetAt(j);
//			Line&	l2 = GetAt((j + 1) % m_nLines);

			double	dist = pDist[fromIndex + i] - fromDist;

			double	angle;
			if( dist < totalDist1 )
				angle = cPi * dist / totalDist1;
			else
				angle = cPi + cPi * (dist - totalDist1) / totalDist2;

			double	offset = (1 - cos(angle)) * 0.5 * delta;
			double	w = l1.m_w + offset;

			if( offset < 0 && w < 0.5 ||
				offset > 0 && w > m_width - 0.5 )
			{
				if( i < (important - from + m_nLines) % m_nLines)
					newFrom = j;
				else
				{
//					newTo = j;

					// work out new (smaller) value of len to try
					double	newOffset = w < 0.5 ? 0.5 - l1.m_w : m_width - 0.5 - l1.m_w;
					double	newAngle = c2Pi - acos(1 - 2 * newOffset / delta);

					// new length is proportional to the old and new angles
					dist = totalDist2 * angle / newAngle;
					do
						newTo = (newTo + m_nLines - 1) % m_nLines;
					while( (newTo + m_nLines - important) % m_nLines > 0 &&
							pDist[(newTo + m_nLines - from - 1) % m_nLines] - impDist > dist );
					break;
				}
			}
		}

		if( oldFrom == newFrom && oldTo == newTo )
			break;
	}

//	int		oldFrom = from;
//	from = newFrom;
	len = (newTo - newFrom + 1 + m_nLines) % m_nLines;

	int		len1 = (important - newFrom + m_nLines) % m_nLines;
	int		len2 = (newTo - important + m_nLines) % m_nLines;
	if( len1 < 10 || len2 < 10 )
	{
		delete [] pDist;
		return false;
	}

	// now we make a better estimate of the distance

	fromDist = pDist[(newFrom + m_nLines - from) % m_nLines];
	impDist = pDist[(important + m_nLines - from) % m_nLines];
	toDist = pDist[(newTo + m_nLines - from) % m_nLines];

	totalDist1 = impDist - fromDist;
	totalDist2 = toDist - impDist;

	int		fromIndex = (newFrom - from + m_nLines) % m_nLines;

	D13Vec2	p0 = GetAt(newFrom).m_p;
	for( /*int*/ i = 1; i < len; i++ )
	{
		int		j = (newFrom + i) % m_nLines;
		Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist = pDist[fromIndex + i] - fromDist;

		double	angle;
		if( dist < totalDist1 )
			angle = cPi * dist / totalDist1;
		else
			angle = cPi + cPi * (dist - totalDist1) / totalDist2;

		double	offset = (1 - cos(angle)) * 0.5 * delta;

		D13Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		double	w = l1.m_w + offset;
		D13Vec2	p1 = l1.m_l + l1.m_v * w;

		double	length = (p1 - p0).GetLength();
		pDist[i] = pDist[i - 1] + length;

		p0 = p1;
	}

	// finally, we actually modify the path

	fromDist = pDist[(newFrom + m_nLines - from) % m_nLines];
	impDist = pDist[(important + m_nLines - from) % m_nLines];
	toDist = pDist[(newTo + m_nLines - from) % m_nLines];

	totalDist1 = impDist - fromDist;
	totalDist2 = toDist - impDist;

	p0 = GetAt((from - 1 + m_nLines) % m_nLines).m_p;
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (newFrom + i) % m_nLines;
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist = pDist[fromIndex + i] - fromDist;

		double	angle;
		if( dist < totalDist1 )
			angle = cPi * dist / totalDist1;
		else
			angle = cPi + cPi * (dist - totalDist1) / totalDist2;

		double	offset = (1 - cos(angle)) * 0.5 * delta;

		D13Vec2	tan = (l2.m_p - p0).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		p0 = l1.m_p;

		l1.m_w += offset;
		l1.m_p = l1.m_l + l1.m_v * l1.m_w;
	}

	delete [] pDist;

	// check we haven't cocked up the path
	double	oldK = 0;
	for( i = len + 1; i >= 0 && i >= len - 10; i-- )
	{
		int		j = (newFrom + i) % m_nLines;
		Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	k = CalcCurvature(l0.m_p, l1.m_p, l2.m_p);
		if( i != len + 1 && fabs(k - oldK) > 0.0002 )
			return false;
		oldK = k;
	}

	return true;
}

void	D13OptPath::ModifySectionCircle( int from, int len, double delta, int important )
{
	if( important < 0 )
		important = (from + len / 2) % m_nLines;

	// find distances...

	double*	pDist = new double[len];
	pDist[0] = 0;
	int	i;
	for( /*int*/ i = 1; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		int		k = (j - 1 + m_nLines) % m_nLines;
		double	length = (GetAt(j).m_p - GetAt(k).m_p).GetLength();
		pDist[i] = pDist[i - 1] + length;
	}

	int		newFrom = from;
	int		newTo = (from + len - 1) % m_nLines;

	// dry run to find limits, and a better estimate of distance

	double	totalDist = pDist[len - 1];
	double	r = (totalDist * totalDist / 4 + delta * delta) / (2 * delta);

	D13Vec2	p0 = GetAt(from).m_p;
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist = pDist[i];

		double	distFromMid = dist - totalDist / 2;
		double	offset = sqrt(r * r - distFromMid * distFromMid) - (r - delta);
		offset = r > 0 ? offset : -offset;

		D13Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		double	w = l1.m_w + offset;
		if( offset < 0 && w < 0.5 ||
			offset > 0 && w > m_width - 0.5 )
		{
			return;
			if( i < (important - from + m_nLines) % m_nLines)
				newFrom = j;
			else
			{
				newTo = j;
				break;
			}
		}

		D13Vec2	p1 = l1.m_l + l1.m_v * w;

		if( i > 0 )
		{

			double	length = (p1 - p0).GetLength();
			pDist[i] = pDist[i - 1] + length;
		}

		p0 = p1;
	}

	int		oldFrom = from;
	from = newFrom;
	len = (newTo - newFrom + 1 + m_nLines) % m_nLines;

	int		len1 = (important - newFrom + m_nLines) % m_nLines;
	int		len2 = (newTo - important + m_nLines) % m_nLines;
	if( len1 < 10 || len2 < 10 )
	{
		delete [] pDist;
		return;
	}

	totalDist = pDist[(newTo   - oldFrom + m_nLines) % m_nLines] -
				pDist[(newFrom - oldFrom + m_nLines) % m_nLines];

	p0 = GetAt((from - 1 + m_nLines) % m_nLines).m_p;
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist =	pDist[(i + from - oldFrom + m_nLines) % m_nLines] -
						pDist[(from - oldFrom + m_nLines) % m_nLines];

		double	distFromMid = dist - totalDist / 2;
		double	offset = sqrt(r * r - distFromMid * distFromMid) - (r - delta);
		offset = r > 0 ? offset : -offset;

		D13Vec2	tan = (l2.m_p - p0).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		p0 = l1.m_p;

		l1.m_w += offset;
		l1.m_p = l1.m_l + l1.m_v * l1.m_w;
	}

	delete [] pDist;
}

bool	D13OptPath::ModifySectionAfter( int from, int len, double delta )
{
	// find distances...

	double*	pDist = new double[len];
	pDist[0] = 0;
	int i;
	for( i = 1; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		int		k = (j - 1 + m_nLines) % m_nLines;
		double	length = (GetAt(j).m_p - GetAt(k).m_p).GetLength();
		pDist[i] = pDist[i - 1] + length;
	}

	// dry run to find limits

	double	totalDist = pDist[len - 1];

	while( len > 20 )
	{
		totalDist = pDist[len - 1];
		double	len2 = len;
		for( /*int*/ i = 0; i < len; i++ )
		{
			int		j = (from + i) % m_nLines;
			Line&	l = GetAt(j);

			double	dist = pDist[i];
//			double	angle = cPi * i / len;
			double	angle = cPi * dist / totalDist;
			double	offset = (1 + cos(angle)) * 0.5 * delta;
			double	w = l.m_w + offset;
			if( w < 0.5 || w > m_width - 0.5 )
			{
				// work out new (smaller) value of len to try
				double	newOffset = w < 0.5 ? 0.5 - l.m_w : m_width - 0.5 - l.m_w;
				double	newAngle = acos(2 * newOffset / delta - 1);

				// new length is proportional to the old and new angles
				dist = pDist[len - 1] * angle / newAngle;
				do
					len--;
				while( len > 0 && pDist[len - 1] > dist );
				break;
			}
		}

		if( len2 == len )
			break;
	}

	if( len < 20 )
	{
		delete [] pDist;
		return false;
	}

	// now make a better estimate of the distance
	totalDist = pDist[len - 1];
	D13Vec2	p0 = GetAt(from).m_p;
	for( i = 1; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
		Line&	l1 = GetAt(j);
		Line&	l2 = GetAt((j + 1) % m_nLines);

		double	dist = pDist[i];
//			double	angle = cPi * i / len;
		double	angle = cPi * dist / totalDist;
		double	offset = (1 + cos(angle)) * 0.5 * delta;

		D13Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		double	w = l1.m_w + offset;
		D13Vec2	p1 = l1.m_l + l1.m_v * w;

		double	length = (p1 - p0).GetLength();
		pDist[i] = pDist[i - 1] + length;

		p0 = p1;
	}

	totalDist = pDist[len - 1];
	for( /*int*/ i = 0; i < len; i++ )
	{
		int		j = (from + i) % m_nLines;
		Line&	l = GetAt(j);

		double	dist = pDist[i];
//		double	offset = (1 + cos(cPi * i / len)) * 0.5 * delta;
		double	offset = (1 + cos(cPi * dist / totalDist)) * 0.5 * delta;
		l.m_w += offset;
		l.m_p = l.m_l + l.m_v * l.m_w;
	}

	delete [] pDist;

	{
		// check we haven't cocked up the path
		double	oldK = 0;
		for( i = len + 1; i >= 0 && i >= len - 10; i-- )
		{
			int		j = (from + i) % m_nLines;
			Line&	l0 = GetAt((j - 1 + m_nLines) % m_nLines);
			Line&	l1 = GetAt(j);
			Line&	l2 = GetAt((j + 1) % m_nLines);

			double	k = CalcCurvature(l0.m_p, l1.m_p, l2.m_p);
			if( i != len + 1 && fabs(k - oldK) > 0.0002 )
				return false;
			oldK = k;
		}
	}

	return true;
}

void	D13OptPath::CopyToPath( D13Path& path ) const
{
	for( int i = 0; i < m_nLines; i++ )
		path.GetAt(i).m_w = GetAt(i).m_w;
}

void	D13OptPath::SetFromPath( const D13Path& path )
{
	for( int i = 0; i < m_nLines; i++ )
	{
		Line&	l = GetAt(i);
		l.m_p = path.GetAt(i).GetPt();
		l.m_w = path.GetAt(i).m_w;
		l.m_lft = 0;
		l.m_rgt = 0;
	}
}

const D13OptPath::Line&	D13OptPath::GetAt( int i ) const
{
	return m_pLines[i];
}

D13OptPath::Line&	D13OptPath::GetAt( int i )
{
	return m_pLines[i];
}

double	D13OptPath::CalcSum() const
{
	double	sum = 0;
	for( int i = 0; i < m_nLines; i++ )
	{
		double	n = m_pLines[i].m_w;
		sum += n * n;
	}
	sum /= m_nLines;
//	sum = sqrt(sum);
	return sum * (25.0 / cSegFt);
}

void	D13OptPath::Optimise( int step, int nIterations )
{
	for( int j = 0; j < nIterations; j++ )
	{
		Line*	l0 = 0;
		Line*	l1 = &m_pLines[m_nLines - 3 * step];
		Line*	l2 = &m_pLines[m_nLines - 2 * step];
		Line*	l3 = &m_pLines[m_nLines - step];
		Line*	l4 = &m_pLines[0];
		Line*	l5 = &m_pLines[step];
		Line*	l6 = &m_pLines[2 * step];

		// go forwards
		int		i = 3 * step;
		int		n = (m_nLines + step - 1) / step;
		for( int count = 0; count < n; count++ )
		{
			l0 = l1;
			l1 = l2;
			l2 = l3;
			l3 = l4;
			l4 = l5;
			l5 = l6;
			l6 = &m_pLines[i];

			Optimise( l3, 0, l0, l1, l2, l4, l5, l6 );

			if( (i += step) >= m_nLines )
				i = 0;//i -= m_nLines;
		}
	}

	// now smooth the values between steps
	if( step > 1 )
		SmoothBetween( step );
}

void	D13OptPath::SmoothBetween( int step )
{
	// now smooth the values between steps
	Line*	l0 = 0;
	Line*	l1 = &m_pLines[((m_nLines - 1) / step) * step];
	Line*	l2 = &m_pLines[0];
	Line*	l3 = &m_pLines[step];

	int		j = 2 * step;
	for( int i = 0; i < m_nLines; i += step )
	{
		l0 = l1;
		l1 = l2;	// l1 represents m_pLines[i];
		l2 = l3;
		l3 = &m_pLines[j];

		j += step;
		if( j >= m_nLines )
			j = 0;

		double	k1 = CalcCurvature(l0->m_p, l1->m_p, l2->m_p);
		double	k2 = CalcCurvature(l1->m_p, l2->m_p, l3->m_p);

		if( i + step > m_nLines )
			step = m_nLines - i;

		for( int k = 1; k < step; k++ )
		{
			double	t;
			Line&	l = m_pLines[(i + k) % m_nLines];
			LineCrossesLine(l.m_l, l.m_v, l1->m_p, l2->m_p - l1->m_p, t);
			l.m_w = t;
			l.m_p = l.m_l + l.m_v * t;

			double	len1 = (l.m_p - l1->m_p).GetLength();
			double	len2 = (l.m_p - l2->m_p).GetLength();
			double	kappa = (k1 * len2 + k2 * len1) / (len1 + len2);

			if( kappa != 0 )
			{
				double	delta = 0.0001;
				double	deltaK = CalcCurvature(l1->m_p, l.m_l + l.m_v * (t + delta), l2->m_p);
				t += delta * kappa / deltaK;
			}

			const double	cInnerGap = 0;
			if( t < cInnerGap )
				t = cInnerGap;
			else if( t > m_width - cInnerGap )
				t = m_width - cInnerGap;

			l.m_w = t;
			l.m_p = l.m_l + l.m_v * t;
		}
	}
}

void	D13OptPath::Optimise(
	Line*		l3,
	double		/*e*/,

	const Line* l0,
	const Line* l1,
	const Line*	l2,
	const Line* l4,
	const Line*	l5,
	const Line* l6 )
{
	double	factor = m_factor;

	double	k1 = CalcCurvature(l1->m_p, l2->m_p, l3->m_p);
	double	k2 = CalcCurvature(l3->m_p, l4->m_p, l5->m_p);

	double	length1 = (l3->m_p - l2->m_p).GetLength();
	double	length2 = (l4->m_p - l3->m_p).GetLength();

	if( k1 * k2 > 0 )
	{
		double	k0 = CalcCurvature(l0->m_p, l1->m_p, l2->m_p);
		double	k3 = CalcCurvature(l4->m_p, l5->m_p, l6->m_p);
		if( k0 * k1 > 0 && k2 * k3 > 0 )
		{
			if( fabs(k0) < fabs(k1) && fabs(k1) * 1.02 < fabs(k2) )
			{
				k1 *= factor;
			}
			else if( fabs(k0) > fabs(k1) * 1.02 && fabs(k1) > fabs(k2) )
			{
				k1 *= factor;
			}
		}
	}
	else if( k1 * k2 < 0 )
	{
		double	k0 = CalcCurvature(l0->m_p, l1->m_p, l2->m_p);
		double	k3 = CalcCurvature(l4->m_p, l5->m_p, l6->m_p);
		if( k0 * k1 > 0 && k2 * k3 > 0 )
		{
			if( fabs(k1) < fabs(k2) && fabs(k1) < fabs(k3) )
			{
				k1 = (k1 * 0.25 + k2 * 0.75);
			}
			else if( fabs(k2) < fabs(k1) && fabs(k2) < fabs(k0) )
			{
				k2 = (k2 * 0.25 + k1 * 0.75);
			}
		}
	}

	double	k = (length2 * k1 + length1 * k2) / (length1 + length2);

	double	t = l3->m_w;
	LineCrossesLine(l3->m_l, l3->m_v, l2->m_p, l4->m_p - l2->m_p, t);
	double	delta = 0.0001;
	double	deltaK = CalcCurvature(l2->m_p, l3->m_l + l3->m_v * (t + delta), l4->m_p);
	t += delta * k / deltaK;

	const double	straightK = 0.000001;
	if( fabs(k) < straightK )
	{
		if( t < cMinGap + l3->m_rgt )
			t = cMinGap + l3->m_rgt;
		else if( t > m_width - cMinGap - l3->m_lft)
			t = m_width - cMinGap - l3->m_lft;
	}
	else
	{
		double	buffer = cMinGap + (cMaxGap - cMinGap) * length1 * 20 *
								(fabs(k) - straightK);
		if( buffer > cMaxGap )
			buffer = cMaxGap;

		if( k > 0 )
		{
			if( t < buffer + l3->m_rgt )
			{
				if( l3->m_w < buffer + l3->m_rgt )
					t = max(l3->m_w, t);
				else
					t = buffer + l3->m_rgt;
			}
			else if( t > m_width - cMinGap - l3->m_lft )
				t = m_width - cMinGap - l3->m_lft;
		}
		else
		{
			if( t < cMinGap + l3->m_rgt )
				t = cMinGap + l3->m_rgt;
			else if( t > m_width - buffer - l3->m_lft )
			{
				if( l3->m_w > m_width - buffer - l3->m_lft )
					t = min(l3->m_w, t);
				else
					t = m_width - buffer - l3->m_lft;
			}
		}
	}

	l3->m_w = t;
	l3->m_p = l3->m_l + l3->m_v * t;
	l3->m_k = k;
}

class D13CarPaths
{
public:
	struct	CarInfo
	{
		int		m_last;			// index of last path segment for this car
		D13Vec2	m_lastPt;
		double	m_lastSpeed;
		D13Path	m_path;			// path taken by this car
	};

public:
	D13CarPaths( int myCarId, const D13Slices& slices, const D13OptPath& opt );
	~D13CarPaths();

	void			Setup( int myCarId );
	const D13Slices&	GetSlices() const				{ return m_slices; }
	int				GetLast( int carId ) const		{ return m_car[carId].m_last; }
	const D13Path&	GetPath( int carId ) const		{ return m_car[carId].m_path; }
	D13Path&			GetPath( int carId )			{ return m_car[carId].m_path; }
	const D13Path&	GetBestPath( int carId ) const	{ return m_carBest[carId].m_path; }
	D13Path&			GetBestPath( int carId )		{ return m_carBest[carId].m_path; }
	const D13Path&	GetOptPath() const				{ return m_optPath; }
	D13Path&			GetOptPath()					{ return m_optPath; }
	const D13Path&	GetBestPath() const				{ return m_bestPath; }
	void			SetBestPath();
	int				GetBestCar() const				{ return m_bestCar; }
	double			GetBestSpeed() const			{ return m_bestSpeed; }
	void			Record( int carId, const ::Car* pCar );
	void			StartRecord(	int carId, int cur_s,
									const D13Vec2& carPt, double speed );
	void			Record( int carId, const D13Vec2& carPt, double speed );

private:
	D13CarPaths&		operator=( const D13CarPaths& paths );

private:
	int				m_myId;
	const D13Slices&	m_slices;	// for path segment info

	int		m_nCars;
	int		m_laps[MAX_CARS];	// how many laps has this car done
	CarInfo	m_car[MAX_CARS];		// path info for each car
	CarInfo	m_carBest[MAX_CARS];	// best path for each car

	D13Path	m_optPath;			// optimised path for dodger

	int		m_bestCar;			// car which holds best lap
	double	m_bestSpeed;		// speed of best lap
	D13Path	m_bestPath;			// best lap of race by any car
};

D13CarPaths::D13CarPaths( int myCarId, const D13Slices& slices, const D13OptPath& opt )
:	m_myId(myCarId),
	m_slices(slices),
	m_nCars(car_count),
	m_bestCar(-1),
	m_bestSpeed(0)
{
	int	i;
	for( /*int*/ i = 0; i < car_count; i++ )
	{
		m_laps[i] = 0;
		m_car[i].m_last = -1;
		m_car[i].m_lastSpeed = 0;
		m_car[i].m_path.Setup( &slices );
		m_carBest[i].m_lastSpeed = 0;
		m_carBest[i].m_path.Setup( &slices );
	}

	m_optPath.Setup( &slices );
	m_bestPath.Setup( &slices );

	// figure out the optimised path for dodger
	opt.CopyToPath( m_optPath );
	m_optPath.CalcSpeed();

	// initialise all paths to the path of dodger
	for( /*int*/ i = 0; i < car_count; i++ )
	{
		m_car[i].m_path = m_optPath;
		m_carBest[i].m_path = m_optPath;
	}

	m_bestPath = m_optPath;
}

D13CarPaths::~D13CarPaths()
{
}

void	D13CarPaths::SetBestPath()
{
	// perform a small optimisation to smooth the path slightly
//	D13OptPath	opt(m_slices);
//	opt.SetFromPath( m_bestPath );
//	opt.Optimise( 1, 10 );

	// copy the other guys path into our path...
//	opt.CopyToPath( m_optPath );
//	m_optPath.CalcSpeed();
	m_optPath = m_bestPath;
	m_car[m_myId].m_path = m_bestPath;
}

void	D13CarPaths::Record( int carId, const ::Car* pCar )
{
	if( pCar->Out )
		return;

	// work out position and speed of car;
	D13Vec2	carPt(pCar->X, pCar->Y);
	double	speed = const_cast<Car*>(pCar)->get_speed();

	Record( carId, carPt, speed );

	if( pCar->Laps > 0 && pCar->Laps != m_laps[carId] )
	{
		// car has just finished a new lap
		m_laps[carId] = pCar->Laps;

		// check if car is the fastest car in the race
		if( pCar->Bestlap_speed > m_bestSpeed )
		{
			m_bestCar	= carId;
			m_bestPath  = m_car[carId].m_path;
			m_bestSpeed = pCar->Bestlap_speed;

			// perform a small optimisation to smooth the path slightly
			D13OptPath	opt(m_slices);
			opt.SetFromPath( m_bestPath );
			opt.Optimise( 1, 10 );
			opt.CopyToPath( m_bestPath );
			m_bestPath.CalcSpeed();
		}

		// check if this car has just done it's fastest lap
		if( pCar->Lastlap_speed > m_carBest[carId].m_lastSpeed )
		{
			m_carBest[carId].m_path = m_car[carId].m_path;
			m_carBest[carId].m_lastSpeed = pCar->Lastlap_speed;
		}
		else
			m_car[carId].m_path = m_carBest[carId].m_path;
	}
}

void	D13CarPaths::StartRecord(
	int carId, int cur_s, const D13Vec2& carPt, double speed )
{
	m_car[carId].m_last = cur_s;
	m_car[carId].m_lastPt = carPt;
	m_car[carId].m_lastSpeed = speed;
}

void	D13CarPaths::Record( int carId, const D13Vec2& carPt, double speed )
{
	// work out which slice the car is in
	int		last_s = m_car[carId].m_last;
	int		cur_s = m_slices.Find(carPt, pcar[carId]->Seg_id, last_s);

	if( !pcar[carId]->On_pit_lane &&
		last_s >= 0 && last_s != cur_s )
	{
		// we have crossed at least one line boundary, so we need to calculate
		//	the crossing point(s), and the speed at that point.

		D13Vec2	lastPt = m_car[carId].m_lastPt;
		double	lastSpeed = m_car[carId].m_lastSpeed;

		int		next_s = (last_s + 1) % m_slices.GetSize();
		while( last_s != cur_s )
		{
			const D13Slices::Slice&	s0 = m_slices.GetAt(next_s);
			double		t;
			if( LineCrossesLine(lastPt, carPt - lastPt, s0.m_p, s0.m_n, t) &&
				t >= 0.0 && t <= 1.0 )
			{
				D13Vec2	crossPt = lastPt + (carPt - lastPt) * t;
				double	crossSpeed = lastSpeed + (speed - lastSpeed) * t;

				D13Path::Path& path = m_car[carId].m_path.GetAt(next_s);
				path.m_w	= s0.CalcW(crossPt);
				path.m_spd	= crossSpeed;
			}

			last_s = next_s;
			next_s = (next_s + 1) % m_slices.GetSize();
		}
	}

	m_car[carId].m_last = cur_s;
	m_car[carId].m_lastPt = carPt;
	m_car[carId].m_lastSpeed = speed;
}

/////////////////////////////////////////////////////////////////////////

static void	PitControl(
	const situation&	s,
	const D13Track&		track,
	con_vec&			control,
	int&				damage,
	double				timeDamageRate )
{
	static int		totalDamage = 0;
	static int		lastDamage = 0;
	static int		lastRepairLap = 0;
	static double	totalFuel = 0;
	static double	fuelPerLap = 10;
	static double	lastFuel = 0;

	if( s.starting )
	{
		if( s.stage == QUALIFYING )
		{
			control.fuel_amount = 30;
			control.request_pit = false;
			control.repair_amount = 0;
		}
		else
		{
			double	mplb = 0.70;
			double	raceMiles = track.GetLength() * s.laps_to_go / 5180;
			double	fuelForRace = raceMiles / mplb;
			control.fuel_amount = min(fuelForRace, MAX_FUEL);
			control.request_pit = false;
			control.repair_amount = 0;
		}

		totalDamage = 0;
		lastDamage = 0;
		lastRepairLap = 0;
		totalFuel = 0;
		fuelPerLap = 10;
		lastFuel = control.fuel_amount;

		return;
	}

	damage = 0;
	if( int(s.damage) > lastDamage )
	{
		// accumulate damage since start, or last repair
		damage = int(s.damage) - lastDamage;
		totalDamage += damage;
	}
	else if( int(s.damage) < lastDamage )
	{
		// must be in the pits... reset the average damage
		//	calculation.  this is done because in most cases
		//	the first few laps of the race produce the most
		//	damage per lap.  if we are in the pits now, then
		//	we don't want to keep the inflated average anymore.
		lastRepairLap = s.laps_done;
		totalDamage = 0;
	}
	lastDamage = s.damage;

	if( s.fuel < lastFuel )
		totalFuel += lastFuel - s.fuel;
	lastFuel = s.fuel;

	if( s.lap_flag )
	{
		if( s.laps_done >= 1 )
			fuelPerLap = totalFuel / s.laps_done;
	}

	int		lapsSinceRepair = s.laps_done - lastRepairLap;
	double	aveDamage = totalDamage / max(1, lapsSinceRepair);
	double	predDamage = s.damage + aveDamage * (s.laps_to_go + 2);
	double	allowedDamage = 15000;
	double	timeToRepair = s.damage * cPitSecsPerDamage;
	double	timeLossRace = s.damage * s.laps_to_go * timeDamageRate;
	bool	repairDamage =	s.damage > allowedDamage ||
							s.damage > 2000 && timeToRepair < timeLossRace;

	int		lapsOnFullTank = int(MAX_FUEL / fuelPerLap);
	int		lapsOnFuel = int(1.1 * s.fuel / fuelPerLap);
	int		nStopsIfRefuelNow = (s.laps_to_go + lapsOnFullTank) /
									lapsOnFullTank;
	int		nStopsIfRefuelLater = (s.laps_to_go - lapsOnFuel +
									lapsOnFullTank - 1) / lapsOnFullTank;
	bool	delayStop = nStopsIfRefuelNow > nStopsIfRefuelLater;
	if( repairDamage && s.damage + aveDamage < 20000 && delayStop )
		repairDamage = false;

	double	trackLen = get_track_description().length;
	double	fuelToEndOfLap = (1 - s.distance / trackLen) * fuelPerLap;
	bool	canFinishLap = fuelToEndOfLap < s.fuel;
	if( s.stage != QUALIFYING && (s.laps_done > 1 || s.damage > 15000) &&
		(s.laps_to_go > 1 || !canFinishLap) &&
		(repairDamage || s.fuel < fuelPerLap * 1.2 ) )
	{
		// ok, we need to go into the pits now
		control.request_pit = true;

		// work out the amount of fuel we want
		double	fuelToEnd = min(MAX_FUEL, fuelPerLap * (s.laps_to_go + 1));
		control.fuel_amount = max(0, fuelToEnd - s.fuel);

		// we can repair some damage for "free" while the fuel is
		//	going in...
		double	freeRepair = (control.fuel_amount - s.fuel) *
								cPitSecsPerFuel / cPitSecsPerDamage;

		// limit repair to min=freeRepair and max=s.damage
		double	repair = predDamage - allowedDamage;
		if( timeToRepair < timeLossRace )
			repair = s.damage;
		control.repair_amount = (int)min(max(freeRepair, repair), s.damage);
	}
	else
		control.request_pit = false;
//	control.request_pit = true;
}

/////////////////////////////////////////////////////////////////////////////

struct	D13Situation
{
	D13Vec2	m_carPt;		// global car coordinates
	D13Vec2	m_carDir;		// global car unit direction vector
	double	m_carAngle;		// global car angle
	double	m_tanDist;		// distance till tangent point

	D13Vec2	m_distDir;		// direction to most distant point down track
	double	m_curvature;	// curvature at current position on path
	int		m_slice;		// current position on path
	bool	m_nextTurnIsLeft;	// true if will be turning left
};

struct	D13Control
{
	bool	m_colliding;	// true if we are colliding right now
	bool	m_needToBrake;	// true if we need to brake
	bool	m_badCollision;	// true if collision is bad
	double	m_sideCollision;// true if collision is into side
	double	m_predDamage;	// predicted amount of damage from collision
	bool	m_edgeCollision;// true if collision may push us off track
	double	m_adjust;		// amount to adjust path around track
	int		m_important;	// relative important place to adjust ahead
	double	m_vc;			// new control speed required
};

/////////////////////////////////////////////////////////////////////////////

static bool	CheckOtherIn( const D13Vec2& p, double sine, double cosine, double f = 1 )
{
	const double	cCarWid = CARWID * f;
	const double	cCarLen = CARLEN * f;// * (1 + (1 - f) * 0.5);

	D13Vec2	l = D13Vec2(cosine, sine) * (cCarLen / 2);
	D13Vec2	w = D13Vec2(sine, -cosine) * (cCarWid / 2);

	D13Vec2	rl = p - l - w;
	if( rl.x > -cCarWid / 2 && rl.x < cCarWid / 2 &&
		rl.y > -cCarLen / 2 && rl.y < cCarLen / 2 )
		return true;

	D13Vec2	rr = p - l + w;
	if( rr.x > -cCarWid / 2 && rr.x < cCarWid / 2 &&
		rr.y > -cCarLen / 2 && rr.y < cCarLen / 2 )
		return true;

	D13Vec2	fl = p + l - w;
	if( fl.x > -cCarWid / 2 && fl.x < cCarWid / 2 &&
		fl.y > -cCarLen / 2 && fl.y < cCarLen / 2 )
		return true;

	D13Vec2	fr = p + l + w;
	if( fr.x > -cCarWid / 2 && fr.x < cCarWid / 2 &&
		fr.y > -cCarLen / 2 && fr.y < cCarLen / 2 )
		return true;

	return false;
}

/////////////////////////////////////////////////////////////////////////////

static bool	CheckCollide( const D13Vec2& p, double angle, double f = 1 )
{
	if( p * p >= cCarDiag * cCarDiag * f * f )
		return false;

	if( p * p < CARWID * CARWID)
		return true;

	double	s = sin(angle + cPi_2);
	double	c = cos(angle + cPi_2);

	if( CheckOtherIn(p, s, c, f) )
		return true;

	D13Vec2	p2(p.y * c - p.x * s, -p.x * c - p.y * s);
	return CheckOtherIn(p2, -s, c, f);
}

/////////////////////////////////////////////////////////////////////////////

static double	Distance( double vel, double accel, double t )
{
	if( vel * accel < 0 && fabs(vel) < fabs(0.5 * accel * t) )
		// limit t to point where we stopped
		t = fabs(2 * vel / accel);

	return (vel + 0.5 * accel) * t;
}

/////////////////////////////////////////////////////////////////////////
//
//	class	D13CarSim
//
//	used to perform a lightweight simulation of the paths of other
//	cars from the best information we have.
//
const int		cMaxSimTime = 4;		// seconds
const double	cSimDeltaTime = delta_time;
const double	cMaxSimDouble = cMaxSimTime / cSimDeltaTime;
const int		cMaxSim = 72;

class	D13CarSim
{
public:
	enum
	{
		cAutoSim,	// simulate using automatic type
		cPathSim,	// simulate using recorded path data
		cLineSim,	// simulate using a straight line
		cArcSim,	// simulate using an arc
	};

public:
	D13CarSim( int id, const D13Track& track, const D13Slices& slices,
				const D13Path& path, ::Car* pCar,
				int simType = cPathSim );

	// ask to update from new car data
	void	Update( int simType, rel_state& state );

	void	ResetTime();	// but time back to 0.0

	void	Forward();		// simulate forward to next time point
	void	Forward( const D13Vec2& velocity );
	void	Backward();		// retract simulation to previous time point

	bool	Colliding( const D13CarSim& sim ) const;
	double	Distance( const D13CarSim& sim ) const;

	int		GetCur() const	{ return m_cur[m_curSim]; }
	double	GetDist() const	{ return m_dist[m_curSim]; }
	double	GetSpeed() const{ return m_vel[m_curSim].GetLength(); }
	D13Vec2	GetPt() const	{ return m_pt[m_curSim]; }
	D13Vec2	GetVel() const	{ return m_vel[m_curSim]; }
	double	GetW() const	{ return m_w[m_curSim]; }
	D13Vec2	GetDir() const	{ return m_vel[m_curSim].GetUnit(); }
	double	GetCurvature() const;
	int		GetNSlices() const;

	void	SetupRelState( const D13CarSim& base, rel_state& state ) const;

private:
	void	ForwardPath();		// simulate using recorded path
	void	ForwardLine();		// simulate using a straight line
	void	ForwardArc();		// simulate using an arc

public:
	int					m_simType;	// type of simulation to use
	int					m_id;		// id of this car
	const D13Track&		m_track;	// track we are using
	const D13Slices&	m_slices;
	const D13Path&		m_path;
	::Car*				m_pCar;		// would like to be const, but member
									// functions of Car not setup for this

	bool				m_out;					// if true, ignore this car
	int					m_nSim;					// num of sim steps calculated
	int					m_curSim;				// current sim step
	int					m_cur[cMaxSim];			// current slice
	double				m_dist[cMaxSim];		// distance travelled
	D13Vec2				m_pt[cMaxSim];			// 2d pos in global coords
	double				m_w[cMaxSim];			// purpendicular pos in track
	D13Vec2				m_vel[cMaxSim];			// simulated velocity
	double				m_uncertainty[cMaxSim];	// 0.0 = certain;
												// 1.0 = totally uncertain
};

D13CarSim::D13CarSim(
	int					id,
	const D13Track&		track,
	const D13Slices&		slices,
	const D13Path&		path,
	::Car*				pCar,
	int					simType )
:	m_simType(simType),
	m_id(id),
	m_track(track),
	m_slices(slices),
	m_path(path),
	m_pCar(pCar),
	m_out(false),
	m_nSim(1),
	m_curSim(0)
{
	m_cur[0] = 0;
//	Update( m_simType, state );
}

void	D13CarSim::Update( int simType, rel_state& state )
{
	D13Vec2	pt(m_pCar->X, m_pCar->Y);
	int		cur = m_slices.Find(pt, m_pCar->Seg_id, m_cur[0]);

	double	dist, k;
	D13Vec2	vel;
	m_path.Calc( cur, pt, dist, vel, k );

	if( simType == cAutoSim )
	{
		simType = cPathSim;

		if( m_pCar->Laps < 1 || fabs(dist) > 2 ||
			fabs(m_pCar->get_speed() - vel.GetLength()) > 2 )
		{
			simType = cLineSim;
		}
	}

	if( simType == cPathSim && m_simType == cPathSim &&
		m_nSim > 10 && fabs(dist) < 2 &&
		(m_pt[1] - pt).GetLength() < 0.1 &&
		fabs(m_pCar->get_speed() - m_vel[1].GetLength()) < 0.1 )
	{
		// re-use previously calculated data
		m_nSim--;
		m_curSim = 0;
		memmove( &m_pt[0], &m_pt[1], m_nSim * sizeof(m_pt[0]) );
		memmove( &m_cur[0], &m_cur[1], m_nSim * sizeof(m_cur[0]) );
		memmove( &m_w[0], &m_w[1], m_nSim * sizeof(m_w[0]) );

		double	diff = m_dist[1] - m_dist[0];
		for( int i = 1; i < m_nSim; i++ )
			m_dist[i - 1] = m_dist[i] - diff;

		memmove( &m_vel[0], &m_vel[1], m_nSim * sizeof(m_vel[0]) );
		memmove( &m_uncertainty[0], &m_uncertainty[1],
					m_nSim * sizeof(m_uncertainty[0]) );
	}
	else
	{
		// re-initialise data
		m_simType = simType;
		m_nSim = 1;
		m_curSim = 0;
		m_pt[0] = pt;
		m_cur[0] = cur;
		const D13Slices::Slice&	s0 = m_slices.GetAt(cur);
		m_w[0] = (pt - s0.m_p) * s0.m_n;
		m_dist[0] = 0;
		m_vel[0] = m_track.CalcCarVel(state);
		m_uncertainty[0] = 0.0;
	}
}

void	D13CarSim::ResetTime()
{
	m_curSim = 0;
}

void	D13CarSim::Forward()
{
	if( m_curSim >= cMaxSim - 1)
		return;
	
	if( m_simType == cLineSim )
	{
		// simulate using straight line
		ForwardLine();
	}
	else
	{
		// simulate using path data
		ForwardPath();
	}
}

void	D13CarSim::ForwardPath()
{
	const int	prevSim = m_curSim++;
	if( m_curSim < m_nSim )
		return;

	int		prev = m_cur[prevSim];
	int		cur = (prev + 1) % m_slices.GetSize();

	D13Vec2	p = m_path.GetAt(prev).GetPt();
	D13Vec2	q = m_path.GetAt(cur).GetPt();
	D13Vec2	l = q - p;

	double	dist, k;
	D13Vec2	vel;
	m_path.Calc(prev, m_pt[prevSim], dist, vel, k );
	double	lineSpd = vel.GetLength();

	double	spd = m_vel[prevSim].GetLength();
	D13Vec2	v;	// we are trying to calculate this

	double	servo = -dist / 100;
	if( servo < -0.9 )
		servo = -0.9;
	else if( servo > 0.9 )
		servo = 0.9;
	double	servoAng = atan(servo);
	double	lineAng = l.GetAngle();

	v = D13Vec2::FromAngle(lineAng + servoAng) * lineSpd;
	D13Vec2	A = v - m_vel[prevSim];
	D13Vec2	t = m_vel[prevSim].GetUnit();
	D13Vec2	n = t.GetNormal();

	double	mass = M + m_pCar->get_fuel() / g;
	double	damage = m_pCar->Damage;
	double	drag = DRAG_CON * spd * spd * (2 * damage + MAX_DAMAGE) /
						MAX_DAMAGE;

	double	w = (m_pt[prevSim] - m_slices.GetAt(prev).m_p) *
					m_slices.GetAt(prev).m_n;
	if( w < 0 || w > m_slices.GetWidth() )
		drag += (0.6 + .008 * spd) * mass * g;

	double	Ad = drag / mass * cSimDeltaTime;

	double	Tn = A * n;
	double	Tt = A * t + Ad;
	double	Et = PM / (spd * mass) * cSimDeltaTime;
	if( Tt > Et )
		Tt = Et;

	D13Vec2	T = t * Tt + n * Tn;
	double	Amax = 2 * MAX_ACCEL * cSimDeltaTime;
	if( T.GetLength() > Amax )
		T = T.GetUnit() * Amax;

	double	At = T * t - Ad;
	double	An = T * n;

	A = t * At + n * An;
	v = m_vel[prevSim] + A;

	// adjust max simulation value
	m_nSim++;
	ASSERT( m_nSim <= cMaxSim );

	// fill in values for next simulated position and velocity.
	m_pt[m_curSim] = m_pt[prevSim] + v * cSimDeltaTime;
	double	length = (m_pt[m_curSim] - m_pt[prevSim]).GetLength();
	m_dist[m_curSim] = m_dist[prevSim] + length;
	m_cur[m_curSim] = m_slices.Find(m_pt[m_curSim], m_cur[prevSim]);
	m_vel[m_curSim] = v;
	m_w[m_curSim] = w;
	m_uncertainty[m_curSim] = 0.0;
}

void	D13CarSim::ForwardLine()
{
	if( ++m_curSim < m_nSim )
		return;

	D13Vec2	pt = m_pt[0];
	double	speed = m_vel[0].GetLength();
	D13Vec2	dir = m_vel[0] / speed;
	double	accel = pcar[m_id]->get_lin_acc() * g * 0.5;
	int		cur = m_cur[0];
	double	dist = 0;

	for( int i = 0; i < cMaxSim; i++ )
	{
		m_cur[i] = cur;
		m_pt[i] = pt;
		m_vel[i] = dir * speed;
		m_dist[i] = dist;
		m_w[i] = m_slices.GetAt(cur).CalcW(pt);

		speed += accel * cSimDeltaTime;
		if( speed < 10 )
			speed = 10;

		pt = pt + dir * (speed * cSimDeltaTime);
		cur = m_slices.Find(pt, m_slices.GetAt(cur).m_seg, cur);
		dist += speed * cSimDeltaTime;
	}

	m_nSim = cMaxSim;
}

void	D13CarSim::Forward( const D13Vec2& velocity )
{
	const int	prevSim = m_curSim++;
	if( m_curSim >= m_nSim )
	{
		// adjust max simulation value
		m_nSim = m_curSim + 1;
		ASSERT( m_nSim <= cMaxSim );
	}

	// need to extrapolate this cars path forward into future using
	//	the velocity we have been passed.
	m_pt[m_curSim] = m_pt[prevSim] + velocity * cSimDeltaTime;
	double	length = (m_pt[m_curSim] - m_pt[prevSim]).GetLength();
	m_dist[m_curSim] = m_dist[prevSim] + length;
	m_cur[m_curSim] = m_slices.Find(m_pt[m_curSim], m_cur[prevSim]);
	m_vel[m_curSim] = velocity;
	m_w[m_curSim] = m_slices.GetWidth() * 0.5;
	m_uncertainty[m_curSim] = 0.0;
}

void	D13CarSim::Backward()
{
	ASSERT( m_curSim > 0 );
	m_curSim--;
}

bool	D13CarSim::Colliding( const D13CarSim& sim ) const
{
	double	uFactor = 1.0 + m_uncertainty[sim.m_curSim];
	uFactor *= uFactor;

	D13Vec2	relPt = sim.m_pt[sim.m_curSim] - m_pt[m_curSim];
	double	distSquared = relPt * relPt;
	if( distSquared > cCarDiag * cCarDiag * uFactor )
		return false;

	D13Vec2	dir0 = m_vel[m_curSim].GetUnit();
	D13Vec2	dir1 = sim.m_vel[sim.m_curSim].GetUnit();

	D13Vec2	pt0 = m_pt[m_curSim] - dir0 * CARWID * 0.5;	// rear
	D13Vec2	pt1 = m_pt[m_curSim] + dir0 * CARWID * 0.5;	// front
	D13Vec2	pt2 = sim.m_pt[sim.m_curSim] - dir1 * CARWID * 0.5;
	D13Vec2	pt3 = sim.m_pt[sim.m_curSim] + dir1 * CARWID * 0.5;

	double	rr = 17 * 17 * uFactor;
	return	(pt1 - pt2) * (pt1 - pt2) < rr ||	// front hits rear
			(pt0 - pt3) * (pt0 - pt3) < rr ||	// rear hits front
			(pt0 - pt2) * (pt0 - pt2) < rr ||	// rear hits rear
			(pt1 - pt3) * (pt1 - pt3) < rr;		// front hits front
}

double	D13CarSim::Distance( const D13CarSim& sim ) const
{
	D13Vec2	relPt = sim.m_pt[sim.m_curSim] - m_pt[m_curSim];
	return relPt.GetLength();
}

double	D13CarSim::GetCurvature() const
{
	double	dist, k;
	D13Vec2	vel;
	m_path.Calc(m_cur[m_curSim], m_pt[m_curSim], dist, vel, k );
	return k;
}

int		D13CarSim::GetNSlices() const
{
	const int	nSlices = m_slices.GetSize();
	return (m_cur[m_curSim] - m_cur[0] + nSlices) % nSlices;
}

void	D13CarSim::SetupRelState(
	const D13CarSim&	base,
	rel_state&		state ) const
{
	double	speed = GetVel().GetLength();
	D13Vec2	relPt = GetPt() - base.GetPt();
	D13Vec2	relVel = GetVel() - base.GetVel();
	D13Vec2	relYDir = base.GetDir();
	D13Vec2	relXDir = -relYDir.GetNormal();

	state.alpha = 0;
	state.braking = m_curSim > 0 && m_vel[m_curSim - 1].GetLength() > speed;
	state.coming_from_pits = 0;
	state.for_position = 0;
	state.rel_x = relPt * relXDir;
	state.rel_xdot = relVel * relXDir;
	state.rel_y = relPt * relYDir;
	state.rel_ydot = relVel * relYDir;
	double	w = max(-1, min(GetW(), m_slices.GetWidth() + 1));
	state.to_lft = m_slices.GetWidth() - w;
	state.to_rgt = w;
	state.v = speed;
	state.vn = GetVel() * m_slices.GetAt(GetCur()).m_n;
	state.who = m_id;
}

/////////////////////////////////////////////////////////////////////////////

static bool	AvoidOtherCars(
	const situation&	s,				// current situation
	const D13Situation&	mySituation,	// global situation values
	const D13Track&		track,			// track we are using
	const D13CarPaths&	carPaths,		// paths of all cars

	D13Control&			myControl )		// returned instructions
{
	const int nSlices = carPaths.GetSlices().GetSize();

	myControl.m_badCollision = false;
	myControl.m_sideCollision = 0;
	myControl.m_edgeCollision = false;
	myControl.m_adjust = 0;
	myControl.m_important = 0;

	double		maxColV = -1;
	bool		compSpd = false;
	bool		gotCar = false;
	rel_state	colCar;			// car we are going to try to avoid
	double		colTime = 10;
	rel_state	colMyCar;
	rel_state	colHisCar;
	D13Vec2		colPos1;
	D13Vec2		colDir1;
	D13Vec2		colPos2;
	D13Vec2		colDir2;
	double		colSpd2;

	static D13CarSim*	pCarSim[MAX_CARS];
	if( pCarSim[0] == 0 )
	{
		// initialise
		for( int i = 0; i < car_count; i++ )
		{
			pCarSim[i] = new D13CarSim(i, track, carPaths.GetSlices(),
								carPaths.GetPath(i),
								pcar[i]);
		}
	}

	const int	me = s.my_ID;
	const D13Path&	myPath = carPaths.GetPath(me);
	D13CarSim&	mySim = *pCarSim[me];
	rel_state	myRelState;
	myRelState.v = s.v;
	myRelState.vn = s.vn;
	myRelState.who = me;
	mySim.Update( D13CarSim::cPathSim, myRelState );
	D13Vec2		myRight = -mySituation.m_carDir.GetNormal();
	D13Vec2		myPt = mySituation.m_carPt;
	int			mySlice = mySituation.m_slice;

	double	k = CalcCurvature(myPath.GetAt(mySlice).GetPt(),
					myPath.GetAt((mySlice + 1) % nSlices).GetPt(),
					myPath.GetAt((mySlice + 2) % nSlices).GetPt() );

	// see if we are going to hit another car
	myControl.m_colliding = false;
	myControl.m_adjust = 0;
	for( int i = 0; i < NEARBY_CARS; i++ )
	{
		if( s.nearby[i].who >= MAX_CARS ||
			pcar[s.nearby[i].who]->On_pit_lane )
			continue;

		rel_state&	cari = s.nearby[i];

		double	x = s.nearby[i].rel_x;
		double	y = s.nearby[i].rel_y;
		double	vx = s.nearby[i].rel_xdot;
		double	vy = s.nearby[i].rel_ydot;
		double	v = s.nearby[i].v;
		double	vn = s.nearby[i].vn;
		double	al = s.nearby[i].alpha;
		int		who = s.nearby[i].who;

		double	rv = hypot(vx, vy);
		if( vy < 0 )
			rv = -rv;

		double	cv = x * vx > 0 ? vy : rv;

		// don't pass other cars going more than 15fps faster than them.
		//	only include cars within ±30° of current direction of travel.
//		brakeDist = BrakingDistanceBend(s.v, s.v + cv + 15);
//		if( y > CARLEN && y < brakeDist && fabs(x) <= y / 2 + CARWID )
		double	brakeDist = BrakingDistanceBend(s.v, s.v + cv + 15);// + CARLEN;
		if( y > CARLEN && brakeDist > s.nearby[i].dist &&
			fabs(x) <= y / 2 + CARWID )
		{
			double	dist, k;
			D13Vec2	vel;
			int		hisSlice = carPaths.GetLast(who);
			myPath.Calc(hisSlice, D13Vec2(pcar[who]->X,
							pcar[who]->Y), dist, vel, k);

			myControl.m_needToBrake = true;
		}

		// check if we are currently colliding with the other car.
		bool	colliding = false;
/*		double	relAngle = vDir.GetAngle() - cPi_2;//atan2(-vx, s.v + vy);
		if( collide(x, y, relAngle) )
		{
			myControl.m_colliding = colliding = true;
			if( y > 0 && vy < 0 )
			{
//				myControl.m_needToBrake = true;
				myControl.m_vc = min(myControl.m_vc, s.v + vy - 0.1);
//				vc = min(vc, s.v + vy + 1);
			}
		}
*/
		// work out if we will hit the other car, up to to 3 seconds into
		//	the future
		double	minDist = 300;		// ft
		double	minTime = 10;		// seconds
		D13Vec2	minP1, minP2;
		D13Vec2	minDir1, minDir2;
		double	minSpd1, minSpd2;
		bool	collide = false;
		double	collDist = 0;
		double	deltaT = cSimDeltaTime;			// 1/20 sec
		mySim.ResetTime();
		D13CarSim&	hisSim = *pCarSim[who];
		hisSim.Update( D13CarSim::cAutoSim, cari );
		rel_state	myCar, hisCar;	// calc'd states of cars at collision time
		double	time = 0;
		for( ; time < cMaxSimTime; time += deltaT )
		{
			D13Vec2	p1 = mySim.GetPt().ToCoordSys(myPt, myRight);
			D13Vec2	dir1 = mySim.GetDir().ToCoordSys(D13Vec2(0, 0), myRight);
			double	a1 = dir1.GetAngle() - cPi_2;

			D13Vec2	p2 = hisSim.GetPt().ToCoordSys(myPt, myRight);
			D13Vec2	dir2 = hisSim.GetDir().ToCoordSys(D13Vec2(0, 0), myRight);
			double	a2 = dir2.GetAngle() - cPi_2;

			D13Vec2	p = p2 - p1;
			double	dist = p.GetLength();
			if( dist < minDist )
			{
				minDist = dist;
				minTime = time;
				minP1 = p1;
				minDir1 = dir1;
				minSpd1 = mySim.GetSpeed();
				minP2 = p2;
				minDir2 = dir2;
				minSpd2 = hisSim.GetSpeed();

				mySim.SetupRelState( hisSim, myCar );
				hisSim.SetupRelState( mySim, hisCar );

				double	scale = 1.2;//min(3.0, 1.05 + 0.4 * time);
				double	fv = minDir1 * minDir2 * minSpd2;
				double	mcv = max(0, s.v - BRAKE_ACCEL_BEND * (minTime - deltaT));
				if( mcv > fv + 30 )
					scale = min(3.0, scale * 1.5);
				if( CheckCollide(p, a2 - a1, scale) )
				{
					collide = true;
					myControl.m_important = mySim.GetNSlices();
					dist = min(dist, cCarDiag);
					minDist = dist;
					collDist = mySim.GetDist();//Distance(alpha, rotA, time - deltaT) * rad;
					if( dist < cCarDiag )
						break;
				}
			}

			mySim.Forward();
			hisSim.Forward();
		}

		if( collide )
		{
			bool	opponentToLeft = hisCar.rel_x < -5;
			bool	opponentToRight = hisCar.rel_x > 5;

			for( ; time < cMaxSimTime; time += deltaT )
			{
				D13Vec2	p1 = mySim.GetPt().ToCoordSys(myPt, myRight);
				D13Vec2	dir1 = mySim.GetDir().ToCoordSys(D13Vec2(0, 0), myRight);
				double	a1 = dir1.GetAngle() - cPi_2;

				D13Vec2	p2 = hisSim.GetPt().ToCoordSys(myPt, myRight);
				D13Vec2	dir2 = hisSim.GetDir().ToCoordSys(D13Vec2(0, 0), myRight);
				double	a2 = dir2.GetAngle() - cPi_2;

				D13Vec2	p = p2 - p1;

				double	scale = 1.2;//min(3.0, 1.05 + 0.4 * time);
				double	fv = minDir1 * minDir2 * minSpd2;
				double	mcv = max(0, s.v - BRAKE_ACCEL_BEND * (minTime - deltaT));
				if( mcv > fv + 30 )
					scale = min(3.0, scale * 1.5);
				if( !CheckCollide(p, a2 - a1, scale) )
					break;

				rel_state	myEdge, hisEdge;
				mySim.SetupRelState( hisSim, myEdge );
				hisSim.SetupRelState( mySim, hisEdge );

				if( hisEdge.rel_y < 15 && hisEdge.rel_y > -15 )
				{
					double	k = mySim.GetCurvature();

					if( opponentToRight &&
						myEdge.to_lft < (k > 0 ? 2 : 5) && //hisCar.rel_x > 7 &&
						hisEdge.rel_x <  10 &&
						hisEdge.vn > 0 && hisEdge.vn > myEdge.vn )
					{
						myControl.m_badCollision = true;
						myControl.m_adjust += -2.5;
						myControl.m_important = mySim.GetNSlices();
//						myControl.m_needToBrake = myEdge.vn > 0;
						break;
					}
					else if( opponentToLeft &&
							 myEdge.to_rgt < (k < 0 ? 2 : 5) && //hisCar.rel_x < -7 &&
							 hisEdge.rel_x > -10 &&
							 hisEdge.vn < 0 && hisEdge.vn < myEdge.vn )
					{
						myControl.m_badCollision = true;
						myControl.m_adjust += 2.5;
						myControl.m_important = mySim.GetNSlices();
//						myControl.m_needToBrake = myEdge.vn < 0;
						break;
					}
				}

				mySim.Forward();
				hisSim.Forward();
			}
		}

		if( colliding && myControl.m_vc > s.v && y < 0 )
		{
			if( mySituation.m_curvature > 0 && x < -4 && s.vn < 0 )
			{
				myControl.m_adjust += 1;
				myControl.m_badCollision = true;
//				sideCollision += 0.07;
//				if( s.to_rgt < 15 )
//					myControl.m_sideCollision += 0.3;
			}
			else if( mySituation.m_curvature < 0 && x > 4 && s.vn > 0 )
			{
				myControl.m_adjust += -1;
				myControl.m_badCollision = true;
//				sideCollision -= 0.07;
//				if( s.to_lft < 15 )
//					myControl.m_sideCollision += 0.3;
			}
		}

		if( minDist < CARLEN * 2.5 )
		{
			double	maxRelSpd = max(3, (minDist - CARWID) / CARLEN * 15);
			double	relSpd = minSpd1 - minDir1 * minDir2 * minSpd2;
//			if( relSpd > maxRelSpd )
//				myControl.m_needToBrake = true;
		}

		if( !collide )
		{
			continue;
		}

/*		double	a1 = Distance(alpha, rotA, minTime);
		double	a2 = relAngle + Distance(al, w, minTime);
		double	aa = a2 - a1;
		double	v1 = max(0, s.v - BRAKE_ACCEL_BEND * (minTime - deltaT));
		double	v2 = max(0, v + la * minTime);
		double	mfv = v2 * cos(aa);
		double	collY = (minP2 - minP1) * minDir1;
		double	collX = (minP2 - minP1) * minDir1.GetNormal();
		D13Vec2	relV = minDir1 * v1 - minDir2 * v2;
		double	collV = (minP2 - minP1).GetUnit() * relV;
		double	sq = sqrt(max(0, v1 * v1 + v2 * v2 - 2 * v1 * v2 * cos(aa)));
		double	sqv = relV.GetLength();
		double	colV = v1 > v2 ? sq : 0;
//		colV = sq;
		if( colV > 10 )
		{
			myControl.m_badCollision = true;
			if( fabs(aa) > 5 * cPi / 180 )
			{
				double adj = max(0.05, min(0.20,
						0.05 + 0.05 * (colV - 30) / 15));
//				adj = 0.05;
				if( collX > 0 &&
					s.to_rgt + minTime * s.vn > 10 && x < 0 )
				{
					myControl.m_sideCollision += -adj;
				}
				else if( collX < 0 &&
						 s.to_lft - minTime * s.vn > 10 && x > 0 )
				{
					myControl.m_sideCollision += adj;
				}
			}
		}
*/
//		rel_state	myCar, hisCar;	// calc'd states of cars at collision time
//		mySim.SetupRelState( hisSim, myCar );
//		hisSim.SetupRelState( mySim, hisCar );

		D13Vec2	myV = minDir1 * myCar.v;
		D13Vec2	hisV = minDir2 * hisCar.v;
		D13Vec2	relV = hisV - myV;
		D13Vec2	relP = minP2 - minP1;
		double	collSpd = -relV * relP.GetUnit();
//		if( hisCar.rel_x * hisCar.rel_xdot < 0 )
//			collSpd = max(collSpd, fabs(hisCar.rel_xdot));

		compSpd = compSpd || collSpd > 30;
//		if( minTime < colTime )
		if( !compSpd && minTime < colTime ||
			compSpd  && collSpd > maxColV )
		{
			gotCar = true;
			colCar = s.nearby[i];
//			car.vn = vn;
			maxColV = collSpd;
			colMyCar = myCar;
			colHisCar = hisCar;
			colTime = minTime;
			colPos1 = minP1;
			colDir1 = minDir1;
			colPos2 = minP2;
			colDir2 = minDir2;
			colSpd2 = minSpd2;
		}

//		double	collideY = (collidePos2 - collidePos1) * collideDir1;
//		double	collideX = (collidePos2 - collidePos1) * collideDir1.GetNormal();
//		brakeDist = max(0, BrakingDistanceBend(s.v, finalV));
		double	mfv = minDir1 * minDir2 * minSpd2;
		mfv = myCar.v + hisCar.rel_ydot;
		brakeDist = max(0, BrakingDistanceBend(s.v, mfv));
		double	d = collDist - CARLEN * 1.5 + hisCar.rel_y;
		double	t = max(delta_time, 2 * d / (mfv + s.v));
		double	a = (mfv - s.v) / t;
		if( a < -28 || brakeDist >= d )
		{
//			D13Vec2	myV = minDir1 * myCar.v;
//			D13Vec2	hisV = minDir2 * hisCar.v;
//			D13Vec2	relV = hisV - myV;
//			D13Vec2	relP = minP2 - minP1;
//			double	collSpd = relV * relP.GetUnit();
			double	target = myControl.m_vc;
			if( y > 0 && vy < 0 ||
				hisCar.rel_y > 0 && hisCar.rel_ydot < 0 )//||
				//collSpd > 10 )
			{
				target = mfv + 3;//mfv > 20 ? mfv + 5 : max(1, mfv - 1);
			}

			myControl.m_badCollision |= (collSpd > 10);
//			else if( y <= 0 )
//				target = mfv - 0.5;//mfv > 20 ? mfv + 5 : max(1, mfv - 1);

			myControl.m_vc = min(myControl.m_vc, target);
//			myControl.m_needToBrake = true;
		}
	}

	if( colTime < 10 )
	{
		D13Vec2	norm = colDir1.GetNormal();
		double	dot = (colPos2 - colPos1) * norm;

		// work out how "urgent" it is to dodge the other car,
		//	related to how far ahead it is
//		double	urgency = max((10 * CARLEN - car.rel_y) / (9 * CARLEN), 0);
//		urgency = min(urgency, 1);
		double	urgency = 0.5;//min(1, sqrt(max(1 - collideTime * 0.15, 0)));
		if( colCar.rel_y < CARLEN && dot * mySituation.m_curvature > 0 )
			urgency = 1.0;
		if( colHisCar.rel_ydot > -15 && mySituation.m_tanDist < 50 )
			urgency = 0.1;

		// got car at side ... which side?
		if( urgency > 0 )
		{
			double	delta = 1.0;
			if( myControl.m_badCollision )//car.rel_ydot < -10 )
				delta = 2.0;
			bool	out = pcar[colCar.who]->Damage > MAX_DAMAGE;
//			const double	delta = 0.5;
			if( dot > 0 )
			{
				// can we get to the right of the car?
//				if( car.to_rgt + car.vn * collideTime > cCarWid )
				if( out && colCar.to_rgt + colCar.vn * colTime > cCarWid )
				{
					if( s.to_rgt > cCarWid * 1.1 )//|| y < CARLEN )
						myControl.m_adjust += -delta * urgency;
				}
				else if( !out && colCar.to_rgt > cCarWid )
				{
					double	edge = mySituation.m_curvature > 0 ? 10 : 3;
					if( s.to_rgt > cCarWid * 1.1 &&
						colMyCar.to_rgt > cCarWid )
					{
						myControl.m_adjust += -delta * urgency;
					}
					else if(
						colHisCar.to_rgt + colHisCar.vn * 0.1 < edge + 8 &&
						colMyCar.to_rgt + colMyCar.vn * 0.1 < edge &&
						colHisCar.to_rgt - 2.5 > colMyCar.to_rgt )
					{
						urgency = 1.0;
						myControl.m_badCollision = true;
						myControl.m_adjust += delta * urgency;
//						myControl.m_important = 20;
					}
				}
				else
				{
					if( colCar.rel_y > CARLEN + 0.5 )
						myControl.m_adjust += delta * urgency;
				}
			}
			else
			{
				// can we get to the left of the car?
//				if( car.to_lft - car.vn * collideTime > cCarWid )
				if( colCar.to_lft > cCarWid )
				{
					double	edge = mySituation.m_curvature < 0 ? 10 : 3;
					if( s.to_lft > cCarWid * 1.1 &&
						colMyCar.to_lft > cCarWid )
					{
						myControl.m_adjust += delta * urgency;
					}
					else if(
						colHisCar.to_lft - colHisCar.vn * 0.1 < edge + 8 &&
						colMyCar.to_lft - colMyCar.vn * 0.1 < edge &&
						colHisCar.to_lft - 2.5 > colMyCar.to_lft )
					{
						urgency = 1.0;
						myControl.m_badCollision = true;
						myControl.m_adjust += -delta * urgency;
//						myControl.m_important = 20;
					}
				}
				else
				{
					if( colCar.rel_y > CARLEN + 0.5 )
						myControl.m_adjust += -delta * urgency;
				}
			}

			// don't try to pass by turning the other way than
			//	we are already
			if( !myControl.m_badCollision &&
				mySituation.m_tanDist < 100 && //car.rel_ydot > -5 &&
				(mySituation.m_nextTurnIsLeft  && myControl.m_adjust < 0 ||
				 !mySituation.m_nextTurnIsLeft && myControl.m_adjust > 0) )
				myControl.m_adjust *= 0;//0.2;//0.5;
//			if( fabs(mySituation.m_curvature) > 0.001 &&
//				(mySituation.m_curvature > 0 && myControl.m_adjust < 0 ||
//				 mySituation.m_curvature < 0 && myControl.m_adjust > 0) )
//				myControl.m_adjust *= 0.5;
		}
	}

	if( myControl.m_needToBrake )
		myControl.m_needToBrake = true;

	return myControl.m_colliding;
}

/////////////////////////////////////////////////////////////////////////////

class	D13InverseFriction
{
public:
	enum { cSize = 5000 };

public:
	D13InverseFriction();
	~D13InverseFriction();

	double	operator()( double friction );

private:
	double	m_graph[cSize + 1];
	double	m_step;
};

/////////////////////////////////////////////////////////////////////////////

D13InverseFriction::D13InverseFriction()
{
	// scan friction function to find max slip, max friction
	double	maxFriction = -1;
	double	maxSlip = 0;
	double	oldF = -1;
	double	deltaSlip = 0.01;
//	double	deltaSlip = 0.0001;
	for( double slip = 0; ; slip += deltaSlip )
	{
		double	 f = friction(slip);

		double	deltaF = f - oldF;
		if( f < oldF + 0.0001 )
//		if( deltaF / deltaSlip < 0.005 )
			break;

		if( f > maxFriction )
		{
			maxFriction = f;
			maxSlip = slip;
		}

		oldF = f;
	}

	// tone down max slip a bit
//	maxSlip *= 0.97;
//	maxFriction = friction(slip);

	// now we fill in the inverse mapping graph...
	m_step = maxFriction / cSize;
	for( int i = 0; i <= cSize; i++ )
	{
		// find slip for fiction value	
		double	targetF = i * m_step;
		double	highSlip = maxSlip;
		double	lowSlip = 0;
		while( lowSlip + 0.0000001 < highSlip )
		{
			double	slip = (lowSlip + highSlip) * 0.5;
			double	f = friction(slip);
			if( f < targetF )
				lowSlip = slip;
			else
				highSlip = slip;
		}
		m_graph[i] = (lowSlip + highSlip) * 0.5;
	}
}

/////////////////////////////////////////////////////////////////////////////

D13InverseFriction::~D13InverseFriction()
{
}

/////////////////////////////////////////////////////////////////////////////

double	D13InverseFriction::operator()( double friction )
{
	int		i = int(floor(friction / m_step));
	if( i < 0 )
		return 0;
	else if( i >= cSize )
		return m_graph[cSize];

	double	delta = (friction - i * m_step) / m_step;
	double	slip = m_graph[i] + (m_graph[i + 1] - m_graph[i]) * delta;
	return slip;
}

/////////////////////////////////////////////////////////////////////////////

con_vec Dodger13( situation& s )
{ 
	const char name[] = "Dodger13";	// This is the robot driver's name! 

	static D13Track		track;
	static D13Slices		slices;
	static D13OptPath*	pOptPath = 0;
	static D13CarPaths*	pCarPaths = 0;

	static int			cur_p = 0;
	static int			upd_p = 0;
	static int			softServo = 0;
	static int			bestCar = -1;
	static bool			offsetTrack = false;
	static double		speedEstimate = 0;
	static double		timeDamageRate = 0;

	static D13InverseFriction	invFriction;

	con_vec	result;		// control vector
	int		damage = 0;

	if( s.starting )
	{
		// first time only, copy name: 
		my_name_is(name);

		track_desc	trackDesc = get_track_description();

		result.alpha = 0;
		result.vc = 300; 

		if( s.my_ID >= car_count )
			return result;

		// change grip available depending on track surface
		switch( surface )
		{
			default:
			case 0:		CORN_MYU = MYU_MAX0;	break;
			case 1:		CORN_MYU = MYU_MAX1;	break;
			case 2:		CORN_MYU = MYU_MAX2;	break;
		}

		MAX_ACCEL = CORN_MYU * g * 0.999;

		if( s.stage == BEFORE || s.stage == FINISHED )
		{
			// clean up
			track.Empty();
			delete pOptPath;
			pOptPath = 0;
			delete pCarPaths;
			pCarPaths = 0;
		}
		else if( track.NSeg() == 0 && trackDesc.NSEG != 0 )
		{
			track.Initialise( trackDesc );
			slices.Setup( track );
			pOptPath = new D13OptPath(slices);
			pOptPath->SetFactor( FindTrackFactor() );
			if( !pOptPath->Load(CURRENT_TRACK_NAME) )
				pOptPath->Optimise();
			pCarPaths = new D13CarPaths(s.my_ID, slices, *pOptPath);

			D13Path&	path = pCarPaths->GetPath(s.my_ID);
			speedEstimate = path.EstimateSpeed();

			double	damSpeedEstimate = path.EstimateSpeed(7500, 50);
			double	time1 = slices.GetLength() / speedEstimate;
			double	time2 = slices.GetLength() / damSpeedEstimate;
			timeDamageRate = (time2 - time1) / 7500;
		}

		// set up initial values for pit, fuel and damage
		PitControl( s, track, result, damage, timeDamageRate );

		cur_p = 0;
		softServo = 100;
		bestCar = s.my_ID;
		offsetTrack = true;

		s.out_pits = 0;

		return result; 
	} 

	// figure whether we need to refuel or repair damage (or both)
	PitControl( s, track, result, damage, timeDamageRate );
	if( s.out_pits )
		softServo = 100;

	// ask to see cars that are to the side, but slightly behind us
	s.side_vision = true;

	// figure if we've just come out of the pits
	static int old_out_pits = 0;
	bool	justExitedPits = old_out_pits && !s.out_pits;
	old_out_pits = s.out_pits;

	// if we just exited from the pits, we want to adjust the track
	if( justExitedPits )
		offsetTrack = true;

	// get details of current track segment
	const D13Track::Segment	seg = track.GetSegment(s.seg_ID);

	// work out current position of car, global coords
	D13Vec2	carPt(pcar[s.my_ID]->X, pcar[s.my_ID]->Y);

	// work out current direction of car, global angle
	double	carAngle = asin(s.vn / s.v);
	if( s.cur_rad == 0 )
		carAngle += (seg.m_end - seg.m_beg).GetAngle();
	else if( s.cur_rad > 0 )
		carAngle += (carPt - seg.m_cen).GetAngle() + cPi_2;
	else
		carAngle += (carPt - seg.m_cen).GetAngle() - cPi_2;
	carAngle = NormaliseAngle(carAngle);

	D13Vec2	carDir = D13Vec2::FromAngle(carAngle);
	D13Vec2	carVel = carDir * s.v;

	//
	// work out where we are on the path around the track

	cur_p = slices.Find(carPt, s.seg_ID, cur_p);
	int		prev_p = (cur_p + slices.GetSize() - 1) % slices.GetSize();
	int		next_p = (cur_p + 1) % slices.GetSize();
	int		aft_p = (next_p + 1) % slices.GetSize();

	//
	// change the path slice at the track position directly opposite us
	//	back to the optimised version

	D13Path&	path = pCarPaths->GetPath(s.my_ID);

	int		upd_cur = (cur_p + slices.GetSize() / 2) % slices.GetSize();
	while( upd_p != upd_cur )
	{
		path.GetAt(upd_p) = pCarPaths->GetOptPath().GetAt(upd_p);
		upd_p = (upd_p + 1) % slices.GetSize();
	}

	//
	// figure out our current distance from the path, etc.

	int		count = 0;

calcTargets:
	D13Vec2	p0 = path.GetAt(prev_p).GetPt();
	D13Vec2	p1 = path.GetAt(cur_p).GetPt();
	D13Vec2	p2 = path.GetAt(next_p).GetPt();
	D13Vec2	p3 = path.GetAt(aft_p).GetPt();

	double	dist1, k1;
	D13Vec2	vel1;
	path.Calc( cur_p, carPt, dist1, vel1, k1 );

	//
	// modify path if we are too far from it

//	if( count == 0 && (fabs(dist1) > 10 || offsetTrack) )
	bool	pitting =	pcar[s.my_ID]->On_pit_lane ||
						pcar[s.my_ID]->Coming_from_pits;
	if( count == 0 && ((fabs(dist1) > 5 && !pitting) || offsetTrack) )
	{
		pOptPath->SetFromPath( pCarPaths->GetPath(s.my_ID) );
		double	adjust = dist1;
		if( !offsetTrack )
		{
			adjust = dist1 < 0 ? -1.0 : 1.0;
			if( fabs(dist1) > 30 )
				adjust *= 10;
		}
		else if( fabs(adjust) > 0.5 &&
				 (s.to_lft < 0.75 || s.to_rgt < 0.75) )
			adjust += adjust < 0 ? 0.5 : -0.5;
		int		len = min(150, slices.GetSize() / 2 - 2);
		pOptPath->ModifySectionAfter( prev_p, len, adjust );
		pOptPath->CopyToPath( pCarPaths->GetPath(s.my_ID) );
		pCarPaths->GetPath(s.my_ID).CalcSpeedSection( cur_p, len );
		count++;
		goto calcTargets;
	}

	offsetTrack = false;

	if( s.lap_flag || justExitedPits )
	{
		// re-calculate path speeds now
		pCarPaths->GetOptPath().CalcSpeed( s.damage, s.fuel );

//		if( s.laps_done == 65 )
//			int fff = 5;
	}

	//
	// record the paths of the other cars

	for( int i = 0; i < car_count; i++ )
		if( i != s.my_ID )
			pCarPaths->Record( i, pcar[i] );

	//
	// steering servo and speed calc

	double	alpha = 0;
	double	vc = 300;

	D13Vec2	pt2 = carPt + carDir * s.v * delta_time;
	double	dist2, k2;
	D13Vec2	vel2;
	path.Calc( cur_p, pt2, dist2, vel2, k2 );

	double	spd2 = vel2.GetLength();

	double	k = (k1 + k2) * 0.5;

	double	latA = s.v * s.v * k;
	double	fric = fabs(latA / g);
	double	slip = invFriction(fric);
	double	ratio = slip / (2 * s.v);
	alpha = 2 * asin(ratio);
	if( latA < 0 )
		alpha = -alpha;
	if( fabs(dist2) > 0.5 && dist2 * alpha < 0 && spd2 > s.v )
	{
//		alpha *= 1.1;
//		vc = (s.v + vc) * 0.5;
	}

	double	minVc = s.v;
	{
	double	inner = MAX_ACCEL * MAX_ACCEL - latA * latA;
	double	tanA = inner > 0 ? sqrt(inner) : 0;
	double	fric2 = fabs(hypot(tanA, latA)) / g;
	double	slip2 = invFriction(fric2);
	D13Vec2	d = D13Vec2(tanA, latA).GetUnit();
	minVc = s.v - d.x * slip2;
	}

	D13Vec2	dir = vel2;
	double	servoAngle = NormaliseAngleLimit(dir.GetAngle() - carAngle, 0);
	static double oldDist = 0;
//	double	pid = -0.05 * dist1 + -0.0133 * (dist1 - oldDist);
	double	pid = -0.05 * dist1 + -0.025 * (dist1 - oldDist);
	oldDist = dist1;
	servoAngle += atan(pid);

	vc = spd2;
	alpha += servoAngle;

	{
		// make a better value for vc, to get to the speed we want
//		D13Vec2	deltaV = vel2 - vel1.GetUnit() * s.v;
		D13Vec2	deltaV = vel2 - carVel;
		D13Vec2	a = deltaV / delta_time;
		const double	cMass = M + s.fuel / g;
//		double	drag = DRAG_CON * s.v * s.v * (2 * damage + MAX_DAMAGE) / 
//							(MAX_DAMAGE * cMass);
//		D13Vec2	f = (a + D13Vec2(drag, 0)) * cMass;
		D13Vec2	f = a * cMass;
		double	slip3 = invFriction(f.GetLength());
		D13Vec2	L = f.GetUnit() * slip3;
//		D13Vec2	vcVec = vel1.GetUnit() * s.v + L;
		D13Vec2	vcVec = carVel + L;
		double	newVc = vcVec.GetLength();
		double	al = NormaliseAngleLimit(vcVec.GetAngle() - carAngle, 0);
//		if( newVc < vc )
//			alpha *= 1.1;
		vc = newVc;
//		alpha = al + servoAngle;
	}

	//
	// see if we are going to hit another car

	D13Situation	mySituation;
	mySituation.m_carPt = carPt;
	mySituation.m_carDir = carDir;
	mySituation.m_carAngle = carAngle;
	mySituation.m_distDir = dir;
	mySituation.m_tanDist = 0;
	int		tanSeg = s.seg_ID;
	count = min(4, track.NSeg());
	while(	count > 0 &&
			(track.GetSegment(tanSeg).m_rad == 0 ||
			 fabs(track.GetSegment(tanSeg).m_rad) > 1000) )
	{
		if( track.GetSegment(tanSeg).m_rad == 0 )
		{
			if( tanSeg == s.seg_ID )
				mySituation.m_tanDist += s.to_end;
			else
				mySituation.m_tanDist += track.GetSegment(tanSeg).Ft();
		}
		else
		{
			if( tanSeg == s.seg_ID )
				mySituation.m_tanDist += fabs(s.to_end * s.cur_rad);
			else
				mySituation.m_tanDist += fabs(track.GetSegment(tanSeg).Ft());
		}
		tanSeg = (tanSeg + 1) % track.NSeg();
	}
	mySituation.m_nextTurnIsLeft = track.GetSegment(tanSeg).m_rad > 0;
	mySituation.m_curvature = k;
	mySituation.m_slice = cur_p;

	double	passAdjust = 0;
	bool	needToBrake = false;
	bool	colliding = false;
	bool	badCollision = false;
	D13Control	myControl;

	memset( &myControl, 0, sizeof(myControl) );

	if( s.stage != QUALIFYING )
	{
		myControl.m_vc = vc;
		myControl.m_adjust = passAdjust;
		myControl.m_needToBrake = needToBrake;
		myControl.m_badCollision = badCollision;

		colliding = AvoidOtherCars( s, mySituation, track, *pCarPaths, myControl );

		vc = myControl.m_vc;
		passAdjust = myControl.m_adjust;
		needToBrake = myControl.m_needToBrake;
		badCollision = myControl.m_badCollision;

//		passAdjust = 0;
		if( passAdjust != 0 )
		{
			// work out how much to modify
			int		len = min(200, slices.GetSize() / 2 - 2);

			// transfer current path to optimiser
			pOptPath->SetFromPath( pCarPaths->GetPath(s.my_ID) );

			bool	modified = false;

			// don't try to adjust track if already turning that way...
			if( badCollision ||
				dist1 * passAdjust < 0 || fabs(dist1 + passAdjust) < 3 )
			{
				// modify the section
//				pOptPath->ModifySectionAfter( prev_p, len, passAdjust );
//				pOptPath->ModifySectionCircle( prev_p, len, passAdjust );
				int		important = max(20, myControl.m_important);
				if( myControl.m_important < important )
					int		fff = 5;
				bool ok = pOptPath->ModifySection( prev_p, len, passAdjust,
					(prev_p + important) % slices.GetSize(), badCollision );

				if( ok )
					modified = true;
			}

/*			if( myControl.m_sideCollision )
			{
				bool ok = pOptPath->ModifySectionAfter( prev_p, len,
					myControl.m_sideCollision ? 1.0 : -1.0 );

				if( ok )
					modified = true;
			}
*/
			if( modified )
			{
				// figure the new speeds
				D13Path	tempPath(pCarPaths->GetPath(s.my_ID));
				pOptPath->CopyToPath( tempPath );
				tempPath.CalcSpeedSection( cur_p, len );

				// if speed is ok, use new path
				if( tempPath.GetAt(cur_p).m_spd >= s.v - 1 )
					pCarPaths->GetPath(s.my_ID) = tempPath;
			}
		}
	}

	if( s.to_lft < 0.2 && s.to_lft - s.vn * delta_time < 0 ||
		colliding && s.to_lft < 2.0 )
	{
		if( s.vn > 0 )
			alpha -= colliding ? (2.0 - s.to_lft) * 0.05 : 0.05;
	}
	else if( s.to_rgt < 0.2 && s.to_rgt + s.vn * delta_time < 0 ||
			 colliding && s.to_rgt < 2.0 )
	{
		if( s.vn < 0 )
			alpha += colliding ? (2.0 - s.to_rgt) * 0.05 : 0.05;
	}

	if( needToBrake || vc < s.v * 0.95 )
	{
		// work out the amount of grip required to take us in a circular
		//	path at the current velocity + 2%
//		double	minMyuN = fabs(1.02 * s.v * alpha / g);
		double	minMyuN = fabs(1.02 * s.v * s.v * k / g);

		// now search for a braking coefficient (bc) that allows
		//	at least this much lateral grip, but with a minimum bc of 0.95
		double	bc = 0.8;//BRAKE_COEF;
		vc = s.v * bc;
		D13Vec2	alphaVec = D13Vec2::FromAngle(alpha);
		D13Vec2	slipVec = D13Vec2(s.v, 0) - alphaVec * vc;
		double	slip = slipVec.GetLength();
		double	myu = friction(slip);
		double	myuN = fabs(slipVec.GetUnit().y * myu);
		while( bc < 0.95 && myuN < minMyuN )
		{
			bc += 0.01;
			vc = s.v * bc;
			slipVec = D13Vec2(s.v, 0) - alphaVec * vc;
			slip = slipVec.GetLength();
			myu = friction(slip);
			myuN = fabs(slipVec.GetUnit().y * myu);
		}
	}

	// return our precious :) control values
	result.vc = vc;
	result.alpha = alpha;

	if( badCollision && s.to_lft > 10 && s.to_rgt > 10 )
	{
//		result.vc = s.v * 0.5;	// brake hard
	}

	if( myControl.m_sideCollision )
	{
		result.alpha += myControl.m_sideCollision;
	}

	return result; 
} 
