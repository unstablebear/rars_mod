/*
Author:				Tim Foden (Newquay, Cornwall, UK)
E-mail:				tim@7sun.com
Robot name:			"Dodger6"
Robot version:		6
Driver function:	Dodger6()
Filename:			Dodger6.cpp
Races:				All
RARS version:		0.75
Data file:			None
Robot status:		Public
Race colours:		White (front) & Black (rear)
Release date:		13-May-2001
Tab size used:		4
*/ 

#include <stdio.h>
#include <string.h> 
#include <stdlib.h> 
#include <math.h> 

#include "car.h" 
#include "track.h"
#include "misc.h"
#include "os.h"

/////////////////////////////////////////////////////////////////////////

#if defined(WIN32) && defined(DEBUG)
#define ASSERT(expr)	{ if( !(expr) ) __asm { int 3 } }
#else
#define ASSERT(expr)	;
#endif

/////////////////////////////////////////////////////////////////////////////

const double cCarWid = CARWID * 1.2;	// make out cars are wider than they really are
const double cCarDiag = hypot(CARWID, CARLEN);

// pit constants from Car::move_car() in carz.cpp
const double cPitSecsPerDamage = 0.005;
const double cPitSecsPerFuel = 0.05;

static double CORN_MYU = 1.05;		// lateral g's expected when cornering

const double BRAKE_ACCEL = 35.5;	// accel available while braking (straight)
const double BRAKE_COEF = 0.8;
//const double BRAKE_ACCEL = 27.0;	// accel available while braking (straight)
//const double BRAKE_COEF = 0.95;

const double BRAKE_ACCEL_BEND = 30;	// accel available while braking (bend)
const double BRAKE_COEF_BEND = 0.95;
//const double BRAKE_ACCEL_BEND = 35.5;	// accel available while braking (bend)
//const double BRAKE_COEF_BEND = 0.8;

const double cMinGap = 0.5;		// in ft from edge
const double cMaxGap = 5.0;		// in ft from edge

const double MAX_ACCEL = 32.2;

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

/* Not used
static bool	AngleInRange( double angle, double startAngle, double sizeAngle )
{
	if( sizeAngle > 0 )
		return NormaliseAngle(angle - startAngle) < sizeAngle;
	else
		return NormaliseAngle(angle - startAngle) < -sizeAngle;
}
*/

/////////////////////////////////////////////////////////////////////////////

/* Not used
static double	AngularPortion( double angle, double startAngle, double sizeAngle )
{
	// get the difference in the angles
	double	deltaAngle = NormaliseAngleLimit(angle - startAngle, sizeAngle);

	// work out the ratio
	return deltaAngle / sizeAngle;
}
*/

/////////////////////////////////////////////////////////////////////////////

static double	FindTrackFactor()
{
        track_desc track = get_track_description();
	for( int i = 0; factorTable[i].m_name; i++ )
	{
		if( strcmp(factorTable[i].m_name, track.sName) == 0 )
			return factorTable[i].m_factor;
	}

	return cDefaultFactor;
}

/////////////////////////////////////////////////////////////////////////////

struct	D6Vec2
{
public:
	D6Vec2();
	D6Vec2( double x, double y );
	D6Vec2( const D6Vec2& vec2 );

	D6Vec2	operator+( const D6Vec2& v ) const;
	D6Vec2	operator-( const D6Vec2& v ) const;
	D6Vec2	operator-() const;
	
	D6Vec2	operator*( double n ) const;		// scalar product
	D6Vec2	operator/( double n ) const;		// scalar divide

	double	operator*( const D6Vec2& v ) const;	// dot product
	double	operator%( const D6Vec2& v ) const;	// cross product

	bool	operator==( const D6Vec2& v ) const;	// equality
	bool	operator!=( const D6Vec2& v ) const;	// inequality

	double	GetLength() const;
	double	GetAngle() const;
	D6Vec2	GetUnit() const;
	D6Vec2	GetNormal() const;

	static D6Vec2	FromAngle( double angle );

public:
	double	x;
	double	y;
};

D6Vec2::D6Vec2()
:	x(0),
	y(0)
{
}

D6Vec2::D6Vec2( double X, double Y )
:	x(X),
	y(Y)
{
}

D6Vec2::D6Vec2( const D6Vec2& v )
:	x(v.x),
	y(v.y)
{
}

D6Vec2	D6Vec2::operator+( const D6Vec2& v ) const
{
	return D6Vec2(x + v.x, y + v.y);
}

D6Vec2	D6Vec2::operator-( const D6Vec2& v ) const
{
	return D6Vec2(x - v.x, y - v.y);
}

D6Vec2	D6Vec2::operator-() const
{
	return D6Vec2(-x, -y);
}

D6Vec2	D6Vec2::operator*( double n ) const
{
	return D6Vec2(x * n, y * n);
}

D6Vec2	D6Vec2::operator/( double n ) const
{
	return D6Vec2(x / n, y / n);
}

double	D6Vec2::operator*( const D6Vec2& v ) const
{
	return x * v.x + y * v.y;
}

double	D6Vec2::operator%( const D6Vec2& v ) const
{
	return x * v.y - y * v.x;
}

bool	D6Vec2::operator==( const D6Vec2& v ) const
{
	return x == v.x && y == v.y;
}

bool	D6Vec2::operator!=( const D6Vec2& v ) const
{
	return x != v.x || y != v.y;
}

double	D6Vec2::GetLength() const
{
	return hypot(x, y);
}

double	D6Vec2::GetAngle() const
{
	return atan2(y, x);
}

D6Vec2	D6Vec2::GetUnit() const
{
	if( x == 0 && y == 0 )
		return D6Vec2();

	double	len = hypot(x, y);
	return D6Vec2(x / len, y / len);
}

D6Vec2	D6Vec2::GetNormal() const
{
	return D6Vec2(-y, x);
}

D6Vec2	D6Vec2::FromAngle( double angle )
{
	return D6Vec2(cos(angle), sin(angle));
}

/////////////////////////////////////////////////////////////////////////////

class D6Track  
{
public:
	struct Segment
	{
		double	m_rad;
		double	m_len;

		D6Vec2	m_beg;
		D6Vec2	m_end;
		D6Vec2	m_cen;
		double	m_dist;

		double	Ft() const { return m_rad == 0 ? m_len : m_len * m_rad; }
	};

public:
	D6Track();
	virtual ~D6Track();

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

D6Track::D6Track()
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

D6Track::~D6Track()
{
	Empty();
}

void	D6Track::Empty()
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

void	D6Track::Initialise( const track_desc& track )
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
		m_pSeg[i].m_beg = D6Vec2(seg.beg_x, seg.beg_y);
		m_pSeg[i].m_end = D6Vec2(seg.end_x, seg.end_y);
		m_pSeg[i].m_cen = D6Vec2(seg.cen_x, seg.cen_y);

		m_pSeg[i].m_dist = track.seg_dist[i];
	}
}

void	D6Track::CalcLength()
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

int		D6Track::NSeg() const
{
	return m_nSeg;
}

double	D6Track::GetLength() const
{
	return m_length;
}

double	D6Track::GetWidth() const
{
	return m_width;
}

double	D6Track::GetMaxX() const
{
	return m_xMax;
}

void	D6Track::SetMaxX( double maxX )
{
	m_xMax = maxX;
}

double	D6Track::GetMaxY() const
{
	return m_yMax;
}

void	D6Track::SetMaxY( double maxY )
{
	m_yMax = maxY;
}

double	D6Track::GetStartX() const
{
	return m_startX;
}

double	D6Track::GetStartY() const
{
	return m_startY;
}

double	D6Track::GetStartAng() const
{
	return m_startAng;
}

double	D6Track::GetStartLine() const
{
	return m_startLine;
}

D6Track::Segment	D6Track::GetSegment( int seg ) const
{
	ASSERT( seg >= 0 && seg < m_nSeg );
	return m_pSeg[seg];
}

static double	CorneringSpeed( double radius )
{ 
	double	centreRadius = fabs(radius);
	return sqrt(centreRadius * g * CORN_MYU);
} 

/* Not used
static double	TurningRadius( double speed )
{ 
	return speed * speed / (g * CORN_MYU);
}
*/

/* Not used
static double	CorneringAccel( double speed, double radius )
{
	return speed * speed / radius;
}
*/

/* Not used
static double	BrakingDistance( double fromVel, double toVel )
{
	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL;
}
*/

static double	BrakingDistanceBend( double fromVel, double toVel )
{

	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL_BEND;
}

/* Not used
static double	VelocityAfterDistance( double u, double s, double a )
{
	return sqrt(2 * a * s + u * u);
}
*/

/* Not used
static double	CalcClosestPointOnLine( const D6Vec2& lp, const D6Vec2& lv, const D6Vec2& p )
{
	double	denom = lv * lv;
	if( denom == 0 )
		return 0;
	return ((p - lp) * lv) / denom;
}
*/

/* Not used
static double	CalcDistanceToLine( const D6Vec2& lp, const D6Vec2& lv, const D6Vec2& p )
{
	double	vLength = lv.GetLength();
	if( vLength == 0 )
		return 0;
	return (lv % (p - lp)) / vLength;
}
*/

inline bool	LineCrossesLine(
	const D6Vec2&	lp0,
	const D6Vec2&	lv0,
	const D6Vec2&	lp1,
	const D6Vec2&	lv1,

	double&		t )
{
	double	denom = lv0 % lv1;
	if( denom == 0 )
		return false;

	double	numer = lv1 % (lp0 - lp1);

	t = numer / denom;

	return true;
}

static double	CalcCurvature(
	const D6Vec2&	p1,
	const D6Vec2&	p2,
	const D6Vec2&	p3 )
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
	D6Vec2	p = p1 - p2;
	D6Vec2	q = p2 - p3;	// this vector has been negated
	D6Vec2	s = p3 - p1;
	double	K = (2 * (p % q)) / sqrt((p * p) * (q * q) * (s * s));
	return K;
}

class D6Slices
{
public:
	enum
	{
		cSliceFt		= 10,	// nominal length of each slice
	};

	struct Slice
	{
		D6Vec2	m_p;	// pt on right edge of track
		D6Vec2	m_n;	// normal at pt, for slice across track
		int		m_seg;	// segment this slice is in

		void		Setup( const D6Vec2& p, const D6Vec2& n, int seg )
						{ m_p = p; m_n = n; m_seg = seg; }
	};

public:
	D6Slices() : m_size(0), m_pSlices(0), m_width(1) {}
	~D6Slices() { delete [] m_pSlices; }

	void			Setup( const D6Track& track );

	int				GetSize() const				{ return m_size; }
	const Slice&	GetAt( int index ) const	{ return m_pSlices[index]; }
	Slice&			GetAt( int index )			{ return m_pSlices[index]; }
	double			GetWidth() const			{ return m_width; }
	double			GetLength() const			{ return m_length; }

	int				Find( const D6Vec2& p, int start = -1 ) const;
	int				Find( const D6Vec2& p, int seg, int start ) const;

private:
	int				m_size;
	Slice*			m_pSlices;
	double			m_width;
	double			m_length;
};

void	D6Slices::Setup( const D6Track& track )
{
	m_size = 0;
	delete [] m_pSlices;
	int		maxLines = int(track.GetLength() * 2 / cSliceFt + 0.5);
	m_pSlices = new Slice[maxLines];

	double	width = m_width = track.GetWidth();
	m_length = track.GetLength();

	int		size = 0;
	// double	totalDist = 0;
	// double	startOffset = track.GetStartLine();
	// double	totalLength = track.GetLength();

	for( int i = 0; i < track.NSeg(); i++ )
	{
		D6Track::Segment	seg = track.GetSegment(i);

		if( seg.m_rad == 0 )
		{
			int		n = int(seg.m_len / cSliceFt);

			D6Vec2	beg = seg.m_beg;
			D6Vec2	normal = (seg.m_end - beg).GetNormal().GetUnit();
			D6Vec2	d = seg.m_end - seg.m_beg;

			if( i == 0 )
			{
				D6Vec2	tan = (beg - seg.m_end).GetUnit();
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
					D6Vec2	normal = D6Vec2::FromAngle(angle + alpha);
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
					D6Vec2	normal = D6Vec2::FromAngle(angle - alpha);
					m_pSlices[size].Setup( seg.m_cen + normal * -seg.m_rad,
											normal, i );
					size++;
				}
			}
/*
			if( i == track.NSeg() - 1 )
			{
				double	angle = (seg.m_end - seg.m_cen).GetAngle();
				D6Vec2	normal = D6Vec2::FromAngle(angle);
				if( seg.m_rad < 0 )
					normal = -normal;
				m_pSlices[size].Setup( seg.m_cen + normal * seg.m_rad,
											-normal, seg.m_dist );
				size++;
			}
*/		}
	}

	// fixup the last points, so they don't extend past the first point
	D6Vec2	n = m_pSlices[0].m_n.GetNormal();
	while(	(m_pSlices[size - 1].m_p - m_pSlices[0].m_p) * n < 0 ||
			(m_pSlices[size - 1].m_p + m_pSlices[size - 1].m_n * m_width -
				m_pSlices[0].m_p) * n < 0 )
		size--;

	m_size = size;
}

int		D6Slices::Find( const D6Vec2& p, int start ) const
{
	start = max(start, 0);
	ASSERT( start >= 0 && start < m_size );

	int		cur_s = start;

	while( true )
	{
		int		next_s = (cur_s + 1) % m_size;
		const Slice&	s = GetAt(next_s);
		if( s.m_n.GetNormal() * (p - s.m_p) > 0 )
			break;

		cur_s = next_s;
	}

	return cur_s;
}

int		D6Slices::Find( const D6Vec2& p, int seg, int start ) const
{
	start = max(start, 0);
	ASSERT( start >= 0 && start < m_size );

	int		cur_s = start;
	int		nSeg = m_pSlices[m_size - 1].m_seg + 1;
	int		nextSeg = (seg + 1) % nSeg;

	while( true )
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
		if( cur_s == start )
			break;
	}

	return cur_s;
}

class D6Path
{
public:
	struct Path
	{
		D6Vec2	m_p;		// point on slice line
		double	m_spd;		// speed at point
	};

public:
	D6Path() : m_pSlices(0), m_pPath(0) {}
	D6Path( const D6Path& path );
	D6Path( const D6Slices* pSlices );
	~D6Path() { delete [] m_pPath; }

	void			Setup( const D6Slices* pSlices );
	int				GetSize() const { return m_pSlices->GetSize(); }

	const Path&		GetAt( int index ) const	{ return m_pPath[index]; }
	Path&			GetAt( int index )			{ return m_pPath[index]; }

	D6Path&			operator=( const D6Path& path );

	void			Calc( int cur_s, const D6Vec2& pt, double& dist,
							D6Vec2& vel, double& k );

	void			CalcSpeed();
	void			CalcSpeedSection( int from, int len );
	void			SetInitialSpeed( int from, int len );
	void			PropagateBraking( int from, int len );
	double			EstimateSpeed(situation &s) const;

private:
	const D6Slices*	m_pSlices;
	Path*			m_pPath;	// size from pSlices
};

D6Path::D6Path( const D6Path& path )
:	m_pSlices(path.m_pSlices),
	m_pPath(0)
{
	m_pPath = new Path[m_pSlices->GetSize()];
	for( int i = 0; i < m_pSlices->GetSize(); i++ )
		m_pPath[i] = path.m_pPath[i];
}

D6Path::D6Path( const D6Slices* pSlices )
:	m_pSlices(0),
	m_pPath(0)
{
	Setup( pSlices );
}

void	D6Path::Setup( const D6Slices* pSlices )
{
	m_pSlices = pSlices;
	delete [] m_pPath;
	m_pPath = new Path[pSlices->GetSize()];

	double	halfWidth = pSlices->GetWidth() * 0.5;
	for( int i = 0; i < pSlices->GetSize(); i++ )
	{
		const D6Slices::Slice&	s = pSlices->GetAt(i);
		m_pPath[i].m_p = s.m_p + s.m_n * halfWidth;
		m_pPath[i].m_spd = 50;
	}
}

D6Path&	D6Path::operator=( const D6Path& path )
{
	ASSERT( m_pSlices == path.m_pSlices );
	for( int i = 0; i < m_pSlices->GetSize(); i++ )
		m_pPath[i] = path.m_pPath[i];

	return *this;
}

void	D6Path::Calc(
	int			cur_s,
	const D6Vec2&	pt,

	double&		dist,
	D6Vec2&		vel,
	double&		k )
{
	int		sl1 = m_pSlices->Find(pt, cur_s);
	int		sl0 = (sl1 + m_pSlices->GetSize() - 1) % m_pSlices->GetSize();
	int		sl2 = (sl1 + 1) % m_pSlices->GetSize();
	int		sl3 = (sl2 + 1) % m_pSlices->GetSize();

	D6Vec2	p0 = m_pPath[sl0].m_p;
	D6Vec2	p1 = m_pPath[sl1].m_p;
	D6Vec2	p2 = m_pPath[sl2].m_p;
	D6Vec2	p3 = m_pPath[sl3].m_p;
	double	k1 = CalcCurvature(p0, p1, p2);
	double	k2 = CalcCurvature(p1, p2, p3);
	D6Vec2	v1 = p2 - p1;
	D6Vec2	v2 = p3 - p2;
	double	t1 = (pt - p1) * v1 / (v1 * v1);
	double	spd1 = m_pPath[sl1].m_spd;
	double	spd2 = m_pPath[sl2].m_spd;
	double	spd = spd1 * (1 - t1) + spd2 * t1;
	D6Vec2	dir = v1.GetUnit() * (1 - t1) + v2.GetUnit() * t1;

	dist = v1.GetUnit().GetNormal() * (pt - p1);
	vel = dir * spd;
	k = k1 * (1 - t1) + k2 * t1;
}

void	D6Path::CalcSpeed()
{
	SetInitialSpeed( 0, m_pSlices->GetSize() );
	PropagateBraking( 0, m_pSlices->GetSize() + 1 );
	PropagateBraking( 0, m_pSlices->GetSize() + 1 );
}

void	D6Path::CalcSpeedSection( int from, int len )
{
	SetInitialSpeed( from, len );
	PropagateBraking( from, len );
}

void	D6Path::SetInitialSpeed( int from, int len )
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

		D6Vec2	v1 = p1->m_p - p0->m_p;
		D6Vec2	v2 = p2->m_p - p1->m_p;

		double	cosAng = (v1 * v2) / (v1.GetLength() * v2.GetLength());
		if( cosAng == 1.0 )
		{
			p1->m_spd = 300;
		}
		else
		{
			double	curvature = CalcCurvature(p0->m_p, p1->m_p, p2->m_p);
			double	speed = CorneringSpeed(1 / curvature);
			p1->m_spd = min(300, speed);
		}
	}
}

void	D6Path::PropagateBraking( int from, int len )
{
	const int		size = m_pSlices->GetSize();

	const double	cMaxA = MAX_ACCEL * CORN_MYU;
	const double	cMaxMass = (M + MAX_FUEL / g);

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
			double	a0 = (p1->m_p - p0->m_p).GetAngle();
			double	a1 = (p2->m_p - p1->m_p).GetAngle();

			double	alpha = NormaliseAngleLimit(a1 - a0, 0);
			double	s = (p1->m_p - p0->m_p).GetLength();

			double	u = p0->m_spd;
			double	v = p1->m_spd;

			while( true )
			{
				double	spd = (u + v) / 2;
				double	cen_a = alpha * spd * spd / s;

				double	inner = cMaxA * cMaxA - cen_a * cen_a;
				double	tan_a = inner <= 0 ? 0 : -sqrt(inner);

				double	drag_a = DRAG_CON * spd * spd / cMaxMass;
				tan_a -= drag_a;

				inner = v * v - 2 * tan_a * s;
				double	max_u = inner <= 0 ? 0 : sqrt(inner);

				if( u <= max_u )
					break;

				u = max_u;
			}

			// double	brakeDist = BrakingDistance(u, v);
			ASSERT( BrakingDistance(u, v) <= s );

			if( u < p0->m_spd )
				p0->m_spd = u;
		}
	}
}

double	D6Path::EstimateSpeed(situation &s) const
{
	const double	cMaxA = MAX_ACCEL * CORN_MYU;
	const double	cMass = (M + MAX_FUEL / g);
	const int		size = m_pSlices->GetSize();

	double	totalTime;
	double*	pSpeed = new double[size];
	pSpeed[size - 1] = m_pPath[size - 1].m_spd;

	for( int j = 0; j < 2; ++j )
	{
		totalTime = 0;

		for( int i = 0, prev = size - 1; i < size; i++ )
		{
			D6Vec2	p0 = m_pPath[prev].m_p;
			D6Vec2	p1 = m_pPath[i].m_p;
			double	spd = pSpeed[prev];

			double	dist = (p1 - p0).GetLength();
			double	time = dist / spd;
			totalTime += time;

			if( spd > m_pPath[i].m_spd )
				pSpeed[i] = m_pPath[i].m_spd;
			else
			{
				D6Vec2	p2 = m_pPath[(i + 1) % size].m_p;

				double	a0 = (p1 - p0).GetAngle();
				double	a1 = (p2 - p1).GetAngle();

				double	alpha = NormaliseAngleLimit(a1 - a0, 0);

				double	cenA = alpha * spd * spd / dist;
				double	inner = cMaxA * cMaxA - cenA * cenA;
				double	maxA = inner <= 0 ? 0 : sqrt(inner);

				double	drag = DRAG_CON * spd * spd / cMass;
				double	tanA = cos(alpha) * s.pm / (cMass * spd);

				if( tanA > maxA )
					tanA = maxA;

				double	newSpeed = spd + time * (tanA - drag);

				pSpeed[i] = min(m_pPath[i].m_spd, newSpeed);
			}

			prev = i;
		}
	}

	delete [] pSpeed;

	return MPH_FPS * m_pSlices->GetLength() / totalTime;
}

class D6OptPath  
{
public:
	enum
	{
		cSegFt = D6Slices::cSliceFt,
	};

	struct	Line
	{
		D6Vec2	m_l;	// start point normal of line
		D6Vec2	m_v;	// direction of normal line
		double	m_w;	// distance along normal line
		D6Vec2	m_p;	// point on path
		double	m_lft;	// buffer from left
		double	m_rgt;	// buffer from right
		double	m_k;	// curvature at this point

		void	Setup( const D6Vec2& l, const D6Vec2& v, double w );
		void	SetW( double w );
	};

public:
	D6OptPath( const D6Slices& slices );
	~D6OptPath();

	bool		Load( const char* pName );

	void		SetFactor( double factor );
	void		Optimise();
	void		ModifySection( int from, int len, double delta, int mid = -1 );
	void		ModifySectionCircle( int from, int len, double delta, int mid = -1 );
	bool		ModifySectionAfter( int from, int len, double delta );
	void		CopyToPath( D6Path& path ) const;
	void		SetFromPath( const D6Path& path );

//	int			GetSize() const;
	const Line&	GetAt( int i ) const;
	Line&		GetAt( int i );
//	double		GetWidth() const { return m_width; }

	double		CalcSum() const;
	void		Optimise( int step, int nIterations );

private:
	D6OptPath();

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

void	D6OptPath::Line::Setup( const D6Vec2& l, const D6Vec2& v, double w )
{
	m_l = l;
	m_v = v;
	m_w = w;
	m_p = l + v * w;
	m_lft = 0;
	m_rgt = 0;
	m_k = 0;
}

void	D6OptPath::Line::SetW( double w )
{
	m_w = w;
	m_p = m_l + m_v * w;
}

D6OptPath::D6OptPath( const D6Slices& slices )
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
//	m_pLines = new Line[10000]

	for( int i = 0; i < m_nLines; i++ )
	{
		const D6Slices::Slice&	s = slices.GetAt(i);
		m_pLines[i].Setup( s.m_p, s.m_n, m_width * 0.5 );
	}
}

D6OptPath::~D6OptPath()
{
	delete [] m_pLines;
}

bool	D6OptPath::Load( const char* pName )
{
	FILE*	pFile = fopen(pName, "r");
	if( pFile == 0 )
		return false;

	int		n =0;
	if( fscanf(pFile, "%d", &n) != 1 || n != m_nLines )
	{
		fclose( pFile );
		return false;
	}

	for( int i = 0; i < n; i++ )
	{
		fscanf( pFile, "%lf", &m_pLines[i].m_w );
		m_pLines[i].SetW( m_pLines[i].m_w );
	}

	return true;
}

void	D6OptPath::SetFactor( double factor )
{
	m_factor = factor;
}

void	D6OptPath::Optimise()
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

void	D6OptPath::ModifySection( int from, int len, double delta, int important )
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

	D6Vec2	p0 = GetAt(from).m_p;
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

		D6Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
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

		D6Vec2	p1 = l1.m_l + l1.m_v * w;

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

		D6Vec2	tan = (l2.m_p - p0).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		p0 = l1.m_p;

		l1.m_w += offset;
		l1.m_p = l1.m_l + l1.m_v * l1.m_w;
	}

	delete [] pDist;
}

void	D6OptPath::ModifySectionCircle( int from, int len, double delta, int important )
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

	D6Vec2	p0 = GetAt(from).m_p;
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

		D6Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
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

		D6Vec2	p1 = l1.m_l + l1.m_v * w;

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

		D6Vec2	tan = (l2.m_p - p0).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		p0 = l1.m_p;

		l1.m_w += offset;
		l1.m_p = l1.m_l + l1.m_v * l1.m_w;
	}

	delete [] pDist;
}

bool	D6OptPath::ModifySectionAfter( int from, int len, double delta )
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
	D6Vec2	p0 = GetAt(from).m_p;
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

		D6Vec2	tan = (l2.m_p - l0.m_p).GetUnit().GetNormal();
		double	dot = tan * l1.m_v;
		offset /= fabs(dot);

		double	w = l1.m_w + offset;
		D6Vec2	p1 = l1.m_l + l1.m_v * w;

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
	return true;
}

void	D6OptPath::CopyToPath( D6Path& path ) const
{
	for( int i = 0; i < m_nLines; i++ )
		path.GetAt(i).m_p = GetAt(i).m_p;
}

void	D6OptPath::SetFromPath( const D6Path& path )
{
	for( int i = 0; i < m_nLines; i++ )
	{
		Line&	l = GetAt(i);
		l.m_p = path.GetAt(i).m_p;
		l.m_w = (l.m_p - l.m_l) * l.m_v;
		l.m_lft = 0;
		l.m_rgt = 0;
	}
}

const D6OptPath::Line&	D6OptPath::GetAt( int i ) const
{
	return m_pLines[i];
}

D6OptPath::Line&	D6OptPath::GetAt( int i )
{
	return m_pLines[i];
}

double	D6OptPath::CalcSum() const
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

void	D6OptPath::Optimise( int step, int nIterations )
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

void	D6OptPath::SmoothBetween( int step )
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

void	D6OptPath::Optimise(
	Line*		l3,
	double		e,

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

class D6CarPaths
{
public:
	struct	CarInfo
	{
		int		m_last;			// index of last path segment for this car
		D6Vec2	m_lastPt;
		double	m_lastSpeed;
		D6Path	m_path;			// path taken by this car
	};

public:
	D6CarPaths( int myCarId, const D6Slices& slices, const D6OptPath& opt );
	~D6CarPaths();

	void			Setup( int myCarId );
	int				GetLast( int carId ) const		{ return m_car[carId].m_last; }
	const D6Path&	GetPath( int carId ) const		{ return m_car[carId].m_path; }
	D6Path&			GetPath( int carId )			{ return m_car[carId].m_path; }
	const D6Path&	GetBestPath( int carId ) const	{ return m_carBest[carId].m_path; }
	D6Path&			GetBestPath( int carId )		{ return m_carBest[carId].m_path; }
	const D6Path&	GetOptPath() const				{ return m_optPath; }
	const D6Path&	GetBestPath() const				{ return m_bestPath; }
	void			SetBestPath();
	int				GetBestCar() const				{ return m_bestCar; }
	double			GetBestSpeed() const			{ return m_bestSpeed; }
	void			Record( int carId, const ::Car* pCar );
	void			StartRecord(	int carId, int cur_s,
									const D6Vec2& carPt, double speed );
	void			Record( int carId, const D6Vec2& carPt, double speed );

private:
	int				m_myId;
	const D6Slices&	m_slices;	// for path segment info

	int		m_nCars;
	CarInfo	m_car[MAX_CARS];		// path info for each car
	CarInfo	m_carBest[MAX_CARS];	// best path for each car

	D6Path	m_optPath;			// optimised path for dodger

	int		m_bestCar;			// car which holds best lap
	double	m_bestSpeed;		// speed of best lap
	D6Path	m_bestPath;			// best lap of race by any car
};

D6CarPaths::D6CarPaths( int myCarId, const D6Slices& slices, const D6OptPath& opt )
:	m_myId(myCarId),
	m_slices(slices),
	m_nCars(args.m_iNumCar),
	m_bestCar(-1),
	m_bestSpeed(0)
{
	int	i;
	for( /*int*/ i = 0; i < args.m_iNumCar; i++ )
	{
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
	for( /*int*/ i = 0; i < args.m_iNumCar; i++ )
	{
		m_car[i].m_path = m_optPath;
		m_carBest[i].m_path = m_optPath;
	}

	m_bestPath = m_optPath;
}

D6CarPaths::~D6CarPaths()
{
}

void	D6CarPaths::SetBestPath()
{
	// perform a small optimisation to smooth the path slightly
//	D6OptPath	opt(m_slices);
//	opt.SetFromPath( m_bestPath );
//	opt.Optimise( 1, 10 );

	// copy the other guys path into our path...
//	opt.CopyToPath( m_optPath );
//	m_optPath.CalcSpeed();
	m_optPath = m_bestPath;
	m_car[m_myId].m_path = m_bestPath;
}

void	D6CarPaths::Record( int carId, const ::Car* pCar )
{
	if( pCar->Out )
		return;

	// work out position and speed of car;
	D6Vec2	carPt(pCar->X, pCar->Y);
	double	speed = const_cast<Car*>(pCar)->get_speed();

	Record( carId, carPt, speed );

	if( pCar->Bestlap_speed > m_bestSpeed )
	{
		m_bestCar	= carId;
		m_bestPath  = m_car[carId].m_path;
		m_bestSpeed = pCar->Bestlap_speed;

		// perform a small optimisation to smooth the path slightly
		D6OptPath	opt(m_slices);
		opt.SetFromPath( m_bestPath );
		opt.Optimise( 1, 10 );
		opt.CopyToPath( m_bestPath );
		m_bestPath.CalcSpeed();
	}

	if( pCar->Lastlap_speed > m_carBest[carId].m_lastSpeed )
	{
		m_carBest[carId].m_path = m_car[carId].m_path;
		m_carBest[carId].m_lastSpeed = pCar->Lastlap_speed;
	}
}

void	D6CarPaths::StartRecord(
	int carId, int cur_s, const D6Vec2& carPt, double speed )
{
	m_car[carId].m_last = cur_s;
	m_car[carId].m_lastPt = carPt;
	m_car[carId].m_lastSpeed = speed;
}

void	D6CarPaths::Record( int carId, const D6Vec2& carPt, double speed )
{
	// work out which slice the car is in
	int		last_s = m_car[carId].m_last;
	int		cur_s = m_slices.Find(carPt, race_data.cars[carId]->Seg_id, last_s);

	if( !race_data.cars[carId]->On_pit_lane &&
		last_s >= 0 && last_s != cur_s )
	{
		// we have crossed at least one line boundary, so we need to calculate
		//	the crossing point(s), and the speed at that point.

		D6Vec2	lastPt = m_car[carId].m_lastPt;
		double	lastSpeed = m_car[carId].m_lastSpeed;

		int		next_s = (last_s + 1) % m_slices.GetSize();
		while( last_s != cur_s )
		{
			const D6Slices::Slice&	s0 = m_slices.GetAt(next_s);
			double		t;
			if( LineCrossesLine(lastPt, carPt - lastPt, s0.m_p, s0.m_n, t) )
			{
				D6Vec2	crossPt = lastPt + (carPt - lastPt) * t;
				double	crossSpeed = lastSpeed + (speed - lastSpeed) * t;

				D6Path::Path& path = m_car[carId].m_path.GetAt(next_s);
				path.m_p	= crossPt;
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

/////////////////////////////////////////////////////////////////////////////

struct	D6Situation
{
	D6Vec2	m_carPt;		// global car coordinates
	D6Vec2	m_carDir;		// global car unit direction vector
	double	m_carAngle;		// global car angle
	double	m_tanDist;		// distance till tangent point

	D6Vec2	m_distDir;		// direction to most distant point down track
	double	m_curvature;	// curvature at current position on path
	int		m_slice;		// current position on path
	bool	m_nextTurnIsLeft;	// true if will be turning left
};

/////////////////////////////////////////////////////////////////////////////

static bool	CheckOtherIn( const D6Vec2& p, double sine, double cosine )
{
	D6Vec2	l = D6Vec2(cosine, sine) * (CARLEN / 2);
	D6Vec2	w = D6Vec2(sine, -cosine) * (CARWID / 2);

	D6Vec2	rl = p - l - w;
	if( rl.x > -CARWID / 2 && rl.x < CARWID / 2 &&
		rl.y > -CARLEN / 2 && rl.y < CARLEN / 2 )
		return true;

	D6Vec2	rr = p - l + w;
	if( rr.x > -CARWID / 2 && rr.x < CARWID / 2 &&
		rr.y > -CARLEN / 2 && rr.y < CARLEN / 2 )
		return true;

	D6Vec2	fl = p + l - w;
	if( fl.x > -CARWID / 2 && fl.x < CARWID / 2 &&
		fl.y > -CARLEN / 2 && fl.y < CARLEN / 2 )
		return true;

	D6Vec2	fr = p + l + w;
	if( fr.x > -CARWID / 2 && fr.x < CARWID / 2 &&
		fr.y > -CARLEN / 2 && fr.y < CARLEN / 2 )
		return true;

	return false;
}

/////////////////////////////////////////////////////////////////////////////

static bool	CheckCollide( const D6Vec2& p, double angle )
{
	if( p * p >= cCarDiag * cCarDiag )
		return false;

	if( p * p < CARWID * CARWID )
		return true;

	double	s = sin(angle + cPi_2);
	double	c = cos(angle + cPi_2);

	if( CheckOtherIn(p, s, c) )
		return true;

	D6Vec2	p2(p.y * c - p.x * s, -p.x * c - p.y * s);
	return CheckOtherIn(p2, -s, c);
}

/////////////////////////////////////////////////////////////////////////////

static bool	AvoidOtherCars(
	const situation&	s,				// current situation
	const D6Situation&	mySituation,	// global situation values

	const D6CarPaths&	carPaths,		// paths of all cars
	double&				vc,				// modified speed
	double&				adjust,			// amount to adjust track
	bool&				needToBrake )	// true if need to brake
{
	bool		gotCar = false;
	rel_state	car;
	double		collideTime = 10;
	D6Vec2		collidePos1;
	D6Vec2		collideDir1;
	D6Vec2		collidePos2;

	// work out a predicted circular path for our car
	double	rad = s.alpha == 0 ? 1000000000 : s.v / s.alpha;

	const D6Path&	myPath = carPaths.GetPath(s.my_ID);
	const int		nSlices = myPath.GetSize();
	int				mySlice = mySituation.m_slice;
	double	k = CalcCurvature(myPath.GetAt(mySlice).m_p,
					myPath.GetAt((mySlice + 1) % nSlices).m_p,
					myPath.GetAt((mySlice + 2) % nSlices).m_p );
	rad = k == 0 ? 1000000000 : 1.0 / k;
		
	double	alpha = s.alpha == 0 ? s.v / rad : s.alpha;
	alpha = s.v / rad;

	// work out the acceleration we could hope to achieve
	double	tanA = 0;
//	double	spd0 = myPath.GetAt(mySlice).m_spd;
	double	spd1 = myPath.GetAt((mySlice + 1) % nSlices).m_spd;
	if( s.v < spd1 )
	{
		const double	cMaxA = MAX_ACCEL * CORN_MYU;
		const double	cMass = (M + MAX_FUEL / g);

		double	cenA = s.v * s.v / rad;
		double	inner = cMaxA * cMaxA - cenA * cenA;
		double	maxA = inner <= 0 ? 0 : sqrt(inner);

		// double	drag = DRAG_CON * s.v * s.v / cMass;
		tanA = cos(alpha) * s.pm / (cMass * s.v);

		if( tanA > maxA )
			tanA = maxA;
	}

//	tanA *= 0.5;
	tanA = 0.5;
	double	rotA = tanA / rad;

	D6Vec2	cen = D6Vec2(-rad, 0);

	// see if we are going to hit another car
	bool	colliding = false;
	adjust = 0;
	bool	badCollision = false;
	for( int i = 0; i < NEARBY_CARS; i++ )
	{
		if( s.nearby[i].who >= MAX_CARS ||
			race_data.cars[s.nearby[i].who]->On_pit_lane )
			continue;

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

		// don't catch up to any cars going more than 40fps faster
		//	by the time we overtake
		double	brakeDist = BrakingDistanceBend(s.v, s.nearby[i].v + 40);
		if( y > CARLEN && brakeDist > y )
		{
//			needToBrake = true;
		}

		// don't pass other cars going more than 15fps faster than them.
		//	only include cars within ±30° of current direction of travel.
		brakeDist = BrakingDistanceBend(s.v, s.v + cv + 15);
		if( y > CARLEN && y < brakeDist && fabs(x) <= y / 2 + CARWID )
		{
			needToBrake = true;
		}

		// if other car has more damage than MAX_DAMAGE, then the automatic
		//	routine takes hold of it, and clears it off of the right edge
		//	of the track.  we make sure that cars in this situation have a
		//	minimum velocity the right, so that we will attempt to drive
		//	around them on the left.
		if( race_data.cars[s.nearby[i].who]->Damage > MAX_DAMAGE &&
			vn > -3 )
		{
			vx = vx - (vn + 3);
			vn = -3;
			al += -0.1;
		}

		// work out the velocity vector of the other car
		D6Vec2	vDir(vx, s.v + vy);
		D6Vec2	vNorm = vDir.GetNormal().GetUnit();

		// use alpha and velocity to predict a circular path for the car.
		double	r = al == 0 ? 1000000000 : v / al;
		if( al == 0 )
			al = v / r;

		double	deltaV = 0;
		if( race_data.cars[who]->Laps > 1 )
		{
			const D6Path&	hisPath = carPaths.GetBestPath(who);
			int				hisSlice = carPaths.GetLast(who);
			k = CalcCurvature(hisPath.GetAt(hisSlice).m_p,
							hisPath.GetAt((hisSlice + 1) % nSlices).m_p,
							hisPath.GetAt((hisSlice + 2) % nSlices).m_p );
			r = k == 0 ? 1000000000 : 1.0 / k;
			al = v / r;
			deltaV = hisPath.GetAt((hisSlice + 10) % nSlices).m_spd -
					 hisPath.GetAt(hisSlice).m_spd;
		}

//		double	la = competitionData.cars[who]->get_lin_acc() * g * 0.5;
		double	la = race_data.cars[who]->get_lin_acc() < 0 ? -15 : 0;
		double	w = la / r;

		D6Vec2	c = D6Vec2(x, y) + vNorm * r;

		// check if we are currently colliding with the other car.
		// work out angle of other car in Sparky's local coord system,
		//	with car pointing along velocity vector.  therefore zero degrees
		//	is along velocity vector, which is the x axis.
		double	relAngle = vDir.GetAngle() - cPi_2;//atan2(-vx, s.v + vy);
		if( collide(x, y, relAngle) )
		{
			colliding = true;
			if( y > 0 && vy < 0 )
			{
//				needToBrake = true;
				vc = s.v + vy - 0.1;
			}
		}

		// work out if we will hit the other car, up to to 3 seconds into
		//	the future
		double	minDist = cCarDiag;	// ft
		double	minTime = 10;			// seconds
		D6Vec2	minP1;
		D6Vec2	minDir;
		D6Vec2	minP2;
		double	collDist = 0;
		double	deltaT = 0.05;			// 1/20 sec
                double  time = 0;
		for( time = 0; time < 3; time += deltaT )
		{
//			double	a1 = alpha * time;
			double	a1 = alpha * time + rotA * time * time;
			D6Vec2	p1 = cen + D6Vec2::FromAngle(a1) * rad;

//			double	a2 = relAngle + al * time;
			double	a2 = relAngle + al * time + w * time * time;
			D6Vec2	p2 = c + D6Vec2::FromAngle(a2) * r;

			D6Vec2	p = p2 - p1;
			double	dist = p.GetLength();
			if( dist < minDist )
			{
				if( CheckCollide(p, a2 - a1) )
					dist = min(dist, cCarDiag);
				else
					continue;

				minDist = dist;
				minTime = time;
				minP1 = p1;
				minDir = D6Vec2::FromAngle(a1 + cPi_2);
				minP2 = p2;
				collDist = fabs(alpha * (time - deltaT) * rad);
				if( dist < cCarDiag )
					break;
			}
		}

		// ignore if collision more than 3 secs away
		if( minTime > 3 )
			continue;

		if( minTime < collideTime )
		{
			gotCar = true;
			car = s.nearby[i];
			car.vn = vn;
			collideTime = minTime;
			collidePos1 = minP1;
			collideDir1 = minDir;
			collidePos2 = minP2;
		}

//		double	a1 = alpha * time + rotA * time * time;
//		double	a2 = relAngle + al * time + w * time * time;
//		double	aa = a2 - a1;
//		double	fv = v + la * collideTime;
//		double	mfv = fv * cos(aa);
//		double	finalV = s.v + vy;
		double	finalV = s.v + vy + (la - tanA) * collideTime;
		brakeDist = max(0, BrakingDistanceBend(s.v, finalV));
//		brakeDist = max(0, BrakingDistance(s.v, mfv));
		if( y > 0 && vy < 0 && brakeDist >= collDist )
		{
			needToBrake = true;
			if( brakeDist > collDist * 1.2 && car.rel_ydot < -10 )
				badCollision = true;
		}
	}

	if( collideTime < 10 )
	{
		D6Vec2	norm = collideDir1.GetNormal();
		double	dot = (collidePos2 - collidePos1) * norm;

		// work out how "urgent" it is to dodge the other car,
		//	related to how far ahead it is
//		double	urgency = max((10 * CARLEN - car.rel_y) / (9 * CARLEN), 0);
//		urgency = min(urgency, 1);
		double	urgency = 0.5;//min(1, sqrt(max(1 - collideTime * 0.15, 0)));
		if( car.rel_y < CARLEN && dot * mySituation.m_curvature > 0 )
			urgency = 1.0;

		// got car at side ... which side?
		if( urgency > 0 )
		{
			double	delta = 1.0;
			if( badCollision )//car.rel_ydot < -10 )
				delta *= 2;
			bool	out = race_data.cars[car.who]->Damage > MAX_DAMAGE;
//			const double	delta = 0.5;
			if( dot > 0 )
			{
				// can we get to the right of the car?
//				if( car.to_rgt + car.vn * collideTime > cCarWid )
				if( out && car.to_rgt + car.vn * collideTime > cCarWid )
				{
					if( s.to_rgt > cCarWid * 1.1 )//|| y < CARLEN )
						adjust = -delta * urgency;
				}
				else if( !out && car.to_rgt > cCarWid )
				{
					if( s.to_rgt > cCarWid * 1.1 )//|| y < CARLEN )
//					if( s.to_rgt > CARWID * 0.5 )//|| y < CARLEN )
						adjust = -delta * urgency;
					else if(
						car.to_rgt + car.vn * collideTime < cCarWid &&
						s.to_rgt + collideTime * s.vn < 5 )
					{
						adjust = delta * urgency;
					}
				}
				else
				{
					if( car.rel_y > CARLEN + 0.5 )
						adjust = delta * urgency;
				}
			}
			else
			{
				// can we get to the left of the car?
//				if( car.to_lft - car.vn * collideTime > cCarWid )
				if( car.to_lft > cCarWid )
				{
					if( s.to_lft > cCarWid * 1.1 )//|| y < CARLEN )
						adjust = delta * urgency;
					else if(
						car.to_lft - car.vn * collideTime < cCarWid &&
						s.to_lft - collideTime * s.vn < 5 )
					{
						adjust = -delta * urgency;
					}
				}
				else
				{
					if( car.rel_y > CARLEN + 0.5 )
						adjust = -delta * urgency;
				}
			}

			// figure the angle we would be using for alpha if we didn't
			//	change it with the adjust
			// double	actualAlpha = NormaliseAngleLimit(
			//	 mySituation.m_distDir.GetAngle() -
			//	 mySituation.m_carAngle, 0);

			// don't try to pass by turning the other way than
			//	we are already
			if( !badCollision &&
				mySituation.m_tanDist < 100 && //car.rel_ydot > -5 &&
				(mySituation.m_nextTurnIsLeft  && adjust < 0 ||
				 !mySituation.m_nextTurnIsLeft && adjust > 0) )
				adjust *= 0;//0.2;//0.5;
//			if( fabs(mySituation.m_curvature) > 0.001 &&
//				(mySituation.m_curvature > 0 && adjust < 0 ||
//				 mySituation.m_curvature < 0 && adjust > 0) )
//				adjust *= 0.5;
		}
	}

	if( needToBrake )
		needToBrake = true;

	return colliding;
}

/////////////////////////////////////////////////////////////////////////////

class	D6InverseFriction
{
public:
	enum { cSize = 500 };

public:
	D6InverseFriction();
	~D6InverseFriction();

	double	operator()( double friction );

private:
	double	m_graph[cSize + 1];
	double	m_step;
};

/////////////////////////////////////////////////////////////////////////////

D6InverseFriction::D6InverseFriction()
{
	// scan friction function to find max slip, max friction
	double	maxFriction = -1;
	double	maxSlip = 0;
	double	oldF = -1;
	for( double slip = 0; ; slip += 0.01 )
	{
		double	 f = friction(slip);

		if( f < oldF + 0.0001 )
			break;

		if( f > maxFriction )
			maxFriction = f;

		maxSlip = slip;
		oldF = f;
	}

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

D6InverseFriction::~D6InverseFriction()
{
}

/////////////////////////////////////////////////////////////////////////////

double	D6InverseFriction::operator()( double friction )
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

class Dodger6 : public Driver
{

public:

  char name[];

  D6Track		track;
  D6Slices		slices;
  D6OptPath*	pOptPath;
  D6CarPaths*	pCarPaths;
  int			cur_p;
  int			upd_p;
  int			softServo;
  int			bestCar;
  bool			offsetTrack;
  double		speedEstimate;
  D6InverseFriction	invFriction;
  int		damage;

  Dodger6() 
  {

    m_iNoseColor = oWHITE;
    m_iTailColor = oBLACK;
    m_sBitmapName2D = "car_white_black";
    m_sModel3D = NULL;
    m_sName = "Dodger6";

    char name[] = "Dodger6";	// This is the robot driver's name! 
    D6OptPath*	pOptPath = 0;
    D6CarPaths*	pCarPaths = 0;
    int			cur_p = 0;
    int			upd_p = 0;
    int			softServo = 0;
    int			bestCar = -1;
    bool			offsetTrack = false;
    double		speedEstimate = 0;
    D6InverseFriction	invFriction;
    int		damage = 0;
  }

  con_vec drive(situation& s);
  void	PitControl( const situation& s, con_vec& control, int& damage );
  
};

void	Dodger6::PitControl( const situation& s, con_vec& control, int& damage )
{
	int		totalDamage = 0;
	int		lastDamage = 0;
	int		lastRepairLap = 0;
	double	totalFuel = 0;
	double	fuelPerLap = 10;
	double	lastFuel = 0;

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
			control.fuel_amount = MAX_FUEL;
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
	bool	repairDamage =	(s.damage >= 25000 ||
							 s.damage >= 20000 && predDamage >= 25000 ||
							 s.damage >= 15000 && predDamage >= 30000);

	int		lapsOnFullTank = int(MAX_FUEL / fuelPerLap);
	int		lapsOnFuel = int(s.fuel / fuelPerLap);
	int		nStopsIfRefuelNow = (s.laps_to_go + lapsOnFullTank - 1) /
									lapsOnFullTank;
	int		nStopsIfRefuelLater = (s.laps_to_go - lapsOnFuel +
									lapsOnFullTank - 1) / lapsOnFullTank;
	bool	delayStop = nStopsIfRefuelNow > nStopsIfRefuelLater;
	if( repairDamage && s.damage + aveDamage < 20000 && delayStop )
		repairDamage = false;

	double	trackLen = get_track_description().length;
	double	fuelToEndOfLap = (1 - s.distance / trackLen) * fuelPerLap;
	bool	canFinishLap = fuelToEndOfLap < s.fuel;
	if( s.stage != QUALIFYING &&
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

		double	repair = 0;
		if( repairDamage )
		{
			// repair the amount needed to get us to the end with
			//	about 15000 damage
			repair = predDamage - 15000;
		}
		else
		{
			// we have just stopped in to re-fuel... but we will still
			//	repair some damage, related to how far we have yet
			//	to travel.
			repair = (s.laps_to_go - 5) * 250;
		}

		// limit repair to min=freeRepair and max=s.damage
		control.repair_amount = (int)min(max(freeRepair, repair), s.damage);
	}
	else
		control.request_pit = false;
//	control.request_pit = true;
}


con_vec Dodger6::drive( situation& s )
{ 

  //  printf("drive start\n");

  con_vec	result;		// control vector

	if( s.starting )
	{
	  	//printf("drive 1\n");

		// first time only, copy name: 
		my_name_is(name);

		track_desc	trackDesc = get_track_description();

		result.alpha = 0;
		result.vc = 300; 

		// set up initial values for pit, fuel and damage
		PitControl( s, result, damage );

		// change grip available depending on track surface
		switch( args.m_iSurface )
		{
			default:
			case 0:		CORN_MYU = MYU_MAX0;	break;
			case 1:		CORN_MYU = MYU_MAX1;	break;
			case 2:		CORN_MYU = MYU_MAX2;	break;
		}

		//printf("drive 2\n");

		// Changed 'FINISH' to 'FINISHED' by Carsten Kjaer
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
			pOptPath = new D6OptPath(slices);
			pOptPath->SetFactor( FindTrackFactor() );
			char	dataFileName[250];
			strcpy( dataFileName, trackDesc.sName );
			strcat( dataFileName, ".txt" );
			if( !pOptPath->Load(dataFileName) )
				pOptPath->Optimise();
			pCarPaths = new D6CarPaths(s.my_ID, slices, *pOptPath);
      if( s.my_ID<=args.m_iNumCar )
			  speedEstimate = pCarPaths->GetPath(s.my_ID).EstimateSpeed(s);
		}

		cur_p = 0;
		softServo = 100;
		bestCar = s.my_ID;
		offsetTrack = true;

		s.out_pits = 0;

		//printf("drive end 1\n");

		return result; 
	} 

	//printf("drive 10\n");

	// figure whether we need to refuel or repair damage (or both)
	PitControl( s, result, damage );

	//printf("drive 11\n");

	if( s.out_pits )
		softServo = 100;

	//printf("drive 12\n");

	// ask to see cars that are to the side, but slightly behind us
	s.side_vision = true;


	//printf("drive 13\n");

	// get details of current track segment
	const D6Track::Segment	seg = track.GetSegment(s.seg_ID);

	// work out current position of car, global coords
	D6Vec2	carPt(race_data.cars[s.my_ID]->X, race_data.cars[s.my_ID]->Y);

	//printf("drive 15\n");

	// work out current direction of car, global angle
	double	carAngle = asin(s.vn / s.v);
	if( s.cur_rad == 0 )
		carAngle += (seg.m_end - seg.m_beg).GetAngle();
	else if( s.cur_rad > 0 )
		carAngle += (carPt - seg.m_cen).GetAngle() + cPi_2;
	else
		carAngle += (carPt - seg.m_cen).GetAngle() - cPi_2;


	//printf("drive 19\n");

	carAngle = NormaliseAngle(carAngle);


	//printf("drive 20\n");

	D6Vec2	carDir = D6Vec2::FromAngle(carAngle);
//	D6Vec2	carVel = carDir * s.v;

	//
	// work out where we are on the path around the track

	cur_p = slices.Find(carPt, s.seg_ID, cur_p);
	int		prev_p = (cur_p + slices.GetSize() - 1) % slices.GetSize();
	int		next_p = (cur_p + 1) % slices.GetSize();
//	int		aft_p = (next_p + 1) % slices.GetSize();

	//
	// change the path slice at the track position directly opposite us
	//	back to the optimised version

	D6Path&	path = pCarPaths->GetPath(s.my_ID);

	int		upd_cur = (cur_p + slices.GetSize() / 2) % slices.GetSize();
	while( upd_p != upd_cur )
	{
		path.GetAt(upd_p) = pCarPaths->GetOptPath().GetAt(upd_p);
		upd_p = (upd_p + 1) % slices.GetSize();
	}

	//
	// figure out our current distance from the path, etc.

	int		count = 0;

	//printf("drive 50\n");

calcTargets:

	//printf("drive 51\n");

//	D6Vec2	p0 = path.GetAt(prev_p).m_p;
//	D6Vec2	p1 = path.GetAt(cur_p).m_p;
//	D6Vec2	p2 = path.GetAt(next_p).m_p;
//	D6Vec2	p3 = path.GetAt(aft_p).m_p;

	double	dist1, k1;
	D6Vec2	vel1;
	path.Calc( cur_p, carPt, dist1, vel1, k1 );

	//
	// modify path if we are too far from it

//	if( count == 0 && (fabs(dist1) > 10 || offsetTrack) )
	if( count == 0 && (fabs(dist1) > 5 || offsetTrack) )
	{
		pOptPath->SetFromPath( pCarPaths->GetPath(s.my_ID) );
		double	adjust = dist1;
		if( !offsetTrack )
			adjust = dist1 < 0 ? -1.0 : 1.0;
		int		len = min(150, slices.GetSize() / 2 - 2);
		pOptPath->ModifySectionAfter( prev_p, len, adjust );
		pOptPath->CopyToPath( pCarPaths->GetPath(s.my_ID) );
		pCarPaths->GetPath(s.my_ID).CalcSpeedSection( cur_p, len );
		count++;
		goto calcTargets;
	}

	offsetTrack = false;

	//
	// record the paths of the other cars

	for( int i = 0; i < args.m_iNumCar; i++ )
		if( i != s.my_ID )
			pCarPaths->Record( i, race_data.cars[i] );

	if( s.lap_flag && s.laps_done >= 5 &&
		pCarPaths->GetBestCar() != s.my_ID )//&&
//		pCarPaths->GetBestCar() != bestCar )
	{
		D6Path	bestPath = pCarPaths->GetBestPath();
		bestPath.CalcSpeed();
		double	bestSpeedEstimate = bestPath.EstimateSpeed(s);

		if( speedEstimate + 0.1 < bestSpeedEstimate )
		{
			bestCar = pCarPaths->GetBestCar();
			speedEstimate = bestSpeedEstimate;
			pCarPaths->SetBestPath();
			offsetTrack = true;
		}
	}

	//
	// steering servo and speed calc

	double	alpha = 0;
	double	vc = 300;

	D6Vec2	pt2 = carPt + carDir * s.v * delta_time;
	double	dist2, k2;
	D6Vec2	vel2;
	path.Calc( cur_p, pt2, dist2, vel2, k2 );

//	double	spd1 = vel1.GetLength();
	double	spd2 = vel2.GetLength();

	double	k = (k1 + k2) * 0.5;

	double	latA = s.v * s.v * k;
	double	fric = fabs(latA / g);
	double	slip = invFriction(fric);
	double	ratio = slip / (2 * s.v);
	alpha = 2 * asin(ratio);
	if( latA < 0 )
		alpha = -alpha;

	D6Vec2	dir = path.GetAt(next_p).m_p - path.GetAt(cur_p).m_p;
	double	servoAngle = NormaliseAngleLimit(dir.GetAngle() - carAngle, 0);
	static double oldDist = 0;
	double	pid = -0.05 * dist1 + -0.0133 * (dist1 - oldDist);
	oldDist = dist1;
	servoAngle += atan(pid);

	vc = spd2;
	alpha += servoAngle;

	//
	// see if we are going to hit another car

	D6Situation	mySituation;
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
	if( s.stage != QUALIFYING )
	{
		colliding = AvoidOtherCars( s, mySituation, *pCarPaths,
									vc, passAdjust, needToBrake);
		if( passAdjust != 0 )
		{
			// work out how much to modify
//			int		len = min(150, slices.GetSize() / 2 - 2);
			int		len = min(200, slices.GetSize() / 2 - 2);

			// transfer current path to optimiser
			pOptPath->SetFromPath( pCarPaths->GetPath(s.my_ID) );

			// don't try to ajust track if already turning that way...
			if( dist1 * passAdjust < 0 || fabs(dist1 + passAdjust) < 1 )
			{
				// modify the section
//				pOptPath->ModifySectionAfter( prev_p, len, passAdjust );
//				pOptPath->ModifySectionCircle( prev_p, len, passAdjust );
				pOptPath->ModifySection( prev_p, len, passAdjust,
					(prev_p + 20) % slices.GetSize() );

				// figure the new speeds
				D6Path	tempPath(pCarPaths->GetPath(s.my_ID));
				pOptPath->CopyToPath( tempPath );
				tempPath.CalcSpeedSection( cur_p, len );

				// if speed is ok, use new path
				if( tempPath.GetAt(cur_p).m_spd >= s.v - 1 )
					pCarPaths->GetPath(s.my_ID) = tempPath;
			}
		}
	}

	if( s.to_lft < 0.2 || colliding && s.to_lft < 2.0 )
	{
		if( s.vn > 0 )
			alpha -= colliding ? (2.0 - s.to_lft) * 0.05 : 0.05;
	}
	else if( s.to_rgt < 0.2 || colliding && s.to_rgt < 2.0 )
	{
		if( s.vn < 0 )
			alpha += colliding ? (2.0 - s.to_rgt) * 0.05 : 0.05;
	}

	if( alpha > 0.4 )
		alpha = 0.4;
	else if( alpha < -0.4 )
		alpha = -0.4;

//	double	zeroLatVDist = 1.5 * s.vn * s.vn * s.vn /
//								(MAX_ACCEL * MAX_ACCEL);
//	if( s.to_lft - zeroLatVDist < 0.5 ||
//		s.to_rgt + zeroLatVDist < 0.5 )
//		alpha *= 2;

	if( needToBrake || vc < s.v * 0.95 )
	{
		// work out the amount of grip required to take us in a circular
		//	path at the current velocity + 2%
		double	minMyuN = 1.02 * s.v * alpha / g;

		// now search for a braking coefficient (bc) that allows
		//	at least this much lateral grip, but with a minimum bc of 0.95
		double	bc = 0.8;//BRAKE_COEF;
		vc = s.v * bc;
		D6Vec2	alphaVec = D6Vec2::FromAngle(alpha);
		D6Vec2	slipVec = D6Vec2(s.v, 0) - alphaVec * vc;
		double	slip = slipVec.GetLength();
		double	myu = friction(slip);
		double	myuN = fabs(slipVec.GetUnit().y * myu);
		while( bc < 0.95 && myuN < minMyuN )
		{
			bc += 0.01;
			vc = s.v * bc;
			slipVec = D6Vec2(s.v, 0) - alphaVec * vc;
			slip = slipVec.GetLength();
			myu = friction(slip);
			myuN = (-slipVec.GetUnit() * myu).y;
		}
	}
	else if( vc > s.v + 1 )
	{
//		double	maxA = g * CORN_MYU;
//		double	latA = s.v * tan(alpha) / delta_time;
//		double	inner = maxA * maxA - latA * latA;
//		double	maxTanA = inner <= 25 ? 5 : sqrt(inner);
//		double	maxVc = s.v + maxTanA * delta_time;

		vc = 300;//s.v + 10;
	}

	// return our precious :) control values
	result.vc = vc;
	result.alpha = alpha;

	//printf("drive end 2\n");

	return result; 
} 

Driver * getDodger6Instance()
{
  return new Dodger6();
}
