/*
Author:				Tim Foden (Newquay, Cornwall, UK)
E-mail:				tim@7sun.com
Robot name:			"Dodger1"
Robot version:		1
Driver function:	Dodger1()
Filename:			Dodger1.cpp
Races:				All
Data file:			None
Robot status:		Public
Race colours:		Black/White
Release date:		27-August-2000
Tab size used:		4
*/ 

#include <stdio.h>
#include <string.h> 
#include <stdlib.h> 
#include <math.h> 

#include "car.h" 
#include "track.h"
#include "misc.h"

// get hold of the external surface variable
extern int surface;

#define	ASSERT

namespace Dgr1
{
const double cGravityAccel = 32.2;

const double cCarWid = CARWID * 1.75;	// make out cars are wider than they really are
const double cCarDiag = hypot(CARWID, CARLEN);

// pit constants from Car::move_car() in carz.cpp
const double cPitSecsPerDamage = 0.005;
const double cPitSecsPerFuel = 0.05;

static double CORN_MYU = 1.05;		// lateral g's expected when cornering

const double BRAKE_ACCEL = 35.5;	// accel available while braking (straight)
const double BRAKE_COEF = 0.8;
//const double BRAKE_ACCEL = 27.0;	// accel available while braking (straight)
//const double BRAKE_COEF = 0.95;

const double BRAKE_ACCEL_BEND = 27;	// accel available while braking (bend)
const double BRAKE_COEF_BEND = 0.95;
//const double BRAKE_ACCEL_BEND = 35.5;	// accel available while braking (bend)
//const double BRAKE_COEF_BEND = 0.8;

const double MAX_ACCEL = 32.2;

const double cPi = 3.1415926535897932384626433832795;
const double cPi_2 = cPi / 2;
const double c3Pi_2 = 3 * cPi / 2;
const double c2Pi = 2 * cPi;

const double cCos1Deg = 0.99984769515639123915701155881391;
const double cCos3Deg = 0.99862953475457387378449205843944;
const double cCos5Deg = 0.99619469809174553229501040247389;
const double cCos7Deg = 0.99254615164132203498006158933058;
const double cCos9Deg = 0.98768834059513772619004024769344;
const double cCos30Deg = 0.86602540378443864676372317075294;
const double cCos40Deg = 0.76604444311897803520239265055542;
const double cCos45Deg = 0.70710678118654752440084436210485;

#ifndef min
#define	min(x, y)	((x) < (y) ? (x) : (y))
#endif

#ifndef max
#define	max(x, y)	((x) > (y) ? (x) : (y))
#endif

double	NormaliseAngle( double angle )
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

double	NormaliseAngleLimit( double angle, double sizeAngle )
{
	angle = NormaliseAngle(angle);

	// allow half of angle outside range to be -ve
	double	halfRest = PI - sizeAngle / 2;
	if( angle > 2 * PI - halfRest )
		angle -= 2 * PI;

	return angle;
}

bool	AngleInRange( double angle, double startAngle, double sizeAngle )
{
	if( sizeAngle > 0 )
		return NormaliseAngle(angle - startAngle) < sizeAngle;
	else
		return NormaliseAngle(angle - startAngle) < -sizeAngle;
}

double	AngularPortion( double angle, double startAngle, double sizeAngle )
{
	// get the difference in the angles
	double	deltaAngle = NormaliseAngleLimit(angle - startAngle, sizeAngle);

	// work out the ratio
	return deltaAngle / sizeAngle;
}

struct	Vec2
{
public:
	Vec2();
	Vec2( double x, double y );
	Vec2( const Vec2& vec2 );

	Vec2	operator+( const Vec2& v ) const;
	Vec2	operator-( const Vec2& v ) const;
	Vec2	operator-() const;
	
	Vec2	operator*( double n ) const;		// scalar product
	Vec2	operator/( double n ) const;		// scalar divide

	double	operator*( const Vec2& v ) const;	// dot product
	double	operator%( const Vec2& v ) const;	// cross product

	double	GetLength() const;
	double	GetAngle() const;
	Vec2	GetUnit() const;
	Vec2	GetNormal() const;

	static Vec2		FromAngle( double angle );

public:
	double	x;
	double	y;
};

Vec2::Vec2()
:	x(0),
	y(0)
{
}

Vec2::Vec2( double X, double Y )
:	x(X),
	y(Y)
{
}

Vec2::Vec2( const Vec2& v )
:	x(v.x),
	y(v.y)
{
}

Vec2	Vec2::operator+( const Vec2& v ) const
{
	return Vec2(x + v.x, y + v.y);
}

Vec2	Vec2::operator-( const Vec2& v ) const
{
	return Vec2(x - v.x, y - v.y);
}

Vec2	Vec2::operator-() const
{
	return Vec2(-x, -y);
}

Vec2	Vec2::operator*( double n ) const
{
	return Vec2(x * n, y * n);
}

Vec2	Vec2::operator/( double n ) const
{
	return Vec2(x / n, y / n);
}

double	Vec2::operator*( const Vec2& v ) const
{
	return x * v.x + y * v.y;
}

double	Vec2::operator%( const Vec2& v ) const
{
	return x * v.y - y * v.x;
}

double	Vec2::GetLength() const
{
	return hypot(x, y);
}

double	Vec2::GetAngle() const
{
	return atan2(y, x);
}

Vec2	Vec2::GetUnit() const
{
	if( x == 0 && y == 0 )
		return Vec2();

	double	len = hypot(x, y);
	return Vec2(x / len, y / len);
}

Vec2	Vec2::GetNormal() const
{
	return Vec2(-y, x);
}

Vec2	Vec2::FromAngle( double angle )
{
	return Vec2(cos(angle), sin(angle));
}

class CTrack  
{
public:
	struct Segment
	{
		double	m_rad;
		double	m_len;

		Vec2	m_beg;
		Vec2	m_end;
		Vec2	m_cen;
	};

public:
	CTrack();
	virtual ~CTrack();

//	bool	Load( const CString& name );
	void	Initialise( const track_desc& track );
	void	Empty();

	int		NSeg() const;
	double	GetWidth() const;
	double	GetMaxX() const;
	void	SetMaxX( double maxX );
	double	GetMaxY() const;
	void	SetMaxY( double maxY );
	double	GetStartX() const;
	double	GetStartY() const;
	double	GetStartAng() const;
	Segment	GetSegment( int seg ) const;

private:
	int		m_nSeg;
 
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

CTrack::CTrack()
:	m_nSeg(0),
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

CTrack::~CTrack()
{
	delete [] m_pSeg;
}

/*
bool	CTrack::Load( const CString& name )
{
	CStdioFile	file;
	if( !file.Open(name, CFile::modeRead) )
		return false;

	CString	line;
	double	maxAngle = 0;
	int		nLines = 0;
	bool	ok = true;
	while( ok && file.ReadString(line) )
	{
		line.TrimRight();
		if( line.IsEmpty() )
			continue;

		switch( nLines )
		{
			case 0:
				ok = sscanf(line, "%d", &m_nSeg) == 1;
				if( ok )
					m_pSeg = new Segment[m_nSeg];
				break;

			case 1:
				ok = sscanf(line, "%lf %lf", &m_xMax, &m_yMax) == 2;
				break;

			case 2:
				ok = sscanf(line, "%lf", &m_width) == 1;
				break;

			case 3:
				ok = sscanf(line, "%lf %lf %lf", &m_startX, &m_startY,
							&m_startAng) == 3;
				break;

			case 4:
				ok = sscanf(line, "%lf %lf", &m_scoreX, &m_scoreY) == 2;
				break;
				
			case 5:
				ok = sscanf(line, "%lf %lf", &m_ldrX, &m_ldrY) == 2;
				break;

			case 6:
				ok = sscanf(line, "%lf %lf", &m_instX, &m_instY) == 2;
				break;

			case 7:
				ok = sscanf(line, "%lf %lf", &m_lenX, &m_lenY) == 2;
				break;

			case 8:
				ok = sscanf(line, "%lf %d", &m_startLine, &m_startRows) == 2;
				break;

			case 9:
				ok = sscanf(line, "%d", &m_pitSide ) == 1;
				break;
 
			case 10:
				ok = sscanf(line, "%lf %lf", &m_pitEntry, &m_pitLaneStart) == 2;
				break;

			case 11:
				ok = sscanf(line, "%lf %lf", &m_pitLaneEnd, &m_pitExit) == 2;
				break;

			case 12:
				ok = sscanf(line, "%lf", &m_pitLaneSpeed) == 1;
				break;

			default:
			{
				// read track segment data
				int		seg = nLines - 13;
				if( seg >= m_nSeg )
					break;
				ok = sscanf(line, "%lf %lf", &m_pSeg[seg].m_rad,
							&m_pSeg[seg].m_len) == 2;

				if( m_pSeg[seg].m_rad != 0 && m_pSeg[seg].m_len > maxAngle )
					maxAngle = m_pSeg[seg].m_len;
			}
		}

		nLines++;
	}

	if( maxAngle > 20 )
	{
		// angles have been given in degrees... convert them
		//	all to radians
		for( int i = 0; i < m_nSeg; i++ )
		{
			if( m_pSeg[i].m_rad != 0 )
				m_pSeg[i].m_len *= PI / 180.0;
		}
	}

	//
	// now calculate the coordinates for all segments.
	//

	Vec2	pt(m_startX, m_startY);
	double	ang(m_startAng);

	for( int i = 0; i < m_nSeg; i++ )
	{
		Segment&	seg = m_pSeg[i];

		if( seg.m_rad == 0 )
		{
			seg.m_beg = pt;
			seg.m_end = pt + Vec2::FromAngle(ang) * seg.m_len;
		}
		else if( seg.m_rad > 0 )
		{
			Vec2	cen(Vec2::FromAngle(ang + PI_2) * seg.m_rad);
			Vec2	end(Vec2::FromAngle(ang - PI_2 + seg.m_len) * seg.m_rad);

			seg.m_beg = pt;
			seg.m_end = pt + cen + end;
			seg.m_cen = pt + cen;

			ang += seg.m_len;
		}
		else
		{
			Vec2	cen(Vec2::FromAngle(ang - PI_2) * -seg.m_rad);
			Vec2	end(Vec2::FromAngle(ang + PI_2 - seg.m_len) * -seg.m_rad);

			seg.m_beg = pt;
			seg.m_end = pt + cen + end;
			seg.m_cen = pt + cen;

			ang -= seg.m_len;
		}

		pt = seg.m_end;
	}

	return ok;
}
*/
void	CTrack::Initialise( const track_desc& track )
{

	m_nSeg = track.NSEG;
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
		m_pSeg[i].m_beg = Vec2(seg.beg_x, seg.beg_y);
		m_pSeg[i].m_end = Vec2(seg.end_x, seg.end_y);
		m_pSeg[i].m_cen = Vec2(seg.cen_x, seg.cen_y);
	}
}

void	CTrack::Empty()
{
	m_nSeg = 0;
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

int		CTrack::NSeg() const
{
	return m_nSeg;
}

double	CTrack::GetWidth() const
{
	return m_width;
}

double	CTrack::GetMaxX() const
{
	return m_xMax;
}

void	CTrack::SetMaxX( double maxX )
{
	m_xMax = maxX;
}

double	CTrack::GetMaxY() const
{
	return m_yMax;
}

void	CTrack::SetMaxY( double maxY )
{
	m_yMax = maxY;
}

double	CTrack::GetStartX() const
{
	return m_startX;
}

double	CTrack::GetStartY() const
{
	return m_startY;
}

double	CTrack::GetStartAng() const
{
	return m_startAng;
}

CTrack::Segment	CTrack::GetSegment( int seg ) const
{
	ASSERT( seg >= 0 && seg < m_nSeg );
	return m_pSeg[seg];
}

double	CorneringSpeed( double radius )
{ 
	double	centreRadius = fabs(radius);
	return sqrt(centreRadius * cGravityAccel * CORN_MYU);
} 

double	TurningRadius( double speed )
{ 
	return speed * speed / (cGravityAccel * CORN_MYU);
} 

double	BrakingDistance( double fromVel, double toVel )
{
	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL;
}

double	BrakingDistanceBend( double fromVel, double toVel )
{
	double	dv = toVel - fromVel;
	return (fromVel + 0.5 * dv) * dv / -BRAKE_ACCEL_BEND;
}

double	VelocityAfterDistance( double u, double s, double a )
{
	return sqrt(2 * a * s + u * u);
}

double	CalcDistanceToLine( const Vec2& lp, const Vec2& lv, const Vec2& p )
{
	return lv % (p - lp) / lv.GetLength();
}

bool	LineCrossesLine(
	const Vec2&	lp0,
	const Vec2& lv0,
	const Vec2&	lp1,
	const Vec2& lv1,
	double&		t )
{
	double	denom = lv0 % lv1;
	if( denom == 0 )
		return false;

	double	numer = lv1 % (lp0 - lp1);

	t = numer / denom;

	return true;
}

Vec2	LineCrossesLinePt(
	const Vec2&	lp0,
	const Vec2& lv0,
	const Vec2&	lp1,
	const Vec2& lv1 )
{
	double	t;
	if( !LineCrossesLine(lp0, lv0, lp1, lv1, t) )
		return lp0;

	return lp0 + lv0 * t;
}

bool	LineCrossesCircle(
	const Vec2& lp,		// start point of line
	const Vec2& lv,		// vector for line
	const Vec2& cp,		// centre point of circle
	double		cr,		// radius of circle
	double&		t1,		// returned solution, smallest
	double&		t2 )	// returned solution, largest
{
	Vec2	dp = lp - cp;

	double	a = lv * lv;
	if( a == 0 )
		return false;

	double	b = 2 * (lv * dp);
	double	c = dp * dp - cr * cr;

	double	inner = b * b - 4 * a * c;

	if( inner < 0 )
		return false;

	inner = sqrt(inner);

	t1 = (-b - inner) / (2 * a);
	t2 = (-b + inner) / (2 * a);

	return true;
}

bool	CalcCircle(
	const Vec2&	p1,
	const Vec2&	p2,
	const Vec2&	p3,

	Vec2&		cen )
{
	Vec2	mid1  = (p1 + p2) / 2;
	Vec2	norm1 = (p2 - p1).GetNormal();
	Vec2	mid2  = (p2 + p3) / 2;
	Vec2	norm2 = (p3 - p2).GetNormal();

	double	t;
	if( !LineCrossesLine(mid1, norm1, mid2, norm2, t) )
		return false;

	cen = mid1 + norm1 * t;
	return true;
}

bool	CircleCrossesCircle(
	const Vec2&	c1,
	double		r1,
	const Vec2&	c2,
	double		r2,
	double&		angle1,		// crossing angles for 1st circle
	double&		angle2 )
{
	double	d = (c1 - c2).GetLength();

	// circle too far away to cross?
	if( r1 + r2 < d )
		return false;

	// 1st circle inside 2nd?
	if( d + r1 < r2 )
		return false;

	// 2nd circle inside 1st?
	if( d + r2 < r1 )
		return false;

	// ok, calc crossing angles now (for 1st circle) (cosine rule)
	double	alpha = acos((r1 * r1 + d * d - r2 * r2) / (2 * r1 * d));
	double	theta = (c2 - c1).GetAngle();
	angle1 = theta + alpha;
	angle2 = theta - alpha;
	return true;
}

// returns false if pt inside circle
bool	CalcTangentPt(
	const Vec2& pt,		// point to calc tangent to
	const Vec2& cen,	// centre of circle
	double		rad,	// radius, -ve if clockwise
	Vec2&		tanPt )	// point calculated
{
	Vec2	v = cen - pt;
	double	r = fabs(rad);
	double	h = v.GetLength();
	if( h < r )
		return false;

	double	t = sqrt(h * h - r * r);
	double	alpha = asin(r / h);

	double	beta = rad > 0 ? v.GetAngle() - alpha : v.GetAngle() + alpha;
	Vec2	v2 = Vec2::FromAngle(beta);

	tanPt = pt + v2 * t;

	return true;
}

class CPath  
{
public:
	enum
	{
		cSegFtStrt	= 25,
		cSegFtBend	= 25,
	};

	struct	Line
	{
		Vec2	m_l;	// start point normal of line
		Vec2	m_v;	// direction of normal line
		double	m_w;	// distance along normal line
		Vec2	m_p;	// point on path
		double	m_spd;	// speed to go at this pt

		void Setup( const Vec2& l, const Vec2& v, double w );
	};

public:
	CPath();
	~CPath();

	void		Setup( const CTrack& track );

	int			GetSize() const;
	const Line&	GetAt( int i ) const;

	double		CalcSum() const;
	void		Optimise();

private:
	void		SmoothBetween( int step );
	void		Optimise(	Line* l2, const Line* l0, const Line* l1,
									const Line* l3, const Line* l4 );
	void		Iterate( int step );
	void		CalcSpeed();

private:
	double	m_width;
	int		m_nLines;
	Line*	m_pLines;
};

void CPath::Line::Setup( const Vec2& l, const Vec2& v, double w )
{
	m_l = l;
	m_v = v;
	m_w = w;
	m_p = l + v * w;
}

CPath::CPath()
:	m_width(1),
	m_nLines(0),
	m_pLines(0)
{
}

CPath::~CPath()
{
	delete [] m_pLines;
}

void	CPath::Setup( const CTrack& track )
{
	m_nLines = 0;
	delete [] m_pLines;
	m_pLines = new Line[10000];

	double	width = m_width = track.GetWidth();

	int	lineNr = 0;

	for( int i = 0; i < track.NSeg(); i++ )
	{
		CTrack::Segment	seg = track.GetSegment(i);

		if( seg.m_rad == 0 )
		{
			int		n = int(seg.m_len / cSegFtStrt);

			Vec2	normal = (seg.m_end - seg.m_beg).GetNormal().GetUnit();
			Vec2	d = seg.m_end - seg.m_beg;

			m_pLines[lineNr].Setup( seg.m_beg, normal, width * 0.5 );
			lineNr++;

			for( int j = 1; j < n; j++ )
			{
				m_pLines[lineNr].Setup( seg.m_beg + d * j / n,
										normal, width * 0.5);
				lineNr++;
			}
		}
		else if( seg.m_rad > 0 )
		{
			double	r = seg.m_rad - width * 0.5;
			int		n = int(r * seg.m_len / cSegFtBend);
			double	angle = (seg.m_beg - seg.m_cen).GetAngle();

			for( int j = 0; j < n; j++ )
			{
				Vec2	normal = Vec2::FromAngle(angle + seg.m_len * j / n);

				m_pLines[lineNr].Setup( seg.m_cen + normal * seg.m_rad,
										-normal, width * 0.5 );
				lineNr++;
			}
		}
		else
		{
			double	r = -seg.m_rad + width * 0.5;
			int		n = int(r * seg.m_len / cSegFtBend);
			double	angle = (seg.m_beg - seg.m_cen).GetAngle();

			for( int j = 0; j < n; j++ )
			{
				Vec2	normal = Vec2::FromAngle(angle - seg.m_len * j / n);

				m_pLines[lineNr].Setup( seg.m_cen + normal * -seg.m_rad,
										normal, width * 0.5 );
				lineNr++;
			}
		}
	}

	m_nLines = lineNr;
}

int		CPath::GetSize() const
{
	return m_nLines;
}

const CPath::Line&	CPath::GetAt( int i ) const
{
	return m_pLines[i];
}

double	CPath::CalcSum() const
{
	double	sum = 0;
	for( int i = 0; i < m_nLines; i++ )
	{
		double	n = m_pLines[i].m_w;
		sum += n * n;
	}
	sum /= m_nLines;
//	sum = sqrt(sum);
	return sum;
}

void	CPath::Optimise()
{
	int		step = 8;
	while( step > 0 )
	{
		int		count = 0;
		double	sum1 = CalcSum();
		double	sum2 = 0;
		double	error = 0;;
		do
		{
			Iterate( step );

			++count;
			if( count > (16 >> step) )
				break;

			sum1 = sum2;
			sum2 = CalcSum();

			error = fabs(sum1 - sum2);
		}
		while( error > (1 << (step - 1)) );

		step /= 2;
	}

	CalcSpeed();
}

void	CPath::SmoothBetween( int step )
{
	// now smooth the values between steps
	for( int i = 0; i < m_nLines; i += step )
	{
		int		j = (i + step) % m_nLines;

		Vec2	p1 = m_pLines[i].m_p;
		Vec2	p2 = m_pLines[j].m_p;

		for( int k = 1; k < step; k++ )
		{
			Line&	l = m_pLines[i + k];
			double	t;
			LineCrossesLine(l.m_l, l.m_v, p1, p2 - p1, t);

			if( t < 0 )
				t = 0;
			else if( t > m_width )
				t = m_width;

			l.m_w = t;
			l.m_p = l.m_l + l.m_v * t;
		}
	}
}

void	CPath::Optimise(
	Line*		l2,

	const Line* l0,
	const Line* l1,
	const Line*	l3,
	const Line* l4 )
{

	double	t1, t2;
	Vec2	cen1, cen2;

	if( CalcCircle(l0->m_p, l1->m_p, l3->m_p, cen1) )
	{
		double	rad = (l0->m_p - cen1).GetLength();
		double	a, b;
		LineCrossesCircle(l2->m_p, l2->m_v, cen1, rad, a, b);
		t1 = l2->m_w + (fabs(a) < fabs(b) ? a : b);
	}
	else
	{
		LineCrossesLine(l2->m_l, l2->m_v, l3->m_p, l1->m_p - l4->m_p, t1);
	}

	if( CalcCircle(l1->m_p, l3->m_p, l4->m_p, cen2) )
	{
		double	rad = (l1->m_p - cen2).GetLength();
		double	a, b;
		LineCrossesCircle(l2->m_p, l2->m_v, cen2, rad, a, b);
		t2 = l2->m_w + (fabs(a) < fabs(b) ? a : b);
	}
	else
	{
		LineCrossesLine(l2->m_l, l2->m_v, l1->m_p, l3->m_p - l0->m_p, t2);
	}

	double	t = (t1 + t2) / 2;
	if( (l1->m_p - l0->m_p).GetNormal() * (l4->m_p - l3->m_p) > 0 )
	{
		if( t < 3 )
			t = 3;
		else if( t > m_width - 3 )
			t = m_width - 3;
	}
	else
	{
		if( t < 3 )
			t = 3;
		else if( t > m_width - 3 )
			t = m_width - 3;
	}

	l2->m_w = t;
	l2->m_p = l2->m_l + l2->m_v * t;
}

void	CPath::Iterate( int step )
{
	for( int j = 0; j < 200; j++ )
	{
		Line*	l0 = 0;
		Line*	l1 = &m_pLines[m_nLines - 4 * step];
		Line*	l2 = &m_pLines[m_nLines - 3 * step];
		Line*	l3 = &m_pLines[m_nLines - 2 * step];
		Line*	l4 = &m_pLines[m_nLines - 1 * step];

		// go forwards
		for( int i = 0; i < m_nLines; i += step )
		{
			l0 = l1;
			l1 = l2;
			l2 = l3;
			l3 = l4;
			l4 = &m_pLines[i];

			Optimise( l2, l0, l1, l3, l4 );
		}
/*
		// now go backwards
		for( i -= step * 2; i >= 0; i -= step )
		{
			l4 = l3;
			l3 = l2;
			l2 = l1;
			l1 = l0;
			l0 = &m_pLines[i];

			Optimise( l2, l0, l1, l3, l4 );
		}
*/	}

	// now smooth the values between steps
	if( step > 1 )
		SmoothBetween( step );
}

void	CPath::CalcSpeed()
{
	Line*	l0 = 0;
	Line*	l1 = &m_pLines[m_nLines - 2];
	Line*	l2 = &m_pLines[m_nLines - 1];

	for( int i = 0; i < m_nLines; i++ )
	{
		l0 = l1;
		l1 = l2;
		l2 = &m_pLines[i];

		Vec2	v1 = l1->m_p - l0->m_p;
		Vec2	v2 = l2->m_p - l1->m_p;

		double	cosAng = (v1 * v2) / (v1.GetLength() * v2.GetLength());

		Vec2	cen;
		CalcCircle(l0->m_p, l1->m_p, l2->m_p, cen);
		double	radius = (l0->m_p - cen).GetLength();
		double	rad2 = (v1 + v2).GetLength() / (2 * acos(cosAng));

		l1->m_spd = min(300, CorneringSpeed(rad2));
	}
/*
	for( int j = 0; j < 10; j++ )
	{
		// run a filter over the speeds, to iron out jitter
		l2 = 0;
		Line*	l3 = &m_pLines[m_nLines - 2];
		Line*	l4 = &m_pLines[m_nLines - 1];

		double	s0 = 0;
		double	s1 = m_pLines[m_nLines - 4].m_spd;
		double	s2 = m_pLines[m_nLines - 3].m_spd;
		double	s3 = l3->m_spd;
		double	s4 = l4->m_spd;

		for( i = 0; i < m_nLines; i++ )
		{
			l2 = l3;
			l3 = l4;
			l4 = &m_pLines[i];

			s0 = s1;
			s1 = s2;
			s2 = s3;
			s3 = s4;
			s4 = l4->m_spd;

			double	spd = (s0 + s1 * 3 + s2 * 18 + s3 * 3 + s4) / 26;
			l2->m_spd = spd;
		}
	}
*/
	const double	max_a = MAX_ACCEL * CORN_MYU;

	for( int j = 0; j < 2; j++ )
	{
		l0 = &m_pLines[0];
		l1 = &m_pLines[1];
		for( int i = m_nLines - 1; i >= 0; i-- )
		{
			l2 = l1;
			l1 = l0;
			l0 = &m_pLines[i];

			if( l0->m_spd > l1->m_spd + 0.01 )
			{
				double	a0 = (l1->m_p - l0->m_p).GetAngle();
				double	a1 = (l2->m_p - l1->m_p).GetAngle();

				double	alpha = NormaliseAngleLimit(a1 - a0, 0);
				double	s = (l1->m_p - l0->m_p).GetLength();

				double	u = l0->m_spd;
				double	v = l1->m_spd;

				while( true )
				{
					double	spd = (u + v) / 2;
					double	cen_a = alpha * spd * spd / s;

					double	inner = max_a * max_a - cen_a * cen_a;
					double	tan_a = inner <= 0 ? 0 : -sqrt(inner);

					double	drag_a = DRAG_CON * spd * spd / (M * g);
					tan_a -= drag_a;

					inner = v * v - 2 * tan_a * s;
					double	max_u = inner <= 0 ? 0 : sqrt(inner);

					if( u <= max_u )
						break;

					u = max_u;
				}

				double	brakeDist = BrakingDistance(u, v);
				ASSERT( brakeDist <= s );

				if( u < l0->m_spd )
					l0->m_spd = u;
			}
		}
	}
}

void	PitControl( const situation& s, con_vec& control, int& damage )
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
	bool	repairDamage =	s.laps_to_go > 1 &&
							(s.damage >= 25000 ||
							 s.damage >= 20000 && predDamage >= 25000 ||
							 s.damage >= 15000 && predDamage >= 30000);
	if( s.stage != QUALIFYING &&
		(repairDamage || s.fuel < fuelPerLap * 1.1 ) )
	{
		// work out the amount of fuel we want
		double fuelToEnd = min(MAX_FUEL, fuelPerLap * s.laps_to_go + 1);
		control.fuel_amount = max(0, fuelToEnd - s.fuel);

		// we can repair some damage for "free" while the fuel is
		//	going in...
		double freeDamageRepair = (control.fuel_amount - s.fuel) *
									cPitSecsPerFuel / cPitSecsPerDamage;

		control.request_pit = true;
		int		repair = 0;
		if( repairDamage )
		{
			// if we are close to the end of the race, then only
			//	repair the amount needed to get us to the end with
			//	about 15000 damage
			repair = int(predDamage) - 15000;
		}
		else
		{
			// we have just stopped in to re-fuel... but we will still
			//	repair some damage, related to how far we have yet
			//	to travel.
			repair = max((s.laps_to_go - 5) * 300, (int)(freeDamageRepair));
		}
		control.repair_amount = min(int(s.damage), repair);

	}
	else
		control.request_pit = false;
//	control.request_pit = true;
}

/////////////////////////////////////////////////////////////////////////////

struct	Situation
{
	Vec2	m_carPt;	// global car coordinates
	Vec2	m_carDir;	// global car unit direction vector
	double	m_carAngle;	// global car angle
	double	m_tanDist;	// distance till tangent point

	Vec2	m_distDir;	// direction to most distant point down track
	bool	m_nextTurnIsLeft;	// true if will be turning left
};

/////////////////////////////////////////////////////////////////////////////

bool	CheckOtherIn( const Vec2& p, double sine, double cosine )
{
	Vec2	l = Vec2(cosine, sine) * (CARLEN / 2);
	Vec2	w = Vec2(sine, -cosine) * (CARWID / 2);

	Vec2	rl = p - l - w;
	if( rl.x > -CARWID / 2 && rl.x < CARWID / 2 &&
		rl.y > -CARLEN / 2 && rl.y < CARLEN / 2 )
		return true;

	Vec2	rr = p - l + w;
	if( rr.x > -CARWID / 2 && rr.x < CARWID / 2 &&
		rr.y > -CARLEN / 2 && rr.y < CARLEN / 2 )
		return true;

	Vec2	fl = p + l - w;
	if( fl.x > -CARWID / 2 && fl.x < CARWID / 2 &&
		fl.y > -CARLEN / 2 && fl.y < CARLEN / 2 )
		return true;

	Vec2	fr = p + l + w;
	if( fr.x > -CARWID / 2 && fr.x < CARWID / 2 &&
		fr.y > -CARLEN / 2 && fr.y < CARLEN / 2 )
		return true;

	return false;
}

/////////////////////////////////////////////////////////////////////////////

bool	CheckCollide( const Vec2& p, double angle )
{
	if( p * p < CARWID * CARWID )
		return true;

	if( p * p >= cCarDiag * cCarDiag )
		return false;

	double	s = sin(angle + cPi_2);
	double	c = cos(angle + cPi_2);

	if( CheckOtherIn(p, s, c) )
		return true;

	Vec2	p2(p.y * c - p.x * s, -p.x * c - p.y * s);
	return CheckOtherIn(p2, -s, c);
}

/////////////////////////////////////////////////////////////////////////////

void	AvoidOtherCars(
	const situation&	s,				// current situation
	const Situation&	mySituation,	// global situation values

	double&				vc,				// modified speed
	double&				passAng,		// pass angle
	bool&				needToBrake )	// true if need to brake
{
	bool		gotCar = false;
	rel_state	car;
	double		collideTime = 10;
	Vec2		collidePos1;
	Vec2		collideDir1;
	Vec2		collidePos2;

	// work out a predicted circular path for our car
	double	rad = s.alpha == 0 ? 1000000000 : s.v / s.alpha;
	double	alpha = s.alpha == 0 ? s.v / rad : s.alpha;
	Vec2	cen = Vec2(-rad, 0);

	// see if we are going to hit another car
	passAng = 0;
	for( int i = 0; i < NEARBY_CARS; i++ )
	{
		if( s.nearby[i].who >= MAXCARS )
			continue;

		double	x = s.nearby[i].rel_x;
		double	y = s.nearby[i].rel_y;
		double	vx = s.nearby[i].rel_xdot;
		double	vy = s.nearby[i].rel_ydot;
		double	v = s.nearby[i].v;
		double	al = s.nearby[i].alpha;

		double	rv = hypot(vx, vy);
		if( vy < 0 )
			rv = -rv;

		double	cv = x * vx > 0 ? vy : rv;

		// don't catch up to any cars going more than 30fps faster
		//	by the time we overtake
		double	brakeDist = BrakingDistanceBend(s.v, s.nearby[i].v + 40);
		if( y > CARLEN && brakeDist > y )
		{
			needToBrake = true;
		}

		// don't pass other cars going more than 15fps faster than them.
		//	only include cars within ±30° of current direction of travel.
		brakeDist = BrakingDistanceBend(s.v, s.v + cv + 15);
//		brakeDist = BrakingDistanceBend(s.v, s.v + cv );//+ 15);
		if( y > CARLEN && y < brakeDist && fabs(x) <= y / 2 + CARWID )
		{
			needToBrake = true;
		}

		// work out the velocity vector of the other car
		Vec2	vDir(vx, s.v + vy);
		Vec2	vNorm = vDir.GetNormal().GetUnit();

		// use alpha and velocity to predict a circular path for the car.
		double	r = al == 0 ? rad = 1000000000 : v / al;
		if( al == 0 )
			al = v / r;
		Vec2	c = Vec2(x, y) + vNorm * r;

		// check if we are currently colliding with the other car.
		// work out angle of other car in Sparky's local coord system,
		//	with car pointing along velocity vector.  therefore zero degrees
		//	is along velocity vector, which is the x axis.
		double	relAngle = vDir.GetAngle() - cPi_2;//atan2(-vx, s.v + vy);
		if( collide(x, y, relAngle) )
		{
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
		Vec2	minP1;
		Vec2	minDir;
		Vec2	minP2;
		double	collDist = 0;
		double	deltaT = 0.05;			// 1/20 sec
		for( double time = 0; time < 3; time += deltaT )
		{
			double	a1 = alpha * time;
			Vec2	p1 = cen + Vec2::FromAngle(a1) * rad;

			double	a2 = relAngle + al * time;
			Vec2	p2 = c + Vec2::FromAngle(a2) * r;

			Vec2	p = p2 - p1;
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
				minDir = Vec2::FromAngle(a1 + cPi_2);
				minP2 = p2;
				collDist = fabs(alpha * time * rad);
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
			collideTime = minTime;
			collidePos1 = minP1;
			collideDir1 = minDir;
			collidePos2 = minP2;
		}

		brakeDist = max(0, BrakingDistanceBend(s.v, s.v + vy) - CARLEN);
		if( y > 0 && vy < 0 && brakeDist >= collDist )
			needToBrake = true;
	}

	if( collideTime < 10 )
	{
		Vec2	norm = collideDir1.GetNormal();
		double	dot = (collidePos2 - collidePos1) * norm;

		// work out how "urgent" it is to dodge the other car,
		//	related to how far ahead it is
//		double	urgency = max((10 * CARLEN - car.rel_y) / (9 * CARLEN), 0);
//		urgency = min(urgency, 1);
		double	urgency = min(1, sqrt(max(1 - collideTime, 0)));

		// got car at side ... which side?
		if( urgency > 0 )
		{
			if( dot > 0 )
			{
				// can we get to the right of the car?
				if( car.to_rgt > cCarWid )
				{
					if( s.to_rgt > cCarWid * 1.5 )//|| y < CARLEN )
						passAng = -0.1 * urgency;
				}
				else
				{
					passAng = 0.1 * urgency;
				}
			}
			else
			{
				// can we get to the left of the car?
				if( car.to_lft > cCarWid )
				{
					if( s.to_lft > cCarWid * 1.5 )//|| y < CARLEN )
						passAng = 0.1 * urgency;
				}
				else
				{
					passAng = -0.1 * urgency;
				}
			}

			// figure the angle we would be using for alpha if we didn't
			//	change it with the passAng
			double	actualAlpha = NormaliseAngleLimit(
									mySituation.m_distDir.GetAngle() -
										mySituation.m_carAngle, 0);

			// don't try to pass by turning the other way than
			//	we are already
			if( mySituation.m_tanDist < 100 &&
				(mySituation.m_nextTurnIsLeft  && passAng < 0 ||
				 !mySituation.m_nextTurnIsLeft && passAng > 0) )
				passAng = 0;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////

// end of namespace Dgr1
}

using namespace Dgr1;

con_vec Dodger1( situation& s )
{ 
	const char name[] = "Dodger1";	// This is the robot driver's name! 

	static CTrack	track;
	static CPath	path;
	static int		cur_p = 0;

	con_vec	result;		// control vector
//	double	alpha, vc;	// components of result 
	int		damage = 0;

	if( s.starting )
	{
		// first time only, copy name: 
		my_name_is(name);

		track_desc	trackDesc = get_track_description();

		result.alpha = 300;
		result.vc = s.v; 

		// set up initial values for pit, fuel and damage
		PitControl( s, result, damage );

		// change grip available depending on track surface
		switch( surface )
		{
			default:
			case 0:		CORN_MYU = MYU_MAX0;	break;
			case 1:		CORN_MYU = MYU_MAX1;	break;
			case 2:		CORN_MYU = MYU_MAX2;	break;
		}
//		CORN_MYU *= 1.01;
//		CORN_MYU *= 0.8;

		if( s.stage == BEFORE || s.stage == FINISH )
		{
			// clean up
			track.Empty();
		}
		else if( track.NSeg() == 0 && trackDesc.NSEG != 0 )
		{
			track.Initialise( trackDesc );
			path.Setup( track );
			path.Optimise();
		}

		cur_p = 0;

		return result; 
	} 

	// figure whether we need to refuel or repair damage (or both)
	PitControl( s, result, damage );

	// ask to see cars that are to the side, but slightly behind us
	s.side_vision = true;

	// work out current position of car, global coords
	CTrack::Segment	seg = track.GetSegment(s.seg_ID);
	Vec2		carPt;
	Vec2		trackDir;
	if( s.cur_rad == 0 )
	{
		Vec2	b(seg.m_beg);
		Vec2	e(seg.m_end);
		Vec2	v = (e - b).GetUnit();
		Vec2	n = v.GetNormal();
		carPt = e - v * s.to_end + n * s.to_rgt;
		trackDir = v;
	}
	else
	{
		Vec2	c(seg.m_cen);
		Vec2	e(seg.m_end);
		double	endAng = (e - c).GetAngle();
		double	carAng = seg.m_rad > 0 ? endAng - s.to_end : endAng + s.to_end;
		Vec2	v(Vec2::FromAngle(carAng));
		double	r = seg.m_rad;
		if( r > 0 )
			r = r - s.to_rgt;
		else
			r = (-r) + s.to_rgt;
		carPt = c + v * r;
		trackDir = seg.m_rad > 0 ? v.GetNormal() : -v.GetNormal();
	}

	// work out current direction of car, global angle
	double	carAngle = asin(s.vn / s.v);
	if( s.cur_rad == 0 )
	{
		Vec2	b(seg.m_beg);
		Vec2	e(seg.m_end);
		carAngle += (e - b).GetAngle();
	}
	else if( s.cur_rad > 0 )
	{
		Vec2	c(seg.m_cen);
		carAngle += (carPt - c).GetAngle() + cPi_2;
	}
	else
	{
		Vec2	c(seg.m_cen);
		carAngle += (carPt - c).GetAngle() - cPi_2;
	}

	carAngle = NormaliseAngle(carAngle);
	Vec2	carDir = Vec2::FromAngle(carAngle);

	//

	int		next_p = (cur_p + 1) % path.GetSize();
	while( path.GetAt(next_p).m_v.GetNormal() *
			(carPt - path.GetAt(next_p).m_p) < 0 )
	{
		cur_p = next_p;
		next_p = (next_p + 1) % path.GetSize();
	}

	int		aft_p = (next_p + 1) % path.GetSize();

	Vec2	p0 = path.GetAt(cur_p).m_p;
	Vec2	p1 = path.GetAt(next_p).m_p;
	Vec2	p2 = path.GetAt(aft_p).m_p;

	Vec2	v0 = (p1 - p0).GetUnit();
	Vec2	v1 = (p2 - p1).GetUnit();

	double	l = (p1 - p0).GetLength();
	double	t = v0 * (carPt - p0) / l;

	Vec2	dir = v0 * (1 - t) + v1 * t;
	Vec2	norm = dir.GetNormal().GetUnit();

	double	spd0 = path.GetAt(cur_p).m_spd;
	double	spd1 = path.GetAt(next_p).m_spd;

	double	vc = spd0 * (1 - t) + spd1 * t; 

	// see if we are going to hit another car
	Situation	mySituation;
	mySituation.m_carPt = carPt;
	mySituation.m_carDir = carDir;
	mySituation.m_carAngle = carAngle;
	mySituation.m_distDir = dir;
	mySituation.m_tanDist = 0;
	mySituation.m_nextTurnIsLeft = s.cur_rad > 0;
	if( s.cur_rad == 0 )
		mySituation.m_nextTurnIsLeft = s.nex_rad > 0;
	double	passAng = 0;
	bool	needToBrake = false;
	AvoidOtherCars( s, mySituation, vc, passAng, needToBrake );

	double	dist = norm * (carPt - p0);

	double	alpha = dir.GetAngle() - carAngle;
	alpha = NormaliseAngleLimit(alpha, 0);

	// now work out the angular rate of turn required to keep us on
	//	the correct path
/*	double	angRate = v1.GetAngle() - v0.GetAngle();
	angRate = NormaliseAngleLimit(angRate, 0);
	// we need to make this a rate (rad/sec), so we need to work out
	//	how long we have to make this angle of turn
	double	angTime = l / s.v;
	alpha += angRate / angTime;
*/
	double	servo = -dist / 40;
	if( servo > 0.8 )
		servo = 0.8;
	else if( servo < -0.8 )
		servo = -0.8;
	double	servoAngle = atan(servo);
	alpha += servoAngle;

	alpha += passAng;

	if( needToBrake || vc < s.v * 0.95 )
	{
//		if( !takeCare && fabs(alpha) > 0.15 )	// 0.115 deg
//			vc = s.v * BRAKE_COEF_BEND;
//		else
//			vc = s.v * 0.8;

		// work out the amount of grip required to take us in a circular
		//	path at the current velocity + 2%
		double	minMyuN = 1.02 * s.v * alpha / cGravityAccel;

		// now search for a braking coefficient (bc) that allows
		//	at least this much lateral grip, but with a minimum bc of 0.95
		double	bc = 0.8;//BRAKE_COEF;
		vc = s.v * bc;
		Vec2	alphaVec = Vec2::FromAngle(alpha);
		Vec2	slipVec = Vec2(s.v, 0) - alphaVec * vc;
		double	slip = slipVec.GetLength();
		double	myu = friction(slip);
		double	myuN = fabs(slipVec.GetUnit().y * myu);
		while( bc < 0.95 && myuN < minMyuN )
		{
			bc += 0.01;
			vc = s.v * bc;
			slipVec = Vec2(s.v, 0) - alphaVec * vc;
			slip = slipVec.GetLength();
			myu = friction(slip);
			myuN = (-slipVec.GetUnit() * myu).y;
		}
	}
	else if( vc > s.v + 1 )
		vc = 300;

	// try to make sure car stays on track
	result.vc = vc;
	result.alpha = alpha;

	return result; 
} 
