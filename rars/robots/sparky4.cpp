/*
Author:				Tim Foden (Newquay, Cornwall, UK)
E-mail:				tim@7sun.com
Robot name:			"Sparky4"
Robot version:		4
Driver function:	Sparky4()
Filename:			Sparky4.cpp
Races:				All
Data file:			None
Robot status:		Public
Race colours:		Red/Black
Release date:		27-August-2000
Tab size used:		4

Notes:
	This robot is now released to the public domain.  It's a mess, and
	I haven't bothered to clean any of it up... so you'll have make
	do I'm afraid.  There is lots of code in here that simply isn't
	used any more, but which I've not deleted in case I need it in
	the future.

	This car takes a small amount of time to start when
	the track has s-bends.  It will take longer the more s-bends
	there are.
*/ 

#include <stdio.h>
#include <string.h> 
#include <stdlib.h> 
#include <math.h> 

#include "car.h" 
#include "track.h"
#include "misc.h"

// get hold of the external surface type variable
extern int surface;

// for team-mates to tell where Sparky is
//int		SparkySeg = -1;
//int		SparkyToEnd = 0;
//int		SparkyToLeft = 0;

#define	WRITE_ELLIPSE_FILE

/*
namespace Spk4
{
*/
//const double cInitialBufferRatio = 0.125;	// ratio of width }- takes min of these
const double cInitialBufferRatio = 0.5;		// ratio of width }- takes min of these
const double cInitialBufferFt = 7.5;		// in ft		  }

const double cTrackBuffer = 0.0;			// in ft from edges of track
const double cApexLift = 0.5;				// in ft from corner

const double cSBendAdjust = 5;				// in ft
const double cSBendMaxBuffer = 0.95;		// ratio of width

const int cTakeCareTime = 100;				// time to take care for

const double cEllRatio = 1.365;				// K = a/b for ellipse. K >= 1
const double cGravityAccel = 32.2;

const double cCarWid = CARWID * 1.75;	// make out cars are wider than they really are
const double cCarDiag = hypot(CARWID, CARLEN);

// pit constants copied from Car::move_car() in carz.cpp
const double cPitSecsPerDamage = 0.005;
const double cPitSecsPerFuel = 0.05;

static double CORN_MYU = 1.05;		// lateral g's expected when cornering

const double BRAKE_ACCEL = 35.5;	// accel available while braking (straight)
const double BRAKE_COEF = 0.8;
//const double BRAKE_ACCEL = 27.0;	// accel available while braking (straight)
//const double BRAKE_COEF = 0.95;

//const double BRAKE_ACCEL_BEND = 27;	// accel available while braking (bend)
//const double BRAKE_COEF_BEND = 0.95;
const double BRAKE_ACCEL_BEND = 35.5;	// accel available while braking (bend)
const double BRAKE_COEF_BEND = 0.8;

const double APEX	= 0.05;
const double OUTER	= 0.75;

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

	bool	operator==( const Vec2& v ) const;
	bool	operator!=( const Vec2& v ) const;

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

bool	Vec2::operator==( const Vec2& v ) const
{
	return x == v.x && y == v.y;
}

bool	Vec2::operator!=( const Vec2& v ) const
{
	return x != v.x || y != v.y;
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

class	Ellipse
{
public:
	Ellipse();
	Ellipse( const Vec2& centre, double a, double b, double angle );
	Ellipse( const Ellipse& ellipse );

	Vec2	Centre() const;
	double	XRadius() const;
	double	YRadius() const;
	double	Angle() const;
	double	Ratio() const;

	bool	LineCrosses(	const Vec2& p, const Vec2& v,
							double& t1, double& t2 ) const;
	double	CalcLocalTheta( const Vec2& point ) const;
	double	CalcTheta( const Vec2& point ) const;
	bool	IsPointInside( const Vec2& point ) const;
	Vec2	CalcLocalPoint( double theta ) const;
	Vec2	CalcPoint( double theta ) const;
	double	CalcRadius( double theta ) const;
	double	CalcThetaForRadius( double radius ) const;
	Vec2	CalcTangent( double theta ) const;
	bool	CalcTangentsToPoint(	const Vec2& point, double& theta1,
									double& theta2 ) const;
	double	CalcDistance( double theta1, double theta2 ) const;

	bool	EllipsesTouch( const Ellipse& ellipse ) const;
	bool	FindCrossingPoints( const Ellipse& ellipse,
								double& theta1, double& theta2 ) const;

private:
	Vec2	m_centre;	// centre of ellipse in global coords
	double	m_a;		// major radius
	double	m_b;		// minor radius
	double	m_ratio;	// = a / b
	double	m_c;		// distance of focal points from centre
	double	m_e;		// eccentricity
	double	m_angle;	// of +ve major axis
	Vec2	m_right;	// local coord system in global coords
	Vec2	m_up;		// local coord system in global coords
	Vec2	m_f1;		// 1st focal point in global coords
	Vec2	m_f2;		// 2nd focal point in global coords
};

Ellipse::Ellipse()
:	m_centre(0, 0),
	m_a(1),
	m_b(1),
	m_ratio(1),
	m_c(0),
	m_e(0),
	m_angle(0),
	m_right(1, 0),
	m_up(0, 1),
	m_f1(0, 0),
	m_f2(0, 0)
{
}

Ellipse::Ellipse(
	const Vec2&	centre,
	double		a,
	double		b,
	double		angle )
:	m_centre(centre),
	m_a(a),
	m_b(b),
	m_ratio(a / b),
	m_c(sqrt(a * a - b * b)),
	m_e(m_c / a),
	m_angle(angle),
	m_right(Vec2::FromAngle(angle)),
	m_up(m_right.GetNormal()),
	m_f1(m_centre + m_right * m_c),
	m_f2(m_centre - m_right * m_c)
{
//	ASSERT( m_a >= m_b );
}

Ellipse::Ellipse( const Ellipse& ellipse )
:	m_centre(ellipse.m_centre),
	m_a(ellipse.m_a),
	m_b(ellipse.m_b),
	m_ratio(ellipse.m_ratio),
	m_c(ellipse.m_c),
	m_e(ellipse.m_e),
	m_angle(ellipse.m_angle),
	m_right(ellipse.m_right),
	m_up(ellipse.m_up),
	m_f1(ellipse.m_f1),
	m_f2(ellipse.m_f2)
{
}

Vec2	Ellipse::Centre() const
{
	return m_centre;
}

double	Ellipse::XRadius() const
{
	return m_a;
}

double	Ellipse::YRadius() const
{
	return m_b;
}

double	Ellipse::Angle() const
{
	return m_angle;
}

double	Ellipse::Ratio() const
{
	return m_ratio;
}

bool	Ellipse::LineCrosses(
	const Vec2& line, const Vec2& lineDir, double& t1, double& t2 ) const
{
	// translate line into ellipse coord system
	Vec2	p = line - m_centre;
	p = Vec2(m_right * p, m_up * p);
	Vec2	v(m_right * lineDir, m_up * lineDir);

	// solve line crossing for local ellipse coord system
	double	a2 = m_a * m_a;
	double	b2 = m_b * m_b;

	double	A = a2 * v.y * v.y + b2 * v.x * v.x;
	double	B = 2 * (a2 * p.y * v.y + b2 * p.x * v.x);
	double	C = a2 * p.y * p.y + b2 * p.x * p.x - a2 * b2;

	double	inner = B * B - 4 * A * C;
	if( inner < 0 )
		return false;

	double	sqroot = sqrt(inner);

	// fill in the answers.  these don't need to be converted
	//	back to the global coord system.
	t1 = (-B - sqroot) / (2 * A);
	t2 = (-B + sqroot) / (2 * A);

	return true;
}

double	Ellipse::CalcLocalTheta( const Vec2& point ) const
{
	double	theta = atan2(point.y * m_ratio, point.x);
	return theta >= 0 ? theta : theta + cPi * 2;
}

double	Ellipse::CalcTheta( const Vec2& point ) const
{
	// translate point into local ellipse coord system
	Vec2	p = point - m_centre;
	p = Vec2(m_right * p, m_up * p);

	return CalcLocalTheta(p);
}

bool	Ellipse::IsPointInside( const Vec2& point ) const
{
	// for a point to be inside the ellipse the sum of the
	//	distances from both focal points must be <= 2a.
	Vec2	f1 = point - m_f1;
	Vec2	f2 = point - m_f2;
	double	d1 = sqrt(f1 * f1);
	double	d2 = sqrt(f2 * f2);
	return d1 + d2 <= 2 * m_a;
}

Vec2	Ellipse::CalcLocalPoint( double theta ) const
{
	return Vec2(m_a * cos(theta), m_b * sin(theta));
}

Vec2	Ellipse::CalcPoint( double theta ) const
{
	Vec2	pt = CalcLocalPoint(theta);

	// need to translate point into global coord system
	Vec2	aright(m_right.x, m_up.x);
	Vec2	aup(m_right.y, m_up.y);
	Vec2	apt(aright * pt, aup * pt);
	return m_centre + apt;
}

double	Ellipse::CalcRadius( double theta ) const
{
/*
	double	sine = sin(theta);
	double	cosine = cos(theta);

	double	a = max(m_radius1, m_radius2);
	double	b = min(m_radius1, m_radius2);
	double	radius = pow(a * a * sine * sine + b * b *  cosine * cosine,
							1.5) / (a * b);
	return 1 / radius;
*/
	double	sine = sin(theta);
	double	e2 = m_e * m_e;
	double	radius = m_a * (1 - e2) / pow(1 - e2 * sine * sine, 1.5);
	return radius;
}

double	Ellipse::CalcThetaForRadius( double radius ) const
{
	if( radius < m_b / m_ratio )
		return 0;
	else if( radius > m_a * m_ratio )
		return cPi;

	double	e2 = m_e * m_e;
	double	sine2 = (1 - pow(m_a * (1 - e2) / radius, 2 / 3.0)) / e2;
	if( sine2 < 0 || sine2 > 1 )
		return 0;

	double	sine = sqrt(sine2);
	double	theta = asin(sine);
	return theta;
}

Vec2	Ellipse::CalcTangent( double theta ) const
{
	double	sine = sin(theta);
	double	cosine = cos(theta);

	double	denom = sqrt(m_b * m_b * cosine * cosine +
							m_a * m_a * sine * sine);

	Vec2	tangent(-m_a * sine / denom, m_b * cosine / denom);

	// need to translate tangent into global coord system
	Vec2	aright(m_right.x, m_up.x);
	Vec2	aup(m_right.y, m_up.y);
	Vec2	atangent(aright * tangent, aup * tangent);
	return atangent;
}

bool	Ellipse::CalcTangentsToPoint(
	const Vec2& point, double& theta1, double& theta2 ) const
{
	// translate point into local ellipse coord system
	Vec2	p = point - m_centre;
	p = Vec2(m_right * p, m_up * p);

	// uses identity: a sin theta + b sin theta == r cos(theta - alpha)
	//		where:		r = sqrt(a² + b²), alpha = atan(b / a)

	// let a be the major radius, and b be the minor radius
	// let Q(X, Y) be the point.
	// let the tangent point on ellipse be P.
	// let V = Q - P.
	// let N be normal at p: N(xb/a, ya/b)
	//
	// therefore:		N·V = 0
	//					(xb/a)(x - X) + (ya/b)(y - Y) = 0
	//					b/a[a²cos²@ - Xa cos @] + a/b[b²sin²@ - Yb sin @] = 0
	//					ab cos²@ - Xb cos @ + ab sin²@ - Ya sin @ = 0
	//					ab[cos²@ + sin²@] - Xb cos @ - Ya sin @ = 0
	//					Xb cos theta + Ya sin theta = ab
	//
	// using identity:	r cos(theta - alpha) = ab
	//		where:		r = sqrt(X²b² + Y²a²)
	//		and:		alpha = atan(Ya / Xb)
	//
	//		gives:		cos(theta - alpha) = ab / r
	//					theta - alpha = acos(ab / r)
	//					theta = alpha + acos(ab / r)
	//					theta = atan(Ya / Xb) + acos(ab/sqrt(X²b² + Y²a²))

	double	r = sqrt(m_a * m_a + m_b * m_b);
	double	alpha = atan2(p.y * m_a, p.x * m_b);

	double	cosine = m_a * m_b /
				sqrt(p.x * p.x * m_b * m_b + p.y * p.y * m_a * m_a);
	if( cosine < -1 || cosine > 1 )
	{
		// does this mean that p is inside the ellipse??
		return false;
	}

	// there are 2 possible solutions for acos(cosine).. primary and
	//	secondary.  acos will give us the primary solution (p).  the
	//	secondary solultion (s) is given by: s = -p
	double	primaryAngle = acos(cosine);

	theta1 = NormaliseAngle(primaryAngle + alpha);
	theta2 = NormaliseAngle(alpha - primaryAngle);

	return true;
}

double	Ellipse::CalcDistance(
	double	startTheta,
	double	deltaTheta ) const
{
	double	distance = 0;
	double	delta = cPi / 16;
	Vec2	pt1;
	Vec2	pt2 = CalcLocalPoint(startTheta);
	for( double t = delta; t < deltaTheta; t += delta )
	{
		double	theta = startTheta + t;

		pt1 = pt2;
		pt2 = CalcLocalPoint(theta);

		distance += (pt2 - pt1).GetLength();
	}

	pt1 = pt2;
	pt2 = CalcLocalPoint(startTheta + deltaTheta);
	distance += (pt2 - pt1).GetLength();

	return distance;
}

bool	Ellipse::EllipsesTouch( 
	const Ellipse&	ellipse ) const
{
//	double f1a = (m_f1 - ellipse.m_f1).GetLength();
//	double f2a = (m_f2 - ellipse.m_f2).GetLength();
//	double fa = f1a + f2a;
//	double f1b = (m_f1 - ellipse.m_f2).GetLength();
//	double f2b = (m_f2 - ellipse.m_f1).GetLength();
//	double fb = f1b + f2b;

	Vec2	d = ellipse.m_centre - m_centre;
	double	d2 = d * d;
	if( d2 > (m_a + ellipse.m_a) * (m_a + ellipse.m_a) )
		// too far away to touch
		return false;

	if( d2 <= (m_b + ellipse.m_b) * (m_b + ellipse.m_b) )
		// so close that they must be touching
		return true;

	if( IsPointInside(ellipse.m_centre) || ellipse.IsPointInside(m_centre) )
		// must be touching
		return true;

	// now find tangent points from centre of this ellipse to other ellipse.
	//	theta values returned are for the other ellipse.
	double	theta1, theta2;
	if( !ellipse.CalcTangentsToPoint(m_centre, theta1, theta2) )
		return true;

	// work out theta values for this ellipse
	theta1 = CalcTheta(ellipse.CalcPoint(theta1));
	theta2 = CalcTheta(ellipse.CalcPoint(theta2));

	// theta1 and theta2 are in no particular order... we need to work out
	//	which section is closest to our centre: either [theta1...theta2]
	//	or [theta2...theta1].  we do this by assuming that a line between
	//	the centres of the 2 ellipses will have a value of theta which is
	//	in the range.
	double	theta = CalcTheta(ellipse.m_centre);
	double	startTheta = min(theta1, theta2);
	double	rangeTheta = fabs(theta1 - theta2);	// doesn't cross 0 deg
	if( !AngleInRange(theta, startTheta, rangeTheta) )
	{
		startTheta = max(theta1, theta2);
		rangeTheta = c2Pi - rangeTheta;			// now crosses 0 deg
	}

	// range is now [startTheta...startTheta + rangeTheta]
	double	step = min(rangeTheta / 32, cPi / 32);
	bool	done = false;
	for( double delta = 0; !done; delta += step )
	{
		if( delta >= rangeTheta )
		{
			delta = rangeTheta;
			done = true;
		}

		Vec2	pt = CalcPoint(startTheta + delta);
		if( ellipse.IsPointInside(pt) )
			return true;
	}

	return false;
}

bool	Ellipse::FindCrossingPoints( 
	const Ellipse&	ellipse,
	double&			retTheta1,
	double&			retTheta2 ) const
{
	// now find tangent points from centre of this ellipse to other ellipse.
	//	theta values returned are for the other ellipse.
	double	theta1, theta2;
	if( !ellipse.CalcTangentsToPoint(m_centre, theta1, theta2) )
		return true;

	// work out theta values for this ellipse
	theta1 = CalcTheta(ellipse.CalcPoint(theta1));
	theta2 = CalcTheta(ellipse.CalcPoint(theta2));

	// theta1 and theta2 are in no particular order... we need to work out
	//	which section is closest to our centre: either [theta1...theta2]
	//	or [theta2...theta1].  we do this by assuming that a line between
	//	the centres of the 2 ellipses will have a value of theta which is
	//	in the range.
	double	theta = CalcTheta(ellipse.m_centre);
	double	startTheta = min(theta1, theta2);
	double	rangeTheta = fabs(theta1 - theta2);	// doesn't cross 0 deg
	if( !AngleInRange(theta, startTheta, rangeTheta) )
	{
		startTheta = max(theta1, theta2);
		rangeTheta = c2Pi - rangeTheta;			// now crosses 0 deg
	}

	// range is now [startTheta...startTheta + rangeTheta]
	double	step = min(rangeTheta / 32, cPi / 32);
	bool	done = false;

	int		nFound = 0;
	Vec2	pt2 = CalcPoint(startTheta);
	for( double delta = step; nFound < 2 && !done; delta += step )
	{
		if( delta >= rangeTheta )
		{
			delta = rangeTheta;
			done = true;
		}

		double	theta = startTheta + delta;

		Vec2	pt1 = pt2;
		pt2 = CalcPoint(theta);

		double	t1, t2;
		if( ellipse.LineCrosses(pt1, pt2 - pt1, t1, t2) &&
			(t1 >= 0 && t1 <= 1 || t2 >= 0 && t2 <= 1) )
		{
			// found a crossing point, maybe two crossing points
			if( t1 >= 0 && t1 <= 1 )
			{
				double	th = CalcTheta(pt1 + (pt2 - pt1) * t1);
				if( nFound++ == 0 )
					retTheta1 = th;
				else
					retTheta2 = th;
			}

			if( nFound < 2 && t1 >= 0 && t1 <= 1 )
			{
				double	th = CalcTheta(pt1 + (pt2 - pt1) * t2);
				if( nFound++ == 0 )
					retTheta1 = th;
				else
					retTheta2 = th;
			}
		}
	}

	return nFound == 2;
}

class	Curve
{
private:
	Ellipse	m_ellipse;
};

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

double	CalcK( double r, double b, double /*apexLift*/ )
{
	double	K = cEllRatio;
	if( b < r * K )
			K = max(1, b / r);

	return K;
}

double	CalcClosestPointOnLine( const Vec2& lp, const Vec2& lv, const Vec2& p )
{
	double	denom = lv * lv;
	if( denom == 0 )
		return 0;
	return ((p - lp) * lv) / denom;
}

double	CalcDistanceToLine( const Vec2& lp, const Vec2& lv, const Vec2& p )
{
	double	vLength = lv.GetLength();
	if( vLength == 0 )
		return 0;
	return (lv % (p - lp)) / vLength;
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

bool	CalcTangentPt(
	const Vec2& pt,		// point to calc tangent to
	const Ellipse& ell,	// ellipse for tangent
	bool isLeft,		// true if anti-clockwise

	Vec2& tanPt,		// point calculated
	double& tanTheta )	// theta of point
{
	double	theta1, theta2;
	if( !ell.CalcTangentsToPoint(pt, theta1, theta2) )
		return false;

	Vec2	pt1 = ell.CalcPoint(theta1);
	Vec2	pt2 = ell.CalcPoint(theta2);

	Vec2	n = (ell.Centre() - pt).GetNormal();

	if( (n * (pt1 - pt) > 0) ^ (!isLeft) )
	{
		tanPt = pt2;
		tanTheta = theta2;
	}
	else
	{
		tanPt = pt1;
		tanTheta = theta1;
	}

	return true;
}

bool	LineCrossesSegment(
	const Vec2&		p,
	const Vec2&		v,
	const segment*	pSeg,
	double			edge,
	double&			t )
{
	Vec2	b(pSeg->beg_x, pSeg->beg_y);

	if( pSeg->radius == 0 )
	{
		Vec2	e(pSeg->end_x, pSeg->end_y);
		if( !LineCrossesLine(p, v, b, e - b, t) )
			return false;

		// work out where crossing is on segment
		double	t2;
		LineCrossesLine( b, e - b, p, v, t2 );

		// test that crossing point is within segment
		return t2 > -0.0001 && t2 < 1.0001;
	}
	else
	{
		Vec2	c(pSeg->cen_x, pSeg->cen_y);

		double	t1, t2;
		if( !LineCrossesCircle(p, v, c, fabs(pSeg->radius) + edge, t1, t2) )
			return false;

		if( edge > 0 && t1 < 0 && t2 >= 0 )
		{
			// we are inside the circle... but we added 'edge' to the
			//	radius.  lets check whether we are inside the
			//	normal radius
			LineCrossesCircle(p, v, c, fabs(pSeg->radius), t1, t2);
		}

		if( t1 > -0.0001 )
		{
			Vec2	p2 = p + v * t1;
			double	angle = (p2 - c).GetAngle();
			double	begAng = (b - c).GetAngle();
			if( pSeg->radius > 0 )
				angle = NormaliseAngleLimit(angle - begAng, pSeg->length);
			else
				angle = NormaliseAngleLimit(begAng - angle, pSeg->length);

			if( angle > -0.0001 && angle < pSeg->length + 0.0001 )
			{
				t = t1;
				return true;
			}
		}

		if( t2 > -0.0001 )
		{
			Vec2	p2 = p + v * t2;
			double	angle = (p2 - c).GetAngle();
			double	begAng = (b - c).GetAngle();
			if( pSeg->radius > 0 )
				angle = NormaliseAngleLimit(angle - begAng, pSeg->length);
			else
				angle = NormaliseAngleLimit(begAng - angle, pSeg->length);

			if( angle > -0.0001 && angle < pSeg->length + 0.0001 )
			{
				t = t2;
				return true;
			}
		}
	}

	return false;
}

void	CalcCurve(
	double rad, double len, double width, double apex, double out,
	double& newRad, Vec2& newCen )
{
	double	r = fabs(rad);
	Vec2	c(-r, 0);
	double	nr = r + apex;
	Vec2	p(cos(len * 0.5) * nr - r, sin(len * 0.5) * nr);
	Vec2	pt = (c - p).GetNormal();
	Vec2	q = LineCrossesLinePt(p, pt, Vec2(width - out, 0), Vec2(0, 1));
	double	h = (p - q).GetLength();
	Vec2	s(q.x, q.y - h);
	Vec2	c2 = LineCrossesLinePt(s, Vec2(-1, 0), c, c - p);
	double	r2 = s.x - c2.x;

	newRad = r2;
	newCen = c2;
}

void	CalcEllipse(
	const segment*	pSeg,		// segment to calc apex ellipse from
	double			apexAngle,	// angle of apex (radians) (from start)
	double			width,		// width of track (ft)
	double			apex,		// apex distance (ft)
	double			out,		// outer distance (ft)
	double			ellRatio,	// ratio of a to b to use (=a/b)

	Ellipse&		ellipse,	// ellipse calculated
	double&			theta,		// parameter angle of tangent point
	double&			straightLen )// length along straight of tan point
{
	double	r = fabs(pSeg->radius);

	// if the length of the arc is more than 180deg, then
	//	generate a circle rather than an ellipse
	if( pSeg->length > cPi + 0.001 )
		ellRatio = 1;

	// centre of bend
	Vec2	c(-r, 0);

	// new radius (inc any inner apex value)
	double	nr = r + apex;

	// apex point
	Vec2	p(apex, 0);

	double	toApex = apexAngle;
	double	fromApex = pSeg->length - apexAngle;

again:

	// now we stretch everything to find an elliptical answer
	Vec2	f(c + Vec2::FromAngle(fromApex) * (r + width - out));
	Vec2	f1(f.x, f.y * ellRatio);

	// tangent at a
	Vec2	ft = (f - c).GetNormal();
	Vec2	f1t = Vec2(ft.x, ft.y * ellRatio).GetUnit();

	// calculate tangent at apex
	Vec2	pt(0, 1);

	// figure where tangent crosses outer line
	Vec2	q1 = LineCrossesLinePt(p, pt, f1, f1t);

	// get distance between p and q'
	double	h = (p - q1).GetLength();

	// work out point where circle (ellipse) will touch the
	//	outer line
	Vec2	s1 = q1 + f1t * h;
	Vec2	s(s1.x, s1.y / ellRatio);

	// work out normal at this point
	Vec2	s1n = f1t.GetNormal();

	// work out the centre of the new circle
	Vec2	c2 = LineCrossesLinePt(s1, s1n, c, c - p);
	double	r2 = p.x - c2.x;

	// work out global coord vector along the major axis
	double	globalApexAngle = (Vec2(pSeg->beg_x, pSeg->beg_y) -
								Vec2(pSeg->cen_x, pSeg->cen_y)).GetAngle();
	if( pSeg->radius > 0 )
		globalApexAngle += apexAngle;
	else
		globalApexAngle -= apexAngle;
	Vec2	v = Vec2::FromAngle(globalApexAngle);

	// work out ellipse in local coords
	double	a = r2;
	double	b = r2 / ellRatio;
	Ellipse	localEll(c2, a, b, 0);

	if( ellRatio > 1 && b / ellRatio < nr )
	{
		ellRatio *= 0.95;
		if( ellRatio < 1 )
			ellRatio = 1;
		goto again;
	}

	// translate the answers back to the ellipse in
	//	global coordinates
	Vec2	cen = Vec2(pSeg->cen_x, pSeg->cen_y) - v * (a - r - apex);

	// fill in the ellipse and the value of theta for the
	//	outer tangent point
	ellipse = Ellipse(cen, a, b, globalApexAngle);
	theta = localEll.CalcTheta(s);
	if( pSeg->radius < 0 )
		theta = NormaliseAngle(2 * cPi - theta);
	straightLen = (s - f).GetLength();
}

bool	ArcCrossesSegment(
	const Vec2& cen,
	double rad,			// +ve=anti-clockwise, -ve=clockwise
	double startAngle,
	const segment* pSeg,
	double& len )
{
	if( pSeg->radius == 0 )
	{
		Vec2	b(pSeg->beg_x, pSeg->beg_y);
		Vec2	e(pSeg->end_x, pSeg->end_y);
		double	t1, t2;
		if( !LineCrossesCircle(b, e - b, cen, fabs(rad), t1, t2) )
			return false;

		if( t1 > -0.0001 && t1 < 1.0001 )
		{
			double	angle = ((b + (e - b) * t1) - cen).GetAngle();
			len = NormaliseAngle(angle - startAngle);
			if( rad < 0 )
				double	len = NormaliseAngle(startAngle - angle);
			return len <= cPi;
		}

		if( t2 > -0.0001 && t2 < 1.0001 )
		{
			double	angle = ((b + (e - b) * t2) - cen).GetAngle();
			len = NormaliseAngle(angle - startAngle);
			if( rad < 0 )
				double	len = NormaliseAngle(startAngle - angle);
			return len <= cPi;
		}
	}
	else
	{
		Vec2	c(pSeg->cen_x, pSeg->cen_y);
		double	t1, t2;
		if( !CircleCrossesCircle(cen, fabs(rad), c, fabs(pSeg->radius), t1, t2) )
			return false;

		// work out len
		double	len1 = rad > 0 ? t1 - startAngle : startAngle - t1;
		len1 = NormaliseAngle(len1);
		double	len2 = rad > 0 ? t2 - startAngle : startAngle - t2;
		len2 = NormaliseAngle(len2);
		len = min(len1, len2);
		return true;
	}

	return false;
}

bool	EllipseCrossesSegment(
	const Ellipse& ell,		// ellipse to test
	bool acw,				// direction to go from apex (theta == 0)
	const segment* pSeg,	// segment to test against
	double offset,			// offset of segment +ve=left, -ve=right

	double&	theta )			// theta value of crossing point
{
	segment	seg = *pSeg;

	if( seg.radius == 0 )
	{
		// adjust straight by offset
		Vec2	b(seg.beg_x, seg.beg_y);
		Vec2	e(seg.end_x, seg.end_y);
		Vec2	v = (e - b).GetUnit();
		Vec2	n = v.GetNormal();

		b = b + n * offset;
		e = e + n * offset;

		double	t1, t2;
		if( ell.LineCrosses(b, e - b, t1, t2) )
		{
			if( t1 >= -0.0001 && t1 < 1.0001 )
			{
				Vec2	p = b + (e - b) * t1;
				theta = NormaliseAngleLimit(ell.CalcTheta(p), 0);
				if( acw && theta >= -0.0001 || !acw && theta <= 0.0001 )
					return true;
			}

			if( t2 >= -0.0001 && t2 < 1.0001 )
			{
				Vec2	p = b + (e - b) * t2;
				theta = NormaliseAngleLimit(ell.CalcTheta(p), 0);
				if( acw && theta >= -0.0001 || !acw && theta <= 0.0001 )
					return true;
			}
		}
	}
	else
	{
		Vec2	c(seg.cen_x, seg.cen_y);
		Vec2	b(seg.beg_x, seg.beg_y);
		Vec2	e(seg.end_x, seg.end_y);

		double	startAngle = (b - c).GetAngle();
		if( seg.radius < 0 )
			startAngle = (e - c).GetAngle();
		startAngle = NormaliseAngle(startAngle);

		double	rangeAngle = seg.length;
		double	r = fabs(seg.radius - offset);

		Vec2	p2 = c + Vec2::FromAngle(startAngle) * r;
		double	step = cPi / 32;
		bool	done = false;
		for( double delta = step; !done; delta += step )
		{
			if( delta >= rangeAngle )
			{
				delta = rangeAngle;
				done = true;
			}

			double	angle = startAngle + delta;
			Vec2	p1 = p2;
			p2 = c + Vec2::FromAngle(angle) * r;
			Vec2	v = p2 - p1;

			double	t1, t2;
			if( !ell.LineCrosses(p1, v, t1, t2) )
				continue;

			if( (t1 < -0.0001 || t1 > 1.0001) &&
				(t2 < -0.0001 || t2 > 1.0001) )
				continue;

			double	theta1 = NormaliseAngleLimit(ell.CalcTheta(p1 + v * t1), 0);
			double	theta2 = NormaliseAngleLimit(ell.CalcTheta(p1 + v * t2), 0);

			if( fabs(theta1) > fabs(theta2) )
			{
				double	temp = t1;
				t1 = t2;
				t2 = temp;

				temp = theta1;
				theta1 = theta2;
				theta2 = temp;
			}

			if( t1 > -0.0001 && t1 < 1.0001 )
			{
				if(  acw && theta1 >= -0.0001 && theta1 <=  cPi + 0.0001 ||
					!acw && theta1 <=  0.0001 && theta1 >= -cPi - 0.0001 )
				{
					theta = theta1;
					return true;
				}
			}

			if( t2 > -0.0001 && t2 < 1.0001 )
			{
				if(  acw && theta2 >= -0.0001 && theta2 <=  cPi + 0.0001 ||
					!acw && theta2 <=  0.0001 && theta2 >= -cPi - 0.0001 )
				{
					theta = theta2;
					return true;
				}
			}
		}
	}

	theta = acw ? cPi : -cPi;
	return false;
}

bool	IsSegBefore( int seg1, int seg2, int nSeg )
{
	if( seg1 < seg2 )
	{
		return seg1 + nSeg / 2 >= seg2;
	}
	else if( seg1 > seg2 )
	{
		return seg1 > seg2 + nSeg / 2;
	}

	return false;
}

class	Info
{
public:
	Info() :	m_set(false), m_isApex(false), m_replacedBySeg(-1),
				m_apexLen(0), m_inBuffer(0), m_inCrossSeg(-1), m_outBuffer(0),
				m_outCrossInnerSeg(-1), m_inTheta(-cPi), m_outTheta(cPi) {}

public:
	bool	m_set;
	Ellipse	m_ell;		// largest apex ellipse to use
	bool	m_isApex;	// true if is really an apex segment
	int		m_replacedBySeg;	// set if this is overruled by another seg
	double	m_apexLen;	// apex angle from start of segment
	Vec2	m_apexDir;	// centre to apex direction vector
	Vec2	m_apexPt;	// point for apex
	double	m_inBuffer;	// buffer from edge, leading into bend
	int		m_inCrossSeg;
	double	m_outBuffer;// buffer from edge, leading out of bend
	int		m_outCrossInnerSeg;
	double	m_inTheta;	// angle of crossing pt, leading into bend
	double	m_outTheta;	// andle of crossing pt, leading out of bend
};

class	Sparky4Track
{
public:
	Sparky4Track();
	~Sparky4Track();

	void			Empty();
	void			Initialise( const track_desc& track );

	int				NSeg() const;
	double			Width() const;

	const track_desc& GetTrack() const;
	track_desc&		GetTrack();
	const segment&	LftWall( int seg ) const;
	segment&		LftWall( int seg );
	const segment&	RgtWall( int seg ) const;
	segment&		RgtWall( int seg );

	Vec2			LftBeg( int seg ) const;
	Vec2			LftEnd( int seg ) const;
	Vec2			LftCen( int seg ) const;
	Vec2			RgtBeg( int seg ) const;
	Vec2			RgtEnd( int seg ) const;
	Vec2			RgtCen( int seg ) const;

	const Info&		GetInfo( int seg ) const;
	Info&			GetInfo( int seg );

	double			GetRadius( int seg ) const;
	double			GetLength( int seg ) const;

	bool			IsAngleInSegment( int seg, double angle ) const;
	bool			SegContainsPt( int seg, const Vec2& pt ) const;
	int				FindSegContainingPt(	const Vec2& pt, int startSeg = 0,
											bool forward = true ) const;

private:
	void			CopyTrack( const track_desc& track );
	void			ListApexes();
	void			CalcApex( int seg, double inBuffer, double outBuffer );
	void			AdjustSBends( int apexesBetween );
	bool			EllipseCrosses( const Ellipse& ell, int ellSeg,
									double buffer, bool acw,
									int& crossSeg, double& crossTheta );

private:
	track_desc		m_track;	// track from RARS system
	Info*			m_pInfo;
	int				m_nApexes;
	int*			m_pApexes;
};

Sparky4Track::Sparky4Track()
:	m_pInfo(0),
	m_nApexes(0),
	m_pApexes(0)
{
	memset( &m_track, 0, sizeof(m_track) );
}

Sparky4Track::~Sparky4Track()
{
	Empty();
}

void	Sparky4Track::Empty()
{
	delete [] m_track.lftwall;
	delete [] m_track.rgtwall;
	memset( &m_track, 0, sizeof(m_track) );
	delete [] m_pInfo;
	m_pInfo = 0;
	m_nApexes = 0;
	delete [] m_pApexes;
	m_pApexes = 0;
}

void	Sparky4Track::Initialise( const track_desc& track )
{
	int seg;

	Empty();

	m_track = track;
	m_pInfo = new Info[m_track.NSEG];
	m_nApexes = 0;
	m_pApexes = new int[m_track.NSEG];

	// make our own copy of the track, with segments which are cTrackBuffer inside
	//	the edges of the actual track.
	CopyTrack( track );

	// work out which segments are to be treated as apexes
	ListApexes();

	// figure the default apex ellipses now...
	double	edge = min(m_track.width * cInitialBufferRatio, cInitialBufferFt);
	for( seg = 0; seg < m_track.NSEG; seg++ )
	{
		CalcApex( seg, edge, edge );
	}

	// now we may need to adjust the track apex ellipses due to s-bends
	AdjustSBends( 0 );	// next to each other
	if( m_nApexes >= 6 )
		AdjustSBends( 1 );	// 1 apart
	if( m_nApexes >= 8 )
		AdjustSBends( 2 );	// 2 apart
	if( m_nApexes >= 10 )
		AdjustSBends( 3 );	// 3 apart

	// spread apex ellipses to curves directly after the apex curve
	for( seg = 0; seg < m_track.NSEG; seg++ )
	{
		if( m_pInfo[seg].m_isApex )
		{
			int		crossSeg = m_pInfo[seg].m_outCrossInnerSeg;
			double	rad = GetRadius(seg);
			int		nextSeg = (seg + 1) % m_track.NSEG;
			while(	nextSeg != crossSeg &&
					rad * GetRadius(nextSeg) > 0 &&
					fabs(rad) < fabs(GetRadius(nextSeg)) )
			{
				m_pInfo[nextSeg].m_replacedBySeg = seg;
				m_pInfo[nextSeg].m_ell = m_pInfo[seg].m_ell;
				m_pInfo[nextSeg].m_apexDir = m_pInfo[seg].m_apexDir;
				m_pInfo[nextSeg].m_apexPt = m_pInfo[seg].m_apexPt;
				m_pInfo[nextSeg].m_inTheta = m_pInfo[seg].m_inTheta;
				m_pInfo[nextSeg].m_outTheta = m_pInfo[seg].m_outTheta;

				rad = GetRadius(nextSeg);
				nextSeg = (nextSeg + 1) % m_track.NSEG;
			}
		}
	}

#ifdef	WRITE_ELLIPSE_FILE
	// write the ellipses out to file so I can have a look at them
	FILE*	pFile = fopen("ell.txt", "w");
	for( seg = 0; seg < m_track.NSEG; seg++ )
	{
		if( pFile && m_pInfo[seg].m_set )
		{
			fprintf( pFile, "%.15g %.15g %.15g %.15g %.15g %.15g %.15g\n",
						m_pInfo[seg].m_ell.Centre().x,
						m_pInfo[seg].m_ell.Centre().y,
						m_pInfo[seg].m_ell.XRadius(),
						m_pInfo[seg].m_ell.YRadius(),
						m_pInfo[seg].m_ell.Angle(),
						min(m_pInfo[seg].m_inTheta, m_pInfo[seg].m_outTheta),
						max(m_pInfo[seg].m_inTheta, m_pInfo[seg].m_outTheta) );
		}
	}

	if( pFile )
		fclose( pFile );
#endif
}

int		Sparky4Track::NSeg() const
{
	return m_track.NSEG;
}

double	Sparky4Track::Width() const
{
	return m_track.width;
}

const track_desc&	Sparky4Track::GetTrack() const
{
	return m_track;
}

track_desc&	Sparky4Track::GetTrack()
{
	return m_track;
}

/////////////////////////////////////////////////////////////////////////////

const segment&	Sparky4Track::LftWall( int seg ) const
{
	return m_track.lftwall[seg];
}

/////////////////////////////////////////////////////////////////////////////

segment&	Sparky4Track::LftWall( int seg )
{
	return m_track.lftwall[seg];
}

/////////////////////////////////////////////////////////////////////////////

const segment&	Sparky4Track::RgtWall( int seg ) const
{
	return m_track.rgtwall[seg];
}

/////////////////////////////////////////////////////////////////////////////

segment&	Sparky4Track::RgtWall( int seg )
{
	return m_track.rgtwall[seg];
}

/////////////////////////////////////////////////////////////////////////////

const Info&	Sparky4Track::GetInfo( int seg ) const
{
	return m_pInfo[seg];
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::LftBeg( int seg ) const
{
	return Vec2(m_track.lftwall[seg].beg_x, m_track.lftwall[seg].beg_y);
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::LftEnd( int seg ) const
{
	return Vec2(m_track.lftwall[seg].end_x, m_track.lftwall[seg].end_y);
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::LftCen( int seg ) const
{
	return Vec2(m_track.lftwall[seg].cen_x, m_track.lftwall[seg].cen_y);
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::RgtBeg( int seg ) const
{
	return Vec2(m_track.rgtwall[seg].beg_x, m_track.rgtwall[seg].beg_y);
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::RgtEnd( int seg ) const
{
	return Vec2(m_track.rgtwall[seg].end_x, m_track.rgtwall[seg].end_y);
}

/////////////////////////////////////////////////////////////////////////////

Vec2	Sparky4Track::RgtCen( int seg ) const
{
	return Vec2(m_track.rgtwall[seg].cen_x, m_track.rgtwall[seg].cen_y);
}

/////////////////////////////////////////////////////////////////////////////

Info&	Sparky4Track::GetInfo( int seg )
{
	return m_pInfo[seg];
}

/////////////////////////////////////////////////////////////////////////////

double	Sparky4Track::GetRadius( int seg ) const
{
	double	radius = m_track.lftwall[seg].radius;
	if( radius < 0 )
		radius = m_track.rgtwall[seg].radius;

	return radius;
}

/////////////////////////////////////////////////////////////////////////////

double	Sparky4Track::GetLength( int seg ) const
{
	return m_track.lftwall[seg].length;
}

/////////////////////////////////////////////////////////////////////////////

bool	Sparky4Track::IsAngleInSegment( int seg, double angle ) const
{
	const segment*	pSeg = &m_track.lftwall[seg];
	if( pSeg->radius > 0 )
		angle = NormaliseAngleLimit(angle - pSeg->beg_ang, pSeg->length);
	else
		angle = NormaliseAngleLimit(pSeg->beg_ang - angle, pSeg->length);
	return angle > -0.0001 && angle < pSeg->length + 0.0001;
}

/////////////////////////////////////////////////////////////////////////////

bool	Sparky4Track::SegContainsPt( int seg, const Vec2& pt ) const
{
	if( m_track.lftwall[seg].radius == 0 )
	{
		// straight

		// work out mid line
		Vec2	b((LftBeg(seg) + RgtBeg(seg)) * 0.5);
		Vec2	e((LftEnd(seg) + RgtEnd(seg)) * 0.5);

		// find closest pt on mid line to pt given
		double	t = CalcClosestPointOnLine(b, e - b, pt);
		if( t < -0.0001 || t > 1.0001 )
			return false;

		// find distance of pt from mid line
		double	dist = CalcDistanceToLine(b, e - b, pt);
		double	halfWidth = m_track.width * 0.5;
		return dist <= halfWidth;
	}
	else
	{
		// curve

		// work out angle of point
		double	angle = (pt - LftCen(seg)).GetAngle();

		// angle within range of arc?
		if( !IsAngleInSegment(seg, angle) )
			return false;

		// find distance from centre line
		double	r = (pt - LftCen(seg)).GetLength();
		double	halfWidth = m_track.width * 0.5;
		double	dist = fabs(r - fabs(GetRadius(seg) + halfWidth));
		return dist <= halfWidth;
	}

	// should never get here!
	return false;
}

/////////////////////////////////////////////////////////////////////////////

int		Sparky4Track::FindSegContainingPt(
	const Vec2& pt,
	int startSeg,
	bool forward ) const
{
	int		offset = forward ? 1 : m_track.NSEG - 1;
	int		seg = startSeg;
	do
	{
		if( SegContainsPt(seg, pt) )
			return seg;

		seg = (seg + offset) % m_track.NSEG;
	}
	while( seg != startSeg );

	// didn't find correct segment -- point not in track
	return -1;
}

/////////////////////////////////////////////////////////////////////////////

void	Sparky4Track::CopyTrack( const track_desc& track )
{
	m_track = track;
	m_track.lftwall = new segment[m_track.NSEG];
	m_track.rgtwall = new segment[m_track.NSEG];
	m_track.width = track.width - 2 * cTrackBuffer;
	for( int seg = 0; seg < m_track.NSEG; seg++ )
	{
		if( track.lftwall[seg].radius == 0 )
		{
			// on a straight
			Vec2	b(track.lftwall[seg].beg_x, track.lftwall[seg].beg_y);
			Vec2	e(track.lftwall[seg].end_x, track.lftwall[seg].end_y);
			Vec2	v((e - b).GetUnit());
			Vec2	n(v.GetNormal() * cTrackBuffer);	// points to left

			m_track.lftwall[seg] = track.lftwall[seg];
			m_track.lftwall[seg].beg_x = track.lftwall[seg].beg_x - n.x;
			m_track.lftwall[seg].beg_y = track.lftwall[seg].beg_y - n.y;
			m_track.lftwall[seg].end_x = track.lftwall[seg].end_x - n.x;
			m_track.lftwall[seg].end_y = track.lftwall[seg].end_y - n.y;

			m_track.rgtwall[seg] = track.rgtwall[seg];
			m_track.rgtwall[seg].beg_x = track.rgtwall[seg].beg_x + n.x;
			m_track.rgtwall[seg].beg_y = track.rgtwall[seg].beg_y + n.y;
			m_track.rgtwall[seg].end_x = track.rgtwall[seg].end_x + n.x;
			m_track.rgtwall[seg].end_y = track.rgtwall[seg].end_y + n.y;
		}
		else
		{
			Vec2	c(track.lftwall[seg].cen_x, track.lftwall[seg].cen_y);
			Vec2	b(track.lftwall[seg].beg_x, track.lftwall[seg].beg_y);
			Vec2	e(track.lftwall[seg].end_x, track.lftwall[seg].end_y);

			Vec2	begN = (b - c).GetUnit();
			Vec2	endN = (e - c).GetUnit();

			double	lftR = track.lftwall[seg].radius + cTrackBuffer;
			double	rgtR = track.rgtwall[seg].radius - cTrackBuffer;

			m_track.lftwall[seg] = track.lftwall[seg];
			m_track.lftwall[seg].radius = lftR;
			m_track.lftwall[seg].beg_x = (c + begN * fabs(lftR)).x;
			m_track.lftwall[seg].beg_y = (c + begN * fabs(lftR)).y;
			m_track.lftwall[seg].end_x = (c + endN * fabs(lftR)).x;
			m_track.lftwall[seg].end_y = (c + endN * fabs(lftR)).y;
			m_track.lftwall[seg].beg_ang = begN.GetAngle();
			m_track.lftwall[seg].end_ang = endN.GetAngle();

			m_track.rgtwall[seg] = track.rgtwall[seg];
			m_track.rgtwall[seg].radius = rgtR;
			m_track.rgtwall[seg].beg_x = (c + begN * fabs(rgtR)).x;
			m_track.rgtwall[seg].beg_y = (c + begN * fabs(rgtR)).y;
			m_track.rgtwall[seg].end_x = (c + endN * fabs(rgtR)).x;
			m_track.rgtwall[seg].end_y = (c + endN * fabs(rgtR)).y;
			m_track.rgtwall[seg].beg_ang = begN.GetAngle();
			m_track.rgtwall[seg].end_ang = endN.GetAngle();
		}
	}

	// modify start-finish straight so it definately connects to
	//	the previous seg.
	m_track.lftwall[0].beg_x = m_track.lftwall[m_track.NSEG - 1].end_x;
	m_track.lftwall[0].beg_y = m_track.lftwall[m_track.NSEG - 1].end_y;
	m_track.rgtwall[0].beg_x = m_track.rgtwall[m_track.NSEG - 1].end_x;
	m_track.rgtwall[0].beg_y = m_track.rgtwall[m_track.NSEG - 1].end_y;

	Vec2	b(track.lftwall[0].beg_x, track.lftwall[0].beg_y);
	Vec2	e(track.lftwall[0].end_x, track.lftwall[0].end_y);
	m_track.lftwall[0].length = (e - b).GetLength();
	m_track.rgtwall[0].length = (e - b).GetLength();
}

/////////////////////////////////////////////////////////////////////////////

void	Sparky4Track::ListApexes()
{
	delete [] m_pApexes;
	m_pApexes = new int[NSeg()];
	m_nApexes = 0;

	for( int i = 0; i < NSeg(); i++ )
	{
		int		prevSeg = (i + NSeg() - 1) % NSeg();
		int		seg = i;
		int		nextSeg = (i + 1) % NSeg();

		double	prevRad = GetRadius(prevSeg);
		double	rad = GetRadius(seg);
		double	nextRad = GetRadius(nextSeg);

		if( rad != 0 )
		{
			if( (prevRad * rad <= 0 || fabs(prevRad) > fabs(rad)) &&
				(nextRad * rad <= 0 || fabs(nextRad) > fabs(rad)) )
			{
				// found an apex...
				m_pApexes[m_nApexes++] = seg;
				m_pInfo[seg].m_isApex = true;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////

void	Sparky4Track::CalcApex( int seg, double inBuffer, double outBuffer )
{
	if( m_track.lftwall[seg].radius == 0 )
		return;

	bool	isLeft = m_track.lftwall[seg].radius > 0;

	segment*	pInner = 0;
	segment*	pOuter = 0;

	if( isLeft )
	{
		pInner = &m_track.lftwall[seg];
		pOuter = &m_track.rgtwall[seg];
	}
	else
	{
		pInner = &m_track.rgtwall[seg];
		pOuter = &m_track.lftwall[seg];
	}

	double	startK = cEllRatio;
	double	apexLift = 0;//cApexLift;
//	if( pInner->length > cPi + 0.0001 )
//		startK = 1.0;

//	double	minR = fabs(pInner->radius) + apexLift;
	double	minR = fabs(pInner->radius) + cApexLift;
	double	maxR = 10000000;

	double	loB = minR;
	double	hiB = maxR;

	double	radius = minR;

	Vec2	c = Vec2(pInner->cen_x, pInner->cen_y);
	Vec2	b = Vec2(pInner->beg_x, pInner->beg_y);
	Vec2	e = Vec2(pInner->end_x, pInner->end_y);

	double	startAngle = (b - c).GetAngle();
	double	loInitAngle = 0;
	double	hiInitAngle = isLeft ? pInner->length : -pInner->length;

	if( pInner->length > cPi )
	{
//		loInitAngle = hiInitAngle * 0.5;
//		hiInitAngle = hiInitAngle * 0.5;
	}

	double	loAngle = loInitAngle;
	double	hiAngle = hiInitAngle;

	enum Mode { maximiseBoth, maximiseIn, maximiseOut };
	Mode	mode = maximiseBoth;

	while( loB < hiB )
	{
		if( loB + 0.1 >= hiB )
			hiB = loB;

		double	midB = (hiB + loB) * 0.5;
		double	K = CalcK(radius, midB, apexLift);
		double	midA = midB * K;
		double	midAngle = startAngle + (loAngle + hiAngle) * 0.5;

		Vec2	v = Vec2::FromAngle(midAngle);
		Vec2	cen = c - v * (midA - (radius + apexLift));

		Ellipse	ell(cen, midA, midB, midAngle);

		int		crossInSeg, crossOutSeg;
		double	crossInTheta, crossOutTheta;
		bool	crossIn = EllipseCrosses(ell, seg, inBuffer, !isLeft, crossInSeg, crossInTheta);
		bool	crossOut = EllipseCrosses(ell, seg, outBuffer, isLeft, crossOutSeg, crossOutTheta);

		if( crossOut )
		{
			if( crossIn )
			{
				// radius is too large
				hiB = midB;

				// adjust maxR so we don't do so many iterations to find
				//	the solution
				if( midB * 1.25 < maxR )
					maxR = midB * 1.25;
			}
			else
			{
				if( fabs(loAngle - hiAngle) > 0.01 )
				{
					// apex needs to move acw
					loAngle = (loAngle + hiAngle) * 0.5;

					// reset size search params
					K = startK;
					loB = minR;
					hiB = maxR;
				}
				else
				{
					if( mode == maximiseBoth )//|| mode == maximiseOut )
					{
						// just make smaller now
						hiB = midB;

						if( loB >= hiB )
						{
							// got to the end... change to "maximise-in" mode
							K = startK;
							loB = minR;
							hiB = maxR;
							mode = maximiseIn;
						}
					}
					else // mode == maximiseIn
					{
						// the angle has been moved right to the end, and we have
						//	found that we can never make the radius small enough
						//	that the Out won't cross.  So we just want to deal with
						//	the In now.
						loB = midB;
					}
				}
			}
		}
		else
		{
			if( crossIn )
			{
				if( fabs(loAngle - hiAngle) > 0.01 )
				{
					// apex need to move cw
					hiAngle = (loAngle + hiAngle) * 0.5;

					// reset size search params
					K = startK;
					loB = minR;
					hiB = maxR;
				}
				else
				{
					if( mode == maximiseBoth )
					{
						// just make smaller now
						hiB = midB;

						if( loB >= hiB )
						{
							// got to the end... change to "maximise-out" mode
							K = startK;
							loB = minR;
							hiB = maxR;
							mode = maximiseOut;
						}
					}
					else
					{
						// the angle has been moved right to the end, and we have
						//	found that we can never make the radius small enough
						//	that the In won't cross.  So we just want to deal with
						//	the Out now.
						loB = midB;
					}
				}
			}
			else
			{
				// radius is too small
				loB = midB;

				// adjust minR so we don't need so many iterations to find
				//	the solution
				if( midB * 0.75 > minR )
					minR = midB * 0.75;
			}
		}
	}

	double	midB = (hiB + loB) * 0.5;
	double	K = CalcK(radius, midB, apexLift);
	double	midA = midB * K;
	double	midAngle = startAngle + (loAngle + hiAngle) * 0.5;

	Vec2	v = Vec2::FromAngle(midAngle);
	Vec2	cen = c - v * (midA - (radius + apexLift));

	Ellipse	ell(cen, midA, midB, midAngle);

	m_pInfo[seg].m_set = true;
	m_pInfo[seg].m_ell = ell;
	m_pInfo[seg].m_apexLen = fabs((loAngle + hiAngle) * 0.5);
	m_pInfo[seg].m_apexDir = v;
	m_pInfo[seg].m_apexPt = c + v * (radius + apexLift);
	m_pInfo[seg].m_inBuffer = inBuffer;
	m_pInfo[seg].m_outBuffer = outBuffer;
	
	int		crossSeg;
	double	crossTheta;
	EllipseCrosses( ell, seg, outBuffer, isLeft, crossSeg, crossTheta );
	if( crossSeg == seg )
		crossSeg = (seg + 1) % NSeg();

	m_pInfo[seg].m_outCrossInnerSeg = crossSeg;
	m_pInfo[seg].m_outTheta = crossTheta;

	EllipseCrosses( ell, seg, inBuffer, !isLeft, crossSeg, crossTheta );
	m_pInfo[seg].m_inCrossSeg = crossSeg;
	m_pInfo[seg].m_inTheta = crossTheta;
}

/////////////////////////////////////////////////////////////////////////////

void	Sparky4Track::AdjustSBends( int apexesBetween )
{
	double	maxBuffer = m_track.width * cSBendMaxBuffer;
	bool	done = false;
	while( !done )
	{
		done = true;

		for( int i = 0; i < m_nApexes; i++ )
		{
			int		seg1 = m_pApexes[(i + m_nApexes - 1 - apexesBetween) % m_nApexes];
			int		seg2 = m_pApexes[i];

			if( GetRadius(seg1) * GetRadius(seg2) >= 0 )
				// bends are in same direction
				continue;

			int		outCrossSeg = m_pInfo[seg1].m_outCrossInnerSeg;
			int		inCrossSeg = m_pInfo[seg2].m_inCrossSeg;
			if( IsSegBefore(outCrossSeg, inCrossSeg, m_track.NSEG) )
				// don't try to move ellipses if they don't touch
				//	whilst still on the track
				continue;

			if( apexesBetween > 0 && outCrossSeg == inCrossSeg )
				continue;

			Ellipse&	seg1Ell = m_pInfo[seg1].m_ell;
			Ellipse&	seg2Ell = m_pInfo[seg2].m_ell;

			if( seg1Ell.EllipsesTouch(seg2Ell) )
			{
				bool	seg1IsLarger;
				double	theta1, theta2;
				if( seg1Ell.FindCrossingPoints(seg2Ell, theta1, theta2) )
				{
					double	s2th1 = seg2Ell.CalcTheta(seg1Ell.CalcPoint(theta1));
					double	s2th2 = seg2Ell.CalcTheta(seg1Ell.CalcPoint(theta2));

					// check that at least 1 of the crossing points is
					//	between the two bends (not both after or before)
					if( (theta1 <= cPi) != (s2th1 <= cPi) &&
						(theta2 <= cPi) != (s2th2 <= cPi) )
						continue;

					double	seg1Radius = min(seg1Ell.CalcRadius(theta1),
												seg1Ell.CalcRadius(theta2));
					double	seg2Radius = min(seg2Ell.CalcRadius(s2th1),
												seg2Ell.CalcRadius(s2th2));
					seg1IsLarger = seg1Radius > seg2Radius;
				}
				else
				{
					seg1IsLarger = m_pInfo[seg1].m_ell.YRadius() >
										m_pInfo[seg2].m_ell.YRadius();
				}

				if( seg1IsLarger )
				{
					if( m_pInfo[seg1].m_outBuffer < maxBuffer )
					{
						CalcApex( seg1, m_pInfo[seg1].m_inBuffer,
										m_pInfo[seg1].m_outBuffer + cSBendAdjust );
						done = false;
					}
					else if( m_pInfo[seg2].m_inBuffer < maxBuffer )
					{
						CalcApex( seg2, m_pInfo[seg2].m_inBuffer + cSBendAdjust,
										m_pInfo[seg2].m_outBuffer );
						done = false;
					}
				}
				else //if( m_pInfo[seg2].m_ell.YRadius() >
					//			m_pInfo[seg1].m_ell.YRadius() )
				{
					if( m_pInfo[seg2].m_inBuffer < maxBuffer )
					{
						CalcApex( seg2, m_pInfo[seg2].m_inBuffer + cSBendAdjust,
										m_pInfo[seg2].m_outBuffer );
						done = false;
					}
					else if( m_pInfo[seg1].m_outBuffer < maxBuffer )
					{
						CalcApex( seg1, m_pInfo[seg1].m_inBuffer,
										m_pInfo[seg1].m_outBuffer + cSBendAdjust );
						done = false;
					}
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////

bool	Sparky4Track::EllipseCrosses(
	const Ellipse& ell,			// ellipse to test
	int ellSeg,					// segment ellipse is apex in
	double buffer,				// distance from edge of track to use
	bool acw,					// direction to test

	int& crossSeg,				// segment where crossing point is found
	double& crossTheta )		// ellipse angle of crossing point
{
	bool	isLeft = m_track.lftwall[ellSeg].radius > 0;
	int		segDir = acw ^ isLeft ? -1 : 1;

	int		seg = ellSeg;

	do
	{
		segment*	pSegIn = 0;
		segment*	pSegOut = 0;
		double		offset = 0;

		if( isLeft )
		{
			pSegIn = &m_track.lftwall[seg];
			pSegOut = &m_track.rgtwall[seg];
			offset = -buffer;
		}
		else
		{
			pSegIn = &m_track.rgtwall[seg];
			pSegOut = &m_track.lftwall[seg];
			offset = buffer;
		}

		double	inTheta, outTheta;
		bool	crossIn = seg == ellSeg ? false :
							EllipseCrossesSegment(ell, acw, pSegIn, 0, inTheta);
		bool	crossOut = EllipseCrossesSegment(ell, acw, pSegOut, -offset, outTheta);

		if( crossIn && !crossOut )
		{
			crossSeg = seg;
			crossTheta = inTheta;
			return false;
		}
		else if( crossOut && !crossIn )
		{
			crossSeg = seg;
			crossTheta = outTheta;
			return true;
		}
		else if( crossOut )
		{
			// need to compare the theta values to see what happened
			crossSeg = seg;
			if( acw )
			{
				crossTheta = min(outTheta, inTheta);
				return outTheta < inTheta;
			}
			else
			{
				crossTheta = max(outTheta, inTheta);
				return inTheta < outTheta;
			}
		}

		seg = (seg + segDir + m_track.NSEG) % m_track.NSEG;
	}
	while( seg != ellSeg );

	// should never get here... but...
	crossTheta = acw ? cPi : -cPi;
	crossSeg = FindSegContainingPt(ell.CalcPoint(crossTheta));
	if( crossSeg < 0 )
		crossSeg = ellSeg;
	return false;
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

bool	CalcCrossDistance(
	const Vec2&	p,
	const Vec2& v,
	int startSeg,
	const track_desc& track,
	const segment* pIgnoreSeg,
	double buffer,

	int& crossSeg,
	double& crossDist,
	bool& crossLeftSide )
{
	// find closest pt where line crosses track
	crossDist = 10000000;
	crossSeg = -1;
	for( int j = 0; crossSeg < 0 && j < track.NSEG; j++ )
	{
		int			seg = (startSeg + j) % track.NSEG;

		segment*	pSeg2 = &track.lftwall[seg];
		segment*	pSeg3 = &track.rgtwall[seg];

		if( pSeg2->radius < 0 )
		{
			pSeg2 = &track.rgtwall[seg];
			pSeg3 = &track.lftwall[seg];
		}

		double	t = 0;
		if( pSeg2 != pIgnoreSeg &&
			LineCrossesSegment(p, v, pSeg2, buffer, t) )
		{
			if( t >= 0 && t < crossDist )
			{
				crossSeg = seg;
				crossDist = t;
				crossLeftSide = pSeg2->radius >= 0;
			}
		}

		if( pSeg3 != pIgnoreSeg &&
			LineCrossesSegment(p, v, pSeg3, buffer, t) )
		{
			if( t >= 0 && t < crossDist )
			{
				crossSeg = seg;
				crossDist = t;
				crossLeftSide = pSeg2->radius < 0;
			}
		}
	}

	return crossSeg >= 0;
}

// this one calcs the crossing distance for an arc
bool	CalcCrossDistance(
	const Vec2&	p,
	const Vec2& v,
	double rad,
	int startSeg,
	const track_desc& track,
	const segment* pIgnoreSeg,

	int& crossSeg,
	double& crossDist,
	bool& crossLeftSide )
{
	// work out centre point of arc
	Vec2	n = v.GetNormal();
	Vec2	cen = p + n * rad;
	double	startAngle = (n * -rad).GetAngle();

	// find closest pt arc line crosses track (if any)
	double	crossAngle = 2 * cPi;
	crossSeg = -1;
	for( int j = 0; crossSeg < 0 && j < track.NSEG; j++ )
	{
		int			seg = (startSeg + j) % track.NSEG;

		segment*	pSeg2 = &track.lftwall[seg];
		segment*	pSeg3 = &track.rgtwall[seg];

		if( pSeg2->radius < 0 )
		{
			pSeg2 = &track.rgtwall[seg];
			pSeg3 = &track.lftwall[seg];
		}

		double	angle = 0;
		if( pSeg2 != pIgnoreSeg &&
			ArcCrossesSegment(cen, rad, startAngle, pSeg2, angle) )
		{
			if( angle < crossAngle )
			{
				crossSeg = seg;
				crossAngle = angle;
				crossDist = angle * fabs(rad);
				crossLeftSide = pSeg2->radius >= 0;
			}
		}

		if( pSeg3 != pIgnoreSeg &&
			ArcCrossesSegment(cen, rad, startAngle, pSeg3, angle) )
		{
			if( angle < crossAngle )
			{
				crossSeg = seg;
				crossAngle = angle;
				crossDist = angle * fabs(rad);
				crossLeftSide = pSeg2->radius < 0;
			}
		}
	}

	return crossSeg >= 0;
}

/////////////////////////////////////////////////////////////////////////////

void	LookAhead(
	const situation& s,
	const track_desc& track,
	const Vec2& car,
	double carAngle,
	const Vec2& trackDir,

	double& aheadDist,
	Vec2& aheadDir,
	int& aheadSeg,
	int& aheadTanSeg,
	Vec2& aheadTanPt )
{
	// look ahead to see how far we can see in a straight line
	aheadDist = 0;
	aheadDir = Vec2::FromAngle(carAngle);

	int		max = min(10, track.NSEG);
	for( int i = 0; i < max; i++ )
	{
		int			tSeg = (s.seg_ID + i) % track.NSEG;
		segment*	pSeg = &track.lftwall[tSeg];
		if( pSeg->radius == 0 )
			continue;

		if( pSeg->radius < 0 )
			pSeg = &track.rgtwall[tSeg];

		// find tangent to curve
		Vec2	tanPt;
		double	rad = pSeg->radius + (pSeg->radius > 0 ? cApexLift : -cApexLift);
		if( !CalcTangentPt(	car, Vec2(pSeg->cen_x, pSeg->cen_y), rad, tanPt) )
			continue;

		// find direction vector to tangent point
		Vec2	v = (tanPt - car).GetUnit();

		// ignore tangents that take us backwards along the track
		if( v * trackDir < 0 )
			continue;

		// find closest pt where line crosses track
		double	crossDist;
		int		crossSeg;
		bool	crossLeftSide;
		CalcCrossDistance( car, v, s.seg_ID, track, pSeg, 0,//cApexLift,
							crossSeg, crossDist, crossLeftSide );
		if( crossSeg >= 0 && crossDist >= aheadDist )
		{
			int		s1 = tSeg >= s.seg_ID ? tSeg : tSeg + track.NSEG;
			int		s2 = crossSeg >= s.seg_ID ? crossSeg : crossSeg + track.NSEG;
			if( s1 <= s2 )
			{
				aheadDist = crossDist;
				aheadDir = v;
				aheadSeg = crossSeg;
				aheadTanSeg = tSeg;
				aheadTanPt = tanPt;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////

void	StayOnTrack(
	const situation& s,
	const track_desc& track,
	const Vec2& carPt,
	double carAngle,
	const Vec2& carDir,
	
	con_vec& control )
{
	if( s.to_lft < 0.1 )
	{
		control.vc = 60;
		control.alpha = s.vn > -10 ? -0.2 : 0;
		return;
	}
	else if( s.to_rgt < 0.1 )
	{
		control.vc = 60;
		control.alpha = s.vn < 10 ? 0.2 : 0;
		return;
	}

	// if we are already braking, then no need to interfere
	if( control.vc < s.v )
		return;

	// work out how far we would travel if we decided to brake to a standstill
	double	brakeDist = BrakingDistanceBend(s.v, 0);

	// look straight ahead and see how far we can go before we will hit a wall
	int		crossSeg;
	double	crossDist;
	bool	crossLeftSide;
	CalcCrossDistance( carPt, carDir, s.seg_ID, track, NULL, 0,
						crossSeg, crossDist, crossLeftSide );

	if( crossDist > brakeDist + 5 )
		// all ok
		return;

	// figure out which way we should be turning
	bool	turnLeft = !crossLeftSide;

	// if we do nothing we will hit a wall now.  so we will now see if we
	//	can avoid that by turning.
	double	maxRad = (4 + crossDist * crossDist) / 8;
	double	minRad = TurningRadius(s.v);
	double	maxCrossDist = crossDist;

	// if we are already turning, check to see if this is good enough
	//	not to crash.  if it is, we need not interfere here.
	if( control.alpha * (turnLeft ? 1 : -1) > 0 )
	{
		// we are attempting to turn in the correct direction, so
		//	lets work check if this will enable us to miss the
		//	wall.
		double	turnRad = s.v / control.alpha;
		bool	crossLeft;
		if( CalcCrossDistance(carPt, carDir, turnRad, s.seg_ID, track, NULL,
								crossSeg, crossDist, crossLeft) )
		{
			if( crossDist > brakeDist + 5 )
				// we don't need to take any action here
				return;
		}
		else
		{
			// radius is to small to hit anything, so is ok
			return;
		}
	}

	for( double r = maxRad; r >= minRad; r *= 0.95 )
	{
		double	rad = turnLeft ? r : -r;
		bool	arcCrossLeftSide;
		if( !CalcCrossDistance(carPt, carDir, rad, s.seg_ID, track, NULL,
								crossSeg, crossDist, arcCrossLeftSide) )
		{
			// radius is too small to hit track now
			// TODO: modify alpha more accurately
			control.alpha = turnLeft ? 1 : -1;
			control.vc = s.v;
			return;
		}

		if( crossDist > brakeDist + 5 )
		{
			// we can turn to miss wall, so modify alpha and return
			// TODO: modify alpha more accurately
			control.alpha = turnLeft ? 1 : -1;
			control.vc = s.v;
			return;
		}
	}

	// if we get here, we must have to brake and turn to miss the wall,
	//	or to minimise the damage
	control.alpha = turnLeft ? 1 : -1;
	control.vc = s.v * BRAKE_COEF_BEND;
}

/////////////////////////////////////////////////////////////////////////////

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
	bool	refuel = s.laps_to_go > 1 && s.fuel < fuelPerLap * 1.5;
	if( s.stage != QUALIFYING && (repairDamage || refuel) )
	{
		// work out the amount of fuel we want
		double fuelToEnd = min(MAX_FUEL, fuelPerLap * (s.laps_to_go + 1.1) );
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

bool	CalcTangentToApex(
	const Sparky4Track&	track,
	const Vec2&		carPt,
	int				carSeg,
	int				apexSeg,
	double			minDist,	// ignore if less than this
	
	Ellipse&		apexEll,
	Vec2&			newTanPt,
	Vec2&			newTanDir,
	double&			newTanDist )
{
	const segment*	pApexSeg = &track.LftWall(apexSeg);
	bool		apexIsLeft = pApexSeg->radius > 0;
	if( !apexIsLeft )
		pApexSeg = &track.RgtWall(apexSeg);

	// find elliptical path into apex segment
	double	delta = 0;
//	newTanPt = tanPt;
//	newTanDir = (distDir);
//	newTanDist = 0;

	const Info*	pApexInfo = &track.GetInfo(apexSeg);

	// figure out the largest value of maxB for the current position
	//	of the car.  e.g. if the car is inside the ellipse, then reduce
	//	the size of the ellipse.
	double	maxB = pApexInfo->m_ell.YRadius();
	double	minR = fabs(pApexSeg->radius) + cApexLift;

	if( track.GetInfo(apexSeg).m_replacedBySeg >= 0 )
	{
		// we are using an ellipse copied from the apex
		//	segment.  we need to use the min radius of the real apex
		//	segment
		int		seg = track.GetInfo(apexSeg).m_replacedBySeg;
		minR = fabs(track.GetRadius(seg)) + cApexLift;
	}

/*	if( pApexInfo->m_ell.IsPointInside(carPt) )
	{
		double	minB = minR;
		while( minB + 0.1 < maxB )
		{
			// calc ellipse
			double	b = (minB + maxB) * 0.5;
			double	K = CalcK(minR, b, cApexLift);
			double	a = b * K;
			Vec2	c = pApexInfo->m_apexPt - pApexInfo->m_apexDir * a;
			Ellipse	ell(c, a, b, pApexInfo->m_ell.Angle());

			if( ell.IsPointInside(carPt) )
			{
				// pt inside ellipse
				maxB = b;
			}
			else
			{
				// pt outside ellipse
				minB = b;
			}
		}

		maxB = minB;
	}
*/
	for( double	b = maxB; b >= minR; b *= 0.95 )
	{
		// calc ellipse
		double	K = CalcK(minR, b, cApexLift);
		double	a = b * K;
		Vec2	c = pApexInfo->m_apexPt - pApexInfo->m_apexDir * a;
		Ellipse	ell(c, a, b, pApexInfo->m_ell.Angle());

		// find tangent pt to this new ellipse
		Vec2	tanPt;
		double	tanTheta;
		if( !CalcTangentPt(carPt, ell, apexIsLeft, tanPt, tanTheta) )
			// ellipse is too large... car is inside
			continue;

		// make sure theta is within the allowed range for the ellipse
		tanTheta = NormaliseAngleLimit(tanTheta, 0);
		const Info&	info = track.GetInfo(apexSeg);
		double minTheta = min(info.m_inTheta, info.m_outTheta);
		double maxTheta = max(info.m_inTheta, info.m_outTheta);
		if( tanTheta < minTheta || tanTheta > maxTheta )
			continue;

		// now find whether this path stays on the track.
		//	1st, find the nearest crossing point.
		double	tanDist = (tanPt - carPt).GetLength();

		Vec2	v = (tanPt - carPt) / tanDist;

		int		crossSeg;
		double	crossDist;
		bool	crossLeftSide;
		if( CalcCrossDistance(carPt, v, carSeg, track.GetTrack(), NULL, 0,//cApexLift,
								crossSeg, crossDist, crossLeftSide) &&
//			crossDist > tanDist && crossDist > minDist )
			crossDist > tanDist )
		{
			// got a successful value of delta
			delta = b;
			newTanPt = tanPt;
			newTanDir = v;
			newTanDist = tanDist;
			apexEll = ell;
			break;
		}
	}

	return delta > 0;
}

/////////////////////////////////////////////////////////////////////////////

void	AvoidOtherCarsOld(
	const situation&	s,				// current situation
	const Situation&	mySituation,	// global situation values
	const Sparky4Track&		track,			// track

	double&				vc,				// modified speed
	int&				takeCareCounter,// if >0 then need to take care
	double&				passAng,		// pass angle
	bool&				needToBrake )	// true if need to brake
{
	bool		gotCar = false;
	rel_state	car;

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

		double	relDist = hypot(x, y);
		double	rv = hypot(vx, vy);
		if( vy < 0 )
			rv = -rv;

		double	cv = x * vx > 0 ? vy : rv;

		// check if we are currently colliding with another car
		// work out angle of other car in Sparky's local coord system,
		//	with car pointing along velocity vector.  therefore zero degrees
		//	is along velocity vector, which is the x axis.
		double	relAngle = atan2(-vx, s.v + vy);
		if( collide(x, y, relAngle) )
		{
			takeCareCounter = cTakeCareTime;
			if( y > 0 && vy < 0 )
			{
//				needToBrake = true;
				vc = s.v + vy - 0.1;
			}
		}

		// don't catch up to any cars going more than 30fps faster
		//	by the time we overtake
		double	brakeDist = BrakingDistanceBend(s.v, s.nearby[i].v + 40);
		if( y > CARLEN && brakeDist > y )
		{
			needToBrake = true;
		}

		// don't pass other cars going more than 0fps faster than them.
		//	only include cars within ±30° of current direction of travel.
		brakeDist = BrakingDistanceBend(s.v, s.v + cv + 15);
//		brakeDist = BrakingDistanceBend(s.v, s.v + cv );//+ 15);
		if( y > CARLEN && y < brakeDist && fabs(x) <= y / 2 + CARWID )
		{
			needToBrake = true;
		}

		if( y >= CARLEN * 0.75 && y <= CARLEN + 2 &&
			fabs(x) < CARWID * 1.2 && vy < 2 && vy > -5 )
		{
			// if we seem to be having a battle with the car in front
			//	where we are moving at about the same speed, and he
			//	is coming towards us in both x and y, and he is
			//	almost a car length ahead, then brake to get
			//	out of making lots of damage for no reason
			needToBrake = true;
		}

		// work out how long till we bump the car
		double	time = 100;	// start by assuming we miss the other car
		if( y > CARLEN )
		{
			// car in front... how long to catch it?
			if( vy < 0 )
			{
				time = (y - CARLEN) / -vy;

				// where do we hit it?
				double	relX = x + vx * time;
//				double	relX = x + (vx - velTanDir.x) * time;
				if( fabs(x) > cCarWid )
				{
					if( fabs(relX) > cCarWid && x * vx < 0 )
					{
						// not running straight into the rear of other
						//	car, but are we going to hit it on the side?
						time = (fabs(x) - cCarWid) / fabs(vx);

						// check to see if we will pass each other
						//	before we bump
						if( vy != 0 && fabs(y + time * vy) > CARLEN )
							time = 100;
					}
				}
				else
				{
					if( fabs(relX) <= cCarWid )
						time = 0;
				}
			}
			else
				time = 100;
		}
		else if( fabs(x) > cCarWid )
		{
			// car to side... how long to bump into it?
			if( x * vx < 0 )
			{
				time = (fabs(x) - cCarWid) / fabs(vx);

				// check to see if we will pass each other
				//	before we bump
				if( vy != 0 && fabs(y + time * vy) > CARLEN )
					time = 100;
			}
			else
				time = 100;
		}
		else
			// must be bumping into in right now
			time = 0;

		// calculate the closest distance of the centres of the cars
		double	closestDist = (x * vy + y * vx) / fabs(rv);

		// if the closest distance is more than the diagonal length of
		//	the cars then they must miss
		if( fabs(closestDist) > cCarDiag * 1.2 )
			// cars miss
			continue;

		// if more than 10 secs away then we should be able to
		//	safely ignore it! :-)
		if( time >= 10 )
			continue;

		// got a car ahead that we are catching up with
//		if( vy < 0 && y >= CARLEN )
		{
			// work out how long till we catch the car
//			double	time = (y - CARLEN) / -vy;

			// work out where the other car will be when we hit
			double	relX = x + vx * time;
//			double	relX = x + (vx - velTanDir.x) * time;

			// figure whether we will hit it
			if( fabs(relX) <= cCarWid )//&& y < CARLEN * 10 )
			{
				// looks like we've got a car in front coming
				//	towards us that we're not going to miss by
				//	letting it drift past
				// choose this car as one to attempt to dodge
				//	(if we have found one previously, we look to
				//	 see which we are likely to hit first, and choose
				//	 that one)
				if(	!gotCar || time < (car.rel_y - CARLEN) / -car.rel_ydot )
				{
					gotCar = true;
					car = s.nearby[i];
				}
			}

			// figure whether we will hit it
			if( fabs(relX) <= cCarWid && y > CARLEN * 0.75 &&
				vy < 0 )
			{
				// looks like we've got a car in front coming
				//	towards us that we're not going to miss by
				//	letting it drift past

				// try to brake with a carlength gap
				double	brakingDist = BrakingDistanceBend(s.v, s.nearby[i].v);
//				if( s.nearby[i].braking )
//					brakingDist *= 1.5;
				if( brakingDist > y - CARLEN * 1.5 )
				{
					if( s.v > car.v + 15 )
					{
						needToBrake = true;
//						brakeCoef = BRAKE_COEF_BEND;
					}
					else
						vc = car.v + 15;
				}
			}
		}
	}

	double	relVel = gotCar ? hypot(car.rel_xdot, car.rel_ydot) : 0;
	double	brakeToStopDist = BrakingDistanceBend(s.v, 0);
	if( gotCar )//&& (relVel > 5 || distAhead > 300) )
//	if( gotCar && (relVel > 21 || distAhead > 3 * brakeToStopDist) )
	{
		// try to change lanes to miss the nearest car

		double	x = car.rel_x;
		double	y = car.rel_y;
		double	vx = car.rel_xdot;
		double	vy = car.rel_ydot;

		double	avoidDist;
		if( y <= CARLEN * 2 )
			avoidDist = CARLEN * 2;
		else if( s.v > car.v )
			avoidDist = BrakingDistanceBend(s.v, car.v);
		else
			avoidDist = 0;

		if( y - CARLEN <= avoidDist )
		{
			// how long till we hit it
			double	time = 0;
			if( y > CARLEN )
			{
				// car in front... how long to catch it?
				time = (y - CARLEN) / -vy;
			}
			else if( fabs(x) > cCarWid )
			{
				// car to side... how long to bump into it?
				if( x < 0 )
					time = (x + cCarWid) / -vx;
				else
					time = (x - cCarWid) / -vx;
			}

			// where in x will it be then, relative to us?
			double	relX = x + (vx - mySituation.m_carDir.x) * time;

			// work out how "urgent" it is to dodge the other car,
			//	related to how far ahead it is
			double	urgency = max((10 * CARLEN - y) / (9 * CARLEN), 0);
			urgency = min(urgency, 1);

			// figure the angle we would be using for alpha if we didn't
			//	change it with the passAng
			double	actualAlpha = NormaliseAngleLimit(
									mySituation.m_distDir.GetAngle() -
										mySituation.m_carAngle, 0);

			// got car at side ... which side?
			if( relX < 0 )
			{
				// can we get to the right of the car?
				if( car.to_rgt > cCarWid && s.to_rgt > cCarWid * 1.5 )//|| y < CARLEN )
				{
					passAng = -0.05 * urgency;
				}
				else
				{
					passAng = 0.05 * urgency;
				}
			}
			else
			{
				// can we get to the left of the car?
				if( car.to_lft > cCarWid && s.to_lft > cCarWid * 1.5 )//|| y < CARLEN )
				{
					passAng = 0.05 * urgency;
				}
				else
				{
					passAng = -0.05 * urgency;
				}
			}

			// don't try to pass by turning the other way than
			//	we are already
			if( actualAlpha > -0.01 && passAng < 0 ||
				actualAlpha <  0.01 && passAng > 0 )
				passAng = 0;
		}
	}
}

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
	const Sparky4Track&		track,			// track

	double&				vc,				// modified speed
	int&				takeCareCounter,// if >0 then need to take care
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
			takeCareCounter = cTakeCareTime;
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

//			takeCareCounter = cTakeCareTime;

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

// end of namespace Spk4
//}

/////////////////////////////////////////////////////////////////////////////

//using namespace Spk4;

/////////////////////////////////////////////////////////////////////////////

con_vec Sparky4( situation& s )
{ 
	const char name[] = "Sparky4";	// This is the robot driver's name! 

	static Sparky4Track		track;		// track details

	static int			exitEllSeg = -1;
	static int			takeCareCounter = 0;	// if >0 then had need to take care

	static int			followingEllipseSeg = -1;	// if >= 0 then following ell

	con_vec	result;		// control vector
	double	alpha, vc;	// components of result 
	Vec2	tangent;	// track direction we desire to be moving in

	double	width = s.to_lft + s.to_rgt;
	double	halfWidth = width * 0.5;
	int		damage = 0;

	if( s.starting )
	{
		// first time only, copy name: 
		my_name_is(name);

		track_desc	trackDesc = get_track_description();

		exitEllSeg = -1;
		takeCareCounter = 0;
		followingEllipseSeg = -1;

		result.alpha = 0;
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
//		CORN_MYU *= 0.99;//1.01;

		if( s.stage == BEFORE || s.stage == FINISH )
		{
			// clean up
			track.Empty();
		}
		else if( track.NSeg() == 0 && trackDesc.NSEG != 0 )
		{
			track.Initialise( trackDesc );
		}

		return result; 
	} 

	// figure whether we need to refuel or repair damage (or both)
	PitControl( s, result, damage );

	// ask to see cars that are to the side, but slightly behind us
	s.side_vision = true;

	// inform team-mates where sparky is
//	SparkySeg = s.seg_ID;
//	SparkyToEnd = s.to_end;
//	SparkyToLeft = s.to_lft;

	// work out if we are to be in 'take care' mode for steering
	if( damage > 0 )
		takeCareCounter = cTakeCareTime;

	double	brakeCoef = BRAKE_COEF;

	// figure distance to end of this segment
	double	distToEnd = s.to_end;
	if( s.cur_rad != 0 )
		distToEnd *= fabs(s.cur_rad);

	// set up defaults... go straight in the middle
	double	curLine = width * 0.5;
	tangent = Vec2(0, 1);

	// work out current position of car, global coords
	segment*	pSeg = &track.RgtWall(s.seg_ID);
	Vec2		carPt;
	Vec2		trackDir;
	if( s.cur_rad == 0 )
	{
		Vec2	b(pSeg->beg_x, pSeg->beg_y);
		Vec2	e(pSeg->end_x, pSeg->end_y);
		Vec2	v = (e - b).GetUnit();
		Vec2	n = v.GetNormal();
		carPt = e - v * s.to_end + n * s.to_rgt;
		trackDir = v;
	}
	else
	{
		Vec2	c(pSeg->cen_x, pSeg->cen_y);
		Vec2	e(pSeg->end_x, pSeg->end_y);
		double	endAng = (e - c).GetAngle();
		double	carAng = pSeg->radius > 0 ? endAng - s.to_end : endAng + s.to_end;
		Vec2	v(Vec2::FromAngle(carAng));
		double	r = pSeg->radius;
		if( r > 0 )
			r = r - s.to_rgt;
		else
			r = (-r) + s.to_rgt;
		carPt = c + v * r;
		trackDir = pSeg->radius > 0 ? v.GetNormal() : -v.GetNormal();
	}

	// work out current direction of car, global angle
	double	carAngle = asin(s.vn / s.v);
	if( s.cur_rad == 0 )
	{
		Vec2	b(pSeg->beg_x, pSeg->beg_y);
		Vec2	e(pSeg->end_x, pSeg->end_y);
		carAngle += (e - b).GetAngle();
	}
	else if( s.cur_rad > 0 )
	{
		Vec2	c(pSeg->cen_x, pSeg->cen_y);
		carAngle += (carPt - c).GetAngle() + cPi_2;
	}
	else
	{
		Vec2	c(pSeg->cen_x, pSeg->cen_y);
		carAngle += (carPt - c).GetAngle() - cPi_2;
	}

	carAngle = NormaliseAngle(carAngle);
	Vec2	carDir = Vec2::FromAngle(carAngle);

	// look ahead to see how far we can see in a straight line
	double	distAhead = 0;
	Vec2	distDir(carDir);
	int		distSeg;
	int		tanSeg = -1;
	Vec2	tanPt;
	double	tanDist = 0;
	if( s.to_lft >= 0 && s.to_rgt >= 0 )
	{
		LookAhead( s, track.GetTrack(), carPt, carAngle, trackDir, distAhead,
					distDir, distSeg, tanSeg, tanPt );

		if( tanSeg < 0 )
		{
			result.alpha = 0;
			result.vc = s.v;
			return result;
		}
	}
	else
	{
		distAhead = 0;
		distSeg = s.seg_ID;
		tanSeg = s.seg_ID;
		tanPt = carPt + carDir;
	}

	int		apexSeg = tanSeg;

	bool	needToBrake = false;

	if( tanSeg >= 0 && s.to_lft >= 0 && s.to_rgt >= 0 )
	{
		tanDist = (tanPt - carPt).GetLength();

		// estimate speed we can go, related to how far we can see ahead
//		vc = distAhead / 3;
		vc = 300;

		// try to plot an elliptical approach to the corner we can see in
		//	the distance
/*		int		oldApexSeg = apexSeg;
		if( apexSeg != distSeg &&
			track.GetRadius(apexSeg) * track.GetRadius(distSeg) > 0 &&
			fabs(track.GetRadius(apexSeg)) > fabs(track.GetRadius(distSeg)) )
		{
			apexSeg = distSeg;
			int		nextSeg = (apexSeg + 1) % track.NSeg();
			if( track.GetRadius(apexSeg) * track.GetRadius(nextSeg) > 0 &&
				fabs(track.GetRadius(apexSeg)) > fabs(track.GetRadius(nextSeg)) )
			{
				apexSeg = nextSeg;
			}
		}
		else
		{
			int		nextSeg = (apexSeg + 1) % track.NSeg();
			if( apexSeg != distSeg &&
				track.GetRadius(apexSeg) * track.GetRadius(nextSeg) > 0 &&
				fabs(track.GetRadius(apexSeg)) > fabs(track.GetRadius(nextSeg)) )
			{
				apexSeg = nextSeg;
			}
			else if( apexSeg == distSeg )
			{
				if( track.GetRadius(apexSeg) * track.GetRadius(nextSeg) > 0 &&
					fabs(track.GetRadius(apexSeg)) > fabs(track.GetRadius(nextSeg)) )
				{
					apexSeg = nextSeg;
				}
			}
			else if( track.GetRadius(distSeg) == 0 )
			{
//				apexSeg = (distSeg + 1) % track.NSeg();
			}
		}
*/
		// find elliptical path into apex segment
		Ellipse apexEll;
		double	delta = 0;
		Vec2	newTanPt(tanPt);
		Vec2	newTanDir(distDir);
		double	newTanDist = 0;

		// first try the apex seg we think will give us the best line
		int		newApexSeg = apexSeg;
		bool	betterLine = false;
/*		bool	betterLine = CalcTangentToApex(track, carPt, s.seg_ID, apexSeg,
												tanDist, apexEll, newTanPt,
												newTanDir, newTanDist);
		if( !betterLine && apexSeg != oldApexSeg )
		{
			// now try the apex seg that was found when we originally
			//	looked ahead
			newApexSeg = oldApexSeg;
			betterLine = CalcTangentToApex(track, carPt, s.seg_ID, oldApexSeg,
											tanDist, apexEll, newTanPt,
											newTanDir, newTanDist);
		}
*/
		// perform search for a better apex corner
		newApexSeg = distSeg;
//		if( newApexSeg == apexSeg )
//			newApexSeg = (newApexSeg + 1) % track.NSeg();
		while( !track.GetInfo(newApexSeg).m_isApex )
			newApexSeg = (newApexSeg + 1) % track.NSeg();

		while( true )
		{
			if( track.GetRadius(newApexSeg) != 0 )
			{
				betterLine = CalcTangentToApex(track, carPt, s.seg_ID, newApexSeg,
												tanDist, apexEll, newTanPt,
												newTanDir, newTanDist);
				if( betterLine )
					break;
			}

			if( newApexSeg == tanSeg )
				break;

			newApexSeg = (newApexSeg - 1 + track.NSeg()) % track.NSeg();
		}

		// now check whether we want to be following the exit ellipse
		//	rather than the new apex ellipse

		bool	useExitEll = false;
		if( betterLine && newApexSeg != exitEllSeg && exitEllSeg >= 0 &&
			carDir * newTanDir < cCos1Deg )
		{
			double	theta = apexEll.CalcTheta(newTanPt);
			double	radius = apexEll.CalcRadius(theta);
			double	spd = CorneringSpeed(radius);
	//		double	spd = CorneringSpeed(arcRadius);
			double	dist = BrakingDistance(s.v, spd);
			if( carDir * distDir < cCos1Deg )
				dist = BrakingDistanceBend(s.v, spd);
//			if( newTanDist > dist * 5 && newTanDist > 700 )
			if( newTanDist > dist * 2 )
			{
				Ellipse	exitEll;
				Vec2	exitEllPt;
				Vec2	exitEllDir;
				double	exitEllDist;
				if( CalcTangentToApex(track, carPt, s.seg_ID, exitEllSeg,
										tanDist, exitEll,
										exitEllPt, exitEllDir, exitEllDist) )
				{
					// now check that exitEllDir is turning the car
					//	in the correct direction.
					Vec2	carNorm = carDir.GetNormal();
					double	dotApex = carNorm * newTanDir;
					double	dotExit = carNorm * exitEllDir;
					if( dotApex * dotExit > 0 &&		// same direction
						fabs(dotExit) < fabs(dotApex) )	// exit ang smaller
					{
						// ok to use exit ell
						useExitEll = true;
						newApexSeg = exitEllSeg;
						apexEll = exitEll;
						newTanPt = exitEllPt;
						newTanDir = exitEllDir;
						newTanDist = exitEllDist;
					}
				}
			}
		}

		// get the car to drive into the next corner following the better
		//	driving line that we have found
		if( betterLine )
		{
			// switch off ellipse following if apex has changed
			if( followingEllipseSeg >= 0 && followingEllipseSeg != newApexSeg )
				followingEllipseSeg = -1;

			// found a better line...
			distDir = newTanDir;
			tanDist = newTanDist;
			tanPt = newTanPt;

			double	theta = apexEll.CalcTheta(newTanPt);
			double	radius = apexEll.CalcRadius(theta);
			double	spd = CorneringSpeed(radius);
			double	dist = BrakingDistance(s.v, spd);
			if( carDir * newTanDir < cCos1Deg )
				dist = BrakingDistanceBend(s.v, spd);

			double	carTheta = apexEll.CalcTheta(carPt);
			double	ellRadius = apexEll.CalcRadius(carTheta);
			double	ellSpd = CorneringSpeed(ellRadius);
			Vec2	p = apexEll.CalcPoint(carTheta);
			double	distFromEll = (p - carPt).GetLength();

			// switch on ellipse following if we are close enough
			if( (distFromEll < 10 || distFromEll < 20 && newTanDist < 20) &&
				followingEllipseSeg != newApexSeg )
				followingEllipseSeg = newApexSeg;

			if( followingEllipseSeg == newApexSeg )//we are on ellipse??
			{
				exitEllSeg = newApexSeg;
				bool	pastApex =
						track.GetRadius(newApexSeg) > 0 && carTheta <= cPi_2 ||
						track.GetRadius(newApexSeg) < 0 && carTheta >= c3Pi_2;
//				if( useExitEll && distFromEll < 5 && pastApex )
//					vc = 300;
//				else

				vc = spd;

				if( pastApex )
				{
					if( distFromEll < 5 )
					{
						if( s.v + 1 < spd )
							vc = s.v + 50;
					}
					else if( distFromEll < 15 )
					{
						// accelerate a bit until back on line
						if( s.v + 1 < spd )
							vc = s.v + (15 - distFromEll) * 5;
					}
					else
					{
						// don't accelerate until back on line
						vc = s.v;
					}
				}

				if( s.v * brakeCoef > spd )
//				if( s.v > vc )
				{
					vc = s.v;
					needToBrake = true;
				}
//				vc = ellSpd;
//				vc = (spd + ellSpd) * 0.5;
			}

/*			if( distFromEll < 1 )
			{
				// we are attached to ellipse now...
				// use same steering/speed alg as Ellipser
				Vec2	ellTan = apexEll.CalcTangent(carTheta);
				if( !apexIsLeft )
					ellTan = -ellTan;
				double	tanAngle = ellTan.GetAngle();
				Vec2	tanNorm = ellTan.GetNormal();
								
				// steering servo from Ellipser, slightly modified :)
				double	servo = distFromEll / 10;
				if( (carPt - p) * tanNorm > 0 )//!apexIsLeft )
					servo = -servo;
				if( servo > 0.8 )
					servo = 0.8;
				else if( servo < -0.8 )
					servo = -0.8;
				double	carAngle = tanAngle + atan(servo);
				distDir = Vec2::FromAngle(carAngle);

				// now work out speed
				double	ellRadius = apexEll.CalcRadius(carTheta);
				double	ellSpd = CorneringSpeed(ellRadius);
				vc = ellSpd;
				if( vc + 1 < s.v )
				{
//					brakeCoef = BRAKE_COEF_BEND;
					needToBrake = true;
				}
			}
*/
			// remember the ellipse we just calculated
	//		pInfo[apexSeg].m_set = true;
	//		pInfo[apexSeg].m_ell = apexEll;

			apexSeg = newApexSeg;

			// if the apex ellipse we are using is not the full size version,
			//	this is probably because we are still around a bend from it, and
			//	we will eventually be able to see the whole ellipse.  this causes
			//	a problem with braking, as the distance to the tangent to the full
			//	size ellipse will be closer to the car.  thus we tend to overshoot
			//	on braking in this case.  we work out the distance from the car to
			//	the tangent point on the full size ellipse, and use this as the
			//	tangent distance to work out the braking.
			if( apexEll.XRadius() < track.GetInfo(apexSeg).m_ell.XRadius() &&
				apexEll.Angle() == track.GetInfo(apexSeg).m_ell.Angle() )
			{
				Vec2	tanPt;
				double	tanTheta;
				Info&	info = track.GetInfo(apexSeg);
				bool	isLeft = track.GetRadius(apexSeg) > 0;
				if( CalcTangentPt(carPt, info.m_ell, isLeft, tanPt, tanTheta) )
				{
					double	tanDist = (tanPt - carPt).GetLength();
					newTanDist = tanDist;
				}
			}

			if( dist + 3 > newTanDist )
//			if( dist + (s.v / 50) > newTanDist )
			{
				needToBrake = true;

//				if( dist - newTanDist > 20 && dist > newTanDist * 5 )
//					takeCareCounter = cTakeCareTime;
			}

//			if( BrakingDistance(s.v, 0) > distAhead )
//				needToBrake = true;
		}
		else
		{
	//		if( distDir * carDir < cCos45Deg )
	//			needToBrake = true;
/*
			if( pInfo[curApexSeg].m_set )
			{
				// try to follow the previous apex ellipse until we find a new one
				apexEll = pInfo[curApexSeg].m_ell;
				double	carTheta = apexEll.CalcTheta(carPt);
				Vec2	p = apexEll.CalcPoint(carTheta);
		//		if(	fabs(arcRadius - (carPt - arcCentre).GetLength()) < 10 )
				double	distFromEll = (p - carPt).GetLength();
				if( (carTheta < cPi_2 || carTheta > cPi) &&
					distFromEll < 20 ) // are we on ellipse??
				{
					distDir = apexEll.CalcTangent(carTheta);
					if( track.lftwall[curApexSeg].radius < 0 )
						distDir = -distDir;
					double	radius = apexEll.CalcRadius(carTheta);
					double	spd = CorneringSpeed(radius);
					if( carTheta < cPi_2 )
						//we are exiting ellipse
						vc = s.v + 50;
					else
						// we are entering ellipse
						vc = spd;
				}
			}
			else
*/			{
//				if( BrakingDistance(s.v, 0) > distAhead - width * 0.7 )
//				if( BrakingDistance(s.v, 0) > distAhead )
//					needToBrake = true;
			}
		}
	}

	// ok, now we look into the distance, each corner in turn, and check
	//	how fast we are allowed to be going when we pass the apex.  if we
	//	are going too fast, we need to slow down (the apex we need to
	//	slow down for may not be visible, so we need to do it this way)
	double	dist = s.to_end;
	bool	pastDist = false;
	if( s.cur_rad != 0 )
		dist *= fabs(s.cur_rad);
	int		startSeg = (s.seg_ID + 1) % track.NSeg();

	int		farSeg = apexSeg;
	if( IsSegBefore(apexSeg, s.seg_ID, track.NSeg()) )
	{
		// we are following a previous seg
		farSeg = s.seg_ID;
	}
/*	if( apexSeg >= s.seg_ID && distSeg >= s.seg_ID && apexSeg > distSeg )
	{
		farSeg = apexSeg;
	}
	else if( apexSeg < s.seg_ID && distSeg >= s.seg_ID )
	{
		farSeg = apexSeg;
	}
*/
	if( s.seg_ID != farSeg && startSeg != farSeg )
	{
		do
		{
			double	segRadius = fabs(track.GetRadius(startSeg));
			double	len = track.GetLength(startSeg);
			if( segRadius != 0 )
				len *= segRadius;
			dist += len;
			startSeg = (startSeg + 1) % track.NSeg();
		}
		while( startSeg != farSeg );
	}

	{
		double	segRadius = fabs(track.GetRadius(startSeg));
		double	len = track.GetLength(startSeg);
		if( segRadius != 0 )
			len *= segRadius;
		dist += len;
		startSeg = (startSeg + 1) % track.NSeg();
	}

	for( int i = 0; i < 8; i++ )
	{
		int		seg = (startSeg + i) % track.NSeg();
		double	segRadius = fabs(track.GetRadius(seg));
		Info&	info = track.GetInfo(seg);
		if( info.m_set && info.m_replacedBySeg < 0 )
		{
			// angle of ellipse around seg...
			double	distToApex = dist + info.m_apexLen * segRadius;
//			double	distToApex = dist + info.m_apexLen * segRadius / 2;
//			double	distToApex = dist;
			double	apexSpeed = CorneringSpeed(info.m_ell.CalcRadius(0));
//			double	apexSpeed = CorneringSpeed(segRadius + width * 0.5);

			double	brakeDist = BrakingDistanceBend(s.v, apexSpeed);
			if( brakeDist > distToApex )
				needToBrake = true;
		}

		if( segRadius == 0 )
			dist += track.GetLength(seg);
		else
			dist += track.GetLength(seg) * segRadius;
	}

	// see if we are going to hit another car
	Situation	mySituation;
	mySituation.m_carPt = carPt;
	mySituation.m_carDir = carDir;
	mySituation.m_carAngle = carAngle;
	mySituation.m_distDir = distDir;
	mySituation.m_tanDist = tanDist;
	mySituation.m_nextTurnIsLeft = track.GetRadius(apexSeg) > 0;
	double	passAng = 0;
//	AvoidOtherCarsOld( s, mySituation, track, vc, takeCareCounter, passAng, needToBrake );
	AvoidOtherCars( s, mySituation, track, vc, takeCareCounter, passAng, needToBrake );

	// figure out the steering now
	double	lineAngle = distDir.GetAngle();
	alpha = NormaliseAngleLimit(lineAngle + passAng - carAngle, 0);
/*
	if( dRgt > distAhead )
	{
		alpha -= min((s.to_rgt - 10) / 70, 0.025);
	}
	else if( dLft > distAhead )
	{
		alpha += min((s.to_lft - 10) / 70, 0.025);
	}
*/
	bool	offTrack = false;
	if( s.to_lft < cTrackBuffer )
	{
		if( s.to_lft < 0 )
		{
			offTrack = true;
			if( s.vn > 40 )
			{
				vc = s.v;
				alpha = -0.01;
			}
			else
			{
				vc = 40;
				if( s.v < 40 )
					vc = s.v + 50;
				if( s.vn > -15 )
					alpha = -0.05;
			}
		}
		else //if( s.vn > 0 )
		{
//			alpha = s.to_lft - cTrackBuffer;//-max(s.vn, 1);
			passAng = 0;
			takeCareCounter = cTakeCareTime;
		}
	}
	else if( s.to_rgt < cTrackBuffer )
	{
		if( s.to_rgt < 0 )
		{
			offTrack = true;
			if( s.vn < -40 )
			{
				vc = s.v;
				alpha = 0.01;
			}
			else
			{
				vc = 40;
				if( s.v < 40 )
					vc = s.v + 50;
				if( s.vn < 15 )
					alpha = 0.05;
			}
		}
		else //if( s.vn < 0 )
		{
//			alpha = cTrackBuffer - s.to_rgt;//max(-s.vn, 1);
			passAng = 0;
			takeCareCounter = cTakeCareTime;
		}
	}

	// need to take care when steering?
	bool	takeCare = takeCareCounter > 0;
	if( takeCare )
		takeCareCounter--;

	// adjust alpha to be more smooth, depending on our current speed,
	//	so we don't try and turn too sharply, and thus skid and lose
	//	acceleration
/*	if( !needToBrake && !takeCare )
	{
		double	maxAlpha = 0.7 * fabs(s.v / TurningRadius(s.v));
//		double	f = 1.525;
//		double	offset = (f - 1.3) * 70;
//		double	maxAlpha = fabs(s.v / TurningRadius(s.v * f - offset));
//		maxAlpha = min(maxAlpha, 0.7 * fabs(s.v / TurningRadius(s.v)));
		if( alpha < -maxAlpha )
			alpha = -maxAlpha;
		else if( alpha > maxAlpha )
			alpha = maxAlpha;
	}
*/
	static double maxSlip = 0;
	Vec2	slipVec = Vec2(s.v, 0) - Vec2::FromAngle(alpha) * s.vc;
	double	slip = slipVec.GetLength();
	double	myu = friction(slip);
	if( slip > maxSlip )
		maxSlip = slip;

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

	if( offTrack )
	{
/*		if( s.v < 20 )
			vc = s.v + 50;
		else if( s.v > 21 )
			vc = s.v * 0.8;
		else
			s.v = 20;
*/
	}
	else if( vc < 40 )
	{
		vc = 40;
	}
	result.vc = vc;
//	result.vc = min(result.vc, 40);
	result.alpha = alpha; 

	// try to make sure car stays on track
//	if( takeCare )
//		StayOnTrack( s, track.GetTrack(), carPt, carAngle, carDir, result );

	return result; 
} 
