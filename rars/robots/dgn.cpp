// ------------------------------------------------
//
// Written by Della Gioia Nicola
// March 2000, San Giuliano Milanese, Milan, Italy
//
// E-Mail Nicola.DellaGioia@netit.Alcatel.it
//
// ------------------------------------------------

#include <string.h>
#include <math.h>
#include "car.h"
#include "track.h"

const double RISK_ALPHA_1=2;		// less improve 1' curve segment direction
const double RISK_ALPHA_2=4;		// less improve 2' curve segment direction
const double RISK_ALPHA_3=8;		// less improve 3' curve segment direction
const double RISK_V=2.9;			// less improve exit curve velocity
const double TOP_SPEED_CONST=38;	// factor to calculate the ideal curve speed
const double BRAKE_CONST=26;		// brake constant factor,- const => + brake distance
const double CURVE_ANT=135;			// factor to calculate how far we start to enter in the fast curves
const double FUEL_USE=2537;			// medium use of fuel for lap
const double QUAL_FUEL=25;			// starting fuel for qualifying
const double DAMAGE_FOR_MILE=0.125;	// average value of damage for mile
const unsigned long DAMAGE_LIMIT=27000;	// limit of damage before a sure pit stop
const int    ACC=50;				// car acceleration
const double CURVE_2=1.9;			// speed factor for curve of type 2
const double CURVE_3=1.5;			// speed factor for curve of type 3
const double CURVE_4=1.3;			// speed factor for curve of type 4
const double CURVE_5=0.8;			// speed factor for curve of type 5
const double DOUBLE_CURVE_INCREMENT=1.5; // speed increment for the double curve
const double DOUBLE_CURVE_DECREMENT=1.1; // speed decrement for the double curve
const double DOUBLE_CURVE_DECREMENT_EXIT=0.6; // speed decrement for the exit of double curve
const double RAD_LIMIT=1200;		// max cur_rad after the cur is like a straight
const double LEN_LIMIT=0.08;		// min cur_len after the cur is like a straight
const double BRAKE_CORRECTION=1.9;	// correction of the brake limit if the car is in a start wrong curve position
const double SPEED_CORRECTION=0.6;	// correction of the speed limit if the car is in a start wrong curve position
const double SLOW_FACTOR=1200;		// slow down factor for long curve

static double track_lenght=0;		// track lenght
static int	  brake=0;				// 1 => brake phase
static int	  sterring=0;			// 1 => the car is sterring
static int	  double_curve=0;		// >0 => the car is in the second part of a double curve with speed problem
static int    to_start=50;			// during the fist meters there is no sterring


// understand the curve type
int curve_type(double cur_rad,
			   double cur_len)
{
	// cur_rad,	radius of inner track wall, 0 = straight, minus = right
	// cur_len,	length of current track segment (angle if curve) 

	if((cur_rad==0) ||
	   (fabs(cur_rad)>RAD_LIMIT) ||
	   (cur_len<LEN_LIMIT))
		return 0; // straight

	// Oval management
	if((fabs(cur_rad)>290) &&
	   (fabs(cur_rad)<310) && 
	   (cur_len>3.0) && 
	   (cur_len<3.2))
		return 1; // Oval2 Curves

	if((fabs(cur_rad)>305) &&
	   (fabs(cur_rad)<325) && 
	   (cur_len>1.5) && 
	   (cur_len<1.7))
		return 1; // Loudon Curves

	if((fabs(cur_rad)>810) &&
	   (fabs(cur_rad)<830) && 
	   (cur_len>1.5) && 
	   (cur_len<1.7))
		return 1; // Indy500 Curves

	if((fabs(cur_rad)>900) &&
	   (fabs(cur_rad)<920) && 
	   (cur_len>1.9) && 
	   (cur_len<2.1))
		return 4; // Michigan Curves

	if((fabs(cur_rad)>380) &&
	   (fabs(cur_rad)<400) && 
	   (cur_len>3.0) && 
	   (cur_len<3.2))
		return 1; // Mlwaukee Curves

	if((fabs(cur_rad)>415) &&
	   (fabs(cur_rad)<455) && 
	   (cur_len>2.5) && 
	   (cur_len<2.9))
		return 1; // Nazareth, Phoenix last Curves

	if((fabs(cur_rad)>285) &&
	   (fabs(cur_rad)<305) && 
	   (cur_len>2.3) && 
	   (cur_len<2.5))
		return 1; // Phoenix 1' Curves

	if((fabs(cur_rad)>230) &&
	   (fabs(cur_rad)<250) && 
	   (cur_len>2.4) && 
	   (cur_len<2.6))
		return 1; // Pocono 1' Curves

	if((fabs(cur_rad)>245) &&
	   (fabs(cur_rad)<265) && 
	   (cur_len>1.3) && 
	   (cur_len<1.5))
		return 3; // Pocono 2' Curves

	if((fabs(cur_rad)>320) &&
	   (fabs(cur_rad)<340) && 
	   (cur_len>2.2) && 
	   (cur_len<2.4))
		return 1; // Pocono 3' Curves

	if((fabs(cur_rad)>160) &&
	   (fabs(cur_rad)<180) && 
	   (cur_len>1.7) && 
	   (cur_len<2.1))
		return 1; // Silverstone 6', 8' Curves

	if(cur_len>3.4)
		return 1; //very long curve

	double top_speed=sqrt(fabs(cur_rad*TOP_SPEED_CONST));

	if(top_speed<50)
		return 5; // Slow Curves

	if(top_speed<90)
	{
		double curve_factor=fabs(cur_rad)*cur_len;

		if(curve_factor<140)
			return 3; // Fast Curves

		if(cur_len>1.0)
			return 5; // Slow Curves

		return 4; // Medium Curves
	}

	if(top_speed<130)
	{
		if(cur_len>0.3)
			return 4;

		return 3; // Fast Curves
	}

	return 2; // Very Fast Curves
}

// return 1 if the two curves are in the same direction
int same_direction(double cur_rad, 
				   double next_cur_rad)
{
	// cur_rad,		radius of inner track wall, 0 = straight, minus = right
	// next_cur_rad, radius of the segment after that one 

	if((cur_rad>0 && next_cur_rad>0) || (cur_rad<0 && next_cur_rad<0))
		return 1;

	return 0;
}

// calculate the actual segment lenght
double segment_lenght(double to_end, 
					  double cur_rad, 
					  double track_width)
{
	// to_end,		how far to end of current track seg. (angle or feet)
	// cur_rad,		radius of inner track wall, 0 = straight, minus = right
	// track_width, current size of the track

	double l=0;

	if (cur_rad==0.0)
		l=to_end; // we are in a straight
	else    
		l=to_end * fabs( cur_rad ); 

	// in the last piece of the curve the lenght became the track width
	if (l<track_width)
		l=track_width;

	return l;
}

// calculate if it's better start to sterring 
int start_sterring(double to_end,
				   double next_cur_rad,
				   double next_cur_len,
				   double left, 
				   double right)
{
	// to_end,		 how far to end of current track seg. (angle or feet)
	// next_cur_rad, radius of the segment after that one 
	// next_cur_len, length of the segment after that one
	// left,		distance to left wall 
	// right,		distance to right wall

	// don't start to curve if to much near to the track border
	if((next_cur_rad>0) && (left<2))
		return 0;

	if((next_cur_rad<0) && (right<2))
		return 0;

	int type_of_curve=curve_type(fabs(next_cur_rad), next_cur_len);
	int result=0;;

	if(type_of_curve==5)
	{
		// Slow Curves of different types
		// High "factor" value means start to curve later
		double factor;

		if((fabs(next_cur_rad*next_cur_len)<100) && (next_cur_len<2))
			factor=1.2;
		else
			factor=1.4;

		if(to_end<(CURVE_ANT/factor))
			result=1;
	}

	if(type_of_curve==4)
	{
		// Medium Curves of different types
		// High "factor" value means start to curve later
		double factor=1.1;

		if(fabs(next_cur_len)>1.7)
			factor=1.2; // slower...

		if(to_end<(CURVE_ANT/factor)) // low value of constant means start before to sterring
			result=1;
	}

	if(type_of_curve==3)
	{
		// Fast Curves 
		// High "factor" value means start to curve later
		double factor=1.2;

		if(to_end<(CURVE_ANT/factor))
			result=1;
	}

	if(type_of_curve==2)
	{
		// Fast Curves of different types
		// High "factor" value means start to curve later
		double factor=1.0;

		if(fabs(next_cur_rad)>400)
			factor=1.1;

		if(to_end<(CURVE_ANT/factor))
			result=1;
	}

	if(type_of_curve==1)
	{
		// Oval Curves
		if(to_end<CURVE_ANT)
			result=1;
	}

	sterring=result;

	return result;
}

// calculate the top speed for a curve
double curve_top_speed(double cur_rad,
					   double cur_len, 
				       double next_cur_rad, 
				       double next_cur_len,
					   double left,
					   double right)
{
	// cur_rad,		 radius of inner track wall, 0 = straight, minus = right
	// cur_len,		 length of current track segment (angle if curve) 
	// next_cur_rad, length of the segment after that one
	// next_cur_len, radius of the segment after that one
	// left,		distance to left wall 
	// right,		distance to right wall

	// default for oval curves
	double top_speed =sqrt(fabs(0.97*cur_rad*TOP_SPEED_CONST));
	double curve_factor=curve_type(cur_rad, cur_len);
	double next_curve_factor=curve_type(next_cur_rad, next_cur_len);

	// management of long curve
	double curve_l=fabs(cur_rad*cur_len);
	double slow_factor=0;

	double too_large=0;
	double limit=fabs(right+left)/3;

	if((cur_rad>0) && (right<limit))
		too_large=left;

	if((cur_rad<0) && (left<limit))
		too_large=right;

	if((curve_l>100) && (curve_factor>1))
	{
		slow_factor=curve_l/SLOW_FACTOR;

		if(too_large!=0)
			slow_factor=slow_factor*1.3;
	}

	// we are in a curve with top speed of:
	if (curve_factor==5)
	{
		// Slow curves of different types...
		double factor=CURVE_5;

		if(slow_factor>0)
			factor=CURVE_5-slow_factor;

		top_speed=sqrt(factor*fabs(cur_rad*TOP_SPEED_CONST));
	}

	if (curve_factor==4)
	{
		// Medium curves of different types...
		double factor=CURVE_4;

		if(slow_factor>0)
			factor=CURVE_4-slow_factor;

		top_speed=sqrt(fabs(factor*cur_rad*TOP_SPEED_CONST));
	}

	if (curve_factor==3)
	{
		// Fast curves of different types...
		double factor=CURVE_3;

		if(slow_factor>0)
			factor=CURVE_3-slow_factor;

		top_speed=sqrt(fabs(factor*cur_rad*TOP_SPEED_CONST));
	}
	
	if (curve_factor==2)
	{
		// Very Fast curves...
		double factor=CURVE_2;

		if(slow_factor>0)
			factor=CURVE_2-slow_factor;

		top_speed=sqrt(fabs(factor*cur_rad*TOP_SPEED_CONST));
	}

	// if the car is in a double curve faster in the exit so the top speed should be high
	if((curve_factor>=next_curve_factor) && 
	   (!same_direction(cur_rad, next_cur_rad)) &&
	   (next_curve_factor>0) &&
	   (curve_factor>0))
		top_speed=top_speed*DOUBLE_CURVE_INCREMENT;

	// if the car is in a double curve faster in the exit so the top speed should be high
	if((curve_factor<next_curve_factor) && 
	   (!same_direction(cur_rad, next_cur_rad)) &&
	   (next_curve_factor>0) &&
	   (curve_factor>0))
		top_speed=top_speed*DOUBLE_CURVE_DECREMENT;
	
	if((curve_factor<next_curve_factor) && 
	   (same_direction(cur_rad, next_cur_rad)) &&
	   (next_curve_factor>0) &&
	   (curve_factor>0))
		top_speed=top_speed/((next_curve_factor-curve_factor));

	if((double_curve>=1) &&
	   (next_curve_factor==0))
		top_speed=top_speed*DOUBLE_CURVE_DECREMENT_EXIT;

	if((double_curve>=2) &&
	   (next_curve_factor==0))
		top_speed=top_speed*DOUBLE_CURVE_DECREMENT_EXIT;

	return top_speed;
}

// calculate the entering speed for the next curve
double start_top_speed(double next_cur_rad,
				       double next_cur_len,
				       double cur_rad,
				       double cur_len)
{
	// next_cur_rad, radius of the segment after that one
	// next_cur_len, length of the segment after that one
	// cur_rad,		 radius of inner track wall, 0 = straight, minus = right
	// cur_len,		 length of current track segment (angle if curve) 

	double next_curve_factor=curve_type(next_cur_rad, next_cur_len);
	double curve_factor=curve_type(cur_rad, cur_len);

	// management of long curve
	double curve_l=fabs(cur_rad*cur_len);
	double slow_factor=0;

	if((curve_l>100) && (curve_factor>1))
	{
		slow_factor=curve_l/SLOW_FACTOR;
	}

	// default for oval curves
	double start_speed=sqrt(1.16*fabs(next_cur_rad*TOP_SPEED_CONST));

	if(next_curve_factor==5)
	{
		// Slow curves of different types...
		double factor=CURVE_5;

		if(slow_factor>0)
			factor=CURVE_5-slow_factor;

		start_speed=sqrt(factor*fabs(next_cur_rad*TOP_SPEED_CONST));
	}

	if(next_curve_factor==4)
	{
		// Medium curves of different types...
		double factor=CURVE_4; 

		if(slow_factor>0)
			factor=CURVE_4-slow_factor;

		start_speed=sqrt(factor*fabs(next_cur_rad*TOP_SPEED_CONST));
	}

	if(next_curve_factor==3)
	{
		// Fast curves of different types...
		double factor=CURVE_3; 

		if(slow_factor>0)
			factor=CURVE_3-slow_factor;

		start_speed=sqrt(factor*fabs(next_cur_rad*TOP_SPEED_CONST));
	}
		
	if(next_curve_factor==2)
	{
		// Very Fast curves...
		double factor=CURVE_2;

		if(slow_factor>0)
			factor=CURVE_2-slow_factor;

		start_speed=sqrt(factor*fabs(next_cur_rad*TOP_SPEED_CONST));
	}

	// if the car is in a double curve faster in the exit so the top speed should be high
	if((curve_factor<next_curve_factor) && 
	   (next_curve_factor>0) &&
	   (curve_factor>0))
		start_speed=start_speed*DOUBLE_CURVE_DECREMENT;

	if((curve_factor<next_curve_factor) &&
	   (same_direction(cur_rad, next_cur_rad)) &&
	   (curve_factor>0))
		start_speed=start_speed*DOUBLE_CURVE_DECREMENT_EXIT;

	return start_speed;
	
}

// calculate the brake limit
double brake_limit_value(double next_cur_rad,
						 double next_cur_len,
						 double v,
						 double cur_rad,
						 double cur_len)
{
	// next_cur_rad, radius of the segment after that one
	// next_cur_len, length of the segment after that one
	// v,			 the speed of the car, feet per second 
	// cur_rad,		 radius of inner track wall, 0 = straight, minus = right
	// cur_len,		 length of current track segment (angle if curve) 

	double next_curve_factor=curve_type(next_cur_rad, next_cur_len);
	double start_speed=start_top_speed(next_cur_rad, next_cur_len, cur_rad, cur_len);

	// default for oval curves
	double result=fabs((v+(0.5*(start_speed-v)))*(start_speed-v)/-(BRAKE_CONST+0.025*v));

	if(next_curve_factor==5)
	{
		// Slow curves of different types...
		// high "factor" means brake later
		double factor=2.1; 

		if((fabs(next_cur_rad*next_cur_len)<100) && (next_cur_len<=2))
			factor=2.8; 

		if((fabs(next_cur_rad*next_cur_len)<100) && (next_cur_len>2))
			factor=3.0;

		result=fabs((v+(0.5*(start_speed-v)))*(start_speed-v)/-((BRAKE_CONST+factor)+0.025*v));
	}

	if(next_curve_factor==4)
	{
		// Medium curves...
		result=fabs((v+(0.5*(start_speed-v)))*(start_speed-v)/-(BRAKE_CONST+0.025*v));
	}

	if(next_curve_factor==3)
	{
		// Fast curves...
		result=fabs((v+(0.5*(start_speed-v)))*(start_speed-v)/-(BRAKE_CONST+0.025*v));
	}
		
	if(next_curve_factor==2)
	{
		// Very Fast curves...
		result=fabs((v+(0.5*(start_speed-v)))*(start_speed-v)/-(BRAKE_CONST+0.025*v));
	}
	
	return result;
	
}

// calculate the ideal car speed
double ideal_speed(double v, 
				   double cur_rad, 
				   double to_end, 
				   double next_cur_len, 
				   double next_cur_rad, 
				   double cur_len, 
				   double to_end_pure,
				   double left, 
				   double right)
{
	// v,			 the speed of the car, feet per second 
	// cur_rad,		 radius of inner track wall, 0 = straight, minus = right
	// to_end,		 result of segment_lenght(...) function
	// next_cur_len, length of the segment after that one
	// next_cur_rad, radius of the segment after that one
	// cur_len,		 length of current track segment (angle if curve) 
	// to_end_pure,	 how far to end of current track seg. (angle or feet)
	// left,		distance to left wall 
	// right,		distance to right wall
 
	double speed=v;
	double brake_limit;
	double start_speed;

	if((cur_rad==0) ||
	   (fabs(cur_rad)>RAD_LIMIT) ||
	   (cur_len<LEN_LIMIT))
	{
		// we are in a straight 
		speed=v+ACC;

		// and also the next segment is a straight
		if((next_cur_rad==0) ||
		   (fabs(next_cur_rad)>RAD_LIMIT) ||
	       (next_cur_len<LEN_LIMIT))
			return speed;
	}
	else
	{
		brake = 0;

		// Determinate the top speed for the actual curve
		double top_speed =curve_top_speed(cur_rad,
										  cur_len, 
										  next_cur_rad, 
										  next_cur_len,
										  left,
										  right);

		// increase speed if:
		//  - we are under the top_speed feet/sec
		//  - we are near at the curve end
		if((v<top_speed) || (to_end_pure<(cur_len/RISK_V)))
			speed=v+ACC;
			
	} // end is a curve

	// the next segment is a curve
	if(next_cur_rad!=0)
	{
		// Next curve managment:		
		brake_limit=brake_limit_value(next_cur_rad, next_cur_len, v, cur_rad, cur_len);
		start_speed=start_top_speed(next_cur_rad, next_cur_len, cur_rad, cur_len);

		if((cur_rad==0) || (fabs(cur_rad)>RAD_LIMIT))
		{
			if((to_end<brake_limit) && (v>start_speed))
				brake = 1;
		}
		else
		{
			if((to_end<(brake_limit*1.8)) && (v>start_speed))
				brake = 1;
		}

		// we have to brake if:
		//  - we are near the end of the actual segment
		//  - we don't break hard in the last meters (ABS)
		//  - we are to much faster
		if(brake==1)
		{
			// improve braking only for not oval curves
			double curve_factor=curve_type(cur_rad, cur_len);

			if((sterring==0) && (curve_factor!=1) && (v>start_speed))
				return 0;

			if(cur_rad==0)
				speed=0.95*v;
			else
				speed=0.96*v;
		}

		if(v<=start_speed)
		{
			speed=v+ACC; 
			brake=0;
		}
	}

	return speed;
}

// calculate the ideal sterring angle
double ideal_sterring(double alpha, 
					  double left, 
					  double right, 
					  double distance_to_end, 
					  double to_end, 
					  double cur_rad, 
					  double cur_len, 
					  double next_rad,
					  double next_len,
					  double speed)
{
	// alpha,		actual car direction [ asin(vn/v) ]
	// left,		distance to left wall 
	// right,		distance to right wall
	// distance_to_end, result of segment_lenght(...) function
	// to_end,		how far to end of current track seg. (angle or feet)
	// cur_rad,		radius of inner track wall, 0 = straight, minus = right
	// cur_len,		length of current track segment (angle if curve) 
	// next_rad,	radius of the segment after that one  
	// next_len,	length of the segment after that one
	// speed,		actual car speed

	double alpha_result=0;
	double correction;
	double start_corr;

	int curve_factor=curve_type(cur_rad, cur_len);
	int next_curve_factor=curve_type(next_rad, next_len);

	if( cur_rad==0 )
	{
		// The Car is in a straight
		double_curve=0;

		if( next_rad==0 )
		{
			// ...also the next segment is a straight, stay in the middle
			alpha_result=(atan((left-right)/(2*distance_to_end)))-alpha;
		}
		else
		{
			// defaul for oval curves

			// this value define how fast the car prepare the next curve
			correction=1.5;
			// this value define how fast the car start   the next curve
			start_corr=12;

			if( (next_curve_factor==1) && ((cur_len-distance_to_end)>15) )
			{
				correction=2.9;
			}
			else
			{
				correction=1.2;

				// double from_start_segment=cur_len-distance_to_end;

				if(distance_to_end>30)
					correction=1.8;

				if(distance_to_end>50)
					correction=2.5;
			}

			// How fast start the next curve
			if( next_curve_factor==5 )
				start_corr = 6;		// Slow curves

			if( next_curve_factor==4 )
				start_corr = 3;		// Medium curves

			if( next_curve_factor==3 )
				start_corr = 3;		// Fast curves

			if( next_curve_factor==2 )
				start_corr = 2;		// Very Fast curves

			// management of out track
			if((left<=0) || (right<=0))
				correction=2;

			if( start_sterring(to_end, next_rad, next_len, left, right) )
			{
				// we start the curve in the straight
				if( next_rad>0 )
					alpha_result=(atan((start_corr*left-right)/(2*distance_to_end)))-alpha;
				else
					alpha_result=(atan((left-start_corr*right)/(2*distance_to_end)))-alpha;
			}
			else
			{
				double middle=fabs(left-right)/2;

				// we prepare the right position on the straight
				if( next_rad>0 )
				{
					//problem with position
					if( left<middle )
						correction=correction*3;

					alpha_result=(atan((left-correction*right)/(2*distance_to_end)))-alpha;
				}
				else
				{
					//problem with position
					if( right<middle )
						correction=correction*3;

					alpha_result=(atan((correction*left-right)/(2*distance_to_end)))-alpha;
				}
			}
		}
	}
	else
	{
		// The Car is in a curve
		correction=2;

		int first_part=0;

		if(curve_factor==1)
		{
			// Oval curves
			correction=1;

			if(to_end>(cur_len/RISK_ALPHA_1))
			{
				correction=16;
				first_part=1;
			}
		
			if(to_end>(cur_len/RISK_ALPHA_2) && (correction==1))
				correction=8;

			if(to_end>(cur_len/RISK_ALPHA_3) && (correction==1))
				correction=1.85;
		}

		if(curve_factor==2)
		{
			// Very Fast curves
			correction=1.1;

			if(to_end>(cur_len/RISK_ALPHA_1))
			{
				correction=2.9;
				first_part=1;
			}

			if(to_end>(cur_len/RISK_ALPHA_2) && (correction==1.1))
				correction=1.9;

			if(to_end>(cur_len/RISK_ALPHA_3) && (correction==1.1))
				correction=1.4;

			// Long curves management
			if(cur_len>1)
				correction=correction*1.3;
		}

		if(curve_factor==3)
		{
			// Fast curves
			correction=1.8;

			if(to_end>(cur_len/RISK_ALPHA_1))
			{
				correction=3.2;
				first_part=1;
			}

			if(to_end>(cur_len/RISK_ALPHA_2) && (correction==1.8))
				correction=2.8;

			if(to_end>(cur_len/RISK_ALPHA_3) && (correction==1.8))
				correction=2.4; 

			// Long curves management
			if(cur_len>1.0)
				correction=correction*1.8;
			else
				if(cur_len>0.8)
					correction=correction*1.4;
		}

		if(curve_factor==4)
		{
			// Medium curves
			correction=1.8;

			if(to_end>(cur_len/RISK_ALPHA_1))
			{
				first_part=1;
				correction=3.5;

				if((curve_type(next_rad, next_len)>2) &&
					!same_direction(cur_rad, next_rad))
					correction=4; // the car will drive a double curves
			}

			if(to_end>(cur_len/RISK_ALPHA_2) && (correction==1.8))
				correction=2.5;

			if(to_end>(cur_len/RISK_ALPHA_3) && (correction==1.8))
				correction=2.3;

			// Long curves management
			if(cur_len>1.5)
				correction=correction*1.8;
			else
				if(cur_len>0.6)
					correction=correction*1.4;
		}

		if(curve_factor==5)
		{
			// Slow curves of different types
			correction=3;

			if(to_end>(cur_len/RISK_ALPHA_1))
			{
				first_part=1;
				correction=3.8;

				if((curve_type(next_rad, next_len)>2) &&
					!same_direction(cur_rad, next_rad))
					correction=4; // the car will drive a double curves
			}

			if(to_end>(cur_len/RISK_ALPHA_2) && (correction==3))
				correction=3.4;

			if(to_end>(cur_len/RISK_ALPHA_3) && (correction==3))
				correction=3.2;

			// Long curves management
			if(cur_len>1.5)
				correction=correction*1.8;
			else
				if(cur_len>0.9)
					correction=correction*1.4;
		}
					
		// management of double curve exit with high different
		// between slow exit and fast enter
		if(double_curve>=1)
		{
			if(next_curve_factor==0)
				correction=correction*3;
			else
				correction=correction*5;
		}

		// wrong curve position management in the exit part of the curve
		if((cur_rad>0) && (right<30) && (first_part==0))
			correction=correction*4;

		if((cur_rad<0) && (left<30) && (first_part==0))
			correction=correction*4;

		// management of long curve
		double curve_l=fabs(cur_rad*cur_len);
		double slow_factor=0;

		// check if too close in the inner and extern curve
		double too_close=0;
		double too_large=0;
		double middle=fabs(right+left)/2;

		if((cur_rad>0) && (left<30))
			too_close=left;

		if((cur_rad>0) && (right<middle))
			too_large=left;

		if((cur_rad<0) && (right<30))
				too_close=right;

		if((cur_rad<0) && (left<middle))
			too_large=right;

		if((curve_l>300) && (curve_factor>1))
		{
			slow_factor=curve_l/200;
		}

		// Determinate the top speed for the actual curve
		double top_speed =curve_top_speed(cur_rad,
										  cur_len, 
										  next_rad, 
										  next_len,
										  left,
										  right);

		if((slow_factor!=0) && 
		   (too_close==0) &&
		   (speed>top_speed) &&
		   (curve_factor>1))
		   correction=correction*slow_factor;

		// double curves management
		double curve_lenght=cur_len*fabs(cur_rad); 

		// default for Oval curves
		double factor_1=2;   // How much is the starring value
		double factor_2=2;   // When start sterring, low value mean start the second curve before

		if((next_curve_factor==3) || (next_curve_factor==2))
		{
			factor_2=2.3; 

			if(same_direction(cur_rad, next_rad))
				factor_2=2.5;
		}

		if(next_curve_factor==4)
		{
			factor_2=2.3; 

			if(same_direction(cur_rad, next_rad))
				factor_2=2.4;
		}

		if(next_curve_factor==5)
		{
			factor_2=2.2; 
			factor_1=2.1;

			if(same_direction(cur_rad, next_rad))
				factor_2=2;
		}

		if((curve_factor!=0) && 
		   (next_curve_factor!=0) &&
		   (!same_direction(cur_rad, next_rad)))
		{
			if(curve_factor<next_curve_factor)
			{
				double_curve=(next_curve_factor-curve_factor);
			}
		}

		// management of out track
		if((left<=0) || (right<=0))
			correction=2;

		if( (next_rad!=0) && 
			(start_sterring(to_end, next_rad, next_len, left, right)) && 
			(distance_to_end<(curve_lenght/factor_2)) &&
			(too_large==0))
		{
			if((next_curve_factor==2) &&
				same_direction(cur_rad, next_rad))
				factor_1=1.8;

			if(factor_1>10)
				factor_1=10;

			// The car start sterring to prepare the next curve (es. double s)
			if( next_rad>0 )
				alpha_result=(atan((factor_1*left-right)/(2*distance_to_end)))-alpha;
			else
				alpha_result=(atan((left-factor_1*right)/(2*distance_to_end)))-alpha;
		}
		else
		{
			if(correction>10)
				correction=10;

			// we are sterring
			if ( cur_rad>0 )
				alpha_result=(atan((correction*left-right)/(2*distance_to_end)))-alpha;
			else
				alpha_result=(atan((left-correction*right)/(2*distance_to_end)))-alpha;
		}
	}

	return alpha_result;
}

// check the car status to manage pit stop strategy
int check_damage_and_fuel(Stage stage, 
						  unsigned long damage, 
						  double fuel,
						  double fuel_for_lap,
						  int lap_to_go,
						  double track_lenght)
{
	// stage,		 various stages of competition
	// damage,		 accumulated damage units (out of race 30000)
	// fuel,		 lbs. of fuel remaining 
	// fuel_for_lap, average value of fuel need for lap
	// lap_to_go,	 laps remaining to be completed
	// track_lenght, average track lenght

	// Specific Strategy for Qualifying
	if((stage==QUALIFYING) && (fuel<fuel_for_lap) && (lap_to_go>1))
		return 1;

	if((stage==QUALIFYING) && (damage>29000) && (lap_to_go>1))
		return 1;

	if(stage==QUALIFYING)
		return 0;

	// Race and Free lap Strategy
	if((damage>25000) &&
	   (damage>(lap_to_go*track_lenght*DAMAGE_FOR_MILE)))
		return 1;

	if(damage>DAMAGE_LIMIT)
		return 1;

	if(fuel<(fuel_for_lap*2))
		return 1;

	return 0;
}

// Main program
con_vec DGN(situation &s)
{
	con_vec result; // Return Value

	// static and constant value
	static int init = 1;

	// Improve start procedure
	if( to_start>0 )
		to_start--;

	if( s.cur_rad!=0 )
		to_start=0; // to solve the "start race in curve" situation

	// Calculate track length
	if( s.distance>track_lenght )
		track_lenght=s.distance;
	else ;
	//	cout<<endl;

	if(init)
	{              
		// Set up driver
		my_name_is("DGN");          

		result.vc=100;
		result.alpha=0;
		init = 0;    

		return result;                  
	}

	// Init the car
	double fuel_for_lap;

	if( track_lenght>0 )
		fuel_for_lap=track_lenght/FUEL_USE;
	else
		fuel_for_lap=s.distance/FUEL_USE;

	if(fuel_for_lap<=0)
		fuel_for_lap=6;

	if(s.starting) 
	{
		result.repair_amount=s.damage;

		if(s.stage==QUALIFYING)
		{
			result.fuel_amount=QUAL_FUEL;
		}
		else
		{
			if( (fuel_for_lap*(s.laps_to_go-1))>MAX_FUEL )
				result.fuel_amount=MAX_FUEL;
			else
				result.fuel_amount=fuel_for_lap*(s.laps_to_go-1);
		}
	}

	// calculate more values
	double track_width  = s.to_lft+s.to_rgt;
	double actual_alpha = asin( s.vn/s.v );
	double end_of_segment = segment_lenght( s.to_end, 
											s.cur_rad, 
											track_width );

	// speed management 
	double speed = ideal_speed( s.v, 
								s.cur_rad, 
								end_of_segment, 
								s.nex_len, 
								s.nex_rad, 
								s.cur_len, 
								s.to_end,
								s.to_lft,
								s.to_rgt);

	result.vc=speed;

	// direction management
	double sterring;

	if( to_start==0 )
		sterring = ideal_sterring(	actual_alpha, 
									s.to_lft, 
									s.to_rgt, 
									end_of_segment, 
									s.to_end, 
									s.cur_rad, 
									s.cur_len, 
									s.nex_rad,
									s.nex_len,
									s.v);
	else
		sterring=0;

	result.alpha=sterring;

	// Pit Stop Strategy
	if( s.laps_done>0 )
	{
		fuel_for_lap=track_lenght/FUEL_USE;

		// check the damage and fuel status
		if ( check_damage_and_fuel(	s.stage, 
									s.damage, 
									s.fuel,
									fuel_for_lap,
									s.laps_to_go,
									track_lenght) )
		{
			// request for a pit stop
			result.request_pit=1;
			
			// Refuel
			if( (fuel_for_lap*(s.laps_to_go+1))>MAX_FUEL )
				result.fuel_amount=MAX_FUEL;
			else
				result.fuel_amount=fuel_for_lap*(s.laps_to_go-1);

			// Repair damage
			result.repair_amount=s.damage;
		}
	}

	return result; 
}

