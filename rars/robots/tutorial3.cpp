/* 
TUTORIAL3.CPP 

History
- yet another RARS 'bot, by M. Timin, April '95 
- Object Oriented Robot + Pitting     January 2001

This file is the fifth in our tutorial series.  It is also a working robot.  The robot function name,
and its displayed name, is "TutMan6". This file may be compiled and linked just as any other RARS
driver. It has been tested, and it runs very well on almost all tracks. 
This code is closely based on the first four tutorials.  I will try to explain where and why it differs
in the comments of the code.  There are two other files that all of our readers should get from the ftp
site. They are vectors.txt and vectors.pcx.  The first is an explanation of the vector relationships
used in the car model.  The second is a PCX file of an accompanying sketch. 

The file CNTRL0.CPP is still useful as a reference for some details 
of robot code that are not explained below. 

m 
*/ 

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include <string.h> 
#include <stdlib.h> 
#include <math.h> 
#include "car.h" 

//--------------------------------------------------------------------------
//                           D E F I N E S
//--------------------------------------------------------------------------

// parameters to tinker with: 
const double DELTA_LANE = 1.0;  // when car "dead_ahead, change "lane" this much 

const double CORN_MYU   =  .92;       // lateral g's expected when cornering 
const double BRAKE_ACCEL = -30.0;     // acceleration when braking on straight 
const double BRAKE_RATIO = .935;      // tire speed ratio when braking   " 
const double BRK_CRV_ACC = -25.0;     // acceleration when braking in curve 
const double BRK_CRV_SLIP = 5.0;      // tire slip for braking in curve 
const double DIST_FROM_INSIDE = 12.0; // target distance from curve's inner rail 
const double STEER_GAIN = 0.5;   // gain of steering servo loop 
const double  DAMP_GAIN = 1.1;        // damping of steering servo loop 
const double  BIG_SLIP = 9.0;     // affects the bias of steering servo loop 

//--------------------------------------------------------------------------
//                           Class Tutorial3
//--------------------------------------------------------------------------

class Tutorial3 : public Driver
{
public:
  Tutorial3::Tutorial3()
  {
    m_sName = "Tuto3";
    m_sAuthor = "Mitchell Timin";
    m_iNoseColor = oBLUE;
    m_iTailColor = oBLUE;
    m_sBitmapName2D = "car_blue_blue";
    m_sModel3D = NULL;
  }

  double corn_spd(double radius)     // returns maximum cornering speed, fps 
  { 
    //  MUST NEVER CALL THIS ROUTINE WITH ZERO ARGUMENT! 
    return sqrt(radius * 32.2 * CORN_MYU);     // compute the speed 
  } 

  // Calculates the critical distance necessary to bring a car from speed 
  // v0 to speed v1 when the braking acceleration is "a", ft per sec^2. 
  // Speeds are in fps.  ("a" should be negative) 
  double CritDist(double v0, double v1, double a) 
  { 
    double dv; 

    dv = v1 - v0; 
    if(dv > 0.0)          // this saves having such a test in the caller 
      return(0.0); 
    return (v0 + .5 * dv) * dv / a; 
  } 

  con_vec drive(situation &s) 
  { 
    con_vec result = CON_VEC_EMPTY;    // This is what is returned. 
    double alpha, vc;                  // components of result 
    static double lane = -10000;       // an absurd value to show not initialized
    double bias, width, to_end, speed, speed_next=0.0;

    if( s.starting )
    {
      result.fuel_amount = MAX_FUEL;   // fuel when starting
    }

    // service routine in the host software to handle getting unstuck from 
    // from crashes and pileups: 
    if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc)) 
      return result; 

    width = s.to_lft + s.to_rgt;   // compute width of track 

    // This is a little trick so that the car will not try to change lanes 
    // during the "dragout" at the start of the race.  We set "lane" to 
    // whatever position we have been placed by the host. 
    if(lane < -9000)              // will be true only once 
      lane = s.to_lft;           // better not to change lanes at the start 

    // Set "lane" during curves.  This robot sets "lane" during curves to 
    // try to maintain a fixed distance to the inner rail. 
    // For straightaways, we leave "lane" unchanged until later. 
    if(s.cur_rad > 0.0)                // turning left 
      lane = DIST_FROM_INSIDE; 
    else if(s.cur_rad < 0.0)           // turning right 
      lane = width - DIST_FROM_INSIDE; 

    // set the bias: 
    // Bias is an additive term in the steering servo, so that the servo 
    // doesn't have to "hunt" much for the correct alpha value.  It is an 
    // estimate of the alpha value that would be found by the servo if there 
    // was plenty of settling time.  It is zero for straightaways. 
    // Also, for convenience, we call the corn_spd() function here.  On 
    // the straightaway, we call it to find out the correct speed for the 
    // corner ahead, using s.nex_rad for the radius.  In the curve we of 
    // course use the radius of the curve we are in.  But also, we call it 
    // for the next segment, to find out our target speed for the end of 
    // the current segment, which we call speed_next. 
    if(s.cur_rad == 0.0) 
    { 
      bias = 0.0; 
      if(s.nex_rad > 0.0) 
        speed = corn_spd(s.nex_rad + DIST_FROM_INSIDE); 
      else if(s.nex_rad < 0.0) 
        speed = corn_spd(-s.nex_rad + DIST_FROM_INSIDE); 
      else 
        speed = 250.0; 
    } 
    else  
    { 
      if(s.nex_rad == 0.0) 
        speed_next = 250.0; 
      else 
        speed_next = corn_spd(fabs(s.nex_rad) + DIST_FROM_INSIDE); 
      speed = corn_spd(fabs(s.cur_rad) + DIST_FROM_INSIDE); 
      bias = (s.v*s.v/(speed*speed)) * atan(BIG_SLIP / speed); 
      if(s.cur_rad < 0.0)   // bias must be negative for right turn 
        bias = -bias; 
    } 

    // set alpha:  (This line is the complete steering servo.) 
    alpha = STEER_GAIN * (s.to_lft - lane)/width - DAMP_GAIN * s.vn/s.v + bias; 

    // set vc:  When nearing end of straight, change "lane" for the turn, also. 
    if(s.cur_rad == 0.0)               // If we are on a straightaway, 
    {
                                       // if we are far from the end, 
      if(s.to_end > CritDist(s.v, speed, BRAKE_ACCEL)) 
        vc = s.v + 50.0;               // pedal to the metal! 
      else                             // otherwise, adjust speed for the coming turn: 
      {
        if(s.v > 1.02 * speed)         // if we're 2% too fast, 
          vc = BRAKE_RATIO * s.v;      // brake hard. 
        else if(s.v < .98 * speed)     // if we're 2% too slow, 
          vc = 1.1 * speed;            // accelerate hard. 
        else                           // if we are very close to speed, 
          vc = .5 * (s.v + speed);     // approach the speed gently. 
        // approach the lane you want for the turn: 
        if(s.nex_rad > 0.0) 
          lane = DIST_FROM_INSIDE; 
        else 
          lane = width - DIST_FROM_INSIDE; 
      }  
    } 
    else       // This is when we are in a curve:  (seek correct speed) 
    {
      // calculate vc to maintain speed in corner 
      vc = .5 * (s.v + speed)/cos(alpha); 
      // calculate distance to end of curve: 
      if(s.cur_rad > 0.0) 
        to_end = s.to_end * (s.cur_rad + DIST_FROM_INSIDE); 
      else 
        to_end = -s.to_end * (s.cur_rad - DIST_FROM_INSIDE); 
      // compute required braking distance and compare: 
      if(to_end <= CritDist(s.v, speed_next, BRK_CRV_ACC))  
      { 
         vc = s.v - BRK_CRV_SLIP; 
      } 
    }  

    // During the acceleration portion of a straightaway, the lane variable 
    // is not changed by the code above.  Hence the code below changes it a 
    // little at a time until there is no car dead_ahead.  This code here has 
    // no affect at all in the turns, nor in the braking portion 
    // of the straight. 
    if(s.dead_ahead)                   // Change the lane a little if someone's 
      if(s.to_lft > s.to_rgt)          // in your way. 
        lane -= DELTA_LANE;            // lane must be a static variable 
      else 
        lane += DELTA_LANE; 

    result.vc = vc;   result.alpha = alpha; 

    // Pit: if the fuel is too low
    //  Fuel: full
    //  Damage: repair all
    if( s.fuel<10.0 ) 
    {
      result.request_pit   = 1;
      result.repair_amount = s.damage;
      result.fuel_amount = MAX_FUEL;
    }

    return result; 
  }
};

Driver * getTutorial3Instance()
{ 
  return new Tutorial3();
}

