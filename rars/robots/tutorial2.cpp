/* 
TUTORIAL2.CPP 

History
- yet another RARS 'bot, by M. Timin, April '95 
- Object Oriented Robot + Pitting     January 2001

This file is the third in our tutorial series.  It is also a working robot.  The robot function name,
and its displayed name, is "TutorMan". This file may be compiled an linked just as any other RARS
driver. It has been tested, and it runs quite well on some of the tracks.  It crashes on the other
tracks because it is not smart enough for certain trick sequences of curves.  Of the tracks that I
tried, it runs best on SPEED2.TRK and OVAL2.TRK.  In the fourth tutorial we will improve the robot so
that he rarely crashes and drives faster as well! 

There is one major change in part of the steering servo algorithm, the "bias" formula.  I stated without
much explanation that the bias could be computed by "bias = atan(BIG_SLIP / speed)".  I'll now explain
how that formula is arrived at:  In move_car(), where it calculates the tire slip vector, we find this
line:   (see also vectors.txt and vectors.pcx) 

  Ln = -vc * sine;   Lt = v - vc * cosine; // vector sum to compute slip vector 

Now if the car is in equilibrium rounding the turn, then Lt will be zero, so that v == vc * cos(alpha)
and therefore vc == v/cos(alpha).  Substituting that result, we get that Ln == -v * tan(alpha), or
tan(alpha) == -Ln/v. 
In our robot code v is s.v, and -Ln is the slip speed, which I call BIG_SLIP, because a high speed turn
is a high g turn and that means a high traction force, which requires a fairly large slip speed.  That
is the derivation. 

Upon first running the new robot, with the above formula, It was soon obvious that a very bad value for
bias was frequently being chosen.  The reason for this is that the above derivation assumed that the car
was moving at the correct speed so that the traction force was balanced by the so-called "centrifugal
force".  Frequently the car is moving at a different speed, and needs to accelerate or decelerate.  If
the car is moving at less than equilibrium speed, then alpha should be lower so that the lateral
component of traction force is lower.  Therefore I added a "fudge factor" to the formula.  I have not
carefully derived this, I just knew that "centrifugal force" is proportional to the square of the speed,
so I just multiplied our bias formula by the square of the ratio of actual speed to target speed.  This
has the desired affect of reducing alpha when the car is accelerating in a turn, while still maintaining
the same bias when the car reaches it full cornering speed. 

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
const double DELTA_LANE = 2.0;         // when car "dead_ahead, change "lane" this much 
const double CORN_MYU   = .95;         // lateral g's expected when cornering 
const double ACCEL_FRACTION = .2;      // fraction of straightaway for braking 
const double DIST_FROM_INSIDE = 12.0;  // target distance from curve's inner rail 
const double STEER_GAIN = .5;          // gain of steering servo loop 
const double  DAMP_GAIN = 2.0;         // damping of steering servo loop 
const double  BIG_SLIP  = 14.0;        // affects the bias of steering servo loop 

//--------------------------------------------------------------------------
//                           Class Tutorial2
//--------------------------------------------------------------------------

class Tutorial2 : public Driver
{
public:
  Tutorial2::Tutorial2()
  {
    m_sName = "Tuto2";
    m_sAuthor = "Mitchell Timin";
    m_iNoseColor = oBLUE;
    m_iTailColor = oBLUE;
    m_sBitmapName2D = "car_blue_blue";
    m_sModel3D = NULL;
  }

  double corn_spd(double radius)       // returns maximum cornering speed, fps 
  { 
    double rad;                        // absolute value of the radius 
    rad = radius<0 ? -radius : radius; // make radius positive 
    return sqrt(rad * 32.2 * CORN_MYU);// compute the speed 
  } 

  con_vec drive(situation &s) 
  { 
    con_vec result = CON_VEC_EMPTY;    // This is what is returned. 
    double alpha, vc;                  // components of result 
    static double lane = -10000;       // an absurd value to show not initialized 
    double bias, speed, width; 

    if( s.starting )
    {
      result.fuel_amount = MAX_FUEL;     // fuel when starting
    }

    // service routine in the host software to handle getting unstuck from 
    // from crashes and pileups: 
    if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc)) 
      return result; 

    width = s.to_lft + s.to_rgt;       // compute width of track 

    // This is a little trick so that the car will not try to change lanes 
    // during the "dragout" at the start of the race.  We set "lane" to 
    // whatever position we have been placed by the host. 
    if(lane < -9000)                   // will be true only once 
      lane = s.to_lft;                 // better not to change lanes at the start 

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
    // course use the radius of the curve we are in. 
    if(s.cur_rad == 0.0) 
    { 
      bias = 0.0; 
      speed = corn_spd(s.nex_rad + DIST_FROM_INSIDE); 
    } 
    else  
    { 
      speed = corn_spd(s.cur_rad + DIST_FROM_INSIDE); 
      // See initial paragraphs for discussion of this formula. 
      bias = (s.v*s.v/(speed*speed)) * atan(BIG_SLIP / speed); 
      if(s.cur_rad < 0.0)   // bias must be negative for right turn 
        bias = -bias; 
    } 

    // set alpha:  (This line is the complete steering servo.) 
    alpha = STEER_GAIN * (s.to_lft - lane)/width - DAMP_GAIN * s.vn/s.v + bias; 

    // set vc:  When nearing end of straight, change "lane" for the turn, also. 
    if(s.cur_rad == 0.0)              // If we are on a straightaway, 
    {
      if(s.to_end > ACCEL_FRACTION * s.cur_len)  // if we are far from the end, 
      {
        vc = s.v + 50.0;                           // pedal to the metal! 
      }
      else                      // otherwise, adjust speed for the coming turn: 
      {
        if(s.v > 1.02 * speed)         // if we're 2% too fast, 
          vc = .95 * s.v;              // brake hard. 
        else if(s.v < .98 * speed)     // if we're 2% too slow, 
          vc = 1.05 * speed;           // accelerate hard. 
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
      vc = .5 * (s.v + speed)/cos(alpha);   // to maintain speed in corner 
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

Driver * getTutorial2Instance()
{ 
  return new Tutorial2();
}
