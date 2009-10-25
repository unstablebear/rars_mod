/**
 *
 * BULLE2.CPP - A robot "driver" for RARS ver. 0.73
 *
 * Filename:  bulle2.cpp
 * Robot:     bulle2
 * Races:     all
 * Source:    Public
 * Color:     Green/Green
 * Data:      -
 * @version    1.0, Sun May 14
 * @author:    Marc Gueury, marc.gueury@skynet.be (Belgium)
 *
 */

//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------

#include <string.h>                      // strcpy
#include <math.h>                        // fabs, sqrt
#include <stdio.h>
#include <stdlib.h>

#include "car.h"                         // stuck
#include "track.h"
#include "os.h"
#include "misc.h"                        // alpha_limit, collide

//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------

#define MAX_POINT               10000   
#define MAX_ALPHA                 0.4
#define STRAIGHT_SPD           400.00   // speed for straights
#define OLD_CPT_MIN               5.0
#define BC1                     0.014
#define BC2                     0.020

//--------------------------------------------------------------------------
// Macros
//--------------------------------------------------------------------------

#ifndef max 
  #define max(x,y)              (((x)<(y))?(y):(x))        // return maximum
  #define min(x,y)              (((x)>(y))?(y):(x))        // return minimum
#endif
#define sqr(x)                  ((x)*(x))                  // return x}
#define my_mod(x,y)             (((x)<(y))?(x):(x-y))      // return absolute
#define error(x, a)             {fprintf(stdout,a);exit(x);}

//--------------------------------------------------------------------------
// Globals
//--------------------------------------------------------------------------

// static double       g_Log;           // display a log variable

double              g_TimePenality;     // optimization (penalty if too close from border)
int                 g_OptiRunning = 0;

//--------------------------------------------------------------------------
// Types
//--------------------------------------------------------------------------

struct BulleParam
{
  double PointCurve;   // point for curve
  double AccelCurve;   // acceleration for curve
  double DecelCurve;   // acceleration for curve
  double SpeedMag;     // to calc. the speed in corner
  double SpeedMag2;    // to calc. the speed in corner
  double StepFeet;     // distancce between POINT
  double AlphaDist;    // distance to calculate alpha
  double MinLane;      // minimum lane
  double MaxLane;      // maximum lane
};

///////////////////////////////////////////////////////////////////////////

BulleParam param =

{
/*
  1.02, // AccelCurve
  1.02, // DecelCurve
  34,   // SpeedMag
  30.6, // SpeedMag2
  7.5,  // StepFeet
  4.0,  // AlphaDist
  0.01, // MinLane
  0.99, // MaxLane
*/
    1.0, 1.02,  1.02, 36.500000, 30.600000,  9.437500,  0.000000,  0.010000,  0.990000
};

///////////////////////////////////////////////////////////////////////////

/** 
 * BullePoint define a point used by a BullePath
 */
class BullePoint
{
 public:
  
  segment     * m_rs;
  double      m_lane;
  double      m_pos;
  double      m_width;
  double      m_x;
  double      m_y;
  double      m_radius;
  double      m_speed;

  double      m_xIn;
  double      m_yIn;

  double      m_xOut;
  double      m_yOut;

  /** 
   * Constructor 
   */
  BullePoint() {}

  /** 
   * setData: Fill the data of an empty BullePoint 
   * 
   * @param rs    track segment
   * @param pos   distance from the begin of the segment
   * @param lane  from 0 to 1 distance from left to right of the segment
   * @param width width of the segment
   */
  void setData( segment * rs, double pos, double lane, double width )
  {
    m_rs    = rs;
    m_pos   = pos;
    m_width = width;

    setLane( lane );
    m_xIn = getX( 0.0 );
    m_yIn = getY( 0.0 );
    m_xOut = getX( 1.0 );
    m_yOut = getY( 1.0 );
  }

  /** 
   * setLane: modify the lane 
   * 
   * @param lane  from 0 to 1 distance from left to right of the segment
   */
  void setLane( double lane )
  {
    m_lane  = lane;

    m_x = getX( lane );
    m_y = getY( lane );
  }

  /** 
   * getX: Calculate X with a lane 
   * 
   * @param lane  from 0 to 1 distance from left to right of the segment
   * @return the X coordinate of the point
   */
  double getX( double lane )
  {
    if( m_rs->radius==0.0 )
      return( m_rs->beg_x + (lane*m_width)*sin( m_rs->beg_ang ) + m_pos*cos( m_rs->beg_ang ) );
    else if( m_rs->radius>0.0 )
      return( m_rs->cen_x + (m_rs->radius+lane*m_width)*sin(m_rs->beg_ang+m_pos) );
    else
      return( m_rs->cen_x + (m_rs->radius+lane*m_width)*sin(m_rs->beg_ang-m_pos) );
  }

  /** 
   * getY: Calculate Y with a lane 
   * 
   * @param lane  from 0 to 1 distance from left to right of the segment
   * @return the Y coordinate of the point
   */
  double getY( double lane )
  {
    if( m_rs->radius==0.0 )
      return( m_rs->beg_y - (lane*m_width)*cos( m_rs->beg_ang ) + m_pos*sin( m_rs->beg_ang ) );
    else if( m_rs->radius>0.0 )
      return( m_rs->cen_y - (m_rs->radius+lane*m_width)*cos(m_rs->beg_ang+m_pos) );
    else
      return( m_rs->cen_y - (m_rs->radius+lane*m_width)*cos(m_rs->beg_ang-m_pos) );
  }

  /** 
   * setLane: define the radius
   * 
   * @param radius  radius of the point and his neighbour
   */
  void setRadius( double radius )
  {
    m_radius  = radius;
  }

  /** 
   * setSpeed: define the speed
   * 
   * @param speed  speed to use near the point
   */
  void setSpeed( double speed )
  {
    m_speed  = speed;
  }
};

///////////////////////////////////////////////////////////////////////////

/** 
 * Bulle2 is a class implenting a Driver
 */
class Bulle2 : public Driver
{
public:
  ///////////////////////////////////////////////////////////////////////////
  track_desc   t;
  int          nbPathPoint;

  int          m_CurSeg;
  int          m_CurPoint;         // point just after the car
  int          m_BefPoint;         // point just before the car
  double       m_FwdX;             // point used by the alpha calculation
  double       m_FwdY;             // point used by the alpha calculation
  double       m_PropCurPoint;     // dist(BefPoint)/(dist(befPoint)+dist(curPoint))
  double       m_ToLft;            // distance to the left of the road
  double       m_ToRgt;            // distance to the rigth of the road
  long         m_Cpt;              //
  double       m_Vn, m_Vt;          // speed of the car ./. car
  double       m_Vx, m_Vy;          // speed of the car ./. car
  double       m_Vmag;             // magnitude of the speed
  double       m_CurRadius;
  double       m_AngTarget;

  BullePoint   m_CarPoint;
  BullePoint   vPathPoint[MAX_POINT];
  ///////////////////////////////////////////////////////////////////////////
  
  /**
   * Constructor
   */
  Bulle2()
  {
    m_sName = "Bulle2";
    m_sAuthor = "Marc Gueury";
    m_iNoseColor = oLIGHTGREEN;
    m_iTailColor = oGREEN;
    m_sBitmapName2D = "car_green_green";

    m_CurPoint = 1;
    m_Cpt = 0;
  }

  /** 
   * initPath create all the point and place them at the middle of the track
   */
  void InitPath()
  {
    int i, j;

    nbPathPoint = 0;

    for( i=0; i<t.NSEG; i++ )
    {
      segment * rs  = &( t.trackin[i] );
      segment * rs2 = &( t.trackout[i] );
      if( rs->radius==0.0 )
      {
        // STRAIGHT
        int nseg = (int)( rs->length/param.StepFeet )+1;
        double step_feet = rs->length/nseg;
        for( j=1; j<=nseg; j++ )
        {
          double pos = j*step_feet;
          vPathPoint[nbPathPoint].setData( rs, pos, 0.5, t.width );
          nbPathPoint ++;
          if( nbPathPoint>=MAX_POINT ) error( 1, "Bulle2, InitPath: Too much point" );
        }
      } else {
        // CURVE
        double min_radius, max_radius;
        if( rs->radius==0.0 )
        {
          min_radius = rs->radius;
          max_radius = rs2->radius;
        }
        else
        {
          min_radius = rs2->radius;
          max_radius = rs->radius;
        }
        double curve_radius = (min_radius*param.PointCurve+max_radius)/(1.0+param.PointCurve);
        int nseg = (int) fabs( curve_radius*rs->length/param.StepFeet );
        double step_radius = rs->length/nseg;
        for( j=1; j<=nseg; j++ )
        {
          double pos = j*step_radius;
          vPathPoint[nbPathPoint].setData( rs, pos, 0.5, t.width );
          nbPathPoint ++;
          if( nbPathPoint>=MAX_POINT ) error( 1, "Bulle2, InitPath: Too much point" );
        }
      }
    }
  }

  /** 
   * Optimize optimize the position of the point
   */
  void Optimize()
  {
    for( int step=128; step>=1; step/=2 )
    {
      for( int i=0; i<nbPathPoint; i++ )
      {
        OptimizeStep1( i, step );
      }

      for( int nb=0; nb<50; nb++ )
      {
        for( int i=0; i<nbPathPoint; i++ )
        {
          OptimizeStep2( i, step );
        }
      }
    }
  }

  /** 
   * OptimizeStep1 place the point on a line between 2 neighbours
   *
   * @param i point 
   * @param step distance from point 
   */
  void OptimizeStep1( int i, int step )
  {
    ////
    //// Step 1: flaten the path.
    ////
    int before = (i-step+nbPathPoint)%nbPathPoint;
    int after  = (i+step)%nbPathPoint;
    BullePoint * bBefore  = &( vPathPoint[before] );
    BullePoint * bCurrent = &( vPathPoint[i]      );
    BullePoint * bAfter   = &( vPathPoint[after]  );

    // intersection entre 2 droites
    double lane =(
                   (bAfter->m_x-bBefore->m_x)*(bCurrent->m_yIn-bBefore->m_y)
                   -(bAfter->m_y-bBefore->m_y)*(bCurrent->m_xIn-bBefore->m_x)
                 ) /
                 (
                   (bAfter->m_y-bBefore->m_y)*(bCurrent->m_xOut-bCurrent->m_xIn)
                  -(bAfter->m_x-bBefore->m_x)*(bCurrent->m_yOut-bCurrent->m_yIn)
                 );
    if( lane<param.MinLane ) lane = param.MinLane;
    if( lane>param.MaxLane ) lane = param.MaxLane;
    bCurrent->setLane( lane );
  }

  /** 
   * OptimizeStep2 place the point on a circle between 2 neighbours
   * (Idea from K1999)
   *
   * @param i point 
   * @param step distance from point 
   */
  void OptimizeStep2( int i, int step )
  {
    ////
    //// Step 2: try to keep the radius constant
    ////
    int before    = (i-step+nbPathPoint)%nbPathPoint;
    int befbef    = (before-step+nbPathPoint)%nbPathPoint;
    int after     = (i+step)%nbPathPoint;
    int aftaft    = (after+step)%nbPathPoint;
    BullePoint * bBefBef   = &( vPathPoint[befbef] );
    BullePoint * bBefore   = &( vPathPoint[before] );
    BullePoint * bCurrent  = &( vPathPoint[i]      );
    BullePoint * bAfter    = &( vPathPoint[after]  );
    BullePoint * bAftAft   = &( vPathPoint[aftaft] );

    double befRadius    = getInverseRadius( bBefBef, bBefore, bCurrent );
    double curRadius    = getInverseRadius( bBefore, bCurrent, bAfter );
    double aftRadius    = getInverseRadius( bCurrent, bAfter, bAftAft );
    double topRadius    = getInverseRadius( bBefBef, bCurrent, bAftAft );
    // double befDist   = getDist( bCurrent, bBefore );
    // double aftDist   = getDist( bCurrent, bAfter );

    // calculate the radius desired
    double radius ;
    if (fabs(aftRadius) < fabs(befRadius))
      radius = topRadius*param.AccelCurve;
    else
      radius = topRadius*param.DecelCurve;

    if( fabs(radius)<0.01 )
    {
      // calculate the radius with a small lane change
      double lane = bCurrent->m_lane;
      double delta_lane = 0.001;
      bCurrent->setLane( lane + delta_lane );
      double radius2 = getInverseRadius( bBefore, bCurrent, bAfter );
      // apply the change to the lane.
      if( fabs(radius2-curRadius)>0.000001 )
      {
        lane += (radius-curRadius)*delta_lane/(radius2-curRadius);
        if( lane<param.MinLane ) lane = param.MinLane;
        if( lane>param.MaxLane ) lane = param.MaxLane;
      } 
      bCurrent->setLane( lane );
    }
  }

  /** 
   * CalcSpeed calculate the radius and the speed to use between points
   *
   * @param s Situation during the race
   */
  void CalcSpeed( situation * s)
  {
    int step = 4;
    for( int i=0; i<nbPathPoint; i++ )
    {
      int before = (i-step+nbPathPoint)%nbPathPoint;
      int after  = (i+step)%nbPathPoint;
      BullePoint * bBefore  = &( vPathPoint[before] );
      BullePoint * bCurrent = &( vPathPoint[i]      );
      BullePoint * bAfter   = &( vPathPoint[after]  );

      // intersection entre 2 droites
      double curRadius = getInverseRadius( bBefore, bCurrent, bAfter );
      double radius = 9999999;
      if( curRadius!=0.0 )
        radius = 1/curRadius;
      bCurrent->setRadius( radius );

      double speed = sqrt( param.SpeedMag * fabs(radius) );
      bCurrent->m_speed = speed;
    }

    step = 1;
    for( int j=0; j<10; j++ )
    {
      for( int i=nbPathPoint-1; i>=0; i-- )
      {
        int after  = (i+step)%nbPathPoint;
        BullePoint * bCurrent = &( vPathPoint[i]      );
        BullePoint * bAfter   = &( vPathPoint[after]  );

        double dist = getDist( bCurrent, bAfter );
        // double speed_medium  = ( bAfter->m_speed  + bCurrent->m_speed  ) / 2.0;
        double speed_medium  = bAfter->m_speed;
        double radius_medium  = ( bAfter->m_radius + bCurrent->m_radius  ) / 2.0;
        double Acentrifuge = speed_medium * speed_medium / radius_medium;
        double Atangent = param.SpeedMag2*param.SpeedMag2 - Acentrifuge*Acentrifuge;
        if (Atangent < 0.0)
          Atangent = 0.0;
        Atangent = sqrt(Atangent);

        double drag = (DRAG_CON * speed_medium * speed_medium) * (2*s->damage+MAX_DAMAGE)/MAX_DAMAGE / (M + s->fuel / g);
        double time = dist / speed_medium;
        double MaxSpeed = bAfter->m_speed + (Atangent + drag) * time;
        if( MaxSpeed<bCurrent->m_speed )
          bCurrent->setSpeed( MaxSpeed );
      }
    }
  }

  /** 
   * getInverseRadius find 1/radius (to avoid infinite) of a circle passing by
   *                  3 points
   *
   * @param bBefore  point 1
   * @param bCurrent point 2
   * @param bAfter   point 3
   * @return 1/radius of the circle passing by the 3 points
   */
  double getInverseRadius( BullePoint * bBefore, BullePoint * bCurrent, BullePoint * bAfter )
  {
    double x1 = bBefore->m_x - bCurrent->m_x;
    double y1 = bBefore->m_y - bCurrent->m_y;
    double x2 = bAfter->m_x - bCurrent->m_x;
    double y2 = bAfter->m_y - bCurrent->m_y;

    double radius = 2.0 *( (x1*y2)-(x2*y1) )
                    / sqrt(
                           ((x1*x1)+(y1*y1))
                           *((x2*x2)+(y2*y2))
                           *(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)))
                          );
    return radius;
  }

  /** 
   * getDist calcuulate the distance between 2 points
   *
   * @param b1 point 1
   * @param b2 point 2
   * @return distance between the points
   */
  double getDist( BullePoint * b1, BullePoint * b2 )
  {
    return( sqrt( (b1->m_x-b2->m_x)*(b1->m_x-b2->m_x) + (b1->m_y-b2->m_y)*(b1->m_y-b2->m_y) ) );
  }

  /** 
   * LoadTrackParam loads the optimized constant of Bulle2 in Bulle2.dat
   *                for a given track
   *
   * @param trackname the name of the track
   * @return 1 if found else 0
   */
  int LoadTrackParam( char * trackname )
  {
    FILE * f;

    char line[10000];
    char s[128];
    char sMode[128];
    char * p;
    int bFound;
  
    if( (f=fopen( "robots/bulle2.dat", "r" )) == NULL ) 
    {
      if( (f=fopen( "bulle2.dat", "r" )) == NULL ) 
      {
        printf( "Bulle2: can not find bulle.dat" );
        exit(12345);
      }
    }
  
    bFound = 0; 
    while( !feof(f) && !bFound ) 
    {
      if( fgets( line, 10000, f ) >0 )
      {
        switch( line[0] ) 
        {
          case '[':
            memset( sMode, 0, 128 );
            p = strchr( line+1, ']' );
            strncpy( sMode, line+1, p-(line+1) );  
            break;
          case '\"':
            if( strcmp( sMode, "NORMAL SURFACE 1" )==0 ) 
            {
              memset( s, 0, 128 );
              p = strchr( line+1, '\"' );
              strncpy( s, line+1, p-(line+1) );  
              if( strcmp( trackname, s )==0 ) 
              {
                fgets( line, 10000, f ); //comments
                fgets( line, 10000, f ); //data
                sscanf( line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                  &(param.PointCurve),
                  &(param.AccelCurve),
                  &(param.DecelCurve),
                  &(param.SpeedMag),
                  &(param.SpeedMag2),
                  &(param.StepFeet),
                  &(param.AlphaDist),
                  &(param.MinLane),
                  &(param.MaxLane)
                );
                bFound = 1;
            }
          } 
          break;
        } 
      }   
    }
    fclose( f );
    return bFound;
  }
  
  ///////////////////////////////////////////////////////////////////////////

  /** 
   * CalcAlpha calc the alpha during the race.
   *           It is based on the calculation of a point at a distance "m_ForwardDist"
   *           on the path of the car. The alpha is the angle to go to this point.
   *           (Enable the Bulle2DrawPath to see it)
   *
   * @return the alpha
   */
  double CalcAlpha( /*situation & s*/ )
  {
    ////
    //// alpha
    ////
    double ang = atan2(m_Vy,m_Vx);

    // find x, y of a point in the trajectory where the dist from the car =FORWARD_DIS
    int before = m_BefPoint;
    int current = m_CurPoint;
    double fdist = (vPathPoint[current].m_speed+param.AlphaDist)/4.0;
    // double fdist = min(  sqrt( param.SpeedMag * fabs(vPathPoint[current].m_radius) ), 300.0)/param.AlphaDist;
    if( m_Cpt<80 ) fdist = max( fdist, 80-m_Cpt );

    double dist = getDist( &vPathPoint[before], &vPathPoint[current] )*(1.0-m_PropCurPoint);
    while( fdist>dist )
    {
      fdist -= dist; 
      before = current;
      current = (current+1)%nbPathPoint;
      dist = getDist( &vPathPoint[before], &vPathPoint[current] );
    }
    double prop = fdist/dist;
    m_FwdX = vPathPoint[before].m_x*(1.0-prop)+vPathPoint[current].m_x*prop;
    m_FwdY = vPathPoint[before].m_y*(1.0-prop)+vPathPoint[current].m_y*prop;

    double xdot_target = m_FwdX - m_CarPoint.m_x;
    double ydot_target = m_FwdY - m_CarPoint.m_y;
    m_AngTarget = atan2(ydot_target,xdot_target);

    double alpha = m_AngTarget-ang;
    if( alpha<-PI ) alpha += 2*PI;
    if( alpha>PI  ) alpha -= 2*PI;

    if (alpha < -MAX_ALPHA) alpha = -MAX_ALPHA;
    if (alpha > MAX_ALPHA) alpha = MAX_ALPHA;

    return( alpha*1.5 );
  }

  ////////////////////////////////////////////////////////////////////////////

  /** 
   * CalcVc calc the vc during the race.
   *        The car is between 2 points. The speed is the medium speed betwee
   *        the 2 points
   *
   * @return vc
   */
  double CalcVc( /*situation & s*/ )
  {
    double vc = vPathPoint[m_BefPoint].m_speed*(1.0-m_PropCurPoint)+vPathPoint[m_CurPoint].m_speed*m_PropCurPoint;
    return( vc );
  }

  ////////////////////////////////////////////////////////////////////////////

  /** 
   * PassingNo passing procedure used when the car is too much damaged.
   *           It slows down when there is a car in front and never pass ...
   * @param s Situation during the race
   * @param vc the speed
   */
  void PassingNo(  situation & s, double &vc )
  {
    for( int i=0;i<3; i++) if (s.nearby[i].who<16) 
    {
      // if there is a close car
      double y=s.nearby[i].rel_y;           // get forward distance
      double vy=s.nearby[i].rel_ydot;       // get relative speed

      // if the car is getting closer
     if( y<2.0*CARLEN ) 
     {
       vc = min(vc,s.v+vy-20);
     }
     else if( vy<0.0 && (y<4.6*CARLEN || y<-10.0*vy) )
     {
       vc = max(50,min(vc,s.v+vy-5.0)); 
     }
    }
  }

  /** 
   * PassingDistanceBrake calculate the braking distance needed when there is 
   *                      a car to pass
   * 
   * @param vnow current speed (s.v)
   * @param vthen wished speed
   * @param radius the radius of the current segment
   * @return the distance
   */
  double PassingDistanceBrake(double vnow, double vthen, double radius)
  {
    if (vnow<vthen)
      return 0.0;  // no need to brake as we are already at the desired speed!
    else if (radius != 0)
      return (BC2 * (vnow+vthen) * (vnow-vthen));
    return (BC1 * (vnow+vthen) * (vnow-vthen));
  
  }

  ////////////////////////////////////////////////////////////////////////////

  /** 
   * Passing4: The 4th algorithm to pass
   * 
   * @param s Situation during the race
   * @param alpha the direction
   * @param vc the speed
   * @param param a parameter (bigger it is bigger is the security margin)
   */
  void Passing4( situation & s, double &alpha, double &vc, double param )
  {
    int danger = 0;
    int kount = 0;
    double vc1;
    double v[4], d[4], x[4], y[4], vx[4], vy[4];
    for(int i=0;i<4;i++)
    {
      if(s.nearby[i].who<16)
      {
        x[i]=s.nearby[i].rel_x;         // distance to right (or left if < 0)
        y[i]=s.nearby[i].rel_y;         // distance ahead, always positive
        vx[i]=s.nearby[i].rel_xdot;     // relative lateral speed component
        vy[i]=s.nearby[i].rel_ydot;     // relative forward speed (negative)
        d[i]=sqrt(x[i]*x[i] + y[i]*y[i]);       // distance to other car
        v[i]=sqrt(vx[i]*vx[i] + vy[i]*vy[i]);   // relative speed

        if (vy[i] > -1)  continue;      // ignore faster cars
        else if (d[i] > 1.2*PassingDistanceBrake(s.v, s.v+vy[i]+5,s.cur_rad))
          continue;       // check whether you need to brake
        ++kount;
        if (fabs(x[i])>1.5*CARWID && x[i]*vx[i]>-10 
            && fabs(vx[i])>10 && vy[i]>-10)
          continue; // if slow car ahead, reserve more safety space
        else if (fabs(x[i])>CARWID && x[i]*vx[i]>-10 
                 &&fabs(vx[i])>10 && vy[i]<=-10)
          continue; // if fast car ahead, reserve less safety space
        else danger = 1;// brake or pass if trouble car within braking distance
      }
    }

    if (danger)
    {
       if(kount > 2)
          vc1 = min((s.v + vy[0] - 5),(s.v + vy[1] - 5));
       else
       {
          vc1 = min((s.v + vy[0] + 5),(param*vc));
          if (!s.cur_rad)
          {  //straigth => pass!
             if (x[0] < 0)  // car in left
                if (s.to_lft + x[0] < t.width - 1.5* CARWID)
                   alpha -= 0.03;
                //if we have room in right, turn right
                else  alpha += 0.03;
             else if (x[0] >= 0)    // car in right
             if (s.to_lft + x[0] > 1.5* CARWID) // if we have room in left
                alpha += 0.03; //turn left
             else 
                alpha -= 0.03;
          }
       } // end of kount < 2
    }// end of danger
    else 
       vc1 = vc; //this executes if no danger seen

    vc1 = max(vc1,5);  // To avoid stopping behind very slow car
    vc = vc1;
  }

  ////////////////////////////////////////////////////////////////////////////

  /** 
   * PassingMain Main algorithm of passing following the circonstances
   *             and the damage it decides which risk to take.
   * 
   * @param s Situation during the race
   * @param alpha the direction
   * @param vc the speed
   */
  void bPassingMain( situation & s, double &alpha, double &vc )
  {
    ////
    //// if the damage is too high, stay alive
    ////  => don't overtake anymore
    if( s.damage>29000 ) 
    {
      PassingNo( s, vc );
      return;
    }
    ////
    //// if the fuel is low, do not overtake  ( fuel 2.0 ~= 0.5 lap )
    ////  => don't want to overtake 2 times the same car :)
    if( s.fuel<2.0 ) 
    {
      Passing4( s, alpha, vc, 0.9 );
      return;
    }
    if( (s.damage>24000 && s.laps_to_go>5) || s.damage>25000  ) 
    {
      Passing4( s, alpha, vc, 0.9 );
      return;
    }
    Passing4( s, alpha, vc, 0.95 );
  }

  ////////////////////////////////////////////////////////////////////////////

  /**
   *  CalcInit initialize at each loop the global variables
   *
   *  @param s situation
   */
  void CalcInit( situation & s )
  {
    double ang = asin( s.vn/s.v );
    m_Vt     = s.v * cos( ang );
    m_Vn     = s.vn;
    m_Cpt++;
    m_Vmag   = sqrt( m_Vt*m_Vt + m_Vn*m_Vn );
    m_CurSeg = s.seg_ID;     
    m_ToLft  = s.to_lft;
    m_ToRgt  = s.to_rgt;
  
    segment * rs = &(t.trackin[m_CurSeg]);
    if( rs->radius==0.0 )
    {
      double sine = sin( rs->beg_ang ); 
      double cose = cos( rs->beg_ang ); 
      m_Vx = cose*m_Vt-sine*m_Vn;
      m_Vy = sine*m_Vt+cose*m_Vn;
    } else if( rs->radius>=0.0 ) {
      double sine = sin( rs->beg_ang+rs->length-s.to_end ); 
      double cose = cos( rs->beg_ang+rs->length-s.to_end ); 
      m_Vx = cose*m_Vt-sine*m_Vn;
      m_Vy = sine*m_Vt+cose*m_Vn;
    } else {
      double sine = sin( -rs->beg_ang+rs->length-s.to_end ); 
      double cose = cos( -rs->beg_ang+rs->length-s.to_end ); 
      m_Vx = cose*m_Vt+sine*m_Vn;
      m_Vy = -sine*m_Vt+cose*m_Vn;
    }

    if(t.trackin[m_CurSeg].radius<0.0 ) 
      m_CurRadius = t.trackout[m_CurSeg].radius;
    else
      m_CurRadius = t.trackin[m_CurSeg].radius;

    // Find the current point
    BullePoint * bCurrent = &( vPathPoint[m_CurPoint] );
    while( (bCurrent->m_rs!=rs) || (bCurrent->m_pos<rs->length-s.to_end) )
    {
      m_CurPoint = (m_CurPoint+1)%nbPathPoint;
      bCurrent = &( vPathPoint[m_CurPoint] );
    }

    // Calculate the car point 
    m_CarPoint.setData( rs, rs->length-s.to_end, s.to_lft/t.width, t.width );

    ////
    //// Calculate the prop between the point Current point and the previous one.
    ////
    m_BefPoint = (m_CurPoint-1+nbPathPoint)%nbPathPoint;
    BullePoint * bBefore = &( vPathPoint[m_BefPoint] );

    double curDist = bCurrent->m_pos-(rs->length-s.to_end);
    if( rs->radius!=0.0 ) curDist *= fabs(rs->radius);
    double befDist = 0.0;
    if( rs!=bBefore->m_rs ) 
    {
      // last point is at the end of the previous segment.
      befDist = rs->length-s.to_end;
      if( rs->radius!=0.0 ) befDist *= fabs(rs->radius); 
    } 
    else 
    {
      befDist = rs->length-s.to_end-bBefore->m_pos;
      if( rs->radius!=0.0 ) befDist *= fabs(rs->radius);
    }

    m_PropCurPoint = befDist/(befDist+curDist);
  }

  ////////////////////////////////////////////////////////////////////////////

  /**
   * Bulle2 Driver Main Function
   *
   * @param s Situation of the race
   * @return control vector
   */
  con_vec drive(situation& s)
  {
    double alpha, vc;                   // components of result
    con_vec result;                     // This is what is returned

    result.request_pit = 0;
    result.repair_amount = 0;

    if( s.starting )
    {
      t = get_track_description();  
      if( !g_OptiRunning )
      {
        LoadTrackParam( t.sName );
      }
      InitPath();
      Optimize();
      CalcSpeed( &s );
      g_TimePenality = 0.0;
      m_PropCurPoint = 0;

      ////
      //// fuel when starting
      ////
      if((s.stage==QUALIFYING)||(s.stage==PRACTICE))
        result.fuel_amount = 30;
      else
        result.fuel_amount = MAX_FUEL;
    }

    // init global variables
    CalcInit( s );

    // get the car out of an abnormal condition, thanks Mitchell :)
    if( stuck( s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc ) )
        return result;

    vc = CalcVc();
    alpha = CalcAlpha();
    bPassingMain( s, alpha, vc );
  
    result.vc = vc;   result.alpha = alpha;
    ////
    //// Fuel
    ////
    if( s.laps_done>1 && s.fuel<1.5*t.length/(s.fuel_mileage * 5280.0) ) 
    {
        int cpt = min( 20, s.laps_to_go );
        int damage_to_repair = max( ((long)s.damage-10000), 0 );
        result.request_pit   = 1;
        result.repair_amount = damage_to_repair*cpt/20;
        result.fuel_amount = (s.laps_to_go + 1) * t.length / (s.fuel_mileage * 5280.0);
    }
    if( s.laps_to_go>1 )
    {
        if( s.damage>25000 )
        {
            int cpt = min( 20, s.laps_to_go );
            result.request_pit   = 1;
            result.repair_amount = s.damage*cpt/20;
            result.fuel_amount = (s.laps_to_go + 1) * t.length / (s.fuel_mileage * 5280.0);
        }
    }

    ////
    //// Optimization
    ////
    double f = min(s.to_lft,s.to_rgt);
    if( f<0.1 ) g_TimePenality += 100.0*(0.1-f);

    return result;
  }
};

////////////////////////////////////////////////////////////////////////////

//#define WIN32
#ifdef WIN32

#include "g_view.h"
#define X_SCALE(a) ( (int)( ((a)-m_TopX)*m_ScaleX ))
#define Y_SCALE(a) ( (int)( ((a)-m_TopY)*m_ScaleY ))

#define LOG_SIZE 200
static double log1[LOG_SIZE];

/**
 * Bulle2DrawPath Draws th path of bulle  
 *                add the following line at the end of Tview2D::Refresh
 *
 *     >>  Bulle2DrawPath( drivers[0],this, m_TopX, m_TopY, m_ScaleX, m_ScaleY );
 *
 * @param vlc the view where it can draw
 * @param m_TopX X position of the pixel 0,0
 * @param m_TopY Y position of the pixel 0,0
 * @param m_ScaleX X zoom
 * @param m_ScaleY Y zoom
 */
void Bulle2DrawPath( Driver * driver, TView * vcl, double m_TopX, double m_TopY, double m_ScaleX, double m_ScaleY )
{
  int x, y;

  Bulle2 * bulle2 = (Bulle2 *)driver;
  for( int i=0; i<bulle2->nbPathPoint; i++ )
  {
    BullePoint * b = &( bulle2->vPathPoint[i] );
    int x = X_SCALE( b->m_x );
    int y = Y_SCALE( b->m_y );
    if( bulle2->m_CurPoint==i )
      vcl->DrawPoint( x, y, 5 );
    else 
      vcl->DrawPoint( x, y, 0 );
  }
  
  // ForwardPoint 
  x = X_SCALE( bulle2->m_FwdX );
  y = Y_SCALE( bulle2->m_FwdY );
  vcl->DrawHorzLine( x-2, x+2, y, 3 );
  vcl->DrawVertLine( x, y-2, y+2, 3 );

//  log1[LOG_SIZE-1] = g_Log;
//  for( i=0; i<LOG_SIZE-1; i++ )
//  {
//    log1[i]=log1[i+1];
//    vcl->DrawPoint( i, 30, 3 );
//    vcl->DrawPoint( i, (int)(30+log1[i]), 4 );
//  }

}
#endif

Driver * getBulle2Instance()
{ 
   return new Bulle2();
}
