/**
* file name:    Jpp.cpp
* author:       Jorge Cervantes
* e-mail:       JorgeCervantesO@netscape.net
* robot:        J++
* races:        All
* data files:   none
* confidential: no
*
* @version   0.1
*/

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream.h>
#include "car.h"
#include "os.h"
#include "misc.h"
#include "track.h"

//--------------------------------------------------------------------------
//                           Class Tutorial1
//--------------------------------------------------------------------------

class Jpp : public Driver
{
public:
  Jpp::Jpp()
  {
    m_sName = "J++";
    m_sAuthor = "Jorge Cervantes";
    m_iNoseColor = oBLACK;
    m_iTailColor = oLIGHTMAGENTA;
    m_sBitmapName2D = "car_orange_orange";
    ALPHA_FACTOR = 2.0;       // cornering alpha factor
    BRAKE_ACCEL = -40;      // acceleration when braking on straight
    BRK_CRV_ACC = -35;      // acceleration when braking in curve
    DIST_FROM_INSIDE = 1;  // target distance from curve's inner rail
    DIST_FROM_OUTSIDE = 5; // target distance from curve's outer rail
    rad=len=NULL;
    DP[0]=DP[1]=NULL;
    real_width  = 0.0,
      max_speed   = 300.0,
      cur_speed   = 0.0,
      last_speed  = 1.0,
      cur_x       = 0.0,
      last_x      = 0.0,
      avg_a       = 40.0,
      last_rad    = 0.0,
      last_damage = 0.0,
      lap_damage  = 0.0,
      max_v       = 100,
      max_lap_damage = 0.0;
    Total_Laps     = 0,
      width_reduced  = 0,
      segment_change = 0,
      outside_lane   = 0,
      curveing       = 0,
      SEGS_AHEAD     = 1,
      numseg         = 1,
      check_delta_percent = 1,
      forwards            = 1,
      recursed            = 0;
  }
  
  double ALPHA_FACTOR;      // cornering alpha factor
  double BRAKE_ACCEL;       // acceleration when braking on straight
  double BRK_CRV_ACC;       // acceleration when braking in curve
  double DIST_FROM_INSIDE;  // target distance from curve's inner rail
  double DIST_FROM_OUTSIDE; // target distance from curve's outer rail
  
  double 
    *rad        ,
    *len        ,
    *DP[2]      ,
    real_width  ,
    max_speed   ,
    cur_speed   ,
    last_speed  ,
    cur_x       ,
    last_x      ,
    avg_a       ,
    last_rad    ,
    last_damage ,
    lap_damage  ,
    max_v       ,
    max_lap_damage;
  
  int    
    Total_Laps     ,
    width_reduced  ,
    segment_change ,
    outside_lane   ,
    curveing       ,
    SEGS_AHEAD     ,
    numseg         ,
    check_delta_percent ,
    forwards            ,
    recursed            ;
  
  
  situation s;
  double e_sp,e_d,vc,to_next; // to calculate speed
  
  
  int sign( double x ){ return x == 0.0 ? 0 : x > 0.0 ? 1 : -1;}
  
  int next( int i ){ return ++i % numseg; };
  int prev( int i ){ return --i < 0 ? numseg + i: i; };
  
  int n( int i )
  { if( forwards ) return next( i ); return prev( i );}
  
  int p( int i )
  { if( forwards ) return prev( i ); return next( i );}
  
  double Mag( double x, double y )
  { return sqrt( x*x + y*y ); }
  
  double cateto( double h, double x )
  { return h>x ? sqrt( h*h - x*x ) : 0.0; }
  
  void set_rad_len( void )
  {
    segment *lftwall = get_track_description().trackin,
      *rgtwall = get_track_description().trackout;
    
    if( len[0] == 0.0 )
      for( int i=0; i<numseg; i++)
      { 
        rad[i] = lftwall[i].radius;
        if( rad[i]<0.0 )
          rad[i] = rgtwall[i].radius;
        len[i] = rgtwall[i].length;
      }
  }
  
  double s_curve_percent( int i, double l_n_i, int nn_i )
  { 
    double x1, y1, x2, y2, xx, yy, d, d2, r1, r2, l_i, l_nn_i, d3,
      cur_factor = DP[forwards][i];
    
    r1     = fabs( radius( i ));      // rad_big
    l_i    = length( i );
    
    r2     = fabs( radius( nn_i ));   // rad_big
    l_nn_i = length( nn_i );
    
    x1 = fabs( rad[ i ] )
      - fabs( rad[ i ] )
      * cos( l_i*0.5 - len[ i ]*DP[!forwards][i] )
      + ( r1 - DIST_FROM_INSIDE )
      * cos( l_i*0.5 - len[ i ]*DP[!forwards][i] );
    
    y1 =
      - fabs( rad[ i ] )
      * sin( l_i*0.5 - len[ i ]*DP[!forwards][i] )
      + ( r1 - DIST_FROM_INSIDE )
      * sin( l_i*0.5 - len[ i ]*DP[!forwards][i] );
    
    x2 = fabs( rad[ i ] )
      - fabs( rad[ i ] ) * cos( len[ i ] )
      + l_n_i * sin( len[ i ] )
      - ( real_width
      + fabs( rad[ nn_i ] )
      )
      * cos( len[ i ] )
      - ( r2
      - DIST_FROM_INSIDE
      - fabs( rad[ nn_i ] )
      )
      * cos( len[ i ]
      + len[ nn_i ]*DP[!forwards][nn_i]
      - l_nn_i * 0.5
      );
    
    y2 = ( fabs( rad[ i ] )
      + real_width
      + fabs( rad[ nn_i ] )
      )
      * sin( len[ i ] )
      + l_n_i * cos( len[ i ] )
      + ( r2
      - DIST_FROM_INSIDE
      - fabs( rad[ nn_i ] )
      )
      * sin( len[ i ]
      + len[ nn_i ]*DP[!forwards][nn_i]
      - l_nn_i * 0.5
      );
    
    
    xx = x2 - x1;
    yy = y2 + y1;
    d  = Mag( xx, yy );
    
    x2 = fabs( rad[ i ] )
      - ( fabs( rad[ i ] )
      + real_width
      + fabs( rad[ nn_i ] )
      )
      * cos( len[ i ] )
      + l_n_i * sin( len[ i ] );
    
    y2 = ( fabs( rad[ i ] )
      + real_width
      + fabs( rad[ nn_i ] )
      )
      * sin( len[ i ] )
      + l_n_i * cos( len[ i ] )
      ;
    
    xx = x2 - x1;
    yy = y2 + y1;
    d2 = Mag( xx, yy );
    
    if( sign( rad[ n(nn_i) ] ) * sign( rad[ i ] ) <= 0 )
    {
      x2 -= ( fabs( rad[ nn_i ] )
        + real_width
        + fabs( rad[ n(nn_i) ] )
        )
        * cos( len[ i ] - len[ nn_i ] );
      y2 -= ( fabs( rad[ nn_i ] )
        + real_width
        + fabs( rad[ n(nn_i) ] )
        )
        * sin( len[ i ] - len[ nn_i ] );
      
      xx = x2 - x1;
      yy = y2 + y1;
      d3 = Mag( xx, yy );
    }
    
    if( ( d > r1 + r2 + CARWID
      ||
      r1 < d - r1 - CARWID
      )
      &&
      d2 - fabs( rad[ nn_i ] ) - DIST_FROM_INSIDE - CARWID*3 > r1
      )
    { 
      // Circles are not overlapping
      if( cur_factor == 0.0 )
        // first time
        return cur_factor - (l_n_i == 0.0 ? 0.01 : 0.00);
      
      // not first time
      if( cur_factor > 0.0 )
        // increasing but now not overlapping
        return cur_factor;
      
      // decreasing and not yet overlapping
      if( cur_factor > -0.99
        &&
        1.0 + cur_factor + DP[!forwards][i] > 0.01
        &&
        ( sign( rad[ n(nn_i) ] ) * sign( rad[ i ] ) <= 0
        ||
        d3 + fabs( rad[ n(nn_i) ] ) + DIST_FROM_INSIDE > r1
        )
        )
        return cur_factor - 0.01;
      
      // the max
      return cur_factor;
    }
    
    // circles are overlapping
    if( cur_factor == 0.0 )
      // first time
      return cur_factor + 0.01;
    
    // not first time
    if( cur_factor < 0.0 )
      // decreasing but now overlapping
      return cur_factor;
    //        break;
    
    // increasing and still overlapping
    if( cur_factor < 0.99
      &&
      l_i < 2*PI
      )
      return cur_factor + 0.01; //s_curve_percent( i, l_n_i, nn_i, cur_factor + 0.1 );
    
    // the max
    return cur_factor;
  }
  
  
  int touching_3rd_curve( int i )
  {
    if( rad[ n(n(i)) ] == 0.0 )
      return 0;
    
    if( sign( rad[ n(n(i)) ] ) == sign( rad[ i ] ))
      return 0;
    
    //circles touch:
    return 0;//forwards;
  }
  
  
  double c_curve_percent( int i )
  {
    double l_i, R, r_o, x, y, d, cur_factor=DP[forwards][i],t;
    
    {
      //    check_delta_percent = 0;
      R   = fabs( radius( i ));
      l_i = length( i );
      //    check_delta_percent = 1;
      
      r_o = fabs( rad[ n(i) ] ) + real_width*0.5;
      
      x = ( R - DIST_FROM_INSIDE - fabs( rad[ i ] ) )
        * cos( len[ i ]*DP[forwards][i] - l_i*0.5 )
        + fabs( rad[ i ] )
        - fabs( rad[ n(i) ] );
      
      y = ( R - DIST_FROM_INSIDE - fabs( rad[ i ] ) )
        * sin( len[ i ]*DP[forwards][i] - l_i*0.5 );
      
      // d = distance between next center and current center of R
      d = Mag( x, y );
      
      // calc distance to tangent of next curve's outer side
      t = r_o
        + ( fabs( rad[i] )
        - fabs( rad[ n(i) ] )
        )
        * cos( len[ n(i) ] )
        + ( fabs(R)
        - DIST_FROM_INSIDE
        - fabs(rad[i])
        )
        * cos( len[n(i)]
        - len[i]*DP[forwards][i]
        + l_i*0.5
        );
      
    }
    
    if( ( touching_3rd_curve( i )
      ||
      R + d > r_o
      &&
      ( t < R
      ||
      r_o > R
      )
      )
      )
    {
      // R too big
      if( cur_factor < .99
        ||
        ( fabs( rad[ i ] ) > fabs( rad[ n(i) ] )
        &&
        len[ i ] * cur_factor < len[ n(i) ]
        )
        )
      {
        if( l_i < 2*PI )
        {
          return cur_factor + 0.01;
        }
      }
    }
    
    // Circles are not overlapping or it has been enough percent
    return cur_factor;
  }
  
  
  double C_curve_percent( int i )
  { 
    double cur_factor = DP[forwards][i], l_i, R, x, y, d, r_in, r2;
    
    //    check_delta_percent = 0;
    R   = fabs( radius( i ));
    l_i = length( i );
    r2  = fabs( radius( n(n(i)) ));
    //    check_delta_percent = 1;
    
    r_in = fabs( rad[ n(n(i)) ] )
      + ( DIST_FROM_INSIDE );
    
    x = len[ n(i) ]
      - ( R - DIST_FROM_INSIDE  - fabs( rad[ i ] ))
      * sin( len[ i ]*(1+DP[!forwards][i]) - l_i*0.5 );
    
    y = ( R - DIST_FROM_INSIDE  - fabs( rad[ i ] ))
      * cos( len[ i ]*(1+DP[!forwards][i]) - l_i*0.5 )
      + fabs( rad[ i ] )
      + DIST_FROM_INSIDE
      - r_in;
    
    d = Mag( x, y );
    
    if( d + r_in < R
      &&
      ( len[ i ] * cur_factor < len[ n(n(i)) ]
      &&
      cur_factor < .99
      &&
      l_i < 2*PI
      )
      &&
      sqrt(R) > sqrt(r2) - 5
      )
      return cur_factor + 0.01;
    
    return cur_factor;
  }
  
  int closest()
  {
    int   min_i=-1;
    const double k=2;
    double min_d=sqrt(2.0)*k*CARLEN,d;
    for( int i=0; i<16; i++ )
    {
      if( s.nearby[i].who >= 0 
        && 
        s.nearby[i].who < 16
        )
      {
        d=sqrt( s.nearby[i].rel_x * s.nearby[i].rel_x 
          + s.nearby[i].rel_y * s.nearby[i].rel_y
          );
        if( d < min_d )
        {
          min_d = d;
          min_i = i;
        }
      }  
      else
        break;
    }
    
    return min_i;
  }
  
  double ending_speed( double Vo, double x, double a, double &e_d )
  {
    double t, Vf;
    
    e_d = x;
    
    //  x = Vo*t + a*t*t/2
    if( Vo*Vo < 4.0*a/2.0*(-x) )
    {
      t = - Vo/a;
      e_d = Vo*t + a*t*t/2;
      return 0.0;
    }
    
    t = (-Vo+sqrt(Vo*Vo-4.0*a/2.0*(-x))) / (2.0*a);
    Vf = Vo + a*t;
    
    return Vf;
  }
  
  
  int consider_nearby( 
    int i,
    double max_dot,
    int n,
    double min_vsqr,
    double max_time,
    double x_factor,
    double y_factor
    )
  {
    double y,x,vy,vx,dot,vsqr,c_time,x_close,y_close,theta,acc=0.0,separation;
    y=s.nearby[i].rel_y;         // get forward distance (center-to-center)
    x=s.nearby[i].rel_x;         // get right distance
    vx=s.nearby[i].rel_xdot;     // get forward relative speed
    vy=s.nearby[i].rel_ydot;     // get lateral relative speed
    
    separation = sqrt( (fabs(x)-CARWID)*(fabs(x)-CARWID)
      +
      (fabs(y)-CARLEN)*(fabs(y)-CARLEN)
      );
    
    min_vsqr < 0.1 ? min_vsqr = 0.1 : 0;
    
    if( y < CARLEN
      &&
      fabs(x) < CARWID
      )
      return y>0;
    
    // if the cars are getting closer, then the dot product of the relative
    // position and velocity vectors will be negative.
    dot = x * vx + y * vy;     // compute dot product of vectors
    if(dot > max_dot)            // no action if car is not approaching (fast).
      return 0;
    
    vsqr = vx*vx + vy*vy;      // compute relative speed squared
    
    // Brake if too much danger
    double v = sqrt(vsqr),
      mv = min_vsqr<25?min_vsqr:25,
      a  = BRAKE_ACCEL,
      e_s,d;
    e_s = ending_speed(v,separation,a,d);
    if( v>sqrt(-2*a*separation+mv) )
      v=v;
    if( ( s.nearby[i].to_rgt > -CARWID*2.0
      &&
      s.nearby[i].to_lft > -CARWID*2.0
      ||
      sign( s.nearby[i].vn ) == sign( s.nearby[i].to_lft )
      )
      && 
      y > 0
      )
    { 
      if( v > sqrt(-2*a*separation+mv) )            
        return 1;
      if( v > 20+mv*20 )
        return 1;
    }
    if( e_s > mv )
      e_s = e_s;
    
    if( vsqr < min_vsqr )
      return 0;
    
    // Time to closest approach is dot product divided by speed squared:
    c_time = -dot / vsqr;     // compute time to closest approach
    if(c_time > max_time)         // ignore if over three seconds
    {
      return 0;
    }
    
    /* If the execution gets this far, it means that there is a car
    ahead of you, and getting closer, and less than 3.0 seconds
    away.  Evaluate the situation more carefully to decide if
    evasive action is warranted: */
    if( s.nearby[i].braking )
      acc = -BRAKE_ACCEL;      // for at^2/2
    x_close = x + c_time * vx;// + c_time*c_time*acc*vx/(2*vx);  // x coord at closest approach
    y_close = y + c_time * vy;// + c_time*c_time*acc*vy/(2*vy);  // y coord at closest approach
    if( y_close < CARLEN/2. )
    {
      return 0;
    }
    
    /*  Due to the length of the cars, a collision will occur if
    x changes sign while y is less than CARLEN.  This
    can happen before the center-to-center distance reaches its
    point of closest approach. */
    // check if collision would occur prior to closest approach
    // if so, reduce c_time, re-calculate x_close and y_close:
    if(x_close * x < 0.0 && y < CARLEN ) 
    {
      if( fabs( vx ) < 0.1 )
        return 0;
      c_time = (fabs(x) - CARWID ) / fabs(vx);
      x_close = x + c_time * vx;      // x coord at closest approach
      y_close = y + c_time * vy;      // y coord at closest approach
      if( y_close < CARLEN/2. )
        return 0;
    }
    // Will it be a hit or a miss?
    theta = s.nearby[i].alpha; 
    //- s.alpha;
    if( !collide( x_close, y_close, theta ))
    {
      if(   fabs(x_close)>CARWID
        ||
        fabs(y_close)>CARLEN
        )
        return 0;
      else
        x=x;
    }
    
    return 1;
  }
  
  int crash_in_front()
  { 
    int min_i = closest();
    return min_i==-1 ? 0: ( s.nearby[min_i].rel_y > 0.0 );
  }
  
  int most_dangerous( float *danger, float *v )
  { 
    float max=0;
    int k=0;
    for( int i=0; i<16; i++)
    {
      if( danger[i]*v[i] > max )
      { 
        max=danger[i]*v[i];
        k=i;
      }
    }
    return k;
  }
  
  int apply_brakes()
  {
    int i,result=0;
    static float v[16]=        {0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.1f};
    static float d[16]=        {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
      a_danger[16]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
      t_danger[16]= {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
      n[16]=        {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    
    if( s.damage > last_damage )
    {
      i = closest();
      if( i >= 0 )
        if( (v[s.nearby[i].who] -= 0.5f) < 0.1f )
          v[s.nearby[i].who] = 0.1f;
        
        if( i != -1 )
        {
          lap_damage += s.damage - last_damage;
          
          a_danger[s.nearby[i].who] *= n[s.nearby[i].who]++;
          a_danger[s.nearby[i].who] += float( s.damage - last_damage );
          a_danger[s.nearby[i].who] /= n[s.nearby[i].who];
          
          t_danger[s.nearby[i].who] += float( s.damage - last_damage );
        }
        else
          return 0;
    }
    
    for( i=0; i<NEARBY_CARS; i++ )
    {
      if( s.nearby[ i ].who >= 16 
        ||
        s.nearby[ i ].who < 0 
        )
        break;
      
      if( i == 4 )
        result |= 1;
      
      if( //s.nearby[ i ].v > s.v
        //||
        !consider_nearby( i, -0.1, 1, 
        pow(1.01*s.nearby[i].rel_y/CARLEN,
        v[s.nearby[i].who]
        )
        /(i/10.+1),
        3, 1, 1 )
        )
        continue;
      
      
      // If we get here there is a collision predicted
      d[s.nearby[i].who] += 0.1f;
      v[s.nearby[i].who] += 0.1f;
      
      result |= 1;
    };
    
    if( s.lap_flag )
    {
      lap_damage = 0.0;
      for( i=0; i<16; i++)
      {
        if( d[i] == 0.0f )
        {  
          if( v[i] < 0.1f )
            v[i] = 0.1f;
        };
        d[i] = 0.0f;
      }
    }
    
    if( !result )
    { 
      int 
        m=most_dangerous(a_danger,v);
      if(( v[m] *= 0.993f ) < 0.01f )
        v[m]=0.01f;
      m=most_dangerous(t_danger,v);
      if(( v[m] *= 0.993f ) < 0.01f )
        v[m]=0.01f;
      m=most_dangerous(n,v);
      v[m] /= 0.993f;
    }
    
    return result;
  }
  
  void request_pit( con_vec &result )
  { 
    static int last_pit_lap = -1;
    static double last_pit_damage = 0,
      last_lap_damage=0,
      max_lap_damage=2500;
    double l = get_track_description().length / 5280., // in miles
      damage_per_gl,
      gl_to_end,
      damage_to_end;
    
    result.request_pit = 0;
    
    if( s.out_pits == 1 )
      last_pit_lap = s.laps_done,
      last_pit_damage = s.damage;
    
    if( s.lap_flag )
    {
      double delta = s.damage - last_lap_damage;
      last_lap_damage = s.damage;
      if( delta > max_lap_damage )
      {
        max_lap_damage = delta;
        if( max_lap_damage > 7500 )
          max_lap_damage = 7500;
      }
    }
    
    if( s.fuel_mileage == 0.0 )
      s.fuel_mileage = 1.0;
    
    damage_per_gl = ( s.damage - last_pit_damage )/( 150 - s.fuel );
    gl_to_end = (s.laps_to_go+5) * l / s.fuel_mileage;
    damage_to_end = damage_per_gl * gl_to_end;
    
    result.repair_amount = 0;
    if( s.damage > 30000-damage_to_end )
      result.repair_amount = int(( s.damage - (30000-damage_to_end) )  );
    
    if( result.repair_amount < (long)s.damage )
      l=l;
    else
      l=l;
    
    result.fuel_amount = 150.0;
    result.fuel_amount -= s.fuel;
    
    if( s.damage > 30000 - 2*max_lap_damage
      &&
      s.laps_to_go > 1
      &&
      s.laps_done > last_pit_lap
      &&
      ( s.damage - last_pit_damage ) / ( s.laps_done - last_pit_lap )
      >
      (30000.-s.damage) / s.laps_to_go
      ||
      s.fuel_mileage > 0.0
      &&
      s.fuel * s.fuel_mileage / l < 2 // less than 2 laps
      )
      result.request_pit = 1;
  }
  
  
  double delta_percent( int i )
  { 
    double dp = 0.0;
    
    if( len[ i ] == 0.0 )
      return 0.0;
    
    if( rad[ i ] == 0.0 )
    {
      DP[forwards][ i ] = 0.0;
      return 0.0;
    }
    
    if( !check_delta_percent )
      return DP[forwards][ i ];
    
    if( rad[ n(i) ] != 0.0 )
    {
      if( sign( rad[ i ] ) != sign( rad[ n(i) ] ))
        // S curve
        dp = s_curve_percent( i, 0.0, n(i) );
      
      else
        // c curve
        dp = c_curve_percent( i );
    }      
    else if( rad[ n(n(i)) ] == 0.0 )
      // 2 straights after segment i
      dp = 0.0;
    
    else if( sign( rad[ n(n(i)) ] ) != sign( rad[ i ] ))
      // kind of S curve.
      dp = s_curve_percent( i, len[ n(i) ], n(n(i)) );
    
    else
      // curve-straight-curve same side
      dp = C_curve_percent( i );
    
    DP[forwards][ i ] = dp;
    
    return( dp );
  }
  
  
  
  double delta_percent_prev( int i )
  {
    double dp = 0.0;
    
    if( len[ i ] == 0.0 )
      return 0.0;
    
    if( rad[ i ] == 0.0 )
    {
      DP[!forwards][ i ] = 0.0;
      return 0.0;
    };
    
    if( !check_delta_percent )
      return DP[ !forwards ][ i ];
    
    forwards = !forwards;
    
    if( rad[ n(i) ] != 0.0 )
    {
      if( sign( rad[ i ] ) != sign( rad[ n(i) ] ))
        // S curve
        dp = s_curve_percent( i, 0.0, n(i) );
      
      else
        // c curve
        dp = c_curve_percent( i );
    }
    else if( rad[ n(n(i)) ] == 0.0 )
      // 2 straights after segment i
      dp = 0.0;
    
    else if( sign( rad[ n(n(i)) ] ) != sign( rad[ i ] ))
      // kind of S curve.
      dp = s_curve_percent( i, len[ n(i) ], n(n(i)) );
    
    else
      // curve-straight-curve same side
      dp = C_curve_percent( i );
    
    forwards = !forwards;
    
    DP[ !forwards ][ i ] = dp;
    
    return ( dp );
  }
  
  
  void init_fast_vars()
  {
    memset( DP[0],  0, 2*numseg*sizeof( double ) );
    
    check_delta_percent = 1;
    for( int j=0; j<200; j++)
    {
      for( int i=0; i<numseg; i++)
      {
        delta_percent_prev( i );
        delta_percent( i );
      }
    } 
    check_delta_percent = 0;
    
  }
  
  
  void allocate_arrays()
  {
    int m;
    
    if( rad != NULL )
      return;
    
    numseg = get_track_description().NSEG;
    if( numseg * 0.75 < SEGS_AHEAD )
      SEGS_AHEAD = int( numseg * 0.75 );
    
    m = numseg
      * sizeof( double )
      * 4;
    
    if( m <= 4096 )
      rad = (double *)s.data_ptr;
    else
      rad = new double[m];
    
    len     = &(rad[numseg  ]);
    DP[0]   = &(rad[numseg*2]);
    DP[1]   = &(rad[numseg*3]);
    
    memset( rad, 0, numseg*2*sizeof( double ));
    
    set_rad_len();
    
    init_fast_vars();
  }
  
  
  double speed( double r )
  {
    double sp;
    if( r == 0.0 )
      sp = cur_speed + 20.0;
    else
      sp = sqrt( fabs( r ) * 32.2 );
    
    return sp;
  }
  
  double length( int i )
  {
    double l = len[ i ];
    
    l += len[ i ] * DP[forwards][i];
    l += len[ i ] * DP[!forwards][i];
    
    return l;
  }
  
  double radius( int i )
  {
    double r = fabs( rad[ i ] ),
      w = real_width-DIST_FROM_OUTSIDE, 
      max_rad;
    
    if( r == 0.0 )
      return r;
    
    r += DIST_FROM_INSIDE;
    double l = length( i );
    r += ( w - DIST_FROM_INSIDE )
      / ( 1.0 - cos( l * 0.5 ));
    
    max_rad = max_speed * max_speed * 1.1 * 1.1 // 10% more
      / 32.2;
    if( fabs( r ) > max_rad )
    {
      r = max_rad;
      if( r < fabs( rad[ i ] ) + DIST_FROM_INSIDE )
        r = fabs( rad[ i ] ) + DIST_FROM_INSIDE;
      
      return r * sign( rad[ i ] );
    }
    
    r *= sign( rad[ i ] );
    
    return r;
  }
  
  double curve_distance( int i )
  {
    double y, l_bw = len[i] * delta_percent_prev( i );
    double r = fabs( rad[ i ] );
    
    if( r == 0.0 ) return 0.0;
    
    r += DIST_FROM_INSIDE;
    
    y = fabs( radius( i )) - r;
    y *= sin( length( i ) * 0.5 - l_bw );
    
    return y;
  }
  
  
  double tangent_alpha( 
    int segment,
    double to_end,
    double d,
    double &d_big,
    double &R )
  {
    double alpha, r, t, l, te, y, z, d2, alpha_t, l_bw;
    
    d -= DIST_FROM_INSIDE;
    ;
    l    = length( segment );
    l_bw = len[ segment ] * delta_percent_prev( segment );
    te   = to_end
      - len[ segment ]
      - l_bw
      + l;
    
    r = fabs( rad[ segment ] ) + DIST_FROM_INSIDE;
    R = fabs( radius( segment ));
    
    y = ( R - r ) * cos( l * 0.5  )
      + ( r + d ) * cos( l - te );
    
    z = ( R - r ) * sin( l * 0.5 )
      + ( r + d ) * sin( l - te );
    
    t = Mag( y, z );
    
    alpha = ( te - l ) * sign( rad[ segment ] );
    
    if( y > 0.0 && z > 0.0 ) // first quarter
      alpha_t = asin( z / t ) * sign( rad[ segment ] );
    
    else if( y < 0.0 && z > 0.0 ) // second quarter
      alpha_t = acos( y / t ) * sign( rad[ segment ] );
    
    else if( y < 0.0 && z < 0.0 ) // third quarter
      alpha_t = acos( - y / t ) * sign( rad[ segment ] )
      + PI * sign( rad[ segment ] );
    
    else if( y > 0.0 && z < 0.0 ) // fourth quarter
      alpha_t = acos( - y / t ) * sign( rad[ segment ] )
      + PI * sign( rad[ segment ] );
    
    alpha += alpha_t;
    if( l < te && z < 0.0 )
      alpha -= PI * 2.0 * sign( rad[ segment ] );
    
    d_big = d2 = t - R;
    
    if( d_big < 0.0 )
    {
      // too close to inner side
      if( te > l * 0.5 )
      { 
        // getting closer to inner side
        double dx,dy,m,x;
        
        dx = r - (r+d) * cos( te - l * 0.5 );
        dy =     (r+d) * sin( te - l * 0.5 );
        m  = dy/dx;
        
        x = dx/2. + m*dy/2.;
        
        R = x;
        alpha = te - l * 0.5
          - acos(( R - dx ) / R );
        
        alpha *= sign( rad[ segment ] );
      }
    }
    else
      alpha += ( PI*0.5
      - asin( R / ( R + d2 ))
      )
      * sign( rad[ segment ] );
    
    if( fabs( alpha ) > PI * 0.5 )
      alpha = PI * 0.5 * sign( alpha );
    
    return alpha;
  }
  
  
  
  double last_to_inner( int i )
  {
    double lti,
      h,
      a,
      m,
      x,
      R = fabs( radius( i )),
      l = length( i ),
      to_center;
    
    h = - ( R - ( fabs( rad[ i ] )
      + DIST_FROM_INSIDE
      )
      );
    
    a = len[ i ] - l * 0.5;
    
    if( fabs(cos(a)) > 0.01 )
    {
      m = tan( a );
      x = ( 2*h + sqrt( 4*h*h - 4*(1+m*m)*(h*h-R*R) ))
        / ( 2*( 1 + m*m ));
      
      to_center = x / cos( a );
    }
    else
      to_center = cateto( R, h );
    
    lti = to_center
      - fabs( rad[ i ] );
    
    return lti;
  };
  
  
  
  double caution_factor( int n )
  {
    return 1.0 ;  ;
  }
  
  // returns maximum cornering speed, fps
  double corn_spd( int i ) 
  {
    // compute the speed
    
    if( len[ i ] == 0.0 )
      return 0.0;
    if( rad[ p(i) ] != 0.0
      &&
      sign( rad[ p(i) ] ) == sign( rad[ i ] )
      &&
      fabs( rad[i] ) * len[i] > fabs( rad[ p(i) ] )*len[ p(i) ]
      )
    { 
      double R,d_big, to_begin, to_inner, y;
      
      y = last_to_inner( p(i) );
      y += fabs( rad[ i ] );
      
      to_begin = 0.0;
      
      to_inner =  y;
      to_inner -= fabs( rad[ i ] );
      
      tangent_alpha( i,
        to_begin + len[ i ],
        to_inner,
        d_big,
        R
        );
      
      return speed( R );
    }
    
    return speed( radius( i ) );
  }
  
  
  
  
  void calculate_max_a( )
  {
    double a;
    
    if( fabs( cur_x - last_x ) < 0.1 )
      return;
    
    a = ( cur_speed*cur_speed - last_speed*last_speed )
      / ( 2.0 * ( cur_x - last_x ));
    
    if( s.vc < last_speed - 4.0
      &&
      fabs( a ) < 40.0
      &&
      s.to_rgt > DIST_FROM_INSIDE
      &&
      s.to_lft > DIST_FROM_INSIDE
      )
    { 
      avg_a += fabs( a ),
        avg_a *= 0.5;
      
      if( fabs( s.alpha ) < 0.01 )
      { 
        BRAKE_ACCEL -= fabs( a );
        BRAKE_ACCEL *= 0.5;
        if(BRAKE_ACCEL>-30.0)
          BRAKE_ACCEL=-30.0;
      }
      else
      { 
        BRK_CRV_ACC -= fabs( a );
        BRK_CRV_ACC *= 0.5;
        if(BRK_CRV_ACC>-25.0)
          BRK_CRV_ACC=-25.0;
      }
    }
    
    else
    { 
      static int count=0;
      
      if( ++count == 4 )
      { 
        count = 0;
        if(BRAKE_ACCEL < -40.0 )
          BRAKE_ACCEL -= 40.0,
          BRAKE_ACCEL *= 0.5;
        
        if(BRK_CRV_ACC < -35.0 )
          BRK_CRV_ACC -= 35.0,
          BRK_CRV_ACC *= 0.5;
      }
    }
  }
  
  
  int final_straight( )
  {
    static int next_seg = 0;
    
    next_seg &= s.laps_to_go != Total_Laps;
    
    next_seg |= s.seg_ID != 0 && s.laps_to_go == 1;
    
    return next_seg
      &&
      s.seg_ID == 0;
  }
  
  double t_distance( int i )
  {
    double R = fabs( radius( i )),
      r = fabs( rad[ i ] ) + DIST_FROM_INSIDE,
      l = len[ i ] * 0.5,
      c = cos( l ),
      m,
      x1,x2,
      t;
    
    if( rad[ i ] == 0.0 )
      return len[ i ];
    
    if( fabs( c ) < 0.1 )
    {
      t = acos((R-r)/R);
      return R*t*2;
    }
    
    m = tan( l );
    
    x1 = (2*(r-R) + sqrt(4*(r-R)*(r-R) - 4*(m*m)*(r*r-2*r*R) ))
      / (2*m*m);
    x2 = (2*(r-R) - sqrt(4*(r-R)*(r-R) - 4*(m*m)*(r*r-2*r*R) ))
      / (2*m*m);
    
    if( len[i] < PI )
      t = atan( m*x1 / ( x1+R-r ));
    else
      t = PI - fabs(atan( m*x2 / ( x2+R-r ))),
      t = fabs( t );
    
    return R*t*2.0;
  }
  
  
  double brake_accel( int i )
  {
    if( rad[ i ] != 0.0 )
      return BRK_CRV_ACC;
    
    if( len[ i ] < curve_distance( n(i) ) + curve_distance( p(i) ))
      return BRK_CRV_ACC;
    
    return BRK_CRV_ACC *    ( curve_distance( n(i) ) + curve_distance( p(i) )) / len[ i ]
      + BRAKE_ACCEL * (1-( curve_distance( n(i) ) + curve_distance( p(i) )) / len[ i ]);
  }
  
  // use speed, segment
  // modify vc, to_next, e_sp, e_d
  void check_segment( double speed, int segment )
  {
    if( speed > 0.0 
        &&
        vc > speed 
      )
      vc = speed;
        
    to_next += t_distance( segment );
    if( ( e_sp = ending_speed( s.v,
        to_next
        - curve_distance( n(segment) ),
        brake_accel( segment ),
        e_d )
        )
        >
        corn_spd( n(segment) )
        &&
        vc > corn_spd( n(segment) )
      )
      vc = corn_spd( n(segment) );
  }
    
  con_vec drive( situation &ss )
  {
    const char name[] = "J++";    // This is the robot driver's name!
    static int init_flag = 1;          // cleared by first call
    con_vec result;                    // This is what is returned.
    double alpha;
      
    double r, d, t, q,
      alpha_n,
      alpha_nn,
      alpha_nnn,
      x, y,
      _speed = 0.,
      n_speed = 0.,
      nn_speed = 0.,
      nnn_speed = 0.,
      to_begin,
      to_inner,
      d_big,
      R,
      e_d;
    
    static
      int //i,
      last_segment   = 0,
      slip_corrected = 0,
      following_next = 0;
    
    int 
      use_alpha_factor = 1,
      slip_correction = 1;
    
    s = ss;
    
    if(init_flag == 1)    // first time only, copy name
    {
      my_name_is(name);        // copy the name string into the host program
      init_flag          =
        result.request_pit = 0;
      result.alpha       =
        result.vc          =
        s.v                =
        s.out_pits         =
        ss.out_pits        = 0;
      s.side_vision      = 1;
      return result;
    }
    
    real_width = s.to_lft + s.to_rgt;
    
    allocate_arrays();
    
    if( rad == NULL )
    {
      result.request_pit = 0;
      result.alpha       =
        result.vc          = 0.0;
      return result;
    }
    
    
    if( Total_Laps == 0 )
    { 
      Total_Laps = s.laps_to_go;
      max_lap_damage = 30000 / ( s.laps_to_go + 1 );
    }
    
    curveing = 0;
    
    request_pit( result );
    if( result.request_pit )
      max_lap_damage = 30000 / ( s.laps_to_go + 1 );
    
    cur_speed = s.v;
    cur_x     = s.to_end*( rad[ s.seg_ID ] == 0.0
      ? 1.0
      : fabs( rad[ s.seg_ID ] )
      );
    
    if( max_speed < s.v )
      max_speed = s.v;
    
    use_alpha_factor = 1;
    
    if( s.seg_ID != last_segment )
    {
      segment_change = 1;
      last_segment = s.seg_ID;
      width_reduced = 0;
      last_rad    = rad[ s.seg_ID ];
      last_x      = s.cur_len * ( rad[ s.seg_ID ]==0.0
        ? 1.0
        : fabs(rad[ s.seg_ID ])
        );
      cur_x       = 0.0;
      following_next = 0;
    }
    else
      segment_change = 0;
    
    calculate_max_a();
    
    if( rad[ s.seg_ID ] == 0.0 )
    {
      x = s.to_end;
      
      y = rad[ n(s.seg_ID) ] < 0.0 ? s.to_rgt : s.to_lft;
      y += fabs( rad[ n(s.seg_ID) ] );
      
      to_begin = atan( x/y );
      
      to_inner =  Mag( x, y );
      to_inner -= fabs( rad[ n(s.seg_ID) ] );
      
      alpha   =
        alpha_n =
        - to_begin * sign( rad[ n(s.seg_ID) ] )
        + tangent_alpha( n(s.seg_ID),
        to_begin + len[ n(s.seg_ID) ],
        to_inner,
        d_big,
        R
        );
      
      if( s.to_end > len[ s.seg_ID ] - curve_distance( p(s.seg_ID) ) )
        curveing = 1;
      
      
      outside_lane = ( d_big > 0.0 );
      if( outside_lane ) //t >= r )
      {
        if( ending_speed( s.v,
          cateto( R+d_big, R ),//t*t - r*r ),
          brake_accel( s.seg_ID ),
          e_d )
          >
          corn_spd( n(s.seg_ID) )
          )
        {
          _speed = corn_spd( n(s.seg_ID) );
        };
        
        if( sign( alpha ) == sign( rad[ n(s.seg_ID) ] )) 
        {
          curveing = 1;
          
        }
        else
          slip_correction = 1;//1;
        
      }
      else 
      {
        // too close to inner side
        curveing = 1;
        _speed = speed( R ); 
      }
      
      // calculate alpha_nn
      if( rad[ n(n(s.seg_ID)) ] == 0.0 )
      {
        // Straight Curve Straight
        if( sign( rad[ n(n(n(s.seg_ID)))] ) == sign( rad[ n(s.seg_ID) ] ))
        {
          // Straight & C curve
          d = fabs( rad[     n(s.seg_ID)   ] )
            - fabs( rad[ n(n(n(s.seg_ID))) ] );
          
          x = s.to_end
            + d*sin( len[ n(s.seg_ID) ] )
            + len[ n(n(s.seg_ID)) ]*cos( len[ n(s.seg_ID) ] );
          
          y = rad[ n(s.seg_ID) ] > 0.0 ? s.to_lft : s.to_rgt;
          y += fabs( rad[ n(s.seg_ID) ] );
          y -= d*cos( len[ n(s.seg_ID) ] );
          y += len[ n(n(s.seg_ID)) ]*sin( len[ n(s.seg_ID) ] );
          
          t = Mag( x, y );
          
          to_begin = atan( x/y )
            + len[ n(s.seg_ID) ];
          
          to_inner = t - fabs( rad[ n(n(n(s.seg_ID))) ] );
          
          alpha_nnn = len[ n(s.seg_ID) ] * sign( rad[ n(s.seg_ID) ] )
            - to_begin           * sign( rad[ n(s.seg_ID) ] )
            
            + tangent_alpha( n(n(n(s.seg_ID))),
            to_begin + len[ n(n(n(s.seg_ID))) ],
            to_inner,
            d_big,
            R
            );
          
          if( d_big <= 0.0
            &&
            fabs( radius( n(n(n(s.seg_ID))) ))
            <
            fabs( radius( n(s.seg_ID) ))
            )
          {
            nnn_speed = speed( R );
          };
        }
        else
          ;
      }
      else
      {
        if( sign( rad[ n(n(s.seg_ID)) ] ) != sign( rad[ n(s.seg_ID) ] )
          &&
          len[ n(s.seg_ID) ] < PI*0.5
          )
        { 
          // Straight and S curve
          x = s.to_end
            + ( fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] ))
            * sin( len[ n(s.seg_ID) ] );
          
          y -= ( fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] ))
            * cos( len[ n(s.seg_ID) ] );
          
          to_begin = PI*0.5
            + atan( y/x )
            - len[ n(s.seg_ID) ]
            ;
          to_inner =  Mag( x, y );
          to_inner -= fabs( rad[ n(n(s.seg_ID)) ] );
          
          alpha_nn =
            PI*0.5      * sign( rad[ n(s.seg_ID) ] )
            + atan( y/x ) * sign( rad[ n(s.seg_ID) ] )
            + tangent_alpha( n(n(s.seg_ID)),
            to_begin + len[ n(n(s.seg_ID)) ],
            to_inner,
            d_big,
            R
            );
          if( alpha_nn * sign( rad[ n(s.seg_ID) ] )
            <
            alpha    * sign( rad[ n(s.seg_ID) ] )
            )
          {
            alpha   =
              alpha_n = alpha_nn;
          }
          if( ending_speed( s.v,
            cateto( R+d_big, R ),
            BRAKE_ACCEL,
            e_d )
            >
            speed( R )
            )
            nn_speed = speed( R );
          
          if( sign( rad[ n(n(n(s.seg_ID))) ] ) == sign( rad[ n(s.seg_ID) ] ))
            // Straight and S curve and S curve
            R = R;
        }
        else
        {
          if( sign( rad[ n(n(s.seg_ID)) ] ) == sign( rad[ n(s.seg_ID) ] )
            &&
            len[ n(s.seg_ID) ] < PI*0.5
            )
          {
            // Straight and 2 curves to the same side
            
            d = fabs( rad[   n(s.seg_ID)  ] )
              - fabs( rad[ n(n(s.seg_ID)) ] );
            
            y = s.to_end + d*sin( len[ n(s.seg_ID) ] );
            x = rad[ n(s.seg_ID) ] > 0.0 ? s.to_lft : s.to_rgt;
            x += fabs( rad[ n(s.seg_ID) ] );
            x -= d*cos( len[ n(s.seg_ID) ] );
            
            t = Mag( x, y );
            
            to_begin = atan( y/x )
              + len[ n(s.seg_ID) ];
            
            to_inner = t - fabs( rad[ n(n(s.seg_ID)) ] );
            
            alpha_nn = len[ n(s.seg_ID) ] * sign( rad[ n(s.seg_ID) ] )
              - to_begin           * sign( rad[ n(s.seg_ID) ] )
              
              + tangent_alpha( n(n(s.seg_ID)),
              to_begin + len[ n(n(s.seg_ID)) ],
              to_inner,
              d_big,
              R
              );
            
            if(        sign( alpha_nn ) == sign( rad[ n( s.seg_ID) ] )
              &&
              ( alpha_nn * sign( rad[ n(s.seg_ID) ] )
              <
              alpha    * sign( rad[ n(s.seg_ID) ] )
              ||
              sign( alpha_nn ) == sign( rad[ s.seg_ID ] )
              )
              &&
              len[ n(s.seg_ID) ] < PI * 0.5
              &&
              fabs( rad[ n(s.seg_ID) ] ) > fabs( rad[ n(n(s.seg_ID)) ] )
              &&
              DP[ !forwards ][ n(n(s.seg_ID)) ] < 1.0
              )
            {
              alpha    =
                alpha_n  =
                alpha_nn = alpha_nn;
            }
            
            if( fabs( rad[ n(n(s.seg_ID)) ] )
              <
              fabs( rad[ n(s.seg_ID) ] )
              )
              if( speed( R )
                <
                ending_speed( s.v,
                cateto( R+d_big, R ),
                brake_accel( s.seg_ID ),
                e_d
                )
                )
                nn_speed = speed( R );
              
          }
        }
      }
    }
    else
    {
      // Curve ------------------------------------------------------------
      
      slip_correction = 1;//1;
      // Calculate alpha to the curve's tangent
      alpha = tangent_alpha( s.seg_ID,
        s.to_end,
        rad[ s.seg_ID ] > 0.0 ? s.to_lft : s.to_rgt,
        d_big,
        R );
      
      if( speed( R )
        <
        ending_speed( s.v,
        cateto( R+d_big, R ),
        brake_accel( s.seg_ID ),
        e_d
        )
        )
        _speed = speed( R );
      
      
      if( rad[ n(s.seg_ID) ] == 0.0 )
      {
        // Curve and Straight
        
        if( sign( rad[ s.seg_ID ] ) * sign( rad[ n(n(s.seg_ID)) ] ) == 1
          )
        { 
          double x;
          // Curve straight and curve to the same side.
          // C curve
          
          // Check if next curves's trayectory is visible.
          r =  fabs( rad[ s.seg_ID ] );
          r += rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
          x =  r * sin( s.to_end );
          x += len[ n(s.seg_ID) ];
          y =  r * cos( s.to_end );
          y -= fabs( rad[ s.seg_ID ] );
          y += fabs( rad[ n(n(s.seg_ID)) ] );
          
          to_begin =  PI * 0.5;
          to_begin -= atan( y / x );
          
          to_inner =  Mag( x, y );
          to_inner -= fabs( rad[ n(n(s.seg_ID)) ] );
          
          alpha_nn  = s.to_end * sign( rad[ s.seg_ID ] );
          alpha_nn -= to_begin * sign( rad[ n(n(s.seg_ID)) ] );
          
          alpha_nn += tangent_alpha( n(n(s.seg_ID)),
            to_begin + len[ n(n(s.seg_ID)) ],
            to_inner,
            d_big,
            R
            );
          
          if( speed( R )
            <
            ending_speed( s.v,
            cateto( R+d_big, R ),
            brake_accel( s.seg_ID ),
            e_d
            )
            )
          {
            nn_speed = speed( R );
            alpha = ( alpha + alpha_nn ) * 0.5;
            
          }
          else
            if( alpha_nn * sign( rad[ s.seg_ID ] )
              <
              alpha    * sign( rad[ s.seg_ID ] )
              )
            {
              alpha = ( alpha + alpha_nn ) * 0.5;
            }
        }
        else
        { 
          // curve straight and curve to the other side
          // kind of S curve
          double t,x,y;
          
          t = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
          t += fabs( rad[ s.seg_ID ] );
          x = t * sin( s.to_end )
            + len[ n(s.seg_ID) ];
          y = t * ( 1.0 - cos( s.to_end ))
            + fabs( rad[ s.seg_ID ] ) + real_width - t
            + fabs( rad[ n(n(s.seg_ID)) ] );
          
          to_begin = atan( x/y );
          to_inner = Mag( x, y ) - fabs( rad[ n(n(s.seg_ID)) ] );
          
          alpha_nn = s.to_end * sign( rad[ s.seg_ID ] )
            + atan( y/x ) * sign( rad[ n(n(s.seg_ID)) ] )
            - PI*0.5 * sign( rad[ n(n(s.seg_ID)) ] )
            + tangent_alpha( n(n(s.seg_ID)),
            to_begin + len[ n(n(s.seg_ID)) ],
            to_inner,
            d_big,
            R );
          
          if( alpha_nn * sign( rad[ s.seg_ID ] ) < alpha * sign( rad[ s.seg_ID ] ))
          {
            alpha = alpha_nn;
          }
          if( speed( R )
            <
            ending_speed( s.v,
            cateto( R+d_big, R ),
            brake_accel( s.seg_ID ),
            e_d
            )
            )
            nn_speed = speed( R );
          
        }
      }
      else
      {
        // Curve and Curve
        if( sign( rad[ s.seg_ID ] ) != sign( rad[ n(s.seg_ID) ] ))
        {
          // S curve
          r = fabs( s.cur_rad ) + real_width + fabs( s.nex_rad );
          t = r*sin( s.to_end );
          q = r*cos( s.to_end );
          q -= fabs( s.cur_rad );
          q -= s.cur_rad > 0.0 ? s.to_lft : s.to_rgt;
          r = Mag( t, q );
          d = r
            - fabs( s.nex_rad );
          if( d < 0.0 )
            d = 0.0;
          r -= d;
          
          double alpha_1, alpha_2;
          alpha_2 = acos( q / ( r+d ));
          
          alpha_1 = tangent_alpha( n(s.seg_ID),
            len[ n(s.seg_ID) ]
            - s.to_end
            + alpha_2,
            d,
            d_big,
            R );
          
          alpha_n = 
            + alpha_2
            - fabs( alpha_1 );
          
          alpha_n *= sign( rad[ s.seg_ID ] );
          
          d = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
          d<DIST_FROM_INSIDE?d=DIST_FROM_INSIDE:d;
          if( alpha_n * sign( rad[ s.seg_ID ] ) < alpha * sign( rad[ s.seg_ID ] )
            )
          {
            alpha = alpha_n;
          }
          if( d_big <= 0.0
            ||
            speed( R )
            <
            ending_speed( s.v,
            cateto( R+d_big, R ),
            brake_accel( s.seg_ID ),
            e_d
            )
            )
            n_speed = speed( R );
          
          if( rad[ n(n(s.seg_ID)) ] == 0.0 )
            // S curve and straight
            R=R;
          
          else
            if( sign( rad[ n(n(s.seg_ID)) ] ) != sign( rad[ n(s.seg_ID) ] ))
            {
              // S curve and S curve
              double d,r,t,a,x,y;
              d = (rad[ s.seg_ID ]<0.0?s.to_rgt:s.to_lft);
              r = fabs( rad[ s.seg_ID ] ) + d;
              t = fabs( rad[ s.seg_ID ] ) + real_width + fabs( rad[ n(s.seg_ID) ] );
              x = r*cos( s.to_end );
              
              y = r*sin( s.to_end );
              x = t - x;
              
              r = Mag( x, y );
              a = atan( y/x );
              
              y = r * sin( a + len[ n(s.seg_ID) ] );
              x = r * cos( a + len[ n(s.seg_ID) ] );
              
              t = fabs( rad[ n(s.seg_ID) ] ) + real_width + fabs( rad[ n(n(s.seg_ID)) ] );
              x = t - x;
              
              to_begin = atan( y/x );
              to_inner = Mag( x, y ) - fabs( rad[ n(n(s.seg_ID)) ] );
              
              alpha_nn = s.to_end
                - len[ n(s.seg_ID) ]
                - to_begin
                + fabs( tangent_alpha( n(n(s.seg_ID)),
                to_begin + len[ n(n(s.seg_ID)) ],
                to_inner,
                d_big,
                R ));
              alpha_nn *= sign( rad[ s.seg_ID ] );
              
              d = rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
              d<DIST_FROM_INSIDE?d=DIST_FROM_INSIDE:d;
              
              if( alpha_nn *sign( rad[ s.seg_ID ] )
                >
                alpha_n  *sign( rad[ s.seg_ID ] )
                &&
                alpha_nn *sign( rad[ s.seg_ID ] )
                <
                acos( (fabs(rad[ s.seg_ID ])+DIST_FROM_INSIDE)
                /(fabs(rad[ s.seg_ID ])+d)
                )
                )
              {
                alpha = alpha_nn;
              }
              if( d_big <= 0.0
                ||
                speed( R )
                <
                ending_speed( s.v,
                cateto( R+d_big, R ),
                brake_accel( s.seg_ID ),
                e_d
                )
                )
                nn_speed = speed( R );
              
              else
                // S curve and c curve
                ;
            }            
        }
        else
        {
          // Curve and Curve. Same side.
          // c curve
          
          double x,y,d;
          
          // Check if next curves's trayectory is visible.
          d =  rad[ s.seg_ID ] < 0.0 ? s.to_rgt : s.to_lft;
          r =  d + fabs( rad[ s.seg_ID ] );
          x =  r * sin( s.to_end );
          y =  r * cos( s.to_end );
          y -= fabs( rad[ s.seg_ID ] );
          y += fabs( rad[ n(s.seg_ID) ] );
          
          to_begin =  PI * 0.5;
          to_begin -= atan( y/x );
          
          to_inner =  Mag( x, y );
          to_inner -= fabs( rad[ n(s.seg_ID) ] );
          
          alpha_n  = s.to_end * sign( rad[ s.seg_ID ] );
          alpha_n -= to_begin * sign( rad[ n(s.seg_ID) ] );
          
          alpha_n += tangent_alpha( n(s.seg_ID),
            to_begin + len[ n(s.seg_ID) ],
            to_inner,
            d_big,
            R );
          
          if( ( alpha_n * sign( rad[ n(s.seg_ID) ] ) < alpha * sign( rad[ s.seg_ID ] )
            ||
            sign( alpha_n - s.to_end * sign( rad[s.seg_ID] ) )
            ==
            sign( rad[ s.seg_ID ] )
            )
            &&
            fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] )
            )
          {
            alpha = alpha * 0.3 + alpha_n * 0.7;
          }
          
          if(      speed( R )
            <
            ending_speed( s.v,
            cateto( R+d_big, R ),
            brake_accel( s.seg_ID ),
            e_d
            )
            &&
            fabs( rad[ s.seg_ID ] ) > fabs( rad[ n(s.seg_ID) ] )
            )
          {
            n_speed = speed( R );
            
          }
        }
      }
    }
    
    // set the speed:
    vc = corn_spd( s.seg_ID );
    to_next = t_distance( s.seg_ID )
            * s.to_end/len[ s.seg_ID ]
            - t_distance( s.seg_ID );
    check_segment( _speed, s.seg_ID);
    check_segment( n_speed, n(s.seg_ID));
    check_segment( nn_speed, n(n(s.seg_ID)));
    check_segment( nnn_speed, n(n(n(s.seg_ID))));

    if( final_straight())
    { 
      vc = s.v + 20.0;
      alpha = 0.0;
    }
    
    if( apply_brakes() )
    {
      if( vc > s.v - 10.0 )
        vc = s.v - 10.0; 
    }
    
    if( vc > s.v + 1.0 )
      vc = s.v + ( rad[ s.seg_ID ] == 0.0 ? 20.0 : 5.0 );
    
    if( vc < s.v )
      vc = s.v - 10.0;
    
    if( vc < 5.0 )
      vc = 5.0;
    
    ALPHA_FACTOR = 1.0;
    
    if( s.damage > last_damage )
    {
      if( s.to_rgt > 0*CARWID
        &&
        s.to_lft > 0*CARWID
        )
        ALPHA_FACTOR = 0.0;
    }
    
    if( s.v > 0.1 )
      alpha -= asin( s.vn/s.v );
    
    if( use_alpha_factor )
      alpha *= ALPHA_FACTOR;
    
    fabs( alpha ) > 1.0 ? alpha = 1.0 * sign( alpha ) : alpha;
    
    if( s.vc > s.v
      &&
      s.vc * cos( fabs( s.alpha )) < s.v
      )
    {
      if( slip_correction )
      {
        alpha = acos( s.v / s.vc ) * sign( alpha );
        slip_corrected = 1;
      }
      else
        slip_corrected = 0;
    }
    else
    {
      if( slip_corrected )
        alpha = ( alpha + s.alpha ) / 2.0;
      slip_corrected = 0;
    }
    
    static int count=0;
    const int steps = 1;
    if( vc > s.v )
    {
      vc = s.v + (vc-s.v)/steps*count;
      ++count>steps ? count-- : count;
    }
    else
      count = 0;
    
    if( vc > s.v )
    {
      if( s.nearby[0].who == 999 )
        vc /= cos( alpha );
      else
        vc *= cos( alpha );
    }
    
    if( s.to_rgt < 0.0
      ||
      s.to_lft < 0.0
      )
    { 
      // offroad
      double width = s.to_rgt + s.to_lft;
      alpha = - atan(5./5.) * sign( s.to_rgt - width * 0.5 ); 
      alpha -= asin( s.vn/s.v );   
      if( s.v > vc ) 
        vc = s.v;
    }
    
    result.vc    = vc;
    result.alpha = alpha;
    
    request_pit( result );
    
    if(s.starting)
    {
      if((s.stage==QUALIFYING)||(s.stage==PRACTICE))
        result.fuel_amount = 30.0;
      else
        result.fuel_amount = MAX_FUEL;
    }
    
    last_damage = s.damage;
    last_speed = s.v;
    last_x = s.to_end * (rad[ s.seg_ID ]==0.0?1.0:fabs(rad[ s.seg_ID ]));
    
    return result;
  }
};

Driver * getJppInstance()
{ 
  return new Jpp();
}
