//------------------------------------------------------------------------------
//                                 -=SAD01=-
//
// Robot Name:   SAD01
// Source File Name: sad01_01.cpp
// Authors: Miguel Nicolau & Michael O'Neill
// Emails: miguel.nicolau@ul.ie, michael.oneill@ul.ie
// Version: 0.00 - Very first attempt at a driver!
//          0.01 - slightly modified version of 0.00
// Submitted for: Race 3 (9/04/2000) 
//                - Imola, and Alternate track for F1 2000 
// Data Files: None
// Licence: Free to use source code but please notify authors of its use 
// Compiled under SuSE Linux 6.2 (Kernel 2.2.10) and Redhat Linux 6.0 (Kernel 2.2.14)
//
// Notes: Based heavily on tutorial drivers with an approach to adapt speeds
//        on the bends. Drivers gets around track at a reasonable speed and then 
//        attempts to improve performance. Definitely a work in progress, we 
//        are new to RARS and just wanted to participate in this years F1
//        season as a learning process. Hopefully major improvements will be
//        incorporated into the driver as the season progresses.
//
// Changes: Accelerates out of bends, fuel strategy improvment, modified 
//          learning process (i.e. it stops learning after 33% of race and uses
//          best settings to date).
//
//------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"
#include "track.h"

#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG           0
#define DEBUGL2         0

// Maths constants
#define   PI             3.14159265
#define   RTD            (180.0/PI)
#define   RTANGLE        (90.0/RTD)
#define distfromto(a,b,c,d)  sqrt((a-c)*(a-c) + (b-d)*(b-d))

// Table fields
#define FIELDS           8		//number of fields
#define LENGTH           0		//Length of sement
#define RADIUS           1		//Radius of segment
#define SPEED            2		//Speed for segment
#define TIME             3		//Best time in segment
#define INCS             4		//Inc for speed in segment
#define OUTSPEED         5		//Distance to end when to accel
#define INCD             6		//Inc for outspeed
#define SLIPPED          7		//If car slipped on this segment

#define PIT              10000
#define MAXSLIP          15
#define INCSPEED         1
#define DECSPEED         .5
#define MAXDECSPEED      1.5
#define STRAIGHTSPEED    400
#define TURNSPEED        60
#define MINSPEED         30
#define MAXINCS          .01
#define MAXINCD	         2
#define DEFAULTOUTSPEED	 100

#define BRAKECONST	 .045		//.0549
#define SPDFORCURVECONST 6.0		// 6.0

static double **segs=NULL, **best=NULL;

track_desc track;
static int track_width;
static int NSEGS =       0;		//Number of track segments
static int CUR =         0;		//Number of current segment
static int NEXT =        1;		//Number of next segment
static int PREV =        0;		//Number of previous segment

static void dump_table(){
  for(int i=0;i<NSEGS;i++)
  if(DEBUG)fprintf(stderr,"%d:[%f][%f][%f][%f][%f][%f][%f]\n",i,segs[i][0],segs[i][1],segs[i][2],segs[i][3],segs[i][4],segs[i][5],segs[i][6]);
  }

static int offtrack(double left, double right){
  if((left<0)||(right<0)){
    return 1;
    }
  return 0;
  }

static double spdforcurve(double dist, double radius){
  if (radius < 0.0)
    radius = -radius;
  else if (radius == 0.0)
    return(400.0);
  return SPDFORCURVECONST * sqrt(radius);
  }

static double brake(double vnow, double vthen){
  if(vnow<=vthen)
    return 0.0;
  return BRAKECONST * (vnow+vthen)*(vnow-vthen)/(1.7*2.0);
  }

//START OF RIPOFFS (-:

static double gc_dot, gc_x, gc_y, gc_lastx, gc_lasty;

static double curveradiuslow(int segnum){
  if(track.lftwall[segnum].radius >= 0)
    return track.lftwall[segnum].radius;
  else
    return track.lftwall[segnum].radius + track.width;
  }

static void normalize(double dot, double &angle){
  if (fabs(dot-angle) > fabs(dot-(angle+2*PI)))
    angle+= 2*PI;
  if (fabs(dot-angle) > fabs(dot-(angle-2*PI)))
    angle-= 2*PI;
  }

static double anglefromto(double x, double y, double destx, double desty){
double a;
if (destx == x)
  if (desty < y)
    return RTANGLE;
  else
    return 270/RTD;
  if (destx<x)
    a = atan( (desty-y) / (destx-x) )+PI;
  else
    a = atan( (desty-y) / (destx-x) );
  return a;
  }

static void getabscoord_str(int segnum, double tolft, double toend, double &x, double &y){

  x =track.lftwall[segnum].end_x-toend*cos(track.lftwall[segnum].beg_ang)
   + tolft*cos(track.lftwall[segnum].beg_ang-RTANGLE);
  y =track.lftwall[segnum].end_y-toend*sin(track.lftwall[segnum].beg_ang)
   + tolft*sin(track.lftwall[segnum].beg_ang-RTANGLE);
  }

static void getabsxy(int segnum, double tolft, double torgt, double toend){
  if (track.trackin[segnum].radius == 0){  // straight
  getabscoord_str(segnum, tolft, toend, gc_x, gc_y);
  }
 else{
  double anglefromctr;
  if (track.trackin[segnum].radius > 0){
    //left bend
    anglefromctr = track.trackin[segnum].end_ang - toend - RTANGLE;
    gc_x = track.trackin[segnum].cen_x +
           (tolft+fabs(track.trackin[segnum].radius))*cos(anglefromctr);
    gc_y = track.trackin[segnum].cen_y +
           (tolft+fabs(track.trackin[segnum].radius))*sin(anglefromctr);
    }
    else{
    //right bend
      anglefromctr = track.trackin[segnum].end_ang + toend + RTANGLE;
      gc_x = track.trackin[segnum].cen_x + (torgt+fabs(track.trackin[segnum].radius)-track_width)*cos(anglefromctr);
      gc_y = track.trackin[segnum].cen_y + (torgt+fabs(track.trackin[segnum].radius)-track_width)*sin(anglefromctr);
      }
    }
  gc_dot = anglefromto(gc_lastx, gc_lasty, gc_x, gc_y);
  gc_lastx = gc_x;
  gc_lasty = gc_y;
  }

static double maxangleinturn(double r, double w, double cushion){
  double answer;
  if (r > 0){
    //left bend
    r+= cushion;
    w-= cushion;
    if (w > r+w)
      answer = 1;
    if (w < 0 )
      answer = -.02;       //  if we are inside wall, just move to outside
     else
    if (r+w == 0.0)
      answer = acos(0.0);  //  straight
     else
      answer = acos(1.0-(w/(r+w)));
    }
   else{
    //right bend
    r = -r;
    r+= cushion;
    w-= cushion;
    if (w > r+w)
      answer = 1;
    if (w < 0 )
      //inside wall
      answer = -.02;
     else
    if (r+w == 0.0)
      answer = acos(0.0);
     else
      answer = acos(1.0-(w/(r+w)));
    answer *= -1;  //  we are going the other way around the circle so flip sign
    }
  return answer;
  }

static double angleforcurve(int segnum){
  double angletoctr, disttowall, answer;
  disttowall = distfromto(gc_x, gc_y, track.lftwall[segnum].cen_x, track.lftwall[segnum].cen_y)-fabs(curveradiuslow(segnum));
  angletoctr = anglefromto(gc_x, gc_y, track.lftwall[segnum].cen_x, track.lftwall[segnum].cen_y);
  answer = maxangleinturn(curveradiuslow(segnum), disttowall, 5.0);
  if (track.lftwall[segnum].radius > 0)
    answer = angletoctr-RTANGLE+answer;
   else
    answer = angletoctr+RTANGLE+answer;
  normalize(gc_dot, answer);
  return answer;
  }

static double angleforstraight(double tolft, double toend, int segnum){
double destx, desty, alpha;
getabscoord_str(segnum, tolft, toend, destx, desty);
alpha = anglefromto( gc_x, gc_y, destx, desty);
normalize(gc_dot, alpha);
return alpha;
}

//END OF RIPOFFS (-:

static void init_table(){
  segs=(double**)malloc(track.NSEG*sizeof(double*));
  best=(double**)malloc(track.NSEG*sizeof(double*));
  for(int i=0;i<track.NSEG;i++){
    segs[i]=(double*)malloc(FIELDS*sizeof(double));
    best[i]=(double*)malloc(FIELDS*sizeof(double));
    segs[i][LENGTH]=track.rgtwall[i].length;
    if(track.lftwall[i].radius < 0.0)
      segs[i][RADIUS]=track.rgtwall[i].radius;
    else
      segs[i][RADIUS]=track.lftwall[i].radius;
    if(segs[i][RADIUS]==0)
      segs[i][SPEED]=STRAIGHTSPEED;
//BRAZIL
    //  else if(segs[i][LENGTH]<0.80)
    //  segs[i][SPEED]=spdforcurve(segs[i][LENGTH],segs[i][RADIUS])*2;
//BRAZIL
    else if(segs[i][LENGTH]<0.80)
      segs[i][SPEED]=spdforcurve(segs[i][LENGTH],segs[i][RADIUS])+20;
    else
      segs[i][SPEED]=spdforcurve(segs[i][LENGTH],segs[i][RADIUS]);
    segs[i][TIME]=50000;
    segs[i][INCS]=MAXINCS;
    segs[i][OUTSPEED]=DEFAULTOUTSPEED;
    segs[i][INCD]=MAXINCD;
    }
  if(DEBUG){
    fprintf(stderr,"-------------STARTING-------------\n");
    dump_table();
    }
  }

static void copy_table(double **origin, double **dest){
  for(int i=0;i<track.NSEG;i++)
    for(int j=0;j<FIELDS;j++)
      dest[i][j]=origin[i][j];
  }

double distancetoend(situation &s){
  segment *seg;
  if(s.cur_rad>0)
    seg = track.rgtwall;
  else
    seg = track.lftwall;
  double a = (gc_x-seg[CUR].end_x)*(gc_x-seg[CUR].end_x);
  double b = (gc_y-seg[CUR].end_y)*(gc_y-seg[CUR].end_y);
  return sqrt(a+b);
  }

/* Not used
static void clearlookup(){
  for(int i=0;i<NSEGS;i++){
    for(int j=0;j<FIELDS;j++)
      segs[i][j]=0;
  }
}
*/

/* Not used
static void output_xys(int segnum){
fprintf(stderr,"-----Data for segment %d-----\n",segnum);
fprintf(stderr,"current position:\nx=%f - y = %f\n",gc_x,gc_y);
fprintf(stderr,"center of curve:\ncx = %f - cy = %f\n",track.lftwall[segnum].cen_x, track.lftwall[segnum].cen_y);
fprintf(stderr,"Radius of curve:\ninner = %f - outerr = %f\n",track.lftwall[segnum].radius,track.rgtwall[segnum].radius);
fprintf(stderr,"Beginning of curve(inner):\nx = %f - y = %f\n",track.lftwall[segnum].beg_x,track.lftwall[segnum].beg_y);
fprintf(stderr,"Beginning of curve(outer):\nx = %f - y = %f\n",track.rgtwall[segnum].beg_x,track.rgtwall[segnum].beg_y);
fprintf(stderr,"End of curve(inner):\nx = %f - y = %f\n",track.lftwall[segnum].end_x,track.lftwall[segnum].end_y);
fprintf(stderr,"End of curve(outer):\nx = %f - y = %f\n",track.rgtwall[segnum].end_x,track.rgtwall[segnum].end_y);
//ANGLES
fprintf(stderr,"Beginning angle:\ninner = %f - outer = %f\n",track.lftwall[segnum].beg_ang,track.rgtwall[segnum].beg_ang);
fprintf(stderr,"End angle:\ninner = %f - outer = %f\n",track.lftwall[segnum].end_ang,track.rgtwall[segnum].end_ang);
fprintf(stderr,"Radius:\ninner = %f - outer = %f\n",track.lftwall[segnum].radius,track.rgtwall[segnum].radius);
fprintf(stderr,"-----End of segment %d-----\n",segnum);
  }
*/

///////////////////
// MAIN FUNCTION //
///////////////////
con_vec SAD01(situation &s){
con_vec r;
static int firstcall =    1;
static int initialised =  0;
static int outtrack =     0;
static int crashed =      0;
static unsigned long lastdamage = 0;
static int segment =      0;
static int learned =      0;
static int in_pits =      0;
// static int num_pits =     0;
static int outlap =       0;		//Lap out of pits
static double seg_time =  0;		//time at start of segment
static double lap_fuel =  MAX_FUEL;	//fuel at start of lap
static int fuel_lap_count =     0;
static double consumption =     0;
static double avg_consumption = 0;
static double best_time =       99999;	//best lap time
static int worse_lap =          0;	//bad lap count
double ideal_alpha;
static int firstlap =         1; // is it our first lap?

// FIRST CALL

if(firstcall){
  my_name_is("SAD01");
  firstcall=0;
  r.vc = r.alpha = 0;
  return r;
  }

//GET TRACK AND INITIALIZE TABLE

if ((!initialised) && (s.starting) && (get_track_description().NSEG > 0)){
  track = get_track_description();
  NSEGS = get_track_description().NSEG;
  init_table();
  copy_table(segs,best);
  initialised=1;
  }

// INITIALIZE TIME COUNTER

if((initialised)&&(s.start_time!=0)&&(seg_time==0))
  seg_time = s.start_time;

// CALCULATE SEGMENT NUMBERS

if(initialised){
  CUR = s.seg_ID;
  NEXT = (CUR+1)%NSEGS;
  if(CUR) PREV = CUR-1; else PREV=NSEGS-1;
  firstlap = 0;
  }

// GET ABSOLUTE COORDINATES

getabsxy(CUR, s.to_lft, s.to_rgt, s.to_end);

// CRASHED WITH SOMEONE?

if((!crashed) && (s.damage>lastdamage) && (!offtrack(s.to_lft,s.to_rgt))){
  crashed=1;
  lastdamage=s.damage;
  if(DEBUGL2)fprintf(stderr,"CRASHED!\n");
  }

// SLIPPED?

if(((fabs(s.vn) > MAXSLIP)||(outtrack))&&(segs[CUR][SPEED]>MINSPEED)){
  segs[CUR][SLIPPED]=1;
  if(DEBUGL2)fprintf(stderr,"SLIPPED!\n");
 }

//OFF TRACK?

if((!crashed) && (offtrack(s.to_lft,s.to_rgt))){
  if(DEBUGL2)fprintf(stderr,"OFFTRACK!\n");
  if(!outtrack){
    //decrease previous outspeed
    segs[PREV][OUTSPEED]-=10;
    if(segs[PREV][OUTSPEED]<0)
      segs[PREV][OUTSPEED]=0;
    //decrease CUR's speed
    segs[CUR][SPEED]-=5;
    if(segs[CUR][SPEED]<MINSPEED)
      segs[CUR][SPEED]=MINSPEED;
    }
  outtrack=1;
  r.vc=20;
  if (s.nex_rad > 0)
    r.alpha = angleforstraight(track_width-5.0, 1.0, CUR)-gc_dot;
  else if(s.nex_rad < 0)
    r.alpha = angleforstraight(5.0, 1.0, CUR)-gc_dot;
  else
    r.alpha = angleforcurve(NEXT)-gc_dot;
  return r;
 }

// SPEED

if(((s.stage==RACING) || (s.stage==QUALIFYING))){
  if(segs[CUR][RADIUS]!=0){
// on a bend
    double dte = distancetoend(s);
    if((s.nex_rad==0.0)&&(dte<segs[CUR][OUTSPEED]))
      r.vc = s.v + (segs[CUR][OUTSPEED]-dte);
    else{
//      if(outlap == s.laps_done){
//        if((!crashed)&&(!learned)){
//          if(((fabs(s.vn) > MAXSLIP)||(outtrack))&&(segs[CUR][SPEED]>MINSPEED)){
//          // sliping
//            if(outtrack)
//              segs[CUR][SPEED]-=MAXDECSPEED;
//            else
//              segs[CUR][SPEED]-=DECSPEED;
//            learned=1;
//            }
//          else if((fabs(s.vn) < MAXSLIP)&&(!outtrack)){
//          // not sliping
//            if(s.v>=segs[CUR][SPEED]-20){
//              segs[CUR][SPEED]+=INCSPEED;
//              learned=1;
//              }
//            }
//          }
//        }
        if(s.v < segs[CUR][SPEED]-10)
          r.vc = segs[CUR][SPEED]+5;
        else
          r.vc = segs[CUR][SPEED];
      }
    }
  else{
//straight
    r.vc = 400;
    }
  }


// BRAKING
  if(!segs[CUR][RADIUS]){
  //straight
    if((s.to_end <= brake(s.v,segs[NEXT][SPEED]))&&(s.v>segs[NEXT][SPEED]))
//      r.vc=0;
      r.vc=segs[NEXT][SPEED];
    }
  else if(segs[NEXT][RADIUS]){
  //bend and next one is a bend as well
    double dte = distancetoend(s);
    if(dte<brake(s.v,segs[NEXT][SPEED]))
      r.vc = segs[NEXT][SPEED];
    }

// DIRECTION

/* double track_middle=(s.to_lft+s.to_rgt)/2;
double diroftravel = asin(s.vn/s.v);
double ideal_alpha = atan((s.to_lft-track_middle)/s.to_end);
if(s.cur_rad !=0)
  ideal_alpha = asin((s.to_lft - track_middle) / (s.to_end + fabs(s.cur_rad)));
r.alpha = ideal_alpha - diroftravel; */

if(s.stage==3 || s.stage==QUALIFYING){
//  getabsxy(CUR, s.to_lft, s.to_rgt, s.to_end);
  track_width = (int) (s.to_lft+s.to_rgt);

  if (s.cur_rad==0.0){
  //straight
//    if((s.cur_len>500)&&
//       ((s.to_end>300)&&(s.to_end<(s.cur_len-100)))){
//      //stick to outside
//      if (s.nex_rad > 0)
//        ideal_alpha = angleforstraight(track_width-5.0, 1.0, CUR);
//      if (s.nex_rad < 0)
//        ideal_alpha = angleforstraight(5.0, 1.0, CUR);
//      }
//    else{
//      //stick to inside
//      if (s.nex_rad > 0)
//        ideal_alpha = angleforstraight(5.0, 1.0, CUR);
//      if (s.nex_rad < 0)
//        ideal_alpha = angleforstraight(track_width-5.0, 1.0, CUR);
      ideal_alpha = angleforcurve(NEXT);
//      }
    }
  else{
  //bend
    ideal_alpha = angleforcurve(CUR);
    if (curveradiuslow(CUR) * curveradiuslow(NEXT)<0){
      double nextcurvealpha = angleforcurve(NEXT);
      if (((curveradiuslow(CUR)>0) && (nextcurvealpha<ideal_alpha))
          ||((curveradiuslow(CUR)<0) && (nextcurvealpha>ideal_alpha)))
        ideal_alpha = nextcurvealpha;
      }
    }
  r.alpha = ideal_alpha - gc_dot;
  }


// Adapt speed to turning angle
// fprintf(stderr,"(%f)\n",r.alpha);
// if( !(firstlap) && (s.cur_rad!=0) && (abs((int)r.alpha) > 1) ){
//   r.vc+=25;
// }
 if(segs[CUR][RADIUS]!=0){
   //fprintf(stderr,"(%f)\n",fabs(segs[CUR][RADIUS]));
   if(fabs(segs[CUR][RADIUS])<30){ /*?30?*/
     r.vc+=50;
   }
 }


// PIT STRATEGY

if(s.starting) r.fuel_amount = MAX_FUEL;
r.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > PIT || s.fuel < avg_consumption+3))
  {
  r.request_pit = 1;
  r.repair_amount=s.damage;
  if(s.laps_to_go*avg_consumption < MAX_FUEL)
    r.fuel_amount = s.laps_to_go*avg_consumption + s.laps_to_go*2;
  else
    r.fuel_amount = MAX_FUEL;
  }
 if(s.out_pits==1){             // LEAVE PITS SLOWLY AND STRAIGHT
    r.vc = track.pit_speed;
    r.alpha = 0;
    in_pits = 1;
    outlap = s.laps_done;
    lap_fuel=s.fuel;
    if(DEBUGL2)fprintf(stderr,"Leaving pits on: %d\n",outlap);
  }
 if(in_pits > 0){          // ON LEAVING PITS CENTER CAR SLOWLY
    in_pits--;
    r.vc = track.pit_speed;
  }

// LEARNING

if((!learned)&&(outlap<s.laps_done)&&(CUR!=segment)){
//beginning of new segment
  int curveb4 = PREV-1;
  if(curveb4<0) curveb4=NSEGS-1;
  double time = s.time_count - seg_time - s.start_time;
  if((time<segs[PREV][TIME]) || (!segs[PREV][SLIPPED])){
  //better time on the finished segment than before,
  //or worse time but we didn't slipped
    segs[PREV][TIME] = time;
    if(segs[PREV][RADIUS])
    //previous was a bend, so evolve speed inside curve
      segs[PREV][SPEED]+=segs[PREV][INCS];
    else{
    //previous was a straight, so evolve outspeed on curve before that
      segs[curveb4][OUTSPEED]+=segs[curveb4][INCD];
      }
    }
  else{
  //worse time than before or we slipped
    //evolve both speed inside curve and outspeed
    segs[PREV][SPEED]-=segs[PREV][INCS];
    segs[PREV][INCS]=segs[PREV][INCS]*3/4;
    segs[PREV][OUTSPEED]-=segs[PREV][INCD];
    segs[PREV][INCD]=segs[PREV][INCD]*3/4;
    }
  segs[curveb4][SLIPPED]=0;
  }

//CHANGING SEGMENT

if(CUR!=segment){
//  if(DEBUG)fprintf(stderr,"%d:[%f][%f][%f][%f][%f][%f][%f]\n",segment,segs[segment][0],segs[segment][1],segs[segment][2],segs[segment][3],segs[segment][4],segs[segment][5],segs[segment][6]);
  segment=CUR;
  crashed=0;
  outtrack=0;
  seg_time = s.time_count;
  }


//CHANGING LAP

if(s.lap_flag){
  if( (s.laps_done)&&(outlap<s.laps_done) ){
    fuel_lap_count = fuel_lap_count+1; 
    consumption = consumption + (lap_fuel-s.fuel);
    avg_consumption = consumption / fuel_lap_count;
    lap_fuel=s.fuel;
    }
  if((s.lap_time)&&(s.lap_time<best_time)){
  //better time than before
    copy_table(segs,best);
    best_time=s.lap_time;
    worse_lap=0;
    }
  else if(s.lap_time>best_time){
  //worse time than before
    worse_lap++;
    if(worse_lap>5){
      copy_table(best,segs);
      worse_lap=0;
      }
    }
  if((!learned) && (s.laps_to_go<s.laps_done*3)){
    learned=1;
    copy_table(best,segs);
    }
  }

//DUMPING STATE

if((DEBUG)&&(s.damage>=30000)){
  fprintf(stderr,"--------------DEAD!---------------\n");
  dump_table();
  }

if((s.stage==FINISHED)&&(DEBUG)){
  fprintf(stderr,"-------------FINISHED-------------\n");
  dump_table();
  }

return r;
}

