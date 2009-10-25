/*
 * SmoothB2.cpp
 * Dennis Lew (av504@torfree.net)
 *
 * ROBOT       : SmoothB2
 * RACES       : Can be used for the remaining races.
 * CONFIDENTIAL: No
 * DATA FILE   : "SmoothB.dat" (or whatever DATA_FILE is set to)
 *               Contains data for: -cstlcomb.trk
 *                                  -hock.trk
 *                                  -hungary.trk
 *
 * Uses the OOP structure, call 'getSmoothBInstance()' to get a
 * new instance.  All instances of SmoothB in a race will share
 * the path data.
 *
 * SmoothB will try and create its own path around the track its
 * on.  The line is o.k, but there are problems with certain
 * combinations of sections.  It will make it around any track
 * with a decent pace (with no data file), but for maximum
 * performance I need to create a better path than its own.  The
 * path (in data file) is created manually using another program
 * I have made.
 *
 * If you want to use any part of this code in your own robot, or
 * are curious about anything I've done here, just let me know.
 *
 * <CHANGES>
 * v1.4: -fixed NULL pointer bug when trying to load data
 *       -improved collision detection
 *       -robot will not actively try to pass other cars (UNDER DEVELOPMENT)
 *       -robot should be able to reliably finish races now
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "os.h"
#include "car.h"
#include "track.h"

//#define SB_TESTING 1

// CONSTANTS
static const char   *SB_AUTHOR     = "Dennis Lew (av504@torfree.net)";
static const char   *SB_DESCRIPTION= "v1.4: Aug 17, 2001";
static const char   *SB_NAME1      = "SmoothB";
static const char   *SB_NAME2      = "SmoothB%d";
static const int     SB_NAME_LEN   = 10;
static const int     SB_NOSE       = oLIGHTGRAY;
static const int     SB_TAIL       = oGREEN;
static const char   *SB_BITMAP2D   = "car_gray_green";
// static const char   *SB_BITMAP3D   = "";

static const char   *DATA_FILE     = "SmoothB.dat";
static const double  MY_PI         = 3.14159265358979323846;
static const double  MY_PI21       = 2.0 * MY_PI;
static const double  MY_PI12       = 0.5 * MY_PI;
static const double  MY_PI14       = 0.25 * MY_PI;
static const double  SCALE_CORNSPD = 0.875;
static const double  SB_FC         = 6.0 / (5.0 * 5280.0);
static const int     PTS           = 1;
static const double  VIEW_ANG      = 0.45 * MY_PI;
static const double  SB_DT         = 1.95 * delta_time;
static const double  SB_SAFE_TIME  = 5.0;
static const int     SB_PASS_COUNT = 60;
static const double  SB_IN_WID     = 2.0;
static const double  SB_OUT_WID    = 5.5;
static const int     SB_NO_PASSING = 1;   // make non-zero to prevent passing

// GLOBALS
static int    SURFACE_TYPE =  1;
static double MAX_ACCEL    = 32.2;


// STATIC FUNCTIONS
//-----START spd_func.h-----//
static double crn_spd(double radius);
static double crn_RAD2FT(double len, double rad, double wid);

static double d_brk(double v);
static double brkdist(double vi, double vf);
//-----END spd_func.h-----//


// CLASSES
//-----START Vector.h-----//
class SBVector {
public:
   // MEMBERS
   double x, y;

   // CONSTRUCTORS
   SBVector();
   SBVector(double vx, double vy);
   SBVector(double angle);
   SBVector(const SBVector &v);

   // METHODS
   void   add(const SBVector &v);
   void   sub(const SBVector &v);
   double mag();
   double mag2();
   double ang();
   void   scale(double factor);
   void   normalize();
   double dot(const SBVector &v);
   void   rotate(double angle);
};
//-----END Vector.h-----//

//-----START Line.h-----//
class SBLine {
public:
   // MEMBERS
   SBVector pt;
   SBVector dir;

   // CONSTRUCTORS
   SBLine();
   SBLine(double px, double py, double dx, double dy);
   SBLine(double px, double py, double d_ang);

   // METHODS
   SBVector *intersect(SBLine &l);
   void      intersect(SBVector &np, SBLine &l);
   SBVector *getPoint(double dist);
   void      getPoint(SBVector &np, double dist);
};
//-----END Line.h-----//

//-----START Circle.h-----//
class SBCircle {
public:
   // MEMBERS
   SBVector c;
   double   R;

   // CONSTRUCTORS
   SBCircle();
   SBCircle(double rad);
   SBCircle(double cx, double cy, double rad);

   // METHODS
   SBVector *intersect(SBLine &l);
   SBVector *intersect_F(SBLine &l);
   void      intersect_F(SBVector &np, SBLine &l);
   void      tangent2(SBLine &s, SBLine &f, SBCircle &a);
   SBVector *getPoint(double angle);
   void      getPoint(SBVector &np, double angle);

private:
   // PRIVATE METHODS
   int intersect(SBLine &l, double *d1, double *d2);
};
//-----END Circle.h-----//

//-----START PathComponent.h-----//
class SBPathComponent {
public:
   // MEMBERS
   int    type;           // type of seg this component is on (1,0,-1)
   double len, wid, rad;  // length, width and radius of this seg
   double Sl, Sw;         // start position
   double Fl, Fw;         // finish position

   // CONSTRUCTORS
   SBPathComponent();

   // METHODS
           void   set_seg_data(double l, double w, double r);
   virtual void   construct();
   virtual double to_side(double pos);
   virtual double get_nex_pos(SBLine &cp, SBVector &np,
                              SBPathComponent &nex);
};
//-----END PathComponent.h-----//

//-----START Straight.h-----//
class SBStraight : public SBPathComponent {
public:
   // MEMBERS
   SBLine l;

   // CONSTRUCTORS
   SBStraight();

   // METHODS
   void   construct();
   double to_side(double pos);
   double get_nex_pos(SBLine &cp, SBVector &np, SBPathComponent &nex);

private:
   // PRIVATE METHODS
   void   construct_S();
   void   construct_T();
   double to_side_S(double pos);
   double to_side_T(double pos);
};
//-----END Straight.h-----//

//-----START Turn.h-----//
class SBTurn : public SBPathComponent {
public:
   // MEMBERS
   SBCircle curv;
   double   Aw;    // Apex position

   // CONSTRUCTORS
   SBTurn();

   // METHODS
   void   construct();
   double to_side(double pos);
   double get_nex_pos(SBLine &cp, SBVector &np, SBPathComponent &nex);

private:
   // PRIVATE METHODS
   void   construct_S();
   void   construct_T();
   double to_side_S(double pos);
   double to_side_T(double pos);
};
//-----END Turn.h-----//

//-----START DrvPath.h-----//
class SBDrvPath {
public:
   // MEMBERS
   // Driving path parameters
   int               seg_type;  // -1 right, 0 straight, 1 left
   int               num;       // number of components in this segment
   SBPathComponent **pc;
   double            spd, spd_cur;   // spd parameters
   double            apex;

   // CONSTRUCTORS
   SBDrvPath(int type, int n);

   // DESTRUCTOR
   ~SBDrvPath();

   // METHODS
          void             construct();
          SBPathComponent *get_PC(double pos);
   static void             best_path(SBTurn &cs, SBTurn &ns, SBTurn &as);
   static void             adjust_path(SBTurn &ps, SBTurn &cs, SBTurn &ns, SBTurn &as);

private:
   // PRIVATE METHODS
          void construct_S();
          void construct_T();
   static void Update_cs(SBTurn &cs, SBTurn &ns);
   static void Update_as(SBTurn &as, SBTurn &ns);
};
//-----END DrvPath.h-----//

//-----START TrkData.h-----//
class SBTrkData {
public:
   // MEMBERS
   char       *name;
   SBDrvPath **path;
   int         nseg;

   // CONSTRUCTORS
   SBTrkData();

   // DESTUCTOR
   ~SBTrkData();

   // METHODS
   void set_path(char *n, track_desc *trk);
   void saveData(const char *fname, const int ver);
   int  loadData(const char *fname);

private:
   // PRIVATE METHODS
   void saveTXTData(const char *fname);
   int  loadTXTData(const char *fname);
   void saveBINData(const char *fname);
   int  loadBINData(const char *fname);
   void saveTurn(FILE *fp, SBTurn &t);
   void saveStraight(FILE *fp, SBStraight &t);
};
//-----END TrkData.h-----//

//-----START SBDriver.h-----//
// GLOBALS
static int        trk_num = -1;
static SBTrkData  trk_data;
static track_desc trk_info;

class SmoothB : public Driver
{
public:
   // CONSTRUCTORS
   SmoothB();
   SmoothB(int i);

   // DESTRUCTOR
   ~SmoothB();

   // METHODS
   con_vec drive(situation &s);

private:
   // PRIVATE MEMBERS
   char    *name;
   int      init;
   con_vec  result;
   con_vec *cp;
   int      lastpit;
   double   fuelstart;
   double   err;
   int      passTime;
#ifdef SB_TESTING
   int      SBnum;
#endif

   // PRIVATE METHODS
   void   get_RARS_ctrl_params(situation &s);
   double fuel_needed(Stage s, int lapsleft, double fpl);
   double car_mass(double fuel);
   double u(double slip);
   double iu(double cur_u);
   void   update_con_vec(SBVector &cV, SBVector &nV);
   void   get_pos(SBPathComponent &pc, double Pl, double Pw,
                  SBVector &pos);
   void   get_vel(SBPathComponent &pc, double v, double vn, double Pl,
                  SBVector &vel);
   double get_tgt_v(situation &s);
   int    anti_collision(SBLine &cV, SBVector &nV, situation &s,
                         double path, int no_pass);
   void   passing_action(situation &s, double lane, double bias, int trouble);
};
//-----END SBDriver.h-----//


//-----START Vector.cpp-----//
// CONSTRUCTORS
SBVector::SBVector()
{
   x = y = 0.0;
}

SBVector::SBVector(double vx, double vy)
{
   x = vx;
   y = vy;
}

SBVector::SBVector(double angle)
{
   x = cos(angle);
   y = sin(angle);
}

SBVector::SBVector(const SBVector &v)
{
   x = v.x;
   y = v.y;
}

// METHODS
void SBVector::add(const SBVector &v)
{
   x += v.x;
   y += v.y;
}

void SBVector::sub(const SBVector &v)
{
   x -= v.x;
   y -= v.y;
}

double SBVector::mag()
{
   return sqrt((x*x) + (y*y));
}

double SBVector::mag2()
{
   return ((x*x) + (y*y));
}

double SBVector::ang()
{
   double retval;

   if (x != 0.0)
      retval = atan(y / x);
   else {
      if (y >= 0.0)
         retval =  MY_PI12;
      else
         retval = -MY_PI12;
   }

   if (x < 0.0)                       // Quad 2, Quad 3
      retval += MY_PI;
   else if ( (x >= 0.0)&&(y < 0.0) )  // Quad 4
      retval += MY_PI21;

   return retval;
}

void SBVector::scale(double factor)
{
   x *= factor;
   y *= factor;
}

void SBVector::normalize()
{
   double m;

   m = mag();
   if (m != 0.0)
      scale(1.0 / m);
}

double SBVector::dot(const SBVector &v)
{
   return (x*v.x) + (y*v.y);
}

void SBVector::rotate(double angle)
{
   double m, a;

   m = mag();
   a = ang();
   x = m * cos(a + angle);
   y = m * sin(a + angle);
}
//-----END Vector.cpp-----//

//-----START Line.cpp-----//
// CONSTRUCTORS
SBLine::SBLine()
   : pt(), dir()
{
}

SBLine::SBLine(double px, double py, double dx, double dy)
   : pt(px, py), dir(dx, dy)
{
   dir.normalize();
}

SBLine::SBLine(double px, double py, double d_ang)
   : pt(px, py), dir(d_ang)
{
}

// METHODS
SBVector *SBLine::intersect(SBLine &l)
{
   SBVector *ret;

   ret = new SBVector(pt);
   intersect(*ret, l);
   return ret;
}

void SBLine::intersect(SBVector &np, SBLine &l)
{
   double N, D;

   N = (dir.x*(l.pt.y - pt.y)) - (dir.y*(l.pt.x - pt.x));
   D = (l.dir.x*dir.y) - (l.dir.y*dir.x);

   if (D == 0.0) {
      np.x = pt.x;
      np.y = pt.y;
      return;
   }

   l.getPoint(np, (N/D));
}

SBVector *SBLine::getPoint(double dist)
{
   SBVector *ret;

   ret = new SBVector();
   getPoint(*ret, dist);
   return ret;
}

void SBLine::getPoint(SBVector &np, double dist)
{
   np.x = pt.x + (dist*dir.x);
   np.y = pt.y + (dist*dir.y);
}
//-----END Line.cpp-----//

//-----START Circle.cpp-----//
// CONSTRUCTORS
SBCircle::SBCircle()
   : c()
{
   R = 0.0;
}

SBCircle::SBCircle(double rad)
   : c()
{
   R = rad;
}

SBCircle::SBCircle(double cx, double cy, double rad)
   : c(cx, cy)
{
   R = rad;
}

// METHODS
SBVector *SBCircle::intersect(SBLine &l)
{
   SBVector *ret;
   double    E, F;
   double    d1, d2;
   int       rc;

   d1 = d2 = 0.0;
   rc = intersect(l, &d1, &d2);

   if (rc == 0) {
      ret = new SBVector(c);
      return ret;
   }
   else if (rc == 1) {
      // Only 1 solution
      ret = l.getPoint(d1);
      return ret;
   }

   E = l.pt.y + (d1 * l.dir.y);
   F = l.pt.y + (d2 * l.dir.y);
   if (  ( (E >= 0.0)&&(F >= 0.0) )||( (E < 0.0)&&(F < 0.0) )  ) {
      // both points are on same side of x axis
      // choose closest point
      if (fabs(d1) < fabs(d2))
         ret = l.getPoint(d1);
      else
         ret = l.getPoint(d2);
   }
   else if (E >= 0.0)
      ret = l.getPoint(d1);
   else
      ret = l.getPoint(d2);

   return ret;
}

SBVector *SBCircle::intersect_F(SBLine &l)
{
   SBVector *ret;

   ret = new SBVector();
   intersect_F(*ret, l);
   return ret;
}

void SBCircle::intersect_F(SBVector &np, SBLine &l)
{
   double d1, d2;
   int    rc;

   d1 = d2 = 0.0;
   rc = intersect(l, &d1, &d2);

   if (rc == 0) {
      np.x = c.x;
      np.y = c.y;
      return;
   }
   else if (rc == 1) {
      // Only 1 solution
      l.getPoint(np, d1);
      return;
   }

   // choose closest point
   if (fabs(d1) < fabs(d2))
      l.getPoint(np, d1);
   else
      l.getPoint(np, d2);
}

void SBCircle::tangent2(SBLine &s, SBLine &f, SBCircle &a)
{
   SBVector I_p, O_p, *A_p;
   double   d;
   SBLine   I;

   f.dir.rotate(MY_PI12);
   if (f.dir.x != -1.0)
      d = (s.pt.x - f.pt.x) / (1.0 + f.dir.x);
   else
      d = 0.0;

   if (d <= (f.pt.mag() - 2.0)) {
      // this situation causes problems, so must adjust (s) and (f)
      d = 0.5 * (s.pt.mag() + f.pt.mag());
      s.pt.normalize(); s.pt.scale(d);
      f.pt.normalize(); f.pt.scale(d);
   }
   f.getPoint(O_p, d);

   f.dir.rotate(-MY_PI12);
   if (fabs(s.dir.ang()-f.dir.ang()) > 0.0436332)
      s.intersect(I_p, f);
   else  { // s and f are parallel
      I_p.x = O_p.x - (10.0*s.dir.x);
      I_p.y = O_p.y - (10.0*s.dir.y);
   }

   I.pt.add(I_p);
   I.dir.add(O_p);
   I.dir.sub(I_p);
   I.dir.normalize();

   if (fabs(s.pt.mag()-a.R) <= 5.0)
      A_p = new SBVector(f.pt);
   else if (fabs(f.pt.mag()-a.R) <= 5.0)
      A_p = new SBVector(f.pt);
   else {
      A_p = a.intersect(I);
      if (A_p->ang() > f.pt.ang()) {
         A_p->x = f.pt.x; A_p->y = f.pt.y;
      }
   }

   I.dir.x = O_p.x; I.dir.y = O_p.y;
   I.dir.sub(*A_p);
   I.dir.normalize();

   if (I.dir.x != -1.0)
      d = (s.pt.x - A_p->x) / (1.0 + I.dir.x);
   else
      d = 0.0;

   R   = d;
   c.x = A_p->x + (d * I.dir.x);
   c.y = A_p->y + (d * I.dir.y);

   I_p.x = R - c.mag();
   if (I_p.x < a.R) {
      // Adjust turn
      I_p.y = a.R - I_p.x;
      I_p.x = c.mag();
      d     = I_p.x / I_p.y;
      if (d > 2.0)
         d = 2.0;
      I_p.x -= d * I_p.y;
      d     -= 1.0;
      R     -= d * I_p.y;
      c.normalize();
      c.scale(I_p.x);
   }

   delete A_p;
}

SBVector *SBCircle::getPoint(double angle)
{
   SBVector *ret;

   ret = new SBVector();
   getPoint(*ret, angle);
   return ret;
}

void SBCircle::getPoint(SBVector &np, double angle)
{
   np.x = c.x + (R * cos(angle));
   np.y = c.y + (R * sin(angle));
}


// PRIVATE METHODS
int SBCircle::intersect(SBLine &l, double *d1, double *d2)
{
   SBVector dif(l.pt);
   double   E, F, G, det;

   dif.sub(c);

   E = l.dir.mag() * l.dir.mag();
   F = 2.0 * dif.dot(l.dir);
   G = (dif.mag() * dif.mag()) - (R * R);

   det = (F*F) - (4.0*E*G);
   if (  (det < 0.0)||( (E == 0.0)&&(F == 0.0) )  ) {
      return 0;
   }

   if (E == 0.0) {
      // Only 1 solution
      *d1 = -G / F;
      return 1;
   }

   *d1 = (-F + sqrt(det)) / (2.0*E);
   *d2 = (-F - sqrt(det)) / (2.0*E);
   return 2;
}
//-----END Circle.cpp-----//

//-----START PathComponent.cpp-----//
// CONSTRUCTORS
SBPathComponent::SBPathComponent()
{
   type = 0;
   len  = wid = rad = 0.0;
   Sl   = Sw  = Fl  = Fw  = 0.0;
}

// METHODS
void SBPathComponent::set_seg_data(double l, double w, double r)
{
   len = l;
   wid = w;
   rad = fabs(r);  // all turns treated as left turns

   if (r == 0.0)
      type =  0;  // straight
   else if (r > 0.0)
      type =  1;  // left turn
   else
      type = -1;  // right turn
}

void SBPathComponent::construct()
{
}

double SBPathComponent::to_side(double pos)
{
   return 0.0;
}

double SBPathComponent::get_nex_pos(SBLine &cp, SBVector &np,
                                    SBPathComponent &nex)
{
   return 0.0;
}
//-----END PathComponent.cpp-----//

//-----START Straight.cpp-----//
// CONSTRUCTORS
SBStraight::SBStraight()
   : SBPathComponent(), l()
{
}

// METHODS
void SBStraight::construct()
{
   if (type == 0)
      construct_S();
   else
      construct_T();

   // make sure S/F positions are in the segment
   if (Sl < 0.0) {
      Sl = 0.0;
      Sw = to_side(Sl);
   }
   else if (Sl > len) {
      Sl = len;
      Sw = to_side(Sl);
   }

   if (Fl > len) {
      Fl = len;
      Fw = to_side(Fl);
   }
   else if (Fl < 0.0) {
      Fl = 0.0;
      Fw = to_side(Fl);
   }

   if (type == 0)
      construct_S();
   else
      construct_T();
}

double SBStraight::to_side(double pos)
{
   if (type == 0)
      return to_side_S(pos);

   return to_side_T(pos);
}

double SBStraight::get_nex_pos(SBLine &cp, SBVector &np, SBPathComponent &nex)
{
   double MAX_dA = acos(1.0 - (0.048/cp.dir.mag2()));
   double dp     = np.x * delta_time;
   double pos, ideal, err_pct, cur_dir;

   MAX_dA = (MAX_dA > 0.0035) ? 0.0035 : MAX_dA;

   // find ideal direction and error pct
   cur_dir = cp.dir.ang();
   ideal   = l.dir.ang() - cur_dir;
   if (ideal > MY_PI)
      ideal -= MY_PI21;
   else if (ideal < -MY_PI)
      ideal += MY_PI21;
   pos     = to_side(cp.pt.y);
   err_pct = fabs(cp.pt.x - pos) / wid;
   err_pct = (err_pct > 1.0) ? 1.0 : err_pct;

   // find target position
   np.y = cp.pt.y + (l.dir.y * dp);
   if (np.y <= Fl)   // target is in this component
      np.x = pos + (l.dir.x * dp);
   else {            // target is in next component
      if (nex.type == 0) {
         pos  = nex.Sl + (np.y - Fl);
         np.x = nex.to_side(pos);
      }
      else {
         pos  = nex.Sl + ((np.y - Fl) / nex.rad);
         np.x = nex.to_side(pos) + nex.rad;
         np.y = 0.0;
         np.rotate(pos);
         if (nex.type == 1) {
            // left turn
            np.x -= nex.rad;
            np.y += Fl;
         }
         else {
            // right turn
            np.x  = wid + nex.rad - np.x;
            np.y += Fl;
         }
      }
   }

   // limit the change in direction
   np.sub(cp.pt);
   pos = np.ang() - cur_dir;
   if (pos > MY_PI)
      pos -= MY_PI21;
   else if (pos < -MY_PI)
      pos += MY_PI21;
   pos *= err_pct;
   pos += (1.0 - err_pct) * ideal;

   if (pos > MAX_dA)
      pos =  MAX_dA;
   else if (pos < -MAX_dA)
      pos = -MAX_dA;

   np.x = dp;
   np.y = 0.0;
   np.rotate(cur_dir + pos);
   return -err_pct;
}

// PRIVATE METHODS
void SBStraight::construct_S()
{
   double Ml, Mw, a;

   l.pt.x  = Sw;
   l.pt.y  = Sl;
   l.dir.x = Fw - l.pt.x;
   l.dir.y = Fl - l.pt.y;

   a = l.dir.ang() - MY_PI12;
   if ( (fabs(a) > 0.2618)&&(Sl != Fl) ) {
      // Sw and Fw are too far apart
      Ml = 0.5*(Sl + Fl);
      Mw = to_side_S(Ml);

      l.pt.x  = Mw;
      l.pt.y  = Ml;
      l.dir.x = 0.0;
      l.dir.y = 1.0;
      if (a > 0.0)
         l.dir.rotate( 0.2618);
      else
         l.dir.rotate(-0.2618);

      Sw = to_side_S(Sl);
      Fw = to_side_S(Fl);
   }
   else
      l.dir.normalize();
}

void SBStraight::construct_T()
{
   l.pt.x  = (Sw + rad) * cos(Sl);
   l.pt.y  = (Sw + rad) * sin(Sl);
   l.dir.x = ((Fw + rad) * cos(Fl)) - l.pt.x;
   l.dir.y = ((Fw + rad) * sin(Fl)) - l.pt.y;
   l.dir.normalize();
}

double SBStraight::to_side_S(double pos)
{
   double    ret;
   SBVector *i;
   SBLine    ln(0.0, pos, 1.0, 0.0);

   i = ln.intersect(l);
   i->sub(ln.pt);
   ret = i->mag();

   delete i;
   return ((ret < SB_OUT_WID)?SB_OUT_WID:((ret > (wid-SB_OUT_WID))?(wid-SB_OUT_WID):ret));
}

double SBStraight::to_side_T(double pos)
{
   double    ret;
   SBVector *i;
   SBLine    ln(rad*cos(pos), rad*sin(pos), pos);

   i = ln.intersect(l);
   i->sub(ln.pt);
   ret = i->mag();

   delete i;
   return ((ret < SB_OUT_WID)?SB_OUT_WID:((ret > (wid-SB_OUT_WID))?(wid-SB_OUT_WID):ret));
}
//-----END Straight.cpp-----//

//-----START Turn.cpp-----//
// CONSTRUCTORS
SBTurn::SBTurn()
   : SBPathComponent(), curv()
{
   Aw = 0.0;
}

// METHODS
void SBTurn::construct()
{
   if (type == 0)
      construct_S();
   else
      construct_T();
}

double SBTurn::to_side(double pos)
{
   if (type == 0)
      return to_side_S(pos);

   return to_side_T(pos);
}

double SBTurn::get_nex_pos(SBLine &cp, SBVector &np, SBPathComponent &nex)
{
   double MAX_dA  = 2.0 * ((cp.dir.mag() * delta_time) / curv.R);
   double dp      = np.x * delta_time;
   int    left    = 1;
   int    trouble = 0;
   double pos, ideal, err_pct, cur_dir;

   MAX_dA = (MAX_dA < 0.0075) ? 0.0075 : MAX_dA;

   // find error pct
   cur_dir = cp.dir.ang();
   if (type) {
      pos     = to_side(cp.pt.ang());
      err_pct = fabs(cp.pt.mag() - rad - pos) / wid;
   }
   else {
      pos     = to_side(cp.pt.y);
      err_pct = fabs(cp.pt.x - pos) / wid;
   }
   err_pct = (err_pct > 1.0) ? 1.0 : err_pct;

   // find target position
   if (type) {
      // turn
      if ((pos+CARWID+rad) < cp.pt.mag())
         trouble = 1;
      np.x = cp.pt.x - curv.c.x;
      np.y = cp.pt.y - curv.c.y;
      if (Aw < 0.0) {
         left = 0;
         np.rotate(MY_PI);
      }
      np.normalize();
      np.scale(curv.R);
      ideal = np.ang() + MY_PI12;
      np.rotate(dp / curv.R);
      np.add(curv.c);
   }
   else {
      // straight
      np.x = pos - curv.c.x;
      np.y = cp.pt.y - curv.c.y;
      if (curv.c.x <= 0.0) {  // left turn
         if ((pos+CARWID) < cp.pt.x)
            trouble = 1;
         ideal = np.ang() + MY_PI12;
         np.rotate( dp / curv.R);
      }
      else {                  // right turn
         if ((pos-CARWID) > cp.pt.x)
            trouble = 1;
         left  = 0;
         ideal = np.ang() - MY_PI12;
         np.rotate(-dp / curv.R);
      }
      np.add(curv.c);

      if (np.y > Fl) {  // target is in next component
         if (nex.type == 0) {
            pos   = nex.Sl + (np.y - Fl);
            np.x  = nex.to_side(pos);
            ideal = MY_PI12;
         }
      }
   }

   ideal -= cur_dir;
   if (ideal > MY_PI)
      ideal -= MY_PI21;
   else if (ideal < -MY_PI)
      ideal += MY_PI21;

   // limit the change in direction
   np.sub(cp.pt);
   pos = np.ang() - cur_dir;
   if (pos > MY_PI)
      pos -= MY_PI21;
   else if (pos < -MY_PI)
      pos += MY_PI21;
   pos *= err_pct;
   pos += (1.0 - err_pct) * ideal;

   if (left) {
      if (pos > MAX_dA)
         pos =  MAX_dA;
      else if (pos < -0.05*MAX_dA)
         pos = -0.05*MAX_dA;
   }
   else {
      if (pos > 0.05*MAX_dA)
         pos =  0.05*MAX_dA;
      else if (pos < -MAX_dA)
         pos = -MAX_dA;
   }

   np.x = dp;
   np.y = 0.0;
   np.rotate(cur_dir + pos);
   return ((trouble) ? err_pct : -err_pct);
}

// PRIVATE METHODS
void SBTurn::construct_S()
{
   // make sure S/F positions are in the segment
   if (Sl < 0.0) {
      Sl = 0.0;
      Sw = to_side(Sl);
   }
   else if (Sl > len) {
      Sl = len;
      Sw = to_side(Sl);
   }

   if (Fl > len) {
      Fl = len;
      Fw = to_side(Fl);
   }
   else if (Fl < 0.0) {
      Fl = 0.0;
      Fw = to_side(Fl);
   }
}

void SBTurn::construct_T()
{
   // make sure S/F positions are in the segment
   if ((Sl+0.087266) >= MY_PI21) {
      Sl = 0.0;
      Sw = to_side(Sl);
   }
}

double SBTurn::to_side_S(double pos)
{
   double    ret;
   SBVector *i;
   SBLine    ln(0.0, pos, 1.0, 0.0);

   i = curv.intersect_F(ln);
   i->sub(ln.pt);
   ret = i->mag();

   delete i;
   return ((ret < SB_IN_WID)?SB_IN_WID:((ret > (wid-SB_IN_WID))?(wid-SB_IN_WID):ret));
}

double SBTurn::to_side_T(double pos)
{
   double    ret;
   SBVector *i;
   SBLine    ln(rad*cos(pos), rad*sin(pos), pos);

   i = curv.intersect_F(ln);
   i->sub(ln.pt);
   ret = i->mag();

   delete i;
   return ((ret < SB_IN_WID)?SB_IN_WID:((ret > (wid-SB_OUT_WID))?(wid-SB_OUT_WID):ret));
}
//-----END Turn.cpp-----//

//-----START DrvPath.cpp-----//
// CONSTRUCTORS
SBDrvPath::SBDrvPath(int type, int n)
{
   int i;

   seg_type = type;
   pc       = new SBPathComponent*[n];
   if (pc != 0)
      num = n;
   else
      num = 0;

   for (i = 0; i < num; i++) {
      pc[i] = 0;
   }

   spd_cur = 0.0;
   spd     = (type == 0) ? 1.0 : SCALE_CORNSPD;
   apex    = 0.0;
}

// DESTRUCTOR
SBDrvPath::~SBDrvPath()
{
   int i;

   for (i = 0; i < num; i++) {
      delete pc[i];
   }
   delete [] pc;
}

// METHODS
void SBDrvPath::construct()
{
   if (seg_type == 0)
      construct_S();
   else
      construct_T();
}

SBPathComponent *SBDrvPath::get_PC(double pos)
{
   int i, s, f;

   s = f = -1;
   for (i = 0; i < num; i++) {
      if ( (s == -1)&&(pc[i]->Sl < pc[i]->Fl) )
         s = i;
      if (pc[i]->Sl < pc[i]->Fl)
         f = i;
   }

   if (s < 0)
      return NULL;
   else if (pos < pc[s]->Sl)
      return pc[s];

   for (i = s; i <= f; i++) {
      if ( (pos >= pc[i]->Sl)&&(pos < pc[i]->Fl) )
         return pc[i];
   }

   return pc[f];
}

// Finds best path through corner.  Assumes (cs) is on straight before
// corner, (ns) is in the corner, and (as) is on a straight after corner
void SBDrvPath::best_path(SBTurn &cs, SBTurn &ns, SBTurn &as)
{
   SBLine   *s, *f;
   SBCircle *a;

   // find the best circle for Turn (ns)
   s = new SBLine((ns.rad + ns.Sw), 0.0, 0.0, -1.0);
   a = new SBCircle(ns.rad + ns.Aw);
   f = new SBLine((ns.rad + ns.Fw) * cos(ns.len),
                  (ns.rad + ns.Fw) * sin(ns.len),
                  (ns.len + MY_PI12));
   ns.curv.tangent2(*s, *f, *a);

   // Update segements cs, as
   Update_cs(cs, ns);
   Update_as(as, ns);

   delete s; delete f;
   delete a;
}

// Adjusts path through corners (cs,ns). Assumes (ps) is on straight
// before 1st corner, (cs) is in the 1st corner, (ns) is in the 2nd
// corner, and (as) is on a straight after 2nd corner.
void SBDrvPath::adjust_path(SBTurn &ps, SBTurn &cs, SBTurn &ns, SBTurn &as)
{
   double   c_wgt, dist;
   SBLine   move;
   SBVector newC;

   if ( (cs.type == 0)||(ns.type == 0) )
      return;

   c_wgt = cs.rad / (cs.rad + ns.rad);
   dist  = cs.to_side(cs.len);
   dist -= ns.wid - ns.to_side(0.0);

   // move (cs) Turn
   move.pt.x  = cs.curv.c.x; move.pt.y  = cs.curv.c.y;
   move.dir.x = 1.0;         move.dir.y = 0.0;
   move.dir.rotate(cs.len + MY_PI);
   move.getPoint(newC, (c_wgt * dist));
   cs.curv.c.x = newC.x; cs.curv.c.y = newC.y;
   cs.Fl = cs.len;
   cs.Fw = cs.to_side(cs.Fl);

   // move (ns) Turn
   move.pt.x  = ns.curv.c.x; move.pt.y  = ns.curv.c.y;
   move.dir.x = -1.0;        move.dir.y = 0.0;
   move.getPoint(newC, ((1.0 - c_wgt) * dist));
   ns.curv.c.x = newC.x; ns.curv.c.y = newC.y;
   ns.Sl = 0.0;
   ns.Sw = ns.to_side(ns.Sl);

   // Update segements ps, as
   Update_cs(ps, cs);
   Update_as(as, ns);
}

// PRIVATE METHODS
void SBDrvPath::construct_S()
{
   double  n_pct, n_len, p_len;
   SBTurn *n, *p;

   pc[0]->construct();
   pc[2]->construct();

   pc[1]->Sl = pc[0]->Fl;
   pc[1]->Sw = pc[0]->Fw;
   pc[1]->Fl = pc[2]->Sl;
   pc[1]->Fw = pc[2]->Sw;
   pc[1]->construct();


   // Initialize speed parameters
   spd_cur = spd * crn_spd(0.0);

   if ((pc[1]->Fl - pc[1]->Sl) > CARLEN)
      return;

   // fix overlap
   p = (SBTurn *)pc[0];
   n = (SBTurn *)pc[2];

   n_pct = p->curv.R + n->curv.R;
   if (n_pct != 0.0)
      n_pct = p->curv.R / n_pct;
   else
      n_pct = 0.5;

   // cut segement into two sections
   n_len = n_pct * pc[1]->len;
   p_len = pc[1]->len - n_len;
   if (n_len > (pc[2]->Fl - pc[2]->Sl)) {  // n_len too big
      n_len = pc[2]->Fl - pc[2]->Sl;
      p_len = pc[1]->len - n_len;
      if (p_len > (pc[0]->Fl - pc[0]->Sl))
         p_len = pc[0]->Fl - pc[0]->Sl;
   }
   if (p_len > (pc[0]->Fl - pc[0]->Sl)) {  // p_len too big
      p_len = pc[0]->Fl - pc[0]->Sl;
      n_len = pc[1]->len - p_len;
      if (n_len > (pc[2]->Fl - pc[2]->Sl))
         n_len = pc[2]->Fl - pc[2]->Sl;
   }

   pc[1]->Sl  = pc[1]->Fl = p_len;
   pc[1]->Fl += pc[1]->len - n_len - p_len;
   pc[1]->Sw  = n_pct       * pc[2]->to_side(pc[1]->Fl);
   pc[1]->Sw += (1.0-n_pct) * pc[0]->to_side(pc[1]->Sl);
   n_len = 1.5 * CARWID;
   if (pc[1]->Sw < n_len)
      pc[1]->Sw = n_len;
   else if (pc[1]->Sw > (pc[1]->wid-n_len))
      pc[1]->Sw = pc[1]->wid - n_len;
   pc[1]->Fw = pc[1]->Sw;
   pc[1]->construct();

   pc[0]->Fl = pc[1]->Sl;
   pc[0]->Fw = pc[1]->Sw;
   pc[2]->Sl = pc[1]->Fl;
   pc[2]->Sw = pc[1]->Fw;
}

void SBDrvPath::construct_T()
{
   int     i;
   double  rad;
   SBTurn *t;

   for (i = 0; i < num; i++) {
      pc[i]->construct();
      if ((i+1) < num) {
         pc[i+1]->Sl = pc[i]->Fl;
         pc[i+1]->Sw = pc[i]->Fw;
      }
      else {
         pc[i]->Fl = pc[i]->len;
         pc[i]->Fw = pc[i]->to_side(pc[i]->len);
      }
   }

   // Initialize speed parameters
   if (num == 1)
      t = (SBTurn *)pc[0];
   else if (num == 3)
      t = (SBTurn *)pc[1];
   else {
      t   = (SBTurn *)pc[0];
      rad = t->curv.R;
      t   = (SBTurn *)pc[1];
      if (rad < t->curv.R)
         t = (SBTurn *)pc[0];
   }
   spd_cur = spd * crn_spd(t->curv.R);
   apex    = t->curv.c.ang() - MY_PI;
}

void SBDrvPath::Update_cs(SBTurn &cs, SBTurn &ns)
{
   double   dist;
   SBVector i;

   // place circle on segments cs
   cs.curv.R = ns.curv.R;

   if (ns.type == 1)  // left turn
      cs.curv.c.x = ns.curv.c.x - ns.rad;
   else               // right turn
      cs.curv.c.x = cs.wid + ns.rad - ns.curv.c.x;
   cs.curv.c.y = ns.curv.c.y + cs.len;


   // Turn in (cs.Sl, cs.Sw)
   ns.curv.getPoint(i, 0.0);
   cs.Sl = cs.len + i.y;
   if (ns.type == 1)  // left turn
      cs.Sw = i.x - ns.rad;
   else               // right turn
      cs.Sw = cs.wid - i.x + ns.rad;

   // Exit straight / Enter turn (cs.Fl, cs.Fw, ns.Sl, ns.Sw)
   if (ns.type == 1)  // left turn
      dist = cs.to_side(cs.len);
   else               // right turn
      dist = cs.wid - cs.to_side(cs.len);
   i.x = ns.rad + dist; i.y = 0.0;

   cs.Fl = cs.len + i.y;
   if (ns.type == 1)  // left turn
      cs.Fw = i.x - ns.rad;
   else               // right turn
      cs.Fw = cs.wid - i.x + ns.rad;

   ns.Sl = i.ang();
   ns.Sw = i.x - ns.rad;
}

void SBDrvPath::Update_as(SBTurn &as, SBTurn &ns)
{
   double   dist;
   SBVector i;

   // place circle on segment as
   as.curv.R = ns.curv.R;

   as.curv.c.x = ns.curv.c.x;
   as.curv.c.y = ns.curv.c.y;
   as.curv.c.rotate(-ns.len);
   if (ns.type == 1)
      as.curv.c.x = as.curv.c.x - ns.rad;
   else
      as.curv.c.x = as.wid + ns.rad - as.curv.c.x;


   // Exit turn / Enter straight (ns.Fl, ns.Fw, as.Sl, as.Sw)
   if (ns.type == 1)  // left turn
      dist = as.to_side(0.0);
   else               // right turn
      dist = as.wid - as.to_side(0.0);
   i.x = cos(ns.len); i.y = sin(ns.len);
   i.scale(ns.rad + dist);

   ns.Fl = i.ang();
   i.rotate(-ns.len);
   ns.Fw = i.x - ns.rad;

   as.Sl = i.y;
   if (ns.type == 1)  // left turn
      as.Sw = i.x - ns.rad;
   else               // right turn
      as.Sw = as.wid - i.x + ns.rad;

   // Turn out (as.Fl, as.Fw)
   ns.curv.getPoint(i, ns.len);
   i.rotate(-ns.len);
   as.Fl = i.y;
   if (ns.type == 1)  // left turn
      as.Fw = i.x - ns.rad;
   else               // right turn
      as.Fw = as.wid - i.x + ns.rad;
}
//-----END DrvPath.cpp-----//

//-----START TrkData.cpp-----//
// CONSTRUCTORS
SBTrkData::SBTrkData()
{
   name = 0;
   path = 0;
   nseg = 0;
}

// DESTRUCTOR
SBTrkData::~SBTrkData()
{
   int i;

   if (path) {
      for (i = 0; i < nseg; i++) {
         delete path[i];
      }
      delete [] path;
   }

   if (name)
      free(name);
}

// METHODS
void SBTrkData::set_path(char *n, track_desc *trk)
{
   int         i, j;
   int         ps, cs, ns, as;
   double      dw;
   SBTurn      seg_cs, seg_as;
   SBTurn     *tp, *ts, *tc, *tf;
   SBStraight *ss;

   this->~SBTrkData();  // clear out old data
   path = new SBDrvPath*[trk->NSEG];
   if (path != 0)
      nseg = trk->NSEG;
   else {
      nseg = 0;
      return;
   }

   // store track name
   j    = strlen(n);
   name = (char *)malloc(j + 1);
   for (i = 0; i < j; i++) {
      name[i] = (char)tolower((int)n[i]);
   }
   name[j] = 0;

   // try to load data if available
   if (loadData(DATA_FILE) == 0)
      return;

   // Initialize
   seg_cs.set_seg_data(5000.0, trk->width, 0.0);
   seg_as.set_seg_data(5000.0, trk->width, 0.0);

   for (i = 0; i < nseg; i++) {
      if (trk->lftwall[i].radius > 0.0) {       // left turn
         path[i] = new SBDrvPath( 1, 1);
         ts      = new SBTurn();
         ts->set_seg_data(trk->lftwall[i].length, trk->width,
                          trk->lftwall[i].radius);
      }
      else if (trk->lftwall[i].radius < 0.0) {  // right turn
         path[i] = new SBDrvPath(-1, 1);
         ts      = new SBTurn();
         ts->set_seg_data(trk->rgtwall[i].length, trk->width,
                          trk->rgtwall[i].radius);
      }
      else {                                    // straight
         path[i] = new SBDrvPath( 0, 3);
         ts      = new SBTurn();
         ss      = new SBStraight();
         tf      = new SBTurn();
         ts->set_seg_data(trk->lftwall[i].length, trk->width, 0.0);
         ss->set_seg_data(trk->lftwall[i].length, trk->width, 0.0);
         tf->set_seg_data(trk->lftwall[i].length, trk->width, 0.0);
      }

      if (path[i]->seg_type != 0) {
         ts->Sw = trk->width - SB_OUT_WID;
         ts->Sl = 0.0;
         ts->Aw = SB_IN_WID;
         ts->Fw = trk->width - SB_OUT_WID;
         ts->Fl = ts->len;
         path[i]->pc[0] = ts;
      }
      else {
         ts->Sw = 0.0;
         ts->Sl = 0.0;
         ts->Aw = 0.0;
         ts->Fw = 0.0;
         ts->Fl = 0.0;
         ss->Sw = 0.0;
         ss->Sl = 0.0;
         ss->Fw = 0.0;
         ss->Fl = ss->len;
         tf->Sw = 0.0;
         tf->Sl = 0.0;
         tf->Aw = 0.0;
         tf->Fw = 0.0;
         tf->Fl = 0.0;
         path[i]->pc[0] = ts;
         path[i]->pc[1] = ss;
         path[i]->pc[2] = tf;
      }
   }

   for (j = 0; j < 2; j++) {
      // Initial run through track, to find preliminary values
      for (i = 0; i < nseg; i++) {
         cs = i;
         ns = (cs + 1) % trk->NSEG;
         as = (cs + 2) % trk->NSEG;
         ps = (cs + trk->NSEG - 1) % trk->NSEG;

         if (path[cs]->seg_type == 0) {  // straight
            if ( (path[ns]->seg_type != 0)&&(path[as]->seg_type == 0) ) {
               // straight, turn, straight
               ts = (SBTurn *)path[cs]->pc[2];
               tc = (SBTurn *)path[ns]->pc[0];
               tf = (SBTurn *)path[as]->pc[0];
               SBDrvPath::best_path(*ts, *tc, *tf);
            }
            else if ( (path[ns]->seg_type != 0)&&(path[as]->seg_type != 0) ) {
               // straight, turn, turn
               ts = (SBTurn *)path[cs]->pc[2];
               tc = (SBTurn *)path[ns]->pc[0];
               tf = (SBTurn *)path[as]->pc[0];

               if ((tc->type - tf->type) == 0) {
                  // turns in same direction
                  tc->Aw = tc->Fw = 2.0 * CARWID;
                  tf->Aw = tf->Sw = 2.0 * CARWID;
               }
               else {
                  // turns in different directions
                  dw = tc->wid - (2.5 * CARWID);
                  if (tc->Sw > dw)
                     tc->Sw = dw;
                  tc->Fw = tc->Sw;

                  if (tf->Fw > dw)
                     tf->Fw = dw;
                  tf->Sw = tf->Fw;
               }

               SBDrvPath::best_path(*ts, *tc, seg_as);
            }
         }
         else {                         // turn
            if ( (path[ns]->seg_type != 0)&&(path[as]->seg_type == 0) ) {
               // turn, turn, straight
               ts = (SBTurn *)path[cs]->pc[0];
               tc = (SBTurn *)path[ns]->pc[0];
               tf = (SBTurn *)path[as]->pc[0];
               SBDrvPath::best_path(seg_cs, *tc, *tf);

               if ((ts->type - tc->type) != 0) {
                  // turns in different directions
                  if (path[ps]->seg_type == 0)
                     tp = (SBTurn *)path[ps]->pc[2];
                  else
                     tp = &seg_cs;
                  SBDrvPath::adjust_path(*tp, *ts, *tc, *tf);
               }
            }
            else if ( (path[ns]->seg_type != 0)&&(path[as]->seg_type != 0) ) {
               // turn, turn, turn
               ts = (SBTurn *)path[cs]->pc[0];
               tc = (SBTurn *)path[ns]->pc[0];
               tf = (SBTurn *)path[as]->pc[0];

               if ((tc->type - tf->type) == 0) {
                  // turns in same direction
                  tc->Aw = tc->Fw = 2.0 * CARWID;
                  tf->Aw = tf->Sw = 2.0 * CARWID;
               }
               else {
                  // turns in different directions
                  dw = tc->wid - (2.5 * CARWID);
                  if (tc->Sw > dw)
                     tc->Sw = dw;
                  tc->Fw = tc->Sw;

                  if (tf->Fw > dw)
                     tf->Fw = dw;
                  tf->Sw = tf->Fw;
               }

               SBDrvPath::best_path(seg_cs, *tc, seg_as);

               if ((ts->type - tc->type) != 0) {
                  // turns in different directions
                  if (path[ps]->seg_type == 0)
                     tp = (SBTurn *)path[ps]->pc[2];
                  else
                     tp = &seg_cs;
                  SBDrvPath::adjust_path(*tp, *ts, *tc, seg_as);
               }
            }
         }
      }

      // Second run to fix straights
      for (i = 0; i < nseg; i++) {
         cs = i;
         ns = (cs + 1) % trk->NSEG;
         as = (cs + 2) % trk->NSEG;
         if ( (path[cs]->seg_type == 0)&&(path[ns]->seg_type == 0) ) {
            // straight, straight
            path[cs]->pc[2]->Sl = path[cs]->pc[0]->len;
            path[cs]->pc[2]->Sw = path[cs]->pc[0]->Fw;
            path[cs]->pc[2]->Fl = path[cs]->pc[0]->len;
            path[cs]->pc[2]->Fw = path[cs]->pc[0]->Fw;

            path[ns]->pc[0]->Sl = 0.0;
            path[ns]->pc[0]->Sw = path[cs]->pc[2]->Fw;
            path[ns]->pc[0]->Fl = 0.0;
            path[ns]->pc[0]->Fw = path[cs]->pc[2]->Fw;
         }
      }

      // Construct path
      for (i = 0; i < nseg; i++) {
         cs = i;
         ns = (cs + 1) % trk->NSEG;
         as = (cs + 2) % trk->NSEG;

         path[ns]->construct();

         if (j == 0) {
            if (path[cs]->seg_type != 0) {
               path[cs]->pc[0]->Fw = trk->width - SB_OUT_WID;
               path[cs]->pc[0]->Fl = path[cs]->pc[0]->len;
            }
            if (path[as]->seg_type != 0) {
               path[as]->pc[0]->Sw = trk->width - SB_OUT_WID;
               path[as]->pc[0]->Sl = 0.0;
            }

            if (path[ns]->seg_type == 0) {
               dw = path[cs]->pc[0]->Fw;
               if (path[cs]->seg_type == 1)
                  dw -= path[ns]->pc[0]->Fw;
               else if (path[cs]->seg_type == -1)
                  dw -= trk->width - path[ns]->pc[0]->Fw;
               else
                  dw  = 0.0;

               if (fabs(dw) > (0.2*CARWID))
                  path[cs]->pc[0]->Fw -= dw;


               dw = path[as]->pc[0]->Sw;
               if (path[as]->seg_type == 1)
                  dw -= path[ns]->pc[2]->Sw;
               else if (path[as]->seg_type == -1)
                  dw -= trk->width - path[ns]->pc[2]->Sw;
               else
                  dw  = 0.0;

               if (fabs(dw) > (0.2*CARWID))
                  path[as]->pc[0]->Sw -= dw;
            }
         }
      }
   }
}

void SBTrkData::saveData(const char *fname, const int ver)
{
   if (ver % 2)
      saveTXTData(fname);
   else
      saveBINData(fname);
}

int SBTrkData::loadData(const char *fname)
{
   FILE *fp;
   char  buf[4];

   fp = fopen(fname, "r");
   if (fp == NULL)
      return -1;

   if (!fgets(buf, 4, fp)) {
      fclose(fp);
      return -1;
   }
   fclose(fp);
   buf[2] = 0;

   if (!strcmp(buf, "v1"))
      return loadTXTData(fname);
   else if (!strcmp(buf, "v2"))
      return loadBINData(fname);
   else
      return -1;
}


// PRIVATE METHODS
void SBTrkData::saveTXTData(const char *fname)
{
   FILE       *fp;
   int         i, j, datalen;
   SBTurn     *ts;
   SBStraight *ss;

   fp = fopen(fname, "w");
   if (fp == NULL)
      return;

   datalen = 0;
   for (i = 0; i < nseg; i++) {
      if (path[i]->seg_type == 0)
         datalen += 13;
      else
         datalen +=  2 + (4*path[i]->num);
   }

   fprintf(fp, "v1\n");
   fprintf(fp, "TRACK: %s  LEN: %d [CARWID=%f][CARLEN=%f]\n",name,datalen,CARWID,CARLEN);
   for (i = 0; i < nseg; i++) {
      fprintf(fp, "<Seg: %3d, ", i);
      fprintf(fp, "type: %2d, ", path[i]->seg_type);
      fprintf(fp, "num: %1d>  ", path[i]->num);

      fprintf(fp, "<spd=%5.3f, spd_cur=%7.3f>\n", path[i]->spd, path[i]->spd_cur);

      for (j = 0; j < path[i]->num; j++) {
         fprintf(fp, "%1d", j);
         if ( ((j%2) == 0)||(path[i]->seg_type != 0) ) {
            ts = (SBTurn *)path[i]->pc[j];
            saveTurn(fp, *ts);
         }
         else {
            ss = (SBStraight *)path[i]->pc[j];
            saveStraight(fp, *ss);
         }
      }
      if ((i + 1) < nseg)
         fprintf(fp, "\n");
   }
	fclose(fp);
}

void SBTrkData::saveTurn(FILE *fp, SBTurn &t)
{
   int p = (t.type == 0) ? 3 : 6;

   fprintf(fp, "     TURN:");
   fprintf(fp, " type = %2d", t.type);
   fprintf(fp, " len = %8.*f", p, t.len);
   fprintf(fp, " wid = %5.1f", t.wid);
   fprintf(fp, " rad = %8.3f", t.rad);
   fprintf(fp, "\n");

   fprintf(fp, "  ");
   fprintf(fp, " Sl = %8.*f", p, t.Sl);
   fprintf(fp, " Sw = %5.1f", t.Sw);
   fprintf(fp, " Fl = %8.*f", p, t.Fl);
   fprintf(fp, " Fw = %5.1f", t.Fw);
   fprintf(fp, "\n");

   fprintf(fp, "  ");
   fprintf(fp, " Aw = %5.2f", t.Aw);
   fprintf(fp, "\n");

   fprintf(fp, "  ");
   fprintf(fp, " Circle{");
   fprintf(fp, "c.x=%9.3f",   t.curv.c.x);
   fprintf(fp, ", c.y=%9.3f", t.curv.c.y);
   fprintf(fp, ", R=%8.3f",   t.curv.R);
   fprintf(fp, "}\n");
}

void SBTrkData::saveStraight(FILE *fp, SBStraight &t)
{
   fprintf(fp, " STRAIGHT:");
   fprintf(fp, " type = %2d", t.type);
   fprintf(fp, " len = %8.3f", t.len);
   fprintf(fp, " wid = %5.1f", t.wid);
   fprintf(fp, " rad = %3.1f", t.rad);
   fprintf(fp, "\n");

   fprintf(fp, "  ");
   fprintf(fp, " Sl = %8.3f", t.Sl);
   fprintf(fp, " Sw = %5.1f", t.Sw);
   fprintf(fp, " Fl = %8.3f", t.Fl);
   fprintf(fp, " Fw = %5.1f", t.Fw);
   fprintf(fp, "\n");

   fprintf(fp, "  ");
   fprintf(fp, " Line{");
   fprintf(fp, "pt.x=%9.3f",    t.l.pt.x);
   fprintf(fp, ", pt.y=%9.3f",  t.l.pt.y);
   fprintf(fp, ", dir.x=%6.3f", t.l.dir.x);
   fprintf(fp, ", dir.y=%6.3f", t.l.dir.y);
   fprintf(fp, "}\n");
}

int SBTrkData::loadTXTData(const char *fname)
{
   char        buf[128], *tok;
   int         i, j, type, num_comp, datalen, straight;
   double      len, wid, rad, Sw, Sl, Fw, Fl;
   SBTurn     *ts;
   SBStraight *ss;
   char       *sep = " \n\t,=<>{}:";
   FILE       *fp = fopen(fname, "r");

   // skip first line
   if (!fgets(buf, 128, fp)) {
      fclose(fp);
      return -1;
   }

   // look for track
   while (1) {
      if (!fgets(buf, 128, fp)) {
         fclose(fp);
         return -1;
      }
      tok = strtok(buf, sep);    // TRACK:
      tok = strtok(NULL, sep);   // Track name
      if (!tok) {
         fclose(fp);
         return -1;
      }
      if (!strcmp(name, tok))
         break;   // track found

      // skip this track
      tok     = strtok(NULL, sep);        // LEN:
      datalen = atoi(strtok(NULL, sep));  // data length
      for (i = 0; i < datalen; i++) {
         if (!fgets(buf, 128, fp)) {
            fclose(fp);
            return -1;
         }
      }
   }

   for (i = 0; i < nseg; i++) {
      if (!fgets(buf, 128, fp)) {
         fclose(fp);
         return -1;
      }
      tok          = strtok(buf, sep);          // Seg:
      tok          = strtok(NULL, sep);         // Seg value
      tok          = strtok(NULL, sep);         // type:
      type         = atoi(strtok(NULL, sep));   // type value
      tok          = strtok(NULL, sep);         // num:
      num_comp     = atoi(strtok(NULL, sep));   // number of components
      path[i]      = new SBDrvPath(type, num_comp);
      tok          = strtok(NULL, sep);         // spd=
      path[i]->spd = atof(strtok(NULL, sep));   // spd value

      for (j = 0; j < num_comp; j++) {
         if (!fgets(buf, 128, fp)) {
            fclose(fp);
            return -1;
         }
         tok = strtok(buf, sep);          // Comp_num
         tok = strtok(NULL, sep);         // Comp_name
         if (!strcmp(tok, "STRAIGHT"))
            straight = 1;
         else if (!strcmp(tok, "TURN"))
            straight = 0;
         else {
            fclose(fp);
            return -1;
         }
         tok = strtok(NULL, sep);         // Type
         tok = strtok(NULL, sep);         // Type_val
         tok = strtok(NULL, sep);         // Len
         len = atof(strtok(NULL, sep));   // Len_val
         tok = strtok(NULL, sep);         // Wid
         wid = atof(strtok(NULL, sep));   // Wid_val
         tok = strtok(NULL, sep);         // Rad
         rad = atof(strtok(NULL, sep));   // Rad_val

         if (!fgets(buf, 128, fp)) {
            fclose(fp);
            return -1;
         }
         tok = strtok(buf, sep);          // Sl
         Sl  = atof(strtok(NULL, sep));   // Sl_val
         tok = strtok(NULL, sep);         // Sw
         Sw  = atof(strtok(NULL, sep));   // Sw_val
         tok = strtok(NULL, sep);         // Fl
         Fl  = atof(strtok(NULL, sep));   // Fl_val
         tok = strtok(NULL, sep);         // Fw
         Fw  = atof(strtok(NULL, sep));   // Fw_val

         if (straight) {
            ss = new SBStraight();
            ss->set_seg_data(len, wid, rad);
            ss->Sl = Sl; ss->Sw = Sw;
            ss->Fl = Fl; ss->Fw = Fw;

            if (!fgets(buf, 128, fp)) {
               fclose(fp);
               return -1;
            }
            tok         = strtok(buf, sep);        // Line
            tok         = strtok(NULL, sep);       // pt.x
            ss->l.pt.x  = atof(strtok(NULL, sep)); // pt.x_val
            tok         = strtok(NULL, sep);       // pt.y
            ss->l.pt.y  = atof(strtok(NULL, sep)); // pt.y_val
            tok         = strtok(NULL, sep);       // dir.x
            ss->l.dir.x = atof(strtok(NULL, sep)); // dir.x_val
            tok         = strtok(NULL, sep);       // dir.y
            ss->l.dir.y = atof(strtok(NULL, sep)); // dir.y_val
            path[i]->pc[j] = ss;
         }
         else {
            ts = new SBTurn();
            ts->set_seg_data(len, wid, rad);
            ts->Sl = Sl; ts->Sw = Sw;
            ts->Fl = Fl; ts->Fw = Fw;
				ts->type = type;

            if (!fgets(buf, 128, fp)) {
               fclose(fp);
               return -1;
            }
            tok    = strtok(buf, sep);          // Aw
            ts->Aw = atof(strtok(NULL, sep));   // Aw_val

            if (!fgets(buf, 128, fp)) {
               fclose(fp);
               return -1;
            }
            tok          = strtok(buf, sep);          // Circle
            tok          = strtok(NULL, sep);         // c.x
            ts->curv.c.x = atof(strtok(NULL, sep));   // c.x_val
            tok          = strtok(NULL, sep);         // c.y
            ts->curv.c.y = atof(strtok(NULL, sep));   // c.y_val
            tok          = strtok(NULL, sep);         // R
            ts->curv.R   = atof(strtok(NULL, sep));   // R_val
            path[i]->pc[j] = ts;
         }
      }
      if ( ((i + 1) < nseg)&&(!fgets(buf, 128, fp)) ) {
         fclose(fp);
         return -1;
      }
      path[i]->construct();
   }
   fclose(fp);
   return 0;
}

void SBTrkData::saveBINData(const char *fname)
{
}

int SBTrkData::loadBINData(const char *fname)
{
   return -1;
}
//-----END TrkData.cpp-----//

//-----START SBDriver.cpp-----//
// CONSTRUCTORS
SmoothB::SmoothB()
{
   m_sName         = SB_NAME1;
   m_sAuthor       = SB_AUTHOR;
   m_sDescription  = SB_DESCRIPTION;
   m_iNoseColor    = SB_NOSE;
   m_iTailColor    = SB_TAIL;
   m_sBitmapName2D = SB_BITMAP2D;
   // m_sBitmapName3D = SB_BITMAP3D;

   name      = NULL;
   init      = 2;
   cp        = &result;
   lastpit   = 0;
   fuelstart = MAX_FUEL;
   err       = 0.0;
   passTime  = 0;
#ifdef SB_TESTING
   SBnum     = 0;
#endif
}

SmoothB::SmoothB(int i)
{
   name = new char[SB_NAME_LEN];
   sprintf(name, SB_NAME2, (i%100));

   m_sName         = name;
   m_sAuthor       = SB_AUTHOR;
   m_sDescription  = SB_DESCRIPTION;
   m_iNoseColor    = SB_NOSE;
   m_iTailColor    = SB_TAIL;
   m_sBitmapName2D = SB_BITMAP2D;
   // m_sBitmapName3D = SB_BITMAP3D;

   init      = 2;
   cp        = &result;
   lastpit   = 0;
   fuelstart = MAX_FUEL;
   err       = 0.0;
   passTime  = 0;
#ifdef SB_TESTING
   SBnum     = i;
#endif
}


// DESTRUCTOR
SmoothB::~SmoothB()
{
   if (name)
      delete [] name;
}


// METHODS
con_vec SmoothB::drive(situation &s)
{
   if (args.m_iCurrentTrack != trk_num) {
      trk_info = get_track_description();
      if (trk_info.NSEG > 1) {
         // set surface type
         SURFACE_TYPE = args.m_iSurface;
         MAX_ACCEL    = 32.2;
         switch (SURFACE_TYPE) {
         case 0:
            MAX_ACCEL *= MYU_MAX0;
            break;
         case 1:
            MAX_ACCEL *= MYU_MAX1;
            break;
         case 2:
            MAX_ACCEL *= MYU_MAX2;
         }

         trk_data.set_path(currentTrack->m_sFileName, &trk_info);
         trk_num    = args.m_iCurrentTrack;
      }
   }
   if ( (s.starting)&&(trk_info.NSEG > 1) ) {
      init               = (init < 1) ? 1 : init;
      result.fuel_amount = fuel_needed(s.stage, (s.laps_to_go+5), (SB_FC*trk_info.length));
   }

   if (init == 2) {
      init--;
      result.vc    = 400.0;
      result.alpha =   0.0;
      return result;
   }
   else if (init == 1) {
      if ( ((!s.seg_ID)&&((s.cur_len - s.to_end) >= trk_data.path[0]->pc[1]->Fl))||
           (s.v > 60.0) )
         init--;
   }

   get_RARS_ctrl_params(s);

   if (init == 1) {
      result.alpha *= 0.25;
      err           = 0.0;
   }
   return result;
}


// PRIVATE METHODS
void SmoothB::get_RARS_ctrl_params(situation &s)
{
   SBLine           cV;
   SBVector         nV;
   SBPathComponent *pc, *nex_pc;
   double           acc, tgt_v, tv, lane, bias;
   double           pos = s.cur_len - s.to_end;
   int              sID2;
   int              brk_hard = 0, passing = 0;


   // Handle fuel / damage
   cp->repair_amount  = (s.damage > 14000) ? 14000 : s.damage;
   cp->repair_amount /= 2;
   if (s.out_pits) {
      cp->request_pit = 0;
      lastpit         = s.laps_done;
      passTime        = 5 * SB_PASS_COUNT;
   }
   if ( (s.lap_flag)&&(s.laps_to_go > 1) ) {
      if (s.laps_done > lastpit) {
         cp->fuel_amount = (fuelstart - s.fuel)/(double)(s.laps_done - lastpit);
         if ( (s.fuel < (cp->fuel_amount*2.25))||
              ((s.laps_to_go > 4)&&(s.damage > 27500)) ) {
            cp->fuel_amount = fuel_needed(s.stage, (s.laps_to_go+2), cp->fuel_amount);
            cp->request_pit = 1;
         }
      }
      else
         fuelstart = s.fuel;
   }


   // Get new control parameters
   // get current and next path components
   sID2 = s.seg_ID;
   pc   = trk_data.path[sID2]->get_PC(pos);
   if (pc->type) {
      acc = 0.5 * s.v * delta_time / (pc->rad + pc->wid);
      acc = (acc < 0.0349) ? 0.0349 : acc;
   }
   else {
      acc = 0.5 * s.v * delta_time;
      acc = (acc < 2.0) ? 2.0 : acc;
   }
   acc += pc->Fl;
   if (acc < s.cur_len)
      nex_pc = trk_data.path[sID2]->get_PC(acc);
   else {
      acc -= s.cur_len;
      if (pc->type)
         acc *= pc->rad;
      if (s.nex_rad != 0.0)
         acc /= fabs(s.nex_rad);
      nex_pc = trk_data.path[(sID2+1)%trk_data.nseg]->get_PC(acc);
   }

   // find current position and velocity in current segment
   if (pc->type >= 0)
      get_pos(*pc, pos, s.to_lft, cV.pt);
   else
      get_pos(*pc, pos, s.to_rgt, cV.pt);
   get_vel(*pc, s.v, s.vn, pos, cV.dir);

   // target velocity
   tgt_v = tv = get_tgt_v(s);

   // find bias and lane for passing_action()
   if (!pc->type) {
      if (pc == trk_data.path[sID2]->pc[0])
         sID2 = (sID2+trk_data.nseg-1) % trk_data.nseg;
      else if (pc == trk_data.path[sID2]->pc[2])
         sID2 = (sID2+1) % trk_data.nseg;
   }
   acc  = (double)trk_data.path[sID2]->pc[0]->type;
   acc *= trk_data.path[sID2]->pc[0]->len;
   if (acc == 0.0)
      bias = 0.0;
   else {
      if (fabs(tgt_v) > 10.273)
         bias = atan(16.0 / fabs(tgt_v));
      else
         bias = 1.0;

      acc /= MY_PI;
      if (acc < 1.0)
         bias *= (acc < -1.0) ? -1.0 : acc;

      bias *= (s.v*s.v) / (tgt_v*tgt_v);

      if ( (s.to_lft < SB_OUT_WID)&&(bias > 0.0) )
         bias *= s.to_lft / SB_OUT_WID;
      else if ( (s.to_rgt < SB_OUT_WID)&&(bias < 0.0) )
         bias *= s.to_rgt / SB_OUT_WID;

      if (bias > 1.0)
         bias =  1.0;
      else if (bias < -1.0)
         bias = -1.0;
   }
   lane = pc->to_side(pos);
   lane = (pc->type < 0) ? (pc->wid - lane) : lane;

   // safety measures
   if ( (s.to_lft < 0.0)||(s.to_rgt < 0.0) ) {
      // car is off track
      passing_action(s, lane, bias, 1);
      cp->vc = 400.0;
      return;
   }
   else if (passTime > SB_PASS_COUNT) {
      // car is leaving pits
      passing_action(s, lane, bias, 0);
      cp->vc = fabs(tgt_v * cos(cp->alpha));
      passTime--;
      passTime = (passTime == SB_PASS_COUNT) ? 0 : passTime;
      return;
   }

   // adjust target velocity
   tgt_v = fabs(tgt_v);
   acc   = (tgt_v - s.v) / delta_time;
   if (acc > MAX_ACCEL)
      tgt_v = s.v + (1.1 * MAX_ACCEL * delta_time);
   else if (acc < -MAX_ACCEL) {
      tgt_v = s.v - (1.1 * MAX_ACCEL * delta_time);
      brk_hard = 1;
   }
   if (tgt_v < 5.0)
      tgt_v = 5.0;
   nV.x = tgt_v; nV.y = 0.0;

   // find target position
   err = pc->get_nex_pos(cV, nV, *nex_pc);
   if (pc->type == -1) {
      cV.pt.x  = -cV.pt.x;
      cV.dir.x = -cV.dir.x;
      nV.x     = -nV.x;
   }
   nV.scale(1.0 / delta_time);

   // get new control parameters
#ifdef SB_TESTING
   passing = anti_collision(cV,nV,s,lane,((SBnum % 2)||(s.damage >= 15000)));
#else
   passing = anti_collision(cV,nV,s,lane,(SB_NO_PASSING||(s.damage >= 15000)));
#endif
   if (passing)
      passTime = SB_PASS_COUNT;
   else if (passTime)
      passTime--;

   if (passing % 2) {
      passing_action(s, nV.y, bias, 0);
      err *= (err <= 0.0) ? -0.6 : 1.175;
      err  = (err > 1.0) ? 1.0 : err;
   }
   else {
      update_con_vec(cV.dir, nV);
      nV.x = cp->alpha;
      nV.y = cp->vc;
      if (err <= 0.0) {
         passing_action(s, lane, bias, 0);
         cp->alpha *= -0.25*err;
         cp->alpha +=  (1.0 + (0.25*err)) * nV.x;
         cp->vc     = nV.y;
         if ( (s.to_lft >= SB_IN_WID)&&(s.to_rgt >= SB_IN_WID)&&
              (tv >= 200.0) ) {
            if ( (!brk_hard)&&(!passing) ) {
               if (pc->type == 0)
                  cp->vc *= 2.5;
               else if ( (s.cur_len < MY_PI14)&&(s.nex_rad == 0.0) )
                  cp->vc *= 2.5;
            }

            if ( (brk_hard)||(passTime) ) {
               cp->alpha *= 0.8;
               cp->alpha += 0.2 * s.alpha;
            }
            else if ( ((s.to_lft > 25.0)||(s.vn <  5.0))&&
                      ((s.to_rgt > 25.0)||(s.vn > -5.0)) ) {
               cp->alpha *= 0.35;
               cp->alpha += 0.65 * s.alpha;
            }
         }
         else {
            if ( ((s.to_lft > 10.0)||(s.vn <  5.0))&&
                 ((s.to_rgt > 10.0)||(s.vn > -5.0)) ) {
               cp->alpha *= 0.9;
               cp->alpha += 0.1 * s.alpha;
            }
         }
         err *= -0.6;
      }
      else {
         passing_action(s, lane, bias, 1);
         cp->alpha *= 0.33*err;
         cp->alpha += (1.0 - (0.33*err)) * nV.x;
         cp->vc    *= 0.07*err;
         cp->vc    += (1.0 - (0.07*err)) * nV.y;

         err *= 1.175;
         err  = (err > 1.0) ? 1.0 : err;
      }
   }
}

double SmoothB::fuel_needed(Stage s, int lapsleft, double fpl)
{
   double retval;

   if (fpl <= 0.0)
      return MAX_FUEL;
   else if (s == PRACTICE)
      retval = fpl * 10.0;
   else if (s == QUALIFYING)
      retval = fpl * 5.0;
   else
      retval = fpl * (double)lapsleft;

   if (retval > MAX_FUEL)
      return MAX_FUEL;
   return retval;
}

// Returns current mass of the car
double SmoothB::car_mass(double fuel)
{
   return M + (fuel / 32.2);
}

// friction function
double SmoothB::u(double slip)
{
   double retval;

   switch (SURFACE_TYPE) {
   case 2:
      retval = MYU_MAX2 * (1.0 - exp(-slip/SLIPPING));
      if (slip > 20.0)
         retval -= 0.2;
      break;
   case 1:
      retval = MYU_MAX1 * (1.0 - exp(-slip/SLIPPING));
      break;
   default:
      retval = MYU_MAX0 * (slip / (SLIPPING + slip));
   }
   return retval;
}

// inverse friction function
double SmoothB::iu(double cur_u)
{
   double retval;

   switch (SURFACE_TYPE) {
   case 2:
      if (cur_u >= MYU_MAX2)
         retval = 20.0;
      else
         retval = -SLIPPING * log(1.0 - (cur_u/MYU_MAX2));
      break;
   case 1:
      if (cur_u >= MYU_MAX1)
         retval = 20.0;
      else
         retval = -SLIPPING * log(1.0 - (cur_u/MYU_MAX1));
      break;
   default:
      if (cur_u >= MYU_MAX0)
         retval = 20.0;
      else
         retval = ((SLIPPING*cur_u)/MYU_MAX0) / (1.0 - (cur_u/MYU_MAX0));
   }
   return retval;
}

void SmoothB::update_con_vec(SBVector &cV, SBVector &nV)
{
   SBVector dV, L, W;
   double   acc;

   acc = -cV.ang();
   nV.rotate(acc);
   cV.rotate(acc);
   cV.y = 0.0;

   dV.add(nV);
   dV.sub(cV);

   L.add(dV);
   L.normalize();
   acc = (dV.mag() / delta_time) / 32.2;
   L.scale(-iu(acc));

   W.add(L);
   W.sub(cV);

   cp->vc    = W.mag();
   cp->alpha = W.ang() - MY_PI;
}

void SmoothB::get_pos(SBPathComponent &pc, double Pl, double Pw,
                      SBVector &pos)
{
   if (pc.type == 0) {
      pos.x = Pw;
      pos.y = Pl;
   }
   else {
      pos.x  = (pc.rad + Pw)*cos(Pl);
      pos.y  = (pc.rad + Pw)*sin(Pl);
   }
}

void SmoothB::get_vel(SBPathComponent &pc, double v, double vn, double Pl,
                      SBVector &vel)
{
   if (v < (fabs(vn)+1.0))
      v = fabs(vn) + 1.0;

   if (pc.type >= 0)
      vel.x = -vn;
   else
      vel.x =  vn;
   vel.y = sqrt((v*v) - (vn*vn));
   if (pc.type != 0)
      vel.rotate(Pl);
}

double SmoothB::get_tgt_v(situation &s)
{
   double cur_spd, nex_spd, tst_spd, dist2seg, brk_dist, end_seg, wid;
   int    i, sID[5];

   // segement IDs
   sID[0] = s.seg_ID;
   for (i = 1; i < 5; i++) {
      sID[i] = (sID[0] + i) % trk_data.nseg;
   }

   wid      = s.to_lft + s.to_rgt;
   dist2seg = ((s.cur_rad == 0.0) ? -1.5 : -2.0) * s.v * delta_time;
   cur_spd  = trk_data.path[sID[0]]->spd_cur;
   end_seg  = -(s.cur_len - s.to_end);
   if (trk_data.path[sID[0]]->seg_type != 0) {
      cur_spd *= 1.0 - err*(1.0 - 0.6/trk_data.path[sID[0]]->spd);
      if ( (err < 0.225)&&(-end_seg > trk_data.path[sID[0]]->apex) )
         cur_spd *= 1.07 - 0.07*(s.to_end/(s.cur_len - trk_data.path[sID[0]]->apex));
      end_seg = crn_RAD2FT(end_seg, s.cur_rad, wid);
   }
   nex_spd = cur_spd;

   // check the next 4 segments
   for (i = 1; i < 5; i++) {
      if (trk_data.nseg <= i)
         break;

      dist2seg += end_seg;
      if ( (trk_data.path[sID[i-1]]->seg_type != 0)&&
           (trk_data.path[sID[i-1]]->num == 1) ) {
         dist2seg += crn_RAD2FT(trk_data.path[sID[i-1]]->pc[0]->len,
                                trk_data.path[sID[i-1]]->pc[0]->rad, wid);
         brk_dist  = 1.1;
         end_seg   = 0.0;
      }
      else if ( (trk_data.path[sID[i-1]]->seg_type != 0)&&
                (trk_data.path[sID[i-1]]->num == 2) ) {
         dist2seg += crn_RAD2FT(trk_data.path[sID[i-1]]->pc[0]->Fl,
                                trk_data.path[sID[i-1]]->pc[0]->rad, wid);
         brk_dist  = 1.1;
         end_seg   = trk_data.path[sID[i-1]]->pc[1]->len - trk_data.path[sID[i-1]]->pc[1]->Sl;
         end_seg   = crn_RAD2FT(end_seg, trk_data.path[sID[i-1]]->pc[1]->rad, wid);
      }
      else if (trk_data.path[sID[i-1]]->seg_type != 0) {
         dist2seg += crn_RAD2FT(trk_data.path[sID[i-1]]->pc[1]->Fl,
                                trk_data.path[sID[i-1]]->pc[1]->rad, wid);
         brk_dist  = 1.1;
         end_seg   = trk_data.path[sID[i-1]]->pc[2]->len - trk_data.path[sID[i-1]]->pc[2]->Sl;
         end_seg   = crn_RAD2FT(end_seg, trk_data.path[sID[i-1]]->pc[2]->rad, wid);
      }
      else {
         dist2seg += trk_data.path[sID[i-1]]->pc[1]->Fl;
         brk_dist  = 1.0;
         end_seg   = trk_data.path[sID[i-1]]->pc[2]->Fl - trk_data.path[sID[i-1]]->pc[2]->Sl;
         if (end_seg < 0.0)
            end_seg = 0.0;
      }

      tst_spd = trk_data.path[sID[i]]->spd_cur;
      if (trk_data.path[sID[i]]->seg_type)
         tst_spd *= 1.0 - err*(1.0 - 0.6/trk_data.path[sID[i]]->spd);
      brk_dist *= brkdist(s.v, tst_spd);
      if ( (brk_dist >= dist2seg)&&(nex_spd > tst_spd) )
         nex_spd = tst_spd;
   }

   if (nex_spd < cur_spd) {
      // some braking might be needed
      if (trk_data.path[sID[0]]->seg_type != 0)
         nex_spd = -nex_spd;
   }
   return nex_spd;
}

int SmoothB::anti_collision(SBLine &cV, SBVector &nV, situation &s,
                            double path, int no_pass)
{
   static const double BIG = 10000.0;
   double   look_ahead;
   int      i, j, k, retval, left, no_lft, no_rgt, braking;
   double   ang, c_ang, wid, min_v;
   SBVector pos, my_pos, tst;
   double   lane[NEARBY_CARS];

   braking  = 0;
   no_lft   = no_rgt = 0;
   wid      = s.to_lft + s.to_rgt;
   ang      = cV.dir.ang();
   my_pos.x = nV.x * SB_DT; my_pos.y = nV.y * SB_DT;
   my_pos.rotate(-ang);

   if ( (s.cur_rad == 0.0)&&(s.nex_rad == 0.0) ) {
      if (s.after_rad > 0.0)
         left = 1;
      else
         left = 0;
   }
   else if ( ((s.cur_rad == 0.0)&&(s.nex_rad > 0.0))||
             (s.cur_rad > 0.0) )
      left = 1;
   else
      left = 0;

   for (j = 0; j < NEARBY_CARS; j++) {
      lane[j] = BIG;
   }

   min_v = BIG;
   for (i = 0; i < PTS; i++) {
      retval = 0;
      for (j = 0; j < NEARBY_CARS; j++) {
         if ( (s.nearby[j].who >= MAX_CARS)||
              (s.nearby[j].to_lft < -30.0)||
              (s.nearby[j].to_rgt < -30.0) )
            continue;

         if (s.nearby[j].dist <= CARLEN) {
            ang     = s.nearby[j].to_lft - s.to_lft;
            no_rgt |= (ang > 0.0)&&(ang < ( 2.5*CARWID));
            no_lft |= (ang < 0.0)&&(ang > (-2.5*CARWID));
         }

         ang = nV.mag() - s.nearby[j].v;
         if (ang > 0.0)
            ang = s.nearby[j].dist / ang;
         else
            ang = BIG;
         if ( (ang > SB_SAFE_TIME)&&(s.nearby[j].dist > 70.0) )
            continue;

         pos.x =   s.nearby[j].rel_y + (s.nearby[j].rel_ydot*SB_DT);
         pos.y = -(s.nearby[j].rel_x + (s.nearby[j].rel_xdot*SB_DT));

         look_ahead = (s.nearby[j].rel_ydot > -10.0)?(1.5*CARLEN):(-54.6*s.nearby[j].rel_ydot*SB_DT);

         tst.x = pos.x; tst.y = pos.y;
         tst.sub(my_pos);
         tst.rotate(-my_pos.ang());
         c_ang = tst.ang();
         if (c_ang > MY_PI)
            c_ang -= MY_PI21;

         pos.x = -5.0 - CARWID;
         pos.y =  5.0 + CARWID;
         if (s.vn < 0.1)
            pos.x -= s.to_rgt + CARLEN;
         else if (s.vn > 0.1)
            pos.y += s.to_lft + CARLEN;

         if ( (tst.y > pos.x)&&(tst.y < pos.y)&&(tst.x < look_ahead)&&
              (fabs(c_ang) <= VIEW_ANG) ) {
            retval++;
            if ( (!i)&&(s.nearby[j].v < min_v) ) {
               if ( (s.nearby[j].to_lft >= 0.0)&&
                    (s.nearby[j].to_rgt >= 0.0) )
                  min_v = s.nearby[j].v;
               else if (s.nearby[j].to_lft < 0.0) {
                  min_v *= s.nearby[j].to_lft / -30.0;
                  min_v += ((-30.0+s.nearby[j].to_lft)/-30.0)*s.nearby[j].v;
               }
               else {
                  min_v *= s.nearby[j].to_rgt / -30.0;
                  min_v += ((-30.0+s.nearby[j].to_rgt)/-30.0)*s.nearby[j].v;
               }
               braking  = s.nearby[j].braking;
               braking |= s.nearby[j].to_lft < 0.0;
               braking |= s.nearby[j].to_rgt < 0.0;
            }
         }

         look_ahead *= 0.7;
         if (look_ahead < (3.0*CARLEN))
            look_ahead = 3.0 * CARLEN;
         if ( (s.nearby[j].dist < look_ahead)&&
              (s.nearby[j].v <= (s.v - 2.0)) ) {
            for (k = 0; k < NEARBY_CARS-1; k++) {
               if (lane[k] >= s.nearby[j].to_lft)
                  break;
            }
            if (lane[k] > s.nearby[j].to_lft) {
               tst.x = s.nearby[j].to_lft;
               for (; k < NEARBY_CARS; k++) {
                  tst.y   = lane[k];
                  lane[k] = tst.x;
                  tst.x   = tst.y;
               }
            }
         }
      }
      if (!retval)
         break;
   }

   no_pass |= (no_lft)&&(no_rgt);
   if (i == PTS) {
      if (!no_pass) {
         // try and find the best alternative
         i = -1;
         j =  NEARBY_CARS;
         for (k = 0; k < NEARBY_CARS; k++) {
            // find cars to left
            if ( (lane[k] != BIG)&&(lane[k] <= s.to_lft) )
               i = k;
            // find cars to right
            if ( (lane[NEARBY_CARS-k-1] != BIG)&&(lane[NEARBY_CARS-k-1] >= s.to_lft) )
               j = NEARBY_CARS-k-1;
         }
         tst.x = -BIG;
         tst.y =  BIG;

// RIGHT LANE
         if ( (j == NEARBY_CARS)&&(i >= 0)&&(!no_rgt) ) {
            // look for free lane to the right
            // all opponents are to the left
            if ((path - lane[i]) > (CARWID + 5.0))
               tst.y = path;
            else if ((s.to_lft - lane[i]) > (CARWID + 5.0))
               tst.y = s.to_lft;
            else if ((wid - lane[i]) > (CARWID + 10.0)) {
               pos.y = 0.5*(wid + lane[i]);
               tst.y = lane[i] + CARWID + 10.0;
               tst.y = (pos.y < tst.y) ? pos.y : tst.y;
            }
         }
         for (k = j; (k < NEARBY_CARS)&&(!no_rgt); k++) {
            // look for free lanes to the right
            if (lane[k] == BIG)
               lane[k] = lane[k-1] + (2.0*CARWID) + 10.5;

            if (k) {
               if ((lane[k] - lane[k-1]) > (2.0*CARWID + 10.0)) {
                  if ( ((path-lane[k-1]) > CARWID)&&
                       ((lane[k] - path) > CARWID) )
                     tst.y = path;
                  else {
                     pos.y = 0.5*(lane[k] + lane[k-1]);
                     tst.y = lane[k] - CARWID - 5.0;
                     tst.y = (pos.y > tst.y) ? pos.y : tst.y;
                  }
               }
            }
            else {
               if (lane[k] > (CARWID + 10.0)) {
                  if ((lane[k] - path) > (CARWID + 5.0))
                     tst.y = path;
                  else {
                     pos.y = 0.5*lane[k];
                     tst.y = lane[k] - CARWID - 5.0;
                     tst.y = (pos.y > tst.y) ? pos.y : tst.y;
                  }
               }
            }
            if ( (tst.y >= s.to_lft)||(tst.y == path) )
               break;
         }

// LEFT LANE
         if ( (i == -1)&&(j < NEARBY_CARS)&&(!no_lft) ) {
            // look for free lane to the left
            // all opponents are to the right
            if ((lane[j] - path) > (CARWID + 5.0))
               tst.x = path;
            else if ((lane[j] - s.to_lft) > (CARWID + 5.0))
               tst.x = s.to_lft;
            else if (lane[j] > (CARWID + 10.0)) {
               pos.x = 0.5*lane[j];
               tst.x = lane[j] - CARWID - 10.0;
               tst.x = (pos.x > tst.x) ? pos.x : tst.x;
            }
         }
         for (k = i; (k > 0)&&(!no_lft); k--) {
            // look for free lanes to the left
            if ((k+1) < NEARBY_CARS) {
               if ((lane[k+1] - lane[k]) > (2.0*CARWID + 10.0)) {
                  if ( ((path - lane[k]) > (CARWID + 5.0))&&
                       ((lane[k+1]-path) > (CARWID + 5.0)) )
                     tst.x = path;
                  else {
                     pos.x = 0.5*(lane[k+1] + lane[k]);
                     tst.x = lane[k] + CARWID + 5.0;
                     tst.x = (pos.x < tst.x) ? pos.x : tst.x;
                  }
               }
            }
            else {
               if ((wid - lane[k]) > (CARWID + 10.0)) {
                  if ((path - lane[k]) > (CARWID + 5.0))
                     tst.x = path;
                  else {
                     pos.x = 0.5*(wid + lane[k]);
                     tst.x = lane[k] + CARWID + 5.0;
                     tst.x = (pos.x < tst.x) ? pos.x : tst.x;
                  }
               }
            }
            if ( (tst.x <= s.to_lft)||(tst.x == path) )
               break;
         }

         pos.x = fabs(tst.x - path);
         pos.y = fabs(tst.y - path);
         ang   = (pos.x <= pos.y) ? pos.x : pos.y;

         if (ang <= 1.0) {
            ang    = nV.mag();
            min_v += 8.0;
            c_ang  = (min_v - s.v) / delta_time;
            if (c_ang < (-0.5*MAX_ACCEL))
               min_v = s.v - (0.5*MAX_ACCEL * delta_time);
            if (min_v < ang) {
               nV.scale(min_v / ang);
               return 6;
            }
            return 2;
         }
         else {
         if (left) {
            if ( (tst.x >= SB_IN_WID)&&(tst.x < (wid-SB_OUT_WID)) ) {
               nV.y = tst.x;
               return 1;
            }
            else if ( (tst.y >= SB_IN_WID)&&(tst.y < (wid-SB_OUT_WID-35.0)) ) {
               nV.y = tst.y;
               return 5;
            }
         }
         else {
            if ( (tst.y > SB_OUT_WID)&&(tst.y <= (wid-SB_IN_WID)) ) {
               nV.y = tst.y;
               return 3;
            }
            else if ( (tst.x > (SB_OUT_WID+35.0))&&(tst.x <= (wid-SB_IN_WID)) ) {
               nV.y = tst.x;
               return 7;
            }
         }
         }
      }

      // alternative paths failed, must slow down
      ang   = nV.mag();
      if ( (braking)||((ang - s.v) < 1.5) )
         min_v *= 0.95;
      else
         min_v *= ((s.v - min_v) > 5.0) ? 0.95 : 1.0;
      c_ang = (min_v - s.v) / delta_time;
      if (c_ang < (-0.9*MAX_ACCEL))
         min_v = s.v - (0.9*MAX_ACCEL * delta_time);
      if (min_v < ang) {
         nV.scale(min_v / ang);
         return 4;
      }
      return 2;
   }
   return 0;
}

void SmoothB::passing_action(situation &s, double lane, double bias,
                             int trouble)
{
   static const double SGs = 1.0;
   static const double SDs = SGs*2.5/1.5;
   static const double SD  = 1.25;
   static const double TD  = 1.75;
   double m = -0.085 / (s.to_lft + s.to_rgt - 25.0);
   double b =  1.0 - (5.0 * m);
   double SPDc;
   double tgt_v, dv, radius, trk_wid;
   double tgt_dist, tst_rad, tst_v, tst_dist;

   trk_wid   = s.to_lft + s.to_rgt;
   cp->alpha = (SGs*(s.to_lft - lane)/trk_wid) - (SDs*(s.vn/s.v)) + bias;
   if (trouble) {
      if (cp->alpha > 0.5)
         cp->alpha =  0.5;
      else if (cp->alpha < -0.5)
         cp->alpha = -0.5;
   }
   else {
      bias *= 0.2;
      if (bias > 0.0) {
         if (cp->alpha > 0.4)
            cp->alpha =  0.4;
         else if (cp->alpha < -bias)
            cp->alpha = -bias;
      }
      else if (bias < 0.0) {
         if (cp->alpha > -bias)
            cp->alpha =  -bias;
         else if (cp->alpha < -0.4)
            cp->alpha = -0.4;
      }
   }

   radius = s.cur_rad;
   if (radius > 0.0) {
      radius += lane;
      SPDc    = (m * lane) + b;
   }
   else if (radius < 0.0) {
      radius -= trk_wid - lane;
      SPDc    = (m * (trk_wid - lane)) + b;
   }
   tgt_v    =  SPDc*crn_spd(radius);
   tgt_dist = -brkdist(s.v, tgt_v) * ((s.cur_rad == 0.0)?SD:TD);
   if ( (fabs(radius) > fabs(s.nex_rad))||(radius == 0.0) ) {
      if (s.nex_rad != 0.0) {
         tst_rad   = s.nex_rad;
         tst_dist  = -2.5 * s.v * delta_time;
         tst_dist += s.to_end*((s.cur_rad == 0.0)?1.0 : fabs(s.cur_rad));

         if (tst_rad > 0.0) {
            tst_rad += lane;
            SPDc     = (m * lane) + b;
         }
         else {
            tst_rad -= trk_wid - lane;
            SPDc     = (m * (trk_wid - lane)) + b;
         }
         tst_v     = SPDc*crn_spd(tst_rad);
         tst_dist -= brkdist(s.v, tst_v) * ((s.cur_rad == 0.0)?SD:TD);

         if (tst_dist < tgt_dist) {
            tgt_dist = tst_dist;
            tgt_v    = tst_v;
            radius   = tst_rad;
         }
      }
   }
   if ( (fabs(radius) > fabs(s.after_rad))||(radius == 0.0) ) {
      if (s.after_rad != 0.0) {
         tst_rad   = s.after_rad;
         tst_dist  = -5.0 * s.v * delta_time;
         tst_dist += s.to_end *((s.cur_rad == 0.0)?1.0 : fabs(s.cur_rad));
         tst_dist += s.nex_len*((s.nex_rad == 0.0)?1.0 : fabs(s.nex_rad));

         if (tst_rad > 0.0) {
            tst_rad += lane;
            SPDc     = (m * lane) + b;
         }
         else {
            tst_rad -= trk_wid - lane;
            SPDc     = (m * (trk_wid - lane)) + b;
         }
         tst_v     = SPDc*crn_spd(tst_rad);
         tst_dist -= brkdist(s.v, tst_v) * ((s.cur_rad == 0.0)?SD:TD);

         if (tst_dist < tgt_dist) {
            tgt_dist = tst_dist;
            tgt_v    = tst_v;
            radius   = tst_rad;
         }
      }
   }
   if ( (fabs(radius) > fabs(s.aftaft_rad))||(radius == 0.0) ) {
      if (s.aftaft_rad != 0.0) {
         tst_rad   = s.aftaft_rad;
         tst_dist  = -5.0 * s.v * delta_time;
         tst_dist += s.to_end   *((s.cur_rad == 0.0)  ?1.0 : fabs(s.cur_rad));
         tst_dist += s.nex_len  *((s.nex_rad == 0.0)  ?1.0 : fabs(s.nex_rad));
         tst_dist += s.after_len*((s.after_rad == 0.0)?1.0 : fabs(s.after_rad));

         if (tst_rad > 0.0) {
            tst_rad += lane;
            SPDc     = (m * lane) + b;
         }
         else {
            tst_rad -= trk_wid - lane;
            SPDc     = (m * (trk_wid - lane)) + b;
         }
         tst_v     = SPDc*crn_spd(tst_rad);
         tst_dist -= brkdist(s.v, tst_v) * ((s.cur_rad == 0.0)?SD:TD);

         if (tst_dist < tgt_dist) {
            tgt_dist = tst_dist;
            tgt_v    = tst_v;
            radius   = tst_rad;
         }
      }
   }

   dv = 32.2 * delta_time * ((s.cur_rad == 0.0)?1.25:0.95);
   if ((tgt_v - s.v) > dv)
      tgt_v = s.v + dv;
   else if ((tgt_v - s.v) < -dv)
      tgt_v = s.v - dv;
   cp->vc = tgt_v * cos(cp->alpha);
}
//-----END SBDriver.cpp-----//


// STATIC FUNCTIONS
//-----START spd_func.cpp-----//
static double crn_spd(double radius)
{
   double retval;

   if (radius == 0.0)
      return 400.0;

   retval = sqrt(fabs(radius) * MAX_ACCEL);

   return (retval < 400.0) ? retval : 400.0;
}

static double crn_RAD2FT(double len, double rad, double wid)
{
   double retval;

   retval = len * (fabs(rad) + (0.5*wid));
   return retval;
}

static double d_brk(double v)
{
   double retval;

   if (v < 250.0) {
      retval = 864.2473 - (0.217221*v) - (0.013173*v*v);
   }
   else
      retval = 0.0;
   return retval;
}

static double brkdist(double vi, double vf)
{
   if (vf >= vi)
      return 0.0;

   return ( (1.0 + ((vi - vf)*0.003)) * (d_brk(vf) - d_brk(vi)) );
}
//-----END spd_func.cpp-----//


Driver * getSmoothBInstance()
{
   static int num = 2;

   return new SmoothB(num++);
}

#undef SB_TESTING
