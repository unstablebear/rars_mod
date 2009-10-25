/* Car name: Grymb1
   Author: Steve 'GryMor' Metke GryMor@cc.wwu.edu

   This is bassically Gryma1 without the super slow car detection and
   dodging code. It doese greate if it survies the first 4 laps

   This is GryMor's car #2 for the BORS mlwaukee race.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "car.h"

#define width (s.to_rgt + s.to_lft)

static double rdiv(double a, double b)
   {
   if(b>0 && b<0.001)
      b=0.001;
   else if(b<0 && b>-0.001)
      b=-0.001;
   return a/b;
   }

static double corn_spd(double radius)
   {
   if(radius<0)
      radius =-radius;
   return sqrt(radius * 32.2 * .92);
   }

static double CritDist(double v0, double v1, double a)
   {
   double dv;

   dv = v1 - v0;
   if(dv > 0.0)
      return(0.0);
   return rdiv((v0 + .5 * dv) * dv , a);
   }

con_vec Grymb1(situation &s)
   {
   static double edgedist = 4.4;
   static double spdadj = 14.6;
   static double acoradj = 2.00;
   static double abaseadj = 0.615;
   static double dodge = 2.5;

   static double radfin = 0.34 * 3.1415;
   // static FILE * fp;
   // char buf[120];
   const char name[] = "Grymb1";
   static int init = 1;
   static double pos = 99999;
   static int crit = 0;
   static int race = 0;
   static double lane_inc = 0;
   static int laps;
   con_vec result;
   double & alpha = result.alpha;
   double & vc = result.vc;
   double spd;
   double spdnex;
   double spdafter;
   double spdaftaft;
   double disnex;
   double disafter;
   double disaftaft;
   alpha = 0;

   if(init)
      {
      my_name_is(name);
      init = 0;
      result.vc = result.alpha = 0;
      return result;
      }

   if(s.starting)
      {
      race++;
      laps=s.laps_to_go;
      }
   // int lap = laps-s.laps_to_go;
   if(stuck(s.backward, s.v,s.vn, s.to_lft,s.to_rgt, &result.alpha,&result.vc))
      return result;

   if(pos > 90000)
      pos = s.to_lft;

   if(s.cur_rad==0 && crit == 0)
      pos = s.to_lft;


   if(s.cur_rad>0)
      {
      if(radfin > s.to_end)
         spd = 300;
      else
         spd = corn_spd(s.cur_rad+pos);
      disnex = (s.cur_rad+pos)*s.to_end;
      }
   else if(s.cur_rad<0)
      {
      spd = corn_spd(-s.cur_rad+pos);
      disnex = (-s.cur_rad+pos)*s.to_end;
      }
   else
      {
      spd = 300;
      disnex = s.to_end;
      }

   if(s.nex_rad>0)
      {
      spdnex = corn_spd(s.nex_rad+pos);
      disafter = (s.nex_rad+pos)*s.nex_len + disnex;
      }
   else if(s.nex_rad<0)
      {
      spdnex = corn_spd(-s.nex_rad+pos);
      disafter = (-s.nex_rad+pos)*s.nex_len + disnex;
      }
   else
      {
      spdnex = 300;
      disafter = s.nex_len + disnex;
      }


   if(s.after_rad>0)
      {
      spdafter = corn_spd(s.after_rad+pos);
      disaftaft = (s.after_rad+pos)*s.after_len + disafter;
      }
   else if(s.after_rad<0)
      {
      spdafter = corn_spd(-s.after_rad+pos);
      disaftaft = (-s.after_rad+pos)*s.after_len + disafter;
      }
   else
      {
      spdafter = 300;
      disaftaft = s.after_len + disafter;
      }

   if(s.aftaft_rad>0)
      {
      spdaftaft = corn_spd(s.aftaft_rad+pos);
      }
   else if(s.aftaft_rad<0)
      {
      spdaftaft = corn_spd(-s.aftaft_rad+pos);
      }
   else
      {
      spdaftaft = 300;
      }
   int critnex = disnex < CritDist(s.v,spdnex,-25)+1;
   int critafter = disafter < CritDist(s.v,spdafter,-25)+1;
   int critaftaft = disaftaft < CritDist(s.v,spdaftaft,-25)+1;

   if(critaftaft)
      {
      crit=1;
      spd = (spd > spdaftaft)?spdaftaft:spd;
      if(s.aftaft_rad > 0)
         pos = edgedist;
      else if(s.aftaft_rad < 0)
         pos = width - edgedist;
      }
   if(critafter)
      {
      crit=1;
      spd = (spd > spdafter)?spdafter:spd;
      if(s.after_rad > 0)
         pos = edgedist;
      else if(s.after_rad < 0)
         pos = width - edgedist;
      }
   if(critnex)
      {
      crit=1;
      spd=spdnex;
      if(s.nex_rad > 0)
         pos = edgedist;
      else if(s.nex_rad < 0)
         pos = width - edgedist;
      }

   if(!critnex && !critafter && !critaftaft)
      {
      if(s.cur_rad > 0)
         pos = edgedist;
      else if(s.cur_rad < 0)
         pos = width - edgedist;
      else
         {
         if(s.nex_rad > 0 )
            pos = width - edgedist*2;
         else if(s.nex_rad < 0)
            pos = edgedist*2;
         }
      crit=0;
      }
   alpha = rdiv(abaseadj*(s.to_lft-pos),width) - rdiv(acoradj * s.vn,s.v);
   vc = spd+spdadj;

   double x, y, vx, vy, dot, vsqr, c_time, y_close, x_close;
   int kount = 0;
   for(int i=0;i<5;i++) if (s.nearby[i].who<16)
      {
      y=s.nearby[i].rel_y;
      x=s.nearby[i].rel_x;
      vx=s.nearby[i].rel_xdot;
      vy=s.nearby[i].rel_ydot;
      dot = x * vx + y * vy;
      if(dot > -0.1)
         continue;
      vsqr = vx*vx + vy*vy;
      c_time = rdiv(-dot , vsqr);
      if(c_time > 3.0)
         continue;
      x_close = x + c_time * vx;
      y_close = y + c_time * vy;
      if(x_close * x < 0.0 && y < 1.1 * CARLEN) {
         c_time = rdiv((fabs(x) - CARWID), fabs(vx));
         x_close = x + c_time * vx;
         y_close = y + c_time * vy;
      }
      if(fabs(x_close) > 3 * CARWID || fabs(y_close) > 2.5 * CARLEN)
         continue;
      ++kount;
      if(kount > 1 || c_time < 1.2)
         vc = s.v - 3.5;
      if(s.cur_rad > 0.0)
         if(x_close < 0.0 || s.to_lft < edgedist)
            lane_inc += dodge;
         else
            lane_inc -= dodge;
      else if(s.cur_rad < 0.0)
         if(x_close > 0.0 || s.to_rgt < edgedist)
            lane_inc -= dodge;
         else
            lane_inc += dodge;
      else if(x_close < 0.0)
         lane_inc += dodge;
      else
         lane_inc -= dodge;
      if(lane_inc > .25 * width)
         lane_inc = .25 * width;
      else if(lane_inc < -.25 * width)
         lane_inc = -.25 * width;
    }

    if(!kount)
      if(lane_inc > .1)
         lane_inc -= .5*dodge;
      else if(lane_inc < -.001)
         lane_inc += .5*dodge;
      else
         lane_inc = 0;
   alpha -= rdiv(abaseadj * lane_inc, width);
   return result;
   }

