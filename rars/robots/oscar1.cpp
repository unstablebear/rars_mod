#include <math.h>
#include "car.h"
#define k(v) 6.15*sqrt(34+fabs(v))
con_vec O1(situation& s){con_vec r;double 
e,v,c,n,x;my_name_is("O1");e=s.to_end;x=s.nex_rad;n=k(x);c=s.cur_rad;if(c){v=k(c);if(x)if(n
<s.v)if(e*55*(20+(c<0?-c:c))<s.v*s.v-n*n)v=n;}else{v=250;if(n<s.v)if(72*e<s.v*s.v-
n*n)v=n;}r.vc=v;r.alpha=((c?(c<0?-50:50):(x<0?-50:50))+s.to_lft-
s.to_rgt)*.0075+(s.dead_ahead?0.1:0)-2*s.vn/s.v;

if(s.starting) r.fuel_amount = MAX_FUEL;
r.request_pit = 0;
if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10))
  {
  r.request_pit = 1;
  r.repair_amount=s.damage;
  r.fuel_amount = MAX_FUEL;
  }
return r;}

