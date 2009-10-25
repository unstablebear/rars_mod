/*---------------------------------------------------------------------------
| Date:       4/05/98                                                       |
| Filename:   sarah.cpp                                                      |
| Author:     Daniel Ash Boise, Idaho USA                                    |
| e-mail:     deash@primenet.com                                             |
| Robot name: Sarah                                                          |
| Races:      All - this driver replaces any drivers that were submitted     |
|             earlier                                                        |
|                                                                            |
| This file is NON-confidential                                              |
| This robot doesn't use any data files.                                     |
| This robot has been tested on rars 0.70a                                   |
|                                                                            |
|  9/19/97 - beginning rewrite of the whole driver program.                  |
| 12/05/97 - touching up c-curves and speed calculations.                    |
| 04/05/98 - touch up curve speeds, c-curves, & s-curves.                    |
|___________________________________________________________________________*/

/*----------------------------------------------------------------------------
|  Driver Information                                                         |
|                                                                             |
|  Machine: Pentium 90                                                        |
|                                                                             |
|  Compiler: Borland C++ 5.0                                                  |
|                                                                             |
|  First Driver: July, 1995 (for version 0.62) based off Timin's tutorials.   |
|                Last run around Feb. of 1997 (I think)                       |
|  Latest Driver: Apr., 1998 (for version 0.70a) based off my own design.     |
|                 Last run in Mar., 1998.                                     |
|                                                                             |
|  Description: My driver takes each curve and situation as it comes.  Uses   |
|               only the information given in the s data structure.  This     |
|               limits the look ahead which means the driver must be cautious |
|               when coming across several short sections.                    |
|                                                                             |
|  Limitations: Still have yet to implement passing. Need to work on the path |
|               my car takes also, some times it takes a very bad path.       |
|____________________________________________________________________________*/

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "car.h"

static double ash_001(double, double, double);
static double ash_002(double, double, double);
static void ash_003(situation, double *, double *, double);
static double ash_004(situation, double, double);
static double ash_005(double, double, double, double);
static double ash_006(situation, double, double, double, double, double, double, double);
// static int ash_007(situation, double *, double *, double, double);
static int ash_008(situation, double *, double, int, double);
static double ash_009(situation, double);
static double ash_010(situation, double);
// static double ash_011 (situation, double);
static int ash_012 (situation, double *, int);
static double ash_013(double, double);
static double ash_014(double, double);
static double ash_015(situation, double, double);
static double ash_016(situation, double, double);
#define min(x,y) (((x)>(y))?(y):(x))

#define BF -23

con_vec Sarah(situation &s)
{
  const char name[]    = "Sarah"; 
  static int init_flag = 1;       
  con_vec result;                 
  double alpha         = 0.0,     
               vc            = 0.0,     
               ash_017     = 0,       
               ash_018     = 0,       
               width         = 0,       
               ash_021     = 250.0,   
               ash_022      = 50.0,    
               next_lane     = 0.0,     
               ash_023 = 0.0,     
         ash_024     = 0.0;     
  int    ash_025         = 0;       

  if (init_flag == 1)
  {                      
    my_name_is(name);
    init_flag = 0;
    result.alpha = result.vc = 0;
    return result;
  }

  
  if (stuck(s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc))
  {
    return result;
  }

  width = s.to_lft + s.to_rgt;   

  if (s.vn != 0.0 && s.v != 0.0)
    ash_024 = ash_013(s.vn, s.v);
  else
    ash_024 = 0.0;

  
  ash_003(s, &ash_017, &ash_018, ash_021);

  
  if (s.time_count < 25 && s.seg_ID == 0 && s.to_end > 0.5 * s.cur_len)
  {
    if (s.vn != 0)
      alpha = - atan(s.vn / s.v);
    else
      alpha = 0.0;
    ash_025 = 2;
  }
  else  
  {
    if (s.fuel < 0.05)   
    {
      vc = s.v - 1;
      ash_025 = 1;
    }

    ash_025 = ash_012 (s, &vc, ash_025);
  }

  
  ash_025 = ash_008(s, &alpha, width, ash_025, ash_024);

  if (ash_025 == 3)
  {               
    result.vc = vc;
    result.alpha = alpha;
    return result;
  }

  if (ash_025 == 1 || ash_025 == 0)
  {
    
    alpha = ash_004(s, width, ash_024);
  }

  if (ash_025 == 2 || ash_025 == 0)
  {
    
    next_lane = ash_009(s, width);
    ash_023 = ash_010(s, ash_024);

    
    vc = ash_006(s, ash_017, ash_018, alpha, ash_023,
                                next_lane, ash_022, ash_021);
  }
  
  if(s.fuel < 7 || s.damage > 25000){
       result.request_pit = 1;
       result.fuel_amount = MAX_FUEL;
       result.repair_amount = s.damage;
  }
  result.vc = vc;
  result.alpha = alpha;

  return result;
}

static int ash_012 (situation s, double *vc, int ash_025)
{
  int    ash_026        = 0;
  double ash_027 = 0,
               ash_028     = 0;

  if (s.nearby[0].who > 16)
    return ash_025;

  for (ash_026 = 0; ash_026 < 3 && s.nearby[ash_026].who < 16; ash_026++)
  {
    if (s.nearby[ash_026].rel_ydot > 0)
      continue;

    if (s.nearby[ash_026].rel_x > 0.0)
      ash_028 = s.nearby[ash_026].rel_x;
    else
      ash_028 = -s.nearby[ash_026].rel_x;

    if (ash_028 > 1.5 * CARWID)
      continue;

    
    if (s.nearby[ash_026].rel_ydot != 0)
      ash_027 = s.nearby[ash_026].rel_y / - s.nearby[ash_026].rel_ydot;
    else
      ash_027 = 0;
      
    
    if (ash_027 < 2.5)
    {
      *vc = s.v + s.nearby[ash_026].rel_ydot;

      if (ash_025 == 2)
        return 3;
      else
        return 1;
    }
  }
  return ash_025;
}

/* Not used
static int ash_007(situation s,
                                               double *vc,
                                               double *alpha,
                                               double width,
                               double ash_024)
{
  double ash_028 = 0.0;
  int    car        = 0,
         ash_029    = 0;

  if (s.nearby[0].who > 16)    
    return 0;

 
  if (s.nearby[0].rel_y > 5 * CARLEN)
    return 0;

  if (s.nearby[0].rel_x > width || s.nearby[0].rel_x < -width)
    return 0;

  
  if (s.cur_rad != 0 && s.cur_len >= 0.524)
  {
    for (car = 0; car < 3; car++)
    {
      if (s.nearby[car].who > 16)  
        break;

                  if (s.nearby[car].rel_y > 3.0 * CARLEN) 
        continue;

      if (s.nearby[car].rel_ydot > 0) 
        continue;

      if (s.nearby[car].rel_x > 0.0)      
        ash_028 = s.nearby[car].rel_x;
      else
        ash_028 = -s.nearby[car].rel_x;

                  if (ash_028 > 1.7 * CARWID)  
        continue;

      *alpha = ash_011(s, ash_024);

      if (*alpha == 0)
                    return 0;
                  else
        *vc = ash_001(s.cur_rad, s.cur_len, s.cur_len) + 6; 

      return 3;
    }       

    return 0;
  }

  
  if (s.cur_rad == 0 || (s.cur_rad != 0 && s.cur_len < 0.524))
  {
    for (car = 0; car < 3; car++)
    {
      if (s.nearby[car].who > 16)  
        break;

      if (s.nearby[car].rel_x > 0.0)      
        ash_028 = s.nearby[car].rel_x;
      else
        ash_028 = -s.nearby[car].rel_x;

      if (ash_028 > 2.5 * CARWID)  
        continue;

      ash_029 = 1;

      if (s.nearby[car].rel_y > 3 * CARLEN) 
        continue;

      if (s.nearby[car].rel_ydot > 0) 
        continue;
    }

    if (ash_029)
    {
      *alpha = ash_011(s, ash_024);

      if (*alpha == 1400 || *alpha == 0)
      {
        *alpha = 0;
        return 0;
      }
      return 2;
    }
  }
  return 0;
}
*/

/* Not used
static double ash_011 (situation s, double ash_024)
{
  if (s.cur_rad == 0)
  {
    if (s.nex_rad > 0.0)
      return ash_024 + 0.007;  
    else
      return ash_024 - 0.007;
  }
  else
  {
    if (s.cur_rad > 0.0)
      return ash_024 + 0.007;
    else
      return ash_024 - 0.007;
  }
}
*/

static double ash_004(situation s, double width, double ash_024)
{
  double alpha = 0.0,
         ash_030 = 0.0,
         ash_031 = 0.0,
         ash_032 = 0.0,
         ash_033 = 0.0;

  if (width == 0.0 || s.v == 0.0)
    return 0.0;

  if (s.nex_rad > 0)
    ash_033 = s.to_lft - 15; 
  else
    ash_033 = s.to_rgt - 15; 

  
  if (s.cur_rad == 0.0)
  {
    
    if (s.to_end > s.cur_len * 0.5 && s.cur_len > 500)
    {
      if (s.nex_rad > 0)  
      {
        ash_031 = 0.8;
        ash_032 = 0.25;
        if (s.nex_len > 0.785) 
        {
          if (s.to_lft > width * ash_031)
            ash_030 = - atan(((width * ash_031) - s.to_lft) / (s.to_end / 2));
          else
            ash_030 = - atan(((width * ash_031) - s.to_lft) / (s.to_end / 2));
        }
        else
        {
          if (s.to_lft > width * ash_032)
            ash_030 = - atan(((width * ash_032) - s.to_lft) / (s.to_end / 2));
          else
            ash_030 = - atan(((width * ash_032) - s.to_lft) / (s.to_end / 2));
        }
      }
      else
      {    
        ash_031 = 0.25;
        ash_032 = 0.8;
        if (s.nex_len > 0.785) 
        {
          if (s.to_lft > width * ash_031)
            ash_030 = - atan(((width * ash_031) - s.to_lft) / (s.to_end / 2));
          else
            ash_030 = - atan(((width * ash_031) - s.to_lft) / (s.to_end / 2));
        }
        else
        {    
          if (s.to_lft > width * ash_032)
            ash_030 = - atan(((width * ash_032) - s.to_lft) / (s.to_end / 2));
          else
            ash_030 = - atan(((width * ash_032) - s.to_lft) / (s.to_end / 2));
        }
      }
    }
    else 
    {
      
      if (s.to_end < ash_033 / atan(0.611))
      {
        if (s.nex_rad > 0.0)
          ash_030 = 0.611;
        else
          ash_030 = -0.611;

        
        if (s.nex_len < 0.087 && s.to_end > 0)
          ash_030 = atan(-((width / 2) - s.to_lft) / s.to_end);
      }
    }
    alpha = ash_030 - ash_024;
  }
  
  else
  {
    if ((s.cur_rad > 0.0 && s.nex_rad < 0.0) ||
        (s.cur_rad < 0.0 && s.nex_rad > 0.0))
    {
      alpha = ash_016(s, width, ash_024);
    }
    else
    {
      alpha = ash_015(s, width, ash_024);
    }
  }

  return alpha;
}



static double ash_015(situation s, double width, double ash_024)
{
  double ash_030,
         to_end,
         alpha;
  int    distance_wanted;

  distance_wanted = 12;

  to_end = ash_005(s.to_end, s.cur_rad, s.to_lft, s.to_rgt);

  if (s.cur_rad > 0)
  
  {
    
    if (s.to_lft < distance_wanted)
      ash_030 = - ash_014(s.cur_rad + distance_wanted, s.cur_rad + s.to_lft);
    else
      ash_030 = ash_014(s.cur_rad + distance_wanted, s.cur_rad + s.to_lft);

    
    if (ash_030 > 0.873)  
      ash_030 = 0.0;
    alpha = ash_030 - ash_024;
  }
  else
  
  {
    if (s.to_rgt < 12)
      ash_030 = ash_014(- s.cur_rad + 12, - s.cur_rad + s.to_rgt);
    else
      ash_030 = - ash_014(- s.cur_rad + 12, - s.cur_rad + s.to_rgt);

    if (ash_030 < -0.873)
      ash_030 = 0.0;
    alpha = ash_030 - ash_024;
  }

  
  if (s.to_end < .1 * s.cur_len) 
    alpha = 0.0;

  return alpha;
}

static double ash_016(situation s, double width, double ash_024)
{
  double ash_030,
         ash_034,
         ash_035,
         alpha = 0.0;
  static int ash_025 = 0,
             seg_ID = 0;

  if (s.cur_rad > 0)
    ash_035 = s.cur_rad - s.nex_rad;
  else
    ash_035 = -s.cur_rad + s.nex_rad;

  ash_034 = ash_014(ash_035, ash_035 + width - 40);


  if (ash_025)
  {
    if (s.seg_ID == seg_ID)
    {
      alpha = 0.0;

      
      if (s.to_end < 0.1)
        ash_025 = 0;
        
      return alpha;
    }
    else
    {
      ash_025 = 0;
    }
  }

  if (s.cur_rad > 0)
  
  {
    
    if (s.to_end > ash_034)
    {
      
      ash_030 = ash_014(s.cur_rad + 10, s.cur_rad + s.to_lft);
      alpha = ash_030 - ash_024;
    }
    else
    {
      
      if (s.to_end > 0.1)
        ash_025 = 1;
      seg_ID = s.seg_ID;
    }
  }
  else
  
  {
    if (s.to_end > ash_034)
    {
      ash_030 = ash_014(s.cur_rad - 10, s.cur_rad - s.to_rgt);
      alpha = ash_030 - ash_024;
    }
    else
    {
      if (s.to_end > 0.1)
        ash_025 = 1;
      seg_ID = s.seg_ID;

    }
  }
  return alpha;
}



static double ash_013(double numerator, double denominator)
{
  if (denominator == 0.0 || numerator == 0.0)
    return 0.0;

  if (numerator / denominator > 1 || numerator / denominator < -1)
  {
    numerator = 0.999 * denominator;
  }

  if (numerator > 0.0 && denominator > 0.0)
    return asin(numerator / denominator);

  if (numerator < 0.0 && denominator < 0.0)
    return - asin (numerator / denominator);

  return - asin (-numerator / denominator);
}



static double ash_014(double numerator, double denominator)
{
  if (denominator == 0.0 || numerator == 0.0)
    return 0.0;

  if (numerator / denominator > 1 || numerator / denominator < -1)
  {
    numerator = 0.999 * denominator;
  }

  if (numerator >= 0.0 && denominator > 0.0)
  {
    return acos(numerator / denominator);
  }

  if (numerator < 0.0 && denominator < 0.0)
    return - acos (numerator / denominator);

  return - acos (-numerator / denominator);
}



static double ash_009(situation s, double width)
{
  double ash_037   = 0.0,
         ash_038 = 0.0;

  if (s.v < 60)  
  {
    if (s.nex_rad > 0.0)
      return 10;
    else
      return width - 10;
  }

  if (s.cur_rad == 0.0)
  {
    ash_038 = fabs(s.nex_rad) / -727 + 0.75;
    if (ash_038 > 0.4)
      ash_038 = 0.4;

    if (ash_038 < 0.1)
      ash_038 = 0.1;

    if ((s.nex_rad < 0.0 && s.after_rad > 0.0) ||
        (s.nex_rad > 0.0 && s.after_rad < 0.0))
      ash_038 = 0.4;

    if (s.nex_rad > 0.0)
      ash_037 = ash_038 * width;
    else
      ash_037 = (1 - ash_038) * width;
  }
  else
  {
    if (s.nex_rad > 0.0)
      ash_037 = 10;
    else
      ash_037 = width - 10;
  }

  return ash_037;
}



static double ash_010(situation s, double ash_024)
{
  double ash_039      = 0.0,
         ash_040 = 0.0,
         ash_044         = 0.0;

  if (s.nex_len < 0.087)
    return 0;

  if (s.nex_rad < 0)
    ash_044 = -s.nex_rad + s.to_rgt;
  else
    ash_044 = s.nex_rad + s.to_lft;

  if (s.to_end * s.to_end + ash_044 * ash_044 < 0)
    ash_040 = sqrt(-(s.to_end * s.to_end + ash_044 * ash_044));
  else
    ash_040 = sqrt(s.to_end * s.to_end + ash_044 * ash_044);

  if (ash_040 != 0)
    ash_039 = asin(ash_044 / ash_040);
  else
    ash_039 = 0;

  if (s.nex_rad < 0)
    ash_039 = - ash_039;

  ash_039 -= ash_024;

  
  if (ash_039 > 0.349) 
    ash_039 = 0.349;

  if (ash_039 < -0.349)
    ash_039 = -0.349;

  return ash_039;
}



static double ash_005(double ash_045,
                                                          double ash_044,
                                  double to_left,
                                                          double to_right)
{
  if (ash_044 == 0.0)
    return ash_045;     

  if (ash_044 > 0.0)
    return (ash_045 * (ash_044 + to_left));

  if (ash_044 < 0.0)
    return (ash_045 * (-ash_044 + to_right));

  return 0;
}



static double ash_006(situation s,
                                           double ash_017,
                                         double ash_018,
                                         double alpha,
                                         double ash_039, 
                                         double early_turn_dist, 
                                         double ash_022,
                                         double ash_021)
{
  double to_end           = 0.0,
               ash_041        = 0.0,
               req_dist_to_sash_032 = 0.0,
         vc               = 0.0;

  to_end = ash_005(s.to_end, s.cur_rad, s.to_lft, s.to_rgt);

  if (ash_039 < 0.0)
    ash_039 = - ash_039;

  if(s.cur_rad == 0.0)
  {                     
    req_dist_to_sash_032 = ash_002(s.v, ash_018, BF); 

    if(to_end > req_dist_to_sash_032)
      vc = s.v + ash_022;                    
    else
    {
      if (to_end > early_turn_dist)
      {                                       
        if(s.v > 1.02 * ash_018)            
          vc = s.v * 0.97;                    

        if(s.v < 0.98 * ash_018)            
          vc = s.v + ash_022;                

        if (s.v <= 1.02 * ash_018 && s.v >= 0.98 * ash_018)
          vc = (s.v + ash_018) / 2;         
      }
      else
        vc = ash_018;
    }
  }
  else  
  {   
    if (s.nex_rad == 0.0 && s.nex_len > 150 && ash_018 == ash_021)
    {
      ash_041 = ash_005(s.cur_len, s.cur_rad, s.to_lft, s.to_rgt);

      
      if (to_end <= 0.4 * ash_041)
        vc = s.v * 1.02;
      else
      {
        if (cos(alpha) != 0)
                vc = (0.47 * (s.v + ash_017)) / cos(alpha);
        else
          vc = s.v;
      }
    }

    if (s.nex_rad == 0.0 && ash_018 != ash_021)
    {
      req_dist_to_sash_032 = ash_002(s.v, ash_018, BF);

      if (to_end + s.nex_len < req_dist_to_sash_032)
        vc = ash_018 * 0.9;
      else
      {
        if (cos(alpha) != 0)
                vc = (0.47 * (s.v + ash_017)) / cos(alpha);
        else
          vc = s.v;
      }
    }

    if (s.nex_rad != 0.0)  
    {         
      to_end = ash_005(s.to_end, s.cur_rad, s.to_lft, s.to_rgt);
      req_dist_to_sash_032 = ash_002(s.v, ash_018, BF * 0.5);

      if(to_end > req_dist_to_sash_032)
      {
        if (cos(alpha) != 0)
                vc = (0.47 * (s.v + ash_017)) / cos(alpha);
        else
          vc = s.v;
      }
      else
      {
        if(s.v > 1.02 * ash_018)
          vc = ash_018;       

        if(s.v < 0.98 * ash_018)
          vc = s.v * 1.02;

        if (s.v <= 1.02 * ash_018 && s.v >= 0.98 * ash_018)
          vc = ash_018;
      }
    }
  }

  if (vc > ash_021)
    vc = ash_021;

  return vc;
}



static void ash_003(situation s,
                                           double *ash_017,
                                           double *ash_018,
                                           double ash_021)
{
  double ash_019      = 0.0,
         to_end           = 0.0,
         req_dist_to_sash_032 = 0.0,
         // ash_0451         = 0.0,
         ash_020       = 0.0;

  static double comb_ash_045 = 0.0,
                left_or_right = 0.0;

  if (s.cur_rad == 0)
  {
    if (s.nex_rad > 0)
    {
      comb_ash_045 = s.nex_len;
      left_or_right = 1;
      if (s.after_rad > 0)
      {
        comb_ash_045 += s.after_len;
        if (s.aftaft_rad > 0)
          comb_ash_045 += s.aftaft_len;
      }
    }
    else
    {
      comb_ash_045 = s.nex_len;
      left_or_right = -1;
      if (s.after_rad < 0)
      {
        comb_ash_045 += s.after_len;
        if (s.aftaft_rad < 0)
          comb_ash_045 += s.aftaft_len;
      }
    }
  }

  if (s.cur_rad > 0 && left_or_right != 1)
  {
    comb_ash_045 = s.cur_len;
    left_or_right = 1;
    if (s.nex_rad > 0)
    {
      comb_ash_045 += s.nex_len;
      if (s.after_rad > 0)
      {
        comb_ash_045 += s.after_len;
        if (s.aftaft_rad > 0)
          comb_ash_045 += s.aftaft_len;
      }
    }
  }

  if (s.cur_rad < 0 && left_or_right != -1)
  {
    comb_ash_045 = s.cur_len;
    left_or_right = -1;
    if (s.nex_rad < 0)
    {
      comb_ash_045 += s.nex_len;
      if (s.after_rad < 0)
      {
        comb_ash_045 += s.after_len;
        if (s.aftaft_rad < 0)
          comb_ash_045 += s.aftaft_len;
      }
    }
  }

  *ash_017 = ash_001(s.cur_rad, s.cur_len, comb_ash_045);
  *ash_018 = ash_001(s.nex_rad, s.nex_len, comb_ash_045);
  ash_019 = ash_001(s.after_rad, s.after_len, s.after_len);
  ash_020 = ash_001(s.aftaft_rad, s.aftaft_len, s.aftaft_len);

  if (s.cur_rad == 0.0)
  {                     
    if (s.nex_rad == 0.0) 
      *ash_018 = ash_021;
    else
    {
      
      if ((s.nex_rad > 0 && s.after_rad > 0) ||
         (s.nex_rad < 0 && s.after_rad < 0))
      {
        if (ash_019 < *ash_018)
          *ash_018 = ash_019;

        if ((s.aftaft_rad > 0 && s.nex_rad > 0) ||
            (s.aftaft_rad < 0 && s.nex_rad < 0))
        {
          if (ash_020 < *ash_018)
            *ash_018 = ash_020;
        }
      }
    }
  }
  else
  {                     
    to_end = ash_005(s.to_end, s.cur_rad, s.to_lft, s.to_rgt);
    req_dist_to_sash_032 = ash_002(s.v, *ash_018, BF * 0.5);

    if (s.nex_rad != 0.0)
    {
      if (to_end < req_dist_to_sash_032)
      {
        *ash_017 = min(*ash_018, *ash_017);
        if (s.after_rad != 0.0)
          *ash_017 = min(ash_019, *ash_017);
      }
    }

    if (s.nex_rad == 0.0)
    {
      req_dist_to_sash_032 = ash_002(s.v, ash_019, BF * 0.5);

      if (to_end + s.nex_len < req_dist_to_sash_032)
      {
        *ash_018 = ash_019;
        *ash_017 = ash_019;
      }
    }
  }
}



static double ash_001(double ash_044,
                                              double ash_045,
                            double comb_ash_045)
{
  double speed = 0.0,
         ash_043 = 0.0,
         rad_factor = 0.0;

  
  if (ash_044 == 0.0 || ash_045 < 0.087)         // if ash_044 = 0,
    return 400.0;

  if (ash_044 > 0.0)
    ash_044 = ash_044 + 30;
  else
    ash_044 = -ash_044 + 30;

  ash_043 = ash_045 * ash_044;


  if (comb_ash_045 < 0.79) 
    rad_factor = 0.1;
  else
    if (comb_ash_045 < 1.57)  
      rad_factor = 0.18;
    else
      rad_factor = 0.2;

  //sqrt(ash_044) + 42;
  speed = 0.04 * ash_043 - rad_factor * (ash_045 * 57.30) + 4.15 * sqrt(ash_044) + 42.5;

  
  if (speed < 30)
  {
    speed = 30;
  }

  return speed;
}

static double ash_002(double cur_velocity,
                                           double ash_046,
                                           double ash_047)
{
  double ash_048;  

  

  if(ash_046 - cur_velocity > 0.0 || ash_047 >= 0)
    return(0.0);       

  ash_048 = ash_046 - cur_velocity;
  return (cur_velocity + .5 * ash_048) * ash_048 / ash_047;
}



static int ash_008(situation s,
                                             double *ash_050,
                                           double width,
                           int ash_025,
                           double ash_024)
{
  if (s.vn == 0.0)
    return 0;

  
  if ((s.vn > 0.0 && s.to_lft / s.vn < 1) ||
      (s.vn < 0.0 && s.to_rgt / (-s.vn) < 1))
  {    
    if (s.cur_rad > 0.0)
    {  
      if (s.to_lft > width / 2) 
      {
        *ash_050 = ash_014(s.cur_rad + 12, s.cur_rad + s.to_lft);
        *ash_050 -= ash_024;
      }
      else
      {  
        *ash_050 = - ash_014(s.cur_rad + 12, s.cur_rad + s.to_lft);
        *ash_050 -= ash_024;
      }

      if (ash_025 == 1)
        return 3;
      else
        return 2; 
    }

    if (s.cur_rad < 0.0)
    {
      if (s.to_lft > width / 2)
      {
        *ash_050 = - ash_014(s.cur_rad - 12, s.cur_rad - s.to_rgt);
        *ash_050 -= ash_024;
      }
      else
      {
        *ash_050 = ash_014(s.cur_rad - 12, s.cur_rad - s.to_rgt);
        *ash_050 -= ash_024;
      }

      if (ash_025 == 1)
        return 3;
      else
        return 2;
    }

    if (s.cur_rad == 0.0)
    { 
      if (ash_024 > 0.0)
        *ash_050 = -0.3;
      else
        *ash_050 = 0.3;
        
      if (ash_025 == 1)
        return 3;
      else
        return 2;
    }
  }

  return ash_025;
}
