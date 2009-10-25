/**
 * Tutorial 1: 
 *
 * This car contains all the code needed by a robot
 * But it goes only straight
 *
 * @author    Marc Gueury / Belgium <mgueury@skynet.be>
 * @version   0.80
 */

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include "car.h"

//--------------------------------------------------------------------------
//                           Class Tutorial1
//--------------------------------------------------------------------------

class Tutorial1 : public Driver
{
public:
  Tutorial1::Tutorial1()
  {
    m_sName = "Tuto1";
    m_sAuthor = "Lucky Luke";
    m_iNoseColor = oBLUE;
    m_iTailColor = oBLUE;
    m_sBitmapName2D = "car_blue_blue";
    m_sModel3D = "futura";
  }

  con_vec drive(situation& s)
  {
    con_vec result = CON_VEC_EMPTY;                     

    if( s.starting )
    {
      result.fuel_amount = MAX_FUEL;     // fuel when starting
    }

    if( stuck( s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc ) )
    {
      return result;
    }

    result.vc = 20;                      // going slowly
    result.alpha = 0.0;                  // straight
    result.request_pit = 0;              // do not ask to pit

    return result;
  }
};

Driver * getTutorial1Instance()
{ 
  return new Tutorial1();
}
