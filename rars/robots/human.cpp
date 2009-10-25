/**
 * Human - Robot driver for RARS v0.90
 *         Allows to drive manually with a joypad.
 *         Everyone is welcome to improve this driver.
 * 
 * Filename:  human.cpp
 * Robot:     Human
 * Races:     all
 * Source:    Public
 * Color:     Yellow/Black
 * Data:      no need for a data file, but needs joystick input.
 *
 * Date:			05.02.2003
 * Version:		1
 *
 * Author:    Eugen Treise / Germany <eugentreise@uni.de>
 *
 */

#include "joystick.h"
#include "car.h"

class Human : public Driver
{
	
private:
  bool RequestedPitStop;

  Joystick JoyStick;
  JoystickStatus js;

public:

  Human::Human()
  {
    m_sName = "Human";
    m_sAuthor = "Eugen Treise";
		m_sDescription = "Human v1 05.02.2003 by ET; for RARS v0.90";
    m_iNoseColor = oYELLOW;	//helm color
    m_iTailColor = oBLACK;	//car color
    m_sBitmapName2D = "car_yellow_black";
		m_sModel3D = NULL;	// model in 3D view

    JoyStick.init();
  }

	// Destructor for the robot
	Human::~Human()
	{
    JoyStick.close();
	}


  con_vec drive(situation& s)
  {
    con_vec result;                     

    if( s.starting )
    {
      result.fuel_amount = MAX_FUEL;
      RequestedPitStop = false;

      result.alpha = 0.0;
      result.vc = s.v + 50;
      return result;
    }


    if( JoyStick.GetStatus(js) )
    {
      // steering
      if( js.x > 5000 )           // right
        result.alpha = -0.1;
      else if( js.x < -5000 )      // left
        result.alpha = 0.1;
      else                        // straight
        result.alpha = 0.0;

      // speed
      if( js.button1 )            // accelerate
        result.vc = s.v + 50;
      else                        // hold speed
        result.vc = s.v;

      if( js.button2 )            // brake
      {
        // simple ABS
        if( result.alpha == 0.0 )
          result.vc = 0.0;
        else
          result.vc = s.v * 0.9;
      }

      // pit stop
      if( js.button3 )
      {
        RequestedPitStop = true;
      }

      if( RequestedPitStop )
      {
        // if is comming out of pits
        if( s.out_pits == 1)
        {
          RequestedPitStop = false;
        }
        else
        {
          // Request pit stop
          result.request_pit = 1;
          result.fuel_amount = MAX_FUEL;
          result.repair_amount = MAX_DAMAGE;
        }
      }
    }


    // use an internal function to get back on track when stuck
    if( s.out_pits!=1 && stuck( s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc ) )
    {
      return result;
    }

		return result;
	}
};

Driver * getHumanInstance()
{ 
  return new Human();
}
