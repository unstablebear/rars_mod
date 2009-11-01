/**
 * DRIVERS.CPP - list of the drivers and their characteristics
 *
 * If you are changing number of drivers here, change also MAXCARS in car.h !!!
 * These are the control or "driver" programs which compete in the race:
 * See drivers[] below, and CAR.H
 *
 * @author    Mitchell E. Timin, State College, PA 
 * @see:      C++ Coding Standard and CCDOC in help.htm
 * @version   0.76
 */

//--------------------------------------------------------------------------
//                           I N C L U D E
//--------------------------------------------------------------------------

#include "car.h"

//--------------------------------------------------------------------------
//                           G L O B A L S
//--------------------------------------------------------------------------

robot Apex1;
robot Apex8;
robot Bug;
robot Bulle;
robot Burns;
//robot chaser1;
robot DGN;
robot Djoefe;
robot Dodger5;
robot Dodger6;
robot Dodger7;
//robot Dodger8;
robot DougE1;
robot Felix16;
robot Gryma1;
robot Jammer;
robot Jas;
robot JOCOSA83;
robot K1999;
robot K2001;
robot Magic;
robot Mat1;
robot O1;
robot OscCar2;
robot Rach221;
robot SAD01;
robot Sarah;
robot Sparky5;
//robot Sparky6;
robot Stocker;
robot Tokkie;
robot Vector;
robot Viper2H;
robot WappuCar;
//robot Weaver1;

robot2 getBulle2Instance;
robot2 getHumanInstance;
robot2 getJppInstance;
robot2 getMafanjaInstance;
robot2 getSmoothBInstance;
robot2 getTutorial1Instance;
robot2 getTutorial2Instance;
robot2 getTutorial3Instance;
robot2 getTutorial4Instance;

#ifndef X_WINDOWS
  // Human robots do not exist in XWindows
  robot2 getHumanInstance;
#endif

/**
 * This is the permanent array of available drivers.
 *
 * Each bitmap name is associated with 2 colors.
 */
Driver * drivers[] = 
{
  ////////////////////////////////////////////////////////////////////////////
  //  Race Season 2001 cars
  ////////////////////////////////////////////////////////////////////////////

  getBulle2Instance(),
  getJppInstance(),
  getMafanjaInstance(),
  getSmoothBInstance(),
  new DriverOld( Sparky5,   oRED,       oBLACK,      "car_red_black",    NULL,     "Sparky5" ),
  new DriverOld( K1999,     oBLACK,     oBLACK,      "car_black_black",  NULL,     "K1999" ),
  new DriverOld( K2001,     oBLACK,     oBLACK,      "car_black_black",  NULL,     "K2001" ),
  new DriverOld( JOCOSA83,  oLIGHTMAGENTA, oLIGHTMAGENTA, "car_orange_orange",NULL,"Jocosa83" ),
  new DriverOld( Felix16,   oLIGHTGREEN,oLIGHTRED,   "car_lgreen_lred",  NULL,     "Felix16" ),
  new DriverOld( Apex1,     oLIGHTGRAY, oLIGHTGRAY,  "car_gray_gray",    NULL,     "Apex1" ),
  new DriverOld( Dodger6,   oWHITE,     oBLACK,      "car_white_black",  NULL,     "Dodger6" ),
  new DriverOld( Apex8,     oLIGHTGRAY, oLIGHTGRAY,  "car_gray_gray",    NULL,     "Apex8" ),
  new DriverOld( Djoefe,    oYELLOW,    oYELLOW,     "car_yellow_yellow",NULL,     "Djoefe" ),
  new DriverOld( O1,        oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "O1" ),
  new DriverOld( OscCar2,   oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "OscCar2" ),
  ////////////////////////////////////////////////////////////////////////////

  new DriverOld( DougE1,    oBLUE,      oBLUE,       "car_blue_blue",    NULL,     "DougE1" ),
  new DriverOld( Vector,    oRED,       oRED,        "car_red_red",      "porshe", "Vector" ),
  new DriverOld( Bulle,     oGREEN,     oGREEN,      "car_green_green",  NULL,     "Bulle" ),
  new DriverOld( WappuCar,  oWHITE,     oWHITE,      "car_white_white",  "futura", "WappuCar" ),

  #ifndef X_WINDOWS
    // Human robots do not exist in XWindows
    getHumanInstance(),
  #endif

  NULL
/*
  new DriverOld( Jas,       oYELLOW,    oBLACK,      "car_yellow_black", NULL,     "Jas" ),
  new DriverOld( Magic,     oWHITE,     oRED,        "car_white_red",    NULL,     "Magic" ),
  new DriverOld( SAD01,     oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "SAD01" ),
  new DriverOld( DGN,       oRED,       oYELLOW,     "car_red_yellow",   NULL,     "DGN" ),
  new DriverOld( Burns,     oYELLOW,    oBLACK,      "car_yellow_black", NULL,     "Burns" ),
  new DriverOld( Sarah,     oLIGHTBLUE, oBLUE,       "car_lblue_blue",   NULL,     "Sarah" ),
  new DriverOld( Stocker,   oLIGHTRED,  oLIGHTRED,   "car_lred_lred",    NULL,     "Stocker" ),
  new DriverOld( Mat1,      oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "Mat1" ),
  new DriverOld( Rach221,   oRED,       oYELLOW,     "car_red_yellow",   NULL,     "Rach221" ),
  getTutorial1Instance(),
  getTutorial2Instance(),
  getTutorial3Instance(),
  getTutorial4Instance(),
  NULL

  new DriverOld( Sparky6,   oRED,       oBLACK,      "car_red_black",    NULL,     "Sparky6" ),
  new DriverOld( Dodger8,   oWHITE,     oBLACK,      "car_white_black",  NULL,     "Dodger8" ),
  new DriverOld( Bug,       oLIGHTGREEN,oLIGHTRED,   "car_lgreen_lred",  NULL,     "Bug" ),
*/
};

Driver* getDriver(char* driver_name)
{
  Driver* driver = NULL;
  
  if (strcmp(driver_name, "Sparky5") == 0) 
    driver = new DriverOld( Sparky5,   oRED,       oBLACK,      "car_red_black",    NULL,     "Sparky5" );
  else
    if (strcmp(driver_name, "K1999") == 0) 
      driver = new DriverOld( K1999,     oBLACK,     oBLACK,      "car_black_black",  NULL,     "K1999" );
    else
      if (strcmp(driver_name, "K2001") == 0) 
	driver = new DriverOld( K2001,     oBLACK,     oBLACK,      "car_black_black",  NULL,     "K2001" );
      else
	if (strcmp(driver_name, "Jocosa83") == 0) 
	  driver = new DriverOld( JOCOSA83,  oLIGHTMAGENTA, oLIGHTMAGENTA, "car_orange_orange",NULL,"Jocosa83" );
	else
	  if (strcmp(driver_name, "Felix16") == 0)
	    driver = new DriverOld( Felix16,   oLIGHTGREEN,oLIGHTRED,   "car_lgreen_lred",  NULL,     "Felix16" );
	  else
	    if (strcmp(driver_name, "Apex1") == 0)
	      driver = new DriverOld( Apex1,     oLIGHTGRAY, oLIGHTGRAY,  "car_gray_gray",    NULL,     "Apex1" );
	    else
	      if (strcmp(driver_name, "Dodger8") == 0)
		driver = new DriverOld( Dodger6,   oWHITE,     oBLACK,      "car_white_black",  NULL,     "Dodger6" );
	      else
		if (strcmp(driver_name, "Apex8") == 0)
		  driver = new DriverOld( Apex8,     oLIGHTGRAY, oLIGHTGRAY,  "car_gray_gray",    NULL,     "Apex8" );
		else
		  if (strcmp(driver_name, "Djoefe") == 0)
		    driver = new DriverOld( Djoefe,    oYELLOW,    oYELLOW,     "car_yellow_yellow",NULL,     "Djoefe" );
		  else
		    if (strcmp(driver_name, "O1") == 0)
		      driver = new DriverOld( O1,        oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "O1" );
		    else
		      if (strcmp(driver_name, "OscCar2") == 0)
			driver = new DriverOld( OscCar2,   oMAGENTA,   oMAGENTA,    "car_pink_pink",    NULL,     "OscCar2" );
		      else
			if (strcmp(driver_name, "DougE1") == 0)
			  driver = new DriverOld( DougE1,    oBLUE,      oBLUE,       "car_blue_blue",    NULL,     "DougE1" );
			else
			  if (strcmp(driver_name, "Vector") == 0)
			    driver = new DriverOld( Vector,    oRED,       oRED,        "car_red_red",      "porshe", "Vector" );
			  else
			    if (strcmp(driver_name, "Bulle") == 0)
			      driver = new DriverOld( Bulle,     oGREEN,     oGREEN,      "car_green_green",  NULL,     "Bulle" );
			    else
			      if (strcmp(driver_name, "WappuCar") == 0)
				driver = new DriverOld( WappuCar,  oWHITE,     oWHITE,      "car_white_white",  "futura", "WappuCar" );
			      else
				if (strcmp(driver_name, "Bulle2") == 0)
				  driver = getBulle2Instance();
				else
				  if (strcmp(driver_name, "J++") == 0)
				    driver = getJppInstance();
				  else
				    if (strcmp(driver_name, "Mafanja") == 0)
				      driver = getMafanjaInstance();
				    else
				      if (strcmp(driver_name, "SmoothB2") == 0)
					driver = getSmoothBInstance();
  #ifndef X_WINDOWS
  // Human robots do not exist in XWindows
				      else
					if (strcmp(driver_name, "Human") == 0)
					  driver = getHumanInstance();
   #endif

  //NULL
  
  return driver;
}
