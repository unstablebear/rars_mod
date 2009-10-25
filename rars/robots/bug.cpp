//
// BUG.CPP
//
// Filename:  bug.cpp
// Robot:     bug
// Author:    Damian Sinclair, damian.sinclair@btinternet.com (UK)
// Races:     all
// Source:    Public
// Data:      bug.dat
//

/*	30/10/99

	Single Evolved NN driver for RARs:

    This code has been written and rewritten and hacked about for
	about 2 years now. I apologise for its readability and for the
	mish-mash of coding conventions it uses. At some point I intend
	to rewrite it in C++ using OO methods. Then again I've been
	saying that for about 2 years too.

	Two input translation functions are taken from Robert Wilderspin's
	work on an NN driver.

    Bulle.cpp was used for general reference in linking with RARS
	so there are several small bits of code from that driver in here.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <malloc.h>
//#include <conio.h>
//#include <io.h>
#include <fcntl.h> 
#include <sys/stat.h>
#include <errno.h>

#include "car.h"
#include "track.h"
#include "os.h"


#define NUMNODES 64
#define INPUTS 15
#define HONODES (NUMNODES-INPUTS)
#define IDACT 6

#define LAMBDA 1.0
#define PAST 2
#define BSCALE 0.009
#define WSCALE 0.001575 //0.009f

/*replacement defines for the config load*/

#define BiasAct 8
#define WeigAct 8


typedef struct _bug
{
	double Nodes[HONODES];				/*bias data for hidden and output nodes*/
	double Weights[HONODES][NUMNODES];   /*Weight data for H+O Nodes*/
	double Status[PAST][NUMNODES];		/*firing states of nodes*/
	double Inputs[INPUTS];				/*input values*/
}BUG;


/*Function Prototypes*/

void Bug_Setup(void);
void DoNeural(void);
void DoNode( BUG *pBug, int NodeNo);
void DoBug( situation& s );


//Global Vars

BUG BugDriver;

unsigned char masks[8] = {1,3,7,15,31,63,127,255};	/*Values for masking bits*/
unsigned char Flip[8] = {1,2,4,8,16,32,64,128};		/*Values for mutation flips*/

long int Timer;								/*Simulation Timer*/

int Current;
int Last;
double TrackWidth, Half_TrackWidth;

  
extern Car* pcar[];


/********************************************************************
*																	*
*	Function: Setup - Environment Initialisation					*
*																	*
*	Parameters:	None												*
*																	*
*	Returns: Nothing												*
*																	*
********************************************************************/

void Bug_Setup(void)
{
	long int i,k,ID;
	FILE *handle;
	int byte;
	int temp;

	if ((handle = fopen("bug.dat", "rb")) == NULL)
	{
		fprintf(stderr,"Cannot open genome input file.\n");
		exit(0);
	}

	fseek(handle,0L,SEEK_SET);
	

	/*Zero the net weights*/

	for(i=0;i<HONODES;i++)
	{
		for(k=0;k<NUMNODES;k++)
		{
			BugDriver.Weights[i][k] = 0.0;
		}
	}
	
	/*Read in Net data*/

	for(i=0;i<HONODES;i++)
	{
		byte = fgetc(handle);

		if(byte == EOF)
		{
			printf("filelength wrong!\n");
			exit(0);
		}
			
		temp = ((byte & masks[BiasAct-1])-127);
		BugDriver.Nodes[i] = temp * BSCALE;
			
		for(k=0;k<NUMNODES;k++)
		{
			byte = fgetc(handle);
				
			if((byte & Flip[IDACT]))
			{
				ID = byte & masks[IDACT-1];
			}
			else
			{
				ID = NUMNODES+1;
			}
				
			byte = fgetc(handle);
				
			if(ID<NUMNODES)
			{
				temp  = ((byte & masks[WeigAct-1])-127);
				BugDriver.Weights[i][ID] += temp * WSCALE;
			}
		}
	}
	
	fclose(handle);
}




/********************************************************************
*																	*
*	Function: Activate - Activation function for neurons			*
*																	*
*	Parameters:	Input value											*
*																	*
*	Returns: float Output											*
*																	*
********************************************************************/


double Activate( double net )
{
	return(  (2.0/(1.0+exp((-LAMBDA)*net) ) )-1.0  );
}
	

#define BUG_MAX(x,y)                (((x)<(y))?(y):(x))        // return maximum
#define BUG_MIN(x,y)                (((x)>(y))?(y):(x))        // return minimum

con_vec Bug(situation& s)
{
	const char name[] = "Bug";        // This is the robot driver's name
	con_vec result;                     // This is what is returned
	
	if( s.starting )
	{
    	my_name_is(name);                  // copy name over
		TrackWidth = s.to_lft + s.to_rgt;	//Just left in for amusement, how handy that
											//during the setup phase most of the environment
											//vars are uninitialised. Much time wasted finding
											//that out.
		result.fuel_amount = MAX_FUEL; 
		result.vc = 0.0f;   
		result.alpha = 0.0f;
		Bug_Setup();			//Load in the bug's brain
		return result;
	}
	
	Timer++; //Crank the global simulation timer

	DoBug( s );	//Crank the NN

	//stick values in con_vec
	result.vc = BugDriver.Status[Current][NUMNODES-2] * 300.0;
	result.alpha = BugDriver.Status[Current][NUMNODES-1] * 0.5 * PI;

		
	//Pitting code taken from Bulle.cpp

	if( s.fuel<6.0 ) 
	{
		int cpt = BUG_MIN( 20, s.laps_to_go );
		int damage_min = BUG_MAX( ((long)s.damage)-10000, 0 );
		
		result.request_pit   = 1;
		result.repair_amount = damage_min*cpt/20;
		result.fuel_amount = MAX_FUEL;
	}
	if( s.laps_to_go>1	  )
	{
		if( s.damage>28000 )
		{
			int cpt = BUG_MIN( 20, s.laps_to_go );
			
			result.request_pit   = 1;
			result.repair_amount = BUG_MAX( s.damage*cpt/20, 5000 );
			result.fuel_amount = MAX_FUEL;
		}
	}

	return result;
}


void LimitInput( double *Input )
{
	if( *Input > 1.0)
		*Input = 1.0;
	else
	{
		if( *Input < -1.0 )
			*Input = -1.0;
	}
}

//length and radius functions adapted from Robert Wilderspin's 
//NN input code

// This returns the length of a segment, measured along the centre
//
double length ( double len, double rad )
{
  if (rad != 0.0)
    len *= ( fabs(rad) + 0.5 * TrackWidth );

  return len;
}


// This returns the radius of a segment (6000 ft for a straight)
//
double radius ( double rad )
{
  if (rad == 0.0) 
	  rad = 2000.0;

  return rad;
}


	
void DoBug( situation& s )
{

	int j;
	
	TrackWidth = s.to_lft + s.to_rgt; //Arsing arse!
	Half_TrackWidth = TrackWidth * 0.5; 


	Current = Timer & 0x0001; //Set correct net state order
	Last = !Current;

	//Place the values in the inputs

	//The inputs need some serious thought applied and I'm afraid I was
	//a little lazy about doing so.

	//Definitely need to supply info in the nearest 2 cars, perhaps
	//differentiating inputs for nearest car in front and nearest behind.

	//Inputs 0-10 need to be examined for scaling, generality across tracks,
	//and extremas.

	BugDriver.Inputs[0] = length( s.to_end, s.cur_rad ) * 0.0005;
	BugDriver.Inputs[1] = radius( s.cur_rad ) * 0.0005;
	BugDriver.Inputs[2] = (s.to_lft-Half_TrackWidth)/Half_TrackWidth;
	BugDriver.Inputs[3] = (s.v*0.01) - 1.0;
	BugDriver.Inputs[4] = (s.vn*0.01) - 1.0;
	BugDriver.Inputs[5] = length( s.nex_len, s.nex_rad ) * 0.0005;
	BugDriver.Inputs[6] = radius( s.nex_rad ) * 0.0005;
	BugDriver.Inputs[7] = length( s.after_len, s.after_rad ) * 0.0005;
	BugDriver.Inputs[8] = radius( s.after_rad ) * 0.0005;
	BugDriver.Inputs[9] = length( s.aftaft_len, s.aftaft_rad ) * 0.0005;
	BugDriver.Inputs[10] = radius( s.aftaft_rad ) * 0.0005;
	
	if( s.nearby[0].who != 999 )
	{
		BugDriver.Inputs[11] = s.nearby[0].rel_x*0.001;
		BugDriver.Inputs[12] = s.nearby[0].rel_y*0.001;
		BugDriver.Inputs[13] = s.nearby[0].rel_xdot*0.005;
		BugDriver.Inputs[14] = s.nearby[0].rel_ydot*0.005;
	}
	else
	{
		BugDriver.Inputs[11] = -1.0;
		BugDriver.Inputs[12] = -1.0;
		BugDriver.Inputs[13] = -1.0;
		BugDriver.Inputs[14] = -1.0;
	}
	
	for( j=0 ; j<INPUTS ; j++)
		LimitInput( &BugDriver.Inputs[j] );

	
	for(j=0; j<NUMNODES; j++)
	{
		DoNode(&BugDriver,j);
	}
	
	/*Reset last timer tick values*/

	for(j=0; j<NUMNODES; j++)
	{
		BugDriver.Status[Last][j] = 0.0;
	}
}



/********************************************************************
*																	*
*	Function: DoNode - Calc overall input value, fire node			*
*																	*
*	Parameters:	Animat and node indices								*
*																	*
*	Returns: Nothing												*
*																	*
********************************************************************/


void DoNode( BUG *pBug, int NodeNo)
{
	int j;   /* Our Counter friends*/
	double totin = 0.0;
	
	if(NodeNo < INPUTS)
	{  /*if Input Node*/
		totin = pBug->Inputs[NodeNo];
	}
	else
	{
		for( j=0 ; j<NUMNODES ; j++ )
		{
			if( pBug->Status[Last][j] != 0.0 ) //if connected
			{
				totin += pBug->Status[Last][j] * pBug->Weights[NodeNo-INPUTS][j];
			}
		}
	}
	
	/*Search status for inputs to Node*/
	
	totin += pBug->Nodes[NodeNo]; /*Threshold*/
	
	pBug->Status[Current][NodeNo] = Activate(totin);
	
	/*Do Sum and put Output in next Status Line*/
}

