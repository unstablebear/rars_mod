/**
 * Mafanja - Robot driver for RARS v0.90
 * 
 * Filename:  mafanja.cpp
 * Robot:     Mafanja
 * Races:     all
 * Source:    Public
 * Color:     Red/Green
 * Data:      no need for a data file.
 *
 * Date:			27.02.2002
 * Version:		7
 *
 * Author:    Eugen Treise / Germany <eugentreise@uni.de>
 *
 */

#include <stdio.h>	// sprintf(),fopen(),fclose(),fprintf()
#include <math.h>		// sqrt()
#include "car.h"
#include "track.h"	// track_desc, get_track_description()
#include "os.h"			// delta_time

#define uint32 unsigned long

#ifndef max
#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))
#endif // max

const double MaxSpeed=250.0;
const double leftSide=0;
const double SecDistance=CARWID;
const int MaxSegmentCount=100;

class Mafanja : public Driver
{
	
private:

	//class for handle pitstops for a driver
	class PitStopET
	{
	private:
		int lastPitLap;						// number of the lap with last pit
    int LapsToGo;             // remaining laps to finish the race
		double lastPitFuel;				// amount of fuel after last pit
		double FuelPerLap;				// fuel per lap
    double FuelBeforePit;       // fuel in last lap
		uint32 lastRepairAmount;	// amount of repair done in last pit
		uint32 DamagePerLap;			// average damage per lap
		uint32 DamageLastLapStart;// damage in beginning of last lap
		uint32 DamageLastLap;			// damage taken during last lap
		uint32 DamageCurLap;			// damage taken during current lap
		bool requesting;					// shows if a pitstop is in request
    int PitStopsDone;         // number of done pitstops

	public:

		PitStopET()
		{
		}

		// init before using other methods
		void initSituation(situation& s, con_vec& resultBox, const track_desc& TrackDesc)
		{
			lastPitLap = PitStopsDone = 0;
			FuelPerLap = 0.0;
			// track length in miles * lap number * liter fuel per mile
			lastPitFuel = min(TrackDesc.length/5280.0*s.laps_to_go*1.5, MAX_FUEL);
			// set s.fuel here to calculate with the right value (s.fuel=MAX_FUEL while starting)
			resultBox.fuel_amount = s.fuel = FuelBeforePit = lastPitFuel;
			lastRepairAmount = DamagePerLap = DamageLastLap = DamageCurLap = 0;
			DamageLastLapStart = s.damage;
			requesting = false;
			resultBox.request_pit = 0;
		}

		// execute once per lap on finish line
		void calculateLastLap(const situation& s)
		{
			//get laps number to go
			LapsToGo=s.laps_to_go+min(0,(int)s.behind_leader);

			if(s.laps_done > lastPitLap)
			{
				FuelPerLap = (lastPitFuel-s.fuel) / (s.laps_done-lastPitLap);
				DamageLastLap = s.damage-DamageLastLapStart;
        DamageCurLap = 0;
				DamagePerLap = (DamagePerLap*(s.laps_done-1) + DamageLastLap) / s.laps_done;
        DamageLastLapStart = s.damage;
			}
		}

		// do we need a pitstop?
		bool needPitStop(const situation& s, con_vec& resultBox)
		{
      DamageCurLap = s.damage-DamageLastLapStart;
 			if( LapsToGo>1 &&                 //  not on last lap
        //pcar[s.my_ID]->Pit_stops == PitStopsDone &&     // if false, the values for fuel and damage are already taken by the Car->control() function
        (requesting ||                  //  already requesting
        s.damage>MAX_DAMAGE*0.7 ||     //  damage is to high
        (s.damage+max(DamagePerLap,max(DamageLastLap,DamageCurLap)))>=MAX_DAMAGE ||   //  average damage or damage in last or current lap was to high
        s.fuel<FuelPerLap*1.1))        //  damage in last lap was to high
        return true;
      
      // set request_pit to 0 to avoid not requested pitstops (happens if driving a race alone)
      resultBox.request_pit = 0;
			return false;
		}

		// request a pitstop with fuel and damage calculations
		void makePitStop(const situation& s,con_vec& resultBox)
		{
			requesting=true;

      double FuelToFill;	//amount of fuel to fill in
			  
			resultBox.request_pit = 1;

			//pit
      FuelBeforePit = s.fuel;
			FuelToFill = min(FuelPerLap*1.05*(LapsToGo+1),MAX_FUEL) - s.fuel;
			if (FuelToFill < 0)
				FuelToFill = 0;
			resultBox.fuel_amount = FuelToFill;

			//repair
			//all damage
			resultBox.repair_amount = min(s.damage,		
				//damage for the rest of race
				max(max((uint32)(DamagePerLap*LapsToGo*1.1),
				//repair more for faster lapspeed	and longer distance
				(uint32)(s.damage*0.8+s.damage*0.2*(2*s.bestlap_speed/MaxSpeed)*(LapsToGo/(LapsToGo+s.laps_done)))),
				//use pit-time for repair (0.05*FuelToFill/0.005)
				(uint32)(10*FuelToFill)));
		}

		// execute when pitstop is done
		bool PitStopDone(const situation& s)
		{
      if( requesting )
      {
        // if is comming out of pits or detect a pitstop by looking on fuel change
			  if( s.out_pits==1 || FuelBeforePit < s.fuel)
			  {
				  DamageLastLapStart=s.damage;
				  lastPitLap=s.laps_done;
				  lastPitFuel=FuelBeforePit=s.fuel;
				  requesting=false;

			    //lap_flag for a new lap was not set in the box
			    //get laps number to go
			    LapsToGo=s.laps_to_go+min(0,(int)s.behind_leader);
          PitStopsDone++;
          return true;
			  }
      }
      return false;
		}

    int getDonePitStops()
    {
      return PitStopsDone;
    }
		
	};

  // static member declaration
  static int DriverNr;

	double FrictionFactor;
	double aBrake,aAccel;	//braking-acceleration, acceleration
  double aBrakeMin;     // min value for braking-acceleration
	double TrackWidth,HalfTrackWidth;	//width of the track
	int curSegNr, nextSegNr, lastSegNr, anextSegNr;	//current segment number
	double v0,v1,v2;		//velocity in the segments
	double s1;	//brake distance
	double Line,lastLine,nextLine,curLine1,curLine2;	//driving line
	double lastRad,curRad, nexRad;
	double toEnd,toEndReal;
	double lastLen,curLen,nexLen;
  double lastvc;    // saves vc for next function call
	double lastTurnPoint,TurnPoint1,TurnPoint2,nextTurnPoint;
	char info[1000];
	PitStopET MafPitStop;
	double mass;				// weight of the car
  bool adjustSpeed;
  double *maxCurveSpeed;	// max possible speed in a segment
	double *realCurveSpeed;	// max reached speed in a segment
  double *outInSegment;		// feet out of track in a segment
  double *maxLatG;				// max lateral acceleration in a segment
  double *aBrakeMax;      // max brake acceleration in this segment
  track_desc TrackDesc;
  int SegmentCount;     // number of track segments
	double maxOffRoad;
	double lastToLeft;		// value of s.to_lft last time
  double curvature;
  bool braking;
  double BrakeRatio, lastVelocity;
  uint32 lastTimeDamage;    // to detect a collision

	int sgn(double zahl)
	{
		return zahl<0.0 ? -1 : 1;
	}

	double getMaxRadius(double rad,double len,double usedTrackWidth)
	{
		double f;
		f=usedTrackWidth/(1-cos(len/2));
		return rad+f;		
	}

	
	double getTurnPoint(double len,double usedTrackWidth)
	{
		double f;
		f=usedTrackWidth/(1-cos(len/2));
		return f*sin(len/2);		
	}

	double getCurveSpeed(double rad,double len,double usedTrackWidth)
	{
		if(rad==0.0)
			//straight
			return MaxSpeed;
		else
		{
			double speed;
			if (fabs(rad)>500)
				speed=sqrt((fabs(rad)) * g * FrictionFactor);
			else
				speed=sqrt((fabs(rad)+usedTrackWidth) * g * FrictionFactor);
			if(len<1.6)
				speed*=1.1;
			if(len<1)
				speed*=1.1;
			if(speed>MaxSpeed)
				speed=MaxSpeed;
			return speed;
		}
	}

	double getOneCurveSpeed(double rad,double len,double usedTrackWidth)
	{
		if(rad==0.0)
			//straight
			return MaxSpeed;
		else
		{
			double speed;
			if (fabs(rad)>500)
				speed=sqrt((fabs(rad)) * g * FrictionFactor);
			else
				speed=sqrt((fabs(rad)+usedTrackWidth) * g * FrictionFactor);
			if(len<1.6)
				speed*=1.1;
			if(len<1)
				speed*=1.1;
			if(speed>MaxSpeed)
				speed=MaxSpeed;
			speed = (speed+sqrt(getMaxRadius(fabs(rad),len,usedTrackWidth) * g * FrictionFactor))/2;
      //speed = sqrt(getMaxRadius(fabs(rad),len,max(0,usedTrackWidth-CARWID)) * g * FrictionFactor);
			return min(speed, MaxSpeed);
		}
	}

  //calculates max steering radius for a given speed
  double getMaxSteerRadius(double speed)
  {
    return speed*speed/(g * FrictionFactor);
  }

	//calculates distance for braking from velocity vStart to vEnd
	double getBrakeDist(double vStart, double vEnd, double aBrk)
	{
		double dv;
		dv=vStart-vEnd;
		if(vStart<vEnd)
			return 0;
		else
      // FIXME: the formula should be: s = (v0²-v²)/(2*a)
			return (vStart*vStart-vEnd*vEnd)/aBrk;
	}

	//calculates a startvelocity for a known brakedistance
	double getvStart(double vEnd, double sBrake)
	{
		return sqrt(sBrake*aBrake+vEnd*vEnd);
	}
	
	//compute end velocity with a const. acceleration
	double getvEnd(double v,double s,double a)
	{
		return sqrt(v*v+2*a*s);
	}

	double getTrackSideSimple(double rad)
	{
		double TrackSide;

		if(rad>0)		//left
			TrackSide=leftSide;
		else
		{
			if(rad<0)		//right
				TrackSide=(TrackWidth-leftSide);
			else	//straight
				TrackSide=HalfTrackWidth;
		}
		return TrackSide;
	}



// Problem: letzte Varibale wird mit einem falschen Wert dargetellt (uint32)
	void dumpStr(char *dumpstr) 
	{
		FILE *fp = fopen("mafanja.txt","at");
		if (fp) 
		{
			fprintf(fp, "%s\n", dumpstr);
			fclose(fp);
		}
	}

	double getSegmLen(double rad, double len)
	{
		double SegmLen;

		if(rad==0)
			SegmLen=len;
		else
			SegmLen=len*(fabs(rad)+TrackWidth*0.5);
		return SegmLen;
	}

public:

  Mafanja::Mafanja()
  {
    char DriverName[32];
    // a counter for drivers in my team
    DriverNr++;

    // necessary variables for the driver
    // give a name with a new number for each driver
    if(DriverNr == 1)
      m_sName = "Mafanja";
    else
    {
      sprintf(DriverName,"Mafanja%i",DriverNr);
      strcpy( m_sName2, DriverName );
      m_sName = m_sName2;
    }
    m_sAuthor = "Eugen Treise";
		m_sDescription = "Mafanja v7 27.02.2002 by ET; for RARS v0.90";
    m_iNoseColor = oGREEN;	//helm color
    m_iTailColor = oRED;	//car color
    m_sBitmapName2D = "car_green_red";
		m_sModel3D = NULL;	// model in 3D view

    // other variables
    adjustSpeed = false;      // make sure it isn't true to avoid problems in destructor
  }

	// Destructor for the robot
	Mafanja::~Mafanja()
	{
		if (adjustSpeed)
		{
			delete [] maxCurveSpeed;
			delete [] realCurveSpeed;
			delete [] outInSegment;
			delete [] maxLatG;
      delete [] aBrakeMax;
		}
	}


  con_vec drive(situation& s)
  {
	  double rad;			//radius of the current driving line

    con_vec result;                     

    if( s.starting )
    {
			FrictionFactor = MYU_MAX1*0.9;
			aBrakeMin = aBrake = MYU_MAX1*g;
			aAccel=16;

      TrackDesc = get_track_description();
      SegmentCount=TrackDesc.NSEG;
      if(SegmentCount<MaxSegmentCount)
      {
        adjustSpeed=true;
        maxCurveSpeed=new double [SegmentCount];
				realCurveSpeed=new double [SegmentCount];
        outInSegment=new double [SegmentCount];
        maxLatG=new double [SegmentCount];
        aBrakeMax = new double [SegmentCount];
				// set the new array values
				for (int i=0;i<SegmentCount;i++)
				{
					outInSegment[i]=0.0;
					maxLatG[i]=0.0;
					realCurveSpeed[i]=0.0;
          aBrakeMax[i] = aBrakeMin;
				}
      }

			TrackWidth = s.to_lft+s.to_rgt;
			HalfTrackWidth = TrackWidth/2;

			//segment halfs
			curLen=nexLen=getSegmLen(s.cur_rad,s.cur_len);

			curRad=0;

			curSegNr = s.seg_ID-1;
			if(curSegNr < 0)
				curSegNr = SegmentCount-1;
      lastSegNr = curSegNr;
			
			//for lastLine
			curLine2=s.to_lft;
			//for curLine1
			nextLine=s.to_lft;

			lastToLeft=s.to_lft;

			//set Turnpoints
			TurnPoint2=nexLen/3;
			nextTurnPoint=curLen/3*2;

			MafPitStop.initSituation(s, result, TrackDesc);

      lastTimeDamage = 0;

      braking=false;
      BrakeRatio = 0.0;
    }


    if(adjustSpeed)
    {

      // check if off road and store
      // don't have s.in_pits, so look at On_pit_lane to see if going in pits
      // On_pit_lane is not (always) set when car is going to pits, so look at the Pit_stops - counter for maked pitstops
      // if vc was changed, car is going to pit
      if (s.out_pits!=1 && (s.to_lft<0.0 || s.to_rgt<0.0) && lastvc == s.vc)//&& pcar[s.my_ID]->Pit_stops == MafPitStop.getDonePitStops())//On_pit_lane != 1)
      {
				double temp=min(s.to_lft,s.to_rgt);
        int SegNr;
        if(s.seg_ID == curSegNr)
          SegNr = curSegNr;
        else
          SegNr = nextSegNr;
				if(outInSegment[SegNr]>temp)
	        outInSegment[SegNr]=temp;

      }

    }


		//driving out of pits
		if(MafPitStop.PitStopDone(s))
		{
		}


		//only once for a segment
		if(curSegNr!=s.seg_ID)
    {
			lastSegNr=curSegNr;
			curSegNr=s.seg_ID;
      nextSegNr=(curSegNr+1)%SegmentCount;
			anextSegNr=(curSegNr+2)%SegmentCount;

			//segment length
			lastLen=curLen;
			curLen=nexLen;
			nexLen=getSegmLen(s.nex_rad,s.nex_len);

			lastRad=curRad;
			curRad=s.cur_rad;

			//Turnpoints
			lastTurnPoint=TurnPoint2;
			TurnPoint1=nextTurnPoint;
			if(s.cur_rad!=0)
			{
				TurnPoint2=curLen/2;
			}
			else
			{
				TurnPoint2=getTurnPoint(s.nex_len,TrackWidth);
				if(TurnPoint2>curLen/2)
					TurnPoint2=curLen/2;
			}
			if(s.nex_rad!=0)
				nextTurnPoint=nexLen/2;
			else
			{
				nextTurnPoint=nexLen-TurnPoint2;
				if(nextTurnPoint<nexLen/2)
					nextTurnPoint=nexLen/2;
			}
			
			//driving lines
			lastLine=curLine2;
			curLine1=nextLine;
			if(s.nex_rad==0)
			{
				if(curLine1<HalfTrackWidth)
					nextLine=TrackWidth-SecDistance-curLine1;
				else
					nextLine=TrackWidth+SecDistance-curLine1;

				double angle;
				angle=atan((nextLine-curLine1)/(TurnPoint2+(nexLen-nextTurnPoint)));
				if (fabs(angle)>0.1)
					nextLine=curLine1+sgn(angle)*tan(0.1)*(TurnPoint2+(nexLen-nextTurnPoint));

			}
			else
				nextLine=getTrackSideSimple(s.nex_rad);
			if(s.cur_rad==0)
			{
				
				curLine2=TrackWidth-nextLine;
				double angle;
				angle=atan((curLine2-curLine1)/(TurnPoint1-TurnPoint2));
				if (fabs(angle)>0.1)
					curLine2=curLine1+sgn(angle)*tan(0.1)*(TurnPoint1-TurnPoint2);
			}
			else
				curLine2=getTrackSideSimple(s.cur_rad);

      if (s.laps_done<1 || !adjustSpeed)
      {
			  if(s.nex_rad==0.0 && s.nex_len>TrackWidth*2)
				  v0=getOneCurveSpeed(s.cur_rad,s.cur_len,(fabs(lastLine-curLine1)+fabs(curLine2-nextLine))/2);
			  else
				  v0=getCurveSpeed(s.cur_rad,s.cur_len,(fabs(lastLine-curLine1)+fabs(curLine2-nextLine))/2);
			  if(s.after_rad==0.0 && s.after_len>TrackWidth*2)
				  v1=getOneCurveSpeed(s.nex_rad,s.nex_len,fabs(curLine2-nextLine));
			  else
				  v1=getCurveSpeed(s.nex_rad,s.nex_len,fabs(curLine2-nextLine));
        if(adjustSpeed)
        {
           maxCurveSpeed[curSegNr]=v0;
           maxCurveSpeed[nextSegNr]=v1;
        }
      }
      else
      {
        v0=maxCurveSpeed[curSegNr];

				// adjust speed for the next segment
				if(outInSegment[anextSegNr]<0.0)
				{
					// Is brake distance longer then segment length?
					if(getBrakeDist(realCurveSpeed[nextSegNr],maxCurveSpeed[anextSegNr],aBrakeMax[nextSegNr])>nexLen)
						//set new segment speed
						maxCurveSpeed[nextSegNr] = getvStart(maxCurveSpeed[anextSegNr],nexLen);
          else
            // speed to high for this segment
            maxCurveSpeed[nextSegNr] = 0.95*min(maxCurveSpeed[nextSegNr],realCurveSpeed[nextSegNr]);
				}

        // drive slower if lateral g-force was to high in next segment
        // dont works now
        /*if (maxLatG[nextSegNr]>0.99999*MYU_MAX1*g)
          maxCurveSpeed[nextSegNr]=0.98*min(maxCurveSpeed[nextSegNr],realCurveSpeed[nextSegNr]);
        // drive faster if lateral g-force didnt reached a certain value
        else*/ if((maxCurveSpeed[nextSegNr]<MaxSpeed) && (maxLatG[nextSegNr]<0.97*MYU_MAX1*g) && (realCurveSpeed[nextSegNr]*1.03>maxCurveSpeed[nextSegNr]) && (outInSegment[nextSegNr]=0.0))
          maxCurveSpeed[nextSegNr]=1.02*max(maxCurveSpeed[nextSegNr],realCurveSpeed[nextSegNr]);
        maxLatG[curSegNr]=0.0;
				realCurveSpeed[nextSegNr]=0.0;

        v1=maxCurveSpeed[nextSegNr];
        aBrake = aBrakeMax[curSegNr];
      }
      
      braking = false;

		}

    if (adjustSpeed)
		{
			// store the highest segment speed
			if(s.v>realCurveSpeed[curSegNr])
				realCurveSpeed[curSegNr]=s.v;

      // get the max lateral g-force for a segment
      if(s.cur_rad != 0.0)
      {
        if(fabs(s.cen_a) > maxLatG[curSegNr])
          maxLatG[curSegNr]=fabs(s.cen_a);
      }
      else
      {
        if(s.to_end > curLen/2)   //first half of segment
        {
          if(fabs(s.cen_a) > maxLatG[lastSegNr])
            maxLatG[lastSegNr]=fabs(s.cen_a);
        }
        else      //second half of segment
        {
          if(fabs(s.cen_a) > maxLatG[nextSegNr])
            maxLatG[nextSegNr]=fabs(s.cen_a);
        }

      }
    }

    // use an internal function to get back on track when stuck
    if( s.out_pits!=1 && stuck( s.backward, s.v, s.vn, s.to_lft, s.to_rgt, &result.alpha, &result.vc ) )
    {
      return result;
    }


		if(s.cur_rad!=0)
		{
			if(s.cur_rad>0)
				rad=s.cur_rad+s.to_lft;
			else
				rad=s.cur_rad-s.to_rgt;
		}
		else
			rad=0.0;

		toEnd=getSegmLen(s.cur_rad,s.to_end);
    // FIXME: substruct TrackWidth*0.5 from rad;   what does +20 here?
		toEndReal=getSegmLen(rad,s.to_end)+20;
		mass = M + s.fuel/g;
		s1=getBrakeDist(s.v,v1,aBrake + (DRAG_CON * s.v * s.v * (2*s.damage+MAX_DAMAGE)/MAX_DAMAGE)/mass);


		//once per lap
		if(s.lap_flag ==1)
		{
			MafPitStop.calculateLastLap(s);
		}

		if( MafPitStop.needPitStop(s, result) )
			MafPitStop.makePitStop(s, result);


		if(toEndReal<s1)
		{
      
			if(adjustSpeed)
      {
				if(outInSegment[nextSegNr]<0.0)
				{
					// Is brake distance longer then segment length?
					if(s1>curLen)
						//set new segment speed calculated with current values
						maxCurveSpeed[curSegNr]=getvStart(v1,curLen);
				}
      }


			//braking
			if(s.v>1.03*v1)
      {
				braking = true;
        result.vc=s.v*0.90;
      }
			else 
      {
        if(s.v<v1)
				  result.vc=v1;
			  else
				  result.vc=(s.v+v1)/2;
				braking = false;
      }
		}
		else
		{
      if(adjustSpeed)
      {
        // if we brake harder then calculated adjust the acceleration faktor
        if(braking && toEndReal > (s1+10))
        {
          braking = false;
          // if there was no collision
          if(lastTimeDamage == s.damage)
          {
            aBrakeMax[curSegNr] += 0.1*(toEndReal-s1);
            aBrake = aBrakeMax[curSegNr];
          }
        }
      }      
			//straight
			if(s.cur_rad==0)
				//full throttle
				result.vc=s.v+50;
			else
			//curve
			{
				if(toEndReal<curLen*0.2)
				{
					if(s.v<v1*0.95)
						result.vc=s.v+50;
					else if(s.v<v1)
						result.vc=v1;
					else
						result.vc=(s.v+v1)/2;
				}
				else
				{
					if(s.v>1.1*v0)        // enter speed was to high
          {
            if(adjustSpeed)
            {
              aBrakeMax[lastSegNr] -= 0.1;
              // don't go below a good value (or in minus)
              if( aBrakeMax[lastSegNr] < aBrakeMin )
                aBrakeMax[lastSegNr] = aBrakeMin;
            }
						result.vc=v0;//s.v*0.90;
          }
					else if(s.v>1.02*v0)
						result.vc=s.v*0.99;
					else if(s.v<v0*0.95)
						result.vc=s.v+50;
					else if(s.v<v0)
						result.vc=v0;
					else
						result.vc=(s.v+v0)/2;
				}
			}
		}

    if(adjustSpeed)
    {
			if(outInSegment[nextSegNr]<0.0)
			{
        // set outInSegment to 0
				outInSegment[nextSegNr]=0.0;
			}
    }

		//for better steering
		toEnd-=100;
		//toEnd-=delta_time*max(s.v, result.vc);
		//toEnd-=delta_time*s.v;
		if(toEnd<-(nexLen-nextTurnPoint))
			toEnd=-(nexLen-nextTurnPoint);

		//driving line
		double Faktor;
		//four TurnPoints:
		if(toEnd>TurnPoint1)		//part 1 of the  current segment
		{
			Faktor=(toEnd-TurnPoint1)/(curLen-TurnPoint1+lastTurnPoint);
			Line=lastLine*Faktor + curLine1*(1-Faktor);
		}
		else if(toEnd>TurnPoint2)		//part 2 of the  current segment
		{
			Faktor=(toEnd-TurnPoint2)/(TurnPoint1-TurnPoint2);
			Line=curLine1*Faktor + curLine2*(1-Faktor);
		} 
		else			//last part of the  current segment
		{
			Faktor=(toEnd+nexLen-nextTurnPoint)/(TurnPoint2+nexLen-nextTurnPoint);
			Line=curLine2*Faktor + nextLine*(1-Faktor);
		}


		// check for nearby cars
    for (int i = 0; i < NEARBY_CARS; i++)
    {
      if(s.nearby[i].who != 999)
      {
				//brake if a car is comming to close and to fast in front
				if(s.nearby[i].rel_y<s.v*0.5 && fabs(s.nearby[i].rel_x)<CARWID && s.nearby[i].rel_ydot<-CARLEN)
        {
          /*if(s.cur_rad == 0)
            Line=Line-sgn(Line-HalfTrackWidth)*CARWID*1.5+s.nearby[i].rel_x;
          else*/
					  result.vc=0.0;
        }
			}
    }


    if(s.cur_rad != 0)
      curvature = s.cur_len/((s.cur_len*s.cur_rad)/s.v/delta_time);
    else
      curvature = 0.0;

		//steering
		//result.alpha = .008 * (s.to_lft-Line) - s.vn/s.v + curvature;
    result.alpha = atan2((s.to_lft-Line),100)-atan2(s.vn,s.v)+curvature;

    lastvc = result.vc;
    lastToLeft=s.to_lft;
    lastTimeDamage = s.damage;

		return result;
	}
};

// static member definition, to allow several drivers with this class
int Mafanja::DriverNr;


Driver * getMafanjaInstance()
{ 
  return new Mafanja();
}

