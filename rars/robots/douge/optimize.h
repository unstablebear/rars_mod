//----------------------------------------------------------------------------
//
// RARS Robot optimizing code
//
// Developed for RARS 0.72 with DJGPP/Allegro, but it should work on 0.74
//
//----------------------------------------------------------------------------
//
// Anybody can do whatever they want with this code, but if you use ideas
// I'd like you to send me an email.  This source code is public.  I
// developed it using DJGPP/Allegro but the source is hopefully fairly
// portable.  If not, mail me and I'll try to fix it.
//
// This file is the header for a genetic optimizer for RARS cars.  You use
// it by declaring a optimizedata object and loading a population from a
// file.  If the file does not exist, it will be made with random data.
// Of course you must also link to the compiled version of OPTIMIZE.CPP.
//
// The load function also needs a population number, genetic length (number
// of parameters to optimize) and a pointer to a function that tries out a
// parameter set.  Probably the best way to figure out how to do this is
// to look at the FELIX13.CPP file and look for the OPTIMIZE defines. Then
// you run rars with the command line:
//
// rars <track> 1 2 -f -k -r10000 -nd
//
// Then a file will be made (when rars exits) called <track>.OPT containing
// the parameter sets and the MPH of the parameter set.  There is a nice
// helper function optiizedata::mph that helps calculate a useful fitness
// for optimizing, which takes into damage into consideration and also if
// a car doesn't make it it the whole way around the track.
//
// If you have any questions just jive me a mail and we can talk...
//
// Happy racing,
//
// Doug Eleveld <deleveld@dds.nl>
//              <deleveld@my-dejanews.com>
//              <delev@anest.azg.nl>
//
// Version Info:
//
// 1.0 Original version
//     August 17, 1999
//
//----------------------------------------------------------------------------


#include <stream.h>


double gaussrand(void);


class optimizeindividual
{
  public:
   optimizeindividual();
   optimizeindividual(const unsigned int size);
   ~optimizeindividual();

   void operator=(const optimizeindividual&);
   
   double& operator[](const unsigned int index);
   const double& operator[](const unsigned int index) const;

   double fitness() const;

   unsigned int size() const;

   void reset();

  private:
   unsigned int _size;
   double* _data;
   double _fitness;

  friend class optimizedata;

  friend istream& operator>>(istream& stream, optimizeindividual& dest);
  
  private:
   optimizeindividual(const optimizeindividual&);
};


class optimizedata
{
  public:
   optimizedata();
   ~optimizedata();

   void load(const char* fname, const unsigned int s, const unsigned int len, void(*eval)(const double*,const unsigned int n));
   void write();
   
   optimizeindividual& operator[](const unsigned int index);
   const optimizeindividual& operator[](const unsigned int index) const;

   void step(double mph);

   // Static helper function for mph calculations
   static double mph(double time, double length, double damage, double distance);

   static double crossover;
   static double mutationrate;
   static double mutationstrength;
  
  private:
   double bestever;

   int index;
   
   unsigned int size;
   optimizeindividual* pop;
   optimizeindividual  trial;
   
   static char filename[100];

   void(*evaluate)(const double*, const unsigned int n);

   void cleanup();

  private: // and unimplemented
   optimizedata(const optimizedata&);
   void operator=(const optimizedata&);
};

