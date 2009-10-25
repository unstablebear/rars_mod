//----------------------------------------------------------------------------
//
// RARS Robot optimizing code
//
// Developed for RARS 0.72 with DJGPP/Allegro, but it should work on 0.74
//
//----------------------------------------------------------------------------
//
// Anybody can do whatever they want with this code, but if you use ideas
// I'd like you to send me an email.  This source code is public.
//
// Read the header for more infomation.
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


#include <stdlib.h>
#include <stdio.h>
#include <stream.h>
#include <values.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "optimize.h"
#include "car.h"
#include "track.h"


#define KEEP_OLD_FITNESS // Define if you want to keep the fitness from the loaded file


double gaussrand(void)
{
  static double V2, fac;
  static int phase = 0;
  double S, Z, U1, U2, V1;

  if (phase)
    Z = V2 * fac;
  else
  {
    do
    {
      U1 = (double)random() / MAXINT;
      U2 = (double)random() / MAXINT;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    }
    while(S >= 1);

    fac = sqrt (-2 * log(S) / S);
    Z = V1 * fac;
  }

  phase = 1 - phase;
  return Z;
}


optimizeindividual::optimizeindividual()
  :_size(1),
  _data(new double[1]),
  _fitness(0.)
{
  reset();
}


optimizeindividual::optimizeindividual(const unsigned int s)
  :_size(s),
  _data(new double[s]),
  _fitness(0.)
{
  reset();
}


optimizeindividual::~optimizeindividual()
{
  delete _data;
}


void optimizeindividual::operator=(const optimizeindividual& other)
{
  if(other._size!=_size)
  {
    delete _data;
    _data = new double[other._size];
    _size = other._size;
  }

  for(unsigned int i=0;i<_size;i++)
    _data[i] = other._data[i];

  _fitness = other._fitness;
}


double& optimizeindividual::operator[](const unsigned int index)
{
  assert(index<_size);
  return _data[index];
}


const double& optimizeindividual::operator[](const unsigned int index) const
{
  assert(index<_size);
  return _data[index];
}


double optimizeindividual::fitness(void) const
{
  return _fitness;
}


unsigned int optimizeindividual::size(void) const
{
  return _size;
}


void optimizeindividual::reset(void)
{
  for(unsigned int i=0;i<_size;i++)
    _data[i] = (double)random()/(double)MAXINT;
    
  _fitness = 0.;
}


istream& operator>>(istream& stream, optimizeindividual& dest)
{
  // Read the old fitness in
  dest._fitness = 0.;

#ifdef KEEP_OLD_FITNESS
  stream >> dest._fitness;
#else
  double temp = 0.;
  stream >> temp;
#endif

  // Read the chromosome in
  for(unsigned int i=0;i<dest.size();i++)
    stream >> dest[i];
  
  return stream;
}

ostream& operator<<(ostream& stream, const optimizeindividual& dest)
{
  stream << dest.fitness() << " ";

  for(unsigned int i=0;i<dest.size()-1;i++)
    stream << dest[i] << " ";
    
  stream << dest[dest.size()-1] << endl;
    
  return stream;
}


double optimizedata::crossover = 0.6;
double optimizedata::mutationrate = 0.05;
double optimizedata::mutationstrength = 0.05;


char optimizedata::filename[100];


optimizedata::optimizedata(void)
  :bestever(0.),
  index(-1),
  size(1),
  pop(new optimizeindividual[1]),
  trial(1),
  evaluate(NULL)
{
  strcpy(filename,"");
}


optimizedata::~optimizedata(void)
{
  write();
  cleanup();
}


void optimizedata::cleanup()
{
  bestever = 0.;
  index = -1;
  
  delete [] pop;
}


void optimizedata::write()
{
  ofstream stream(filename);

  // Save the genetic data
  for(unsigned int i=0;i<size;i++)
    stream << pop[i];
}


void optimizedata::load(const char* fname, const unsigned int s, const unsigned int len, void(*eval)(const double*,const unsigned int n))
{
  // Figure out the filename for the optimization file
  sprintf(filename,"%s.",fname);
  char* point = strchr(filename,'.');
  *point = '\0';
  strcat(filename,".opt");

  evaluate = eval;
  size = s;

  cleanup();
  pop = new optimizeindividual[s];

  trial = optimizeindividual(len);

  ifstream stream(filename);

  // Load and resize in the genetic data
  for(unsigned int i=0;i<size;i++)
  {
    // This forces the size to be correct
    // with random data
    pop[i] = trial;
    pop[i].reset();

    // Maybe read the data in from file
    if(stream.good())
      stream >> pop[i];
  }

  bestever = 0.;
  index = -1;
}


optimizeindividual& optimizedata::operator[](const unsigned int index)
{
  return pop[index];
}


const optimizeindividual& optimizedata::operator[](const unsigned int index) const
{
  return pop[index];
}


void optimizedata::step(double fitness)
{
  trial._fitness = fitness;

  // Check if we got reasonable data to compare
  if(index<0)
  {
    index = 0;
    trial = pop[index];
    evaluate(&trial[0],trial.size());
    return;
  }

  // Was the fitness better than what we were testing
  if(fitness>pop[index].fitness())
  {
    // Put the improved back into the pop
    pop[index] = trial;
    pop[index]._fitness = fitness;

    // Is it better than the best ever seen
    if(fitness>bestever)
    {
      // Inform that a new best has been reached
      cout << "!! New Best: "
           << fitness
           << endl;

      // Save the latest population
      write();

      bestever = fitness;
    }
  }
  
  // Pick a random mother and father
  int mother = random()%size;
  int father = random()%size;

  if(pop[mother].fitness()==0.)
  {
    index = mother;
    trial = pop[index];
    evaluate(&trial[0],trial.size());
    return;
  }
  if(pop[father].fitness()==0.)
  {
    index = father;
    trial = pop[index];
    evaluate(&trial[0],trial.size());
    return;
  }
  
  // Choose the one with the lowest fitness to possibly replace
  if(pop[mother].fitness()>pop[father].fitness())
  {
    int temp = father;
    father = mother;
    mother = temp;
  }

  // Find the lowest fitness one to replace
  double lowest = MAXDOUBLE;
  unsigned int j;
  for(j=0;j<size;j++)
  {
    const double fit = pop[j].fitness();
    if(fit<lowest)
    {
      index = j;
      lowest = fit;
    }
  }

  // Breed the mother and father
  for(j=0;j<pop[mother].size();j++)
  {
    double r = (double)random()/(double)MAXINT;
    
    if(r<crossover)
      trial[j] = pop[father][j];
    else
      trial[j] = pop[mother][j];

    // Mutate the mother and father
    r = (double)random()/(double)MAXINT;
    
    if(r<mutationrate)
      trial[j] += gaussrand()*mutationstrength;
  }

  // Try out the new data set
  evaluate(&trial[0],trial.size());
}


double optimizedata::mph(double time, double length, double damage, double distance)
{
  // Find the approximate mph
  double mph = length/5280./(time+0.01)*3600.;

  // if we were damaged and didn't finish, return a
  // little speed related to distance around the track
  if(damage>MAX_DAMAGE)
    return distance/length;

  // Adjust the mph by the damage
  return mph - mph*damage/MAX_DAMAGE + 0.01;
}

