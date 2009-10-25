/* 
   Brute-Force optimizer for Djoefe
   by Gian-Carlo Pascutto
*/

/* 	$Id: bfpar.c,v 1.1 2001/03/27 18:52:53 timfoden Exp $	 */

/*
 *      $Log: bfpar.c,v $
 *      Revision 1.1  2001/03/27 18:52:53  timfoden
 *      Added version 0.82 files into new module in SourceForge repository
 *
 *      Revision 1.1.1.1  2000/09/16 20:27:06  mgueury
 *      First release on sourceforge
 *
 *      Revision 1.3  1999/04/08 23:00:37  giancarlo
 *      Ouputs a file with best speeds & corresponding parameters
 *      for statistical data gathering.
 *
 *      Revision 1.2  1999/04/08 22:10:02  giancarlo
 *      Parses output.
 *      Remembers lap records.
 *    
*/


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "pvm3.h"

int my_system (const char *command) 
{
  int pid, status;

  pid = fork();
  
  if (pid == -1)
    return -1;
  
  if (pid == 0) 
  {
    char *argv[4];
    argv[0] = "sh";
    argv[1] = "-c";
    argv[2] = command;
    argv[3] = 0;
    execve("/bin/sh", argv, environ);
    exit(127);
  }
  
  do 
  {
    if (waitpid(pid, &status, 0) == -1) 
    {
      if (errno != EINTR)
	return -1;
    } 
    else
    {  
	return status;
    }
  } while(1);       

}


void test_parameters(double p1, 
		     double p2, 
		     double p3 ,
		     double p4, 
		     double p5, 
		     double p6, 
		     double p7, 
		     int p8, 
		     int p9)
{
  FILE *bfout;
  FILE *racerep;
  FILE *stats;
  
  char dummy[80], dummyc;
  int dummya, dummyb, dummyd, dummye;
  int damage;
  int i;
  
  double avg_speed, best_speed;
  double dg = 0.8;
 
  static double best_best_speed = 0;
 
  bfout = fopen("bfopt.dat", "w");
  
  fprintf(bfout, "%lf %lf %lf %lf %lf %lf %lf %lf %d %d\n",
	  p1, p2, p3, dg, p4, p5, p6, p7, p8, p9);

  fclose(bfout);

  my_system("rars 1 -f -nR -nd 2 hungary.trk");

  racerep = fopen("hungary.out", "r");

  for(i = 0; i < 12; i++)
    fgets(dummy, 80, racerep);      // skip 11 lines

  if(fscanf(racerep, "%d %d %s %d %d %lf %lf %d",
	 &dummya, &dummyb, &dummyc, &dummyd, &dummye, 
	 &avg_speed, &best_speed, &damage) != 8) 
    {
      printf("Error parsing RARS output.\n");
      abort();
    };

  fclose(racerep);

  printf("%3.2lf %3.2lf %5d", avg_speed, best_speed, damage);

  if (damage == 0)
  {
    stats = fopen("stats.dat", "a");

    fprintf(stats, "%3.2lf %f %f %f %f %f %f %f %f\n",
	    best_speed, p1, p2, p3, dg, p4, p5, p6, p7);

    fclose(stats);    

    if (best_speed > best_best_speed)   
    {
      printf("! New lap record."); 
      best_best_speed = best_speed;
  
      my_system("cp bfopt.dat djoefe.dat");
    }
  }

  putchar('\n');
}

int main(void)
{
  double p1, p2, p3, p4, p5, p6, p7;
  int p8, p9;

  printf("BF-Optimizer for Djoefe: starting...\n");

  for(p1 = 20.0; p1 <= 40; p1 += 5)
//    for(p2 = 5.1; p2 <= 16.0; p2 += 5)
      for(p3 = 0.35; p3 <= 0.55; p3 += 0.05)  
	for(p4 = 0.013; p4 <= 0.017; p4 += 0.001)
	  for(p5 = 0.8; p5 <= 1.0; p5 += 0.1)
	    for(p6 = 1.05; p6 <= 1.15; p6 += 0.2)
	      for(p7 = 0.6; p7 <= 0.8; p7 += 0.05)
	//	for(p8 = 0; p8 <= 1; p8++)
//		  for(p9 = 0; p9 <= 1; p9++)
		    test_parameters(p1, 5.1, p3, p4, p5, p6, p7, 0, 0);
  
  return 0;
}












