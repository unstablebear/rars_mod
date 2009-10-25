/* 
   Brute-Force optimizer for Djoefe
   by Gian-Carlo Pascutto
*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "pvm3.h"

int *p_taskids = 0;
int p_cpus = 1;

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
  FILE *stats;
  
  int damage;
  int i;
  
  double avg_speed, best_speed;
  double dg = 0.8;
 
  static double best_best_speed = 0;

  int p_bufid;
  static int p_give = 0;
  static int p_firsttime = 1;
  int p_take;

  p_bufid = pvm_initsend(PvmDataDefault);

  pvm_packf("%lf %lf %lf %lf %lf %lf %lf %f %d %d",
	    p1, p2, p3, dg, p4, p5, p6, p7, p8, p9);

  if (p_firsttime == 1)
  {
  	pvm_send(p_taskids[0], 1);
	pvm_send(p_taskids[1], 1);
  }
  else
  {
	pvm_send(p_give, 1);
  }

  p_take = pvm_recv(-1, 2);

  pvm_bufinfo(p_take, (int*)NULL, (int*)NULL, &p_give);

  pvm_unpackf("%lf %lf %d", &avg_speed, &best_speed, &damage);

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
    }
  }

  putchar('\n');
}

int main(void)
{
  double p1, p2, p3, p4, p5, p6, p7;
  int p8, p9;
  
  int p_info;
  int p_numt;
  int p_i;

  printf("Parallel BF-Optimizer for Djoefe: starting master...\n");

  p_info = pvm_mytid();
  
  if (p_info < 0) 
    {
      printf("Error entering PVM\n");
      exit(1);
    }

  printf("PVM started\n");

  printf("Spawning slaves...\n");
  
  pvm_config(&p_cpus, (int*)NULL, (struct pvmhostinfo**)NULL);
  printf("%d tasks\n", p_cpus);

  p_taskids = (int*) malloc(p_cpus * sizeof(int));

  for (p_i = 0; p_i < p_cpus; p_i++)
    {
      if(pvm_spawn("bfpars",(char**)NULL, 0, 0, 1, &p_taskids[p_i]) < 0)
	printf("Error starting slave\n");
    }
  
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
  

  for(p_i = 0; p_i < p_cpus; p_i++)
    pvm_kill(p_taskids[p_i]);
  
  p_info = pvm_exit();

  if (p_info < 0)
    printf("Error exiting PVM\n");

  return 0;
}












