/* 
   Brute-Force optimizer for Djoefe
   by Gian-Carlo Pascutto
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


void test_parameters(void)
{
  FILE *bfout;
  FILE *racerep;

  
  char dummy[80], dummyc;
  int dummya, dummyb, dummyd, dummye;
  int damage;
  int i;
  
  double avg_speed, best_speed;
  double dg = 0.8;
  double p1, p2, p3, p4, p5, p6, p7;
  int p8, p9;
 
  static double best_best_speed = 0;
 
  int p_msgid;

  p_msgid = pvm_recv(pvm_parent(), 1);

  pvm_unpackf("%lf %lf %lf %lf %lf %lf %lf %lf %d %d\n",
	      &p1, &p2, &p3, &dg, &p4, &p5, &p6, &p7, &p8, &p9);

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

  pvm_initsend(PvmDataDefault);

  pvm_packf("%lf %lf %d", avg_speed, best_speed, damage);

  pvm_send(pvm_parent(), 2);

}

int main(void)
{
  int p_mytaskid;
  
  printf("Parallel BF-Optimizer for Djoefe: starting slave...\n");

  p_mytaskid = pvm_mytid();

  while(1)
    test_parameters();

  pvm_exit();
  exit(1);

  return 0;
}












