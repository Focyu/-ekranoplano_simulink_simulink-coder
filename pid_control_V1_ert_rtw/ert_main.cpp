#include <stdio.h>
#include <stdlib.h>
#include "pid_control_V1.h"
#include "pid_control_V1_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

static pid_control_V1 pid_control_V1_Obj;/* Instance of model class */

#define NAMELEN                        16

/* Function prototype declaration*/
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(pid_control_V1_Obj.getRTM()) == (NULL)) &&
    !rtmGetStopRequested(pid_control_V1_Obj.getRTM());
  while (runModel) {
    sem_wait(&baserateTaskSem);
    pid_control_V1_Obj.step();

    /* Get model outputs here */
    stopRequested = !((rtmGetErrorStatus(pid_control_V1_Obj.getRTM()) == (NULL))
                      && !rtmGetStopRequested(pid_control_V1_Obj.getRTM()));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(pid_control_V1_Obj.getRTM(), "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  /* Terminate model */
  pid_control_V1_Obj.terminate();
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  rtmSetErrorStatus(pid_control_V1_Obj.getRTM(), 0);

  /* Initialize model */
  pid_control_V1_Obj.initialize();

  /* Call RTOS Initialization function */
  myRTOSInit(0.01, 0);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}
