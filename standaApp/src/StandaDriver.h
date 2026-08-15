/*
FILENAME...   StandaDriver.h
USAGE...      Standa motor driver support
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_STANDA_AXES 32     /* motor.h sets the maximum number of axes */
//#define BUFF_SIZE 20		/* Maximum length of string to/from Standa */

// No controller-specific parameters yet
#define NUM_STANDA_PARAMS 0  

class epicsShareClass StandaAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  StandaAxis(class StandaController *pC, int axisNo);
  //StandaAxis(class StandaController *pC, int axisNo, double stepSize);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  //asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  //asynStatus setClosedLoop(bool closedLoop);

private:
  StandaController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;
  //double stepsSize_;
  asynStatus sendAccelAndVelocity(double accel, double velocity, double baseVelocity);
  
friend class StandaController;
};

class epicsShareClass StandaController : public asynMotorController {
public:
  StandaController(const char *controllerPortName, const char *communicationPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  StandaAxis* getAxis(asynUser *pasynUser);
  StandaAxis* getAxis(int axisNo);

//private:
//  char buff_[BUFF_SIZE];

  protected:
  /* These are functions implementing the Standa controller communication protocol */
  asynStatus writeReadControllerNBytes(const char *output, size_t outChars, char *response, size_t maxResponseLen, size_t *responseLen, double timeout);
  asynStatus writeReadStanda(size_t outChars, size_t maxResponseLen);

friend class StandaAxis;
};
