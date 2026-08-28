/*
FILENAME...   StandaDriver.cpp
USAGE...      Standa motor driver support
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "StandaDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/****************************************************
 * These are the StandaController data packet types *
 ****************************************************/

/* Get and set position data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    int32_t  position;   //
    uint16_t uPosition;  //
    int64_t encPosition; //
    uint8_t posFlags;    //
    uint8_t reserved1;
    uint32_t reserved2;
    uint16_t crc;
} position_data_t;

/* Status data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    uint8_t moveSts;
    uint8_t moveCmdSts;
    uint8_t pwrSts;
    uint8_t encSts;
    uint8_t windSts;
    int32_t curPosition;   //
    int16_t uCurPosition;  //
    int64_t encPosition; //
    int32_t curSpeed;   //
    int16_t uCurSpeed;  //
    int16_t iPwr;
    int16_t uPwr;
    int16_t iUsb;
    int16_t uUsb;
    int16_t curT;
    uint32_t flags;
    uint32_t gpioFlags;
    uint8_t cmdBufFreeSpace;
    uint32_t reserved1;
    uint16_t crc;
} status_data_t;

/* Get and set speed and accel data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    uint32_t  speed;        // target speed steps/s
    uint8_t uSpeed;         // u-step speed
    uint16_t accel;         // steps/s^2
    uint16_t decel;         // steps/s^2
    uint32_t antiplaySpeed; // antiplay mode speed steps/s
    uint8_t uAntiplaySpeed; // antiplay mode u-step speed
    uint16_t reserved1;
    uint64_t reserved2;
    uint16_t crc;
} speed_data_t;

/* Movement data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    int32_t  position;
    uint16_t uPosition;
    int64_t encPosition;
    uint8_t posFlags;
    uint8_t reserved1;
    uint32_t reserved2;
    uint16_t crc;
} move_data_t;

/* Engine data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    uint16_t nomVoltage;
    uint16_t nomCurrent;
    uint32_t nomSpeed;
    uint8_t uNomSpeed;
    uint16_t engineFlags;
    int16_t antiplay;
    uint8_t microstepMode;
    uint16_t stepsPerRev;
    uint32_t reserved1;
    uint64_t reserved2;
    uint16_t crc;
} engine_data_t;

/* Accessories data */
typedef struct __attribute__((packed)) {
    char cmd[4];
    char magneticBrakeInfo[24];
    uint32_t brake1;
    uint32_t brake2;
    uint32_t brake3;
    uint32_t brake4;
    char temperatureSensorInfo[24];
    uint32_t ts1;
    uint32_t ts2;
    uint32_t ts3;
    uint32_t ts4;
    uint32_t limitSwitchSettings;
    char reserved[24];
    uint16_t crc;
} access_data_t;


/************************************************
 * These are the StandaController methods *
 ************************************************/


/** Creates a new StandaController object.
  * \param[in] controllerPortName      The name of the asyn port that will be created for this driver
  * \param[in] communicationPortName   The name of the drvAsynSerialPort that was created previously to connect to the Standa controller 
  * \param[in] numAxes                 The number of axes that this controller supports 
  * \param[in] movingPollPeriod        The time between polls when any axis is moving 
  * \param[in] idlePollPeriod          The time between polls when no axis is moving 
  */
StandaController::StandaController(const char *controllerPortName, const char *communicationPortName, int numAxes, 
                                 double movingPollPeriod,double idlePollPeriod)
  :  asynMotorController(controllerPortName, numAxes, NUM_STANDA_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  int axis;
  StandaAxis *pAxis;
  static const char *functionName = "StandaController::StandaController";

  /* Connect to Standa controller */
  status = pasynOctetSyncIO->connect(communicationPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to standa controller\n",
      functionName);
  }

  /*
   * Controller, NOT axis-specific, initialization can go here
   */

  // If additional information is required for creating axes (stepsPerUnit), comment out 
  // the following loop and make the user call StandaCreateAxis from the cmd file
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new StandaAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new StandaController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] StandaPortName       The name of the drvAsynIPPPort that was created previously to connect to the Standa controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int StandaCreateController(const char *portName, const char *StandaPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  StandaController *pStandaController
    = new StandaController(portName, StandaPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pStandaController = NULL;
  return(asynSuccess);
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void StandaController::report(FILE *fp, int level)
{
  fprintf(fp, "Standa Motor Controller driver %s\n", this->portName);
  fprintf(fp, "    numAxes=%d\n", numAxes_);
  fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
  fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

  /*
   * It is a good idea to print private variables that were added to the StandaController class in StandaDriver.h, here
   * This allows you to see what is going on by running the "dbior" command from iocsh.
   */

  // Call the base class method
  asynMotorController::report(fp, level);
}


/** Returns a pointer to an StandaAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
StandaAxis* StandaController::getAxis(asynUser *pasynUser)
{
  return static_cast<StandaAxis*>(asynMotorController::getAxis(pasynUser));
}


/** Returns a pointer to an StandaAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
StandaAxis* StandaController::getAxis(int axisNo)
{
  return static_cast<StandaAxis*>(asynMotorController::getAxis(axisNo));
}


/** CRC Calculation */
unsigned short CRC16(uint8_t *pbuf, unsigned short n)
{
  unsigned short crc, i, j, carry_flag, a;
  crc = 0xffff;
  for(i = 0; i < n; i++)
  {
    crc = crc ^ pbuf[i];
    for(j = 0; j < 8; j++)
    {
      a = crc;
      carry_flag = a & 0x0001;
      crc = crc >> 1;
      if ( carry_flag == 1 ) crc = crc ^ 0xa001;
    }
  }
  return crc;
}


/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[in] outChars Number of bytes in the output buffer to send.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Number of bytes to read into the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus StandaController::writeReadControllerNBytes(const char *output, size_t outChars, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadControllerNBytes";
  
  status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                       outChars, input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);

  return status;
}


/** Writes N bytes string to the Standa controller and reads N bytes response.
  * Calls writeReadControllerNBytes() with default locations of the input and
  * output strings and default timeout.
  * Puts CRC into last two bytes, N bytes to write includes CRC bytes */
asynStatus StandaController::writeReadStanda(size_t outChars, size_t inChars)
{
  asynStatus status;
  size_t nread;
  uint16_t crc;
  uint16_t crcIn;

  // Add CRC if outChars > 4
  if (outChars > 4) {
    crc = CRC16((unsigned char*)outString_ + 4, outChars - 6);
    memcpy(outString_ + outChars - 2, &crc, 2);
  }

  status = writeReadControllerNBytes(outString_, outChars, inString_, inChars, &nread, DEFAULT_CONTROLLER_TIMEOUT);

  // Test CRC if inChars > 4
  if (inChars > 4) {
    crc = CRC16((unsigned char*)inString_ + 4, inChars - 6);
    crcIn = *(uint16_t*)(inString_ + inChars - 2);
    if (crc != crcIn) {
      status = asynError;
    }
  }

  return status;
}


/** Assenble output string from integer inputs */
size_t concatIntList(char* dest, int count, ...)
{
  va_list args;
  va_start(args, count);

  char* ptr = dest;
//  printf("%d ", count);

  for (int i = 0; i < count; i++) {
    size_t len = va_arg(args, size_t);
    uint64_t val = va_arg(args, uint64_t);
//    printf("%d %ld ", len, val);
    memcpy(ptr, &val, len);
    ptr += len;
  }
//  printf("\n");
  va_end(args);
  return (size_t)(ptr - dest);
}


/******************************************
 * These are the StandaAxis methods *
 ******************************************/


/** Creates a new StandaAxis object.
  * \param[in] pC Pointer to the StandaController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
// Note: the following constructor needs to be modified to accept the stepSize argument if StandaCreateAxis
// will be called from iocsh, which is necessary for controllers that work in EGU instead of steps.
StandaAxis::StandaAxis(StandaController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  //asynStatus status;
  asynStatus status;
  // static const char *functionName = "Standa::sendAccelAndVelocity";
  engine_data_t *pData;
  access_data_t *pDataAcc;
  
  axisIndex_ = axisNo + 1;

  /*
   * Axis-specific initialization can go here
   */

  sprintf(pC_->outString_, "geng");
  status = pC_->writeReadStanda(4, 30);
  pData = (engine_data_t *)&pC_->inString_;
  printf("%d, %d, %d, %d, %d, %d %d %d\n", pData->nomVoltage, pData->nomCurrent, pData->nomSpeed, pData->uNomSpeed, pData->engineFlags, pData->antiplay, pData->microstepMode, pData->stepsPerRev);

  sprintf(pC_->outString_, "gacc");
  status = pC_->writeReadStanda(4, 114);
  pDataAcc = (access_data_t *)&pC_->inString_;
  printf("%d\n", pDataAcc->limitSwitchSettings);

  
  // Zero the encoder position (this only appears to be a problem on windows)
  setDoubleParam(pC_->motorEncoderPosition_, 0.0);

  // Tell the motor record the axis has an encoder
  setIntegerParam(pC->motorStatusHasEncoder_, 1);
  // Allow CNEN to turn motor power on/off
  //setIntegerParam(pC->motorStatusGainSupport_, 1);

  // Make the changed parameters take effect
  callParamCallbacks();
}

/*
 * If the controller reports positions in EGU, rather than integer steps, and the number of stepsPerUnit
 * can vary between axes, the user is required to configure each axis.  The following function, as well as
 * the corresponding registration code at the end of this file should be uncommented.  The declarations in
 * StandaDrive.h also need to be uncommented.  The StandaAxis construtor will need to be modifed
 * to accept the (double) stepSize argument.
 * The Newport XPS support is an standa of how this is done for a real controller.
 */
/*
extern "C" int StandaCreateAxis(const char *StandaName, int axisNo, const char *setpsPerUnit)
{
  StandaController *pC;
  double stepSize;
   static const char *functionName = "StandaCreateAxis";
 
  pC = (StandaController*) findAsynPortDriver(StandaName);
  if (!pC) 
  {
    printf("Error port %s not found\n", StandaName);
    return asynError;
  }

  stepSize = strtod(stepsPerUnit, NULL);

  pC->lock();
  new StandaAxis(pC, axisNo, 1./stepSize);
  pC->unlock();
  return asynSuccess;
}
*/

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void StandaAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "    Axis #%d\n", axisNo_);
    fprintf(fp, "        axisIndex_=%d\n", axisIndex_);
 }

  /*
   * It is a good idea to print private variables that were added to the StandaAxis class in StandaDriver.h, here
   * This allows you to see what is going on by running the "dbior" command from iocsh.
   */

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/*
 * sendAccelAndVelocity() is called by StandaAxis methods that result in the motor moving: move(), moveVelocity(), home()
 *
 * Arguments in terms of motor record fields:
 *     baseVelocity (steps/s) = VBAS / abs(MRES)
 *     velocity (step/s) = VELO / abs(MRES)
 *     acceleration (step/s/s) = (velocity - baseVelocity) / ACCL
 */
asynStatus StandaAxis::sendAccelAndVelocity(double acceleration, double velocity, double baseVelocity) 
{
  asynStatus status;
  // static const char *functionName = "Standa::sendAccelAndVelocity";
  speed_data_t *pDataIn, *pData;

/*
  // Send the base velocity
  sprintf(pC_->outString_, "%d BAS %f", axisIndex_, baseVelocity);
  status = pC_->writeReadController();

  // Send the velocity
  sprintf(pC_->outString_, "%d VEL %f", axisIndex_, velocity);
  status = pC_->writeReadController();

  // Send the acceleration
  sprintf(pC_->outString_, "%d ACC %f", axisIndex_, acceleration);
  status = pC_->writeReadController();
*/

//  printf("%f, %f, %f\n", acceleration, velocity, baseVelocity);

  // Get the present speed settings
  sprintf(pC_->outString_, "gmov");
  status = pC_->writeReadStanda(4, 30);
  pDataIn = (speed_data_t *)&pC_->inString_;
//  printf("%d, %d, %d, %d, %d, %d\n", pDataIn->speed, pDataIn->uSpeed, pDataIn->accel, pDataIn->decel, pDataIn->antiplaySpeed, pDataIn->uAntiplaySpeed);

  // Copy present speed settings into output buffer
  pData = (speed_data_t *)&pC_->outString_;
  *pData = *pDataIn;

  // Send the velocity and acceleration
  sprintf(pC_->outString_, "smov");
  pData->speed = NINT(abs(velocity));
  pData->accel = NINT(acceleration);
  pData->decel = NINT(acceleration*2.5); // deceleration at 2.5x acceleration
  pData->reserved1 = 0;
  pData->reserved2 = 0;
  status = pC_->writeReadStanda(30, 4);
//  printf("%d, %d, %d, %d, %d, %d\n", pData->speed, pData->uSpeed, pData->accel, pData->decel, pData->antiplaySpeed, pData->uAntiplaySpeed);

  return status;
}


/*
 * move() is called by asynMotor device support when an absolute or a relative move is requested.
 * It can be called multiple times if BDST > 0 or RTRY > 0.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = RVAL = DVAL / MRES
 *     baseVelocity (steps/s) = VBAS / abs(MRES)
 *     velocity (step/s) = VELO / abs(MRES)
 *     acceleration (step/s/s) = (velocity - baseVelocity) / ACCL
 */
asynStatus StandaAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "StandaAxis::move";
  int32_t pos;
  int16_t upos;
  int16_t zero;

  status = sendAccelAndVelocity(acceleration, maxVelocity, minVelocity);
  
  // Set the target position
  if (relative) {
    //sprintf(pC_->outString_, "%d MR %d", axisIndex_, NINT(position));
    sprintf(pC_->outString_, "movr");
  } else {
    //sprintf(pC_->outString_, "%d MV %d", axisIndex_, NINT(position));
    sprintf(pC_->outString_, "move");
  }

  pos = NINT(position);
  upos = 0;
  zero = 0;
  concatIntList(pC_->outString_ + 4, 5, 4, pos, 2, upos, 2, zero, 2, zero, 2, zero);

  //status = pC_->writeReadController();
  status = pC_->writeReadStanda(18, 4);

  // If controller has a "go" command, send it here

  return status;
}


/*
 * home() is called by asynMotor device support when a home is requested.
 * Note: forwards is set by device support, NOT by the motor record.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = HVEL / abs(MRES)
 *     acceleration (step/s/s) = (maxVelocity - minVelocity) / ACCL
 *     forwards = 1 if HOMF was pressed, 0 if HOMR was pressed
 */

asynStatus StandaAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "StandaAxis::home";

  sprintf(pC_->outString_, "home");
  status = pC_->writeReadStanda(4, 4);
  return status;
}


/*
 * moveVelocity() is called by asynMotor device support when a jog is requested.
 * If a controller doesn't have a jog command (or jog commands), this a jog can be simulated here.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = (jog_direction == forward) ? JVEL * DIR / MRES : -1 * JVEL * DIR / MRES
 *     acceleration (step/s/s) = JAR / abs(EGU)
 */
asynStatus StandaAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  //static const char *functionName = "StandaAxis::moveVelocity";

  // Call this to set the max current and acceleration
  status = sendAccelAndVelocity(acceleration, maxVelocity, minVelocity);

  //sprintf(pC_->outString_, "%d JOG %f", axisIndex_, maxVelocity);
  if (maxVelocity < 0)
  {
    // Send "left" command
    sprintf(pC_->outString_, "left");
  }
  else
  {
    // Send "rigt" command
    sprintf(pC_->outString_, "rigt");
  }

  //status = pC_->writeReadController();
  status = pC_->writeReadStanda(4, 4);
  return status;
}


/*
 * stop() is called by asynMotor device support whenever a user presses the stop button.
 * It is also called when the jog button is released.
 *
 * Arguments in terms of motor record fields:
 *     acceleration = ??? 
 */
asynStatus StandaAxis::stop(double acceleration)
{
  asynStatus status;
  //static const char *functionName = "StandaAxis::stop";

  //sprintf(pC_->outString_, "%d AB", axisIndex_);
  sprintf(pC_->outString_, "stop");
//  sprintf(pC_->outString_, "sstp");
  //status = pC_->writeReadController();
  status = pC_->writeReadStanda(4, 4);
  return status;
}


/*
 * setPosition() is called by asynMotor device support when a position is redefined.
 * It is also required for autosave to restore a position to the controller at iocInit.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = DVAL / MRES = RVAL 
 */
asynStatus StandaAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "StandaAxis::setPosition";
  position_data_t *pData;

  //sprintf(pC_->outString_, "%d POS %d", axisIndex_, NINT(position));
  sprintf(pC_->outString_, "spos");
  pData = (position_data_t *)&pC_->outString_;
  pData->position = NINT(position);
  pData->uPosition = 0;
  pData->encPosition = NINT(position);
  pData->posFlags = 0; // Flags set to 0 to both reload motor and encoder position
  pData->reserved1 = 0;
  pData->reserved2 = 0;

  //status = pC_->writeReadController();
  status = pC_->writeReadStanda(26, 4);
  return status;
}


/*
 * setClosedLoop() is called by asynMotor device support when a user enables or disables torque, 
 * usually from the motorx_all.adl, but only for drivers that set the following params to 1:
 *   pC->motorStatusGainSupport_
 *   pC->motorStatusHasEncoder_
 * What is actually implemented here varies greatly based on the specfics of the controller.
 * 
 * Arguments in terms of motor record fields:
 *     closedLoop = CNEN 
 */
/*
asynStatus StandaAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "StandaAxis::setClosedLoop";
  
  if (closedLoop)
  {
    // Build "Enable" command
    sprintf(pC_->outString_, "%d EN", axisIndex_);
  }
  else
  {
    // Build "Disable" command
    sprintf(pC_->outString_, "%d DI", axisIndex_);
  }

  // Send the command
  status = pC_->writeController();
  return status;
}
*/


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus StandaAxis::poll(bool *moving)
{ 
  int position;
  int status;
  int done;
  int direction;
  int limit;
  asynStatus comStatus;
  unsigned char* test;
//  int32_t pos;
//  int16_t upos;
//  int64_t encpos;
  uint8_t stat;
  position_data_t *pData;
  status_data_t *pDataStat;

  // Read the current motor position
  //sprintf(pC_->outString_, "%d POS?", axisIndex_);
  sprintf(pC_->outString_, "gpos");
  //comStatus = pC_->writeReadController();
  comStatus = pC_->writeReadStanda(4, 26);
  if (comStatus) 
    goto skip;
  // The response string is of the form "0.00000"

//  test = (unsigned char *) &pC_->inString_;
//  pos = *(int32_t*)(test + 4);
  //position = atof((const char *) &pC_->inString_);
  pData = (position_data_t *)&pC_->inString_;
  position = pData->position;
  setDoubleParam(pC_->motorPosition_, position);

  // Read the current feedback position
  //sprintf(pC_->outString_, "%d FBK?", axisIndex_);
  //comStatus = pC_->writeReadController();
  //if (comStatus) 
  //  goto skip;
  // The response string is of the form "0.00000"
  //position = atof((const char *) &pC_->inString_);
  position = pData->encPosition;
  setDoubleParam(pC_->motorEncoderPosition_, position);

  // Read the moving status of this motor
  //sprintf(pC_->outString_, "%d ST?", axisIndex_);
  sprintf(pC_->outString_, "gets");
  //comStatus = pC_->writeReadController();
  comStatus = pC_->writeReadStanda(4, 54);
  if (comStatus) 
    goto skip;
  // The response string is of the form "1"
  //status = atoi((const char *) &pC_->inString_);
  test = (unsigned char *) &pC_->inString_;
//  stat = *(test + 4);
//  printf("%i\n", stat);
  stat = *(test + 5);
//  printf("%i\n", stat);
//  status = stat;
  pDataStat = (status_data_t *)&pC_->inString_;
  status = pDataStat->moveCmdSts;
//  printf("%d %d %d %d %d %d %d %ld %d %d\n",pDataStat->moveSts, pDataStat->moveCmdSts, pDataStat->pwrSts, pDataStat->encSts, pDataStat->windSts, pDataStat->curPosition, pDataStat->uCurPosition, pDataStat->encPosition, pDataStat->curSpeed, pDataStat->uCurSpeed);
//  printf("%d %d %d %d %d %d %d %d\n",pDataStat->iPwr, pDataStat->uPwr, pDataStat->iUsb, pDataStat->uUsb, pDataStat->curT, pDataStat->flags, pDataStat->gpioFlags, pDataStat->cmdBufFreeSpace);

  // Read the direction
  //direction = (status & 0x1) ? 1 : 0;
  direction = 0;
  setIntegerParam(pC_->motorStatusDirection_, direction);

  // Read the moving status
  //done = (status & 0x2) ? 1 : 0;
  done = (status & 0x80) ? 0 : 1;
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_, !done);
  *moving = done ? false:true;
  goto skip;

  // Read the limit status
  limit = (status & 0x8) ? 1 : 0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (status & 0x10) ? 1 : 0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);

  // Read the home status
  // TODO: implementing homing
  
  // Read the drive power on status
  //driveOn = (status & 0x100) ? 0 : 1;
  //setIntegerParam(pC_->motorStatusPowerOn_, driveOn);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}


/** Code for iocsh registration */
static const iocshArg StandaCreateControllerArg0 = {"Controller Port name", iocshArgString};
static const iocshArg StandaCreateControllerArg1 = {"Communication port name", iocshArgString};
static const iocshArg StandaCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg StandaCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg StandaCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const StandaCreateControllerArgs[] = {&StandaCreateControllerArg0,
                                                             &StandaCreateControllerArg1,
                                                             &StandaCreateControllerArg2,
                                                             &StandaCreateControllerArg3,
                                                             &StandaCreateControllerArg4};
static const iocshFuncDef StandaCreateControllerDef = {"StandaCreateController", 5, StandaCreateControllerArgs};
static void StandaCreateContollerCallFunc(const iocshArgBuf *args)
{
  StandaCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* StandaCreateAxis */
/*
static const iocshArg StandaCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg StandaCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg StandaCreateAxisArg2 = {"stepsPerUnit", iocshArgString};
static const iocshArg * const StandaCreateAxisArgs[] = {&StandaCreateAxisArg0,
                                                     &StandaCreateAxisArg1,
                                                     &StandaCreateAxisArg2};
static const iocshFuncDef StandaCreateAxisDef = {"StandaCreateAxis", 3, StandaCreateAxisArgs};
static void StandaCreateAxisCallFunc(const iocshArgBuf *args)
{
  StandaCreateAxis(args[0].sval, args[1].ival, args[2].sval);
}
*/


static void StandaRegister(void)
{
  iocshRegister(&StandaCreateControllerDef, StandaCreateContollerCallFunc);
  //iocshRegister(&StandaCreateAxisDef,       StandaCreateAxisCallFunc);
}


extern "C" {
epicsExportRegistrar(StandaRegister);
}
