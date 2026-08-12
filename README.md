# motorStanda

An standa motor driver module, created with makeBaseApp.pl from EPICS base
7.0.8, and manually modified to build as a standalone module or as a submoule
of motor.  Includes an standa motor driver based on the support in [motorVMC](https://github.com/epics-motor/motorVMC).

## Usage

The [createMotorDriverModule](https://github.com/epics-motor/createMotorDriverModule) python script can be used to create a new EPICS 
motor driver module, based on motorStanda.
