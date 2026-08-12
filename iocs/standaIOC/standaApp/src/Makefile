TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = standa
# standa.dbd will be created and installed
DBD += standa.dbd

# standa.dbd will be made up from these files:
standa_DBD += base.dbd

# Include dbd files from all support applications:
standa_DBD += asyn.dbd 
standa_DBD += drvAsynIPPort.dbd
standa_DBD += motorSupport.dbd
standa_DBD += StandaSupport.dbd

# Add all the support libraries needed by this IOC
standa_LIBS += Standa
standa_LIBS += motor
standa_LIBS += asyn

# The std module provides the doAfterIocInit iocsh command
ifdef STD
  standa_DBD += stdSupport.dbd
  standa_LIBS := std $(standa_LIBS)
endif

# standa_registerRecordDeviceDriver.cpp derives from standa.dbd
standa_SRCS += standa_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
standa_SRCS_DEFAULT += standaMain.cpp
standa_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#standa_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
standa_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

