TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += Standa

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install StandaSupport.dbd into <top>/dbd
DBD += StandaSupport.dbd

StandaSupport_DBD += Standa.dbd

# specify all source files to be compiled and added to the library
Standa_SRCS += StandaDriver.cpp

Standa_LIBS += motor asyn
Standa_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

