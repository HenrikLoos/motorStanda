#!../../bin/linux-x86_64/standa

< envPaths

cd "${TOP}/iocBoot/${IOC}"

## Register all support components
dbLoadDatabase "$(TOP)/dbd/standa.dbd"
standa_registerRecordDeviceDriver pdbbase

# Define the IOC prefix
< settings.iocsh

# Allstop, alldone
iocshLoad("$(MOTOR)/iocsh/allstop.iocsh", "P=$(PREFIX)")

## Standa Motor Controller
< standa.iocsh

iocInit

# Boot complete
