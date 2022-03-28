#!../../bin/linux-x86_64/adq-test

## Set environment variables
< envPaths

# Define serial number of the digitizer
epicsEnvSet("ROOT", "ADQ")
epicsEnvSet("ADQSN", "06905")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/adq-test.dbd"
adq_test_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadTemplate("db/ADQDevice.substitutions4", "PREFIX = $(ROOT), ADQSN = $(ADQSN)")

## Load library of ADQ module
# ndsLoadDriver("${ADQ_LIB}/libtspd-adq.so")
ndsLoadDriver("${ADQ_LIB}/tspd-adq.dll")

## Create device with parameters: driver name (from NDS_DEFINE_DRIVER, not .SO name), rootNode, digitizer's serial number
ndsCreateDevice(tspd_adq, "$(ROOT)", "ADQSN=$(ADQSN)")

## Enable info logging in NDS, needs a rootNode name
# nds setLogLevelInfo $(ROOT)

cd "${TOP}/iocBoot/${IOC}"
iocInit
