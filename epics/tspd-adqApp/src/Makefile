TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library
# Name of the device library
LIBRARY_IOC += tspd-adq
LIB += tspd-adq
# tspd-adq.dbd only exists if $(CODAC_DBD) is defined, i.e. only within the ITER environment.
# DBD += tspd-adq.dbd

# We enable the c++11 flag, plus we define the NDS3_DLL preprocessor macro.
USR_CXXFLAGS_Linux = -std=c++11 -Wall -Wextra -Werror -pedantic -DNDS3_DLL -pthread -DLINUX
USR_INCLUDES_WIN32 += -I../../../../ADQAPI

ifeq ($(SHARED_LIBRARIES), YES)
   USR_CXXFLAGS_WIN32 += -DNDS3_DLL -DNDS3_DLL_EXPORTS
endif


USR_CXXFLAGS_WIN32 += -DNOMINMAX

# Dependency on NDS3 and ADQAPI
tspd-adq_LIBS = nds3-epics

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
tspd-adq_LIBS_WIN32 += nds3 ../../../../ADQAPI/x86/ADQAPI
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
tspd-adq_LIBS_WIN32 += nds3 ../../../../ADQAPI/x64/ADQAPI
endif

tspd-adq_SYS_LIBS_Linux += nds3 adq

# tspd-adq_registerRecordDeviceDriver.cpp derives from tspd-adq.dbd
tspd-adq_SRCS += ADQAIChannel.cpp  ADQAIChannelGroup.cpp ADQInfo.cpp ADQDevice.cpp

#=============================
# Include makefile snippets to add support for additional modules

-include $(EPICS_ROOT)/mk/codac-common.mk

tspd-adq_DBD  += $(CODAC_DBD)
tspd-adq_LIBS += $(CODAC_LIBS)

#=============================
# Include SNL program makefile snippets

#-include ../*.snlprog

#=============================
# Finally link to the EPICS Base libraries

tspd-adq_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

