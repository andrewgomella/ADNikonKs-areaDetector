TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC += Ks
LIB_SRCS    += Ks.cpp
LIB_LIBS_WIN32     += KsCam
LIB_LIBS    += KsCam

DBD += KsSupport.dbd

#This needs to be set otherwise you must modify KsCam.h
#such that it uses: typedef wchar_t lx_wchar;
USR_CXXFLAGS += /Zc:wchar_t-

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

