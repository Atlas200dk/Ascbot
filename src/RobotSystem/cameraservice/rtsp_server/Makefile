
LIB_DIR      =.
LIVE_DIR    = $(LIB_DIR)/live
INCLUDES = -I$(LIVE_DIR)/UsageEnvironment/include \
		   -I$(LIVE_DIR)/groupsock/include\
           -I$(LIVE_DIR)/liveMedia/include \
		   -I$(LIVE_DIR)/BasicUsageEnvironment/include


USEDYNAMIC:=n


LIBINCLUDE+=-L./   -lrt -lpthread



# Default library filename suffixes for each library that we link with.  The "config.*" file might redefine these later.
libliveMedia_LIB_SUFFIX = $(LIB_SUFFIX)
libBasicUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libUsageEnvironment_LIB_SUFFIX = $(LIB_SUFFIX)
libgroupsock_LIB_SUFFIX = $(LIB_SUFFIX)
##### Change the following for your environment: aarch64-linux-gnu-
CROSS_COMPILE?=		aarch64-linux-gnu-
COMPILE_OPTS =		$(INCLUDES) -I. -O2 -DSOCKLEN_T=socklen_t -DNO_SSTREAM=1 -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64
C =			c
C_COMPILER =		$(CROSS_COMPILE)gcc
C_FLAGS =		$(COMPILE_OPTS)
CPP =			cpp
CPLUSPLUS_COMPILER =	$(CROSS_COMPILE)g++
CPLUSPLUS_FLAGS =	$(COMPILE_OPTS) -Wall -DBSD=1 -DLOCALE_NOT_USED
OBJ =			o
LINK =			$(CROSS_COMPILE)g++ -o
LINK_OPTS =
CONSOLE_LINK_OPTS =	$(LINK_OPTS)
LIBRARY_LINK =		$(CROSS_COMPILE)ar cr
LIBRARY_LINK_OPTS =	$(LINK_OPTS)
LIB_SUFFIX =			a
LIBS_FOR_CONSOLE_APPLICATION =
LIBS_FOR_GUI_APPLICATION =
EXE =##### End of variables to change


STREAMER_APPS = rtsp-server$(EXE)



PREFIX = /usr/local
ALL = $(STREAMER_APPS)
all:
	cd live; $(MAKE)
	$(MAKE) $(ALL)


.$(C).$(OBJ):
	$(C_COMPILER) -c $(C_FLAGS) $<
.$(CPP).$(OBJ):
	$(CPLUSPLUS_COMPILER) -c $(CPLUSPLUS_FLAGS) $<


RTSP_SERVER_OBJS        =   H264VideoServerMediaSubsession.$(OBJ) \
							CircularQueue.$(OBJ)\
							H264VideoSource.$(OBJ)\
							RtspServer.$(OBJ)


USAGE_ENVIRONMENT_DIR = $(LIVE_DIR)/UsageEnvironment
USAGE_ENVIRONMENT_LIB = $(USAGE_ENVIRONMENT_DIR)/libUsageEnvironment.$(libUsageEnvironment_LIB_SUFFIX)
BASIC_USAGE_ENVIRONMENT_DIR = $(LIVE_DIR)/BasicUsageEnvironment
BASIC_USAGE_ENVIRONMENT_LIB = $(BASIC_USAGE_ENVIRONMENT_DIR)/libBasicUsageEnvironment.$(libBasicUsageEnvironment_LIB_SUFFIX)
LIVEMEDIA_DIR = $(LIVE_DIR)/liveMedia
LIVEMEDIA_LIB = $(LIVEMEDIA_DIR)/libliveMedia.$(libliveMedia_LIB_SUFFIX)
GROUPSOCK_DIR = $(LIVE_DIR)/groupsock
GROUPSOCK_LIB = $(GROUPSOCK_DIR)/libgroupsock.$(libgroupsock_LIB_SUFFIX)
LOCAL_LIBS =	$(LIVEMEDIA_LIB) $(GROUPSOCK_LIB) \
		$(BASIC_USAGE_ENVIRONMENT_LIB) $(USAGE_ENVIRONMENT_LIB)
LIBS =			$(LOCAL_LIBS) $(LIBS_FOR_CONSOLE_APPLICATION)


$(STREAMER_APPS):	$(RTSP_SERVER_OBJS) $(LOCAL_LIBS)
	$(LINK)$@ $(CONSOLE_LINK_OPTS) $(RTSP_SERVER_OBJS) $(LIBS)   $(LIBINCLUDE)

clean:
	-rm -rf *.$(OBJ) $(ALL) core *.core *~ include/*~

install: $(ALL)
	  install -d $(DESTDIR)$(PREFIX)/bin
	  install -m 755 $(ALL) $(DESTDIR)$(PREFIX)/bin

##### Any additional, platform-specific rules come here:
