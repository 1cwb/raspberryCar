include ../common.mak

$(info ---------------------------------------------------------------------)
ifeq ($(OLED_SUPPORT), true) 
GLOBAL_CFLAGS += -DOLED_DRIVER_ON
$(info -------------------add oled mode device------------------------------)
endif
ifeq ($(DAEMON), true) 
GLOBAL_CFLAGS += -DRUN_AS_DAEMON
$(info -------------------run as a daemon-----------------------------------)
endif

CUR_PATH := $(dir $(abspath $(firstword $(MAKEFILE_LIST))))
CUR_PATH_SRC_FILE_CXX := $(CUR_PATH)*.cpp
CUR_PATH_SRC_FILE_C := $(CUR_PATH)*.c

# 目标
TARGET += maintest
# 源文件
LIBSRC_CXX := $(wildcard $(CUR_PATH_SRC_FILE_CXX))
LIBSRC_C := $(wildcard $(CUR_PATH_SRC_FILE_C))


# 源文件编译为目标文件
GLOBAL_OBJS += $(LIBSRC_C:.c=.o)
GLOBAL_OBJS += $(LIBSRC_CXX:.cpp=.o)
include ../build.mak