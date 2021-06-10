CUR_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CUR_PATH_SRC_FILE_CXX := $(CUR_PATH)*.cpp
CUR_PATH_SRC_FILE_C += $(CUR_PATH)*.c

# 头文件查找路径
GLOBAL_INC += -I $(CUR_PATH)


# 源文件
LIBSRC_CXX := $(wildcard $(CUR_PATH_SRC_FILE_CXX))
LIBSRC_C := $(wildcard $(CUR_PATH_SRC_FILE_C))

# 源文件编译为目标文件
GLOBAL_OBJS += $(LIBSRC_C:.c=.o)
GLOBAL_OBJS += $(LIBSRC_CXX:.cpp=.o)