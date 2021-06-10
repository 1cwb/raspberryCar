DIRS := $(shell find $(GLOBAL_TOP_PATH) -maxdepth 5 -type d)
DIRS := $(filter-out $(GLOBAL_TOP_PATH), $(DIRS))

MAKEFILES += $(foreach dir, $(DIRS), $(wildcard $(dir)/*.mak))
include $(MAKEFILES)

#include $(GLOBAL_TOP_PATH)stdlib/stdlibbuild.mak
#include $(GLOBAL_TOP_PATH)oled/oledbuild.mak
#include $(GLOBAL_TOP_PATH)network/networkbuild.mak
#include $(GLOBAL_TOP_PATH)fonts/fontsbuild.mak
#include $(GLOBAL_TOP_PATH)network/networkbuild.mak

CLOBAL_OUTPOT_OBJS_FILE += $(foreach text,$(GLOBAL_OBJS), $(addprefix $(GLOBAL_OUTPUT_OBJS_PATH)/,$(notdir $(text))))
$(info $(CLOBAL_OUTPOT_OBJS_FILE))

all: creat_dir $(TARGET)

# 链接为可执行文件
$(TARGET): $(GLOBAL_OBJS)
	$(GLOBAL_CC) $(CLOBAL_OUTPOT_OBJS_FILE) -o $@ $(GLOBAL_LDFLAGS) $(GLOBAL_LDLIBS)

# 清除可执行文件和目标文件
clean:
#	rm -f $(GLOBAL_OBJS)
	rm -f $(TARGET)
	@$(call RM_DIR,$(GLOBAL_OUTPUT_OBJS_PATH))

# 编译规则 加入头文件 $@代表目标文件 $< 代表第一个依赖文件
%.o:%.cpp 
	$(GLOBAL_CXX) $(GLOBAL_CFLAGS) $(GLOBAL_INC) -o $@ -c $<
	@`mv $@ $(GLOBAL_OUTPUT_OBJS_PATH)`

%.o:%.c
	$(GLOBAL_CC) $(GLOBAL_CFLAGS) $(GLOBAL_INC) -o $@ -c $<
	@`mv $@ $(GLOBAL_OUTPUT_OBJS_PATH)`

creat_dir:
	@$(call CRT_DIR,$(GLOBAL_OUTPUT_OBJS_PATH))