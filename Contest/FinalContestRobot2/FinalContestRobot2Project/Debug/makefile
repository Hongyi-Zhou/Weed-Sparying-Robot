################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.3.3

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"./COECSL_edma3.obj" \
"./COECSL_mcbsp.obj" \
"./ColorLCD.obj" \
"./ColorVision.obj" \
"./LCDprintf.obj" \
"./Ladar.obj" \
"./MatrixMath.obj" \
"./pru.obj" \
"./user_FinalContestRobot2.obj" \
"./user_xy.obj" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/LabFiles/bsl_forSYSBIOS/lib/evmomapl138_bsl.lib" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
FinalContestRobot2.out \

EXE_OUTPUTS__QUOTED += \
"FinalContestRobot2.out" \

BIN_OUTPUTS += \
FinalContestRobot2.hex \

BIN_OUTPUTS__QUOTED += \
"FinalContestRobot2.hex" \


# All Target
all: $(OBJS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "FinalContestRobot2.out" secondary-outputs

# Tool invocations
FinalContestRobot2.out: $(OBJS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C6000 Linker'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.3.3/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 -z -m"FinalContestRobot2.map" --heap_size=0x200 --stack_size=0x2000 -i"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.3.3/lib" -i"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.3.3/include" --reread_libs --define=DSP_CORE=1 --diag_wrap=off --display_error_number --warn_sections --xml_link_info="FinalContestRobot2_linkInfo.xml" --rom_model -o "FinalContestRobot2.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

FinalContestRobot2.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: C6000 Hex Utility'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.3.3/bin/hex6x"  "..\..\..\..\LabFiles\SYSBIOShex\out2bootbin.cmd" -o "FinalContestRobot2.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_CMDS__QUOTED)$(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "COECSL_edma3.obj" "COECSL_mcbsp.obj" "ColorLCD.obj" "ColorVision.obj" "LCDprintf.obj" "Ladar.obj" "MatrixMath.obj" "pru.obj" "user_FinalContestRobot2.obj" "user_xy.obj" 
	-$(RM) "COECSL_edma3.d" "COECSL_mcbsp.d" "ColorLCD.d" "ColorVision.d" "LCDprintf.d" "Ladar.d" "MatrixMath.d" "pru.d" "user_FinalContestRobot2.d" "user_xy.d" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

