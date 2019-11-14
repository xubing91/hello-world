################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/WZM/software/ccsv6/tools/compiler/c2000_6.2.5

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./src/trajectory.obj" \
"./src/message.obj" \
"./src/mathex.obj" \
"./src/formation.obj" \
"./src/flightFSM.obj" \
"./src/filter.obj" \
"./src/decentralized.obj" \
"./src/constants.obj" \
"./src/car.obj" \
"./src/MS5803-01BA.obj" \
"./src/Hal.obj" \
"./src/HMC5983.obj" \
"./src/Fml.obj" \
"./src/DSP28xxx_usDelay.obj" \
"./src/DSP28xxx_SectionCopy_nonBIOS.obj" \
"./src/DSP28xxx_CodeStartBranch_FLASH.obj" \
"./src/DSP2833x_Xintf.obj" \
"./src/DSP2833x_SysCtrl.obj" \
"./src/DSP2833x_PieVect.obj" \
"./src/DSP2833x_PieCtrl.obj" \
"./src/DSP2833x_I2C.obj" \
"./src/DSP2833x_GlobalVariableDefs.obj" \
"./src/DSP2833x_DefaultIsr.obj" \
"./src/DSP2833x_CpuTimers.obj" \
"./src/DSP2833x_ADC_cal.obj" \
"../cmd/DSP2833x_Headers_nonBIOS.cmd" \
"../cmd/F2833x_nonBIOS_flash.cmd" \
-l"rts2800_fpu32.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir_vars.mk
-include cmd/subdir_vars.mk
-include src/subdir_rules.mk
-include cmd/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
car0407.out \

EXE_OUTPUTS__QUOTED += \
"car0407.out" \


# All Target
all: car0407.out

# Tool invocations
car0407.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"E:/WZM/software/ccsv6/tools/compiler/c2000_6.2.5/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --diag_warning=225 --display_error_number --diag_wrap=off -z -m"car0407.map" --stack_size=0x300 --warn_sections -i"E:/WZM/software/ccsv6/tools/compiler/c2000_6.2.5/lib" -i"E:/WZM/car0407/include" -i"E:/WZM/car0407/src" -i"E:/WZM/software/ccsv6/tools/compiler/c2000_6.2.5/include" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="car0407_linkInfo.xml" --rom_model -o "car0407.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "src\DSP2833x_CpuTimers.pp" "src\DSP2833x_DefaultIsr.pp" "src\DSP2833x_GlobalVariableDefs.pp" "src\DSP2833x_I2C.pp" "src\DSP2833x_PieCtrl.pp" "src\DSP2833x_PieVect.pp" "src\DSP2833x_SysCtrl.pp" "src\DSP2833x_Xintf.pp" "src\Fml.pp" "src\HMC5983.pp" "src\Hal.pp" "src\MS5803-01BA.pp" "src\car.pp" "src\constants.pp" "src\decentralized.pp" "src\filter.pp" "src\flightFSM.pp" "src\formation.pp" "src\mathex.pp" "src\message.pp" "src\trajectory.pp" 
	-$(RM) "src\DSP2833x_ADC_cal.obj" "src\DSP2833x_CpuTimers.obj" "src\DSP2833x_DefaultIsr.obj" "src\DSP2833x_GlobalVariableDefs.obj" "src\DSP2833x_I2C.obj" "src\DSP2833x_PieCtrl.obj" "src\DSP2833x_PieVect.obj" "src\DSP2833x_SysCtrl.obj" "src\DSP2833x_Xintf.obj" "src\DSP28xxx_CodeStartBranch_FLASH.obj" "src\DSP28xxx_SectionCopy_nonBIOS.obj" "src\DSP28xxx_usDelay.obj" "src\Fml.obj" "src\HMC5983.obj" "src\Hal.obj" "src\MS5803-01BA.obj" "src\car.obj" "src\constants.obj" "src\decentralized.obj" "src\filter.obj" "src\flightFSM.obj" "src\formation.obj" "src\mathex.obj" "src\message.obj" "src\trajectory.obj" 
	-$(RM) "src\DSP2833x_ADC_cal.pp" "src\DSP28xxx_CodeStartBranch_FLASH.pp" "src\DSP28xxx_SectionCopy_nonBIOS.pp" "src\DSP28xxx_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

