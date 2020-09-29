################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
op_phoenix_glove.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
op_phoenix_glove.list \

OBJCOPY_BIN += \
op_phoenix_glove.bin \


# All Target
all: op_phoenix_glove.elf secondary-outputs

# Tool invocations
op_phoenix_glove.elf: $(OBJS) $(USER_OBJS) /Users/gkao/STM32CubeIDE/workspace_1.4.0/op_phoenix_glove/STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "op_phoenix_glove.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/Users/gkao/STM32CubeIDE/workspace_1.4.0/op_phoenix_glove/STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="op_phoenix_glove.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

op_phoenix_glove.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "op_phoenix_glove.list"
	@echo 'Finished building: $@'
	@echo ' '

op_phoenix_glove.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "op_phoenix_glove.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
