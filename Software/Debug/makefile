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
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CAN_MITM.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CAN_MITM.list \

OBJCOPY_HEX += \
CAN_MITM.hex \

OBJCOPY_BIN += \
CAN_MITM.bin \


# All Target
all: CAN_MITM.elf secondary-outputs

# Tool invocations
CAN_MITM.elf: $(OBJS) $(USER_OBJS) C:\Users\damo\Dropbox\Leaf\CAN_MITM\Software\CAN_MITM\STM32F105RBTX_FLASH.ld
	arm-none-eabi-gcc -o "CAN_MITM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\damo\Dropbox\Leaf\CAN_MITM\Software\CAN_MITM\STM32F105RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CAN_MITM.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CAN_MITM.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CAN_MITM.list"
	@echo 'Finished building: $@'
	@echo ' '

CAN_MITM.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "CAN_MITM.hex"
	@echo 'Finished building: $@'
	@echo ' '

CAN_MITM.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CAN_MITM.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
