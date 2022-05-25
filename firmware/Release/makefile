################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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
LSS += \
temp-control.lss \

FLASH_IMAGE += \
temp-control.hex \

EEPROM_IMAGE += \
temp-control.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: temp-control.elf secondary-outputs

# Tool invocations
temp-control.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,temp-control.map -mmcu=atmega128 -o "temp-control.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

temp-control.lss: temp-control.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S temp-control.elf  >"temp-control.lss"
	@echo 'Finished building: $@'
	@echo ' '

temp-control.hex: temp-control.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex temp-control.elf  "temp-control.hex"
	@echo 'Finished building: $@'
	@echo ' '

temp-control.eep: temp-control.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex temp-control.elf  "temp-control.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: temp-control.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega128 temp-control.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) temp-control.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
