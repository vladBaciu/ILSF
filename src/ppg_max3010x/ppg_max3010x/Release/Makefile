################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../i2c_driver \
../i2c_driver_c/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../main.c \
../i2c_driver_c/i2c_driver.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
main.o \
i2c_driver_c/i2c_driver.o

OBJS_AS_ARGS +=  \
main.o \
i2c_driver_c/i2c_driver.o

C_DEPS +=  \
main.d \
i2c_driver_c/i2c_driver.d

C_DEPS_AS_ARGS +=  \
main.d \
i2c_driver_c/i2c_driver.d

OUTPUT_FILE_PATH +=ppg_max3010x.elf

OUTPUT_FILE_PATH_AS_ARGS +=ppg_max3010x.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./main.o: .././main.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.31
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.31\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega4809  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega4809__ -DNDEBUG  -I"../i2c_driver"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

i2c_driver_c/i2c_driver.o: ../i2c_driver_c/i2c_driver.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.31
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.31\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega4809  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega4809__ -DNDEBUG  -I"../i2c_driver"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking:  XC8 C Linker : 2.31
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.31\bin\xc8-cc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mcpu=ATmega4809  -Wl,-Map="ppg_max3010x.map" -funsigned-char -funsigned-bitfields -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax --memorysummary,memoryfile.xml  
	@echo Finished building target: $@
	"C:\Program Files\Microchip\xc8\v2.31\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "ppg_max3010x.elf" "ppg_max3010x.hex"
	"C:\Program Files\Microchip\xc8\v2.31\bin\avr-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "ppg_max3010x.elf" "ppg_max3010x.eep" || exit 0
	"C:\Program Files\Microchip\xc8\v2.31\bin\avr-objdump.exe" -h -S "ppg_max3010x.elf" > "ppg_max3010x.lss"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "ppg_max3010x.elf" "ppg_max3010x.hex" "ppg_max3010x.eep" "ppg_max3010x.map" "ppg_max3010x.usersignatures" "ppg_max3010x.a"  "ppg_max3010x.lss"
	