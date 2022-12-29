# Adapted from: https://www.kuon.ch/post/2018-07-11-avr-cmake/

cmake_minimum_required(VERSION 3.11)

# Set to build for simulation
set(BUILD_FOR_SIM OFF)

# Use AVR GCC toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)

# Project name
project(ATtinyRTC C ASM)

# AVR Chip Configuration
set(F_CPU 16000000UL)

# CPU, you can find the list here:
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
set(MCU attiny85)

# The programmer to use, read avrdude manual for list
set(PROG_TYPE usbasp-clone)

# AVR Fuses, must be in concordance with your hardware and F_CPU
# http://eleccelerator.com/fusecalc/fusecalc.php?chip=attiny85&LOW=EE&HIGH=5F&EXTENDED=FF&LOCKBIT=FF
set(L_FUSE 0xee)
set(H_FUSE 0x5f)
set(E_FUSE 0xff)
set(LOCK_BIT 0xff)

# Pass defines to compiler
add_definitions(
	-DF_CPU=${F_CPU}
)

# mmcu MUST be passed to both the compiler and linker, this handles the linker
set(CMAKE_EXE_LINKER_FLAGS "-mmcu=${MCU} -Wl,-Map,${PROJECT_NAME}.map")

add_compile_options(
	-mmcu=${MCU} # MCU
	-std=gnu99 # C99 standard
	-Os # optimize
	-Wall # enable warnings
	-Wno-main
	-Wundef
	-pedantic
	-Wstrict-prototypes
	-Werror
	-Wfatal-errors
	-Wl,--relax,--gc-sections
	-g
	-gdwarf-2
	-funsigned-char # a few optimizations
	-funsigned-bitfields
	-fpack-struct
	-fshort-enums
	-ffunction-sections
	-fdata-sections
	-fno-split-wide-types
	-fno-tree-scev-cprop
)

# Create one target
add_executable(${PROJECT_NAME}
	main.c
)

# Prepare for simulation
find_package(PkgConfig REQUIRED)
pkg_check_modules(SIMAVR simavr)
if(SIMAVR_FOUND)
	# Add definitions and includes for simulator
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DBUILD_FOR_SIM)
	target_include_directories(${PROJECT_NAME} PUBLIC ${SIMAVR_INCLUDE_DIRS})
endif()

# Rename the output to .elf as we will create multiple files
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME}.elf)

# Report size
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND avr-size -C --mcu=${MCU} ${PROJECT_NAME}.elf
)

# Dissassemble binary to produce listing
add_custom_target(list ALL avr-objdump -S -w -l ${PROJECT_NAME}.elf > ${PROJECT_NAME}.lst DEPENDS ${PROJECT_NAME})

# Strip binary for upload
add_custom_target(strip ALL avr-strip -o ${PROJECT_NAME}_stripped.elf ${PROJECT_NAME}.elf DEPENDS ${PROJECT_NAME})

# Transform binary into hex file, we ignore the eeprom segments in the step (and the .mmcu segment added for simulation)
add_custom_target(hex ALL avr-objcopy -R .eeprom -R .fuse -R .mmcu -O ihex ${PROJECT_NAME}_stripped.elf ${PROJECT_NAME}.hex DEPENDS strip)

# Transform binary into hex file, this is the eeprom part (empty if you don't
# use eeprom static variables)
add_custom_target(eeprom avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 -O ihex ${PROJECT_NAME}_stripped.elf ${PROJECT_NAME}.eep DEPENDS strip)

# Upload the firmware with avrdude
add_custom_target(upload avrdude -v -c ${PROG_TYPE} -p ${MCU} -U flash:w:${PROJECT_NAME}.hex DEPENDS hex)

# Upload the eeprom with avrdude
add_custom_target(upload_eeprom avrdude -v -c ${PROG_TYPE} -p ${MCU}  -U eeprom:w:${PROJECT_NAME}.eep DEPENDS eeprom)

# Burn fuses
add_custom_target(fuses avrdude -c ${PROG_TYPE} -p ${MCU}  -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m -U lock:w:${LOCK_BIT}:m)

# Clean extra files
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${PROJECT_NAME}.hex;${PROJECT_NAME}.eep;${PROJECT_NAME}.map;${PROJECT_NAME}.lst;${PROJECT_NAME}_stripped.elf")
