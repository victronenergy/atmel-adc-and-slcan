#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

include lib/include.mk

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = .

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m0plus

# Target part: none, sam3n4 or sam4l4aa
PART = samc21g18a

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = cmake-build-debug/samc21_slcan_adc.elf
TARGET_SRAM = cmake-build-debug/samc21_slcan_adc.elf

# List of C source files.
CSRCS = ${LIB_CSRCS}\
		main.c \
		tasks/stack_task.c	\
		tasks/can_task.c	\
		tasks/adc_task.c \
		tasks/uart_commands.c \
		utils/board_setup.c \
		utils/comm_interface_setup.c \
		canbus/usb.c	\
		

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
		${LIB_INC_PATH} \
		config \
		utils \
		tasks \
		canbus	\
		


#       ASF/sam0/boards/samc21_xplained_pro                    
# Additional search paths for libraries.
LIB_PATH =  \
       lib/ASF/thirdparty/CMSIS/Lib/GCC

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM0l_math                                

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = lib/ASF/sam0/utils/linker_scripts/samc21/gcc/samc21j18a_flash.ld
LINKER_SCRIPT_SRAM  = lib/ASF/sam0/utils/linker_scripts/samc21/gcc/samc21j18a_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = lib/ASF/sam0/boards/samc21_xplained_pro/debug_scripts/gcc/samc21_xplained_pro_flash.gdb
DEBUG_SCRIPT_SRAM  = lib/ASF/sam0/boards/samc21_xplained_pro/debug_scripts/gcc/samc21_xplained_pro_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ARM_MATH_CM0PLUS=true			\
       -D BOARD=USER_BOARD				\
       -D CUSTOM_BOARD=CERBO_SLCAN_ADC	\
       -D __SAMC21G18A__				\
       -D USART_CALLBACK_MODE=true		\
       -D ADC_CALLBACK_MODE=true		\
       -D __FREERTOS__	\
		-D SYSTICK_MODE	\
		-D NDEBUG		\
		-D TC_ASYNC=true

# Extra flags to use when linking
LDFLAGS = \

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 
