set(CMAKE_OSX_SYSROOT "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk")
cmake_minimum_required(VERSION 3.8)
project(samc_freertos)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++98")

set(SOURCE_FILES
		lib/ASF/common/boards/board.h
		lib/ASF/common/utils/interrupt/interrupt_sam_nvic.c
		lib/ASF/common/utils/interrupt/interrupt_sam_nvic.h
		lib/ASF/common/utils/interrupt.h
		lib/ASF/common/utils/parts.h
		lib/ASF/common2/services/delay/sam0/systick_counter.c
		lib/ASF/common2/services/delay/sam0/systick_counter.h
		lib/ASF/common2/services/delay/delay.h
		lib/ASF/sam0/drivers/adc/adc.h
		lib/ASF/sam0/drivers/adc/adc_callback.h
		lib/ASF/sam0/drivers/adc/adc_sam_l_c/adc_feature.h
		lib/ASF/sam0/drivers/adc/adc_sam_l_c/adc.c
		lib/ASF/sam0/drivers/adc/adc_sam_l_c/adc_callback.c
		lib/ASF/sam0/drivers/can/can.h
		lib/ASF/sam0/drivers/can/can.c
		lib/ASF/sam0/drivers/dma/dma.h
		lib/ASF/sam0/drivers/dma/dma_crc.h
		lib/ASF/sam0/drivers/dma/dma.c
		lib/ASF/sam0/drivers/nvm/nvm.c
		lib/ASF/sam0/drivers/port/quick_start/qs_port_basic.h
		lib/ASF/sam0/drivers/port/port.c
		lib/ASF/sam0/drivers/port/port.h
		lib/ASF/sam0/drivers/sercom/usart/usart.c
		lib/ASF/sam0/drivers/sercom/usart/usart.h
		lib/ASF/sam0/drivers/sercom/usart/usart_interrupt.c
		lib/ASF/sam0/drivers/sercom/usart/usart_interrupt.h
		lib/ASF/sam0/drivers/sercom/i2c/i2c_common.h
		lib/ASF/sam0/drivers/sercom/i2c/i2c_slave.h
		lib/ASF/sam0/drivers/sercom/i2c/i2c_slave_interrupt.h
		lib/ASF/sam0/drivers/sercom/i2c/i2c_sam0/i2c_slave.c
		lib/ASF/sam0/drivers/sercom/i2c/i2c_sam0/i2c_slave_interrupt.c
		lib/ASF/sam0/drivers/sercom/sercom.c
		lib/ASF/sam0/drivers/sercom/sercom.h
		lib/ASF/sam0/drivers/sercom/sercom_interrupt.c
		lib/ASF/sam0/drivers/sercom/sercom_interrupt.h
		lib/ASF/sam0/drivers/sercom/sercom_pinout.h
		lib/ASF/sam0/drivers/tc/tc_sam_l_c/tc.c
		lib/ASF/sam0/drivers/tc/tc.h
		lib/ASF/sam0/drivers/tc/tc_interrupt.h
		lib/ASF/sam0/drivers/tc/tc_interrupt.c
		lib/ASF/sam0/drivers/system/clock/clock_samc20_c21/clock.c
		lib/ASF/sam0/drivers/system/clock/clock_samc20_c21/clock_config_check.h
		lib/ASF/sam0/drivers/system/clock/clock_samc20_c21/clock_feature.h
		lib/ASF/sam0/drivers/system/clock/clock_samc20_c21/gclk.c
		lib/ASF/sam0/drivers/system/clock/clock.h
		lib/ASF/sam0/drivers/system/clock/gclk.h
		lib/ASF/sam0/drivers/system/interrupt/system_interrupt_samc20_c21/system_interrupt_features.h
		lib/ASF/sam0/drivers/system/interrupt/system_interrupt.c
		lib/ASF/sam0/drivers/system/interrupt/system_interrupt.h
		lib/ASF/sam0/drivers/system/pinmux/quick_start/qs_pinmux_basic.h
		lib/ASF/sam0/drivers/system/pinmux/pinmux.c
		lib/ASF/sam0/drivers/system/pinmux/pinmux.h
		lib/ASF/sam0/drivers/system/power/power_sam_c/power.h
		lib/ASF/sam0/drivers/system/reset/reset_sam_c/reset.h
		lib/ASF/sam0/drivers/system/system.c
		lib/ASF/sam0/drivers/system/system.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/ac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/adc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/can.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/ccl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/dac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/divas.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/dmac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/dsu.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/eic.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/evsys.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/freqm.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/gclk.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/hmatrixb.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/mclk.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/mtb.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/nvmctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/osc32kctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/oscctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/pac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/pm.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/port.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/rstc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/rtc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/sdadc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/sercom.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/supc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/tal.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/tc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/tcc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/tsens.h
		lib/ASF/sam0/utils/cmsis/samc21/include/component/wdt.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/ac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/adc0.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/adc1.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/can0.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/can1.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/ccl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/dac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/divas.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/dmac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/dsu.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/eic.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/evsys.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/freqm.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/gclk.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/hmatrixhs.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/mclk.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/mtb.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/nvmctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/osc32kctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/oscctrl.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/pac.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/pm.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/port.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/rstc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/rtc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sdadc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom0.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom1.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom2.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom3.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom4.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/sercom5.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/supc.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tal.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tc0.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tc1.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tc2.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tc3.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tc4.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tcc0.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tcc1.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tcc2.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/tsens.h
		lib/ASF/sam0/utils/cmsis/samc21/include/instance/wdt.h
		lib/ASF/sam0/utils/cmsis/samc21/include/pio/samc21j18a.h
		lib/ASF/sam0/utils/cmsis/samc21/include/samc21.h
		lib/ASF/sam0/utils/cmsis/samc21/include/samc21j18a.h
		lib/ASF/sam0/utils/cmsis/samc21/source/gcc/startup_samc21.c
		lib/ASF/sam0/utils/cmsis/samc21/source/system_samc21.c
		lib/ASF/sam0/utils/cmsis/samc21/source/system_samc21.h
		lib/ASF/sam0/utils/header_files/io.h
		lib/ASF/sam0/utils/preprocessor/mrecursion.h
		lib/ASF/sam0/utils/preprocessor/mrepeat.h
		lib/ASF/sam0/utils/preprocessor/preprocessor.h
		lib/ASF/sam0/utils/preprocessor/stringz.h
		lib/ASF/sam0/utils/preprocessor/tpaste.h
		lib/ASF/sam0/utils/syscalls/gcc/syscalls.c
		lib/ASF/sam0/utils/compiler.h
		lib/ASF/sam0/utils/status_codes.h
		lib/ASF/thirdparty/CMSIS/Include/arm_math.h
		lib/ASF/thirdparty/CMSIS/Include/core_cm0plus.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/croutine.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/deprecated_definitions.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/event_groups.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/FreeRTOS.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/list.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/mpu_wrappers.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/portable.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/projdefs.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/queue.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/semphr.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/stack_macros.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/task.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include/timers.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM0/port.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM0/portmacro.h
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_1.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_2.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_3.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_4.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_5.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/croutine.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/event_groups.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/list.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/queue.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/tasks.c
		lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/timers.c
		lib/freemodbus-1.5.0/modbus/mb.c
		lib/freemodbus-1.5.0/modbus/functions/mbfunccoils.c
		lib/freemodbus-1.5.0/modbus/functions/mbfuncdiag.c
		lib/freemodbus-1.5.0/modbus/functions/mbfuncdisc.c
		lib/freemodbus-1.5.0/modbus/functions/mbfuncholding.c
		lib/freemodbus-1.5.0/modbus/functions/mbfuncinput.c
		lib/freemodbus-1.5.0/modbus/functions/mbfuncother.c
		lib/freemodbus-1.5.0/modbus/functions/mbutils.c
		lib/freemodbus-1.5.0/modbus/include/mb.h
		lib/freemodbus-1.5.0/modbus/include/mbconfig.h
		lib/freemodbus-1.5.0/modbus/include/mbframe.h
		lib/freemodbus-1.5.0/modbus/include/mbfunc.h
		lib/freemodbus-1.5.0/modbus/include/mbport.h
		lib/freemodbus-1.5.0/modbus/include/mbproto.h
		lib/freemodbus-1.5.0/modbus/include/mbutils.h
		lib/freemodbus-1.5.0/modbus/rtu/mbrtu.c
		lib/freemodbus-1.5.0/modbus/rtu/mbcrc.c
		lib/freemodbus-1.5.0/modbus/rtu/mbrtu.h
		lib/freemodbus-1.5.0/modbus/rtu/mbcrc.h
		lib/freemodbus-1.5.0/port/sam0/mport.h
		lib/freemodbus-1.5.0/port/sam0/portevent.c
		lib/freemodbus-1.5.0/port/sam0/portother.c
		lib/freemodbus-1.5.0/port/sam0/portserial.c
		lib/freemodbus-1.5.0/port/sam0/porttimer.c
		lib/utils/log.c
		lib/utils/log.h
		lib/utils/usart_module.h
		main.c
		config/asf.h
		config/conf_clocks.h
		config/conf_can.h
		config/conf_dma.h
		config/FreeRTOSConfig.h
		config/custom_board_template.h
		config/custom_board.h
		config/samc21_usbcan.h
		config/cerbo_gx_slcan_adc.h
		utils/board_setup.c
        utils/comm_interface_setup.c
        utils/comm_interface_setup.h
		utils/board_setup.h
		tasks/stack_task.c
		tasks/stack_task.h
		tasks/can_task.c
		tasks/can_task.h
		tasks/uart_methods.c
		tasks/uart_methods.h
		tasks/slcan.h
		tasks/adc_task.c
		tasks/adc_task.h
		canbus/usb.h
		canbus/usb.c
		tasks/i2c_vitual_eeprom.c
		tasks/i2c_vitual_eeprom.h
		tasks/adc_methods.c
		tasks/adc_methods.h
		tasks/can_methods.c
		tasks/can_methods.h
		utils/control_leds.c
		utils/control_leds.h)



include_directories(lib/ASF/common/boards)
include_directories(lib/ASF/common/utils)
include_directories(lib/ASF/common2/services/delay)
include_directories(lib/ASF/sam0/boards)
include_directories(lib/ASF/sam0/drivers/adc)
include_directories(lib/ASF/sam0/drivers/can)
include_directories(lib/ASF/sam0/drivers/dma)
include_directories(lib/ASF/sam0/drivers/can/module_config)
include_directories(lib/ASF/sam0/drivers/port)
include_directories(lib/ASF/sam0/drivers/sercom)
include_directories(lib/ASF/sam0/drivers/sercom/i2c)
include_directories(lib/ASF/sam0/drivers/sercom/usart)
include_directories(lib/ASF/sam0/drivers/system)
include_directories(lib/ASF/sam0/drivers/system/clock)
include_directories(lib/ASF/sam0/drivers/system/clock/clock_samc20_c21)
include_directories(lib/ASF/sam0/drivers/system/interrupt)
include_directories(lib/ASF/sam0/drivers/system/interrupt/system_interrupt_samc20_c21)
include_directories(lib/ASF/sam0/drivers/system/pinmux)
include_directories(lib/ASF/sam0/drivers/system/power)
include_directories(lib/ASF/sam0/drivers/system/power/power_sam_c)
include_directories(lib/ASF/sam0/drivers/system/reset)
include_directories(lib/ASF/sam0/drivers/system/reset/reset_sam_c)
include_directories(lib/ASF/sam0/drivers/tc)
include_directories(lib/ASF/sam0/drivers/nvm)
include_directories(lib/ASF/sam0/drivers/wdt)
include_directories(lib/ASF/sam0/utils)
include_directories(lib/ASF/sam0/utils/cmsis/samc21/include)
include_directories(lib/ASF/sam0/utils/cmsis/samc21/source)
include_directories(lib/ASF/sam0/utils/header_files)
include_directories(lib/ASF/sam0/utils/preprocessor)
include_directories(lib/ASF/thirdparty/CMSIS/Lib/GCC)
include_directories(lib/ASF/thirdparty/CMSIS/Include)
include_directories(lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/include)
include_directories(lib/ASF/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM0)
include_directories(lib/freemodbus-1.5.0/port/sam0)
include_directories(lib/freemodbus-1.5.0/modbus/include)
include_directories(lib/freemodbus-1.5.0/modbus/rtu)
include_directories(lib/utils)
include_directories(config)
include_directories(tasks)
include_directories(utils)
include_directories(sunspec)
include_directories(vebus)
include_directories(velib/inc)
include_directories(canbus)
include_directories(.)

file (STRINGS "sw_version.inc" SW_VERSION)
add_definitions(-D ARM_MATH_CM0PLUS=true)
add_definitions(-D BOARD=USER_BOARD)
add_definitions(-D CUSTOM_BOARD=CERBO_SLCAN_ADC)
add_definitions(-D __SAMC21G17A__)
add_definitions(-D USART_CALLBACK_MODE=true)
add_definitions(-D ADC_CALLBACK_MODE=true)
add_definitions(-D I2C_SLAVE_CALLBACK_MODE=true)
add_definitions(-D __FREERTOS__)
add_definitions(-D SYSTICK_MODE)
add_definitions(-D NDEBUG)
add_definitions(-D TC_ASYNC=true)
add_definitions(-D SW_VERSION=${SW_VERSION})

add_executable(src ${SOURCE_FILES})

add_custom_target(samc_freertos COMMAND make  -C ${samc_freertos_SOURCE_DIR}
		CLION_EXE_DIR=${PROJECT_BINARY_DIR} CROSS="/opt/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-" clean all)

add_custom_target(samc_freertos-clean COMMAND make -C ${samc_freertos_SOURCE_DIR}
		CLION_EXE_DIR=${PROJECT_BINARY_DIR} clean)