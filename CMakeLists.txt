# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

if (DEFINED ENV{SdkRootDirPath})
	SET(SdkRootDirPath "$ENV{SdkRootDirPath}")
endif()

if (NOT DEFINED SdkRootDirPath)
	MESSAGE(FATAL_ERROR "SDK path could not be determined. Please update SdkRootDirPath value from your CMake presets file(s).")
endif()

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

# RESET FLAG VALUES
SET(CMAKE_ASM_FLAGS_DEBUG "")
SET(CMAKE_ASM_FLAGS_RELEASE "")
SET(CMAKE_C_FLAGS_DEBUG "")
SET(CMAKE_C_FLAGS_RELEASE "")
SET(CMAKE_CXX_FLAGS_DEBUG "")
SET(CMAKE_CXX_FLAGS_RELEASE "")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "")

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})

set(CMAKE_USER_MAKE_RULES_OVERRIDE "user_override.cmake")

project(nxpcup_official)
set(MCUX_SDK_PROJECT_NAME nxpcup_official)

include(${ProjDirPath}/flags.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} "${ProjDirPath}/CMSIS/tz_context.h"
"${ProjDirPath}/CMSIS/core_cm33.h"
"${ProjDirPath}/CMSIS/cmsis_version.h"
"${ProjDirPath}/CMSIS/cmsis_gcc.h"
"${ProjDirPath}/CMSIS/cmsis_compiler.h"
"${ProjDirPath}/CMSIS/m-profile/cmsis_gcc_m.h"
"${ProjDirPath}/CMSIS/m-profile/armv8m_mpu.h"
"${ProjDirPath}/CMSIS_driver/Include/Driver_I2C.h"
"${ProjDirPath}/CMSIS_driver/Include/Driver_Common.h"
"${ProjDirPath}/board/RTE_Device.h"
"${ProjDirPath}/board/pin_mux.h"
"${ProjDirPath}/board/pin_mux.c"
"${ProjDirPath}/board/peripherals.h"
"${ProjDirPath}/board/peripherals.c"
"${ProjDirPath}/board/hardware_init.c"
"${ProjDirPath}/board/clock_config.h"
"${ProjDirPath}/board/clock_config.c"
"${ProjDirPath}/board/board.h"
"${ProjDirPath}/board/board.c"
"${ProjDirPath}/board/app.h"
"${ProjDirPath}/component/uart/fsl_adapter_uart.h"
"${ProjDirPath}/component/uart/fsl_adapter_lpuart.c"
"${ProjDirPath}/device/system_MCXN947_cm33_core0.h"
"${ProjDirPath}/device/system_MCXN947_cm33_core0.c"
"${ProjDirPath}/device/MCXN947_cm33_core0_features.h"
"${ProjDirPath}/device/MCXN947_cm33_core0_COMMON.h"
"${ProjDirPath}/device/MCXN947_cm33_core0.h"
"${ProjDirPath}/device/fsl_device_registers.h"
"${ProjDirPath}/device/periph/PERI_WWDT.h"
"${ProjDirPath}/device/periph/PERI_WUU.h"
"${ProjDirPath}/device/periph/PERI_VREF.h"
"${ProjDirPath}/device/periph/PERI_VBAT.h"
"${ProjDirPath}/device/periph/PERI_UTICK.h"
"${ProjDirPath}/device/periph/PERI_USDHC.h"
"${ProjDirPath}/device/periph/PERI_USBPHY.h"
"${ProjDirPath}/device/periph/PERI_USBNC.h"
"${ProjDirPath}/device/periph/PERI_USBHSDCD.h"
"${ProjDirPath}/device/periph/PERI_USBHS.h"
"${ProjDirPath}/device/periph/PERI_USBDCD.h"
"${ProjDirPath}/device/periph/PERI_USB.h"
"${ProjDirPath}/device/periph/PERI_TSI.h"
"${ProjDirPath}/device/periph/PERI_TRDC.h"
"${ProjDirPath}/device/periph/PERI_SYSPM.h"
"${ProjDirPath}/device/periph/PERI_SYSCON.h"
"${ProjDirPath}/device/periph/PERI_SPC.h"
"${ProjDirPath}/device/periph/PERI_SMARTDMA.h"
"${ProjDirPath}/device/periph/PERI_SINC.h"
"${ProjDirPath}/device/periph/PERI_SEMA42.h"
"${ProjDirPath}/device/periph/PERI_SCT.h"
"${ProjDirPath}/device/periph/PERI_SCG.h"
"${ProjDirPath}/device/periph/PERI_S50.h"
"${ProjDirPath}/device/periph/PERI_RTC.h"
"${ProjDirPath}/device/periph/PERI_QDC.h"
"${ProjDirPath}/device/periph/PERI_PWM.h"
"${ProjDirPath}/device/periph/PERI_PUF.h"
"${ProjDirPath}/device/periph/PERI_POWERQUAD.h"
"${ProjDirPath}/device/periph/PERI_PORT.h"
"${ProjDirPath}/device/periph/PERI_PLU.h"
"${ProjDirPath}/device/periph/PERI_PKC.h"
"${ProjDirPath}/device/periph/PERI_PINT.h"
"${ProjDirPath}/device/periph/PERI_PDM.h"
"${ProjDirPath}/device/periph/PERI_OTPC.h"
"${ProjDirPath}/device/periph/PERI_OSTIMER.h"
"${ProjDirPath}/device/periph/PERI_OPAMP.h"
"${ProjDirPath}/device/periph/PERI_NPX.h"
"${ProjDirPath}/device/periph/PERI_MRT.h"
"${ProjDirPath}/device/periph/PERI_MAILBOX.h"
"${ProjDirPath}/device/periph/PERI_LP_FLEXCOMM.h"
"${ProjDirPath}/device/periph/PERI_LPUART.h"
"${ProjDirPath}/device/periph/PERI_LPTMR.h"
"${ProjDirPath}/device/periph/PERI_LPSPI.h"
"${ProjDirPath}/device/periph/PERI_LPI2C.h"
"${ProjDirPath}/device/periph/PERI_LPDAC.h"
"${ProjDirPath}/device/periph/PERI_LPCMP.h"
"${ProjDirPath}/device/periph/PERI_ITRC.h"
"${ProjDirPath}/device/periph/PERI_INTM.h"
"${ProjDirPath}/device/periph/PERI_INPUTMUX.h"
"${ProjDirPath}/device/periph/PERI_I3C.h"
"${ProjDirPath}/device/periph/PERI_I2S.h"
"${ProjDirPath}/device/periph/PERI_HPDAC.h"
"${ProjDirPath}/device/periph/PERI_GPIO.h"
"${ProjDirPath}/device/periph/PERI_GDET.h"
"${ProjDirPath}/device/periph/PERI_FREQME.h"
"${ProjDirPath}/device/periph/PERI_FMUTEST.h"
"${ProjDirPath}/device/periph/PERI_FMU.h"
"${ProjDirPath}/device/periph/PERI_FLEXSPI.h"
"${ProjDirPath}/device/periph/PERI_FLEXIO.h"
"${ProjDirPath}/device/periph/PERI_EWM.h"
"${ProjDirPath}/device/periph/PERI_EVTG.h"
"${ProjDirPath}/device/periph/PERI_ERM.h"
"${ProjDirPath}/device/periph/PERI_ENET.h"
"${ProjDirPath}/device/periph/PERI_EMVSIM.h"
"${ProjDirPath}/device/periph/PERI_EIM.h"
"${ProjDirPath}/device/periph/PERI_DMA.h"
"${ProjDirPath}/device/periph/PERI_DM.h"
"${ProjDirPath}/device/periph/PERI_DIGTMP.h"
"${ProjDirPath}/device/periph/PERI_CTIMER.h"
"${ProjDirPath}/device/periph/PERI_CRC.h"
"${ProjDirPath}/device/periph/PERI_CMC.h"
"${ProjDirPath}/device/periph/PERI_CDOG.h"
"${ProjDirPath}/device/periph/PERI_CAN.h"
"${ProjDirPath}/device/periph/PERI_CACHE64_POLSEL.h"
"${ProjDirPath}/device/periph/PERI_CACHE64_CTRL.h"
"${ProjDirPath}/device/periph/PERI_BSP32.h"
"${ProjDirPath}/device/periph/PERI_AHBSC.h"
"${ProjDirPath}/device/periph/PERI_ADC.h"
"${ProjDirPath}/drivers/fsl_spc.h"
"${ProjDirPath}/drivers/fsl_spc.c"
"${ProjDirPath}/drivers/fsl_reset.h"
"${ProjDirPath}/drivers/fsl_reset.c"
"${ProjDirPath}/drivers/fsl_pwm.h"
"${ProjDirPath}/drivers/fsl_pwm.c"
"${ProjDirPath}/drivers/fsl_port.h"
"${ProjDirPath}/drivers/fsl_lpuart.h"
"${ProjDirPath}/drivers/fsl_lpuart.c"
"${ProjDirPath}/drivers/fsl_lpi2c_edma.h"
"${ProjDirPath}/drivers/fsl_lpi2c_edma.c"
"${ProjDirPath}/drivers/fsl_lpi2c_cmsis.h"
"${ProjDirPath}/drivers/fsl_lpi2c_cmsis.c"
"${ProjDirPath}/drivers/fsl_lpi2c.h"
"${ProjDirPath}/drivers/fsl_lpi2c.c"
"${ProjDirPath}/drivers/fsl_lpflexcomm.h"
"${ProjDirPath}/drivers/fsl_lpflexcomm.c"
"${ProjDirPath}/drivers/fsl_inputmux_connections.h"
"${ProjDirPath}/drivers/fsl_inputmux.h"
"${ProjDirPath}/drivers/fsl_inputmux.c"
"${ProjDirPath}/drivers/fsl_gpio.h"
"${ProjDirPath}/drivers/fsl_gpio.c"
"${ProjDirPath}/drivers/fsl_edma_soc.h"
"${ProjDirPath}/drivers/fsl_edma_soc.c"
"${ProjDirPath}/drivers/fsl_edma_core.h"
"${ProjDirPath}/drivers/fsl_edma.h"
"${ProjDirPath}/drivers/fsl_edma.c"
"${ProjDirPath}/drivers/fsl_ctimer.h"
"${ProjDirPath}/drivers/fsl_ctimer.c"
"${ProjDirPath}/drivers/fsl_common_arm.h"
"${ProjDirPath}/drivers/fsl_common_arm.c"
"${ProjDirPath}/drivers/fsl_common.h"
"${ProjDirPath}/drivers/fsl_common.c"
"${ProjDirPath}/drivers/fsl_clock.h"
"${ProjDirPath}/drivers/fsl_clock.c"
"${ProjDirPath}/source/simple_movement.c"
"${ProjDirPath}/source/servo.c"
"${ProjDirPath}/source/semihost_hardfault.c"
"${ProjDirPath}/source/pixy.c"
"${ProjDirPath}/source/mcux_config.h"
"${ProjDirPath}/source/main.c"
"${ProjDirPath}/source/hbridge.c"
"${ProjDirPath}/source/esc.c"
"${ProjDirPath}/source/ticks.c"
"${ProjDirPath}/source/track.c"
"${ProjDirPath}/source/racing_line.c"
"${ProjDirPath}/source/driver.c"
"${ProjDirPath}/source/telemetry.c"
"${ProjDirPath}/include/race_config.h"
"${ProjDirPath}/include/ticks.h"
"${ProjDirPath}/include/track.h"
"${ProjDirPath}/include/racing_line.h"
"${ProjDirPath}/include/driver.h"
"${ProjDirPath}/include/telemetry.h"
"${ProjDirPath}/startup/startup_mcxn947_cm33_core0.c"
"${ProjDirPath}/startup/boot_multicore_slave.h"
"${ProjDirPath}/startup/boot_multicore_slave.c"
"${ProjDirPath}/utilities/fsl_memcpy.S"
"${ProjDirPath}/utilities/fsl_assert.h"
"${ProjDirPath}/utilities/fsl_assert.c"
"${ProjDirPath}/utilities/str/fsl_str.h"
"${ProjDirPath}/utilities/str/fsl_str.c"
"${ProjDirPath}/utilities/debug_console_lite/fsl_debug_console.h"
"${ProjDirPath}/utilities/debug_console_lite/fsl_debug_console.c"
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES SUFFIX ".axf")
endif()

if(${LIBRARY_TYPE} STREQUAL "REDLIB")
    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $ENV{ARMGCC_DIR}/redlib/include)
endif()

set(asm_include_directories "${ProjDirPath}/drivers"
"${ProjDirPath}/CMSIS"
"${ProjDirPath}/CMSIS/m-profile"
"${ProjDirPath}/device"
"${ProjDirPath}/device/periph"
"${ProjDirPath}/utilities"
"${ProjDirPath}/utilities/str"
"${ProjDirPath}/utilities/debug_console_lite"
"${ProjDirPath}/component/uart"
"${ProjDirPath}/CMSIS_driver/Include"
"${ProjDirPath}/board"
"${ProjDirPath}/source"
"${ProjDirPath}/include"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:ASM>:${asm_include_directories}>)

set(c_include_directories "${ProjDirPath}/drivers"
"${ProjDirPath}/CMSIS"
"${ProjDirPath}/CMSIS/m-profile"
"${ProjDirPath}/device"
"${ProjDirPath}/device/periph"
"${ProjDirPath}/utilities"
"${ProjDirPath}/utilities/str"
"${ProjDirPath}/utilities/debug_console_lite"
"${ProjDirPath}/component/uart"
"${ProjDirPath}/CMSIS_driver/Include"
"${ProjDirPath}/board"
"${ProjDirPath}/source"
"${ProjDirPath}/include"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:C>:${c_include_directories}>)

set(cpp_include_directories "")

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${cpp_include_directories}>)

set(include_files "")

target_precompile_headers(${MCUX_SDK_PROJECT_NAME} PRIVATE ${include_files})

set(link_directories "")

set(link_libraries "")

target_link_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE ${link_directories})
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES} ${link_libraries})
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	add_custom_command(
		TARGET ${MCUX_SDK_PROJECT_NAME}
		POST_BUILD
		COMMAND $ENV{ARMGCC_DIR}/bin/arm-none-eabi-size "$<TARGET_FILE_NAME:${MCUX_SDK_PROJECT_NAME}>" || true
	)
endif()

