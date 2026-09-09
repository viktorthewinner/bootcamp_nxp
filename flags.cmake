if(NOT DEFINED FPU)
	set(FPU "-mfpu=fpv5-sp-d16 -mfloat-abi=hard")
endif()

if(${LIBRARY_TYPE} STREQUAL "REDLIB")
	set(SPECS "-specs=redlib.specs")
elseif(${LIBRARY_TYPE} STREQUAL "NEWLIB_NANO")
	set(SPECS "--specs=nano.specs")
endif()

if(NOT DEFINED DEBUG_CONSOLE_CONFIG)
	set(DEBUG_CONSOLE_CONFIG "-DSDK_DEBUGCONSOLE=0")
endif()

set(CMAKE_ASM_FLAGS_DEBUG " \
    ${CMAKE_ASM_FLAGS_DEBUG} \
    ${FPU} \
    -mcpu=cortex-m33 \
    -mthumb \
")

set(CMAKE_C_FLAGS_DEBUG " \
    ${CMAKE_C_FLAGS_DEBUG} \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
    -std=gnu99 \
    -DCPU_MCXN947VDF \
    -DCPU_MCXN947VDF_cm33 \
    -DCPU_MCXN947VDF_cm33_core0 \
    -DMCUXPRESSO_SDK \
    -DMCUX_META_BUILD \
    -DMCXN947_cm33_core0_SERIES \
    -DCR_INTEGER_PRINTF \
    -DPRINTF_FLOAT_ENABLE=0 \
    -D__MCUXPRESSO \
    -D__USE_CMSIS \
    -DDEBUG \
    -O0 \
    -fno-common \
    -fmerge-constants \
    -g3 \
    -mcpu=cortex-m33 -ffunction-sections -fdata-sections -fno-builtin -imacros \"${ProjDirPath}/source/mcux_config.h\" \
    -fstack-usage \
    -mcpu=cortex-m33 \
    -mthumb \
")

set(CMAKE_CXX_FLAGS_DEBUG " \
    ${CMAKE_CXX_FLAGS_DEBUG} \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
    -O0 \
    -fno-common \
    -fmerge-constants \
    -g3 \
    -Wall \
    -fstack-usage \
    -mcpu=cortex-m33 \
    -mthumb \
")

set(CMAKE_EXE_LINKER_FLAGS_DEBUG " \
    ${CMAKE_EXE_LINKER_FLAGS_DEBUG} \
    ${FPU} \
    ${SPECS} \
    -nostdlib \
    -Xlinker \
    -no-warn-rwx-segments \
    -Xlinker \
    -Map=output.map \
    -Xlinker \
    --gc-sections \
    -Xlinker \
    -print-memory-usage \
    -Xlinker \
    --sort-section=alignment \
    -Xlinker \
    --cref \
    -mcpu=cortex-m33 \
    -mthumb \
    -T\"${ProjDirPath}/nxpcup_official_Debug.ld\" \
    -L\"${ProjDirPath}/linkerScripts/Debug\" \
")
