project(app.ledblink)
cmake_minimum_required(VERSION 2.8)

set (CMAKE_BUILD_TYPE Release)

########################################################################
enable_language(C ASM)
set (CROSS_COMPILER /home/thanhbinh/App/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-)
set (CMAKE_C_FLAGS  "")
set (CMAKE_ASM_FLAGS "")
set (CMAKE_ASM_FLAGS_RELEASE "")
set (CMAKE_C_FLAGS_RELEASE   "-O2")
set (CMAKE_C_COMPILER "${CROSS_COMPILER}gcc")
set (CMAKE_ASM_COMPILER "${CROSS_COMPILER}as")
set (CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set (CP ${CROSS_COMPILER}objcopy)
set (OD ${CROSS_COMPILER}objdump)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections -fno-builtin")
set (CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -Wl,-gc-sections")

set (MCFLAGS  "-mcpu=cortex-m4 -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork")
set (DEFINEDS "-DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DSTM32F407xx -DSTM32CUBEF4 -DOS_UCOS -DHSE_VALUE=8000000")
set (LDSCRIPT "${PROJECT_SOURCE_DIR}/BSP/stm32f4_flash.ld")

set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${MCFLAGS} ${DEFINEDS}")
set (CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCFLAGS}")
set (CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -T ${LDSCRIPT} --specs=nosys.specs --specs=nano.specs ${MCFLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/app.map")

set (OS_PATH ${PROJECT_SOURCE_DIR}/../../os/uCOS)
set (HW_PATH ${PROJECT_SOURCE_DIR}/../../hw/STM32CubeF4)
set (KERNEL_PATH ${PROJECT_SOURCE_DIR}/../../kernel)

include_directories(.)
include_directories(BSP)
include_directories(DRV)
include_directories(OS)

include_directories(${OS_PATH}/uC-CPU)
include_directories(${OS_PATH}/uC-CPU/ARM-Cortex-M4/GNU)
include_directories(${OS_PATH}/uC-LIB)
include_directories(${OS_PATH}/uC-LIB/Ports/ARM-Cortex-M4/GNU)
include_directories(${OS_PATH}/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU)
include_directories(${OS_PATH}/uCOS-III/Source)

include_directories(${HW_PATH}/Drivers/CMSIS/Device/ST/STM32F4xx/Include)
include_directories(${HW_PATH}/Drivers/CMSIS/Include)
include_directories(${HW_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc)
include_directories(${HW_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)

include_directories(${KERNEL_PATH}/include)
include_directories(${KERNEL_PATH}/include/sys)

aux_source_directory( .   SRC_LIST)
aux_source_directory( BSP SRC_LIST)
aux_source_directory( DRV SRC_LIST)
aux_source_directory( OS  SRC_LIST)
aux_source_directory( ${KERNEL_PATH}/src SRC_LIST )
aux_source_directory( ${KERNEL_PATH}/src/drv SRC_LIST )
aux_source_directory( ${KERNEL_PATH}/src/sys SRC_LIST )
aux_source_directory( ${OS_PATH}/uC-CPU SRC_LIST)
aux_source_directory( ${OS_PATH}/uC-CPU/ARM-Cortex-M4/GNU SRC_LIST)
aux_source_directory( ${OS_PATH}/uC-LIB SRC_LIST)
aux_source_directory( ${OS_PATH}/uC-LIB/Ports/ARM-Cortex-M4/GNU SRC_LIST)
aux_source_directory( ${OS_PATH}/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU SRC_LIST)
aux_source_directory( ${OS_PATH}/uCOS-III/Source SRC_LIST)
aux_source_directory( ${HW_PATH}/Drivers/STM32F4xx_HAL_Driver/Src SRC_LIST)

#set (ASM_LIST "BSP/startup.s")
#set (ASM_LIST "${OS_PATH}/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU/os_cpu_a.s")
#set (ASM_LIST "${ASM_LIST} ${OS_PATH}/uC-CPU/ARM-Cortex-M4/GNU/cpu_a.s")
#set (ASM_LIST "${ASM_LIST} ${OS_PATH}/uC-LIB/Ports/ARM-Cortex-M4/GNU/lib_mem_a.s")

add_executable (${PROJECT_NAME} ${SRC_LIST} BSP/startup.s ${OS_PATH}/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU/os_cpu_a.s ${ASM_LIST} ${OS_PATH}/uC-CPU/ARM-Cortex-M4/GNU/cpu_a.s ${ASM_LIST} ${OS_PATH}/uC-LIB/Ports/ARM-Cortex-M4/GNU/lib_mem_a.s)

#set_source_files_properties(${PROJECT_NAME}_BIN PROPERTIES_GENERATED true)
#add_custom_target(${PROJECT_NAME}_BIN COMMAND ${CP} -O binary ${PROJECT_BINARY_DIR}/${PROJECT_NAME} ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_BIN)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CP} -O binary ${PROJECT_BINARY_DIR}/${PROJECT_NAME} ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

set_source_files_properties(flash PROPERTIES_GENERATED true)
add_custom_target(flash COMMAND sudo st-flash --reset write ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin 0x08000000)


