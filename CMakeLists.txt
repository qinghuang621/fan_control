cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME pwm_snail)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_fric.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_led_tilt.c
   ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_modbus.c
   ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_pulse.c
   ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_spi.c
   ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_imu_pwm.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/MahonyAHRS.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/ins_task.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/controller/pid.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/BMI088driver.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/BMI088Middleware.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/ist8310driver.c
   ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/ist8310driver_middleware.c
   ${CMAKE_CURRENT_SOURCE_DIR}/Src/i2c.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/components/controller
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# Add FreeRTOS kernel and heap sources
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    # queue.c：CAN 接收帧走 FreeRTOS 队列（HAL_CAN_RxFifo0MsgPendingCallback
    # → xQueueSendFromISR，CanTask/ModbusTask → xQueueReceive），与 running 一致。
    # 之前只用任务通知，故未编入；改成队列方案后必须加。
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)
target_include_directories(FreeRTOS PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    FreeRTOS
)

# 生成 hex 文件，CubeProgrammer 烧录用
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex  $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating HEX and BIN files"
)
