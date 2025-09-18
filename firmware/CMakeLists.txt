cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE cmake/toolchain.cmake)

# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

project(firmware VERSION 0.1.0 LANGUAGES C ASM)

include(cmake/stm32.cmake)

add_executable(firmware
  src/main.c
)
set_target_properties(firmware PROPERTIES C_STANDARD 99)

stm32_add_linker_script(firmware PRIVATE basic.stm32.ld)
target_link_libraries(firmware stm32)

add_custom_command(
  TARGET firmware
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware> $<TARGET_FILE_BASE_NAME:firmware>.bin
)

add_custom_command(
  TARGET firmware
  POST_BUILD
  COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:firmware>
)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE config)
set(FREERTOS_PORT GCC_ARM_CM3)
set(FREERTOS_HEAP 1)

include(FetchContent)
FetchContent_Declare(freertos
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel
  GIT_TAG V11.2.0
)
FetchContent_MakeAvailable(freertos)

target_link_libraries(firmware freertos_kernel)
