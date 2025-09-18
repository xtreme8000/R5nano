include(FetchContent)
FetchContent_Declare(libopencm3
  GIT_REPOSITORY https://github.com/libopencm3/libopencm3
  GIT_TAG master
)
FetchContent_MakeAvailable(libopencm3)

# create a target to build libopencm3 -- only for the target we need
add_custom_target(libopencm3 make TARGETS=stm32/f1 WORKING_DIRECTORY ${libopencm3_SOURCE_DIR})

# Create a specific CPU target with the appropriate options etc
add_library(stm32 STATIC IMPORTED)
set_property(TARGET stm32 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${libopencm3_SOURCE_DIR}/include)
set_property(TARGET stm32 PROPERTY IMPORTED_LOCATION ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32f1.a)
add_dependencies(stm32 libopencm3)
target_link_directories(stm32 INTERFACE ${libopencm3_SOURCE_DIR}/lib)

target_compile_definitions(stm32 INTERFACE -DSTM32F1)

set(COMPILE_OPTIONS
    -nostartfiles
    -fno-common
    -mcpu=cortex-m3
    -mthumb
    -mfloat-abi=soft
)

add_compile_options(${COMPILE_OPTIONS})
add_link_options(${COMPILE_OPTIONS})
