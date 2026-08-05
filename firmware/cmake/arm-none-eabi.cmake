# Cross-compilation toolchain for the STM32F405ZGT6 (Cortex-M4F, hard float).
#
# Usage:
#   cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         arm-none-eabi-size    CACHE FILEPATH "size")

# The compiler cannot link a hosted test binary, so skip the default probe.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(FC_MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT   "${FC_MCU_FLAGS} -ffunction-sections -fdata-sections")
# -fno-use-cxa-atexit: firmware never exits, so registering destructors for
# objects with static storage duration is pure overhead (and drags in the
# __cxa_atexit / __dso_handle machinery that -nostartfiles removes).
set(CMAKE_CXX_FLAGS_INIT "${FC_MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${FC_MCU_FLAGS} -Wl,--gc-sections")
