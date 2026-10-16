cmake_minimum_required(VERSION 3.16)
project(fc_firmware CXX)

# ---------------------------------------------------------------------------
# Host build.
#
# This CMakeLists builds the portable core (maths, control, protocol codecs,
# state machines) and its unit tests for the HOST machine. No board, no
# cross-compiler and no vendor HAL are required.
#
# Anything that touches STM32 registers lives under src/platform/ and is
# excluded here; it is compiled only by the separate cross build.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  if(CMAKE_CROSSCOMPILING)
    set(CMAKE_BUILD_TYPE Release)
  else()
    set(CMAKE_BUILD_TYPE Debug)
  endif()
endif()

# The portable core: identical sources in both builds.
file(GLOB_RECURSE FC_CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER FC_CORE_SOURCES EXCLUDE REGEX ".*/src/platform/.*")
list(FILTER FC_CORE_SOURCES EXCLUDE REGEX ".*/src/main\\.cpp$")

add_library(fc_core STATIC ${FC_CORE_SOURCES})
target_include_directories(fc_core PUBLIC include)

if(CMAKE_CROSSCOMPILING)
  # ---------------------------------------------------------------------
  # Target build: STM32F405ZGT6 firmware image.
  #
  # -Wconversion is not applied here. Register-level code casts between widths
  # constantly by nature, and the noise would bury real warnings. The portable
  # core is still compiled with it in the host build.
  # ---------------------------------------------------------------------
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)

  file(GLOB_RECURSE FC_PLATFORM_SOURCES CONFIGURE_DEPENDS src/platform/*.cpp)

  add_executable(fc_firmware.elf ${FC_PLATFORM_SOURCES} src/main.cpp)
  target_include_directories(fc_firmware.elf PRIVATE include src)
  target_link_libraries(fc_firmware.elf PRIVATE fc_core)

  set(FC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/platform/stm32f405zg.ld)
  target_link_options(fc_firmware.elf PRIVATE
    -T${FC_LINKER_SCRIPT}
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=fc_firmware.map
    -Wl,--print-memory-usage
  )
  target_link_libraries(fc_firmware.elf PRIVATE m)
  set_target_properties(fc_firmware.elf PROPERTIES LINK_DEPENDS ${FC_LINKER_SCRIPT})

  add_custom_command(TARGET fc_firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:fc_firmware.elf> fc_firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:fc_firmware.elf> fc_firmware.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:fc_firmware.elf>
    COMMENT "Generating fc_firmware.bin / .hex and reporting size"
  )

else()
  # ---------------------------------------------------------------------
  # Host build: unit tests. No board, no cross-compiler required.
  # ---------------------------------------------------------------------
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

  enable_testing()

  # Software-in-the-loop simulator: the portable core closed around a
  # simulated quadrotor through the same HAL mocks the unit tests use, plus
  # the host-side sensor models (sim/sensors) the driver tests run against.
  add_library(fc_sil_lib STATIC sim/sil/quad_plant.cpp sim/sil/sil_harness.cpp)
  target_include_directories(fc_sil_lib PUBLIC sim/sil sim/sensors test)
  target_link_libraries(fc_sil_lib PUBLIC fc_core)

  add_executable(fc_sil sim/sil/fc_sil_main.cpp)
  target_link_libraries(fc_sil PRIVATE fc_sil_lib)

  file(GLOB FC_TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_src ${FC_TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE fc_core fc_sil_lib)
    target_include_directories(${test_name} PRIVATE test)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
