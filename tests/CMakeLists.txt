set(BIN_NAME "motologger_firmware")

set(TEST_SOURCES
    ../src/app/session.cpp
    ../src/app/utils.cpp
    fatfs_stubs.cpp
    system_stubs.cpp
    session_test.cpp)

configure_file(../cmake/gtest.cmake.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download)
if (result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif ()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download)
if (result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif ()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                  ${CMAKE_BINARY_DIR}/googletest-build)
add_executable(${BIN_NAME}_test ${TEST_SOURCES})
target_include_directories(${BIN_NAME}_test
    PRIVATE ../include
    PRIVATE ../stm32cube/drivers/cmsis/include
    PRIVATE ../stm32cube/drivers/bsp
    PRIVATE ../stm32cube/drivers/hal/include
    PRIVATE ../stm32cube/drivers/hal/include/Legacy
    PRIVATE ../stm32cube/middlewares/fatfs/src/drivers
    PRIVATE ../stm32cube/middlewares/fatfs/src)
target_compile_definitions(${BIN_NAME}_test
    PRIVATE "__weak=__attribute__((weak))"
    PRIVATE "__packed=__attribute__((__packed__))"
    PRIVATE STM32F412Zx
    PRIVATE USE_HAL_DRIVER)
target_link_libraries(${BIN_NAME}_test gtest_main)
enable_testing()
add_test(NAME ${BIN_NAME}_test
          COMMAND ${BIN_NAME}_test
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
