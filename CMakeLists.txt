cmake_minimum_required(VERSION 3.0)

project(firmware LANGUAGES C CXX ASM VERSION 1.0.0)
set(BIN_NAME "motologger_firmware")

#---- SOURCES -----------------------------------------------------------------#

set(SOURCES
    include/fs.hpp
    include/ffconf.h
    include/main.hpp
    include/stm32f4xx_hal_conf.h
    include/utils.hpp
    include/drivers/lsm9ds1.hpp
    include/motologger/application.hpp
    include/motologger/can_frame.hpp
    include/motologger/sample.hpp
    include/motologger/session.hpp
    include/motologger/moto.hpp
    include/motologger/moto/yamaha_mt07.hpp
    include/hw.hpp
    src/hw.cpp
    src/hw_init.cpp
    src/hw_isr.cpp
    src/hw_io.cpp
    src/main.cpp
    src/fs.cpp
    src/system_stm32f4xx.c
    src/syscalls.cpp
    src/drivers/lsm9ds1.cpp
    src/app/utils.cpp
    src/app/application.cpp
    src/app/session.cpp
    src/app/moto/yamaha_mt07.cpp)

#---- BUILD OPTIONS -----------------------------------------------------------#

include_directories(include)
set(CMAKE_C_FLAGS           "${CMAKE_C_FLAGS} -std=c11 -pedantic -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG     "-g")
set(CMAKE_C_FLAGS_RELEASE   "-s -O3 -flto")
set(CMAKE_CXX_STANDARD      17)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -pedantic -Wall -Wextra -Wno-register")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")

#---- OUTPUT ------------------------------------------------------------------#

include(cmake/cubemx.cmake)

target_compile_definitions(${BIN_NAME}
  PRIVATE YAMAHA_MT07_2014)
