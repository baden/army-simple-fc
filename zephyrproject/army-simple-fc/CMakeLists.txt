# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_VERBOSE_MAKEFILE ON)

#set(BOARD army_simple_fc_v1_0)

#list(APPEND ZEPHYR_EXTRA_MODULES
#    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/zephyr-atmodem
#)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(army_simple_fc LANGUAGES C)

#add_subdirectory(src)
#include_directories(include)
target_sources(app PRIVATE src/main.c)


zephyr_cc_option(-Wfatal-errors)
