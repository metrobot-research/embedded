cmake_minimum_required(VERSION 2.8)

find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

list(APPEND ALL_TARGET_LIBRARIES ${CURSES_LIBRARIES})