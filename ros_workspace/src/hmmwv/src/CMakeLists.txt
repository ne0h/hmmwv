cmake_minimum_required(VERSION 2.8)

PROJECT("hmmwv")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
INCLUDE_DIRECTORIES(gamepad)
FILE(GLOB GamepadSourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/gamepad/*.cpp)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    INCLUDE_DIRECTORIES(/usr/local/include)
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

ADD_EXECUTABLE(console enginecontrol_console.cpp ${GamepadSourceFiles})
TARGET_LINK_LIBRARIES(console SDL pthread ncurses)
