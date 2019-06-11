cmake_minimum_required(VERSION 2.8)

PROJECT("hmmwv")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
INCLUDE_DIRECTORIES(gamepad)
FILE(GLOB GamepadSourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/gamepad/*.cpp)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    INCLUDE_DIRECTORIES(/usr/local/include)
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

#
# console
#
ADD_EXECUTABLE(console enginecontrol_console.cpp ${GamepadSourceFiles})
TARGET_LINK_LIBRARIES(console SDL pthread ncurses)

#
# testrunner
#
FILE(GLOB Tests ${CMAKE_CURRENT_SOURCE_DIR}/*Test.cpp )
ADD_EXECUTABLE(testrunner testrunner.cpp ${Tests})
TARGET_LINK_LIBRARIES(testrunner cppunit)
FOREACH(test ${Tests})
        GET_FILENAME_COMPONENT(TestName ${test} NAME_WE)
        ADD_TEST(${TestName} UnitTester ${TestName})
ENDFOREACH()
