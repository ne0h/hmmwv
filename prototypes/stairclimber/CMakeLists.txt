cmake_minimum_required (VERSION 2.8)
project (stairclimber)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(../../ros_workspace/src/hmmwv/src/)
add_executable(stairclimber
	../../ros_workspace/src/hmmwv/src/joystick.cpp
	../../ros_workspace/src/hmmwv/src/joystickevent.cpp
	enginecontroller.cpp
)
target_link_libraries(stairclimber SDL)
