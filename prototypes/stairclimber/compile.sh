#!/bin/bash
g++ -std=c++0x enginecontroller.cpp joystick.cpp joystickevent.cpp `sdl-config --libs` -o enginecontroller
