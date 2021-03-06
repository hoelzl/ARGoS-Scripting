#!/bin/bash
C_INCLUDE_DIRS="-I/usr/include -I/opt/local/include"
SWIG_INCLUDE_DIRS="$C_INCLUDE_DIRS -I/usr/include/c++/4.2.1"
DEFINES="-DTIXML_USE_TICPP"
LUA_DEFINES="-DSWIG_LUA"
LINK="-llua -largos2_simulator -largos2_common_utility -largos2_simulator_sensors -largos2_simulator_actuators -largos2_common_control_interface -L/opt/local/lib -L/usr/lib/argos2"
#CC=/opt/local/bin/c++-mp-4.7
CC=c++

swig -c++ -lua -external-runtime argos-lua-wrapper.h
swig -v $SWIG_INCLUDE_DIRS $DEFINES $LUA_DEFINES -c++ -lua -oh argos-lua-wrapper.h -o argos-lua-wrapper.cpp argos-lua.i 
# echo "Compiling with " `$CC --version`
# $CC $C_INCLUDE_DIRS $DEFINES $LINK -o argos-lua-wrapper.o argos-lua-wrapper.cpp
# $CC $C_INCLUDE_DIRS $DEFINES -c argos-lua-wrapper.cpp
echo "Done"
