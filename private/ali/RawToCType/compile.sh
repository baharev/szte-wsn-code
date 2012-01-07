#!/bin/bash
g++ -c -Os -fdata-sections -ffunction-sections -Wall \
Action.cpp \
BinaryFileFormatter.cpp \
BlockChecker.cpp \
BlockDevice.cpp \
BlockIterator.cpp \
Compare.cpp \
Console.cpp \
Copy.cpp \
demangle.cpp \
DeviceFormatter.cpp \
Dispatcher.cpp \
FileAsBlockDevice.cpp \
Header.cpp \
Line.cpp \
MoteRegistrar.cpp \
Sample.cpp \
SDCard.cpp \
SDCardImpl.cpp \
Tracker.cpp \
Utility.cpp \
Win32BlockDevice.cpp \
Win32DeviceHelper.cpp \
Win32DriveFormatter.cpp \
Writer.cpp \
ZeroDevice.cpp \
main.cpp

g++ -static \
Action.o \
BinaryFileFormatter.o \
BlockChecker.o \
BlockDevice.o \
BlockIterator.o \
Compare.o \
Console.o \
Copy.o \
demangle.o \
DeviceFormatter.o \
Dispatcher.o \
FileAsBlockDevice.o \
Header.o \
Line.o \
MoteRegistrar.o \
Sample.o \
SDCard.o \
SDCardImpl.o \
Tracker.o \
Utility.o \
Win32BlockDevice.o \
Win32DeviceHelper.o \
Win32DriveFormatter.o \
Writer.o \
ZeroDevice.o \
main.o \
-o sdc.exe -Wl,-static,--gc-sections

strip --strip-all -R.comment -R.note sdc.exe

upx --overlay=strip --lzma sdc.exe
