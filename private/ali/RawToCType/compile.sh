#!/bin/bash
g++ -c -Os -fdata-sections -ffunction-sections -Wall \
BinaryFileFormatter.cpp \
BlockChecker.cpp \
BlockDevice.cpp \
BlockIterator.cpp \
Console.cpp \
demangle.cpp \
DeviceFormatter.cpp \
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
main.cpp

g++ -static \
BinaryFileFormatter.o \
BlockChecker.o \
BlockDevice.o \
BlockIterator.o \
Console.o \
demangle.o \
DeviceFormatter.o \
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
main.o \
-o sd_card -Wl,-static,--gc-sections

strip --strip-all -R.comment -R.note sd_card

upx --overlay=strip --lzma sd_card
