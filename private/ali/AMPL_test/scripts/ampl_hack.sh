#!/bin/bash
# input should be record.csv...
rm -f ./gyro.dat ./rotmat.txt ./gyro.log
./gyro_test in gyro.dat 
./ampl gyro_mm.mod gyro.dat gyro_mm.run >gyro.log 
./gyro_test out rotmat.txt 



