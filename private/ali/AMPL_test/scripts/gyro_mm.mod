
####### input definition

param N > 1 integer;
set SAMPLES = { 1..N };

param X := 1 integer;
param Y := 2 integer;
param Z := 3 integer;

set COORDS := { X, Y, Z };

param accelGain { COORDS, COORDS };
param accelOffset { COORDS };

param RAW_TIME    := 1 integer;
param RAW_ACCEL_X := 2 integer;
param RAW_ACCEL_Y := 3 integer;
param RAW_ACCEL_Z := 4 integer;
param RAW_GYRO_X  := 5 integer;
param RAW_GYRO_Y  := 6 integer;
param RAW_GYRO_Z  := 7 integer;

set   RAW_VARS = { RAW_TIME, RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z, RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z };

param samples { SAMPLES, RAW_VARS };

param ticks default 32768.0;

####### calculated constants 

param elapsedTime { s in SAMPLES } := 
	if s >= 2 then (samples[s, RAW_TIME] - samples[s-1, RAW_TIME]) / ticks;

param rawGyro  { s in SAMPLES, i in COORDS } :=
	if      i = X then samples[s, RAW_GYRO_X]
	else if i = Y then samples[s, RAW_GYRO_Y]
	else if i = Z then samples[s, RAW_GYRO_Z];

param rawAccel { s in SAMPLES, i in COORDS } :=
	if      i = X then samples[s, RAW_ACCEL_X]
	else if i = Y then samples[s, RAW_ACCEL_Y]
	else if i = Z then samples[s, RAW_ACCEL_Z];

param calibAccel { s in SAMPLES, i in COORDS } :=
	accelOffset[i] + sum { k in COORDS } accelGain[i, k] * rawAccel[s, k];

####### indexing and record description constants

param CALIBRATED_GYRO := 1 integer;
param ROT_DELTA       := 2 integer;
param ROT_UPDATE      := 3 integer;
param ROT_ERROR       := 4 integer;
param ROT_ORTHOGONAL  := 5 integer;
param ROT_CORRECTION  := 6 integer;
param ROTATION_MATRIX := 7 integer;

set VARS =    { CALIBRATED_GYRO } cross COORDS cross    {0}
	union { ROT_DELTA }       cross COORDS cross COORDS
	union { ROT_UPDATE }      cross COORDS cross COORDS
	union { ROT_ERROR }       cross    {0} cross    {0}
	union { ROT_ORTHOGONAL }  cross COORDS cross COORDS
	union { ROT_CORRECTION }  cross COORDS cross    {0}
	union { ROTATION_MATRIX } cross COORDS cross COORDS
;

#### Variable Lower bounds, Upper bounds, Initial estimates

param gyroGain_L { COORDS, COORDS };
param gyroGain_U { COORDS, COORDS };
param gyroGain_0 { COORDS, COORDS };

param gyroOffset_L { COORDS };
param gyroOffset_U { COORDS };
param gyroOffset_0 { COORDS };

####### parameters needed to compute initial rotation matrix after successful solve;

param sumLength_ default 0.0;
param vLength_ default 0.0;

param u_ { COORDS } default 0.0;
param v_ { COORDS } default 0.0;
param w_ { COORDS } default 0.0;
param R0_ { COORDS, COORDS } default 0.0;

####### recursive definition of goal

var gyroGain{i in COORDS, j in COORDS} >=gyroGain_L[i,j], <=gyroGain_U[i,j],:=gyroGain_0[i,j];

var gyroOffset{i in COORDS}  >=gyroOffset_L[i], <=gyroOffset_U[i],:=gyroOffset_0[i];

var variable{s in SAMPLES, (v,i,j) in VARS} =

if s = 1 then
	(if v = ROTATION_MATRIX and i = j then 1.0 else 0.0)

else if v = CALIBRATED_GYRO then 
	elapsedTime[s] * (gyroOffset[i] + sum { k in COORDS } gyroGain[i, k] * rawGyro[s, k]) *(if i=Y then -1 else 1)

else if v = ROT_DELTA then
       (if      i = X and j = Y then -variable[s, CALIBRATED_GYRO, Z, 0]
	else if i = X and j = Z then  variable[s, CALIBRATED_GYRO, Y, 0]
	else if i = Y and j = X then  variable[s, CALIBRATED_GYRO, Z, 0]
	else if i = Y and j = Z then -variable[s, CALIBRATED_GYRO, X, 0]
	else if i = Z and j = X then -variable[s, CALIBRATED_GYRO, Y, 0]
	else if i = Z and j = Y then  variable[s, CALIBRATED_GYRO, X, 0]
	else 1.0)

else if v = ROT_UPDATE then
	sum { k in COORDS } variable[s-1, ROTATION_MATRIX, i, k] * variable[s, ROT_DELTA, k, j]

else if v = ROT_ERROR then
	(sum { k in COORDS } variable[s, ROT_UPDATE, X, k] * variable[s, ROT_UPDATE, Y, k])/2.0

### R_x , R_y and R_z vectors

else if v = ROT_ORTHOGONAL and i = X then
	variable[s, ROT_UPDATE, X, j] - variable[s, ROT_ERROR, 0, 0] * variable[s, ROT_UPDATE, Y, j]

else if v = ROT_ORTHOGONAL and i = Y then
	variable[s, ROT_UPDATE, Y, j] - variable[s, ROT_ERROR, 0, 0] * variable[s, ROT_UPDATE, X, j]

else if v = ROT_ORTHOGONAL and i = Z and j = X then
	variable[s, ROT_ORTHOGONAL, X, Y] * variable[s, ROT_ORTHOGONAL, Y, Z] -
	variable[s, ROT_ORTHOGONAL, X, Z] * variable[s, ROT_ORTHOGONAL, Y, Y]
	
else if v = ROT_ORTHOGONAL and i = Z and j = Y then
	variable[s, ROT_ORTHOGONAL, X, Z] * variable[s, ROT_ORTHOGONAL, Y, X] -
	variable[s, ROT_ORTHOGONAL, X, X] * variable[s, ROT_ORTHOGONAL, Y, Z]

else if v = ROT_ORTHOGONAL and i = Z and j = Z then
	variable[s, ROT_ORTHOGONAL, X, X] * variable[s, ROT_ORTHOGONAL, Y, Y] -
	variable[s, ROT_ORTHOGONAL, X, Y] * variable[s, ROT_ORTHOGONAL, Y, X]

else if v = ROT_CORRECTION then
	(3.0 - sum { k in COORDS } variable[s, ROT_ORTHOGONAL, i, k] * variable[s, ROT_ORTHOGONAL, i, k]) / 2.0

else if v = ROTATION_MATRIX then
        variable[s, ROT_ORTHOGONAL, i, j] * variable[s, ROT_CORRECTION, i, 0]
;

var rotatedAccel {s in SAMPLES, i in COORDS } =
   sum { k in COORDS } variable[s, ROTATION_MATRIX, i, k] * calibAccel[s, k];

var accelSum { i in COORDS } =
  sum { s in SAMPLES } rotatedAccel[s, i];

maximize total:
	sum { i in COORDS } (accelSum[i]/N) * (accelSum[i]/N);
