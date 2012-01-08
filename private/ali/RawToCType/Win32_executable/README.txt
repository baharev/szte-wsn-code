
The SD card appears as a drive. You do not have to take it out of the mote, the
computer recognizes it when the mote is docked.

You have to first sdc -format the SD card before using it. It will take a long
while.

You can sdc -download the new records. It will create the correspondig .csv 
files in the current working directory. Be careful and do not change the .mdb
and .rdb files.

The columns in the .csv files are as follows:

time stamp in ticks, 32768 ticks/second
counter, used for debugging, ignore it
acceleration x
acceleration y
acceleration z
gyro x
gyro y
gyro z
voltage
temperature
