/** Copyright (c) 2011, University of Szeged
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* - Neither the name of University of Szeged nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Author: Andras Biro
*/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <getopt.h>
#include <string.h>

int main(int argc, char **argv){
  #define M_ON 0
  #define M_OFF 1
  #define M_SPIKE 2
  #define M_HOLE 3
  char path[255]="";
  int portnum=-1, mode=-1;
  struct option long_options[] =
    {
      {"device",    required_argument, 0, 'd'},
      {"num",    required_argument, 0, 'n'},
      {"mode",    required_argument, 0, 'm'},
      {"help",    no_argument,       0, 'h'},
      {0, 0, 0, 0}
    };
  char c;
  int option_index;
  while ((c = getopt_long (argc, argv, "hd:n:m:",long_options,&option_index)) != -1){
    switch (c)
    {
      case 'h':
        printf("Usage: mcp2200gpio <arguments>\
        \narguments:\
        \n-h, --help        Prints this help\
        \n-d, --device      Selects the HID device to use\
        \n-n, --number      Selects the GPIO port to manipulate (0..7)\
        \n-m, --mode        Selects the GPIO manipulation mode (on/off/spike/hole)\
        \n example:  mcp2200gpio -d /dev/usb/hiddev0 -n 4 -m hole\n");
        exit(1);
      break;
      case 'd':
        strncpy(path,optarg,255);
      break;
      case 'n':
        portnum=atoi(optarg);
      break;
      case 'm':
        if(strcmp(optarg,"on")==0){
          mode=M_ON;
        } else if(strcmp(optarg,"off")==0){
          mode=M_OFF;
        } else if(strcmp(optarg,"spike")==0){
          mode=M_SPIKE;
        } else if(strcmp(optarg,"hole")==0){
          mode=M_HOLE;
        } else {
          perror("Invalid mode. Valid modes are: on/off/spike/hole\n");
          exit(1);
        }
      break;
    }
  }
  if(strcmp(path,"")==0){
    fprintf(stderr,"No device selected\n");
    exit(1);
  }
  if(portnum==-1){
    fprintf(stderr,"No GPIO port selected\n");
    exit(1);
  }
  if(mode==-1){
    fprintf(stderr,"No mode selected\n");
    exit(1);
  }
  
 
  struct hiddev_report_info response;  
  struct hiddev_usage_ref_multi command;
  int fd=-1;
  fd=open(path, O_RDONLY);
  if(fd<0){
    fprintf(stderr,"Can't open device: %s\n",path);
    exit(1);
  }
  response.report_type=HID_REPORT_TYPE_OUTPUT;
  response.report_id=HID_REPORT_ID_FIRST;
  response.num_fields=1;
  command.uref.report_type=HID_REPORT_TYPE_OUTPUT;
  command.uref.report_id=HID_REPORT_ID_FIRST;
  command.uref.field_index=0;
  command.uref.usage_index=0;
  command.num_values=16;
  command.values[0]=8;
  
  if(mode==M_ON||mode==M_SPIKE){
     command.values[11]=1<<portnum;
     command.values[12]=0;
  }else{
    command.values[11]=0;
    command.values[12]=1<<portnum;
  }
  ioctl(fd,HIDIOCSUSAGES, &command); 
  ioctl(fd,HIDIOCSREPORT, &response);
  if(mode==M_HOLE||mode==M_SPIKE){
    if(mode==M_HOLE){
      command.values[11]=1<<portnum;
      command.values[12]=0;
    }else if(mode==M_SPIKE){
      command.values[11]=0;
      command.values[12]=1<<portnum;
    }
    ioctl(fd,HIDIOCSUSAGES, &command); 
    ioctl(fd,HIDIOCSREPORT, &response);  
  }
  close(fd);
  return 0;
}