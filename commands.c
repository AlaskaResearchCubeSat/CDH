#include <msp430.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctl.h>
#include <terminal.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include <crc.h>
#include <commandLib.h>
#include "CDH.h"
#include <Error.h>
#include "CDH_errors.h"

int statCmd(char **argv,unsigned short argc){
  int i;
  unsigned char *ptr;
  //Read status
  ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_GET_STAT_CMD,0);
  //wait a bit so status can be returned
  ctl_timeout_wait(ctl_get_current_time()+3*1024);
  //check status
  if(system_stat.flags&STAT_ALL_VALID==STAT_ALL_VALID){
    printf("All subsystems reported status\r\n");
  }else{
    if(!(system_stat.flags&STAT_EPS_VALID)){
      printf("No Status Info for EPS\r\n");
    }
    if(!(system_stat.flags&STAT_LEDL_VALID)){
      printf("No Status Info for LEDL\r\n");
    }
    if(!(system_stat.flags&STAT_ACDS_VALID)){
      printf("No Status Info for ACDS\r\n");
    }
    if(!(system_stat.flags&STAT_COMM_VALID)){
      printf("No Status Info for COMM\r\n");
    }
    if(!(system_stat.flags&STAT_IMG_VALID)){
      printf("No Status Info for IMG\r\n");
    }
  }
 //       ptr=(unsigned char*)&system_stat;
 //     for(i=0;i<sizeof(STAT_PACKET);i++){
 //       printf("0x%02X ",ptr[i]);
 //       if(i%15==14){
 //         printf("\r\n");
 //       }
 //     }
 //     printf("\r\n");
  //send status
  ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_SEND_STAT_CMD,0);
  return 0;
}
//Turn on CDH
int onCmd(char *argv[],unsigned short argc){
  //output lower four bits CDH address (Ox15) to P7 LED's

  P7OUT=BIT2|BIT0;

  //Perhaps should set a register here that says we are commanded on.
  
  printf("CDH On.  Check LEDs: 0bxxxx0101\r\n");
}

//Turn off CDH
int offCmd(char *argv[],unsigned short argc){
   //output lower four bits CDH address (Ox15) to P7 LED's
  P7OUT=0;

  //Perhaps should set a register here that says we are commanded off.
  
  printf("CDH Off.  Check LEDs: 0bxxxx0000\r\n");
}

//Retreive status CDH
int statusCmd(char *argv[],unsigned short argc){

  int i;
   //flash lower four bits CDH address (Ox15) to P7 LED's 10 times
   P7OUT=BIT2|BIT0;
   for (i=0;i<10;i++){
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=~(BIT2|BIT0);
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=(BIT2|BIT0);
   }
  //Need to send back status through terminal.
  
  P7OUT=BIT2|BIT0; //finish present CDH address
  printf("CDH On.  Check LEDs: flashing 0bxxxx0101 - 0bxxxx1010\r\n");
}

//reset a MSP430 on command
int resetCmd(char **argv,unsigned short argc){
  //force user to pass no arguments to prevent unwanted resets
  if(argc!=0){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print reset message
  puts("Initiating reset\r\n");
  //wait for UART buffer to empty
  while(UCA1_CheckBusy());
  //write to WDTCTL without password causes PUC
  WDTCTL=0;
  //Never reached due to reset
  puts("Error : Reset Failed!\r");
  return 0;
}

int beaconCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"on")){
      beacon_on=1;
    }else if(!strcmp(argv[1],"off")){
      beacon_on=0;
    }else{
      printf("Error : Unknown argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  printf("Beacon : %s\r\n",beacon_on?"on":"off");
  return 0;
}


//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    ARC_COMMANDS,CTL_COMMANDS,ERROR_COMMANDS,ARC_ASYNC_PROXY_COMMAND,
                    {"reset","\r\n\t""Reset the MSP430",resetCmd},
                    {"OnCDH","[bgnd|stop]\r\n\t""Command ON CDH",onCmd},
                    {"OffCDH","port [port ...]\r\n\t""Command OFF CDH",offCmd},
                    {"StatusCDH","\r\n\t""Get CDH status - flash LED's",statusCmd},
                    {"stat","\r\n\t""Get status from all subsystems.", statCmd},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                   //end of list

                   {NULL,NULL,NULL}};

