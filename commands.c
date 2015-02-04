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
#include <math.h>
#include "CDH_errors.h"
#include "mag.h"

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
        
        
enum{COR_X_BASE=0,COR_Y_BASE=2,COR_Z_BASE=4,COR_PLUS_OFFSET=0,COR_MINUS_OFFSET=1};

const char * const (cor_axis_names[])={"X+","X-","Y+","Y-","Z+","Z-"};
    
int mag_sample_stop(void* buf){
    unsigned char *ptr;

    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SAMPLE_STOP;
    //send packet
    return BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0,BUS_I2C_SEND_FOREGROUND);
}

int mag_sample_start(void* buf,unsigned short time,unsigned char count){
    unsigned char *ptr;
    int resp;
    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SAMPLE_START;
    //set time MSB
    *ptr++=time>>8;
    //set time LSB
    *ptr++=time;
    //set count
    *ptr++=count;
    //send packet
    resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,4,0,BUS_I2C_SEND_FOREGROUND);
    //return response
    return resp;
}

int mag_sample_single(void* buf){
    unsigned char *ptr;
    int resp;
    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SINGLE_SAMPLE;
    //send packet
    resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0,BUS_I2C_SEND_FOREGROUND);
    //return response
    return resp;
}

int mag_test_mode(void *buf,unsigned char state){
    unsigned char *ptr;
    int resp;
    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=state;
    //send packet
    resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0,BUS_I2C_SEND_FOREGROUND);
    //return response
    return resp;
}
        
int magCmd(char **argv,unsigned short argc){
    const char *term;
    int single=0,print_all=0,single_axis=-1;
    unsigned short output_type=HUMAN_OUTPUT;
    unsigned short time=32768,count=0;
    int i,j,res,timeout=0;
    CTL_EVENT_SET_t e;
    unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
    //parse arguments
    for(i=1;i<=argc;i++){
        if(!strcmp("single",argv[i])){
            single=1;
        }else if(!strcmp("sdata",argv[i])){
            //print_sdata=1;
        }else if(!strcmp("all",argv[i])){
            print_all=1;
            //print_sdata=1;
        }else if(!strcmp("raw",argv[i])){
            //print_raw=1;
        }else{                        
            //look for symbolic axis name
            for(j=0;j<6;j++){
                if(!strcmp(argv[i],cor_axis_names[j])){
                    single_axis=j;
                    break;
                }
            }
            if(single_axis==-1){
                printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
                return -1;
            }
        }
    }
    //set line terminator
    term=(output_type==MACHINE_OUTPUT)?"\t":"\r\n";
    //put LEDL in test mode so it sends data to CDH and not ACDS
    mag_test_mode(buff,MAG_TEST_MODE_ON);
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
    //check if reading a single sample
    if(single){
        //read single measurement
        res=mag_sample_single(buff);
    }else{
        //start reading magnetometer data
        res=mag_sample_start(buff,time,count);
    }
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
    //clear event
    ctl_events_set_clear(&mag_evt,0,MAG_EVT_COMMAND_SENSOR_READ);
    if(single){
        do{
            //wait for measurement
            e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&mag_evt,MAG_EVT_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,2048);
            if(!e){
                //send packet again
                res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);     
                //increse timeout count
                timeout++;
            }
        }while(!e && timeout<5);
        if(!e){
            printf("Error : timeout while waiting for sensor data\r\n");
            return 2;
        }
        if(single_axis==-1){
            //print out individual sensor data
            //print seperator
            if(output_type==HUMAN_OUTPUT){   
                printf("========================================================================================\r\n");
            }
            //loop through all SPBs
            for(i=0;i<6;i++){
                //check for valid measurements
                if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1))){
                        printf(" %s : % i % i%s",cor_axis_names[i],magData.meas[i].c.a,magData.meas[i].c.b,term);
                }else if(print_all){
                    //print error
                    printf(" %s : ### ###%s",cor_axis_names[i],term);
                }
            }
            if(output_type==MACHINE_OUTPUT){   
                printf("\r\n");
            }
        }else{
            printf("% i\t% i\r\n",magData.meas[single_axis].c.a,magData.meas[single_axis].c.b);
        }
    }else{
        printf("Reading Magnetometer, press any key to stop\r\n");
        //run while no keys pressed
        while(async_CheckKey()==EOF){
            //wait for data from LEDL
            e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&mag_evt,MAG_EVT_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,1800);
            //check if data was received
            if(e&MAG_EVT_COMMAND_SENSOR_READ){
                if(single_axis==-1){
                    //print out individual sensor data
                    //print seperator
                    if(output_type==HUMAN_OUTPUT){   
                        printf("========================================================================================\r\n");
                    }
                    //loop through all SPBs
                    for(i=0;i<6;i++){
                        //check for valid measurements
                        if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1))){
                            printf(" %s : % i % i%s",cor_axis_names[i],magData.meas[i].c.a,magData.meas[i].c.b,term);
                        }else if(print_all){
                            //print error
                            printf(" %s : ### ###%s",cor_axis_names[i],term);
                        }
                    }
                    //print seperator                   
                    if(output_type==HUMAN_OUTPUT){   
                        printf("========================================================================================\r\n");
                    }
                    if(output_type==MACHINE_OUTPUT){   
                        printf("\r\n");
                    }
                }else{
                    printf("% i\t% i\r\n",magData.meas[single_axis].c.a,magData.meas[single_axis].c.b);
                }
                //message recived, reduce timeout count
                if(timeout>-10){
                    timeout--;
                }
            }else{
                //message timeout, increase timeout count
                timeout+=4;
                //check if too many time outs have happened
                if(timeout>20){
                    //print error 
                    printf("Error : timeout while waiting for sensor data\r\n");            
                    //exit loop
                    break;
                }
                //if not aborting, print a warning
                printf("Warning : timeout while waiting for sensor data\r\n"); 
            }
        }
        //send stop sample command
        res=mag_sample_stop(buff);
        //check result
        if(res<0){
            printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
            //return error
            return 1;
        }
        //clear event flag
        ctl_events_set_clear(&mag_evt,0,MAG_EVT_COMMAND_SENSOR_READ);
        //wait for straggalers
        ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&mag_evt,MAG_EVT_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,900);
    }
    return 0;
}

int CDH_print_cmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : too many arguments\r\n");
    return 1;
  }
  if(argc==1){
    if(!strcmp("on",argv[1])){
      CDH_print=1;
    }else if(!strcmp("off",argv[1])){
      CDH_print=0;
    }else{
      printf("Error : unrecognized argument \"%s\"\r\n",argv[1]);
      return 2;
    }
  }
  printf("CDH printing is : %s\r\n",CDH_print?"on":"off");
  return 0;
}

int mag_test_mode_Cmd(char **argv,unsigned short argc){
  unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
  unsigned char state;
  int res;
  if(argc!=1){
    printf("Error : %s requires only 1 argument\r\n",argv[0]);
    return -1;
  }

  if(!strcmp("on",argv[1])){
    state=MAG_TEST_MODE_ON;
  }else if(!strcmp("off",argv[1])){
    state=MAG_TEST_MODE_OFF;
  }else{
    printf("Error : unknown argument \"%s\"\r\n",argv[1]);
    return -2;
  }
  res=mag_test_mode(buff,state);
  if(res!=RET_SUCCESS){
    printf("Error sending packet : %s\r\n",BUS_error_str(res));
  }else{
    printf("Command sent successfully!!\r\n");
  }
}

int burnCmd(char **argv,unsigned short argc){
    float burn_time=1,wait_time=0;
    unsigned long burn_delay,wait_delay;
    char *end;
    if(argc!=0){
        if(argc>2){
            printf("Error : too many arguments\r\n");
            return -1;
        }
        if(argc>=2){
            wait_time=strtof(argv[2],&end);
            //check result
            if(!isfinite(wait_time)){
                printf("Error : failed to read time \"%s\" %f returned\r\n",argv[2],wait_time);
                return -2;
            }
            //check for extra chars
            if(*end!=NULL){
                printf("Error : unknown suffix \"%s\" while parsing \"%s\" \r\n",end,argv[2]);
                return -3;
            }
        }
        burn_time=strtof(argv[1],&end);
        //check result
        if(!isfinite(burn_time)){
            printf("Error : failed to read time \"%s\" %f returned\r\n",argv[1],burn_time);
            return -2;
        }
        //check for extra chars
        if(*end!=NULL){
            printf("Error : unknown suffix \"%s\" while parsing \"%s\" \r\n",end,argv[1]);
            return -3;
        }
    }
    //check if we need to wait
    if(wait_time>10*1/1024.0){
        //calculate delay
        wait_delay=(wait_time*1024)+0.5;
        //recalculate time
        wait_time=wait_delay/1024;
        //print message with time
        printf("Waiting %.0fs before burn\r\n",wait_time);
        //wait
        ctl_timeout_wait(ctl_get_current_time()+wait_delay);
    }
    //calculate delay in clocks
    burn_delay=(burn_time*1024)+0.5;
    //check if delay is too short
    if(burn_delay<3){
        burn_delay=3;
    }
    //recalculate time
    burn_time=burn_delay/1024.0;
    //print out delay time
    printf("Activating burn circuit for %.1f secconds\r\n",burn_time);
    //turn on resistor
    burn_on();
    //delay for specified time
    ctl_timeout_wait(ctl_get_current_time()+burn_delay);
    //turn off resistor
    burn_off();
    //print completion message
    printf("Burn Activation complete!\r\n");
    return 0;
}


int mode_Cmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    if(!strcmp("testing",argv[1])){
      //testing mode
      USB_power=1;
    }else if(!strcmp("flight",argv[1])){
      //flight mode
      USB_power=0;
    }else{
      printf("Error : unknown argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  printf("mode = %s\r\n",USB_power?"testing":"flight");
  return 0;
}


int power_Cmd(char **argv,unsigned short argc){
  unsigned char cmd,addr;
  unsigned char buf[BUS_I2C_HDR_LEN+0+BUS_I2C_CRC_LEN],*ptr;
  int resp;
  //check number of arguments
  if(argc!=2){
    printf("Error : %s requires 2 aruments but %i given\r\n",argv[0],argc);
    return -1;
  }
  //get command
  if(!strcmp("on",argv[1])){
    cmd=CMD_SUB_ON;
  }else if(!strcmp("off",argv[1])){
    cmd=CMD_SUB_OFF;
  }else{
    printf("Error : unknown argument %s\r\n",argv[1]);
    return -2;
  }
  //get address
  addr=getI2C_addr(argv[2],0,busAddrSym);
  //check if address was found
  if(addr==0xFF){
    return -3;
  }
  //setup command memory
  ptr=BUS_cmd_init(buf,cmd);
  //send packet
  resp=BUS_cmd_tx(addr,buf,0,0,BUS_I2C_SEND_FOREGROUND);
  //check result
  if(resp!=RET_SUCCESS){
    printf("Error sending packet : %s\r\n",BUS_error_str(resp));
  }else{
    printf("Command sent successfully!!\r\n");
  }
}

int LEDLtm_Cmd(char **argv,unsigned short argc){
  if(argc==1){
  if(!strcmp("on",argv[1])){
      //Turn off MOSFET to power LEDL from EPS
      P6OUT&=~BIT6;
      //clear interrupt pin
      BUS_int_clear(BIT0);
    }else if(!strcmp("off",argv[1])){
      //Turn on MOSFET to power LEDL from EPS
      P6OUT|=BIT6;
      //set interrupt pin
      BUS_int_set(BIT0);
    }else{
      printf("Error : unknown argument %s\r\n",argv[1]);
      return -1;
    }
  }else if(argc>1){
    printf("Error : too many arguments\r\n");
    return -2;
  }
  //print status
  printf("LEDL MOSFET : %s\r\n",(P6OUT&BIT6)?"on":"off");
  printf("LEDL interrupt : %s\r\n",(P1DIR&BIT0)?"on":"off");
  return 0;
}

//EPS commands
enum{EPS_ADC_CMD=0,EPS_STAT_CMD=1,EPS_PDM_OFF_CMD=2,EPS_VERSION_CMD=4,EPS_HEATER_CMD=5,EPS_WATCHDOG_CMD=128};

int EPS_cmd(char **argv,unsigned short argc){
  unsigned char addr,chan,pdm,heat;
  unsigned char buf[BUS_I2C_HDR_LEN+2+BUS_I2C_CRC_LEN],*ptr;
  unsigned long num;
  char *end;
  int res,i;
  unsigned short val;
  if(argc<1){
    printf("Error : %s requires at least 1 argument\r\n");
    return -1;
  }
  //setup packet 
  ptr=BUS_cmd_init(buf,CMD_EPS_SEND);
  if(!strcmp(argv[1],"pdm")){
    for(i=2,pdm=0;i<=argc;i++){
      if(!strcmp(argv[i],"3.3V")){
        //reset 3.3V rail
        pdm|=BIT2;
      }else if(!strcmp(argv[i],"5V")){
        //reset 5V rail
        pdm|=BIT1;
      }else if(!strcmp(argv[i],"vbat")){
        //reset battery bus rail
        pdm|=BIT0;
      }else{
        printf("Error : unknown bus \"%s\"\r\n",argv[i]);
        return -7;
      }
    }
    //create packet
    ptr[0]=EPS_PDM_OFF_CMD;
    ptr[1]=pdm;
    //print message
    printf("Sending PDM off command with 0x%02X\r\n",pdm);
  }else if(!strcmp(argv[1],"heater")){
    if(argc!=2){
      printf("Error : %s command %s requires 2 arguments but %i given\r\n",argv[0],argv[1],argc);
      return -1;
    }
    if(!strcmp(argv[1],"off")){
      //turn heater off
      heat=0x01;
    }else if(!strcmp(argv[1],"on")){
      //turn heater on
      heat=0x00;
    }else{
      printf("Error : unknown heater value \"%\"\r\n",argv[1]);
      return -5;
    }
    //create packet
    ptr[0]=EPS_HEATER_CMD;
    ptr[1]=heat;
    //print message
    printf("Sending heater %s command\r\n",heat?"off":"on");
  }else if(!strcmp(argv[1],"wdt")){
    if(argc!=2){
      printf("Error : %s command %s requires 1 arguments but %i given\r\n",argv[0],argv[1],argc);
      return -1;
    }
    //create packet
    ptr[0]=EPS_WATCHDOG_CMD;
    ptr[1]=0;
    //print message
    printf("Sending watchdog command\r\n");
  }else{
    //attempt to parse numeric value
    num=strtoul(argv[1],&end,0);
    //check if anything worked
    if(end==argv[1]){
      printf("Error : unknown command %s\r\n",argv[1]);
      return -3;
    }
    //check for second argument
    if(argc!=2){
      printf("Error : numeric commands require 2 arguments\r\n");
      return -7;
    }
    //check for suffix
    if(*end!='\0'){
      printf("Error : unknown suffix \"%s\" for \"%s\"\r\n",end,argv[1]);
      return -5;
    }
    //check range
    if(num>0xFF){
      printf("Error : argument %lu is too large\r\n",num);
      return -6;
    }
    //store in packet
    ptr[0]=num;
    //parse argument
    num=strtoul(argv[2],&end,0);
    //check if anything worked
    if(end==argv[1]){
      printf("Error : could not parse argument \"%s\"\r\n",argv[2]);
      return -3;
    }
    //check for suffix
    if(*end!='\0'){
      printf("Error : unknown suffix \"%s\" for \"%s\"\r\n",end,argv[2]);
      return -5;
    }
    //check range
    if(num>0xFF){
      printf("Error : argument %lu is too large\r\n",num);
      return -6;
    }
    //store in packet
    ptr[1]=num;
    //print sending info
    printf("Sending 0x%02X, 0x%02X\r\n",ptr[0],ptr[1]);
    //send
  }   
  res=BUS_cmd_tx(BUS_ADDR_LEDL,buf,2,0,BUS_I2C_SEND_FOREGROUND);
  //check result
  if(res!=RET_SUCCESS){
    printf("Error sending command : %s\r\n",BUS_error_str(res));
    return 2;
  }
  //print success
  printf("Request sent successfully\r\n");
  return 0;
}

//reset pins for subsystems
enum  {COMM_RST_PIN=BIT0,LEDL_RST_PIN=BIT1,ACDS_RST_PIN=BIT2,IMG_RST_PIN=BIT3};

int hard_reset_Cmd(char **argv,unsigned short argc){
  int i,hold=0;
  unsigned long num;
  char *end;
  char reset=0;
  if(argc==0){
    printf("Error : %s requires one or more arguments\r\n",argv[0]);
    return -1;
  }
  for(i=1;i<=argc;i++){
    if(!strcmp(argv[i],"hold")){
      hold=1;
    }else if(!strcmp(argv[i],"LEDL")){
      //set LEDL reset pin
      reset|=LEDL_RST_PIN;
    }else if(!strcmp(argv[i],"ACDS")){
      //set LEDL reset pin
      reset|=ACDS_RST_PIN;
    }else if(!strcmp(argv[i],"COMM")){
      //set LEDL reset pin
      reset|=COMM_RST_PIN;
    }else if(!strcmp(argv[i],"IMG")){
      //set LEDL reset pin
      reset|=IMG_RST_PIN;
    }else if(!strcmp(argv[i],"all")){
      //set all reset pins
      reset|=LEDL_RST_PIN|ACDS_RST_PIN|COMM_RST_PIN|IMG_RST_PIN;
    }else{
      //attempt to parse numeric value
      num=strtoul(argv[i],&end,0);
      //check if anything worked
      if(end==argv[i]){
        printf("Error : unknown argument %s\r\n",argv[i]);
        return -3;
      }
      //check for second argument
      if(argc!=2){
        printf("Error : numeric commands require 2 arguments\r\n");
        return -7;
      }
      //check for suffix
      if(*end!='\0'){
        printf("Error : unknown suffix \"%s\" for \"%s\"\r\n",end,argv[i]);
        return -5;
      }
      //check range
      if(num>0xFF){
        printf("Error : argument %lu is too large\r\n",num);
        return -6;
      }
      //add value in value
      reset|=num;
    }
  }
  //print out value
  printf("Setting pins 0x%02X\r\n",reset);
  //set correct output values
  P8OUT=
  //set pins to output
  P8DIR|=reset;
  //check if holding
  if(hold){
    printf("Reset Active press any key to stop\r\n");
    //wait for keypres
    getchar();
  }else{
    //wait 100ms for reset to happen
    ctl_timeout_wait(ctl_get_current_time()+100);
  }
  //set pins back to input
  P8DIR&=~reset;
  //print message
  printf("Reset Complete\r\n");
  //done
  return 0;
}


//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    ARC_COMMANDS,CTL_COMMANDS,ERROR_COMMANDS,ARC_ASYNC_PROXY_COMMAND,ARC_SPI_DREAD,
                    {"reset","\r\n\t""Reset the MSP430",resetCmd},
                    {"OnCDH","[bgnd|stop]\r\n\t""Command ON CDH",onCmd},
                    {"OffCDH","port [port ...]\r\n\t""Command OFF CDH",offCmd},
                    {"StatusCDH","\r\n\t""Get CDH status - flash LED's",statusCmd},
                    {"stat","\r\n\t""Get status from all subsystems.", statCmd},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                    {"mag","[all|single]...""\r\n\t""read data from magnetomiters",magCmd},
                    {"cdhp","[on|off]...""\r\n\t""turn on or off printing",CDH_print_cmd},
                    {"tm","[on|off]""\r\n\t""Turn on or off test mode for the magnetometer",mag_test_mode_Cmd},
                    {"burn","[time [delay]]""\r\n\t""trigger the burn circuit to deploy the antenna for [time] in secconds",burnCmd},
                    {"mode","[testing|flight]""\r\n\t""Set/get orbit or flight mode",mode_Cmd},
                    {"power","on|off addr""\r\n\t""Power on or off a subsystem",power_Cmd},
                    {"LEDLtm","[on|off]""\r\n\t""Turn on/off LEDL test mode",LEDLtm_Cmd},
                    {"eps","[cmd] [value]""\r\n\t""Send commands to the EPS",EPS_cmd},
                    {"hreset","\r\n\t""hard reset a given system",hard_reset_Cmd},
                   //end of list

                   {NULL,NULL,NULL}};

