#include <msp430.h>
#include <ctl_api.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ARCbus.h>
#include "CDH.h"
#include <Error.h>
#include "mag.h"
#include "CDH_errors.h"

CTL_EVENT_SET_t cmd_parse_evt;

short beacon_on=0;
short USB_power=0;
char GS_CMD[30];

STAT_PACKET system_stat;

void CDH_timer_setup(void){
  //enable 2 sec interrupt
  TACTL|=TAIE;
}

//================[Time Tick interrupt]=========================
void task_tick(void) __ctl_interrupt[TIMERA1_VECTOR]{
  static int sec=0;
  switch(TAIV){
    case TAIV_TACCR1:
    break;
    case TAIV_TACCR2:
    break;
    case TAIV_TAIFG:
 //     ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_GET_STAT,0);
      sec+=2;
//      P7OUT^=BIT7; //toggle bit 7
//      ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_SEND_STAT,0);
      if(sec==8){
        //collect status
        ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_GET_STAT,0);
        //clear status valid flags
        system_stat.flags=0;
      }
      if(sec>=10){
      P7OUT^=BIT5; //toggle bit 5
        sec=0;
        //send status
        ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_SEND_STAT,0);
      }
    break;
  }
}

CTL_EVENT_SET_t mag_evt;

MAG_DAT magData;

//handle subsystem specific commands //called by the main ARCbus task don't linger here
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len){
  int i;
  
  switch(cmd){
    case CMD_MAG_DATA:
      //check packet length
      if(len!=sizeof(magData)){
        //length incorrect, report error and exit
        report_error(ERR_LEV_ERROR,CDH_ERR_SRC_MAG,MAG_ERR_BAD_PACKET_LENGTH,len);
        return ERR_PK_LEN;
      }
      memcpy(&magData,dat,sizeof(magData));
      //sensor data recieved set event
      ctl_events_set_clear(&mag_evt,MAG_EVT_COMMAND_SENSOR_READ,0);
    return RET_SUCCESS;
      
    case CMD_GS_DATA:				           //Ground Station Command
      if(len==(dat[1]+3)){			   //check length, min 3 bytes + data
         memcpy(&GS_CMD,dat,len);
         ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_GS_CMD,0);
	 return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
                
    case CMD_SUB_POWERUP: //Each subsystem sends this command on powerup.
      if(len==0){
        switch(src){
                case BUS_ADDR_LEDL: 
                        system_stat.LEDL_powerup++;
                        return RET_SUCCESS;
                case BUS_ADDR_ACDS: 
                        system_stat.ACDS_powerup++;
                        return RET_SUCCESS;
                case BUS_ADDR_COMM: 
                        system_stat.COMM_powerup++;
                        if(beacon_on){                      // If COMM cycles after deployment need to tell COMM to RF on
                          ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_RF_ON,0);
                          ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_BEACON_ON,0);
                        }
                        return RET_SUCCESS;
                case BUS_ADDR_IMG:  
                        system_stat.IMG_powerup++;
                        return RET_SUCCESS;
        }
        //TODO: return appropriate error code
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;
      }
	  
    case CMD_SPI_CLEAR:
      //set event
      //TODO: keep track of who is using SPI
      ctl_events_set_clear(&cmd_parse_evt,CMD_PARSE_SPI_CLEAR,0);
      return RET_SUCCESS;
    
    case CMD_EPS_STAT:							//EPS Status
      if(len==sizeof(system_stat.EPS_stat)){ 	//check length
        memcpy(&system_stat.EPS_stat,dat,len); 	//copy data into structure
        system_stat.flags|=STAT_EPS_VALID;		//set flag for EPS status
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
    
    case CMD_LEDL_STAT:							//LEDL status
      if(len==sizeof(system_stat.LEDL_stat)){	//check length
        memcpy(system_stat.LEDL_stat,dat,len);	//copy data into structure
        system_stat.flags|=STAT_LEDL_VALID;		//set flag for LEDL status
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
	  
    case CMD_ACDS_STAT:							//ACDS status
      if(len==sizeof(system_stat.ACDS_stat)){	//check length
        memcpy(system_stat.ACDS_stat,dat,len);	//copy data into structure
        system_stat.flags|=STAT_ACDS_VALID;		//set flag for ACDS status
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
	  
    case CMD_COMM_STAT:							//COMM status
      if(len==sizeof(system_stat.COMM_stat)){	//check length
        memcpy(system_stat.COMM_stat,dat,len);	//copy data into structure
        system_stat.flags|=STAT_COMM_VALID;		//set flag for COMM status
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
	  
    case CMD_IMG_STAT:							//IMG status
      if(len==sizeof(system_stat.IMG_stat)){	//check length
        memcpy(system_stat.IMG_stat,dat,len);	//copy data into structure
        system_stat.flags|=STAT_IMG_VALID;		//set flag for IMG status
        return RET_SUCCESS;
      }else{
        return ERR_PK_LEN;						//Packet Len Error
      }
	  

  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}

int CDH_print=1;

void cdh_print(const char *fmt,...){
  va_list args;
  if(CDH_print){
    //initialize va_list
    va_start(args,fmt);
    //call printf
    vprintf(fmt,args);
    //cleanup va_list
    va_end(args);
  }
}

// this is where the work happens
void cmd_parse(void *p) __toplevel{
  unsigned int e, launch=0;
  static unsigned char buff[128];
  unsigned char *ptr;
  ticker time;
  int resp,i;
  system_stat.type = SPI_BEACON_DAT;
  system_stat.CDH_addr=BUS_ADDR_CDH;
  system_stat.LEDL_addr=BUS_ADDR_LEDL;
  system_stat.ACDS_addr=BUS_ADDR_ACDS;
  system_stat.COMM_addr=BUS_ADDR_COMM;
  system_stat.IMG_addr=BUS_ADDR_IMG;
  system_stat.EPS_addr=0x16;

  USB_power = USB_power_check();

  cdh_print("Turn on LEDL NMOS P6.6\r\n");
  //Turn on MOSFET to power LEDL from EPS
  P6OUT|=BIT6;
  P6DIR|=BIT6;
  cdh_print("Send interrupt to LEDL inidcating CLYDE is ON P1.0\r\n");
  BUS_int_set(BIT0);

  //init event
  ctl_events_init(&cmd_parse_evt,0);
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&cmd_parse_evt,CMD_PARSE_ALL,CTL_TIMEOUT_NONE,0);

    if(e&CMD_PARSE_SPI_CLEAR){
      cdh_print("SPI bus Free\r");
      //TODO: keep track of who is using SPI
    }

    if(e&CMD_PARSE_GET_STAT || e&CMD_PARSE_GET_STAT_CMD){
      system_stat.flags=0;						//clear status flags from old status packet
      cdh_print("Requesting status\r\n");                                  //print message
      ptr=BUS_cmd_init(buff,CMD_SUB_STAT);                              //setup packet 
      time=get_ticker_time();                                           //get time
      ptr[0]=time>>24;							//write time into the array
      ptr[1]=time>>16;
      ptr[2]=time>>8;
      ptr[3]=time;
      resp=BUS_cmd_tx(BUS_ADDR_GC,buff,4,0,BUS_I2C_SEND_FOREGROUND);	//send command
      //Check if there was an error
      if(resp!=RET_SUCCESS){
        //Print error
        cdh_print("Error sending status %s\r\n",BUS_error_str(resp));
      }
    }

    if((e&CMD_PARSE_SEND_STAT) || e&CMD_PARSE_SEND_STAT_CMD){
    // if beacon_on true send data to comm
    // if beacon_on false check eps for positive power set time counter for antenna deployment and beacon_on
      if(beacon_on){							// beacon_on = true, Send data to COMM
        cdh_print("Sending status\r\n");
        system_stat.type=SPI_BEACON_DAT;        // Type = SPI_BEACON_DAT
        time=get_ticker_time();					//get time for beacon
        system_stat.time0=time>>24;				//write time into status
        system_stat.time1=time>>16;
        system_stat.time2=time>>8;
        system_stat.time3=time;
												//send SPI data
        resp=BUS_SPI_txrx(BUS_ADDR_COMM,(unsigned char*)&system_stat,NULL,sizeof(STAT_PACKET)-BUS_SPI_CRC_LEN);
        if(resp!=RET_SUCCESS){
          cdh_print("Error : failed to send beacon data : %s\r\n",BUS_error_str(resp));
        }
        ptr=(unsigned char*)&system_stat;
		
		//FOR TEST ONLY - Print out status buffer
        for(i=0;i<11;i++){
          cdh_print("0x%02X ",ptr[i]);
        }
        cdh_print("\r\n");
        cdh_print("0x%02X ",system_stat.LEDL_addr);
        PrintBuffer(system_stat.LEDL_stat, sizeof(system_stat.LEDL_stat));
        cdh_print("0x%02X ",system_stat.ACDS_addr);
        PrintBuffer(system_stat.ACDS_stat, sizeof(system_stat.ACDS_stat));
        cdh_print("0x%02X ",system_stat.COMM_addr);
        PrintBuffer(system_stat.COMM_stat, sizeof(system_stat.COMM_stat));
        cdh_print("0x%02X ",system_stat.IMG_addr);
        PrintBuffer(system_stat.IMG_stat, sizeof(system_stat.IMG_stat));
        cdh_print("0x%02X ",system_stat.EPS_addr);
        PrintBuffer((char*)&system_stat.EPS_stat, sizeof(system_stat.EPS_stat));
        
        if(system_stat.flags&STAT_EPS_VALID){
          //print solar cell voltages for testing
          cdh_print("\r\n=================================================\r\nSolar Cell voltages:\r\n");
          cdh_print("\tX-voltage = %u\r\n",system_stat.EPS_stat.X_voltage);
          cdh_print("\tY-voltage = %u\r\n",system_stat.EPS_stat.Y_voltage);
          cdh_print("\tZ-voltage = %u\r\n",system_stat.EPS_stat.Z_voltage);
          cdh_print("=================================================\r\n\r\n");
        }

        if((system_stat.flags&STAT_ALL_VALID)==STAT_ALL_VALID){
          cdh_print("All subsystems reported status\r\n");
        }else{
          if(!(system_stat.flags&STAT_EPS_VALID)) {
            cdh_print("No Status Info for EPS\r\n");
          }
          if(!(system_stat.flags&STAT_LEDL_VALID)){
            cdh_print("No Status Info for LEDL\r\n");
          }
          if(!(system_stat.flags&STAT_ACDS_VALID)){
            cdh_print("No Status Info for ACDS\r\n");
          }
          if(!(system_stat.flags&STAT_COMM_VALID)){
            cdh_print("No Status Info for COMM\r\n");
          }
          if(!(system_stat.flags&STAT_IMG_VALID)){
            cdh_print("No Status Info for IMG\r\n");
          }
        }
		//FOR TEST ONLY - Print out status buffer
		
      } else { // beacon_on = FALSE start_up routine
      // check eps status for positive power
        cdh_print("Launch = %d; USB_power = %d\r\n",launch,USB_power);
        cdh_print("Waiting for Solar Cell voltage above threshold: \r\n");
        cdh_print("\tSolar Cell X-voltage = %u\r\n",system_stat.EPS_stat.X_voltage);
        cdh_print("\tSolar Cell Y-voltage = %u\r\n",system_stat.EPS_stat.Y_voltage);
        cdh_print("\tSolar Cell Z-voltage = %u\r\n",system_stat.EPS_stat.Z_voltage);
        if((system_stat.EPS_stat.Y_voltage>= minV) || (system_stat.EPS_stat.X_voltage>=minV) || (system_stat.EPS_stat.Z_voltage>=minV)){ // positive voltage detected
          cdh_print("Solar Cell voltage above threshold\r\n");
          if(!launch){ //assuming we haven't been here before start the deployment timers.
            cdh_print("Set Antenna Deployment and RF ON timers\r\n");
            BUS_set_alarm(BUS_ALARM_0,get_ticker_time()+DeployAntennaTime,&cmd_parse_evt,CMD_PARSE_ANTENNA_DEPLOY);
            BUS_set_alarm(BUS_ALARM_1,get_ticker_time()+RFONTime,&cmd_parse_evt,CMD_PARSE_RF_ON);
            launch=1;
          }
        }
      }
    }

    if(e&CMD_PARSE_ANTENNA_DEPLOY){
      cdh_print("Deploy antenna\r");
      if(!USB_power){
           burn_on();
          //delay for specified time
          ctl_timeout_wait(ctl_get_current_time()+BURN_DELAY);
          //turn off resistor
          burn_off();
          //Deploy the antenna only if the USB_power is not on!
          //Deployment should happen 10+ minutes after launch. P6.7
      } else cdh_print("USB power do not deploy.\r\n");

    }

    if(e&CMD_PARSE_RF_ON){                          // TURN ON HELLO MESSAGE IN BEACON
      cdh_print("Turn Beacon ON\r");
      // when timer times out send on command to comm, set beacon_on = 1;
      beacon_on = 1;
      if(!USB_power){
        ptr=BUS_cmd_init(buff,CMD_SUB_ON);
        resp=BUS_cmd_tx(BUS_ADDR_COMM,buff,0,0,BUS_I2C_SEND_FOREGROUND);
        if(resp!=RET_SUCCESS){
          cdh_print("Failed to send POWER ON to COMM %s\r\n",BUS_error_str(resp));
        }
        resp=BUS_set_alarm(BUS_ALARM_1,get_ticker_time()+BeaconONTime,&cmd_parse_evt,CMD_PARSE_BEACON_ON);
        if(resp!=RET_SUCCESS){
          cdh_print("Failed to set STATUS ON Alarm %s\r\n",BUS_error_str(resp));
        }  
      }
    }

    if(e&CMD_PARSE_BEACON_ON){                        // TURN ON STATUS MESSAGE IN BEACON
       cdh_print("Turn on Status Beacon Message\r");
       ptr=BUS_cmd_init(buff,CMD_BEACON_ON);
       resp=BUS_cmd_tx(BUS_ADDR_COMM,buff,0,0,BUS_I2C_SEND_FOREGROUND);
       if(resp!=RET_SUCCESS){
          cdh_print("Failed to send BEACON ON to COMM %s\r\n",BUS_error_str(resp));
      }
    }

    if(e&CMD_PARSE_GS_CMD){ //Figure out what the command is and where it needs to go, then send it.
        switch(GS_CMD[0]){
        case BUS_ADDR_IMG:
           cdh_print("GS CMD to IMG\r\n");
          // for(i=0;i<GS_CMD[1]+3;i++) cdh_print("0x%02x \r\n",GS_CMD[i]);
           switch(GS_CMD[2]){
           case CMD_IMG_TAKE_TIMED_PIC:
             cdh_print("Sending CMD_IMG_TAKE_TIMED_PIC to IMG\r\n");
              //setup packet 
              ptr=BUS_cmd_init(buff,CMD_IMG_TAKE_TIMED_PIC);
              //fill in telemetry data
              for(i=0;i<GS_CMD[1];i++){
                  ptr[i]=GS_CMD[3+i];
              }
              //send command
              resp=BUS_cmd_tx(BUS_ADDR_IMG,buff,GS_CMD[1],0,BUS_I2C_SEND_FOREGROUND);
              if(resp!=RET_SUCCESS){
                  cdh_print("Failed to send GS CMD to CDHs %s\r\n",BUS_error_str(resp));
              }
             break;
           }
           break;
        case BUS_ADDR_ACDS:
           cdh_print("GS CMD to ADCS\r\n");
           for(i=0;i<GS_CMD[1]+3;i++) cdh_print("0x%02x \r\n",GS_CMD[i]);
           break;
        case BUS_ADDR_LEDL:
           cdh_print("GS CMD to LEDL\r\n");
           for(i=0;i<GS_CMD[1]+3;i++) cdh_print("0x%02x \r\n",GS_CMD[i]);
           break;
        }

    }
    //end event loop
  }
}

//parse events from the bus for the subsystem
void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  unsigned char buf[10],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);

    if(e&SUB_EV_PWR_OFF){
      //print message
      cdh_print("System Powering Down\r");
    }

    if(e&SUB_EV_PWR_ON){
      //print message
      cdh_print("System Powering Up\r");
    }

    if(e&SUB_EV_SPI_DAT){
      cdh_print("SPI data recived:\r");
      //get length
      len=arcBus_stat.spi_stat.len;
      //print out data
      for(i=0;i<len;i++){
        //cdh_print("0x%02X ",rx[i]);
        cdh_print("%03i ",arcBus_stat.spi_stat.rx[i]);
      }
      cdh_print("\r\n");
      //free buffer
      BUS_free_buffer_from_event();
    }

    if(e&SUB_EV_SPI_ERR_CRC){
      cdh_print("SPI bad CRC\r");
    }
  }
}

void PrintBuffer(char *dat, unsigned int len){
   int i;

   for(i=0;i<len;i++){
      cdh_print("0X%02X ", dat[i]);
      if((i)%15==14){
        cdh_print("\r\n");
      }
    }
    cdh_print("\r\n");
}

short USB_power_check(void){
  cdh_print("Check to see if powered by USB\r\n");
  return !(P1IN & BIT1);
//  return 0; //test
}

void burn_on(void){
    //turn on LED
    P7OUT|=BIT7;
    //turn on resistor
    P6OUT|=BIT7;
}

void burn_off(void){
    //turn off resistor
    P6OUT&=~BIT7;
    //turn off LED
    P7OUT&=~BIT7;
}

