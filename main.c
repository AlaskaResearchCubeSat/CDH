//CDH FLIGHT
//Denise Thorsen
//2014-07-10

#include <msp430.h>
#include <string.h>
#include <ctl_api.h>
#include <ARCbus.h>
#include <terminal.h>
#include <UCA1_uart.h>
#include <stdio.h>
#include <Error.h>
#include "CDH.h"

CTL_TASK_t tasks[3];

//stacks for tasks, STACKSize is the middle number, don't know how one determines size
unsigned stack1[1+360+1];
unsigned stack2[1+360+1];
unsigned stack3[1+360+1];

//set printf and friends to send chars out UCA1 uart
int __putchar(int c){
  //don't print if async connection is open
  if(!async_isOpen()){
    return UCA1_TxChar(c);
  }else{
    return EOF;
  }
}

//set scanf and friends to read chars from UCA1 uart
int __getchar(void){
    return UCA1_Getc();
}

int main(void){
 
  //Do this first
  ARC_setup();

  //setup subsystem specific peripherals
  CDH_timer_setup();

  //setup UCA1 UART
  UCA1_init_UART();

  //setup bus interface - CDH
  initARCbus(BUS_ADDR_CDH);

  //Set-up P7 I/O output
  P7OUT = BUS_ADDR_CDH;
  P7DIR = 0xFF;
  P7SEL = 0;

  //Set-up P6 I/O output;
  P6OUT = 0;
  P6DIR = 0xFF;
  P6SEL = 0;

  //initialize stacks
  memset(stack1, 0xcd, sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack
 
  memset(stack2, 0xcd, sizeof(stack2));  // write known values into the stack
  stack2[0]=stack2[sizeof(stack2)/sizeof(stack2[0])-1]=0xfeed; // put marker values at the words before/after the stack

  memset(stack3, 0xcd, sizeof(stack3));  // write known values into the stack
  stack3[0]=stack3[sizeof(stack3)/sizeof(stack3[0])-1]=0xfeed; // put marker values at the words before/after the stack

//create tasks
  ctl_task_run(&tasks[0], BUS_PRI_LOW, cmd_parse, NULL, "cmd_parse", sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1], BUS_PRI_LOW, terminal, "Test CDH code", "terminal", sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
  ctl_task_run(&tasks[2], BUS_PRI_NORMAL, sub_events, NULL, "sub_events", sizeof(stack3)/sizeof(stack3[0])-2,stack3+1,0);
   
 //Call mainLoop to initialize the ARCbus task and drop the idle task priority to zero allowing other tasks to run.  This is the idle loop.
  mainLoop();

}
