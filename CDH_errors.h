#ifndef __CDH_ERRORS_H
  #define __CDH_ERRORS_H
  #include <commandLib.h>
  //error sources for BUS test program
  enum{CDH_ERR_SRC_SUBSYSTEM=ERR_SRC_CMD+1,CDH_ERR_SRC_MAG};
  
  //errors for magnetometer data
  enum{MAG_ERR_BAD_PACKET_LENGTH};
    
#endif
  