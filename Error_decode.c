#include <Error.h>
#include "CDH_errors.h"

//decode errors from CDH code
char *err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case ERR_SRC_CMD:
      switch(err){
        case CMD_ERR_RESET:
          return "Command Line : Commanded reset";
      }
      break;
      case CDH_ERR_SRC_MAG:
        switch(err){
          case MAG_ERR_BAD_PACKET_LENGTH:
            sprintf(buf,"Mag : Bad packet length %i",argument);
            return buf;
        }
    break;         
    case ERR_SRC_STAT:
      switch(err){
        case ERROR_TOO_MANY_ERRORS:
          return "Status : Too many errors";
      }
    break;
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
