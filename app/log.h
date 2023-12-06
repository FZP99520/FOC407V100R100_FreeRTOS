#ifndef _LOG_H
#define _LOG_H

#include "stm32f4xx.h"
#include "display.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FR_OS.h"


#if DEBUG_LCD
#define DBG(fmt,args...)    DisplayLogMessage(LCD_WHITE,fmt,args)
#define DebugError(fmt,...) DisplayLogMessage(LCD_RED,"[%s][%d]"fmt,__FUNCTION__,__LINE__,##__VA_ARGS__)
#define DebugInfo(fmt,...)  DisplayLogMessage(LCD_GREEN,"[%s][%d]"fmt,__FUNCTION__,__LINE__,##__VA_ARGS__)

#else
#define DebugError(fmt,...)
#define DebugInfo(fmt,...)
#endif
//#define print(fmt,...)      DisplayLogMessage(LCD_WHITE,fmt,##__VA_ARGS__)

void debug(const char * format,...);
extern u8 g_u8LogLevel;


#define LOG_LEVEL 0x0F
#define LOGLEVEL_PRINT_ENABLE  0x01
#define LOGLEVEL_INFO_ENABLE   0x02
#define LOGLEVEL_TRACE_ENABLE  0x04
#define LOGLEVEL_ERROE_ENABLE  0x08

#define DEBUG_PRINT(fmt, ...)  \
    if(LOG_LEVEL&LOGLEVEL_PRINT_ENABLE)\
        {\
            debug(fmt, ##__VA_ARGS__);\
        }
#define DEBUG_INFO(fmt, ...)   \
    if(LOG_LEVEL&LOGLEVEL_INFO_ENABLE)\
        {\
            debug("[INFO][%s][L%d]"fmt,__FUNCTION__, __LINE__, ##__VA_ARGS__);\
        }
#define DEBUG_TRACE(fmt, ...)  \
    if(LOG_LEVEL&LOGLEVEL_TRACE_ENABLE)\
        {\
            debug("[TRACE][%s][L%d]"fmt,__FUNCTION__, __LINE__, ##__VA_ARGS__);\
        }
#define DEBUG_ERROR(fmt, ...)  \
    if(LOG_LEVEL&LOGLEVEL_ERROE_ENABLE)\
        {\
            debug("[ERROR][%s][L%d]"fmt,__FUNCTION__, __LINE__, ##__VA_ARGS__);\
        }
#define DEBUG_VAR_INT(var)    debug("%s:%d\n",#var, var)
#define DEBUG_VAR_PIONT(var)  debug("%s:%#x\n",#var, var)
#define DEBUG_VAR_HEX(var)    debug("%s:%#x\n",#var, var)
#define DEBUG_VAR_CHAR(var)   debug("%s:%s\n",#var, var)


#endif

