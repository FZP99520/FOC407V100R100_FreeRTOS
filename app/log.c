#include "log.h"
#include "stdarg.h"

#define LOG_BUFFER_MAX_NUM 32
#define log_send(pdata,len) //USART1_SendData(pdata,len)
#define DEBUG


static u8 _pu8Buff[LOG_BUFFER_MAX_NUM][128];
u8 g_u8LogLevel = 0;


#ifdef DEBUG
void debug(const char * format,...)
{
    static u8 _u8Index = 0;
    u16 u16Strlen = 0;

    va_list args;
    va_start(args,format);
    u16Strlen = vsprintf(_pu8Buff[_u8Index],format,args);
    va_end(args);

    log_send(_pu8Buff[_u8Index], u16Strlen);
    
    _u8Index++;
    if(_u8Index == LOG_BUFFER_MAX_NUM) _u8Index = 0;
}
#else
#define debug(args...)
#endif


