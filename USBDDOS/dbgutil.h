#ifndef _DBGUTIL_H_
#define _DBGUTIL_H_
#include <stdlib.h>
#include <stdarg.h>
#include "USBDDOS/DPMI/dpmi.h"

#define DUMP_BUFF_SIZE 1024U

//log should be disabled or write to file in final shipping. use output function (printf/puts) to show msg to user.
#ifndef _LOG_ENABLE
#define _LOG_ENABLE DEBUG
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if _LOG_ENABLE

//_LOG() can be called in interrupt handler. DO NOT use sys calls, i.e. printf/INT10h in interrupt handler.
void DBG_Logv(const char* fmt, va_list aptr);
void DBG_Log(const char* fmt, ...);
#define _LOG DBG_Log

#else //_LOG_ENABLE

#if defined(__BC__) //no variadic macro, so string constants may not optimized out
int DBG_Nolog(const char* fmt, ...);
#define _LOG DBG_Nolog
#else
#define _LOG(...) 
#endif

#endif //_LOG_ENABLE


#if DEBUG

typedef struct
{
    uint16_t enable;
    uint16_t cur;
    char buff[DUMP_BUFF_SIZE];
}DBG_DBuff;

//printf to screen if targetbuff==NULL
void DBG_DumpB(uint8_t* StartPtr, unsigned n, DBG_DBuff* nullable buff);
void DBG_DumpD(uint32_t* StartPtr, unsigned n, DBG_DBuff* nullable buff);
void DBG_DumpReq8(uint8_t* req, DBG_DBuff* nullable buff);

void DBG_DumpLB(uint32_t addr, unsigned n, DBG_DBuff* nullable buff);  //linear memory dump
void DBG_DumpLD(uint32_t addr, unsigned n, DBG_DBuff* nullable buff);
void DBG_DumpPB(uint32_t addr, unsigned n, DBG_DBuff* nullable buff);  //physical memory dump
void DBG_DumpPD(uint32_t addr, unsigned n, DBG_DBuff* nullable buff);

void DBG_Printf(DBG_DBuff* nullable buff, const char* fmt, ...);
void DBG_Flush(DBG_DBuff* buff);

void DBG_DumpREG(DPMI_REG* reg);

#endif//DEBUG

#ifdef __cplusplus
}//extern "C"
#endif

#endif
