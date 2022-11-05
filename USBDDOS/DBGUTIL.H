#ifndef _DBGUTIL_H_
#define _DBGUTIL_H_
#include <stdlib.h>
#include "USBDDOS/USB.H"
#include "USBDDOS/DPMI/DPMI.H"

#define DEBUG 1

#if DEBUG

#define DUMP_BUFF_SIZE 1024
typedef struct
{
    uint16_t enable;
    uint16_t cur;
    char buff[DUMP_BUFF_SIZE];
}DBG_DBuff;

//printf to screen if targetbuff==NULL
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
void DBG_DumpB(uint8_t* StartPtr, int n, DBG_DBuff* buff = NULL);
void DBG_DumpD(uint32_t* StartPtr, int n, DBG_DBuff* buff = NULL);
void DBG_DumpReq(USB_Request* q, DBG_DBuff* buff = NULL);

void DBG_DumpLB(uint32_t addr, int n, DBG_DBuff* buff = NULL);  //linear memory dump
void DBG_DumpLD(uint32_t addr, int n, DBG_DBuff* buff = NULL);
void DBG_DumpPB(uint32_t addr, int n, DBG_DBuff* buff = NULL);  //physical memory dump
void DBG_DumpPD(uint32_t addr, int n, DBG_DBuff* buff = NULL);
#else
void DBG_DumpB(uint8_t* StartPtr, int n, DBG_DBuff* buff);
void DBG_DumpD(uint32_t* StartPtr, int n, DBG_DBuff* buff);
void DBG_DumpReq(USB_Request* q, DBG_DBuff* buff);

void DBG_DumpLB(uint32_t addr, int n, DBG_DBuff* buff);
void DBG_DumpLD(uint32_t addr, int n, DBG_DBuff* buff;
void DBG_DumpPB(uint32_t addr, int n, DBG_DBuff* buff);
void DBG_DumpPD(uint32_t addr, int n, DBG_DBuff* buff);
#endif

void DBG_Printf(DBG_DBuff* nullable buff, const char* fmt, ...);
void DBG_Flush(DBG_DBuff* buff);

void DBG_DumpREG(DPMI_REG* reg);

#ifdef __cplusplus
}//extern "C"
#endif

#endif//DEBUG

#endif