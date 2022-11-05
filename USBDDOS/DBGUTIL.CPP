#include <stdio.h>
#include <stdlib.h>
#include <mem.h>
#include <malloc.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include "USBDDOS/DBGUTIL.H"
#include "USBDDOS/DPMI/DPMI.H"

#if DEBUG
static DBG_DBuff dbuff;

static inline DBG_DBuff* db() {dbuff.cur = 0; return &dbuff;}

void DBG_DumpB(uint8_t* StartPtr, int n, DBG_DBuff* buff/* = NULL*/)
{
    buff = buff && buff->enable ? buff : db();
    char* p = buff->buff + buff->cur;
    char* end = buff->buff + DUMP_BUFF_SIZE - 1;

    for(int i = 0; i < n; i++)
    {
        sprintf(p, "%02x", StartPtr[i]);
        p+=2;
        *(p++) = ' ';
        if(i && (i+1)%8 == 0 && (i+1)%16) { sprintf(p, "| "); p+=2;}
        if(i && (i+1)%16 ==0 && i != n-1) {*(p++) = '\n';}
        if( p + 7 >= end)
            break;
    }
    *(p++) = '\n';
    buff->cur = p - buff->buff;
    assert(buff->cur < DUMP_BUFF_SIZE);
    *p = '\0';

    if(buff == &dbuff)
        printf("%s", buff->buff);
    return;
}

void DBG_DumpD(uint32_t* StartPtr, int n, DBG_DBuff* buff/* = NULL*/)
{
    buff = buff && buff->enable ? buff : db();
    char* p = buff->buff + buff->cur;
    char* end = buff->buff + DUMP_BUFF_SIZE - 1;

    for(int i = 0; i < n; i++)
    {
        sprintf(p, "%08lx", StartPtr[i]);
        p += 8;
        *(p++) = ' ';
        if(i && (i+1)%4 == 0 && i != n-1) *(p++) = '\n';
        if(p + 11 >= end)
            break;
    }
    *(p++) = '\n';
    buff->cur = p - buff->buff;
    assert(buff->cur < DUMP_BUFF_SIZE);
    *p = '\0';

    if(buff == &dbuff)
        printf("%s", buff->buff);
    return;
}

void DBG_DumpReq(USB_Request* q, DBG_DBuff* buff/* = NULL*/)
{
    DBG_Printf(buff, "Request: %x, %d, %d, %d, %d\n", q->bmRequestType, q->bRequest, q->wValue, q->wIndex, q->wLength);
}

void DBG_DumpLB(uint32_t addr, int n, DBG_DBuff* buff/* = NULL*/)
{
    uint8_t* b8 = (uint8_t*)alloca(n * sizeof(uint8_t));
    for(int i = 0; i < n; ++i)
        b8[i] = DPMI_LoadB(addr+i);
    DBG_DumpB(b8, n, buff);
}

void DBG_DumpLD(uint32_t addr, int n, DBG_DBuff* buff/* = NULL*/)
{
    uint32_t* d32 = (uint32_t*)alloca(n * sizeof(uint32_t));
    for(int i = 0; i < n; ++i)
        d32[i] = DPMI_LoadD(addr+i*4);
    DBG_DumpD(d32, n, buff);
}

void DBG_DumpPB(uint32_t addr, int n, DBG_DBuff* buff/* = NULL*/)
{
#if defined(__BC__) && 0//disable paging to make sure paing is correct
    DBG_DBuff bf = {1};
    buff = buff == NULL ? &bf : buff;
    __asm {cli; mov eax, cr3; push eax; mov eax, cr0; and eax, 0x7FFFFFFF; mov cr0, eax; xor eax, eax; mov cr3, eax}
#endif

    DBG_DumpLB(DPMI_P2L(addr), n, buff);

#if defined(__BC__) && 0
    __asm {pop eax; mov cr3, eax; mov eax, cr0; or eax, 0x80000000; mov cr0, eax; sti;}
    if(buff == &bf)
        DBG_Flush(&bf);
#endif
}

void DBG_DumpPD(uint32_t addr, int n, DBG_DBuff* buff/* = NULL*/)
{
#if defined(__BC__) && 0//disable paging to make sure paing is correct
    DBG_DBuff bf = {1};
    buff = buff == NULL ? &bf : buff;
    __asm {cli; mov eax, cr3; push eax; mov eax, cr0; and eax, 0x7FFFFFFF; mov cr0, eax; xor eax, eax; mov cr3, eax}
#endif

    DBG_DumpLD(DPMI_P2L(addr), n, buff);

#if defined(__BC__) && 0
    __asm {pop eax; mov cr3, eax; mov eax, cr0; or eax, 0x80000000; mov cr0, eax; sti;}
    if(buff == &bf)
        DBG_Flush(&bf);
#endif
}

void DBG_Printf(DBG_DBuff* nullable buff, const char* fmt, ...)
{
    va_list vl;
    va_start(vl, fmt);
    if(!buff || !buff->enable)
    {
        vprintf(fmt, vl);
        va_end(vl);
        return;
    }

    dbuff.cur = vsprintf(dbuff.buff, fmt, vl);
    va_end(vl);

    uint16_t count = dbuff.cur;
    count = count < DUMP_BUFF_SIZE - buff->cur - 1 ? count : DUMP_BUFF_SIZE - buff->cur - 1;
    memcpy(buff->buff + buff->cur, dbuff.buff, count);
    buff->cur += count;
    *(buff->buff + buff->cur) = 0;
}

void DBG_Flush(DBG_DBuff* buff)
{
    if(buff->enable)
    {
        *(buff->buff+buff->cur) = 0;
        printf("%s", buff->buff);
    }
}

void DBG_DumpREG(DPMI_REG* reg)
{
    printf("eax:%08lx ebx:%08lx ecx:%08lx edx:%08lx\n", reg->d.eax, reg->d.ebx, reg->d.ecx, reg->d.edx);
    printf("ds:%04x es:%04x esi:%08lx edi:%08lx\n", reg->w.ds, reg->w.es, reg->d.esi, reg->d.edi);
    printf("ss:sp:%04x:%04x cs:ip:%04x:%04x flags:%04x\n", reg->w.ss, reg->w.sp, reg->w.cs, reg->w.ip, reg->w.flags);
}
#endif