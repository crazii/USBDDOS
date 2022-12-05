#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <fcntl.h>
#include <assert.h>
#include "USBDDOS/PLATFORM.H"

#if defined(__DJ2__)
#include <sys/ioctl.h>
#else

#if defined(__BC__)
#include <io.h>
#endif
#define DOS_RCVDATA 2
#define DOS_SNDDATA 3

#endif

#include <stdio.h>
#include "EMM.h"
#include "USBDDOS/DBGUTIL.H"

#define EMM_IOTRAP_DIRECT_INT 0

uint16_t EMM_GetVersion(void)
{
    //DPMI_REG r0 = {0};
    //r0.ax = 0x4600;
    //DPMI_SimulateRealModeINT(0x67, &r0);
    //printf("%04x %04x\n",r0.ax&0xF0, r0.ax&0x0F);

    int fd = 0;
    unsigned int result = _dos_open("386MAX$$", O_RDONLY, &fd);
    if(result == 0)
    {
        _dos_close(fd);
        return 0xFFFF;
    }
    result = _dos_open("EMMXXXX0", O_RDONLY, &fd);
    if(result != 0)
        result = _dos_open("EMMXXXQ0", O_RDONLY, &fd);
    if(result != 0)
        result = _dos_open("EMMQXXX0", O_RDONLY, &fd);
    if(result != 0)
        return 0;
#if defined(__DJ2__) || defined(__BC__)
    //control code 2: read version
    uint16_t v = 2;
    //ioctl - read from character device control channel
    #if defined(__BC__)
    if(ioctl(fd, DOS_RCVDATA, &v, 2) != 2)
    #else
    if(ioctl(fd, DOS_RCVDATA, 2, &v) != 2)
    #endif
    {
        _dos_close(fd);
        return 0xAD00;
    }
    _dos_close(fd);
    return v;
#elif defined(__WC__)
    uint32_t mem = DPMI_DOSMalloc(1); //1 paragraph, 16 bytes
    uint16_t* buff = (uint16_t*)((mem&0xFFFF) << 4);
    //int 21h, ax=4402h, ax(bx,ds:dx,cx)
    DPMI_REG r = {0};
    r.w.bx = fd;
    r.w.ds = (uint16_t)(mem&0xFFFF);
    r.w.dx = 0;
    r.w.cx = DOS_RCVDATA; //ioctl function
    r.w.ax = 0x4402;
    r.w.flags = CPU_CFLAG;
    //control code 2 and read version
    *buff = 2;
    if( DPMI_CallRealModeINT(0x21, &r) != 0 || (r.w.flags&CPU_CFLAG) || r.w.ax != 2 )
    {
        _dos_close(fd);
        DPMI_DOSFree(mem);
        return 0xAD00;
    }
    uint16_t ver = *buff;
    _dos_close(fd);
    DPMI_DOSFree(mem);
    return ver;
#else
#if !defined(_WIN32) && !defined(__linux__)   //make editor happy
    #error not supported.
#endif
#endif
 }

typedef struct
{
    //we will load new gdt instead of hook EMM386's own like SoftMPU
    GDTR oldgdtr;   //0+2
    IDTR oldidtr;   //8+2
    uint16_t oldcs;     //16
    uint16_t oldss;     //18
    uint32_t oldesp;    //20
    IDTR nullidtr;  //24+2, static
    GDTR gdtr;      //32+2, static
    uint32_t esp;       //40  target stack, static
    uint32_t wrapper;   //44, static, local wrapper function. EMM_IOPortTrap_Wrapper
    uint16_t code_offset;//48, static
    uint16_t unused;    //50
    uint16_t tableOffset; //52, static
    uint16_t tableCount;  //54, static
    //new field can be added here so that above offset are fixed
                        //56
    GDT gdt[6];         //gdt, static. //null,cs,ds,16bitcs,16bitds,4gds. not directly addressed by assembly, put at last.
                        //jumptable, static. EMM_IODT_FarAddr. addressed by tableOffset
}EMM_IODT_Header;

//real iodt that used
typedef struct EMM_IODispatchTable_V86
{
    uint16_t port;
    uint16_t offset;
}EMM_IODT_V86;

typedef struct
{
    uint32_t offset;
    uint16_t segment;
    uint16_t port;
}EMM_IODT_FarAddr;

#if defined(__DJ2__)
_Static_assert(sizeof(GDT) == 8, "incorrect size/alignment");
_Static_assert(sizeof(GDTR) == 8, "incorrect size/alignment");
_Static_assert(sizeof(IDTR) == 8, "incorrect size/alignment");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, oldcs) == 16, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, oldss) == 18, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, oldesp) == 20, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, gdtr) == 32, "incorrect offset");

_Static_assert(__builtin_offsetof(EMM_IODT_Header, esp) == 40, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, wrapper) == 44, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, code_offset) == 48, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, tableOffset) == 52, "incorrect offset");
_Static_assert(__builtin_offsetof(EMM_IODT_Header, tableCount) == 54, "incorrect offset");
#endif

//protected mode ring0 entry for EMM386 callback
__EXTERN void __NAKED EMM_IOPortTrap_StartCode()
{
    _ASM_BEGIN16
    _ASM(pushf)
    _ASM(cli)
    _ASM(push ebx)
    _ASM(push ecx)
    _ASM(push edx)
    _ASM(push esi)
    _ASM(push edi)
    _ASM(push ebp)
    _ASM(push ds)
    _ASM(push es)

    //find entry in jump table
    //edx:port, ecx inout, eax val
    _ASM2(mov si, word ptr ds:[54]) //talbeCount
    _ASM2(mov bp, word ptr ds:[52]) //table offset
    _ASM(dec si)
_ASMLBL(jmptable_loop:) //for(int i = talbeCount-1; i >= 0; --i)
    _ASM2(mov di, word ptr ds:[ebp + 6 + esi*8]) //port
    _ASM2(cmp dx, di)
    _ASM(je found_entry)
    _ASM(dec si)
    _ASM(jno jmptable_loop)
    _ASM(jmp failure)
_ASMLBL(found_entry:)
    _ASM2(lea edi, [ebp + esi*8])

    /**setup code**/
    _ASM(push cs) //save registers
    _ASM(pop word ptr ds:[16])
    _ASM(push ss)
    _ASM(pop word ptr ds:[18])
    _ASM2(mov dword ptr ds:[20], esp)

    _ASM_SGDT(ds:[2])
    _ASM_SIDT(ds:[10])
    _ASM_LIDT(ds:[26])
    _ASM_LGDT(ds:[34])

    _ASM2(mov bx, 0x20) //current ds
    _ASM2(mov ds, bx)

    _ASM2(mov ebx, cr0) //disable paging. the stack provided by EMM386 might not be available anymore, need a stack with physical base
    _ASM2(mov ebp, ebx)
    _ASM2(and ebx, 0x7FFFFFFF)
    _ASM2(mov cr0, ebx)
    _ASM2(mov ebx, cr3)
    _ASM2(xor esp, esp)
    _ASM2(mov cr3, esp) //flush TLB

    _ASM2(mov sp, 0x10) //target ds
    _ASM2(mov ss, sp) //new stack
    _ASM2(mov esp, dword ptr ds:[40])
    _ASM(push ebp)  //save old cr0 to new stack
    _ASM(push ebx) //cr3
    _ASM2(xor ebp, ebp)

    /**handler code**/

#if !defined(__BC__)
    _ASM2(mov ebx, 0x18)   //setup return addr. 0x18=current cs
    _ASM(push ebx)  //segment hiword ignored by cpu
    _ASM2(mov ebx, offset callret)
    _ASM(call translate_label)
    _ASM(push ebx)

    _ASM2(mov ebx, dword ptr ds:[edi]) //target function
    _ASM(push dword ptr ds:[edi+4]) //dword segment+port, port is ignored
    _ASM(push dword ptr ds:[44]) //wrapper function

    _ASM2(mov bp, 0x10)
    _ASM2(mov ds, bp) //target ds
    _ASM2(mov es, bp)
    //Intel 64 and IA-32 Architectures Software Developer's Manuals Vol. 2A 3-583
    _ASM_OPERAND_SIZE _ASM(retf) //opsize toggle. ret to 32 bit stack frame (as we pushed)
#else
    _ASM(push 0x18)  //segment hiword ignored by cpu
    _ASM2(mov bx, offset callret)
    _ASM(call translate_label)
    _ASM(push bx)

    _ASM2(mov bx, word ptr ds:[di]) //target function
    _ASM(push word ptr ds:[di+4]) //segment
    _ASM(push word ptr ds:[44]) //wrapper function

    _ASM2(mov bp, 0x10)
    _ASM2(mov ds, bp) //target ds
    _ASM2(mov es, bp)
    _ASM(retf)
#endif
    _ASMLBL(callret:)

    /** clean up code **/
_ASMLBL(cleanup:)
    _ASM2(mov bx, 0x20)//current ds
    _ASM2(mov ds, bx)

    _ASM(pop ebx)
    _ASM2(mov cr3, ebx)
    _ASM(pop ebx) //restore paging setting
    _ASM2(mov cr0, ebx)
    _ASM_LGDT(ds:[2]) //restore gdt

    //old ss:esp
    _ASM2(mov bx, word ptr ds:[18])
    _ASM2(mov ss, bx)
    _ASM2(mov esp, dword ptr ds:[20])

    //old cs
    _ASM(push word ptr ds:[16])
    _ASM2(mov bx, offset load_cs2)
    _ASM(call translate_label)
    _ASM(push bx)
    _ASM(retf)
    _ASMLBL(load_cs2:)
    _ASM_LIDT(ds:[10]) //restore idt

    _ASM(clc)
    _ASM(jmp done)
_ASMLBL(failure:)
    _ASM2(xor eax, eax)
    _ASM(stc)
_ASMLBL(done:)
    _ASM(pop es)
    _ASM(pop ds)
    _ASM(pop ebp)
    _ASM(pop edi)
    _ASM(pop esi)
    _ASM(pop edx)
    _ASM(pop ecx)
    _ASM(pop ebx)
    _ASM(popf)
    _ASM_RETF

    //the tricky part: inline assembly generate label in the exe's code space
    //but this piece of code will run in another allocated memory at runtime,
    //need to tranlsate it to current space where it is running
_ASMLBL(translate_label:)  //in bx: abs label offset. return bx: abs offset for runtime
#if defined(__DJ2__)
    _ASM2(sub bx, offset _EMM_IOPortTrap_StartCode)
#else
    _ASM2(sub bx, offset EMM_IOPortTrap_StartCode) //relative to code start
#endif
    _ASM2(add bx, word ptr ds:[48]) //code_offset in EMM_IODT_Header
    _ASM(ret)

    _ASM_END16
}

void __NAKED EMM_IOPortTrap_EndCode() {} //dummy function

void __NAKED EMM_IOPortTrap_Install_V86()//this is a real(/v86) mode call
{
    _ASM_BEGIN16
    _ASM2(mov ax, 0x4A15)
    _ASM(int 0x2F)
    _ASM_RETF
    _ASM_END16
}
void __NAKED EMM_IOPortTrap_Install_V86End() {} //dummy function

static DPMI_ADDRESSING EMM_Addressing;
static DPMI_ADDRESSING EMM_OldAddressing;
__EXTERN uint32_t __CDECL EMM_IOPortTrap_WrapperC(uint32_t handler32, uint32_t port, uint32_t val, uint32_t out)//handler not pointer for compatile reason (BC)
{
    DPMI_SetAddressing(&EMM_Addressing, &EMM_OldAddressing);
    EMM_IOTRAP_HANDLER handler = (EMM_IOTRAP_HANDLER)(uintptr_t)handler32;
    uint32_t result = handler(port, val, out);
    DPMI_SetAddressing(&EMM_OldAddressing, &EMM_Addressing);
    return result;
}
//RETF wrapper function for client, port trap handler code will call this entry
static /*uint32_t*/ void __NAKED EMM_IOPortTrap_Wrapper()
{ //edx: port, ecx:inout 0-in. eax:data, ebx/bx: target function. return eax.
    _ASM_BEGIN
    _ASM(push ecx)
    _ASM(push eax)
    _ASM(push edx)
    _ASM(push ebx)
#if defined(__DJ2__)
    _ASM(call _EMM_IOPortTrap_WrapperC)
#elif defined(__WC__)
    _ASM(call EMM_IOPortTrap_WrapperC)
#elif defined(__BC__)
    _ASM(call EMM_IOPortTrap_WrapperC)
    _ASM2(shl dx, 16)
    _ASM2(mov dx, ax)
    _ASM2(mov eax, edx)
#endif
    _ASM2(add esp, 16)
    _ASM_RETF
    _ASM_END
}

//our code & data are in himem, we can allocate a seperate small segment and fill with
//real mode TSR data & code and do TSR with almost 0 mem residence.
BOOL EMM_Install_IOPortTrap(uint16_t start, uint16_t end, EMM_IODT* inputp iodt, uint16_t count, EMM_IOPT* outputp iopt)
{
    //build real mode block
    uint8_t buff[1024] = {0};
    uint16_t offset = 0;

    //header
    {
        //preserve header, write later, need allocate mem block to get linear address
        offset = (uint16_t)(offset+sizeof(EMM_IODT_Header));
        offset = (uint16_t)align(offset, 4);
    }

    //jump table
    uint16_t table_offset = offset;
    {
        EMM_IODT_FarAddr* addr = (EMM_IODT_FarAddr*)malloc(sizeof(EMM_IODT_FarAddr)*count);
        for(int i = 0; i < count; ++i)
        {
            addr[i].offset = (uintptr_t)iodt[i].handler;
            addr[i].segment = 0x08; //second entry in GDT, target cs
            addr[i].port = (uint16_t)iodt[i].port;
            //_LOG("IODT %d offset:%08lx\n", i, addr[i].offset);
        }
        memcpy(buff + offset, addr, sizeof(EMM_IODT_FarAddr)*count);
        free(addr);
        offset = (uint16_t)(offset + sizeof(EMM_IODT_FarAddr)*count);
        offset = (uint16_t)align(offset, 4);
    }

    //handler entry code
    uint16_t code_offset = offset;
    size_t code_size = (uintptr_t)&EMM_IOPortTrap_EndCode - (uintptr_t)&EMM_IOPortTrap_StartCode;
    memcpy_c2d(buff + offset, (void*)(uintptr_t)&EMM_IOPortTrap_StartCode, code_size);
    offset = (uint16_t)(offset+code_size);
    offset = (uint16_t)align(offset, 4);

    //iodt
    uint16_t iodt_offset = offset;
    {
        EMM_IODT_V86* v86iodt = (EMM_IODT_V86*)malloc(sizeof(EMM_IODT_V86)*count);
        for(int i = 0; i < count; ++i)
        {
            v86iodt[i].port = (uint16_t)iodt[i].port;
            v86iodt[i].offset = code_offset;
        }
        memcpy(buff + offset, v86iodt, sizeof(EMM_IODT_V86)*count);
        free(v86iodt);
        offset = (uint16_t)(offset + sizeof(EMM_IODT_V86)*count);
        offset = (uint16_t)align(offset, 4);
    }

#if EMM_IOTRAP_DIRECT_INT //not working in DOS/4GW.
    //function call working in CauseWay but sill freeze in real mode client
    //not working for CWSDPMI (probably all DPMI hosts), since simulate INT direct call the addr in ivt which makes cs!=ds
    uint16_t installfunc_offset = 0;
#else
    //installation code
    uint16_t installfunc_offset = offset;
    size_t installcode_size = (uintptr_t)&EMM_IOPortTrap_Install_V86End - (uintptr_t)&EMM_IOPortTrap_Install_V86;
    memcpy_c2d(buff + offset, (void*)(uintptr_t)&EMM_IOPortTrap_Install_V86, installcode_size);
    offset = (uint16_t)(offset + installcode_size);
    //_LOG("installation code size:%d\n", installcode_size);
#endif

    offset = (uint16_t)align(offset, 16);
    assert(offset >= 16 && offset <= 1024);
    //_LOG("TSR memory: %d bytes\n", offset);

    uint32_t mem = DPMI_HighMalloc((uint16_t)(offset>>4), FALSE);//TODO:use UMB to avoid fragments. UMB need paging,
    //another way is to use LH, and then right before TSR, copy this code to overwrite app DOS memory below PSP.
    //but LH may load the app into UMB which need paging too.
    uint32_t segment = (mem&0xFFFFL);

    { //write header
        EMM_IODT_Header header = {0};
        header.gdtr.size = sizeof(header.gdt) - 1;
        header.gdtr.offset = (segment<<4) + offsetof(EMM_IODT_Header, gdt); //linear,paged

        //cannot get himem linear address for EMM386, it has differnet paging from DPMI host
        //move data & code to a known physical address to
        //1.setup custom paging or 2.disable paging
        DPMI_SPACE spc = {0};
        DPMI_GetPhysicalSpace(&spc);
        uint32_t ds_physical_addr = spc.baseds;
        uint32_t cs_physical_addr = spc.basecs;
        uint32_t cs_base = cs_physical_addr; //temmporarily disable paging in the handler
        uint32_t ds_base = ds_physical_addr;
        uint32_t ds_limit = ((spc.limitds+1)>>12)-1;
        uint32_t cs_limit = ((spc.limitcs+1)>>12)-1;

        //target cs
        header.gdt[1].accessed = 0;
        header.gdt[1].read_write = 1;
        header.gdt[1].CE = 0;
        header.gdt[1].type = 1;
        header.gdt[1].nonsystem = 1;
        header.gdt[1].privilege = 0;
        header.gdt[1].present = 1;
        header.gdt[1].available = 0;
        header.gdt[1].zero = 0;
        #if defined(__BC__)    //always use 16bit seg for BC, even in protected mode.
        header.gdt[1].bits32 = 0;
        #else
        header.gdt[1].bits32 = 1;
        #endif
        header.gdt[1].granuarity = 1;
        header.gdt[1].limit_low = (uint16_t)(cs_limit&0xFFFF);
        header.gdt[1].limit_high = (cs_limit >> 16)&0xF;
        header.gdt[1].base_low = (uint16_t)(cs_base&0xFFFF);
        header.gdt[1].base_middle = (uint8_t)(cs_base>>16);
        header.gdt[1].base_high = (uint8_t)(cs_base>>24);
        //_LOG("CS base %08lx, limit: %08lx\n", cs_base, cs_limit);

        //target ds
        header.gdt[2] = header.gdt[1];
        header.gdt[2].CE = 0;   //expand up
        header.gdt[2].type = 0; //data
        header.gdt[2].limit_low = (uint16_t)(ds_limit&0xFFFF);
        header.gdt[2].limit_high = (ds_limit >> 16)&0xF;
        header.gdt[2].base_low = (uint16_t)(ds_base&0xFFFF);
        header.gdt[2].base_middle = (uint8_t)(ds_base>>16);
        header.gdt[2].base_high = (uint8_t)(ds_base>>24);

        //code segment after switching GDT, 64K limit
        header.gdt[3] = header.gdt[1];
        header.gdt[3].limit_low = 0xFFFF;
        header.gdt[3].limit_high = 0xF;
        header.gdt[3].base_low = (uint16_t)(segment<<4);
        header.gdt[3].base_middle = (uint8_t)(segment>>12);
        header.gdt[3].base_high = 0;
        header.gdt[3].bits32 = 0;
        header.gdt[3].granuarity = 0;
        #if !defined(__WC__)    //watcom inline asm doesn't support use16, we need to set segment to 32bit
                                //TODO: not working. EMM386 set a 16 bit segment for the handler code
        #endif

        //data segment after switching GDT, based at allocated memory start, 64k limit
        header.gdt[4] = header.gdt[2];
        header.gdt[4].limit_low = 0xFFFF;
        header.gdt[4].limit_high = 0xF;
        header.gdt[4].base_low = (uint16_t)(segment<<4);
        header.gdt[4].base_middle = (uint8_t)(segment>>12);
        header.gdt[4].base_high = 0;
        header.gdt[4].bits32 = 0;
        header.gdt[4].granuarity = 0;

        //4G selector for driver
        header.gdt[5] = header.gdt[2];
        header.gdt[5].limit_low = 0xFFFF;
        header.gdt[5].limit_high = 0xF;
        header.gdt[5].base_low = 0;
        header.gdt[5].base_middle = 0;
        header.gdt[5].base_high = 0;
        header.gdt[5].bits32 = 1;
        header.gdt[5].granuarity = 1;

        //_LOG("wrapper: %08x, physical:%08x\n", (uintptr_t)&EMM_IOPortTrap_Wrapper, spc.basecs + (uintptr_t)&EMM_IOPortTrap_Wrapper);
        header.esp = spc.stackpointer;
        header.code_offset = code_offset;
        header.wrapper = (uintptr_t)&EMM_IOPortTrap_Wrapper;
        header.tableCount = count;
        header.tableOffset = table_offset;
        memcpy(buff, &header, sizeof(header));

        EMM_Addressing.selector = 0x28; //5th sel in gdt. 4G
        EMM_Addressing.physical = TRUE;
    }
    DPMI_CopyLinear(segment << 4, DPMI_PTR2L(buff), offset);
    //_LOG("%08lx %08lx, %d\n", segment<<4, DPMI_PTR2L(buff), offset);

    _LOG("io trap entry: %04lx:%04x, linear entry: %08lx\n", segment, code_offset, (segment<<4L)+code_offset);
    _LOG("installation entry: %04lx:%04x\n", segment, installfunc_offset);
    //DBG_DumpB(buff, offset);
    //DBG_DumpLB(DPMI_PTR2L(buff)+installfunc_offset, installcode_size);
    //DBG_DumpLB((segment<<4L)+installfunc_offset, installcode_size);
    //DBG_DumpLB((segment<<4L)+table_offset, sizeof(EMM_IODT_FarAddr)*count);
    //DBG_DumpLB((segment<<4L)+iodt_offset, sizeof(EMM_IODT_V86)*count);
    //DBG_DumpLB((segment<<4L)+code_offset, code_size);

    DPMI_REG r = {0};
    r.w.bx = 0;
    r.w.dx = start;
    r.w.hdx= end;
    r.w.cx = count;
    r.w.ds = (uint16_t)segment;
    r.w.si = iodt_offset;
    r.w.di = offset; //size

    r.w.cs = r.w.ds; //cs==ds or it will fail
    r.w.ip = installfunc_offset;
    r.w.flags = CPU_CFLAG; //EMM386 will not set CF on failure, only clear it on success.

#if EMM_IOTRAP_DIRECT_INT
    if(DPMI_CallRealModeINT(0x2F, &r) != 0 || (r.flags&CPU_CFLAG))
#else
    //not supported by DOS/4GW, supported by Causeway or DOS/4GW Professional. Causeway still freeze on client
    if(DPMI_CallRealModeRETF(&r) != 0 || (r.w.flags&CPU_CFLAG))
#endif
    {
        //DBG_DumpREG(&r);
        memset(iopt, 0, sizeof(*iopt));
        return FALSE;
    }

    iopt->memory = mem;
    iopt->func = installfunc_offset;
    iopt->handle = r.w.ax;
    return TRUE;
}

BOOL EMM_Uninstall_IOPortTrap(EMM_IOPT* inputp iopt)
{
    BOOL result = TRUE;
    DPMI_REG r = {0};
    r.w.bx = 1;
    r.w.si = iopt->handle;
    r.w.ds = (uint16_t)(iopt->memory&0xFFFFL);
    r.w.cs = r.w.ds;
    r.w.ip = iopt->func;
    r.w.flags = CPU_CFLAG;

    if(DPMI_CallRealModeRETF(&r) != 0 || (r.w.flags&CPU_CFLAG))
    {
        //DBG_DumpREG(&r);
        result = FALSE;
    }
    DPMI_HighFree(iopt->memory);
    return result;
}