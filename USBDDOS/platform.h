#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#if defined(__BORLANDC__) || defined(__BCPLUSPLUS__)
#define __BC__ 1
#elif defined(__WATCOMC__) || defined(__WATCOM_CPLUSPLUS__)
#define __WC__ 1
#elif defined(__DJGPP__)
#if (__DJGPP__ == 2)
#define __DJ2__ 1
#endif
#endif

//compiler specific preprossor directive
#if defined(__BC__)

#if !defined(__TINY__) && !defined(__SMALL__) && !defined(__MEDIUM__)
#error only tiny/small/medium model supported.
#endif

#include <stddef.h>
#include <stdlib.h>    //min max
typedef char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef long int32_t;
typedef unsigned long uint32_t;
//typedef long long int64_t; //compiled, but not correct
//typedef unsigned long long uint64_t;

typedef int16_t intptr_t;   //only use tiny or small model. far pointer not supported by this type.
typedef uint16_t uintptr_t;

#define __NAKED //not supported on BC3.1. uncheck 'standard stack frame' might work but it is implicit: no frame when no param&local vars.
                //#pragma option -k- (no standard stack frame) might work depending on above but it needs an ending pragma
#define __CDECL __cdecl
#ifdef __cplusplus
#define __INLINE inline
#else
#define __INLINE
#endif

#define _ASM_BEGIN __asm {
#define _ASM_END }
#define _ASM(x) x;
#define _ASMLBL(x) }x __asm { x //the first label makes compiler happy and the second used by TASM when offset used
#define _ASM2(x,y) x,y;
#define _ASM_OPERAND_SIZE _ASM(db 0x66)  //switch oprand size, 32bit to 16bit or 16 to 32
//#define _ASM_RETF _ASM(leave) _ASM(retf) //hack to naked function. use on function return, DO NOT use on indirect jump //now disable stack frame by default. any function without parameters&localvars will have no ebp stup.
#define _ASM_LIDT(x) _ASM(lidt fword ptr x)
#define _ASM_LGDT(x) _ASM(lgdt fword ptr x)
#define _ASM_SIDT(x) _ASM(sidt fword ptr x)
#define _ASM_SGDT(x) _ASM(sgdt fword ptr x)

#define _ASM_BEGIN16 _ASM_BEGIN
#define _ASM_END16 _ASM_END
//this is hack to origianl assembly code. need -B (compile via assembler) for BC
#define _ASM_BEGIN32 _ASM_BEGIN16 _TEXT ends; _TEXT32 segment public use32 'CODE'; assume cs:_TEXT32;
#define _ASM_END32 _TEXT32 ends; _TEXT segment; assume cs:_TEXT;_ASM_END16

#define NOP() __asm nop
#define CLI() __asm cli
#define STI() __asm sti
extern uint32_t PLTFM_BSF(uint32_t x); //386+ (386 included)
#define PLTFM_CPU_FLAGS() _FLAGS

#elif defined(__DJ2__)
#include <stdint.h>
#include <stddef.h>

#define __NAKED __attribute__((naked))
#define __CDECL __attribute__((cdecl))
#define __INLINE inline

//looks ugly. only if we can work preprocessing with raw string literals (R"()")
//raw string can work with preprocessor using gcc -E or cpp in the triditional way. need a special pass for file with asm
#define _ASM_BEGIN __asm__ __volatile__(".intel_syntax noprefix\n\t" 
#define _ASM_END ".att_syntax noprefix");
#define _ASM(...) #__VA_ARGS__"\n\t"
#define _ASMLBL _ASM
#define _ASM2 _ASM
#define _ASM_OPERAND_SIZE _ASM(.byte 0x66)  //switch oprand size, 32bit to 16bit or 16 to 32
#define _ASM_LIDT(x) _ASM(lidt x)
#define _ASM_LGDT(x) _ASM(lgdt x)
#define _ASM_SIDT(x) _ASM(sidt x)
#define _ASM_SGDT(x) _ASM(sgdt x)

#define _ASM_BEGIN16 _ASM_BEGIN ".code16\n\t" 
#define _ASM_END16 ".code32\n\t" _ASM_END
#define _ASM_BEGIN32 _ASM_BEGIN
#define _ASM_END32 _ASM_END

#define NOP() __asm__ __volatile__("nop")
#define CLI() __asm__ __volatile__("cli")
#define STI() __asm__ __volatile__("sti")
static inline uint32_t PLTFM_BSF(uint32_t x) {uint32_t i; __asm__("bsf %1, %0" : "=r" (i) : "rm" (x)); return i;} //386+
static inline uint16_t PLTFM_CPU_FLAGS_ASM(void) { uint32_t flags = 0; __asm__("pushf\n\t" "pop %0\n\t" : "=r"(flags)); return (uint16_t)flags; }
static inline uint16_t PLTFM_CPU_FLAGS() { uint16_t (* volatile VFN)(void) = &PLTFM_CPU_FLAGS_ASM; return VFN();} //prevent optimization, need get FLAGS every time

#elif defined(__WC__)

#if !defined(__TINY__) && !defined(__SMALL__) && !defined(__MEDIUM__)
#error only tiny/small/medium model supported.
#endif

#include <stdint.h>
#include <stddef.h>

#define __NAKED __declspec(naked)
#define __CDECL __cdecl
#define __INLINE inline

#define _ASM_BEGIN __asm {
#define _ASM_END }
#define _ASM(...) __VA_ARGS__;
#define _ASMLBL _ASM
#define _ASM2 _ASM
#define _ASM_OPERAND_SIZE _ASM(db 0x66)  //switch oprand size, 32bit to 16bit or 16 to 32
#define _ASM_LIDT(x) _ASM(lidt fword ptr x)
#define _ASM_LGDT(x) _ASM(lgdt fword ptr x)
#define _ASM_SIDT(x) _ASM(sidt fword ptr x)
#define _ASM_SGDT(x) _ASM(sgdt fword ptr x)

//wc inline asm doesn't support use16 codes, except .286 but that will disble 32bit registers too
#define _ASM_BEGIN16 _ASM_BEGIN //not supported. TODO:
#define _ASM_END16 _ASM_END //not supported. TODO:
#define _ASM_BEGIN32 _ASM_BEGIN 
#define _ASM_END32 _ASM_END

#define NOP() __asm {nop}
#define CLI() __asm {cli}
#define STI() __asm {sti}

inline static uint32_t PLTFM_BSF(uint32_t x) { uint32_t r = 0;
    __asm {
        push eax //better not touch high word of eax
        bsf eax, x
        mov r, eax
        pop eax
    }
    return r;
}

uint16_t _AX_ASM();
#pragma aux _AX_ASM = \
value[ax]
#define _AX _AX_ASM()

uint8_t _AL_ASM();
#pragma aux _AL_ASM = \
value[al]
#define _AL _AL_ASM()

uint16_t _DS_ASM();
#pragma aux _DS_ASM = \
"mov ax, ds" \
value[ax]
#define _DS _DS_ASM()

uint16_t _ES_ASM();
#pragma aux _ES_ASM = \
"mov ax, es" \
value[ax]
#define _ES _ES_ASM()

uint16_t _SS_ASM();
#pragma aux _SS_ASM = \
"mov ax, ss" \
value[ax]
#define _SS _SS_ASM()

uint16_t _SP_ASM();
#pragma aux _SP_ASM = \
value[sp]
#define _SP _SP_ASM()

uint16_t _BP_ASM();
#pragma aux _BP_ASM = \
value[bp]
#define _BP _BP_ASM()

uint16_t _CS_ASM();
#pragma aux _CS_ASM = \
"mov ax, cs" \
value[ax]
#define _CS _CS_ASM()

#if DEBUG
#include <dos.h>
inline const uint16_t far* DBG_STK()
{
    return (uint16_t far*)MK_FP(_SS, _SP);
}
#endif

static __NAKED uint16_t PLTFM_CPU_FLAGS_ASM(void) { __asm {
    pushf;
    pop ax;
    ret
}}
typedef uint16_t (*PFN_PLTFM_CPU_FLAGS_ASM)(void);
static const volatile PFN_PLTFM_CPU_FLAGS_ASM pfnPLTFM_CPU_FLAGS_ASM = &PLTFM_CPU_FLAGS_ASM;  //prevent optimization, need get FLAGS every time
static inline uint16_t PLTFM_CPU_FLAGS() { return pfnPLTFM_CPU_FLAGS_ASM();}

#else //stub
#if !defined(_WIN32) && !defined(__linux__)   //make editor happy
#error "Not supported."
#endif
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

#define __NAKED
#define __CDECL
#define __INLINE

//make text editor happy. i.e. vscode
#define _ASM_BEGIN { 
#define _ASM_END }
#define _ASM(...) 
#define _ASM2 _ASM
#define _ASMLBL _ASM

#define _ASM_BEGIN16 _ASM_BEGIN
#define _ASM_END16 _ASM_END
#define _ASM_BEGIN32 _ASM_BEGIN
#define _ASM_END32 _ASM_END
#define _ASM_OPERAND_SIZE
#define _ASM_LIDT(x) 
#define _ASM_LGDT(x) 
#define _ASM_SIDT(x) 
#define _ASM_SGDT(x) 

//not defined
extern void NOP();
extern void CLI();
extern void STI();
extern uint32_t PLTFM_BSF(uint32_t x);
extern uint16_t PLTFM_CPU_FLAGS(void);

extern void delay(int);
extern uint8_t inp(uint16_t port);
extern uint16_t inpw(uint16_t port);
extern uint32_t inpd(uint16_t port);
extern void outp(uint16_t port, uint8_t val);
extern void outpw(uint16_t port, uint16_t val);
extern void outpd(uint16_t port, uint32_t val);
extern int _dos_open(const char* file, int mode, int* fd);
extern int _dos_close(int fd);
extern int _fstrlen(const char*);
extern int _fmemcmp(const void*, const void*, int);
extern int _bios_keybrd(int);
extern void _dos_keep(int code, int para);

extern uint16_t _ES;
extern uint16_t _DS;
extern uint16_t _CS;
extern uint16_t _SS;
extern uint16_t _SP;
extern uint16_t _BP;
extern uint16_t _AX;
extern uint8_t _AH;
extern uint8_t _AL;

#define __far
#define far
#define _FAR 
#define near 
#define _FARFUNC 
#define _Cdecl 
#define _pascal
#define stackavail() 0
#define _psp 0

#define _fmemset memset
#define _fmemcpy memcpy

#define FP_SEG(x) ((uintptr_t)(x))
#define FP_OFF(x) ((uintptr_t)(x))
#define MK_FP(x,y) (NULL)

#define static_assert(e, m) extern int static_assert_xxxx

#endif //compiler specific preprossor directive

#if defined(__DJ2__)
#define static_assert _Static_assert
#elif defined(__BC__) || defined(__WC__)

#define static_assert_CONCAT2(a, b) a ## b
#define static_assert_CONCAT(a, b) static_assert_CONCAT2(a, b)

#ifdef __cplusplus
#define static_assert(e,m) extern "C" char static_assert_CONCAT(static_assert,__LINE__)[(e) ? 1 : -1]
#else
#define static_assert(e,m) extern char static_assert_CONCAT(static_assert,__LINE__)[(e) ? 1 : -1]
#endif

#endif//defined(__DJ2__)

//general preprossor directive

#ifndef DEBUG
#define DEBUG 0
#elif DEBUG != 0
#undef DEBUG
#define DEBUG 1
#endif

#ifdef __cplusplus
#define __EXTERN extern "C"
#else
#define __EXTERN extern
#endif

#define nullable
#define outputp
#define inoutp
#define inputp

#define unused(x) (void)x

#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof(x[0]))
#endif

//align down
#define align(x,a) ((uint32_t)((x)+(a)-1)&(uint32_t)(~((uint32_t)(a)-1)))
//align up
#define alignup(x,a) ((uint32_t)((x))&(uint32_t)(~((uint32_t)(a)-1)))

typedef enum {FALSE, TRUE} BOOLEAN;
typedef int BOOL;

#undef min
#undef max
#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))

static __INLINE uint16_t EndianSwap16(uint16_t x) {return (uint16_t)((x<<8) | (x>>8)); }
static __INLINE uint32_t EndianSwap32(uint32_t x) {return (x<<24) | ((x<<8)&0xFF0000UL) | ((x>>8)&0xFF00UL) | (x>>24); }

#define CPU_CFLAG 0x0001    //carry flag (CF)
#define CPU_IFLAG 0x0200    //interrupt flag (IF)
#define CPU_ZFLAG 0x0040    //zero flag
#define CPU_TFLAG 0x0100    //trap flag

#define BSF PLTFM_BSF
#define CPU_FLAGS() PLTFM_CPU_FLAGS()

//CLI is not enough. this works for normal code. but driver code during intrrupt (which keeps IF always 0) don't do CLI
//CLIS will store IF and do cli.
#define CLIS() int FLTFM_IFlag = (CPU_FLAGS()&CPU_IFLAG); CLI()
//STIL will load IF
#define STIL() do { if(FLTFM_IFlag) STI(); } while(0)

//Intel® 64 and IA-32 Architectures Software Developer's Manual, Volume 3A, 3.4.5 (3-9)
typedef struct DESCRIPTOR //segment descriptors
{
    uint16_t limit_low;
    uint16_t base_low;
    uint8_t base_middle;

    uint8_t accessed : 1;
    uint8_t read_write : 1; //read for code, write for data
    uint8_t CE : 1;     //code: conforming(1), data: expand down (1)
    uint8_t type : 1;   //1 code, 0 data
    uint8_t nonsystem : 1;  //S: 0: system, 1: code or data
    uint8_t privilege : 2;  //DPL
    uint8_t present : 1;    //P

    uint8_t limit_high : 4;
    uint8_t available : 1;  //AVL: available for programmer
    uint8_t zero: 1;    //always 0
    uint8_t bits32 : 1; //16 or 32
    uint8_t granuarity : 1; //0: 1B, 1: 4K

    uint8_t base_high;
}GDT,LDT;
static_assert(sizeof(GDT) == 8, "size error");

//Intel® 64 and IA-32 Architectures Software Developer's Manual, Volume 3A, (6-11)
//gate type
#define IDT_GT_TASK 0x5   //task gate, offset should be 0
#define IDT_GT_IG 0x6     //interrupt gate
#define IDT_GT_TG 0x7     //trap gate
typedef struct _DIT
{
    uint16_t offset_low;
    uint16_t selector;
    uint8_t reserved;
    uint8_t gate_type : 3;
    uint8_t bit32 : 1;
    uint8_t zero2 : 1;
    uint8_t privilege : 2;
    uint8_t present : 1;
    uint16_t offset_high;
}IDT;
static_assert(sizeof(IDT) == 8, "size error");

typedef struct DESC_PTR
{
    //easy alignment for 32 bit code
    uint16_t unused; //make clear of size/alignment, care about the offset (2)
    uint16_t size;
    uint32_t offset; //linear offset (page mapped)
}GDTR, LDTR, IDTR;

//Intel® 64 and IA-32 Architectures Software Developer's Manual, Volume 3A, Charpter 4: Paging
typedef union _PTE_PDE//bc3.1 doesn't support 32bit bitfields
{
    uint32_t value;
    struct
    {
        uint16_t present : 1;       //
        uint16_t writable : 1;      //R/W
        uint16_t user : 1;          //1: user, 0: system
        uint16_t writethrough : 1;  //PWT
        uint16_t disable_cache: 1;  //PCD
        uint16_t accessed : 1;      //A
        uint16_t dirty : 1;         //D
        uint16_t PS_PAT : 1;        //PS page size: 4M for pde, need Pentium Pro (CPUID). PAT Page Attribute Table: for pte(PIII+). both not used.
        uint16_t global: 1;         //G valid when PGE(bit7) set in cr4
        uint16_t ignored : 3;       //user feilds
        uint16_t address_low : 4;   //
        uint16_t address_high;
    }bits;
}PDE,PTE;
static_assert(sizeof(PDE) == 4, "size error");

#define PDE_INIT(addr) { ((addr)&~0xFFFL) | 0x7L }
#define PDE_ADDR(pde) ((pde).value&~0xFFFL)
//for old compiler support
#define PDE_INITA(pde, addr) do{\
    (pde).value = (((uint32_t)(addr))&~0xFFFL) | 0x7L;\
}while(0)

#define PTE_INIT PDE_INIT
#define PTE_INITA PDE_INITA
#define PTE_ADDR PDE_ADDR

//Intel® 64 and IA-32 Architectures Software Developer's Manual, Volume 3A, 7.2.1 (7-3)
typedef struct _TSS32
{
    uint16_t prev;  //previous task link
    uint16_t reserved;
    uint32_t esp0;
    uint16_t ss0;
    uint16_t reserved0;
    uint32_t esp1;
    uint16_t ss1;
    uint16_t reserved1;
    uint32_t esp2;
    uint16_t ss2;
    uint16_t reserved2;
    uint32_t cr3;   //cr3 register, task paging
    uint32_t eip;
    uint32_t eflags;
    uint32_t eax;
    uint32_t ecx;
    uint32_t edx;
    uint32_t ebx;
    uint32_t esp;
    uint32_t ebp;
    uint32_t esi;
    uint32_t edi;
    struct
    {
        uint16_t es;
        uint16_t reserved3;
        uint16_t cs;
        uint16_t reserved4;
        uint16_t ss;
        uint16_t reserved5;
        uint16_t ds;
        uint16_t reserved6;
        uint16_t fs;
        uint16_t reserved7;
        uint16_t gs;
        uint16_t reserved8;
        uint16_t ldt_sel;
        uint16_t reserved9;
    }sel;
    uint16_t trap : 1; //debug trap
    uint16_t reserved10 : 15;
    uint16_t iomap;
    uint32_t ssp;   //shadow stack pointer
}TSS32;

typedef struct _TSS16
{
    uint16_t prev;  //previous task link
    uint16_t sp0;
    uint16_t ss0;
    uint16_t sp1;
    uint16_t ss1;
    uint16_t sp2;
    uint16_t ss2;
    uint16_t ip;
    uint16_t flags;
    uint16_t ax;
    uint16_t cx;
    uint16_t dx;
    uint16_t bx;
    uint16_t sp;
    uint16_t bp;
    uint16_t si;
    uint16_t di;
    struct
    {
        uint16_t es;
        uint16_t cs;
        uint16_t ss;
        uint16_t ds;
        uint16_t ldt_sel;
    }sel16;
}TSS16;

#endif
