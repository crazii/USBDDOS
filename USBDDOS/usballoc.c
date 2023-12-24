#include <assert.h>
#include "USBDDOS/usballoc.h"
#include "USBDDOS/dbgutil.h"

#if USBALLOC_ENABLE

#define USBALLOC_FLAG_USED  0x01
#define USBALLOC_FLAG_32    0x02

#define USBALLOC_MEMORY_SIZE (1*1024)

//the code need more test, not enabled for now. the way of pooling waste memory and may cause problem
#define USBALLOC_DEBUG 1    //use DPMI_DMAMalloc directly (probably with more debug info)

#if defined(__BC__)
#define USBALLOC_MEMORY_COUNT 16
#undef USBALLOC_DEBUG
#define USBALLOC_DEBUG 1
#else
#define USBALLOC_MEMORY_COUNT 64
#endif

#define USBALLOC_PREALLOC (0 && !USBALLOC_DEBUG)

typedef struct
{
    uint8_t* arena;
    union
    {
        uint32_t mask;
        struct
        {
            uint16_t offset;
            uint16_t left;
        }buff;
    };
    uint16_t flags;
    uint16_t index;
}USBALLOC_Memory;

static USBALLOC_Memory USBALLOC_Pool[USBALLOC_MEMORY_COUNT];
static USBALLOC_Memory* USBALLOC_CurrentBuf = NULL;
static USBALLOC_Memory* USBALLOC_Current32 = NULL;

static USBALLOC_Memory* USBALLOC_GetMemory(void)
{
    int i;
    for(i = 0; ; ++i) //will wait for interrupt to release finished memory in dead loop
    { //if alreay in interrupt, it'll be dead. i.e. EMM386 IO port trap with interrupt disabled.
        USBALLOC_Memory* memory = &USBALLOC_Pool[i%USBALLOC_MEMORY_COUNT];
        if(!(memory->flags&USBALLOC_FLAG_USED))
        {
            #if USBALLOC_PREALLOC
            assert(memory->arena != NULL);
            #else
            assert(memory->arena == NULL);
            memory->arena = (uint8_t*)DPMI_DMAMalloc(USBALLOC_MEMORY_SIZE, 32); //alignment to 32 for EHCI
            #endif
            //_LOG("memory arena: %08lx\n", ((uint32_t)(memory->arena)));
            assert((((uint32_t)(memory->arena))&0x1F) == 0);
            memory->flags = USBALLOC_FLAG_USED;
            memory->index = (uint16_t)(i%USBALLOC_MEMORY_COUNT);
            return memory;
        }
    }
    return NULL;
}

static void USBALLOC_ReturnMemory(USBALLOC_Memory* memory)
{
    assert(memory->arena);
#if !USBALLOC_PREALLOC
    DPMI_DMAFree(memory->arena);
    memory->arena = NULL;
#endif
    memory->mask = 0;
    memory->flags = 0;
}

void USBALLOC_Init(void)
{
    assert(USBALLOC_CurrentBuf == NULL);
    assert(USBALLOC_Current32 == NULL);
    //USBALLOC_CurrentBuf = USBALLOC_GetMemoryBuf();
    //USBALLOC_Current32 = USBALLOC_GetMemory32();
#if USBALLOC_PREALLOC
    for(int i = 0; i < USBALLOC_MEMORY_COUNT; ++i)
    {
        USBALLOC_Pool[i].arena = (uint8_t*)DPMI_DMAMalloc(USBALLOC_MEMORY_SIZE, 32);
        assert(USBALLOC_Pool[i].arena);
    }
#endif
}

void USBALLOC_Shutdown(void)
{
    //assert(USBALLOC_CurrentBuf != NULL);
    //assert(USBALLOC_Current32 != NULL);
    USBALLOC_CurrentBuf = NULL;
    USBALLOC_Current32 = NULL;
    int i;
    for(i = 0; i < USBALLOC_MEMORY_COUNT; ++i)
    {
        USBALLOC_Memory* memory = &USBALLOC_Pool[i];
#if USBALLOC_PREALLOC
        DPMI_DMAFree(memory->arena);
#else
        if((memory->flags&USBALLOC_FLAG_USED))
            USBALLOC_ReturnMemory(memory);
#endif
    }
}


void* USBALLOC_TransientAlloc(uint16_t size, uint16_t alignment)
{
    #if USBALLOC_DEBUG
    {
        CLIS();
        void* ptr = DPMI_DMAMalloc(size, alignment);
        STIL();
        return ptr;
    }
    #endif

    CLIS();
    alignment = max(alignment, 4);
    uint16_t actual = (uint16_t)(size + alignment + 4);
    uint16_t index = USBALLOC_CurrentBuf->index;
    uint8_t* ptr = NULL;
    if(!USBALLOC_CurrentBuf || USBALLOC_MEMORY_SIZE - USBALLOC_CurrentBuf->buff.offset < actual)
    {
        if(actual > USBALLOC_MEMORY_SIZE/2)
        {
            ptr = (uint8_t*)DPMI_DMAMalloc(actual, 1) + 4;
            index = 0xFFFF;
        }
        else
        {
            USBALLOC_CurrentBuf = USBALLOC_GetMemory();
            USBALLOC_CurrentBuf->buff.offset = 0;
            USBALLOC_CurrentBuf->buff.left = USBALLOC_MEMORY_SIZE;
            index = USBALLOC_CurrentBuf->index;
        }
    }
    if(!ptr)
    {
        ptr = USBALLOC_CurrentBuf->arena + USBALLOC_CurrentBuf->buff.offset + 4;
        USBALLOC_CurrentBuf->buff.offset = (uint16_t)(USBALLOC_CurrentBuf->buff.offset + actual);
        assert(USBALLOC_CurrentBuf->buff.offset <= USBALLOC_MEMORY_SIZE);
        assert(USBALLOC_CurrentBuf->buff.left >= actual);
        USBALLOC_CurrentBuf->buff.left = (uint16_t)(USBALLOC_CurrentBuf->buff.left - actual);
    }
    uint32_t addr = DPMI_PTR2L(ptr);
    uint32_t offset = align(addr, alignment) - addr;
    uint16_t* aptr = (uint16_t*)(ptr + offset);
    aptr[-1] = index;
    aptr[-2] = (index != 0xFFFF) ? actual : (uint16_t)(offset + 4);
    STIL();
    return aptr;
}

void USBALLOC_TransientFree(void* ptr)
{
    #if USBALLOC_DEBUG
    {
        CLIS();
        DPMI_DMAFree(ptr);
        STIL();
        return;
    }
    #endif

    uint16_t index = ((uint16_t*)ptr)[-1];
    uint16_t actual = ((uint16_t*)ptr)[-2];
    if(index == 0xFFFF)
    {
        CLIS();
        DPMI_DMAFree((uint8_t*)ptr - actual);
        STIL();
        return;
    }
    CLIS();
    assert(index < USBALLOC_MEMORY_COUNT);
    USBALLOC_Memory* memory = &USBALLOC_Pool[index];
    memory->buff.left = (uint16_t)(memory->buff.left + actual);
    assert(USBALLOC_CurrentBuf->buff.left <= USBALLOC_MEMORY_SIZE);

    if(memory->buff.left == USBALLOC_MEMORY_SIZE)
    {
        memory->buff.offset = 0;
        if(memory != USBALLOC_CurrentBuf)
            USBALLOC_ReturnMemory(memory);
        else
        {
            int i;
            for(i = 0; i < USBALLOC_MEMORY_COUNT; ++i)
            {
                USBALLOC_Memory* memory = &USBALLOC_Pool[i];
                if(memory != USBALLOC_CurrentBuf && (memory->flags&(USBALLOC_FLAG_USED)) && !(memory->flags&(USBALLOC_FLAG_32)) && (USBALLOC_MEMORY_SIZE-memory->buff.offset) >= USBALLOC_MEMORY_SIZE/4)
                {
                    assert(memory->arena);
                    USBALLOC_ReturnMemory(USBALLOC_CurrentBuf);
                    USBALLOC_CurrentBuf = memory;
                    break;
                }
            }
        }
    }
    STIL();
}

void* USBALLOC_TransientAlloc32(uint16_t sizeverify)
{
    if(sizeverify > 32)
    {
        assert(FALSE);
        return NULL;
    }

    #if USBALLOC_DEBUG
    {
        CLIS();
        sizeverify = max(32, sizeverify);
        void* ptr = DPMI_DMAMalloc(sizeverify, 32);
        for(int i = 0; i < 8; ++i)
            ((int32_t*)ptr)[i] = 0x0BAD0ACE;
        STIL();
        return ptr;
    }
    #endif

    CLIS();
    if(USBALLOC_Current32 == NULL || USBALLOC_Current32->mask == 0)
    {
        USBALLOC_Current32 = USBALLOC_GetMemory();
        assert(USBALLOC_Current32->arena);
        assert((((uint32_t)(USBALLOC_Current32->arena))&0x1F) == 0);
        assert(USBALLOC_Current32->flags&USBALLOC_FLAG_USED);
        USBALLOC_Current32->mask = 0xFFFFFFFFL;
        USBALLOC_Current32->flags |= USBALLOC_FLAG_32;
    }
    register uint16_t index = (uint16_t)BSF(USBALLOC_Current32->mask);
    //_LOG("M32 Mask %08lx ", USBALLOC_Current32->mask);
    USBALLOC_Current32->mask &= ~(1UL << index);
    //_LOG("%08lx\n", USBALLOC_Current32->mask);
    STIL();
    return USBALLOC_Current32->arena + (index << 5);
}

void USBALLOC_TransientFree32(void* ptr)
{
    #if USBALLOC_DEBUG
    {
        CLIS();
        for(int i = 0; i < 8; ++i)
            ((uint32_t*)ptr)[i] = 0xF00DFEEDU;
        DPMI_DMAFree(ptr);
        STIL();
        return;
    }
    #endif

    CLIS();
    assert((((uint32_t)ptr)&0x1F) == 0);
    if(USBALLOC_Current32 && ((uint8_t*)ptr) >= USBALLOC_Current32->arena && ((uint8_t*)ptr) < USBALLOC_Current32->arena + USBALLOC_MEMORY_SIZE)
    {
        assert(USBALLOC_Current32->flags&(USBALLOC_FLAG_USED|USBALLOC_FLAG_32));
        assert(USBALLOC_Current32->arena);
        uint16_t index = (uint16_t)((uint8_t*)ptr - USBALLOC_Current32->arena)>>5;
        assert(index < 32);
        //_LOG("FreeM32 Mask %08lx ", USBALLOC_Current32->mask);
        assert(USBALLOC_Current32->mask | (1UL << index) > USBALLOC_Current32->mask);
        USBALLOC_Current32->mask |= (1UL << index);
        //_LOG("%08lx\n", USBALLOC_Current32->mask);
        if(USBALLOC_Current32->mask != 0xFFFFFFFFL)
        {
            STIL();
            return;
        }

        int i;
        for(i = 0; i < USBALLOC_MEMORY_COUNT; ++i)
        {
            USBALLOC_Memory* memory = &USBALLOC_Pool[i];
            if(memory != USBALLOC_Current32 && (memory->flags&(USBALLOC_FLAG_USED|USBALLOC_FLAG_32)) && memory->mask != 0)
            {
                USBALLOC_ReturnMemory(USBALLOC_Current32);
                USBALLOC_Current32 = memory;
                break;
            }
        }
        STIL();
        return;
    }

    int i;
    for(i = 0; i < USBALLOC_MEMORY_COUNT; ++i)
    {
        USBALLOC_Memory* memory = &USBALLOC_Pool[i];
        if(memory != USBALLOC_Current32 && (memory->flags&(USBALLOC_FLAG_USED|USBALLOC_FLAG_32)))
        {
            assert(memory->arena);
            if(((uint8_t*)ptr) >= memory->arena && ((uint8_t*)ptr) < memory->arena + USBALLOC_MEMORY_SIZE)
            {
                uint16_t index = (uint16_t)(((uint8_t*)ptr - memory->arena)>>5);
                memory->mask |= 1U << index;
                if(memory->mask == 0xFFFFFFFFL)
                    USBALLOC_ReturnMemory(memory);
                STIL();
                return;
            }
        }
    }
    STIL();
    assert(FALSE);
}

#endif //USBALLOC_ENABLE