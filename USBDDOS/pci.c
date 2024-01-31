#include <string.h>
#include <assert.h>
#include <dos.h>
#include "USBDDOS/pci.h"
#include "USBDDOS/pic.h"

#define PCI_ADDR  0x0CF8
#define PCI_DATA  0x0CFC
#define ENABLE_BIT 0x80000000

#if defined(__BC__) || defined(__WC__)
uint32_t inpd(uint16_t port)
{
    uint32_t val = 0;
    __asm {
        mov dx, port
        in eax, dx
        mov val, eax
    }
    return val;
}

void outpd(uint16_t port, uint32_t val)
{
    __asm {
        mov   dx, port
        mov   eax, val
        out   dx, eax
    }
}
#endif

uint8_t PCI_ReadByte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    int shift = ((reg & 3) * 8);
    uint32_t val = ENABLE_BIT |
        ((uint32_t)bus << 16) |
        ((uint32_t)dev << 11) |
        ((uint32_t)func << 8) |
        ((uint32_t)reg & 0xFC);
    outpd(PCI_ADDR, val);
    return (inpd(PCI_DATA) >> shift) & 0xFF;
}

uint16_t PCI_ReadWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    if ((reg & 3) <= 2)
    {
        const int shift = ((reg & 3) * 8);
        const uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        return (inpd(PCI_DATA) >> shift) & 0xFFFF;
    }
    else
        return (uint16_t)((PCI_ReadByte(bus, dev, func, (uint8_t)(reg + 1)) << 8) | PCI_ReadByte(bus, dev, func, reg));
}

uint32_t PCI_ReadDWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    if ((reg & 3) == 0)
    {
        uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        return inpd(PCI_DATA);
    }
    else
        return ((uint32_t)PCI_ReadWord(bus, dev, func, (uint8_t)(reg + 2)) << 16L) | PCI_ReadWord(bus, dev, func, reg);
}

void PCI_WriteByte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t value)
 {
    int shift = ((reg & 3) * 8);
    uint32_t val = ENABLE_BIT |
        ((uint32_t)bus << 16) |
        ((uint32_t)dev << 11) |
        ((uint32_t)func << 8) |
        ((uint32_t)reg & 0xFC);
      outpd(PCI_ADDR, val);
      outpd(PCI_DATA, (uint32_t)(inpd(PCI_DATA) & ~(0xFFU << shift)) | ((uint32_t)value << shift));
}


void PCI_WriteWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t value)
{
    if ((reg & 3) <= 2)
    {
        int shift = ((reg & 3) * 8);
        uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        outpd(PCI_DATA, (inpd(PCI_DATA) & ~(0xFFFFU << shift)) | ((uint32_t)value << shift));
    }
    else
    {
        PCI_WriteByte(bus, dev, func, (uint8_t)(reg + 0), (uint8_t)(value & 0xFF));
        PCI_WriteByte(bus, dev, func, (uint8_t)(reg + 1), (uint8_t)(value >> 8));
    }
}


void PCI_WriteDWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t value)
{
    if ((reg & 3) == 0)
    {
        uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        outpd(PCI_DATA, value);
    }
    else
    {
        PCI_WriteWord(bus, dev, func, (uint8_t)(reg + 0), (uint16_t)(value & 0xFFFF));
        PCI_WriteWord(bus, dev, func, (uint8_t)(reg + 2), (uint16_t)(value >> 16));
    }
}

void PCI_ReadDevice(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* device)
{
    memset(device, 0, sizeof(PCI_DEVICE));
    if (PCI_ReadWord(bus, dev, func, 0x00) == 0xFFFF)
        return;
    for(int i = 0; i < 64; ++i)
        device->Registers[i] = PCI_ReadDWord(bus, dev, func, (uint8_t)(i * 4));
}

uint32_t PCI_Sizing(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    //PCI Local Bus Specification Revision 3.0, 6.2.5.1
    //OHCI_Specification_Rev.1.0a
    uint16_t CmdReg;
    uint32_t OldVal;
    uint32_t range;

    CmdReg = PCI_ReadWord(bus, dev, func, 0x04);
    PCI_WriteWord(bus, dev, func, 0x04, (uint16_t)(CmdReg & ~0x03U));

    OldVal = PCI_ReadDWord(bus, dev, func, reg);

    // write 0xFFFFFFFF and read back
    PCI_WriteDWord(bus, dev, func, reg, 0xFFFFFFFF);
    range = PCI_ReadDWord(bus, dev, func, reg);

    // restore
    PCI_WriteDWord(bus, dev, func, reg, OldVal);
    PCI_WriteWord(bus, dev, func, 0x04, CmdReg);

    if (range & 0x1UL)//IO bit
    {
        range &= ~0x1UL; //clear encoding bits (0)
        range |= 0xFFFF0000L;
    }
    else //Mem bit
        range &= ~0xFUL; //clear encoding bits (0-3)
    return (~range) + 1UL;
}


#include "USBDDOS/DPMI/dpmi.h"
#include "USBDDOS/dbgutil.h"
typedef struct
{
    uint16_t size;
    uint16_t off;
    uint16_t seg;
    uint16_t padding;
}IRQRoutingOptionBuffer;

#if defined(__DJ2__)
#pragma pack(1)
#endif
typedef struct
{
    uint8_t bus;
    uint8_t dev; //high 5 bit, low 3 bit unspecified
    struct 
    {
        uint8_t link;
        uint16_t map;
    }intpins[4];
    uint8_t slot;
    uint8_t reserved;
}IRQRoutingTable; //table entry
#if defined(__DJ2__)
#pragma pack()
#endif
static_assert(sizeof(IRQRoutingTable) == 16, "size error");

static BOOL PCI_SetIRQ_Stack(uint8_t bus, uint8_t dev, uint8_t func, uint8_t INTPIN, uint8_t IRQ, uint32_t ss, uint16_t sp, BOOL freestack)
{
    DPMI_REG r = {0};
    r.h.ah = PCI_FUNCTION_ID;
    r.h.al = SET_PCI_IRQ;
    r.h.cl = (uint8_t)(0xA + INTPIN - 1);
    r.h.ch = IRQ;
    r.h.bh = bus;
    r.h.bl = (uint8_t)((func&0x7) | (dev<<3));
    r.w.ds = 0xF000;
    r.w.ss = (uint16_t)(ss&0xFFFFU);
    r.w.sp = sp;
    uint16_t ret = DPMI_CallRealModeINT(PCI_INT_NO, &r);
    _LOG("PCI_SetIRQ: INT%c#->%d: %d %x\n", 'A'+INTPIN-1, IRQ, ret, r.h.ah);
    if(freestack)
        DPMI_DOSFree(ss);
    return ret == 0 && r.h.ah == SUCCESSFUL;
}

uint8_t PCI_AssignIRQ(uint8_t bus, uint8_t dev, uint8_t func, uint8_t INTPIN)
{
    const uint16_t STACKSIZE = 1024;
    uint32_t dosmem = DPMI_DOSMalloc((sizeof(IRQRoutingOptionBuffer)+15)>>4);
    if(dosmem == 0)
        return 0xFF;
    uint32_t dosstack = DPMI_DOSMalloc(STACKSIZE>>4);
    if(dosstack == 0)
    {
        DPMI_DOSFree(dosmem);
        return 0xFF;
    }

    uint32_t seg = (dosmem&0xFFFFU);
    IRQRoutingOptionBuffer buf;
    buf.size = 0;
    buf.off = sizeof(buf);
    buf.seg = (uint16_t)seg;
    DPMI_CopyLinear(seg<<4UL, DPMI_PTR2L(&buf), sizeof(buf));

    DPMI_REG r = {0};
    r.h.ah = PCI_FUNCTION_ID;
    r.h.al = GET_IRQ_ROUTING_OPTIONS;
    r.w.bx = 0;
    r.w.ds = 0xF000;
    r.w.es = (uint16_t)seg;
    r.w.di = 0;
    r.w.ss = (uint16_t)(dosstack&0xFFFFU);
    r.w.sp = STACKSIZE;
    uint16_t ret = DPMI_CallRealModeINT(PCI_INT_NO, &r);
    if(ret == 0 && r.h.ah == BUFFER_TOO_SMALL)
    {
        DPMI_CopyLinear(DPMI_PTR2L(&buf), seg<<4UL, sizeof(buf));
        DPMI_DOSFree(dosmem);
        dosmem = DPMI_DOSMalloc((uint16_t)((sizeof(IRQRoutingOptionBuffer)+buf.size+15)>>4));
        if(dosmem == 0)
        {
            DPMI_DOSFree(dosstack);
            return 0xFF;
        }

        seg = (dosmem&0xFFFFU);
        buf.off = sizeof(buf);
        buf.seg = (uint16_t)seg;
        DPMI_CopyLinear(seg<<4UL, DPMI_PTR2L(&buf), sizeof(buf));

        r.h.ah = PCI_FUNCTION_ID;
        r.h.al = GET_IRQ_ROUTING_OPTIONS;
        r.w.bx = 0;
        r.w.ds = 0xF000;
        r.w.es = (uint16_t)seg;
        r.w.di = 0;
        r.w.ss = (uint16_t)(dosstack&0xFFFFU);
        r.w.sp = STACKSIZE;
        ret = DPMI_CallRealModeINT(PCI_INT_NO, &r);
    }
    else
        r.h.ah = 0xFF;
    if(ret != 0 || r.h.ah != SUCCESSFUL)
    {
        DPMI_DOSFree(dosmem);
        DPMI_DOSFree(dosstack);
        _LOG("PCI: Get IRQ Routing Options failed %d %x.\n", ret, r.h.ah);
        return 0xFF;
    }

    IRQRoutingTable* table = (IRQRoutingTable*)malloc(buf.size);
    DPMI_CopyLinear(DPMI_PTR2L(table), (seg<<4UL)+sizeof(buf), buf.size);
    DPMI_DOSFree(dosmem);
    uint16_t count = buf.size/sizeof(IRQRoutingTable);

    uint16_t map = 0;
    uint8_t link = 0;
    for(uint16_t i = 0; i < count; ++i)
    {
        if(table[i].bus == bus && (table[i].dev>>3) == dev)
        {
            link = table[i].intpins[INTPIN-1].link;
            map = table[i].intpins[INTPIN-1].map;
            _LOG("LINK: %d, MAP:%x\n", link, map);
            break;
        }
    }

    uint8_t linkedIRQ = 0xFF;
    uint16_t originalmap = map;
    if(map && link)
    {
        //iterate all devices to find devices with the same link (wire-ORed)
        for(uint16_t i = 0; (i < count) && (map != 0); ++i)
        {
            for(int j = 0; (j < 4) && (linkedIRQ == 0xFF) && (map != 0); ++j)
            {
                if(table[i].intpins[j].link != link)
                    continue;
                for(uint8_t f = 0; (f < 8) && (linkedIRQ == 0xFF); ++f)
                {
                    uint8_t intpin = PCI_ReadByte(table[i].bus, table[i].dev>>3, f, PCI_REGISTER_INTPIN);
                    if(intpin-1 == j)
                    {
                        linkedIRQ = PCI_ReadByte(table[i].bus, table[i].dev>>3, f, PCI_REGISTER_IRQ);
                        _LOG("link: %d, intpin INT%c#, irq %d\n", table[i].intpins[j].link, 'A'+j, linkedIRQ);
                        if(linkedIRQ != 0xFF) 
                        {
                            _LOG("MAP: %x\n",table[i].intpins[j].map);
                            map &= table[i].intpins[j].map;
                            if(((1<<linkedIRQ)&map) == 0) //not in target map
                                linkedIRQ = 0xFF;
                            if(map == 0) //IRQ routing options conflict: same link value but options have no intersection
                                break;
                        }
                    }
                }
            }
            if(linkedIRQ != 0xFF)
                break;
        }
    }
    const uint16_t MouseIRQ = (1<<12);
    const uint16_t ATAIRQ = (1<<14)|(1<<15);

    uint8_t irq = linkedIRQ;
    if(irq != 0xFF)
        PCI_WriteByte(bus, dev, func, PCI_REGISTER_IRQ, irq); //shared irq found
    else if(map != 0) //no IRQ conflict we can set
    {
        if(map&r.w.bx)
            map &= r.w.bx; //prefer PCI dedicated IRQ
        if(map&~ATAIRQ)
            map &= (uint16_t)(~ATAIRQ);
        if(map&~MouseIRQ)
            map &= (uint16_t)(~MouseIRQ);

        //find the highset available
        while(map)
        {
            map>>=1;
            ++irq;
        }
        _LOG("PIC_AssignIRQ: %x %d\n", map, irq);

        assert(irq > 2 && irq <= 15);
        PIC_MaskIRQ(irq);
        if(PCI_SetIRQ_Stack(bus, dev, func, INTPIN, irq, dosstack, STACKSIZE, FALSE))
        {
            //assign to all wire-ORed pin, including input (ppkey)
            for(int i = 0; i < count; ++i)
            {
                for(int j = 0; j < 4; ++j)
                {
                    if(table[i].intpins[j].link != link || (table[i].intpins[j].map&(1<<irq)) == 0)
                        continue;
                    for(uint8_t f = 0; f < 8; ++f)
                    {
                        uint8_t intpin = PCI_ReadByte(table[i].bus, table[i].dev>>3, f, PCI_REGISTER_INTPIN);
                        if(intpin-1 == j)
                        {
                            assert(PCI_ReadByte(table[i].bus, table[i].dev>>3, f, PCI_REGISTER_IRQ) == 0xFF);
                            PCI_WriteByte(table[i].bus, table[i].dev>>3, f, PCI_REGISTER_IRQ, irq);
                        }
                    }
                }
            }
        }
        else
            irq = 0xFF;
    }
    else if(originalmap != 0) //there's IRQ conflict, we just set the PCI config space reg.
    {
        map = originalmap;
        if(map&r.w.bx)
            map &= r.w.bx; //prefer PCI dedicated IRQ
        if(map&~ATAIRQ)
            map &= (uint16_t)(~ATAIRQ);
        if(map&~MouseIRQ)
            map &= (uint16_t)(~MouseIRQ);
        //find the highset available
        while(map)
        {
            map>>=1;
            ++irq;
        }
        assert(irq > 2 && irq <= 15);
        PCI_WriteByte(bus, dev, func, PCI_REGISTER_IRQ, irq);
    }

    DPMI_DOSFree(dosstack);
    free(table);
    return irq;
}

BOOL PCI_SetIRQ(uint8_t bus, uint8_t dev, uint8_t func, uint8_t INTPIN, uint8_t IRQ)
{
    const uint16_t STACKSIZE = 1024;
    uint32_t dosstack = DPMI_DOSMalloc(STACKSIZE>>4);
    if(dosstack == 0)
        return FALSE;
    return PCI_SetIRQ_Stack(bus, dev, func, INTPIN, IRQ, dosstack, STACKSIZE, TRUE);
}
