#include "USBDDOS/PCI.H"

#define PCI_ADDR  0x0cf8
#define PCI_DATA  0x0cfc
#define ENABLE_BIT 0x80000000

#if defined(__BC__)
uint32_t inpd(uint16_t port)
{
    uint32_t val;
    asm {
        mov dx, port
        in eax, dx
        mov val, eax
    }
    return val;
}

void outpd(uint16_t port, uint32_t val)
{
    asm {
        mov   dx, port
        mov   eax, val
        out   dx, eax
    }
}
#endif


uint8_t PCI_Read_Byte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    const int shift = ((reg & 3) * 8);
    const uint32_t val = ENABLE_BIT |
        ((uint32_t)bus << 16) |
        ((uint32_t)dev << 11) |
        ((uint32_t)func << 8) |
        ((uint32_t)reg & 0xFC);
    outpd(PCI_ADDR, val);
    return (inpd(PCI_DATA) >> shift) & 0xFF;
}


uint16_t PCI_Read_Word(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
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
        return (PCI_Read_Byte(bus, dev, func, reg + 1) << 8) | PCI_Read_Byte(bus, dev, func, reg);
}

// 'reg' is the uint8_t reg from 0x00
uint32_t PCI_Read_DWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    if ((reg & 3) == 0)
    {
        const uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        return inpd(PCI_DATA);
    }
    else
        return (PCI_Read_Word(bus, dev, func, reg + 2) << 16) | PCI_Read_Word(bus, dev, func, reg);
}

// 'reg' is the uint8_t reg from 0x00
void PCI_Write_Byte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t value)
 {
    const int shift = ((reg & 3) * 8);
    const uint32_t val = ENABLE_BIT |
        ((uint32_t)bus << 16) |
        ((uint32_t)dev << 11) |
        ((uint32_t)func << 8) |
        ((uint32_t)reg & 0xFC);
      outpd(PCI_ADDR, val);
      outpd(PCI_DATA, (inpd(PCI_DATA) & ~(0xFF << shift)) | (value << shift));
}


void PCI_Write_Word(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t value)
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
        outpd(PCI_DATA, (inpd(PCI_DATA) & ~(0xFFFF << shift)) | (value << shift));
    }
    else
    {
        PCI_Write_Byte(bus, dev, func, reg + 0, (uint8_t) (value & 0xFF));
        PCI_Write_Byte(bus, dev, func, reg + 1, (uint8_t) (value >> 8));
    }
}


void PCI_Write_DWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t value)
{
    if ((reg & 3) == 0)
    {
        const uint32_t val = ENABLE_BIT |
            ((uint32_t)bus << 16) |
            ((uint32_t)dev << 11) |
            ((uint32_t)func << 8) |
            ((uint32_t)reg & 0xFC);
        outpd(PCI_ADDR, val);
        outpd(PCI_DATA, value);
    }
    else
    {
        PCI_Write_Word(bus, dev, func, reg + 0, (uint16_t) (value & 0xFFFF));
        PCI_Write_Word(bus, dev, func, reg + 2, (uint16_t) (value >> 16));
    }
}

uint32_t PCI_Sizing(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
    //PCI Local Bus Specification Revision 3.0, 6.2.5.1
    //OHCI_Specification_Rev.1.0a
    uint16_t CmdReg;
    uint32_t OldVal;
    uint32_t range;

    CmdReg = PCI_Read_Word(bus, dev, func, 0x04);
    PCI_Write_Word(bus, dev, func, 0x04, CmdReg & ~0x03);

    OldVal = PCI_Read_DWord(bus, dev, func, reg);

    // write 0xFFFFFFFF and read back
    PCI_Write_DWord(bus, dev, func, reg, 0xFFFFFFFF);
    range = PCI_Read_DWord(bus, dev, func, reg);

    // restore
    PCI_Write_DWord(bus, dev, func, reg, OldVal);
    PCI_Write_Word(bus, dev, func, 0x04, CmdReg);

    if (range & 0x1L)//IO bit
    {
        range &=~0x1L; //clear encoding bits (0)
        range |= 0xFFFF0000L;
    }
    else //Mem bit
        range &=~0xFL; //clear encoding bits (0-3)
    return (~range) + 1L;
}
