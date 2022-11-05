#include <string.h>
#include <assert.h>
#include "USBDDOS/PCI.H"

#define PCI_ADDR  0x0CF8
#define PCI_DATA  0x0CFC
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
