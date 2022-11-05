#ifndef _PCI_H
#define _PCI_H_
#include "USBDDOS/PLATFORM.H"

#if defined(__DJ2__)
#include <pc.h>
#elif defined(__WC__)
#include <conio.h>
#endif

#if __cplusplus
extern "C"
{
#endif

//32bit extension
#if defined(__DJ2__)
#define inpd inportl
#define outpd outportl
#elif defined(__BC__)
uint32_t inpd(uint16_t port);
void outpd(uint16_t port, uint32_t value);
#endif

//BDF
typedef struct _PCIAddress
{
    uint8_t Bus;
    uint8_t Device;
    uint8_t Function;
    uint8_t unused;
}PCI_Addr;

//PCI configuration space access
//https://en.wikipedia.org/wiki/PCI_configuration_space#Software_implementation
//https://wiki.osdev.org/PCI

uint8_t PCI_Read_Byte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);
uint16_t PCI_Read_Word(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);
uint32_t PCI_Read_DWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);

void PCI_Write_Byte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t value);
void PCI_Write_Word(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t value);
void PCI_Write_DWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t value);

uint32_t PCI_Sizing(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);

#if __cplusplus
}
#endif

#endif
