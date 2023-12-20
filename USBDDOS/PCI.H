#ifndef _PCI_H
#define _PCI_H_
#include "USBDDOS/PLATFORM.H"

#if defined(__DJ2__)
#include <pc.h>
#elif defined(__WC__) || defined(__BC__)
#include <conio.h>
#endif

#ifdef __cplusplus
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

//PCI configuration space access
//https://en.wikipedia.org/wiki/PCI_configuration_space#Software_implementation
//https://wiki.osdev.org/PCI

#define  PCI_MAX_BUS        16  //256 by specs, but..
#define  PCI_MAX_DEV        32
#define  PCI_MAX_FUNC       8

#define PCI_HEADER_DEVICE 0x0
#define PCI_HEADER_PCI_BRIDGE 0x1
#define PCI_HEADER_CARDBUS_BRIDGE 0x2
#define PCI_HEADER_MULTI_FUNCTION 0x80 //multi-function bit (mask)

//PCI BIOS functions (PCI BIOS specification 2.1)
#define PCI_FUNCTION_ID         0xB1
#define PCI_BIOS_PRESENT        0x01
#define FIND_PCI_DEVICE         0x02
#define READ_CONFIG_BYTE        0x08
#define READ_CONFIG_WORD        0x09
#define READ_CONFIG_DWORD       0x0A
#define WRITE_CONFIG_BYTE       0x0B
#define WRITE_CONFIG_WORD       0x0C
#define WRITE_CONFIG_DWORD      0x0D
#define GET_IRQ_ROUTING_OPTIONS 0x0E
#define SET_PCI_IRQ             0x0F //SET_PCI_HW_INT
#define PCI_INT_NO              0x1A

#define SUCCESSFUL              0x00
#define FUNC_NOT_SUPPORTED      0x81
#define BAD_VENDOR_ID           0x83
#define DEVICE_NOT_FOUND        0x86
#define BAD_REGISTER_NUMBER     0x87
#define SET_FAILED              0x88
#define BUFFER_TOO_SMALL        0x98

//BDF
typedef struct _PCIAddress
{
    uint8_t Bus;
    uint8_t Device;
    uint8_t Function;
    uint8_t unused;
}PCI_Addr;

typedef struct
{
    uint32_t Base0;             // BAR0 (OHCI, EHCI)
    uint32_t Base1;
    uint32_t Base2;
    uint32_t Base3;
    uint32_t Base4;             // BAR4 (UHCI)
    uint32_t Base5;
    uint32_t CardbusCIS;        //CardBus CIS Pointer
    uint16_t SubsystemVendorID;
    uint16_t SubsytemID;
    uint32_t ROMBase;           //Expansion ROM base address
    uint8_t Capabilities;
    uint8_t Reserved[3];
    uint32_t Reserved2;
    uint8_t IRQ;                //Interrupt Line (legacy ISA IRQ)
    uint8_t INTPIN;             //PCI Interrupt PIN (INTA#...INTD#)
    uint8_t MinGrant;           //(RO) burst period length in 1/4 ms units
    uint8_t MaxLatency;         //(RO) how often the device needs access to the PCI bus, 1/4ms units
}PCI_DEVICE_HEADER;

typedef struct
{
    uint32_t Base0;
    uint32_t Base1;
    uint8_t PrimaryBusNO;
    uint8_t SeconaryBusNO;
    uint8_t SubordinateBusNO;
    uint8_t LatencyTimer2;
    uint8_t IOBase;
    uint8_t IOLimt;
    uint16_t Status2;
    uint16_t MemoryBase;
    uint16_t MemoryLimit;
    uint16_t PMemoryBase;       //prefetchable memory base
    uint16_t PMemoryLimit;      //prefetchable memory limit
    uint32_t PMemoryBase32;     //upper 32 bit
    uint32_t PMemoryLimit32;    //upper 32 bit
    uint16_t IOBase16;          //upper 16 bit
    uint16_t IOLimt16;          //upper 16 bit
    uint8_t Capabilities;
    uint8_t Reserved[3];
    uint32_t Reserved2;
    uint8_t IRQ;
    uint8_t INTPIN;
    uint16_t BridgeControl;
}PCI_PCI_BRIDGE_HEADER;

typedef struct
{
    uint32_t Base;
    uint8_t CapabilityOffset;
    uint8_t Reserved;
    uint16_t Status2;
    uint8_t PCIBusNO;
    uint8_t CardBusNO;
    uint8_t SubordinateBusNO;
    uint8_t CardBusLatencyTimer;
    uint32_t Base0;
    uint32_t Limit0;
    uint32_t Base1;
    uint32_t Limit1;
    uint32_t IOBase0;
    uint32_t IOLimit0;
    uint32_t IOBase1;
    uint32_t IOLimit1;
    uint8_t IRQ;
    uint8_t INTPIN;
    uint16_t BridgeControl;
    uint16_t SubsystemVendorID;
    uint16_t SubsytemID;
    uint32_t PCCardBase;    //legacy 16 bit
}PCI_CARDBUS_BRIDGE_HEADER;

typedef struct
{
    //common header register 0~3
    uint16_t VendorID;          // Vendor ID (FFFFh invalid)
    uint16_t DeviceID;          // Device ID
    uint16_t Command;           // Command (0x04)
    uint16_t Status;
    uint8_t RevisionID;
    uint8_t PInterface;         //register-level programming interface
    uint8_t SubClass;
    uint8_t Class;
    uint8_t CacheLineSize;
    uint8_t LatencyTimer;
    uint8_t HeaderType;         //HeaderType: 0: general device, 1: PCI-to-PCI bridge, 2: PCI-to-CardBus bridge
    uint8_t BIST;               //builtin self tet
    //
    union
    {
        PCI_DEVICE_HEADER Device;
        PCI_PCI_BRIDGE_HEADER PCIBridge;
        PCI_CARDBUS_BRIDGE_HEADER CardBusBridge;
    }DevHeader;

}PCI_HEADER;

typedef union
{
    PCI_HEADER Header;
    uint32_t Registers[64];
    uint8_t Offset[256];
}PCI_DEVICE;

//command register (word)
#define PCI_REGISTER_CMD 0x04
//status register (word)
#define PCI_REGISTER_STS 0x06
typedef union
{
    uint16_t reg16;
    struct
    {
        uint16_t IOSpace : 1;       //enable io space
        uint16_t MemorySpace : 1;   //enable memory space (MMIO)
        uint16_t BusMaster : 1;     //enable PCI access
        uint16_t SpecialCircles : 1;
        uint16_t MemoryInvalidate : 1;
        uint16_t VGAPaletteSnoop : 1;
        uint16_t ParityError : 1;   //Parity Error Response
        uint16_t Reserved : 1;
        uint16_t SERR : 1;
        uint16_t FastB2B : 1;       //fast bck to back
        uint16_t InterruptDisable : 1; //disable interrupt
        uint16_t Reserved2 : 5;
    }bits;
}PCI_CMD;

#define PCI_REGISTER_IRQ 0x3C //word: IRQ (low byte) & INTPIN

uint8_t PCI_ReadByte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);
uint16_t PCI_ReadWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);
uint32_t PCI_ReadDWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);

void PCI_WriteByte(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t value);
void PCI_WriteWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t value);
void PCI_WriteDWord(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t value);

void PCI_ReadDevice(uint8_t bus, uint8_t dev, uint8_t func, PCI_DEVICE* device);

uint32_t PCI_Sizing(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg);

uint16_t PCI_GetIRQMap(uint8_t bus, uint8_t dev, uint8_t INTPIN);

BOOL PCI_SetIRQ(uint8_t bus, uint8_t dev, uint8_t func, uint8_t INTPIN, uint8_t IRQ); //SET_PCI_IRQ, INTPIN=0x1...4

#ifdef __cplusplus
}
#endif

#endif
