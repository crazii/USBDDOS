#include <dos.h>
#include <conio.h>
#include "USBDDOS/PIC.H"
#include "USBDDOS/DBGUTIL.H"
//reference: https://wiki.osdev.org/PIC

//master PIC
#define PIC_PORT1 0x20
#define PIC_DATA1 0x21
//slave PIC
#define PIC_PORT2 0xA0
#define PIC_DATA2 0xA1

#define PIC_READISR 0x0B    //read interrupte service register (current interrupting IRQ)
#define PIC_READIRR 0x0A

#define PIC_SLAVE_IRQ 2

void PIC_SendEOI(void)
{
    //CLIS();
    //get irq mask
    outp(PIC_PORT1, PIC_READISR);
    uint16_t mask = inp(PIC_PORT1);
    if(mask&(1<<PIC_SLAVE_IRQ))
        outp(PIC_PORT2, 0x20);
    //_LOG("EOI: %04x ", mask);
    outp(PIC_PORT1, 0x20);
    //STIL();
}

uint8_t PIC_GetIRQ(void)
{
    //CLIS();
    //get irq mask
    outp(PIC_PORT1, PIC_READISR);
    uint16_t mask = inp(PIC_PORT1);
    if(mask&(1<<PIC_SLAVE_IRQ))
    {
        outp(PIC_PORT2, PIC_READISR);
        mask = (uint16_t)(inp(PIC_PORT2)<<8);
    }
    //STIL();
    if(mask == 0)
        return 0xFF;
    return (uint8_t)BSF(mask);
}

uint16_t PIC_GetPendingInterrupts(void)
{
    outp(PIC_PORT1, PIC_READIRR);
    outp(PIC_PORT2, PIC_READIRR);
    uint16_t mask = (uint16_t)(inp(PIC_PORT1) | (inp(PIC_PORT2)<<8));
    return mask;
}

void PIC_RemapMaster(uint8_t vector)
{
    CLIS();
    uint8_t oldmask = inp(PIC_DATA1);
    outp(PIC_PORT1, 0x11);
    outp(PIC_DATA1, vector);
    outp(PIC_DATA1, 4);
    outp(PIC_DATA1, 1);
    outp(PIC_DATA1, oldmask);
    STIL();
}

void PIC_RemapSlave(uint8_t vector)
{
    CLIS();
    uint8_t oldmask = inp(PIC_DATA2);
    outp(PIC_PORT2, 0x11);
    outp(PIC_DATA2, vector);
    outp(PIC_DATA2, 2);
    outp(PIC_DATA2, 1);
    outp(PIC_DATA2, oldmask);
    STIL();
}

void PIC_MaskIRQ(uint8_t irq)
{
    uint16_t port = PIC_DATA1;
    if(irq >= 8)
    {
        port = PIC_DATA2;
        irq = (uint8_t)(irq - 8);
    }
    CLIS();
    outp(port, (uint8_t)(inp(port)|(1<<irq)));
    STIL();
}

void PIC_UnmaskIRQ(uint8_t irq)
{
    uint16_t port = PIC_DATA1;
    if(irq >= 8)
    {
        outp(port, (uint8_t)(inp(port)&~(1<<PIC_SLAVE_IRQ))); //make sure slave enabled
        port = PIC_DATA2;
        irq = (uint8_t)(irq - 8);
    }
    CLIS();
    outp(port, (uint8_t)(inp(port)&~(1<<irq)));
    STIL();
}

uint16_t PIC_GetIRQMask(void)
{
    //CLIS();
    uint16_t mask = (uint16_t)((inp(PIC_DATA2)<<8) | inp(PIC_DATA1));
    //STIL();
    return mask;
}

void PIC_SetIRQMask(uint16_t mask)
{
    //CLIS();
    outp(PIC_DATA1, (uint8_t)(mask & ~(1<<PIC_SLAVE_IRQ))); //make sure savle enabled
    outp(PIC_DATA2, (uint8_t)(mask>>8));
    //STIL();
}

BOOL PIC_SetLevelTriggered(uint8_t irq, BOOL LevelTriggered)
{
    uint16_t ELCR = inpw(0x4D0);
    if(LevelTriggered)
    {
        if(irq == 0 || irq == 1 || irq == 2 || irq == 13)
            return FALSE;
        ELCR |= 1<<irq;
    }
    else
        ELCR &= 1<<irq;
    outpw(0x4D0, ELCR);
    return TRUE;
}
