#ifndef _HDPMIPT_H_
#define _HDPMIPT_H_
//HDPMI port trap utility

#include "emm.h" //EMM386 compatible interface

BOOL HDPMIPT_Install_IOPortTrap(uint16_t start, uint16_t end, EMM_IODT* inputp iodt, uint16_t count, EMM_IOPT* outputp iopt);

BOOL HDPMIPT_Uninstall_IOPortTrap(EMM_IOPT* inputp iopt);

#endif //_HDPMIPT_H_