#include <stdio.h>
#include <stdlib.h>
#include <DOS.h>
#include "USB.H"
#include "DPMI/DPMI.h"

#define RETROWAVE_VENDOR 0x04d8
#define RETROWAVE_DEVID 0x000a

int main()
{
    DPMI_Init();
    USB_Init();

    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
        {
            if(!HCD_IS_DEVICE_VALID(pHCI->DeviceList[i]))
                continue; //PnP: unplugged gaps
            USB_Device* dev = HC2USB(pHCI->DeviceList[i]);
            if(dev->bStatus)

            if(dev->Desc.widVendor == RETROWAVE_VENDOR)
            {
                printf("Vendor: %s, Name: %s\n", dev->sManufacture, dev->sProduct);
                //printf("Base Address: %08lx\n", dev->HCDDevInfo.pHCI->dBaseAddress);
                break;
            }
            //USB_ShowDeviceInfo(device);
        }
    }
    
    return 0;
}
