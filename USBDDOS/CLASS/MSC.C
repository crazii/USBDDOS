#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include "USBDDOS/CLASS/MSC.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

BOOL USB_MSC_InitDevice(USB_Device* pDevice)
{
    assert(pDevice->bStatus == DS_Configured);

    uint8_t bNumInterfaces = pDevice->pConfigList[pDevice->bCurrentConfig].bNumInterfaces;
    USB_InterfaceDesc* pIntfaceDesc = pDevice->pConfigList[pDevice->bCurrentConfig].pInterfaces;

    for(int j = 0; j < bNumInterfaces; ++j)
    {
        USB_InterfaceDesc* pIntfaceDescI = pIntfaceDesc + j;
        _LOG("MSC bInterfaceProtocol: %x\n", pIntfaceDescI->bInterfaceProtocol);
        if(pIntfaceDescI->bInterfaceClass == USBC_MASSSTORAGE //must be MSC
            && pIntfaceDescI->bInterfaceProtocol == USB_MSC_PROTOCOL_BBB)//currently only support BOT
        {
            pIntfaceDesc = pIntfaceDescI;
            break;
        }
    }
    assert(pIntfaceDesc->bInterfaceClass == USBC_MASSSTORAGE); 
    if(pIntfaceDesc->bInterfaceProtocol != USB_MSC_PROTOCOL_BBB)
        return FALSE;
    uint8_t bInterface = pIntfaceDesc->bInterfaceNumber;

    //get max LUN
    USB_Request Req = { USB_REQ_READ | USB_REQ_TYPE_MSC, USB_REQ_MSC_GET_MAX_LUN, 0, bInterface, 1, };
    uint8_t* pMaxLUN = (uint8_t*)DPMI_DMAMalloc(1, 16);
    *pMaxLUN = 0;
    USB_SyncSendRequest(pDevice, &Req, pMaxLUN); //ignore return. device may not support multiple LUN.
    uint8_t maxLUN = *pMaxLUN;
    DPMI_DMAFree(pMaxLUN);
    pMaxLUN = NULL;
    _LOG("MSC MaxLUN: %d\n", maxLUN);
    
    USB_Request Req2 =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, bInterface, 0};
    if(USB_SyncSendRequest(pDevice, &Req2, NULL) != 0)
    {
        _LOG("MSC Error reset bulk.\n");
        return FALSE;
    }
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)malloc(sizeof(USB_MSC_DriverData));
    memset(pDriverData, 0, sizeof(USB_MSC_DriverData));
    pDevice->pDriverData = pDriverData;
    pDriverData->bInterface = bInterface;
    pDriverData->MaxLUN = maxLUN;
    for(int i = 0; i < pIntfaceDesc->bNumEndpoints; ++i)
    {
        USB_EndpointDesc* pEndpointDesc = pIntfaceDesc->pEndpoints + i;
        assert(pEndpointDesc->bmAttributesBits.TransferType == USB_ENDPOINT_TRANSFER_TYPE_BULK); //BULK only
        pDriverData->pDataEP[pEndpointDesc->bEndpointAddressBits.Dir] = USB_FindEndpoint(pDevice, pEndpointDesc);
        pDriverData->bEPAddr[pEndpointDesc->bEndpointAddressBits.Dir] = pEndpointDesc->bEndpointAddress;
    }
    assert(pDriverData->pDataEP[0] != NULL && pDriverData->pDataEP[1] != NULL);

    USB_ClearHalt(pDevice, pDriverData->bEPAddr[0]);
    USB_ClearHalt(pDevice, pDriverData->bEPAddr[1]);

    //perform other readings as sanity check
    {
        USB_MSC_INQUIRY_CMD cmd;
        USB_MSC_INQUIRY_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_INQUIRY;
        cmd.LUN = 0;
        cmd.AllocationLength = sizeof(data);
        if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), &data, sizeof(data), HCD_TXR))
        {
            _LOG("MSC Failed INQUIRY.\n");
            return FALSE;
        }
        _LOG("PDT: %x\n", data.PDT);
    }
    {
        USB_MSC_READCAP_CMD cmd;
        USB_MSC_READCAP_DATA data;
        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = USB_MSC_SBC_READ_CAP;
        for(int i = 0; i <= pDriverData->MaxLUN; ++i)
        {
            cmd.LUN = ((uint8_t)i)&0x7U;
            if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), &data, sizeof(data), HCD_TXR))
            {
                _LOG("MSC Failed get capacity.\n");
                return FALSE;
            }
            uint32_t maxLBA = EndianSwap32(data.LastLBA);
            uint32_t BlockSize = EndianSwap32(data.BlockSize);
            float cap = (float)maxLBA / 1024.0f / 1024.0f / 1024.0f * (float)BlockSize;
            _LOG("LUN %d Capcacity: %.1f GB\n", i, cap);
            pDriverData->SizeGB = (uint16_t)(pDriverData->SizeGB + (uint16_t)cap);
        }
    }
    return TRUE;
}

BOOL USB_MSC_DeinitDevice(USB_Device* pDevice)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL)
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;
    if(pDriverData)
    {
        free(pDriverData);
        pDevice->pDriverData = NULL;
    }
    return TRUE;
}

BOOL USB_MSC_BulkReset(USB_Device* pDevice)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL)
        return FALSE;
    USB_Request Req =  {USB_REQ_TYPE_MSC, USB_REQ_MSC_RESET, 0, ((USB_MSC_DriverData*)pDevice->pDriverData)->bInterface, 0};
    return USB_SyncSendRequest(pDevice, &Req, NULL) != 0;
}

BOOL USB_MSC_IssueCommand(USB_Device* pDevice, void* inputp cmd, uint32_t CmdSize, void* inoutp nullable data, uint32_t DataSize, HCD_TxDir dir)
{
    if(pDevice == NULL || pDevice->pDriverData == NULL || cmd == NULL || CmdSize > 16 || CmdSize < 1
        || (DataSize != 0 && data == NULL))
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;

    //CBW
    USB_MSC_CBW cbw;
    memset(&cbw, 0, sizeof(cbw));
    cbw.dCBWSignature = USB_MSC_CBW_SIGNATURE;
    cbw.dCBWTag = (uintptr_t)&cbw; //use addr as tag
    cbw.dCBWDataTransferLength = DataSize;
    cbw.bmCBWFlags = dir == HCD_TXR ? 0x80U : 0;
    cbw.bCBWLUN = 0;
    cbw.bCBWCBLength = ((uint8_t)CmdSize)&0x1FU;
    memcpy(cbw.CBWCB, cmd, CmdSize);
    uint8_t* dma = (uint8_t*)DPMI_DMAMalloc(sizeof(cbw), 16);
    memcpy(dma, &cbw, sizeof(cbw));
    uint16_t len = 0;
    uint16_t size = sizeof(cbw);
    uint8_t error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[0], dma, size, &len); //TODO: error code now is HC specific, need abstraction (wrapper).
    DPMI_DMAFree(dma);
    if(len != size || error)
    {
        _LOG("MSC CBW failed: %x, %d, %d.\n", error, size, len);
        USB_ClearHalt(pDevice, pDriverData->bEPAddr[0]);
        return FALSE;
    }
    
    //DATA
    if(DataSize)
    {
        dma = (uint8_t*)DPMI_DMAMalloc(DataSize, 16);
        memset(dma, 0, DataSize);
        if(dir == HCD_TXW)
            memcpy(dma, data, DataSize);
        len = 0;
        error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[dir&0x1], dma, (uint16_t)DataSize, &len);
        if(dir == HCD_TXR)
            memcpy(data, dma, DataSize);
        DPMI_DMAFree(dma);
        if(len != DataSize || error)
        {
            _LOG("MSC DATA Failed: %x, %d, %d, %d.\n", error, dir, DataSize, len);
            USB_ClearHalt(pDevice, pDriverData->bEPAddr[dir&0x1]);
            return FALSE;
        }
    }

    //CSW
    len = 0;
    dma = (uint8_t*)DPMI_DMAMalloc(sizeof(USB_MSC_CSW), 16);
    error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[1], dma, sizeof(USB_MSC_CSW), &len);
    USB_MSC_CSW csw = *(USB_MSC_CSW*)dma;
    DPMI_DMAFree(dma);
    if(len != sizeof(USB_MSC_CSW) || csw.dCSWSignature != USB_MSC_CSW_SIGNATURE || csw.dCSWTag != cbw.dCBWTag
    || csw.dCSWDataResidue != 0 || error)
    {
        USB_ClearHalt(pDevice, pDriverData->bEPAddr[1]);
        _LOG("MSC CSW: %x, %x, %x, %x\n", csw.dCSWSignature, csw.dCSWTag, csw.dCSWDataResidue, csw.bCSWStatus);
        _LOG("MSC CSW Failed: %x, %d, %d\n", error, sizeof(USB_MSC_CSW), len);
        _LOG("MSC CBW length: %d\n", cbw.dCBWDataTransferLength);
        return FALSE;
    }
    
    return TRUE;
}



#if defined(__DJ2__)
#pragma pack(1)
#endif

//https://stanislavs.org/helppc/bios_parameter_block.html
//https://en.wikipedia.org/wiki/BIOS_parameter_block
typedef struct DOS_BIOSParameterBlock //DOS 3.0+ 0 BPB 
{
    //2.0
    uint16_t BPS;   //bytes per sector
    uint8_t SPC;    //sectors per cluster
    uint16_t RsvdSectors; //reserved sectors from the begginning
    uint8_t FATs;
    uint16_t RootDirs;
    uint16_t Sectors; //sectors in total
    uint8_t MediaDesc; //media descriptor
    uint16_t SPF; //sectors per fat
    //3.31
    uint16_t SPT;   //sectors per track
    uint16_t Heads; //heads count
    uint32_t HiddenSectors;
    uint32_t Sectors2; //used if Sectors=0
}DOS_BPB;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_BPB) == 51, "incorrect size");
#endif

#define DOS_DDHA_32BIT_ADDRESSING   0x0002
#define DOS_DDHA_OPENCLOSE          0x0800

typedef struct DOS_DeviceDriverHeader
{
    uint32_t NextDDH;
    uint16_t Attribs; //http://www.delorie.com/djgpp/doc/rbinter/it/48/16.html
    uint16_t StrategyEntryPoint;
    uint16_t IntEntryPoint;
    uint8_t Drives;
    uint8_t Signature[7];
}DOS_DDH;

typedef struct DOS_DriveParameterBlock //DOS4.0+
{
    uint8_t Drive; //0:A, 2:B, 3:C...
    uint8_t Units;
    uint16_t BPS; //bytes per sector
    uint8_t SPC; //sectors per cluster
    uint8_t C2SShift; //shift count for cluster to sector
    uint16_t RsvdSectors; //reserved sectors from the begginning
    uint8_t FATs;
    uint16_t RootDirs; //root dir count
    uint16_t User1stSector; //no. of first sector for user
    uint16_t MaxCluster;
    uint16_t SPF;    //sectors per fat
    uint16_t Dir1stSector; //no. of first sector for dir
    uint32_t DriverHeader; //addr of driver header
    uint8_t MediaID;
    uint8_t Accessed; //00 for true, FF for none
    uint32_t NextDPB;
    uint16_t Free1stCluster;
    uint16_t FreeCulsters;
}DOS_DPB;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_DPB) == 32, "incorrect size");
#endif

#define DOS_CDS_FLAGS_USED 0xC0

typedef struct DOS_CurrentDirectoryStructure
{
    char path[67];
    uint16_t flags;
    uint32_t DPBptr;
    uint8_t  unknown[15];
}DOS_CDS;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_CDS) == 88, "incorrect size");
#endif

#if defined(__DJ2__)
#pragma pack()
#endif

//"List of Lists" offsets
#define DOS_LOL_CDS_PTR             0x16    //4 bytes (far ptr)
#define DOS_LOL_BLOCK_DEVICE_COUNT  0x20    //installed device count, 1byte
#define DOS_LOL_DRIVE_COUNT         0x21    //drive count (and CDS count), 1 byte
#define DOS_LOL_NULDEV_HEADER       0x22    //NUL device header (18 bytes)
#define DOS_LOL_SIZE                0x60

//ease of use memory layout
typedef struct 
{
    DOS_DDH ddh;
    DOS_BPB bpb;
    DOS_DPB dpb;
    uint8_t STG_opcodes[1];     //retf
    uint8_t INT_opcodes[6+1];   //call far ptr + retf
    //PM entry points, called by INT_opcodes
    uint16_t INT_RMCB_Off;  //TODO: allocate RMCB as entry point
    uint16_t INT_RMCB_CS;
}USB_MSC_DOS_TSRDATA;

static void USB_MSC_DOS_DriverINT()    //PM IntEntryPoint for DDH
{

}

BOOL USB_MSC_DOS_Install()     //ref: https://gitlab.com/FreeDOS/drivers/rdisk/-/blob/master/SOURCE/RDISK/RDISK.ASM
{
    uint32_t DrvMem = DPMI_HighMalloc(sizeof(USB_MSC_DOS_TSRDATA), TRUE); //allocate resident memory
    //TODO: read valid device and put DrvMem to DOSDriverMem of USB_MSC_DriverData (for uninstall)
    USB_MSC_DOS_TSRDATA TSRData;
    {
        DOS_DDH ddh = {0xFFFFFFFF, DOS_DDHA_32BIT_ADDRESSING|DOS_DDHA_OPENCLOSE, offsetof(USB_MSC_DOS_TSRDATA, STG_opcodes), offsetof(USB_MSC_DOS_TSRDATA, INT_opcodes), 1, };
        memcpy(ddh.Signature, "USBDDOS", 7);
        TSRData.ddh = ddh;
    }
    {
        DOS_BPB bpb;
        memset(&bpb, 0, sizeof(bpb));
        bpb.BPS;
        bpb.SPC;
        bpb.RsvdSectors;
        bpb.FATs;
        bpb.RootDirs;
        bpb.Sectors;
        bpb.MediaDesc;
        bpb.SPF;
        bpb.SPT;
        bpb.Heads;
        bpb.HiddenSectors;
        bpb.Sectors2;
        TSRData.bpb = bpb;
    }
    DPMI_REG reg;

    //DOS 2.0+ internal get list of lists http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2983.htm
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg); //return ES:BX pointing to buffer

    uint8_t* buf = (uint8_t*)malloc(DOS_LOL_SIZE);
    DPMI_CopyLinear(DPMI_PTR2L(buf), (uint32_t)(reg.w.es << 4) + reg.w.bx, DOS_LOL_SIZE); //copy to dpmi mem for easy access
    uint8_t DriveCount = buf[DOS_LOL_DRIVE_COUNT];
    uint8_t DeviceCount = buf[DOS_LOL_BLOCK_DEVICE_COUNT];
    DOS_DDH* NULHeader = (DOS_DDH*)&buf[DOS_LOL_NULDEV_HEADER];

    //fill CDS
    DOS_CDS* cds = (DOS_CDS*)malloc(DriveCount*sizeof(DOS_CDS)); //another local cache
    uint32_t CDSFarptr = *(uint32_t*)&buf[DOS_LOL_CDS_PTR];
    DPMI_CopyLinear(DPMI_PTR2L(cds), ((CDSFarptr>>12)&0xFFFFF)+(CDSFarptr&0xFFFF), sizeof(DOS_CDS)*DriveCount); //copy cds memory to dpmi
    int CDSIndex; //find an empty entry in CDS list
    for(CDSIndex = 0; CDSIndex < DriveCount; ++CDSIndex)
    {
        if(!(cds[CDSIndex].flags&DOS_CDS_FLAGS_USED))
            break;
    }
    cds[CDSIndex].flags |= DOS_CDS_FLAGS_USED;
    cds[CDSIndex].DPBptr = ((DrvMem&0xFFFF)<<16) | offsetof(USB_MSC_DOS_TSRDATA, dpb);
    memcpy(cds[CDSIndex].path, "", 0); //TODO:
    DPMI_CopyLinear(((CDSFarptr>>12)&0xFFFFFF)+(CDSFarptr&0xFFFF), DPMI_PTR2L(cds), sizeof(DOS_CDS)*DriveCount); //cds write back to dos mem

    //alter driver chain
    uint32_t next = NULHeader->NextDDH;
    NULHeader->NextDDH = ((DrvMem&0xFFFF)<<16) | offsetof(USB_MSC_DOS_TSRDATA, ddh);
    TSRData.ddh.NextDDH = next;

    //write tsr mem
    TSRData.dpb.DriverHeader = (DrvMem&0xFFFF) | offsetof(USB_MSC_DOS_TSRDATA, ddh);
    DPMI_CopyLinear((DrvMem&0xFFFF)<<4, DPMI_PTR2L(&TSRData), sizeof(TSRData));

    //build DPB http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2985.htm
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x53;
    reg.w.ds = (DrvMem&0xFFFF);
    reg.w.si = offsetof(USB_MSC_DOS_TSRDATA, bpb);
    reg.w.es = (DrvMem&0xFFFF);
    reg.w.bp = offsetof(USB_MSC_DOS_TSRDATA, dpb);
    DPMI_CallRealModeINT(0x21, &reg);

    //write back DOS lists
    //++buf[DOS_LOL_BLOCK_DEVICE_COUNT];
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg);
    DPMI_CopyLinear((uint32_t)(reg.w.es << 4) + reg.w.bx, DPMI_PTR2L(buf), DOS_LOL_SIZE); //copy to dpmi mem for easy access

    //clean up
    free(cds);
    free(buf);
    return TRUE;
}

BOOL USB_MSC_DOS_Uninstall()
{
    return FALSE;
}