#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <malloc.h>
#include "USBDDOS/CLASS/MSC.H"
#include "USBDDOS/DPMI/DPMI.H"
#include "USBDDOS/DBGUTIL.H"

static BOOL USB_MSC_ReadSector(USB_Device* pDevice, uint32_t sector, uint16_t count, void* buf, size_t size);

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
            uint32_t MaxLBA = EndianSwap32(data.LastLBA);
            uint32_t BlockSize = EndianSwap32(data.BlockSize);
            float cap = (float)MaxLBA / 1024.0f / 1024.0f / 1024.0f * (float)BlockSize;
            _LOG("LUN %d Capcacity: %.1f GB\n", i, cap);
            pDriverData->SizeGB = (uint16_t)(pDriverData->SizeGB + (uint16_t)cap);
            pDriverData->MaxLBA = MaxLBA;
            pDriverData->BlockSize = BlockSize;
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
    uint8_t* dma = (uint8_t*)DPMI_DMAMalloc(max(max(sizeof(USB_MSC_CBW),sizeof(USB_MSC_CSW)),DataSize), 16);
    memcpy(dma, &cbw, sizeof(cbw));
    uint16_t len = 0;
    uint16_t size = sizeof(cbw);
    uint8_t error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[0], dma, size, &len); //TODO: error code now is HC specific, need abstraction (wrapper).
    if(len != size || error)
    {
        _LOG("MSC CBW failed: %x, %d, %d.\n", error, size, len);
        USB_ClearHalt(pDevice, pDriverData->bEPAddr[0]);
        return FALSE;
    }
    
    //DATA
    if(DataSize)
    {
        memset(dma, 0, DataSize);
        if(dir == HCD_TXW)
            memcpy(dma, data, DataSize);
        len = 0;
        error = USB_SyncTransfer(pDevice, pDriverData->pDataEP[dir&0x1], dma, (uint16_t)DataSize, &len);
        if(dir == HCD_TXR)
            memcpy(data, dma, DataSize);
        if(len != DataSize || error)
        {
            _LOG("MSC DATA Failed: %x, %d, %d, %d.\n", error, dir, DataSize, len);
            USB_ClearHalt(pDevice, pDriverData->bEPAddr[dir&0x1]);
            return FALSE;
        }
    }

    //CSW
    len = 0;
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
    struct
    {
        uint16_t BPS;   //bytes per sector
        uint8_t SPC;    //sectors per cluster
        uint16_t RsvdSectors; //reserved sectors from the begginning
        uint8_t FATs;
        uint16_t RootDirs;
        uint16_t Sectors; //sectors in total(FAT12)
        uint8_t MediaDesc; //media descriptor
        uint16_t SPF; //sectors per fat
    }DOS20;
    //3.31
    struct
    {
        uint16_t SPT;   //sectors per track
        uint16_t Heads; //heads count
        uint32_t HiddenSectors;
        uint32_t Sectors2; //used if Sectors=0
    }DOS331;
    union
    {
        struct
        {
            //3.4
            uint8_t Drive;   //unit(drive) no.
            uint8_t ChkdskFlag;
            uint8_t Signature;
            uint32_t Serial;
            //4.0
            char Label[11];
            char FS[8]; //"FAT12" or "FAT16"
            uint8_t reserved[8];
        }DOS40;
        struct
        {
            //7.0+
            uint32_t SPF;   //sector per FAT
            uint16_t MirrorFlags;
            uint16_t Version;
            uint32_t RootDirCluster;
            uint16_t FSSector; //sector containing FS info
            uint16_t BackupSector;
            char BootFile[12];
            uint8_t Drive; //unit(drive) no
            uint8_t Flags;
            uint8_t Signature;
            uint32_t Serial;
            char Label[11];
            char FS[8]; //"FAT32"
        }DOS70;
    };
    uint8_t unused; //padding
}DOS_BPB;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_BPB) == 80, "incorrect size");
#endif

#define DOS_DDHA_32BIT_ADDRESSING   0x0002
#define DOS_DDHA_OPENCLOSE          0x0800

//https://faydoc.tripod.com/structures/25/2597.htm
typedef struct DOS_DeviceDriverHeader
{
    uint32_t NextDDH;
    uint16_t Attribs; //http://www.delorie.com/djgpp/doc/rbinter/it/48/16.html
    uint16_t StrategyEntryPoint; //newar ptr. segment is the header (far ptr)'s segment
    uint16_t IntEntryPoint; //near ptr
    uint8_t Drives; //supported drive count. TODO: mount multiple partitions? USB disk can have multiple partitions but usually only 1.
    uint8_t Signature[7];
}DOS_DDH;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_DDH) == 18, "incorrect size");
#endif

//https://faydoc.tripod.com/structures/13/1395.htm
//https://stanislavs.org/helppc/drive_parameter_table.html
typedef struct DOS_DriveParameterBlock //DOS4.0+
{
    uint8_t Drive; //0:A, 2:B, 3:C...
    uint8_t SubUnit;  //sub unit no.
    uint16_t BPS; //bytes per sector
    uint8_t SPC; //sectors per cluster - 1
    uint8_t C2SShift; //shift count for cluster to sector
    uint16_t RsvdSectors; //reserved sectors from the begginning
    uint8_t FATs;
    uint16_t RootDirs; //root dir count
    uint16_t User1stSector; //no. of first sector for user
    uint16_t MaxCluster;
    uint16_t SPF;    //sectors per fat //DOS 4.0+: 2 bytes
    uint16_t Dir1stSector; //no. of first sector for dir
    uint32_t DriverHeader; //addr of driver header
    uint8_t MediaID;
    uint8_t Accessed; //00 for true, FF for none
    uint32_t NextDPB;
    uint16_t Free1stCluster;
    uint32_t FreeClusters; //DOS7.0+: 4 bytes (teseted out)
    char cwd[63]; //current working directory, not sure for DOS7.0 but it execeeds the 33 limit for FAT32
}DOS_DPB;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_DPB) == 98, "incorrect size");
#endif

#define DOS_CDS_FLAGS_USED      0xC0
#define DOS_CDS_FLAGS_PHYSICAL  0x4000

//http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2983.htm#Table1643
typedef struct DOS_CurrentDirectoryStructure
{
    char path[67];
    uint16_t Flags; //DOS_CDS_FLAGS_*
    uint32_t DPBptr;
    uint16_t Cluster; //cwd's cluster. 0=root
    uint32_t FFFFFFFFh;
    uint16_t SlashPos; //usually 2
    uint8_t RemoteType; //04h=network
    uint32_t IFS; //IFS driver or redirector block
    uint16_t IFSData;
}DOS_CDS;
#if defined(__DJ2__)
_Static_assert(sizeof(DOS_CDS) == 88, "incorrect size"); //MUST be 88 as DOS stores it in arrays
#endif

//https://github.com/microsoft/MS-DOS/blob/master/v2.0/source/DEVDRIV.txt
//https://faydoc.tripod.com/structures/25/2597.htm

#define DOS_DRSCMD_MEDIACHECK   1
#define DOS_DRSCMD_BUILD_BPB    2
#define DOS_DRSCMD_READ         4
#define DOS_DRSCMD_WRITE        8
#define DOS_DRSCMD_WRITEVERIFY  9

#define DOS_DRSS_ERRORBIT       0x800
#define DOS_DRSS_DONEBIT        0x100
#define DOS_DRSS_BUSY           0x200
#define DOS_DRSS_UNKNOWN_UNIT   0x01
#define DOS_DRSS_DEV_NOT_READY  0x02
//3: unkown command
#define DOS_DRSS_CRC_ERROR      0x04
//5: bad DriverRequestStruct length
//6: seek erorr
//7: unknown media
#define DOS_DRSS_SECT_NOTFOUND  0x08
//9: printer out of paper
#define DOS_DRSS_WRITE_FAULT    0x0A
#define DOS_DRSS_READ_FAULT     0x0B
#define DOS_DRSS_GENERAL_FAULT  0x0C

//driver function parameter
typedef struct DOS_DriverRequestStruct
{
    struct
    {
        uint8_t Len;
        uint8_t SubUnit;
        uint8_t Cmd;
        uint16_t Status; //DOS_DRSS_*
        uint8_t link[8]; //DOS maintained 2 links (dos queue & device queue), don't care
    }Header;
    union 
    {
        struct
        {
            uint8_t MediaDesc;  //media descriptor from BPB
            uint32_t Address;   //transfer addr in memory
            uint16_t Count;     //byte/sector count
            uint16_t Start;     //starting sector to read/write
            uint32_t VolumeID;  //DOS 3.0+
            uint32_t Start32;   //DOS 4.0+ starting sector. used when DOS_DDHA_32BIT_ADDRESSING and Start=0xFFFF
        }ReadWrite;
        struct
        {
            uint8_t MediaDesc;
            uint8_t Returned;   //extra code. -1:changed, 1: not changed, 0: don't know. -1 wil cause clear cache and rebuild BPB
        }MediaCheck;
        struct 
        {
            uint8_t MediaDesc;
            uint32_t Address; //
            uint32_t BPBPtr;
        }BuildBPB;
    };

}DOS_DRS;

#if defined(__DJ2__)
#pragma pack()
#endif

//"List of Lists" offsets
#define DOS_LOL_DPB_PTR             0x00    //DPB list far ptr, 4 bytes
#define DOS_LOL_CDS_PTR             0x16    //CDS list far ptr, 4 bytes
#define DOS_LOL_BLOCK_DEVICE_COUNT  0x20    //installed device count, 1 byte
#define DOS_LOL_DRIVE_COUNT         0x21    //drive count (and CDS count), 1 byte
#define DOS_LOL_NULDEV_HEADER       0x22    //NUL device header, 18 bytes
#define DOS_LOL_SIZE                0x60

//ease of use memory layout
typedef struct 
{
    DOS_DDH ddh; //better put device driver header at the beginning
    DOS_BPB bpb;
    DOS_DPB dpb;
    uint32_t RequestPtr;        //far pointer to a DOS_DRS, saved by STG_opcodes (strategy entry point)
    uint8_t STG_opcodes[5+5+1]; //mov es:bx to RequestPtr + retf
    uint8_t INT_opcodes[5+4+1]; //call far ptr + retf + 4 push/pop
    uint8_t unused;             //padding
    uint32_t StartSector;       //base sector if mounted a partition instead of a whole disk. this value may > 0xFFFF if the first parition is not FAT and large enough.
                                //some weirld disk may have only 1 parition but with this offset > 0xFFFF, with unallocated space before it
    //PM entry points, called by INT_opcodes
    uint32_t INT_RMCB;
}USB_MSC_DOS_TSRDATA;

//https://en.wikipedia.org/wiki/Partition_type
static char USB_MSC_FAT_PartitionTypes[] = 
{
    0x01,   //FAT12
    0x04,   //FAT16
    0x0B,   //FAT32 with CHS
    0x0C,   //FAT32 with LBA
    0x0E,   //FAT16 with LBA
    0
};

static DOS_BPB* MSC_BPBs; //temp cache of all BPBs from DOS.
static uint32_t MSC_DriverINT_RMCB;
static DPMI_REG MSC_DOSDriverReg;   //RM regs

static void USB_MSC_DOS_DriverINT()
{
    //DBG_DumpREG(&MSC_DOSDriverReg);
    uint16_t cs = MSC_DOSDriverReg.w.ds; //cs point to device deriver header (begin of USB_MSC_DOS_TSRDATA struct), cs saved to ds

    //find target device
    USB_Device* pDevice = NULL;
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
        {
            if(!HCD_IS_DEVICE_VALID(pHCI->DeviceList[i]))
                continue;
            USB_Device* dev = HC2USB(pHCI->DeviceList[i]);
            if(dev->Desc.bDeviceClass == USBC_MASSSTORAGE && dev->bStatus == DS_Ready)
            {
                 USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)dev->pDriverData;
                 if((pDriverData->DOSDriverMem&0xFFFF) == cs)
                 {
                    pDevice = dev;
                    break;
                 }
            }
        }
    }

    uint32_t ReqFarPtr = DPMI_LoadD(DPMI_SEGOFF2L(cs, offsetof(USB_MSC_DOS_TSRDATA, RequestPtr)));
    DOS_DRS request;
    DPMI_CopyLinear(DPMI_PTR2L(&request), DPMI_FP2L(ReqFarPtr), sizeof(request));
    request.Header.Status = DOS_DRSS_DONEBIT;

    if(pDevice == NULL)
    {
        request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_DEV_NOT_READY;
        if(request.Header.Cmd == DOS_DRSCMD_MEDIACHECK)
            request.MediaCheck.Returned = 0xFF;
        if(request.Header.Cmd == DOS_DRSCMD_BUILD_BPB)
            request.BuildBPB.BPBPtr = 0;
    }
    else switch(request.Header.Cmd)
    {
        case DOS_DRSCMD_MEDIACHECK:
        {
            HCD_Interface* pHCI = pDevice->HCDDevice.pHCI;
            uint16_t PortStatus = pHCI->pHCDMethod->GetPortStatus(pHCI, pDevice->HCDDevice.bHubPort);
            if((PortStatus&USB_PORT_ATTACHED) && (PortStatus&USB_PORT_ENABLE) && !(PortStatus&USB_PORT_CONNECT_CHANGE))
                request.MediaCheck.Returned = 1;
            else
            {
                //request.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_DEV_NOT_READY;
                request.MediaCheck.Returned = 0xFF;
            }
            break;
        }
        case DOS_DRSCMD_BUILD_BPB:
        {
            request.BuildBPB.BPBPtr = DPMI_MKFP(cs, offsetof(USB_MSC_DOS_TSRDATA, bpb));
            break;
        }
        case DOS_DRSCMD_READ:
        {
            USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;

            USB_MSC_READ_CMD cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.opcode = USB_MSC_SBC_READ10;
            cmd.LUN = (uint8_t)request.Header.SubUnit&0x7U;
            uint32_t start = request.ReadWrite.Start == 0xFFFF ? request.ReadWrite.Start32 : (uint32_t)request.ReadWrite.Start;
            start += DPMI_LoadD(DPMI_SEGOFF2L(cs, offsetof(USB_MSC_DOS_TSRDATA, StartSector)));
            cmd.LBA = EndianSwap32(start);
            uint16_t len = (uint16_t)(request.ReadWrite.Count*pDriverData->BlockSize);
            cmd.TransferLength = EndianSwap16(request.ReadWrite.Count);
            //_LOG("START: %d, COUNT %d\n", start, request.ReadWrite.Count);
            uint8_t* dma = (uint8_t*)DPMI_DMAMalloc(len, 16);
            if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), dma, len, HCD_TXR))
            {
                request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_READ_FAULT;
                request.ReadWrite.Count = 0;
            }
            DPMI_CopyLinear(DPMI_FP2L(request.ReadWrite.Address), DPMI_PTR2L(dma), len);
            DPMI_DMAFree(dma);
            break;
        }
        case DOS_DRSCMD_WRITE: case DOS_DRSCMD_WRITEVERIFY:
        {
            USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;

            USB_MSC_WRITE_CMD cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.opcode = USB_MSC_SBC_WRITE10;
            cmd.LUN = (uint8_t)request.Header.SubUnit&0x7U;
            uint32_t start = request.ReadWrite.Start == 0xFFFF ? request.ReadWrite.Start32 : (uint32_t)request.ReadWrite.Start;
            start += DPMI_LoadD(DPMI_SEGOFF2L(cs, offsetof(USB_MSC_DOS_TSRDATA, StartSector)));
            cmd.LBA = EndianSwap32(start);
            uint16_t len = (uint16_t)(request.ReadWrite.Count*pDriverData->BlockSize);
            cmd.TransferLength = EndianSwap16(request.ReadWrite.Count);
            uint8_t* dma = (uint8_t*)DPMI_DMAMalloc(len, 16);
            DPMI_CopyLinear(DPMI_PTR2L(dma), DPMI_FP2L(request.ReadWrite.Address), len);
            if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), dma, len, HCD_TXW))
                request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_WRITE_FAULT;
            DPMI_DMAFree(dma);

            if(request.Header.Status == DOS_DRSS_DONEBIT && request.Header.Cmd == DOS_DRSCMD_WRITEVERIFY)
            {
                {
                    USB_MSC_VERIFY_CMD cmd;
                    memset(&cmd, 0, sizeof(cmd));
                    cmd.ByteChk = 1; //TODO: need this?
                    cmd.LUN = 0;
                    cmd.opcode = USB_MSC_SBC_VERIFY;
                    cmd.LBA = EndianSwap32((uint32_t)request.ReadWrite.Start);
                    cmd.VerificationLen = EndianSwap16(request.ReadWrite.Count);
                    if( USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), NULL, 0, HCD_TXW) != 0)
                        request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_WRITE_FAULT;
                }
                {//REQUEST SENSE to get verify result
                    USB_MSC_REQSENSE_CMD cmd;
                    memset(&cmd, 0, sizeof(cmd));
                    cmd.opcode = USB_MSC_SBC_REQSENSE;
                    cmd.AllocationLength = sizeof(USB_MSC_REQSENSE_DATA);
                    USB_MSC_REQSENSE_DATA* dma = (USB_MSC_REQSENSE_DATA*)DPMI_DMAMalloc(sizeof(USB_MSC_REQSENSE_DATA), 16);
                    if(!USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), dma, sizeof(USB_MSC_REQSENSE_DATA), HCD_TXR))
                        request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_GENERAL_FAULT;
                    if(dma->SenseKey) //TODO: spec on errorcode & sense code
                        request.Header.Status = DOS_DRSS_ERRORBIT | DOS_DRSS_GENERAL_FAULT;
                    DPMI_DMAFree(dma);
                }
            }
            if(request.Header.Status != DOS_DRSS_DONEBIT)
                request.ReadWrite.Count = 0;
            break;
        }
    }

    //writ back status
    DPMI_StoreW(DPMI_FP2L(ReqFarPtr) + offsetof(DOS_DRS, Header.Status), request.Header.Status);
    if(request.Header.Cmd == DOS_DRSCMD_MEDIACHECK)
        DPMI_StoreB(DPMI_FP2L(ReqFarPtr) + offsetof(DOS_DRS, MediaCheck.Returned), request.MediaCheck.Returned);
    if(request.Header.Cmd == DOS_DRSCMD_BUILD_BPB)
        DPMI_StoreD(DPMI_FP2L(ReqFarPtr) + offsetof(DOS_DRS, BuildBPB.BPBPtr), request.BuildBPB.BPBPtr);
    if(request.Header.Cmd == DOS_DRSCMD_READ || request.Header.Cmd == DOS_DRSCMD_WRITE || request.Header.Cmd == DOS_DRSCMD_WRITEVERIFY)
        DPMI_StoreW(DPMI_FP2L(ReqFarPtr) + offsetof(DOS_DRS, ReadWrite.Count), request.ReadWrite.Count);
}

static BOOL USB_MSC_DOS_InstallDevice(USB_Device* pDevice)     //ref: https://gitlab.com/FreeDOS/drivers/rdisk/-/blob/master/SOURCE/RDISK/RDISK.ASM
{
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;
    assert(pDriverData);

    USB_MSC_DOS_TSRDATA TSRData;
    memset(&TSRData, 0, sizeof(TSRData));
    {
        DOS_DDH ddh = {0xFFFFFFFF, DOS_DDHA_32BIT_ADDRESSING|DOS_DDHA_OPENCLOSE, offsetof(USB_MSC_DOS_TSRDATA, STG_opcodes), offsetof(USB_MSC_DOS_TSRDATA, INT_opcodes), 1, };
        memcpy(ddh.Signature, "USBDDOS", 7);
        TSRData.ddh = ddh;
    }

    uint32_t VBRSector = 0;
    {
        //read boot sector
        uint8_t* BootSector = (uint8_t*)malloc(pDriverData->BlockSize);
        BOOL result = USB_MSC_ReadSector(pDevice, 0, 1, BootSector, pDriverData->BlockSize); //read sector 0
        memcpy(&TSRData.bpb, BootSector+11, sizeof(DOS_BPB)); //BPB started at 0x0B
        
        //no BPB in MBR, try FAT partition's boot sector, aka volume boot record (VBR)
        if(!result || TSRData.bpb.DOS20.BPS != pDriverData->BlockSize || (memcmp(TSRData.bpb.DOS40.FS, "FAT",3) != 0 && memcmp(TSRData.bpb.DOS70.FS, "FAT",3) != 0))
        {
            int partition = 0;
            //find FAT partition (even unique partition in MBR might be any entry with empty entries around)
            while(partition < 4) //MBR max 4 paritions
            { //https://en.wikipedia.org/wiki/Master_boot_record#PTE 0x01BE=first entry offset in partition table, 16=entry size, 0x08=partition first sector offset(LBA)
                VBRSector = *(uint32_t*)&BootSector[0x01BE + partition*16 + 0x08]; 
                if(VBRSector != 0)
                {
                    char type = (char)BootSector[0x01BE + partition*16 + 0x04];
                    if(type != 0 && strchr(USB_MSC_FAT_PartitionTypes, type) != NULL)
                        break;
                }
                ++partition;
            }
            if(partition != 4)
            { //try mount the partition. during I/O, the actual sector is "Inputsector + VBRSector", VBRSector is recorded in USB_MSC_DOS_TSRDATA as StartSector
                result = USB_MSC_ReadSector(pDevice, VBRSector, 1, BootSector, pDriverData->BlockSize);
                memcpy(&TSRData.bpb, BootSector+11, sizeof(DOS_BPB));
            }
        }
        free(BootSector);

        if(!result)
        {
            printf("MSC: Error reading boot sector.\n");
            return FALSE;
        }
        if(TSRData.bpb.DOS20.BPS != pDriverData->BlockSize || (memcmp(TSRData.bpb.DOS40.FS, "FAT",3) != 0 && memcmp(TSRData.bpb.DOS70.FS, "FAT",3) != 0))
        {
            printf("MSC: BPB not found.\n");
            return FALSE;
        }
    }
    uint32_t FreeClusters = 0xFFFF;
    {
        TSRData.dpb.BPS = TSRData.bpb.DOS20.BPS;
        TSRData.dpb.SPC = TSRData.bpb.DOS20.SPC;
        TSRData.dpb.RsvdSectors = TSRData.bpb.DOS20.RsvdSectors;
        TSRData.dpb.FATs = TSRData.bpb.DOS20.FATs;
        TSRData.dpb.RootDirs = TSRData.bpb.DOS20.RootDirs;
        TSRData.dpb.SPF = TSRData.bpb.DOS20.SPF;
        TSRData.dpb.MediaID = TSRData.bpb.DOS20.MediaDesc; //hard disk=0xF8
        TSRData.dpb.NextDPB = 0xFFFF;
        //those fields are too short for FAT32, there must be extended fields to hold 32bit free cluster
        //TSRData.dpb.Free1stCluster = 0;
        //TSRData.dpb.FreeClusters = 0xFFFF;
        #if 0
        _LOG("SPF: %04x, %08x\n", TSRData.bpb.SPF, TSRData.bpb.DOS70.SPF);
        _LOG("RootDirCluster: %08x\n", TSRData.bpb.DOS70.RootDirCluster);
        _LOG("Label: %s\n", TSRData.bpb.DOS70.Label);
        _LOG("Serial: %x\n", TSRData.bpb.DOS70.Serial);
        _LOG("VBRSector: %x\n", VBRSector);
        #endif

        if(memcmp(TSRData.bpb.DOS70.FS, "FAT32",5) == 0)
        {
            //read FS Information sector to fill free cluster
            USB_MSC_READ_CMD cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.opcode = USB_MSC_SBC_READ10;
            cmd.LUN = 0;
            cmd.LBA = EndianSwap32(VBRSector + TSRData.bpb.DOS70.FSSector);
            cmd.TransferLength = EndianSwap16(1); //sector count
            uint8_t* FSSector = (uint8_t*)malloc(pDriverData->BlockSize);
            BOOL result = USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), FSSector, pDriverData->BlockSize, HCD_TXR);
            if(result)
            { //https://en.wikipedia.org/wiki/Design_of_the_FAT_file_system#FS_Information_Sector
                FreeClusters = *(uint32_t*)&FSSector[0x1E8];
                _LOG("MSC: FAT32 Free Clusters: %08x\n", FreeClusters);
                //TSRData.dpb.FreeClusters = FreeClusters; //need to set it after build DPB
            }
            free(FSSector);
        }
    }
    DPMI_REG reg;

    uint32_t DrvMem = DPMI_HighMalloc((sizeof(USB_MSC_DOS_TSRDATA)+15)>>4, TRUE); //allocate resident memory

    //DOS 2.0+ internal: get list of lists http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2983.htm
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg); //return ES:BX pointing to buffer

    uint8_t* buf = (uint8_t*)malloc(DOS_LOL_SIZE);
    DPMI_CopyLinear(DPMI_PTR2L(buf), DPMI_SEGOFF2L(reg.w.es, reg.w.bx), DOS_LOL_SIZE); //copy to dpmi mem for easy access
    uint8_t DriveCount = buf[DOS_LOL_DRIVE_COUNT]; //usually 26
    uint8_t DeviceCount = buf[DOS_LOL_BLOCK_DEVICE_COUNT];  //actual device count
    DOS_DDH* NULHeader = (DOS_DDH*)&buf[DOS_LOL_NULDEV_HEADER];
    _LOG("Drive Count: %d, Device Count: %d\n", DriveCount, DeviceCount); unused(DeviceCount);

    //fill CDS
    DOS_CDS* cds = (DOS_CDS*)malloc(DriveCount*sizeof(DOS_CDS)); //local cache
    uint32_t CDSFarPtr = *(uint32_t*)&buf[DOS_LOL_CDS_PTR];
    DPMI_CopyLinear(DPMI_PTR2L(cds), DPMI_FP2L(CDSFarPtr), sizeof(DOS_CDS)*DriveCount); //copy cds memory to dpmi
    uint8_t CDSIndex; //find an empty entry in CDS list
    BOOL overrided = FALSE;
    for(CDSIndex = 2; CDSIndex < DriveCount; ++CDSIndex) //start from C:
    {
        //_LOG("CDS Flags %d: %x ", CDSIndex, cds[CDSIndex].flags);
        if(!(cds[CDSIndex].Flags/*&DOS_CDS_FLAGS_USED*/)) //DOS_CDS_FLAGS_USED is not set on MSDOS7.0
            break;
        if((cds[CDSIndex].Flags&DOS_CDS_FLAGS_PHYSICAL))
        {
            //get the BPB of this disk/parition (probaby setup by BIOS) to see if it is the same as our USB disk/partition, if it is, override it.
            //compare BPB becausse it contains serial number which should be identical for disk/partition
            if(DPMI_CompareLinear(DPMI_PTR2L(MSC_BPBs+CDSIndex), DPMI_PTR2L(&TSRData.bpb), sizeof(TSRData.bpb.DOS20) + sizeof(TSRData.bpb.DOS331) + sizeof(TSRData.bpb.DOS40)) == 0)
            {
                overrided = TRUE;
                break;
            }
        }
    }
    //_LOG("Current BPB:\n");
    //DBG_DumpLB(DPMI_PTR2L(&TSRData.bpb), sizeof(DOS_BPB), NULL);

    if(CDSIndex == DriveCount)
    {
        DPMI_HighFree(DrvMem);
        free(buf);
        free(cds);
        printf("MSC: No free drive letter available.\n");
        return FALSE;
    }

    memset(&cds[CDSIndex], 0, sizeof(DOS_CDS));
    memcpy(cds[CDSIndex].path, "A:\\", 4);
    cds[CDSIndex].path[0] = (char)(cds[CDSIndex].path[0] + CDSIndex);
    cds[CDSIndex].Flags |= DOS_CDS_FLAGS_USED | DOS_CDS_FLAGS_PHYSICAL;
    cds[CDSIndex].DPBptr = DPMI_MKFP(DrvMem, offsetof(USB_MSC_DOS_TSRDATA, dpb));
    cds[CDSIndex].FFFFFFFFh = 0xFFFFFFFF;
    cds[CDSIndex].SlashPos = 2;
    DPMI_CopyLinear(DPMI_FP2L(CDSFarPtr) + CDSIndex*sizeof(DOS_CDS), DPMI_PTR2L(cds) + CDSIndex*sizeof(DOS_CDS), sizeof(DOS_CDS)); //cds write back to dos mem

    //write tsr mem
    TSRData.dpb.Drive = CDSIndex;
    TSRData.dpb.DriverHeader = DPMI_MKFP(DrvMem, offsetof(USB_MSC_DOS_TSRDATA, ddh));
    TSRData.StartSector = VBRSector;
    TSRData.INT_RMCB = MSC_DriverINT_RMCB;
    //mov cs:[RequestPtr+2], es
    int idx = 0;
    TSRData.STG_opcodes[idx++] = 0x2E;  //CS prefix
    TSRData.STG_opcodes[idx++] = 0x8C;  //mov sreg
    TSRData.STG_opcodes[idx++] = 0x06;  //ModR/M
    TSRData.STG_opcodes[idx++] = (offsetof(USB_MSC_DOS_TSRDATA, RequestPtr)+2) & 0xFF;
    TSRData.STG_opcodes[idx++] = (offsetof(USB_MSC_DOS_TSRDATA, RequestPtr)+2) >> 8;
    //mov cs:[RequestPtr], bx
    TSRData.STG_opcodes[idx++] = 0x2E;  //CS prefix
    TSRData.STG_opcodes[idx++] = 0x89;  //mov reg
    TSRData.STG_opcodes[idx++] = 0x1E;  //ModR/M
    TSRData.STG_opcodes[idx++] = offsetof(USB_MSC_DOS_TSRDATA, RequestPtr) & 0xFF;
    TSRData.STG_opcodes[idx++] = offsetof(USB_MSC_DOS_TSRDATA, RequestPtr) >> 8;
    //retf
    TSRData.STG_opcodes[idx++] = 0xCB;
    assert(idx == sizeof(TSRData.STG_opcodes));

    //RMCB real mode registers' CS is RMCB wrapper itself, save current CS to DS.
    idx = 0;
    //push ds
    TSRData.INT_opcodes[idx++] = 0x1E;
    //push cs
    TSRData.INT_opcodes[idx++] = 0x0E;
    //pop ds
    TSRData.INT_opcodes[idx++] = 0x1F;
    //call far(dword) ptr cs:[INT_RMCB]
    TSRData.INT_opcodes[idx++] = 0x2E;  //CS prefix
    TSRData.INT_opcodes[idx++] = 0xFF;  //call
    TSRData.INT_opcodes[idx++] = 0x1E;  //ModR/M
    TSRData.INT_opcodes[idx++] = offsetof(USB_MSC_DOS_TSRDATA, INT_RMCB) & 0xFF;
    TSRData.INT_opcodes[idx++] = offsetof(USB_MSC_DOS_TSRDATA, INT_RMCB) >> 8;
    //pop ds
    TSRData.INT_opcodes[idx++] = 0x1F;
    //retf
    TSRData.INT_opcodes[idx++] = 0xCB;
    assert(idx == sizeof(TSRData.INT_opcodes));
    DPMI_CopyLinear(DPMI_SEGOFF2L(DrvMem,0), DPMI_PTR2L(&TSRData), sizeof(TSRData));

    //build DPB http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2985.htm
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x53;
    reg.w.ds = (DrvMem&0xFFFF);
    reg.w.si = offsetof(USB_MSC_DOS_TSRDATA, bpb);
    reg.w.es = (DrvMem&0xFFFF);
    reg.w.bp = offsetof(USB_MSC_DOS_TSRDATA, dpb);
    if(memcmp(TSRData.bpb.DOS70.FS,"FAT32", 5) == 0) //FAT32?
    {
        reg.w.cx = 0x4558; //win95 FAT32
        reg.w.dx = 0x4152;
    }
    DPMI_CallRealModeINT(0x21, &reg);
    DPMI_StoreD(DPMI_SEGOFF2L(DrvMem, offsetof(USB_MSC_DOS_TSRDATA, dpb.FreeClusters)), FreeClusters); //this will boost initial DIR speed for calc free clusters

    //alter driver chain
    {
        uint32_t next = NULHeader->NextDDH;
        NULHeader->NextDDH = DPMI_MKFP(DrvMem, offsetof(USB_MSC_DOS_TSRDATA, ddh));
        DPMI_StoreD(DPMI_SEGOFF2L(DrvMem, offsetof(USB_MSC_DOS_TSRDATA, ddh.NextDDH)), next);//TSRData.ddh.NextDDH = next;
    }
    //DPB chain
    {
        uint32_t DPBFarPtr = *(uint32_t*)&buf[DOS_LOL_DPB_PTR];
        uint32_t NextDPB = DPBFarPtr;
        do
        {
            DPBFarPtr = NextDPB;
            //_LOG("DPB: %08x\n", DPBFarPtr);
            NextDPB = DPMI_LoadD(DPMI_FP2L(DPBFarPtr) + offsetof(DOS_DPB, NextDPB));
        } while((NextDPB&0xFFFF) != 0xFFFF);
        DPMI_StoreD(DPMI_FP2L(DPBFarPtr) + offsetof(DOS_DPB, NextDPB), cds[CDSIndex].DPBptr);
    }
    #if DEBUG && 0
    _LOG("DDH:\n");
    DBG_DumpLB(DPMI_SEGOFF2L(DrvMem,offsetof(USB_MSC_DOS_TSRDATA, ddh)), sizeof(DOS_DDH), NULL);
    _LOG("BPB:\n");
    DBG_DumpLB(DPMI_SEGOFF2L(DrvMem,offsetof(USB_MSC_DOS_TSRDATA, bpb)), sizeof(DOS_BPB), NULL);
    _LOG("DPB:\n");
    DBG_DumpLB(DPMI_SEGOFF2L(DrvMem,offsetof(USB_MSC_DOS_TSRDATA, dpb)), sizeof(DOS_DPB), NULL);
    _LOG("ENTRY POINTS:\n");
    DBG_DumpLB(DPMI_SEGOFF2L(DrvMem,offsetof(USB_MSC_DOS_TSRDATA, STG_opcodes)), 11+10, NULL);
    #endif
    
    //write back DOS lists
    if(!overrided)
        ++buf[DOS_LOL_BLOCK_DEVICE_COUNT];
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg);
    DPMI_CopyLinear(DPMI_SEGOFF2L(reg.w.es, reg.w.bx), DPMI_PTR2L(buf), DOS_LOL_SIZE); //copy dpmi mem to dos

    //_LOG("%04x:%04x %08x\n", DrvMem&0xFFFF,offsetof(USB_MSC_DOS_TSRDATA, dpb), cds[CDSIndex].DPBptr);
    //clean up
    free(cds);
    free(buf);

    //put DrvMem to DOSDriverMem of USB_MSC_DriverData (for uninstall)
    assert(pDriverData->DOSDriverMem == 0);
    pDriverData->DOSDriverMem = DrvMem;
    if(overrided)
        printf("Disk \'%s %s\': BIOS driver overrided, remounted as drive %c:.\n", pDevice->sManufacture, pDevice->sProduct, 'A' + CDSIndex);
    else
        printf("Disk \'%s %s\' mounted as drive %c:.\n", pDevice->sManufacture, pDevice->sProduct, 'A' + CDSIndex);
    return TRUE;
}

BOOL USB_MSC_DOS_Install()
 {
    assert(MSC_DriverINT_RMCB == 0);
    MSC_DriverINT_RMCB = DPMI_AllocateRMCB(&USB_MSC_DOS_DriverINT, &MSC_DOSDriverReg);
    if(MSC_DriverINT_RMCB == 0)
    {
        printf("MSC Driver Install: Error Allocating RMCB.\n");
        return FALSE;
    }

    int count = 0;
    for(int j = 0; j < USBT.HC_Count; ++j)
    {
        HCD_Interface* pHCI = USBT.HC_List+j;

        for(int i = 0; i < HCD_MAX_DEVICE_COUNT; ++i)
        {
            if(!HCD_IS_DEVICE_VALID(pHCI->DeviceList[i]))
                continue;
            USB_Device* dev = HC2USB(pHCI->DeviceList[i]);
            if(dev->Desc.bDeviceClass == USBC_MASSSTORAGE && dev->bStatus == DS_Ready)
                count += USB_MSC_DOS_InstallDevice(dev) ? 1 : 0;
        }
    }

    USB_MSC_PostDeInit(); //if TSRed, post-deinit clean up will not be called. call it here.

    if(count == 0)
        printf("No USB disk found.\n");
    return count > 0;
 }

BOOL USB_MSC_DOS_Uninstall()
{
    return FALSE;
}

//note: after USB driver take ownership from BIOS, reading sector is not available
//so do it before USB init (pre init) and cache the BPBs. those BPBs are used to compare later to decide if it is the same drive that to be mounted.
void USB_MSC_PreInit()
{
   //DOS 2.0+ internal: get list of lists http://mirror.cs.msu.ru/oldlinux.org/Linux.old/docs/interrupts/int-html/rb-2983.htm
    DPMI_REG reg;
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg); //return ES:BX pointing to buffer
    MSC_BPBs = (DOS_BPB*)malloc(sizeof(DOS_BPB)*26);

    uint8_t* buf = (uint8_t*)malloc(DOS_LOL_SIZE);
    DPMI_CopyLinear(DPMI_PTR2L(buf), DPMI_SEGOFF2L(reg.w.es, reg.w.bx), DOS_LOL_SIZE); //copy to dpmi mem for easy access
    uint8_t DriveCount = buf[DOS_LOL_DRIVE_COUNT]; //usually 26

    DOS_CDS* cds = (DOS_CDS*)malloc(DriveCount*sizeof(DOS_CDS)); //local cache
    uint32_t CDSFarPtr = *(uint32_t*)&buf[DOS_LOL_CDS_PTR];
    DPMI_CopyLinear(DPMI_PTR2L(cds), DPMI_FP2L(CDSFarPtr), sizeof(DOS_CDS)*DriveCount); //copy cds memory to dpmi
    uint32_t DRSMem = DPMI_HighMalloc((sizeof(DOS_DRS)+15)>>4, FALSE);
    for(int CDSIndex = 2; CDSIndex < DriveCount; ++CDSIndex) //start from C:
    {
        if((cds[CDSIndex].Flags&DOS_CDS_FLAGS_PHYSICAL))
        {
            //get the BPB of this disk to see if it is the same as our USB disk (probaby setup by BIOS), if it is, override it.
            uint32_t dpb = DPMI_LoadD(DPMI_PTR2L(&cds[CDSIndex])+offsetof(DOS_CDS, DPBptr));
            uint32_t DrvHeader = DPMI_LoadD(DPMI_FP2L(dpb)+offsetof(DOS_DPB, DriverHeader));
            uint32_t SectorBuffer = DPMI_HighMalloc((uint16_t)((DPMI_LoadW(DPMI_FP2L(dpb)+offsetof(DOS_DPB, BPS))+15U)>>4U), FALSE);

            DOS_DRS drs;
            drs.Header.Len = sizeof(drs.Header) + sizeof(drs.ReadWrite);
            drs.Header.Cmd = DOS_DRSCMD_READ;
            drs.Header.SubUnit = DPMI_LoadB(DPMI_FP2L(dpb)+offsetof(DOS_DPB,SubUnit));
            drs.Header.Status = 0;//DOS_DRSS_ERRORBIT;
            drs.ReadWrite.MediaDesc = DPMI_LoadB(DPMI_FP2L(dpb)+offsetof(DOS_DPB, MediaID));
            drs.ReadWrite.Address = DPMI_MKFP(SectorBuffer, 0);
            drs.ReadWrite.Count = 1;
            drs.ReadWrite.Start = (DPMI_LoadW(DPMI_FP2L(DrvHeader)+offsetof(DOS_DDH, Attribs))&DOS_DDHA_32BIT_ADDRESSING) ? 0xFFFF : 0;
            drs.ReadWrite.Start32 = 0;
            drs.ReadWrite.VolumeID = 0;
            DPMI_CopyLinear(DPMI_SEGOFF2L(DRSMem, 0), DPMI_PTR2L(&drs), sizeof(drs));
            //_LOG("Drive %c subunit: %d.\n", 'A' + CDSIndex, drs.Header.SubUnit);

            memset(&reg, 0, sizeof(reg));
            reg.w.cs = (uint16_t)(DrvHeader>>16);
            reg.w.ds = reg.w.cs;
            reg.w.ip = DPMI_LoadW(DPMI_FP2L(DrvHeader) + offsetof(DOS_DDH, StrategyEntryPoint));
            reg.w.es = (uint16_t)(DRSMem&0xFFFF);
            reg.w.bx = 0;
            if(DPMI_CallRealModeRETF(&reg) == 0)
            {
                _LOG("Drive %c Strategy OK.\n", 'A' + CDSIndex);
                memset(&reg, 0, sizeof(reg));
                reg.w.cs = (uint16_t)(DrvHeader>>16);
                reg.w.ds = reg.w.cs;
                reg.w.ip = DPMI_LoadW(DPMI_FP2L(DrvHeader) + offsetof(DOS_DDH, IntEntryPoint));
                reg.w.es = (uint16_t)(DRSMem&0xFFFF);
                reg.w.bx = 0;
                if(DPMI_CallRealModeRETF(&reg) == 0 && DPMI_LoadW(DPMI_SEGOFF2L(DRSMem, offsetof(DOS_DRS, Header.Status))) == DOS_DRSS_DONEBIT)
                {
                    _LOG("Drive %c IntEntry OK.\n", 'A' + CDSIndex);
                    //uint32_t bpb = DPMI_LoadD(DPMI_SEGOFF2L(DrvMem, offsetof(DOS_DRS, BuildBPB.BPBPtr)));
                    #if DEBUG
                    _LOG("Drive %c BPB:\n", 'A'+ CDSIndex);
                    DBG_DumpLB(DPMI_SEGOFF2L(SectorBuffer, 11), sizeof(DOS_BPB), NULL);
                    #endif
                    DPMI_CopyLinear(DPMI_PTR2L(MSC_BPBs+CDSIndex), DPMI_SEGOFF2L(SectorBuffer, 11), sizeof(DOS_BPB));
                }
                else
                    _LOG("Drive %c IntEntry: %x\n", 'A'+CDSIndex, DPMI_LoadW(DPMI_SEGOFF2L(DRSMem, offsetof(DOS_DRS, Header.Status))));
            }
            DPMI_HighFree(SectorBuffer);
        }
    }

    DPMI_HighFree(DRSMem);
    free(cds);
    free(buf);
}

void USB_MSC_PostDeInit()
{
    if(MSC_BPBs)
        free(MSC_BPBs);
    MSC_BPBs = NULL;
}

static BOOL USB_MSC_ReadSector(USB_Device* pDevice, uint32_t sector, uint16_t count, void* buf, size_t size)
{
    if(pDevice == NULL)
        return FALSE;
    USB_MSC_DriverData* pDriverData = (USB_MSC_DriverData*)pDevice->pDriverData;
    if(pDriverData == NULL)
        return FALSE;
    if(size < count * pDriverData->BlockSize || sector + count > pDriverData->MaxLBA)
    {
        assert(FALSE);
        return FALSE;
    }

    USB_MSC_READ_CMD cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = USB_MSC_SBC_READ10;
    cmd.LUN = 0;
    cmd.LBA = EndianSwap32(sector);
    cmd.TransferLength = EndianSwap16(count); //sector count
    return USB_MSC_IssueCommand(pDevice, &cmd, sizeof(cmd), buf, pDriverData->BlockSize, HCD_TXR);
}

#if DEBUG
void USB_MSC_Test() //dump DPB
{
    DPMI_REG reg;
    memset(&reg, 0, sizeof(reg));
    reg.h.ah = 0x52;
    DPMI_CallRealModeINT(0x21, &reg); //return ES:BX pointing to buffer

    uint8_t* buf = (uint8_t*)malloc(DOS_LOL_SIZE);
    memset(buf, 0, DOS_LOL_SIZE);
    DPMI_CopyLinear(DPMI_PTR2L(buf), DPMI_SEGOFF2L(reg.w.es, reg.w.bx), DOS_LOL_SIZE); //copy to dpmi mem for easy access
    {
        uint32_t DPBFarPtr = *(uint32_t*)&buf[DOS_LOL_DPB_PTR];
        uint32_t NextDPB = DPBFarPtr;
        do
        {
            DPBFarPtr = NextDPB;
            //_LOG("DPB: %08x\n", DPBFarPtr);
            NextDPB = DPMI_LoadD(DPMI_FP2L(DPBFarPtr) + offsetof(DOS_DPB, NextDPB));
        } while((NextDPB&0xFFFF) != 0xFFFF);
        DBG_DumpLB(DPMI_FP2L(DPBFarPtr), sizeof(DOS_DPB), NULL);
    }
    free(buf);
}
#endif