#ifndef _MSC_H_
#define _MSC_H_
//Mass Storage Class 
//ref: https://www.usb.org/document-library/mass-storage-class-specification-overview-14
// https://www.usb.org/sites/default/files/usbmassbulk_10.pdf
// https://www.usb.org/sites/default/files/usb_msc_boot_1.0.pdf

//the mass storage class support 2 modes: CBI(control/bulk/interrupt) and BBB(bulk only transport, BOT).
//CBI is for floppy disk only. and here it is not intended to support.

//Tested devices:
//Kingston DataTraveller        2G, FAT16, BPB in boot sector
//Toshiba Transmemory           8G, FAT32, BPB in FAT32 VBR
//USB2.0 disk (unknown vendor)  64G,FAT32, BPB in FAT32 VBR
//HP x306w                      64G,FAT32, BPB in FAT32 VBR

#include "USBDDOS/USB.H"

#define USB_REQ_TYPE_MSC    (USB_REQTYPE_CLASS | USB_REQREC_INTERFACE)

//bInterfaceProtocol
#define USB_MSC_PROTOCOL_CBI_I  0x0 //with command complettion interrupt
#define USB_MSC_PROTOCOL_CBI    1
#define USB_MSC_PROTOCOL_BBB    0x50

//bInterfaceSubClass
#define USB_MSC_CB_RBC      0x01 //www.t10.org/scsi-3.htm
#define USB_MSC_CB_ATAPI    0x02 //MMC-5, ftp.seagate.com/sff/INF-8070.PDF
#define USB_MSC_CB_UFI      0x04
#define USB_MSC_CB_SCSI     0x06 //www.t10.org/scsi-3.htm, www.t10.org/scsi-3.htm
#define USB_MSC_CB_LSDFS    0x07
#define USB_MSC_CB_IEEE1667 0x08

//bRequest
#define USB_REQ_MSC_RESET       0xFF //bulk only rest (out)
#define USB_REQ_MSC_GET_MAX_LUN 0xFE //get max lun (in)

//signature
#define USB_MSC_CBW_SIGNATURE 0x43425355
#define USB_MSC_CSW_SIGNATURE 0x53425355

//CSW status
#define USB_MSC_CSW_STATUS_PASSED       0
#define USB_MSC_CSW_STATUS_FAILED       1
#define USB_MSC_CSW_STATUS_PHASE_ERROR  2


//CBW & CSW are transported via data (bulk) transfer. CBW(OUT) -> data(IN/OUT) -> CSW(IN)

#if defined(__DJ2__)
#pragma pack(1)
#endif

//CBW
typedef struct USB_MSC_CommandBlockWrapper
{
    uint32_t dCBWSignature; //USB_MSC_CBW_SIGNATURE
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t bmCBWFlags; //bit 7: 0: out, 1: in. other bits obsolete or reserved and must be 0

    uint8_t bCBWLUN : 4;
    uint8_t reserved0 : 4;

    uint8_t bCBWCBLength : 5; //valid length of CBWCB
    uint8_t reserved1 : 3;

    uint8_t CBWCB[16]; //command block. significant bytes first
}USB_MSC_CBW;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_CBW) == 31, "incorrect size");
#endif

//CSW
typedef struct USB_MSC_CommandStatusWrapper
{
    uint32_t dCSWSignature; //USB_MSC_CSW_SIGNATURE
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t bCSWStatus; //USB_MSC_CSW_STATUS_*
}USB_MSC_CSW;

//below are SCSI commands used by CBW (the CBWCB field)
//not all commands, but essential for a usb booting device

//NCITS 306-1998 SCSI Block Commands (SBC), www.t10.org/scsi-3.htm
#define USB_MSC_SBC_INQUIRY         0x12
#define USB_MSC_SBC_READ10          0x28
#define USB_MSC_SBC_REQSENSE        0x03 //request sense
#define USB_MSC_SBC_TESTUNITREADY   0x00 //test unity ready
#define USB_MSC_SBC_MODESENSE       0x5A
#define USB_MSC_SBC_READ_CAP        0x25 //read capacity
#define USB_MSC_SBC_READ_TOC        0x43 //read TOC for CDROM
#define USB_MSC_SBC_FORMAT_UNIT     0x04
#define USB_MSC_SBC_VERIFY          0x2F
#define USB_MSC_SBC_WRITE10         0x2A

///INQUIRY
//Peripheral Device Type for INQUIRY
#define USB_MSC_PDT_SBC_DAD         0x00 //SBC Direct-access device
#define USB_MSC_PDT_CDROM           0x05
#define USB_MSC_PDT_OPTICAL         0x07 //non-CD optical
#define USB_MSC_PDT_RBC_DAD         0x0E //RBC Direct-access device

typedef struct USB_MSC_InquiryCommand
{
    uint8_t opcode; //USB_MSC_SBC_INQUIRY
    uint8_t reserved0 : 5; //zero
    uint8_t LUN : 3;
    uint8_t reserved1[2];
    uint8_t AllocationLength; //24h=sizeof(USB_MSC_INQUIRY_DATA)
    uint8_t reserved2;
    uint8_t PAD[6]; //zero
}USB_MSC_INQUIRY_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_INQUIRY_CMD) == 12, "incorrect size");
#endif

typedef struct USB_MSC_InquiryData
{
    uint8_t PDT : 5; //USB_MSC_PDT_*, byte 0, bit 0~4
    uint8_t reserved0 : 3;
    uint8_t reserved1 : 7;
    uint8_t RMB : 1;    //removable media bit
    uint8_t reserved2[2];
    uint8_t AdditionalLength;
    uint8_t reserved3[3]; //byte 5~7
    uint8_t VendorID[8];
    uint8_t ProductID[16];
    uint8_t ProductRevisionLevel[4];
}USB_MSC_INQUIRY_DATA;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_INQUIRY_DATA) == 36, "incorrect size");
#endif

///READ(10)
typedef struct USB_MSC_ReadCommand //read(10) command
{
    uint8_t opcode; //USB_MSC_SBC_READ10
    uint8_t reserved0 : 5;
    uint8_t LUN : 3;
    uint32_t LBA;   //need reverse endian
    uint8_t reserved1;
    uint16_t TransferLength; //length in logical blocks //need reverse endian
    uint8_t reserved2;
    uint8_t PAD[2];   //padding, need set to 0
}USB_MSC_READ_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_READ_CMD) == 12, "incorrect size");
#endif

///REQUEST SENSE
typedef USB_MSC_INQUIRY_CMD USB_MSC_REQSENSE_CMD; //opcode = USB_MSC_SBC_REQSENSE
typedef struct USB_MSC_ReqSenseData
{
    uint8_t ErrorCode : 7; //0x70~0x71
    uint8_t Valid : 1;
    uint8_t SenseKey : 4;
    uint8_t Reserved0 : 1;
    uint8_t ILI : 1;
    uint8_t Reserved : 2;
    uint32_t Information;
    uint8_t AdditionalSenseLength; //n-7
    uint32_t CommandSpecificInformation;
    uint8_t AdditionalSenseCode;
    uint8_t AdditionalSenseCodeQualifierOPT;
}USB_MSC_REQSENSE_DATA;


///TEST UNIT READY
typedef USB_MSC_INQUIRY_CMD USB_MSC_TESTUNITREADY_CMD; //opcode = USB_MSC_SBC_TESTUNITREADY, AllocationLength = 0

///MODE SENSE
typedef struct USB_MSC_ModeSenseCommand
{
    uint8_t opcode; //USB_MSC_SBC_MODESENSE
    uint8_t reserved0 : 3;
    uint8_t DBD : 1;
    uint8_t reserved1 : 1;
    uint8_t LUN : 3;
    uint8_t PageCode : 6;
    uint8_t reserved2 : 2;
    uint8_t reserved3[4];
    uint16_t ParamListLen; //need swap endian
    uint8_t reserved;
    uint8_t PAD[2];
}USB_MSC_MODESENSE_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_MODESENSE_CMD) == 12, "incorrect size");
#endif

///READ CAPACITY
typedef USB_MSC_INQUIRY_CMD USB_MSC_READCAP_CMD; //opcode = USB_MSC_SBC_READ_CAP, AllocationLength = 0
typedef struct USB_MSC_ReadCapData
{
    uint32_t LastLBA;   //last LBA address.need swap endian
    uint32_t BlockSize; //need swap endian
}USB_MSC_READCAP_DATA;

///READ TOC
typedef struct USB_MSC_ReadTOCCommand
{
    uint8_t opcode; //USB_MSC_SBC_READ_TOC
    uint8_t reserved0 : 1;
    uint8_t MSF : 1;
    uint8_t reserved1 : 6;
    uint8_t FormatA : 4;
    uint8_t reserved2 : 4;
    uint8_t reserved3[4];
    uint16_t AllocationLength; //need swap endian
    uint8_t reserved4 : 6;
    uint8_t FormatB : 2;
    uint8_t PAD[2];
}USB_MSC_READTOC_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_READTOC_CMD) == 12, "incorrect size");
#endif

typedef struct USB_MSC_ReadTOCData
{
    uint16_t TOCDataLen; //need swap endian = 0Ah, TOCDataLen itself excluded
    uint8_t FirstCompleteSN;    //first complete session number
    uint8_t LastCompleteSN;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t FirstTNInLastSession; //first track number in last complete session
    uint8_t reserved2;
    uint32_t LBA;   //LBA of FirstTNInLastSession
}USB_MSC_READTOC_DATA;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_READTOC_DATA) == 12, "incorrect size");
#endif

///FORMAT UNIT
typedef struct USB_MSC_FormatUnitCommand
{
    uint8_t opcode; //USB_MSC_SBC_FORMAT_UNIT
    uint8_t DefectListFormat : 3; //7
    uint8_t CmpList : 1;    //0
    uint8_t FmtData : 1;    //1
    uint8_t LUN : 3;
    uint8_t VendorSpecific;
    uint16_t Interleave; //need swap endian
    uint8_t reserved0;
    uint8_t PAD[6];
}USB_MSC_FORMATUNIT_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_FORMATUNIT_CMD) == 12, "incorrect size");
#endif

///VERIFY
typedef struct USB_MSC_VerifyCommand
{
    uint8_t opcode; //USB_MSC_SBC_VERIFY
    uint8_t reserved0 : 1;
    uint8_t ByteChk : 1;
    uint8_t reserved1 : 3;
    uint8_t LUN : 3;
    uint32_t LBA; //need swap endian
    uint8_t reserved2;
    uint16_t VerificationLen;
    uint8_t reserved3;
    uint8_t PAD[2];
}USB_MSC_VERIFY_CMD;
#if defined(__DJ2__)
_Static_assert(sizeof(USB_MSC_VERIFY_CMD) == 12, "incorrect size");
#endif

///WRITE(10)
typedef USB_MSC_READ_CMD USB_MSC_WRITE_CMD; //opcode = USB_MSC_SBC_WRITE10

#if defined(__DJ2__)
#pragma pack()
#endif

typedef struct
{
    uint8_t bInterface;
    uint8_t MaxLUN;
    uint32_t MaxLBA;
    uint32_t BlockSize;
    uint16_t SizeGB;
    void* pDataEP[2]; //bulk in/out
    uint8_t bEPAddr[2];
    uint32_t DOSDriverMem;
}USB_MSC_DriverData;

#ifdef __cplusplus
extern "C"
{
#endif

BOOL USB_MSC_InitDevice(USB_Device* pDevice);

BOOL USB_MSC_DeinitDevice(USB_Device* pDevice);

BOOL USB_MSC_BulkReset(USB_Device* pDevice);

BOOL USB_MSC_IssueCommand(USB_Device* pDevice, void* inputp cmd, uint32_t CmdSize, uint32_t LinearData, uint32_t DataSize, HCD_TxDir dir);

//separate dos installation routine, might be called delayed
BOOL USB_MSC_DOS_Install();
BOOL USB_MSC_DOS_Uninstall();

void USB_MSC_PreInit();
void USB_MSC_PostDeInit();

#ifdef __cplusplus
}
#endif

#endif //_MSC_H_