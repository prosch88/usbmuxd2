//
//  kis.h
//  usbmuxd2
//
//  Created by erd on 07.11.24.
//

#ifndef kis_h
#define kis_h

#include <stdint.h>

#define KIS_PORTAL_CONFIG       0x01
#define KIS_PORTAL_RSM          0x10
#define KIS_PORTAL_UNK_INFO     0x00

#define KIS_OPCODE_CFG_READ    0x00
#define KIS_OPCODE_CFG_WRITE   0x01
#define KIS_OPCODE_DATA_READ   0x02
#define KIS_OPCODE_DATA_WRITE  0x03

#define KIS_OPCODE_SOFTWARE    0x80


#define KIS_CONFIG_OPCODE_WRITE 0x01

#define KIS_CONFIG_OPCODE_ENABLE_A              0x0A // macOS writes to this
#define KIS_CONFIG_OPCODE_ENABLE_B              0x14 // macOS writes to this
#define KIS_CONFIG_OPCODE_ENABLE_EP_STATUS      0x16 // enable endpoint 2

#define KIS_CONFIG_ENABLE_A_VAL                 0x21 // Value to write to KIS_CONFIG_OPCODE_ENABLE_A
#define KIS_CONFIG_ENABLE_B_VAL                 0x01 // Value to write to KIS_CONFIG_OPCODE_ENABLE_B
#define KIS_CONFIG_ENABLE_EP_STATUS_USB_VAL     0x20000 // enable endpoint 2
#define KIS_CONFIG_ENABLE_EP_STATUS_KIS_VAL     0x30000 // enable endpoint 2


#define KIS_RSM_PARAM_TYPE_RSM1_RESPONSE        0x09
#define KIS_RSM_PARAM_TYPE_USB_PACKET           0x0D
#define KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET   0x16F

#define KIS_RSM_READSTATE_PARAM_GETINFO         0x100


#define KIS_INFO_TAG_aDFU 'aDFU'
#define KIS_INFO_TAG_RSM1 'RSM1'

#pragma pack(push)
#pragma pack(1)

#pragma mark static types

typedef struct{
  union {
    struct {
      uint8_t recipient :  5; ///< Recipient type tusb_request_recipient_t.
      uint8_t type      :  2; ///< Request type tusb_request_type_t.
      uint8_t direction :  1; ///< Direction type. tusb_dir_t
    } bmRequestType_bit;

    uint8_t bmRequestType;
  };

  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} usb_control_request_t;

typedef struct {
  uint8_t  bLength            ; ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType    ; ///< DEVICE Descriptor Type.
  uint16_t bcdUSB             ; ///< BUSB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H). This field identifies the release of the USB Specification with which the device and its descriptors are compliant.

  uint8_t  bDeviceClass       ; ///< Class code (assigned by the USB-IF). \li If this field is reset to zero, each interface within a configuration specifies its own class information and the various interfaces operate independently. \li If this field is set to a value between 1 and FEH, the device supports different class specifications on different interfaces and the interfaces may not operate independently. This value identifies the class definition used for the aggregate interfaces. \li If this field is set to FFH, the device class is vendor-specific.
  uint8_t  bDeviceSubClass    ; ///< Subclass code (assigned by the USB-IF). These codes are qualified by the value of the bDeviceClass field. \li If the bDeviceClass field is reset to zero, this field must also be reset to zero. \li If the bDeviceClass field is not set to FFH, all values are reserved for assignment by the USB-IF.
  uint8_t  bDeviceProtocol    ; ///< Protocol code (assigned by the USB-IF). These codes are qualified by the value of the bDeviceClass and the bDeviceSubClass fields. If a device supports class-specific protocols on a device basis as opposed to an interface basis, this code identifies the protocols that the device uses as defined by the specification of the device class. \li If this field is reset to zero, the device does not use class-specific protocols on a device basis. However, it may use classspecific protocols on an interface basis. \li If this field is set to FFH, the device uses a vendor-specific protocol on a device basis.
  uint8_t  bMaxPacketSize0    ; ///< Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid). For HS devices is fixed to 64.

  uint16_t idVendor           ; ///< Vendor ID (assigned by the USB-IF).
  uint16_t idProduct          ; ///< Product ID (assigned by the manufacturer).
  uint16_t bcdDevice          ; ///< Device release number in binary-coded decimal.
  uint8_t  iManufacturer      ; ///< Index of string descriptor describing manufacturer.
  uint8_t  iProduct           ; ///< Index of string descriptor describing product.
  uint8_t  iSerialNumber      ; ///< Index of string descriptor describing the device's serial number.

  uint8_t  bNumConfigurations ; ///< Number of possible configurations.
} usb_device_descriptor_t;

typedef union {
    struct {
        uint8_t     pad[0x3c];
        union {
            struct{
                uint32_t    hostTxEnd : 18;
                uint8_t     hostTxEndConfirmed;
            };
            uint32_t    hostTx;
        };
        uint8_t     pad2[0x3c];
        uint32_t    devTxEnd : 18;
        uint8_t     devTxEndConfirmed;
    };
} KIS_RSM1_info_transferstate;

typedef union {
    struct {
        uint32_t            tag;
        uint32_t            unk1;
        uint64_t            unk2;
        uint32_t            handles[0x20];
        uint8_t             pad[0x68];
        uint64_t            usbTxAddr;
        uint8_t             pad2[0x78];
        uint64_t            usbRxAddr;
        KIS_RSM1_info_transferstate state;
    };
    uint8_t deviceInfo[0x200];
} KIS_RSM1_info;

#pragma mark request

typedef struct {
    uint16_t identifier;        // Communication identifier
    uint8_t  version;           // Protocol version
    uint8_t  portal;            // The "portal" to connect to
    uint8_t  opcode;            // opcode
    uint32_t param : 24;
    uint32_t size;              // Size of the complete request, including the arguments and payload, excluding the header
    // Followed by arguments and payload data
} KIS_req_header;

/*
    good for KIS_OPCODE_DATA_READ and KIS_OPCODE_DATA_WRITE
 */
typedef struct {
    KIS_req_header hdr;
    uint64_t addr;
    uint32_t size;
    //data
} KIS_req_data;

/*
    good for KIS_OPCODE_CFG_READ and KIS_OPCODE_CFG_WRITE
 */
typedef union {
    KIS_req_header hdr;
    struct{
        uint16_t identifier;       // Communication identifier
        uint8_t  version;          // Protocol version
        uint8_t  portal;           // The "portal" to connect to
        uint8_t  opcode;           // opcode
        uint16_t param      : 10;
        uint16_t cfgSize    : 14;
        uint32_t reqSize;          // zero on read,
    };
} KIS_req_cfg;

typedef struct {
    KIS_req_cfg chdr;
    uint32_t value;
} KIS_req_config_wr32;

typedef struct {
    KIS_req_data dhdr;
    uint32_t value;
} KIS_req_data_wr32;

#pragma mark reply

typedef struct {
    uint16_t identifier;       // Communication identifier
    uint8_t  version;          // Protocol version
    uint8_t  portal;           // The "portal" to connect to
    uint8_t  opcode;           // opcode
    uint8_t  param;            // An index
    uint16_t flags;
    uint32_t dataSize;
    uint8_t data[0];
} KIS_reply_hdr;

typedef struct {
    KIS_reply_hdr hdr;
    KIS_RSM1_info deviceinfo;
    uint32_t rspsize;
    uint32_t statuscode;
} KIS_reply_RSM1_info;

typedef struct {
    uint32_t size; // Number of bytes read/written
    uint32_t status;
} KIS_status_reply_footer;

typedef struct {
    uint32_t usbPktLen : 24;
    uint8_t  usbTagEndpoint : 4;
    uint8_t  usbTagType : 4;
    uint8_t  usbData[0];
} KIS_USB_read_reply;

typedef uint8_t kis_rsm1_dirtymap[0x28];

typedef struct {
    kis_rsm1_dirtymap dirtymap;
    KIS_RSM1_info rsm1Info;
    uint32_t    devTxStart : 18;
    uint8_t     devTxStartUnk;
    uint8_t usbdata[0];
} KIS_RSM1_packet;

#define USB_TAG_PACKET_TYPE_USB 5
#define USB_TAG_PACKET_TYPE_INFO 8

typedef struct {
    KIS_reply_hdr hdr;
    uint32_t size; // Number of bytes read/written
    uint32_t status;
} KIS_reply_generic;

typedef struct {
    KIS_reply_hdr hdr;
    uint32_t data[];
//    uint32_t size; // Number of bytes read/written
//    uint32_t status;
} KIS_reply_unk32;

//typedef struct {
//    KIS_req_header hdr;
//    uint32_t vals[3];
//} KIS_req_config_wr32_3;
//

//

//
//
//
//
//
//
//
//
typedef struct {
    KIS_req_data req;
    uint32_t tag;
    //usb packet
} KIS_req_rsm1_packet;
//
////typedef struct {
////    KIS_req_header hdr;
////    uint16_t seq;
////    uint16_t tag;
////    uint32_t __zeropad;
////    uint32_t rcv_size;
////    uint32_t snd_size;
////} KIS_req_rsm1;
//
typedef struct {
    KIS_req_rsm1_packet pkt;
    usb_control_request_t usb;
} KIS_req_rsm1_control;

typedef struct {
    KIS_req_rsm1_packet pkt;
    uint32_t data[4];
} KIS_req_rsm1_packet_wr32_4;


#pragma pack(pop)

#endif /* kis_h */
