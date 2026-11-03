//
//  USBDeviceKIS.cpp
//  usbmuxd2
//
//  Created by erd on 27.11.24.
//
#include <libgeneral/macros.h>

#include "USBDeviceKIS.hpp"
#include "../Manager/USBDeviceManager.hpp"
#include "../MUXException.hpp"

#include "../external/libusb/descriptor.h"

#include <unistd.h>
#include <string.h>

#define USB_TIMEOUT 10000

//#ifdef XCODE
//#   undef USB_TIMEOUT
//#   define USB_TIMEOUT 0
//#endif



//#ifdef XCODE
//#define REPLICATE_TRAFFIC
//#endif


#define member_size(type, member) (sizeof( ((type *)0)->member ))

#pragma mark helpers
static void getUsbString(uint8_t *usbstr_, char *str){
    uint16_t *usbstr = (uint16_t*)usbstr_;
    retassure((usbstr[0] >> 8) == 3, "Not type string!");
    uint8_t len = (usbstr[0] & 0xff)/2 - 1;
    for (int i=0; i<len; i++) {
        str[i] = usbstr[i+1];
    }
    str[len] = '\0';
}

#pragma mark libusb_callback definitions
void rx_callback(struct libusb_transfer *xfer) noexcept;

#pragma mark libusb_callback implementations
void rx_ep_outrsp_callback(struct libusb_transfer *xfer) noexcept{
    std::shared_ptr<USBDeviceKIS> dev = *(std::shared_ptr<USBDeviceKIS> *)xfer->user_data;
//    debug("RX callback dev %d-%d len %d status %d", dev->_bus, dev->_address, xfer->actual_length, xfer->status);
    if(xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        try {
            uint32_t needsRxRemoved = dev->_killOutRSPLoop;
            bool removeTransfer = false;
            while (!removeTransfer && needsRxRemoved > 0) {
                removeTransfer = dev->_killOutRSPLoop.compare_exchange_strong(needsRxRemoved, needsRxRemoved-1);
                needsRxRemoved = dev->_killOutRSPLoop;
            }

            dev->_arrived_outrsp.post({(const void*)xfer->buffer,(size_t)xfer->actual_length});

            if (removeTransfer) goto error;

            retassure(libusb_submit_transfer(xfer) == LIBUSB_SUCCESS, "Failed to submit KIS transfer");
            return;
        } catch (tihmstar::exception &e) {
            debug("Failed to post _arrived_rsm1_info_xfer with err:\n%s",e.dumpStr().c_str());
        }
    }else{
        switch(xfer->status) {
            case LIBUSB_TRANSFER_ERROR:
                // funny, this happens when we disconnect the device while waiting for a transfer, sometimes
                info("[KIS] Device %d-%d RX aborted due to error or disconnect", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_TIMED_OUT:
                if (dev->_initIdx >= 1) {
                    if (libusb_submit_transfer(xfer) == LIBUSB_SUCCESS) {
                        return;
                    }
                    error("Failed to submit KIS transfer. Device no longer active?");
                }
                error("[KIS] RX transfer timed out for inactive device %d-%d", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_CANCELLED:
                debug("[KIS] Device %d-%d RX transfer cancelled", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_STALL:
                error("[KIS] RX transfer stalled for device %d-%d", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_NO_DEVICE:
                // other times, this happens, and also even when we abort the transfer after device removal
                info("[KIS] Device %d-%d RX aborted due to disconnect", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_OVERFLOW:
                error("[KIS] RX transfer overflow for device %d-%d", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_COMPLETED: //shut up compiler
            default:
                // this should never be reached.
                break;
        }
        dev->kill();
    }
error:
    //remove transfer
    {
        guardWrite(dev->_rx_xfers_Guard);
        dev->_rx_xfers.erase(xfer);
    }

    debug("[KIS] freing rx xfer for USBDeviceKIS(%s)",dev->_serial);
    safeFree(xfer->buffer);
    {
        std::shared_ptr<USBDevice> *cbargref = (std::shared_ptr<USBDevice> *)xfer->user_data; xfer->user_data = NULL;
        safeDelete(cbargref);
    }
    libusb_free_transfer(xfer);
}


#pragma mark USBDeviceKIS
USBDeviceKIS::USBDeviceKIS(Muxer *mux, USBDeviceManager *parent, uint16_t pid)
: USBDevice(mux, parent, pid)
,_ep_outrsp(-1)
, _kis_interface(-1), _kis_ep_in(-1), _kis_ep_out(-1)
, _initIdx(0xff01)
, _usbTxAddr(0), _usbRxAddr(0),
_trasferState{}, _transferDevReceivedFromHost(0)
, _rsm1PktSync(-1)
, _killOutRSPLoop(0)
{
    _usb_mtu = 0x8000 - 4; //TCP packets should be smaller than this!
}

USBDeviceKIS::~USBDeviceKIS(){
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf), "Detached KIS device %d-%d %s",_bus,_address, _serial);
        info("%s",buf);
        _mux->desktop_notification("USBDeviceKIS", buf);
    }
}

#pragma mark private

void USBDeviceKIS::kis_request_data_init(KIS_req_data *hdr, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t size){
    assure(size >= sizeof(hdr->hdr));
    size -= sizeof(hdr->hdr);
    assure(param < (1<<24));
    
    if (_initIdx < 0) {
        hdr->hdr.identifier = _initIdx++;
    }else{
        hdr->hdr.identifier = getpid();
    }
    
    hdr->hdr.version          = 0xA0;
    hdr->hdr.portal           = portal;
    hdr->hdr.opcode           = opcode;
    hdr->hdr.param            = param;
    hdr->hdr.size             = size;
}

void USBDeviceKIS::kis_request_cfg_init(KIS_req_cfg *hdr, uint8_t portal, uint8_t opcode, uint16_t param, uint16_t size){
    assure(size >= sizeof(hdr->hdr));
    size -= sizeof(hdr->hdr);
    assure(param < (1<<10));
    assure(size < (1<<14));

    if (size & 3) {
        size &= ~3;
        size += 4;
    }

    if (_initIdx < 0) {
        hdr->hdr.identifier = _initIdx++;
    }else{
        hdr->hdr.identifier = getpid();
    }
    hdr->version          = 0xA0;
    hdr->portal           = portal;
    hdr->opcode           = opcode;
    hdr->param            = param;
    hdr->cfgSize          = size>>2;
    if (opcode == KIS_OPCODE_CFG_READ) {
        hdr->reqSize      = 0;
    }else{
        hdr->reqSize      = size;
    }
}

void USBDeviceKIS::kis_process_data_for_portal_0(uint8_t *data, size_t dataSize, uint64_t reqaddr){
    
    if (_initIdx < 0) {
        kis_getinfo();
        
        retassure(reqaddr <= 0xfe0 && reqaddr + dataSize >= 0xfe4, "missing word at 0xfe0");
        uint8_t *targetEP = &data[0xfe0-reqaddr];
        
        bool isS9AndNewer = *targetEP == 2;
        if (isS9AndNewer) {
            _ep_out = 3;
            _ep_outrsp = 0x83;
            _ep_in = 0x84;
            {
                auto usbdev = _selfref.lock();
                usb_start_rx_loop(usbdev, _ep_outrsp, rx_ep_outrsp_callback, new USBDevice_receiver(usbdev.get()), USB_TIMEOUT);
            }
        }else{
            _ep_out = 1;
            _ep_outrsp = 0x81;
            _ep_in = 0x82;
        }
        kis_start_rxloops();
    }
    if (reqaddr <= 0x9000 && reqaddr + dataSize >= 0x9000+512) {
        uint8_t *rspdata = &data[0x9000-reqaddr];
        size_t rspDataSize = 512;
        /*
         In the init process, we check what pipes are available and enable them... I think
         */
        

        
        for (uint32_t *ptr = (uint32_t*)rspdata; (uint8_t*)(ptr+2)-(uint8_t*)rspdata < rspDataSize && *ptr; ptr+=2) {
            uint32_t v1 = ptr[0] & 0xffffffe0;
            uint32_t v2 = ptr[1] & 0xffffffe0;
            if (v1 == v2) {
                kis_data_write32(0x11, 0, 0x04 | ((uint64_t)v1<<8), 1);
            }
        }
    }
}

void USBDeviceKIS::kis_process_cfg_for_portal_rsm(uint8_t *data, size_t dataSize, uint64_t reqaddr){
    if (reqaddr == 0x0) {
        if (dataSize == 0) {
            debug("Got empty reply from kis_getinfo");
        }else{
            retassure(dataSize == 0x200, "getinfo response too small!");
            KIS_RSM1_info *deviceinfo = (KIS_RSM1_info*)data;
            retassure(deviceinfo->tag == KIS_INFO_TAG_RSM1, "Got unexpected response from KIS_PORTAL_RSM. Expected RSM1, but got '%.4s'",(char*)&deviceinfo->tag);
            std::unique_lock<std::mutex> ul(_trasferStateLck);
            _usbRxAddr = deviceinfo->usbRxAddr;
            _usbTxAddr = deviceinfo->usbTxAddr;
            _trasferState.devTxEnd = deviceinfo->state.devTxEnd;
            _trasferState.devTxEndConfirmed = deviceinfo->state.devTxEndConfirmed;
            _transferDevReceivedFromHost = deviceinfo->state.hostTxEnd;
            _trasferState.hostTxEndConfirmed = _trasferState.devTxEnd >> 10;
        }
    } else if (reqaddr == 0x30) {
        debug("Got reply from init sequce call");
    } else {
        reterror("%s: Unexpected reqaddr 0x%016llx found!",__FUNCTION__,reqaddr);
    }
}

void USBDeviceKIS::kis_process_rsm1_packet(uint8_t *data, size_t dataSize, uint16_t identifier){
    KIS_RSM1_packet *pkt = (KIS_RSM1_packet*)data;
    retassure(dataSize >= sizeof(*pkt), "rsm1 response packet too small to hold KIS_RSM1_packet");
    retassure(pkt->rsm1Info.tag == KIS_INFO_TAG_RSM1, "Got unexpected response from KIS_PORTAL_RSM. Expected RSM1, but got '%.4s'",(char*)&pkt->rsm1Info.tag);

    KIS_req_data pullreq = {};
    bool needsPullingData = false;
    {
        std::unique_lock<std::mutex> ul(_rsm1PktSyncLck);
        while (_rsm1PktSync != (uint16_t)(identifier - 1)) {
            /*
                Synchronise inbound packet processing
             */
            _rsm1PktSyncEvent.registerForTag(identifier - 1);
            ul.unlock();
            _rsm1PktSyncEvent.waitForTagArrival(identifier - 1);
            ul.lock();
        }
        
        if (_initIdx >= 0 && (*(uint32_t*)&pkt->dirtymap[0x14] <= 1 || _trasferState.devTxEnd != pkt->devTxStart)) {
            /*
                Note:   This should be (_trasferState.devTxEnd != pkt->devTxStart), but
                        when before an overflow _trasferState.devTxEnd == 0, then the check (_trasferState.devTxEnd != pkt->devTxStart) doesn't work.
                        
                        However by observing dirty words, we can figure it out too.
                        bit0 at pkt->dirtymap[0x14] is pkt->devTxStart
                        bit1 at pkt->dirtymap[0x14] is pkt->usbdata
                        If we have real data, then both, pkt->devTxStart and pkt->usbdata are available and more data follows, so *(uint32_t*)&pkt->dirtymap[0x14] >= 0b11
                        If no data is available, then either pkt->usbdata doesn't exist and pkt->devTxStart == 0, or sometimes even pkt->devTxStart doesn't exist. In that case also no payload data follows pkt->usbdata for the size of the usb buffer.
                        This means at that position the bitmap is either 1 or 0.
             */
            
            uint32_t pullAddr = 0;
            struct {
                uint32_t s : 20;
            } pullSize;
            pullAddr = _trasferState.devTxEnd << 2;
            pullSize.s = (pkt->rsm1Info.state.devTxEnd - _trasferState.devTxEnd) << 2;
            pullreq = (KIS_req_data){
                .hdr = {},
                .addr = _usbRxAddr + pullAddr,
                .size = pullSize.s,
            };
            kis_request_data_init(&pullreq, KIS_PORTAL_RSM, KIS_OPCODE_DATA_READ, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(pullreq));
            needsPullingData = true;
        }
        
        {
            _usbRxAddr = pkt->rsm1Info.usbRxAddr;
            _usbTxAddr = pkt->rsm1Info.usbTxAddr;
            _trasferState.devTxEnd = pkt->rsm1Info.state.devTxEnd;
            _trasferState.devTxEndConfirmed = pkt->rsm1Info.state.devTxEndConfirmed;
            _transferDevReceivedFromHost = pkt->rsm1Info.state.hostTxEnd;
            _trasferState.hostTxEndConfirmed = _trasferState.devTxEnd >> 10;
        }
        
        _rsm1PktSync = identifier;
        _rsm1PktSyncEvent.notifyWithTag(identifier);
    }

    if (needsPullingData) {
        kis_request(&pullreq.hdr, sizeof(pullreq));
    } else {
        uint8_t *usbpktdata = pkt->usbdata;
        struct {
            uint32_t s : 18;
        } size;
        size.s = (pkt->rsm1Info.state.devTxEnd-pkt->devTxStart)<<2;
        usb_packets_process(usbpktdata, size.s);
    }
}

void USBDeviceKIS::usb_packets_process(uint8_t *data, uint32_t len){
    int padding = 0;
    bool dontPull = false;
    for (KIS_USB_read_reply *usbHdr = (KIS_USB_read_reply*)data; len >= sizeof(*usbHdr); usbHdr = (KIS_USB_read_reply*)&usbHdr->usbData[usbHdr->usbPktLen+padding]) {
        retassure(len >= sizeof(*usbHdr), "packets len too small!");
        len -= sizeof(*usbHdr);
        
#ifdef XCODE
        assert(usbHdr->usbPktLen <= len);
#endif
        retassure(usbHdr->usbPktLen <= len, "packet goes out of bounds!");
        len -= usbHdr->usbPktLen;
        if (int remainder = usbHdr->usbPktLen & 3) {
            padding = (4-remainder);
            len -= padding;
        }else{
            padding = 0;
        }

        if (usbHdr->usbTagType == USB_TAG_PACKET_TYPE_INFO) {
            if (usbHdr->usbTagEndpoint == 1) {
                _infoPktEP1.post({(const void*)usbHdr->usbData,usbHdr->usbPktLen});
            }else if (usbHdr->usbTagEndpoint == 4) {
                if (_initIdx < 1) {
                    _infoPktEP4.post({(const void*)usbHdr->usbData,usbHdr->usbPktLen});
                }
            }
//#ifdef XCODE
//            char buf[0x100+usbHdr->usbPktLen*2];
//            int idx = 0;
//            idx = snprintf(buf, sizeof(buf), "Got infopacket on EP %d with content: ",usbHdr->usbTagEndpoint);
//            for (int i=0; i<usbHdr->usbPktLen; i++) {
//                snprintf(&buf[idx+i*2], sizeof(buf)-(idx+i*2), "%02x",usbHdr->usbData[i]);
//            }
//            debug("%s",buf);
//#endif
        }else if (usbHdr->usbTagType == USB_TAG_PACKET_TYPE_USB) {
            if (usbHdr->usbTagEndpoint == 0){
                _ep0Packets.post({(const void*)usbHdr->usbData,usbHdr->usbPktLen});
                
            } else if (usbHdr->usbTagEndpoint == (_kis_ep_in & LIBUSB_ENDPOINT_ADDRESS_MASK)) {
                if (!dontPull) {
                    dontPull = true;
                    debug("%s: kis_trigger_receive_interrupt_packet morelen=0x%x",__FUNCTION__,len);
                    kis_trigger_receive_interrupt_packet(_kis_ep_in);
                }
                USBDevice::device_data_input(usbHdr->usbData,usbHdr->usbPktLen);
            }else{
                reterror("Got USB packet on unexpected endpoint %d",usbHdr->usbTagEndpoint);
            }
        }else{
            reterror("Got USB packet of unexpected type 0x%x",usbHdr->usbTagType);
        }
    }
}

void USBDeviceKIS::kis_input_packet_process(KIS_reply_hdr *rsp, size_t rspSize, uint64_t reqaddr){
    retassure(sizeof(*rsp) <= rspSize, "rsp too small to fit reply header");
    retassure(sizeof(*rsp)+rsp->dataSize <= rspSize, "rsp too small to fit reply header");
    
#ifdef XCODE
    assert(sizeof(*rsp)+rsp->dataSize == rspSize); //response contains garbage at the end??
#endif
    retassure(rsp->dataSize >= sizeof(KIS_status_reply_footer), "rsp too small to fit reply footer");
    KIS_status_reply_footer *footer = (KIS_status_reply_footer*)&rsp->data[rsp->dataSize-sizeof(KIS_status_reply_footer)];
    retassure(footer->status == 0 || footer->status == 0x40, "rsp footer has unexpected status %d",footer->status);

    if (rsp->opcode == KIS_OPCODE_CFG_WRITE) {
//        debug("[RSP] KIS_OPCODE_CFG_WRITE wrote %d bytes to idx 0x%03x",footer->size,(uint16_t)reqaddr);
    } else if (rsp->opcode == KIS_OPCODE_CFG_READ) {
        switch (rsp->portal) {
            case KIS_PORTAL_RSM:
                kis_process_cfg_for_portal_rsm(rsp->data, footer->size, reqaddr);
                break;
                
            default:
                reterror("Got KIS_OPCODE_CFG_READ response from unknown portal %d",rsp->portal);;
                break;
        }
    } else if (rsp->opcode == KIS_OPCODE_DATA_READ) {
        switch (rsp->portal) {
            case 0:
                kis_process_data_for_portal_0(rsp->data, footer->size, reqaddr);
                break;
                
            case KIS_PORTAL_RSM:
                usb_packets_process(rsp->data, footer->size);
                break;
                
            default:
                reterror("Got KIS_OPCODE_DATA_READ response from unknown portal %d",rsp->portal);;
                break;
        }
    } else if (rsp->opcode == KIS_OPCODE_DATA_WRITE) {
//        debug("[RSP] KIS_OPCODE_DATA_WRITE wrote %d bytes to idx 0x%016llx",footer->size,reqaddr);

    } else if (rsp->opcode == KIS_OPCODE_SOFTWARE) {
        retassure(rsp->param == KIS_RSM_PARAM_TYPE_RSM1_RESPONSE, "Got KIS_OPCODE_SOFTWARE response with unexpected param 0x%x",rsp->param);
        kis_process_rsm1_packet(rsp->data, footer->size, rsp->identifier);
    } else {
        reterror("Got response from unknown opcode %d",rsp->opcode);
    }
}

void USBDeviceKIS::kis_request(KIS_req_header *req, size_t reqSize, bool dorealloc){
    size_t rspDataSize = 0;
    uint64_t reqAddr = 0;
    tihmstar::Mem rsp;
    {
        std::unique_lock<std::mutex> ul(_requestLck);
        switch (req->opcode) {
            case KIS_OPCODE_CFG_READ:
            {
                KIS_req_cfg *creq = (KIS_req_cfg*)req;
                retassure(req->size+sizeof(*req) >= sizeof(*creq), "KIS_req_data packet too small");
                rspDataSize = sizeof(KIS_reply_hdr) + (creq->cfgSize<<2) + sizeof(KIS_status_reply_footer);
                reqAddr = req->param & 0x3f;
            }
                break;

            case KIS_OPCODE_CFG_WRITE:
                rspDataSize = sizeof(KIS_reply_hdr) + sizeof(KIS_status_reply_footer);
                reqAddr = req->param & 0x3f;
                break;

            case KIS_OPCODE_DATA_READ:
            {
                KIS_req_data *dreq = (KIS_req_data*)req;
                retassure(req->size+sizeof(*req) >= sizeof(*dreq), "KIS_req_data packet too small");
                rspDataSize = sizeof(KIS_reply_hdr) + dreq->size + sizeof(KIS_status_reply_footer);
                reqAddr = dreq->addr;
            }
                break;

            case KIS_OPCODE_DATA_WRITE:
            {
                KIS_req_data *dreq = (KIS_req_data*)req;
                retassure(req->size+sizeof(*req) >= sizeof(*dreq), "KIS_req_data packet too small");
                rspDataSize = sizeof(KIS_reply_hdr) + sizeof(KIS_status_reply_footer);
                reqAddr = dreq->addr;

            }
                break;
            default:
                reterror("Unknown opcode %d sent to KIS!",req->opcode);
                break;
        }
        if (dorealloc) {
            void *buf = NULL;
            assure(buf = malloc(reqSize));
            memcpy(buf, req, reqSize);
            USBDevice::usb_send(buf, reqSize);
        }else{
            USBDevice::usb_send(req, reqSize);
        }
        rsp = _arrived_outrsp.wait();
        retassure(sizeof(KIS_reply_hdr) <= rsp.size(), "rsp too small to fit reply header");
        while (sizeof(KIS_reply_hdr)+((KIS_reply_hdr*)rsp.data())->dataSize > rsp.size()) {
            auto r2 = _arrived_outrsp.wait();
            rsp.append(r2.data(), r2.size());
        }
    }
    kis_input_packet_process((KIS_reply_hdr*)rsp.data(), rsp.size(), reqAddr);
}

void USBDeviceKIS::kis_request_data(KIS_req_data *req, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t requestSize, bool dorealloc){
    kis_request_data_init(req, portal, opcode, param, requestSize);
    kis_request(&req->hdr, requestSize, dorealloc);
}

void USBDeviceKIS::kis_request_cfg(KIS_req_cfg *req, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t requestSize, bool dorealloc){
    kis_request_cfg_init(req, portal, opcode, param, requestSize);
    kis_request(&req->hdr, requestSize, dorealloc);
}

void USBDeviceKIS::kis_config_write32(uint8_t portal, uint16_t param, uint32_t value){
    KIS_req_config_wr32 req  = {
        .value = value,
    };
    kis_request_cfg(&req.chdr, portal, KIS_OPCODE_CFG_WRITE, param, sizeof(req));
}

void USBDeviceKIS::kis_config_read(uint8_t portal, uint16_t param, uint16_t size){
    KIS_req_cfg req = {};
    kis_request_cfg_init(&req, portal, KIS_OPCODE_CFG_READ, param, size+sizeof(req));
    kis_request(&req.hdr, sizeof(req));
}

void USBDeviceKIS::kis_data_write32(uint8_t portal, uint16_t param, uint64_t addr, uint32_t value){
    KIS_req_data_wr32 req = {
        .dhdr = {
            .hdr = {},
            .addr = addr,
            .size = 4,
        },
        .value = value,
    };
    kis_request_data(&req.dhdr, portal, KIS_OPCODE_DATA_WRITE, param, sizeof(req));
}

void USBDeviceKIS::kis_wait_for_info_ep1(uint8_t v0, uint32_t v1){
    auto rsp = _infoPktEP1.wait();
    retassure(rsp.size() == 0x10, "_infoPktEP1 response got bad size!");
    uint32_t *v = (uint32_t*)rsp.data();
    retassure((v[0] & 0xff) == v0, "_infoPktEP1: v0 mismatch");
    retassure(v[1] == v1, "_infoPktEP1: v1 mismatch");
}

void USBDeviceKIS::kis_wait_for_info_ep4(uint8_t v0, uint32_t v1, uint32_t v2){
    auto rsp = _infoPktEP4.wait();
    retassure(rsp.size() == 0x10, "_infoPktEP4 response got bad size!");
    uint32_t *v = (uint32_t*)rsp.data();
    retassure((v[0] & 0xff) == v0, "_infoPktEP4: v0 mismatch");
    retassure(v[1] == v1, "_infoPktEP4: v1 mismatch");
    retassure(v[2] == v2, "_infoPktEP4: v2 mismatch");
}

void USBDeviceKIS::kis_wait_for_usb_process(uint32_t insize, uint32_t outsize){
    tihmstar::Mem pkt1;
    tihmstar::Mem pkt2;
    uint32_t *out = NULL;
    uint32_t *in = NULL;
    
    {
        pkt1 = _infoPktEP4.wait();
        retassure(pkt1.size() == 0x10, "_infoPktEP4 pkt1 response got bad size!");
        uint32_t *v = (uint32_t*)pkt1.data();
        switch ((v[0] & 0xff)) {
            case 0:
                retassure(in == NULL, "unexpected double in pkt");
                in = v;
                break;

            case 5:
                retassure(out == NULL, "unexpected double out pkt");
                out = v;
                break;

            default:
                reterror("unexpected packet type 0x%x",(v[0] & 0xff));
                break;
        }
    }

    {
        pkt2 = _infoPktEP4.wait();
        retassure(pkt2.size() == 0x10, "_infoPktEP4 pkt2 response got bad size!");
        uint32_t *v = (uint32_t*)pkt2.data();
        switch ((v[0] & 0xff)) {
            case 0:
                retassure(in == NULL, "unexpected double in pkt");
                in = v;
                break;

            case 5:
                retassure(out == NULL, "unexpected double out pkt");
                out = v;
                break;

            default:
                reterror("unexpected packet type 0x%x",(v[0] & 0xff));
                break;
        }
    }
    
    if (out) {
        uint32_t *v = out;
        retassure((v[0] & 0xff) == 5, "_infoPktEP4: outsize v0 mismatch");
        retassure(v[1] == 1, "_infoPktEP4: outsize v1 mismatch");
        retassure(v[2] <= outsize, "_infoPktEP4: outsize v2 mismatch");
    }

    if (in) {
        uint32_t *v = in;
        retassure((v[0] & 0xff) == 0, "_infoPktEP4: insize v0 mismatch");
        retassure(v[1] == 1, "_infoPktEP4: insize v1 mismatch");
        retassure(v[2] == insize, "_infoPktEP4: insize v2 mismatch");
    }
}

void USBDeviceKIS::kis_wait_for_interrupt_receive_on_ep(uint8_t ep){
    if (_initIdx < 1) {
        kis_wait_for_info_ep4(0x00, 0x01 | ((uint16_t)ep << 8), 0x8000); //receive dummy packet
    }
}

void USBDeviceKIS::kis_rsm_usb_write_32_4(uint32_t tag, uint32_t v0, uint32_t v1, uint32_t v2, uint32_t v3, bool waitForEP1){
    std::unique_lock<std::mutex> ul(_cfgWriteLck);
    {
        std::unique_lock<std::mutex> ul(_trasferStateLck);
        KIS_req_rsm1_packet_wr32_4 kproc = {
            .pkt ={
                .req = {
                    .addr = _usbTxAddr,
                    .size = 0x14,
                },
                    .tag = tag,
            },
                .data = {
                    v0,
                    v1,
                    v2,
                    v3
                }
        };
        
        kproc.pkt.req.addr += _trasferState.hostTxEnd<<2;
        /*
         Transfer trigger USB Packet to KIS
         */
        kis_request_data(&kproc.pkt.req, KIS_PORTAL_RSM, KIS_OPCODE_DATA_WRITE, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(kproc));
        
        /*
         Trigger processing of USB packet
         */
        _trasferState.hostTxEnd+=5;
        kis_config_write32(KIS_PORTAL_RSM, KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET, _trasferState.hostTx);
    }
    if (waitForEP1) {
        kis_wait_for_info_ep1(v0, v1);
    }
}

void USBDeviceKIS::kis_rsm_usb_write_32_0(uint32_t tgt, bool wait){
    std::unique_lock<std::mutex> ul(_cfgWriteLck);
    {
        std::unique_lock<std::mutex> ul(_trasferStateLck);
        KIS_req_rsm1_packet kproc = {
            .req = {
                .addr = _usbTxAddr,
                .size = 0x4,
            },
            .tag = tgt,
        };
        
        kproc.req.addr += _trasferState.hostTxEnd<<2;
        /*
         Transfer trigger USB Packet to KIS
         */
        kis_request_data(&kproc.req, KIS_PORTAL_RSM, KIS_OPCODE_DATA_WRITE, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(kproc));
        
        /*
         Trigger processing of USB packet
         */
        _trasferState.hostTxEnd+=5;
        kis_config_write32(KIS_PORTAL_RSM, KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET, _trasferState.hostTx);
    }
    if (wait) {
        assert(0); //TODO
    }
}

void USBDeviceKIS::kis_clear_flag_status(){
    kis_config_write32(KIS_PORTAL_RSM, 1, 0xffffffff);
}

void USBDeviceKIS::kis_getinfo(){
    kis_config_read(KIS_PORTAL_RSM, KIS_RSM_READSTATE_PARAM_GETINFO, member_size(KIS_RSM1_info, deviceInfo));
}

int USBDeviceKIS::kis_send_control_packet(uint8_t bm_request_type, uint8_t b_request, uint16_t w_value, uint16_t w_index, unsigned char *data, uint16_t w_length){
    std::unique_lock<std::mutex> ull(_ep0Lck);
    {
        std::unique_lock<std::mutex> ul(_trasferStateLck);

        KIS_req_rsm1_control kreq = {
            .pkt ={
                .req = {
                    .addr = _usbTxAddr,
                    .size = static_cast<uint32_t>(sizeof(kreq)-sizeof(kreq.pkt.req)),
                },
                .tag = 0x10000008,
            },
            .usb = {
                .bmRequestType = bm_request_type,
                .bRequest = b_request,
                .wValue = w_value,
                .wIndex = w_index,
                .wLength = (uint16_t) w_length,
            },
        };
        
        KIS_req_rsm1_packet_wr32_4 kproc = {
            .pkt ={
                .req = {
                    .addr = _usbTxAddr,
                    .size = 0x14,
                },
                .tag = 0x84000010,
            },
            .data = {
                0x1000,
                0x01,
                0x000000, //reply len!
                0x0
            },
        };
        
        if ((bm_request_type & 0x80) == 0) {
            kreq.pkt.req.size += w_length;
            assert(w_length == 0); //TODO
        }else{
            kproc.data[2] = w_length;
        }
        
        if (kreq.pkt.req.size & 3) {
            kreq.pkt.req.size &= ~3;
            kreq.pkt.req.size += 4;
        }

        kreq.pkt.req.addr += _trasferState.hostTxEnd<<2;
        /*
            Transfer USB Packet to KIS
         */
        kis_request_data(&kreq.pkt.req, KIS_PORTAL_RSM, KIS_OPCODE_DATA_WRITE, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(kreq));

        /*
            Trigger processing of USB packet
         */
        _trasferState.hostTxEnd+=3;
        kis_config_write32(KIS_PORTAL_RSM, KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET, _trasferState.hostTx);
        
        if (bm_request_type & 0x80) {
            kproc.pkt.req.addr += _trasferState.hostTxEnd<<2;
            /*
                Transfer trigger USB Packet to KIS
             */
            kis_request_data(&kproc.pkt.req, KIS_PORTAL_RSM, KIS_OPCODE_DATA_WRITE, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(kproc));

            /*
                Trigger processing of USB packet
             */
            _trasferState.hostTxEnd+=5;
            kis_config_write32(KIS_PORTAL_RSM, KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET, _trasferState.hostTx);
        }
    }
    kis_wait_for_usb_process(sizeof(usb_control_request_t), w_length);
    if (bm_request_type & 0x80) {
        tihmstar::Mem rsp = _ep0Packets.wait();
        retassure(rsp.size() <= w_length, "received ctrl packet size mismatch!");
        memcpy(data, rsp.data(), rsp.size());
        return (int)rsp.size();
    }else{
        return w_length;
    }
}

usb_device_descriptor_t USBDeviceKIS::usb_request_descriptor(){
    usb_device_descriptor_t ret = {};
    int rsp = 0;
    rsp = kis_send_control_packet(0x80, 0x06, 0x0100, 0x0000, (unsigned char*)&ret, sizeof(ret));
    retassure(rsp == sizeof(ret), "Failed to receive usb descriptor");
    return ret;
}

uint16_t USBDeviceKIS::usb_request_langid(){
    uint8_t data[0x10];
    int rsp = 0;
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_DESCRIPTOR, LIBUSB_DT_STRING << 8, 0x0000, (unsigned char*)&data, sizeof(data));
    retassure(rsp <= sizeof(data) && rsp >=4, "Failed to receive usb descriptor");

    uint16_t langid = (uint16_t)(data[2] | (data[3] << 8));
    return langid;
}

libusb_bos_descriptor *USBDeviceKIS::usb_request_bos_descriptor(){
    libusb_bos_descriptor *tmp = NULL;
    cleanup([&]{
        safeFree(tmp);
    });
    size_t retSize = sizeof(*tmp);
    int rsp = 0;

    tmp = (libusb_bos_descriptor*)calloc(1, retSize);
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_DESCRIPTOR, LIBUSB_DT_BOS << 8, 0x0000, (unsigned char*)tmp, retSize);
    retassure(rsp == retSize, "Failed to receive bos descriptor header");

    assure(tmp = (libusb_bos_descriptor*)realloc(tmp, retSize = tmp->wTotalLength));
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_DESCRIPTOR, LIBUSB_DT_BOS << 8, 0x0000, (unsigned char*)tmp, retSize);
    retassure(rsp == retSize, "Failed to receive bos descriptor");
    
    
    libusb_bos_descriptor *ret = NULL;
    ret = (libusb_bos_descriptor*)calloc(1, sizeof(*ret) + sizeof(*ret->dev_capability)*tmp->bNumDeviceCaps);
    ret->bLength = tmp->bLength;
    ret->bDescriptorType = tmp->bDescriptorType;
    ret->wTotalLength = tmp->wTotalLength;
    ret->bNumDeviceCaps = 0;
    {
        struct libusb_bos_dev_capability_descriptor *curCap = (struct libusb_bos_dev_capability_descriptor *)(&tmp->bNumDeviceCaps+1);
        for (int i=0; i<tmp->bNumDeviceCaps; i++) {
            ret->dev_capability[i] = (struct libusb_bos_dev_capability_descriptor *)calloc(1, curCap->bLength);
            memcpy(ret->dev_capability[i], curCap, curCap->bLength);
            curCap = (struct libusb_bos_dev_capability_descriptor *)((uint8_t*)curCap + curCap->bLength);
            ret->bNumDeviceCaps++;
        }
    }

    return ret;
}

libusb_config_descriptor *USBDeviceKIS::usb_request_config_descriptor(uint8_t config_index){
    libusb_config_descriptor *tmp = NULL;
    libusb_config_descriptor *ret = NULL;
    cleanup([&]{
        safeFreeCustom(ret, libusb_free_config_descriptor);
        safeFree(tmp);
    });
    size_t retSize = sizeof(*tmp);
    int rsp = 0;
    
#ifdef REPLICATE_TRAFFIC
    static_assert(offsetof(libusb_config_descriptor, wTotalLength) + sizeof(tmp->wTotalLength) <= 9, "libusb_config_descriptor too small, wtf");
    retSize = 9;
#endif
    
    tmp = (libusb_config_descriptor*)calloc(1, retSize);
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_CONFIG << 8) | config_index, 0x0000, (unsigned char*)tmp, retSize);
    retassure(rsp == retSize, "Failed to receive config descriptor header");
    
    assure(tmp = (libusb_config_descriptor*)realloc(tmp, retSize = tmp->wTotalLength));
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_CONFIG << 8) | config_index, 0x0000, (unsigned char*)tmp, retSize);
    retassure(rsp == retSize, "Failed to receive config descriptor");
    
    ret = (libusb_config_descriptor*)calloc(1, sizeof(libusb_config_descriptor));
    rsp = mylibusb_parse_configuration(ret, (uint8_t*)tmp, (int)retSize);
    
    retassure(rsp == LIBUSB_SUCCESS, "Failed to parse libusb_config_descriptor");
    
    {
        libusb_config_descriptor *r = ret; ret = NULL;
        return r;
    }
}

uint8_t USBDeviceKIS::usb_request_get_configuration(){
    uint8_t ret = -1;
    int rsp = 0;
    rsp = kis_send_control_packet(0x80, LIBUSB_REQUEST_GET_CONFIGURATION, 0x0000, 0x0000, (unsigned char*)&ret, sizeof(ret));
    retassure(rsp == sizeof(ret), "Failed to get configuration");
    return ret;
}

void USBDeviceKIS::usb_request_set_configuration(uint8_t cfg){
    int rsp = 0;
    rsp = kis_send_control_packet(0x00, LIBUSB_REQUEST_SET_CONFIGURATION, cfg, 0x0000, NULL, 0);
    retassure(rsp == 0, "Failed to set configuration");
}

void USBDeviceKIS::performInternalDeviceInit(){
    /*
        Now perform KIS configuration
     */
    uint16_t waittag = _rsm1PktSync+1;
    _rsm1PktSyncEvent.registerForTag(waittag);
    _rsm1PktSyncEvent.registerForTag(waittag+1);

    kis_config_write32(KIS_PORTAL_CONFIG, KIS_CONFIG_OPCODE_ENABLE_A, KIS_CONFIG_ENABLE_A_VAL);
    kis_config_write32(KIS_PORTAL_CONFIG, KIS_CONFIG_OPCODE_ENABLE_B, KIS_CONFIG_ENABLE_B_VAL);

    kis_clear_flag_status();
    kis_config_write32(KIS_PORTAL_CONFIG, KIS_CONFIG_OPCODE_ENABLE_EP_STATUS, KIS_CONFIG_ENABLE_EP_STATUS_USB_VAL);
    {
        KIS_req_data req = {
            .hdr = {},
            .addr = 0x0,
            .size = 0x10000,
        };
        kis_request_data(&req, KIS_PORTAL_UNK_INFO, KIS_OPCODE_DATA_READ, 0x0D, sizeof(req));
    }
    info("Done writing initial init sequence, waiting for response on EP %d", _ep_in & 0x7f);

    _rsm1PktSyncEvent.waitForTagArrival(waittag);

    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf), "Device %d-%d, got stage 1 response on EP%d, performing stage 2",_bus,_address,_ep_in & 0x7f);
        info("%s",buf);
        _mux->desktop_notification("USBDeviceKIS", buf);
    }

    _initIdx = 0; //we are no longer in init state!

    /*
        Continue KIS init
     */
    kis_rsm_usb_write_32_4(0x80000010, 0x01, 0x7114, 0, 0);
    
    kis_rsm_usb_write_32_4(0x80000010, 0x05, 0, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x03, 0, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x04, 0, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x03, 0, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x10, 1, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x16, 1, 0, 0);

    {
        kis_rsm_usb_write_32_4(0x80000010, 0x04, 0, 0, 0);
        kis_wait_for_info_ep1(0x18, 1);
    }
    
    kis_rsm_usb_write_32_4(0x80000010, 0x03, 0, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x16, 1, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x16, 1, 0x40000, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x16, 1, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x14, 1, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x16, 1, 0, 0);
    kis_rsm_usb_write_32_4(0x80000010, 0x30, 1, 0, 0);

    {
        kis_rsm_usb_write_32_4(0x80000010, 0x40, 1, 0x400000, 0x40);
        kis_wait_for_info_ep4(0x00, 1, 8);
    }

    /*
        KIS is inited, now actually talk to the device
     */
    libusb_bos_descriptor *bos = NULL;
    libusb_config_descriptor *config = NULL;
    cleanup([&]{
        safeFreeCustom(config, libusb_free_config_descriptor);
        safeFreeCustom(bos, libusb_free_bos_descriptor);
    });
    uint8_t cfg = -1;
    
    usb_device_descriptor_t desc = usb_request_descriptor();
    
    retassure(desc.idVendor == VID_APPLE, "USBDevice is not an Apple device");
    retassure((desc.idProduct >= PID_RANGE_LOW && desc.idProduct <= PID_RANGE_MAX), "USBDevice is Apple, but PID is not in observe range");
    info("Found new KIS device with v/p %04x:%04x at %d-%d", desc.idVendor, desc.idProduct, _bus, _address);

#ifdef REPLICATE_TRAFFIC
    {
        uint8_t data[0x400] = {};
        int rsp = 0;
        
        /*
            Get device Type
         */
        rsp = kis_send_control_packet(0x80, 0x06, 0x0302, 0x0409, data, 2);
        retassure(rsp > 0, "Failed to get KIS device type len");
        rsp = kis_send_control_packet(0x80, 0x06, 0x0302, 0x0409, data, sizeof(data));
        retassure(rsp > 0, "Failed to get KIS device type");
        getUsbString(data, (char*)&data[0x200]);
        info("Device Type: '%s'",(char*)&data[0x200]);

        /*
            Get device vendor
         */
        rsp = kis_send_control_packet(0x80, 0x06, 0x0301, 0x0409, data, 2);
        retassure(rsp > 0, "Failed to get KIS device vendor len");
        rsp = kis_send_control_packet(0x80, 0x06, 0x0301, 0x0409, data, data[0]);
        retassure(rsp > 0, "Failed to get KIS device vendor");
        getUsbString(data, (char*)&data[0x200]);
        info("Device Vendor: '%s'",(char*)&data[0x200]);
        
        
        /*
            Get device serial
         */
        rsp = kis_send_control_packet(0x80, 0x06, 0x0303, 0x0409, data, 2);
        retassure(rsp > 0, "Failed to get KIS device serial len");
        rsp = kis_send_control_packet(0x80, 0x06, 0x0303, 0x0409, data, data[0]);
        retassure(rsp > 0, "Failed to get KIS device serial");
        getUsbString(data, (char*)&data[0x200]);
        info("Device Serial: '%s'",(char*)&data[0x200]);
        {
            unsigned int di = 0, si = 2;
            for (; si < data[0] && di < sizeof(_serial)-1; si += 2) {
                if ((data[si] & 0x80) || (data[si + 1])) /* non-ASCII */
                    _serial[di++] = '?';
                else if (data[si] == '\0')
                    break;
                else
                    _serial[di++] = data[si];
            }
            //should already be zero terminated at the correct offset (hopefully)
            //just doing sanity zero termination
            _serial[sizeof(_serial)-1] = 0;

            /* new style UDID: add hyphen between first 8 and following 16 digits */
            if (di == 24) {
                memmove(&_serial[9], &_serial[8], 16);
                _serial[8] = '-';
                _serial[di+1] = '\0';
            }
        }
        
        bos = usb_request_bos_descriptor();
    }
#else
    uint16_t langid = usb_request_langid();
    info("Got lang ID %u for device %d-%d", langid, _bus, _address);
    {
        int rsp = 0;
        char data[sizeof(_serial)*2];
        memset(data, 0, sizeof(data));
        rsp = kis_send_control_packet(LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR, (uint16_t)((LIBUSB_DT_STRING << 8) | desc.iSerialNumber), langid, (unsigned char*)data, sizeof(data));
        retassure(rsp > 0, "Failed to get KIS serial");
        {
            unsigned int di = 0, si = 2;
            for (; si < data[0] && di < sizeof(_serial)-1; si += 2) {
                if ((data[si] & 0x80) || (data[si + 1])) /* non-ASCII */
                    _serial[di++] = '?';
                else if (data[si] == '\0')
                    break;
                else
                    _serial[di++] = data[si];
            }
            //should already be zero terminated at the correct offset (hopefully)
            //just doing sanity zero termination
            _serial[sizeof(_serial)-1] = 0;

            /* new style UDID: add hyphen between first 8 and following 16 digits */
            if (di == 24) {
                memmove(&_serial[9], &_serial[8], 16);
                _serial[8] = '-';
                _serial[di+1] = '\0';
            }
        }

        info("Got serial '%s' for device %d-%d", _serial, _bus, _address);
    }
#endif
    
    {
        /*
            Enable endpoint maybe??
         */
        kis_rsm_usb_write_32_4(0x80000010, 0x42, 1, 1, 0, false);
        kis_wait_for_info_ep4(0x42, 1, 1);
        kis_rsm_usb_write_32_4(0x80000010, 0x42, 1, 0, 0, false);
        kis_wait_for_info_ep4(0x00, 1, 8);
        kis_wait_for_info_ep4(0x42, 1, 0);
    }

    config = usb_request_config_descriptor(0);

    cfg = usb_request_get_configuration();

    {
        {
            for(int j=0; j<config->bNumInterfaces; j++) {
                const struct libusb_interface_descriptor *intf = &config->interface[j].altsetting[0];
                if(intf->bInterfaceClass != INTERFACE_CLASS ||
                   intf->bInterfaceSubClass != INTERFACE_SUBCLASS ||
                   intf->bInterfaceProtocol != INTERFACE_PROTOCOL)
                    continue;
                if(intf->bNumEndpoints != 2) {
                    warning("Endpoint count mismatch for interface %d of device %d-%d", intf->bInterfaceNumber, _bus, _address);
                    continue;
                }
                if((intf->endpoint[0].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_OUT &&
                   (intf->endpoint[1].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_IN) {
                    _kis_interface = intf->bInterfaceNumber;
                    _kis_ep_out = intf->endpoint[0].bEndpointAddress;
                    _kis_ep_in = intf->endpoint[1].bEndpointAddress;
                    debug("Found interface %d with endpoints %02x/%02x for device %d-%d", _kis_interface, _kis_ep_out, _kis_ep_in, _bus, _address);
                    goto found_device;
                } else if((intf->endpoint[1].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_OUT &&
                          (intf->endpoint[0].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_IN) {
                    _kis_interface = intf->bInterfaceNumber;
                    _kis_ep_out = intf->endpoint[1].bEndpointAddress;
                    _kis_ep_in = intf->endpoint[0].bEndpointAddress;
                    warning("Found interface %d with swapped endpoints %02x/%02x for device %d-%d", _kis_interface, _kis_ep_out, _kis_ep_in, _bus, _address);
                    goto found_device;
                } else {
                    warning("Endpoint type mismatch for interface %d of device %d-%d", intf->bInterfaceNumber, _bus, _address);
                }
            }
            reterror("Could not find a suitable USB interface for device %d-%d", _bus, _address);
        found_device:;
        }
    }
    
    usb_request_set_configuration(1);

    
    {
        /*
            Enable endpoint??
         */
        kis_rsm_usb_write_32_4(0x80000010, 0x0040, 0x0101, 0x02000022, 0x200);
        kis_wait_for_info_ep4(0x00, 0x0101, 0x8000);
        kis_wait_for_info_ep4(0x00, 0x0101, 0x8000);
        kis_rsm_usb_write_32_4(0x80000010, 0x0040, 0x8101, 0x02000022, 0x200);
    }

    {
        kis_trigger_receive_interrupt_packet(_kis_ep_in);
        kis_trigger_receive_interrupt_packet(_kis_ep_in);
        kis_trigger_receive_interrupt_packet(_kis_ep_in);
        kis_trigger_receive_interrupt_packet(_kis_ep_in);
        kis_rsm_usb_write_32_0(0x11000000, false); //trigger dummy packet
        kis_wait_for_interrupt_receive_on_ep(_kis_ep_out); //receive dummy packet
    }
    
    debug("Done KIS init, performing mux init now!");
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf), "Finished KIS init for device %d-%d %s",_bus,_address, _serial);
        _mux->desktop_notification("USBDeviceKIS", buf);
    }
    mux_init();

    _parent->del_constructing(_bus, _address);
    _initIdx = 1;
}

#pragma mark public

#pragma mark inheritence provider
void USBDeviceKIS::deconstruct() noexcept{
    _arrived_outrsp.kill();
    _rsm1PktSyncEvent.kill();
    _ep0Packets.kill();
    _infoPktEP1.kill();
    _infoPktEP4.kill();
    if (_initIdx < 1) {
        _parent->del_constructing(_bus, _address);
    }
    USBDevice::deconstruct();
}

void USBDeviceKIS::kis_start_rxloops(){
    // Spin up NUM_RX_LOOPS parallel usb data retrieval loops
    // Old usbmuxd used only 1 rx loop, but that leaves the
    // USB port sleeping most of the time
    {
        
        auto usbdev = _selfref.lock();
        int didStartRxLoops = 0;
        for (int rx_loops = 0; rx_loops < NUM_RX_LOOPS; rx_loops++) {
            try {
                usb_start_rx_loop(usbdev, _ep_in, rx_callback, new USBDevice_receiver(usbdev.get()));
                didStartRxLoops++;
            } catch (tihmstar::exception &e) {
                warning("Failed to start RX loop number %d", NUM_RX_LOOPS - rx_loops);
            }
        }
        
        //Ensure we have at least 1 RX loop going
        retassure(didStartRxLoops, "Failed to start any RX loop for device %d-%d", _bus, _address);
        if (didStartRxLoops != NUM_RX_LOOPS) {
            warning("Failed to start all %d RX loops. Going on with %d loops. This may have negative impact on device read speed.", NUM_RX_LOOPS, didStartRxLoops);
        } else {
            debug("All %d RX loops started successfully on EP 0x%x", NUM_RX_LOOPS,_ep_in);
        }
    }
}

void USBDeviceKIS::initDevice(){
    struct libusb_config_descriptor *config = NULL;
//    std::shared_ptr<USBDevice> *transferdevref = nullptr;
//    unsigned char *transfer_buffer = NULL;
//    struct libusb_transfer *transfer = NULL;
    cleanup([&]{
//        safeFreeCustom(transfer, libusb_free_transfer);
//        safeFree(transfer_buffer);
//        safeDelete(transferdevref);
        safeFreeCustom(config, libusb_free_config_descriptor);
    });
    int err = 0;
    int current_config = 0;

    retassure(!(err = libusb_get_configuration(_usbdev, &current_config)), "Could not get configuration for device %d-%d: %d", _bus, _address, err);

    if (current_config != _devdesc.bNumConfigurations) {
        if((err = libusb_get_active_config_descriptor(libusb_get_device(_usbdev), &config)) != 0) {
            debug("Could not get old configuration descriptor for device %d-%d: %d", _bus, _address, err);
        } else {
            for(int j=0; j<config->bNumInterfaces; j++) {
                const struct libusb_interface_descriptor *intf = &config->interface[j].altsetting[0];
                if((err = libusb_kernel_driver_active(_usbdev, intf->bInterfaceNumber)) < 0) {
                    debug("Could not check kernel ownership of interface %d for device %d-%d: %d", intf->bInterfaceNumber, _bus, _address, err);
                    continue;
                }
                if(err == 1) {
                    debug("Detaching kernel driver for device %d-%d, interface %d", _bus, _address, intf->bInterfaceNumber);
                    if((err = libusb_detach_kernel_driver(_usbdev, intf->bInterfaceNumber)) < 0) {
                        warning("Could not detach kernel driver (%d), configuration change will probably fail!", err);
                        continue;
                    }
                }
            }
        }
        debug("Setting configuration for device %d-%d, from %d to %d", _bus, _address, current_config, _devdesc.bNumConfigurations);
        retcustomassure(MUXException_device_disconnected,!(err = libusb_set_configuration(_usbdev, _devdesc.bNumConfigurations)), "Could not set configuration %d for device %d-%d: 0x%08x", _devdesc.bNumConfigurations, _bus, _address, err);
    }
    retassure(!(err = libusb_get_active_config_descriptor(libusb_get_device(_usbdev), &config)), "Could not get configuration descriptor for device %d-%d: %d", _bus, _address, err);
    

    
    {
        retassure(config->bNumInterfaces == 1, "unexpected number of interfaces for KIS!");
        const struct libusb_interface_descriptor *intf = &config->interface[0].altsetting[0];
        retassure(intf->bNumEndpoints >= 3, "unexpected number of endpoints for KIS intf!");

        struct EPInfo{
            bool hasIn;
            bool hasOut;
        };
        
        std::map<uint8_t, EPInfo> eps;
        
        for (int i=0; i<intf->bNumEndpoints; i++) {
            debug("KIS interface EP %d addr=0x%x",i,intf->endpoint[i].bEndpointAddress);
            uint8_t epAddr = intf->endpoint[i].bEndpointAddress & 0x7f;
            bool EPDirIsOUT = (intf->endpoint[i].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_OUT;
            if (EPDirIsOUT) {
                eps[epAddr].hasOut = true;
            }else{
                eps[epAddr].hasIn = true;
            }
        }
        
        _interface = intf->bInterfaceNumber;
        
        retassure(eps.find(1) != eps.end(), "EP 1 Missing!");
        
        retassure(eps[1].hasOut, "EP 1 doesn't have OUT EP");
        retassure(eps[1].hasIn, "EP 1 doesn't have IN EP");
        _ep_out = 1 | LIBUSB_ENDPOINT_OUT;
        _ep_outrsp = 1 | LIBUSB_ENDPOINT_IN;

        debug("Found KIS interface %d with endpoints %02x/%02x for device %d-%d", _interface, _ep_out, _ep_outrsp, _bus, _address);
    }
    
    retassure(!(err = libusb_claim_interface(_usbdev, _interface)), "Could not claim interface %d for device %d-%d: %d", _interface, _bus, _address, err);

    _serial[0] = 0;
    _speed = 480000000;
    _wMaxPacketSize = libusb_get_max_packet_size(libusb_get_device(_usbdev), _ep_out);

    if (_wMaxPacketSize <= 0) {
        error("Could not determine wMaxPacketSize for device %d-%d, setting to 64", _bus, _address);
        _wMaxPacketSize = 64;
    } else {
        debug("Using wMaxPacketSize=%d for device %d-%d", _wMaxPacketSize, _bus, _address);
    }
    
    switch (libusb_get_device_speed(libusb_get_device(_usbdev))) {
        case LIBUSB_SPEED_LOW:
            _speed = 1500000;
            break;
        case LIBUSB_SPEED_FULL:
            _speed = 12000000;
            break;
        case LIBUSB_SPEED_SUPER:
            _speed = 5000000000;
            break;
        case LIBUSB_SPEED_HIGH:
        case LIBUSB_SPEED_UNKNOWN:
        default:
            _speed = 480000000;
            break;
    }
    
    info("USB Speed is %g MBit/s for device %d-%d", (double)(_speed / 1000000.0), _bus, _address);

    
    {
        auto usbdev = _selfref.lock();
        usb_start_rx_loop(usbdev, _ep_outrsp, rx_ep_outrsp_callback, new USBDevice_receiver(usbdev.get()), USB_TIMEOUT);
    }
    
    {
        char buf[0x100] = {};
        snprintf(buf,sizeof(buf), "Attached KIS device %d-%d, performing stage 1",_bus,_address);
        info("%s",buf);
        _mux->desktop_notification("USBDeviceKIS", buf);
    }

    /*
        We cannot send and receive requests (the way i implemented it) on the caller thread (which is libusb hotplug callback)
     */
    std::shared_ptr<USBDeviceKIS> usbdev = static_pointer_cast<USBDeviceKIS>(_selfref.lock());
    std::thread initThread([](std::shared_ptr<USBDeviceKIS> dev){
        try {
            dev->performInternalDeviceInit();
        } catch (tihmstar::exception &e) {
#ifdef DEBUG
            e.dump();
#endif
            dev->kill();
        }
    },usbdev);
    initThread.detach();
}

int USBDeviceKIS::kis_send_interrupt_packet(uint8_t endpoint, void *data, uint32_t dataSize){
    assert(endpoint == 1); //how do i actually specify endpoints???
    std::unique_lock<std::mutex> ull(_ep1Lck);
        
    uint32_t alignedDataSize = dataSize & ~3;
    if (dataSize & 3) alignedDataSize += 4;
    {
        KIS_req_rsm1_packet *kreq = NULL;
        cleanup([&]{
            safeFree(kreq);
        });
        std::unique_lock<std::mutex> ul(_trasferStateLck);
        uint32_t transferDataSize = dataSize;
        
        if (transferDataSize & 3) transferDataSize = (transferDataSize &~3) + 4;
        
        kreq = (KIS_req_rsm1_packet*)calloc(1, sizeof(KIS_req_rsm1_packet)+dataSize);
        *kreq = {
            .req = {
                .addr = _usbTxAddr,
                .size = static_cast<uint32_t>(dataSize + 4),
            },
            .tag = static_cast<uint32_t>(0x11000000 + dataSize),
        };
        memcpy(kreq+1, data, dataSize);
        
        kreq->req.addr += _trasferState.hostTxEnd<<2;
        /*
            Transfer USB Packet to KIS
         */
        kis_request_data_init(&kreq->req, KIS_PORTAL_RSM, KIS_OPCODE_DATA_WRITE, KIS_RSM_PARAM_TYPE_USB_PACKET, sizeof(*kreq) + dataSize);
        {
            KIS_req_rsm1_packet *kreqSend = kreq; kreq = NULL;
            kis_request(&kreqSend->req.hdr, sizeof(*kreqSend) + dataSize, false);
        }
        _trasferState.hostTxEnd+=(alignedDataSize>>2);

        /*
            Trigger processing of USB packet
         */
        _trasferState.hostTxEnd++;
        kis_config_write32(KIS_PORTAL_RSM, KIS_RSM_PROCESS_PARAM_TYPE_USB_PACKET, _trasferState.hostTx);
    }
    kis_wait_for_interrupt_receive_on_ep(endpoint);

    return dataSize;
}

void USBDeviceKIS::kis_trigger_receive_interrupt_packet(uint8_t endpoint){
    kis_rsm_usb_write_32_4(0x84000010, 0x1000, 0x01 | ((uint16_t)endpoint << 8), 0x8000, 0, false);
}

void USBDeviceKIS::device_data_input(unsigned char *buffer, uint32_t length){
    KIS_reply_hdr *rsp = (KIS_reply_hdr*)buffer;
    debug("[EP2] Got packet with id 0x%04x",rsp->identifier);
    kis_input_packet_process(rsp, length, 0);
}

void USBDeviceKIS::usb_send(void *buf, size_t length){
    kis_send_interrupt_packet(_kis_ep_out, buf, (uint32_t)length);
    free(buf);
}

void USBDeviceKIS::add_rx_loop(){
    auto usbdev = _selfref.lock();
    usb_start_rx_loop(usbdev, _ep_in, rx_callback, new USBDevice_receiver(usbdev.get()));
}

void USBDeviceKIS::del_rx_loop(){
    ++_killExtraRXLoops;
}
