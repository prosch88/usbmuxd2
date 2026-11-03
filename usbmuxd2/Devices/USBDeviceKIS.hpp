//
//  USBDeviceKIS.hpp
//  usbmuxd2
//
//  Created by erd on 27.11.24.
//

#ifndef USBDeviceKIS_hpp
#define USBDeviceKIS_hpp

#include "USBDevice.hpp"
#include "kis.h"

#include <libgeneral/Event.hpp>
#include <libgeneral/TaggedEvent.hpp>
#include <libgeneral/Mem.hpp>

#include <mutex>

class USBDeviceKIS_receiver;
class USBDeviceKIS : public USBDevice {
    uint8_t _ep_outrsp;
    std::mutex _requestLck;
    uint8_t _kis_interface, _kis_ep_in, _kis_ep_out;

    int16_t _initIdx;
    
    uint64_t                    _usbTxAddr;
    uint64_t                    _usbRxAddr;
    KIS_RSM1_info_transferstate _trasferState;
    uint32_t                    _transferDevReceivedFromHost;
    std::mutex _trasferStateLck;

    tihmstar::DeliveryEvent<tihmstar::Mem> _arrived_outrsp;
    
    uint16_t _rsm1PktSync;
    std::mutex _rsm1PktSyncLck;
    tihmstar::TaggedEvent _rsm1PktSyncEvent;
    
    tihmstar::DeliveryEvent<tihmstar::Mem> _ep0Packets;
    std::mutex _ep0Lck;

    tihmstar::DeliveryEvent<tihmstar::Mem> _infoPktEP1;
    tihmstar::DeliveryEvent<tihmstar::Mem> _infoPktEP4;
    std::mutex _cfgWriteLck;
    
    std::mutex _ep1Lck;

    std::atomic<uint32_t> _killOutRSPLoop;


    void kis_start_rxloops();
    
    void kis_request_data_init(KIS_req_data *hdr, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t requestSize);
    void kis_request_cfg_init(KIS_req_cfg *hdr, uint8_t portal, uint8_t opcode, uint16_t param, uint16_t requestSize);

    void kis_process_data_for_portal_0(uint8_t *data, size_t dataSize, uint64_t reqaddr);
    void kis_process_cfg_for_portal_rsm(uint8_t *data, size_t dataSize, uint64_t reqaddr);
    void kis_process_rsm1_packet(uint8_t *data, size_t dataSize, uint16_t identifier);

    void usb_packets_process(uint8_t *data, uint32_t dataSize);
    
    void kis_input_packet_process(KIS_reply_hdr *rsp, size_t rspSize, uint64_t reqaddr);
        
    void kis_request(KIS_req_header *req, size_t reqSize, bool dorealloc = true);
    void kis_request_data(KIS_req_data *req, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t requestSize, bool dorealloc = true);
    void kis_request_cfg(KIS_req_cfg *req, uint8_t portal, uint8_t opcode, uint32_t param, uint32_t requestSize, bool dorealloc = true);
    
    void kis_config_write32(uint8_t portal, uint16_t param, uint32_t value);
    void kis_config_read(uint8_t portal, uint16_t param, uint16_t size);

    void kis_data_write32(uint8_t portal, uint16_t param, uint64_t addr, uint32_t value);

    void kis_wait_for_info_ep1(uint8_t v0, uint32_t v1);
    void kis_wait_for_info_ep4(uint8_t v0, uint32_t v1, uint32_t v2);
    void kis_wait_for_usb_process(uint32_t insize, uint32_t outsize);
    void kis_wait_for_interrupt_receive_on_ep(uint8_t ep);
    
    void kis_rsm_usb_write_32_4(uint32_t tgt, uint32_t v0, uint32_t v1, uint32_t v2, uint32_t v3, bool waitForEP1 = true);
    void kis_rsm_usb_write_32_0(uint32_t tgt, bool wait = true);

    void kis_clear_flag_status();
    void kis_getinfo();
    
    int kis_send_control_packet(uint8_t bm_request_type, uint8_t b_request, uint16_t w_value, uint16_t w_index, unsigned char *data, uint16_t w_length);
    
    usb_device_descriptor_t usb_request_descriptor();
    uint16_t usb_request_langid();
    libusb_bos_descriptor *usb_request_bos_descriptor();
    libusb_config_descriptor *usb_request_config_descriptor(uint8_t config_index);
    uint8_t usb_request_get_configuration();
    void usb_request_set_configuration(uint8_t cfg);

    void performInternalDeviceInit();
    
    int kis_send_interrupt_packet(uint8_t endpoint, void *data, uint32_t dataSize);
    void kis_trigger_receive_interrupt_packet(uint8_t endpoint);
    
    void add_rx_loop();
    void del_rx_loop();
public:
    USBDeviceKIS(Muxer *mux, USBDeviceManager *parent, uint16_t pid);
    virtual ~USBDeviceKIS() override;
    
#pragma mark inheritence provider
    virtual void deconstruct() noexcept override;
    virtual void initDevice() override;
    virtual void device_data_input(unsigned char *buffer, uint32_t length) override;
    virtual void usb_send(void *buf, size_t length) override;

#pragma mark friends
    friend USBDeviceManager;
    friend void rx_ep_outrsp_callback(struct libusb_transfer *transfer) noexcept;
};

#endif /* USBDeviceKIS_hpp */
