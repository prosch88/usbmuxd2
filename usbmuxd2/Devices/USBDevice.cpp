//
//  USBDevice.cpp
//  usbmuxd2
//
//  Created by tihmstar on 20.07.23.
//
#include <libgeneral/macros.h>

#include "USBDevice.hpp"
#include "../Manager/USBDeviceManager.hpp"
#include "../MUXException.hpp"
#include "TCP.hpp"

#include <mutex>

#include <string.h>

#define ENV_DEVICE_MODE "USBMUXD_DEFAULT_DEVICE_MODE"
#define APPLE_VEND_SPECIFIC_GET_MODE 0x45
#define APPLE_VEND_SPECIFIC_SET_MODE 0x52

#pragma mark libusb_callback definitions
void usb_get_vendormode_callback(struct libusb_transfer *transfer) noexcept;
void usb_set_vendormode_callback(struct libusb_transfer *transfer) noexcept;
void usb_get_langid_callback(struct libusb_transfer *transfer) noexcept;
void usb_get_serial_callback(struct libusb_transfer *transfer) noexcept;
void rx_callback(struct libusb_transfer *xfer) noexcept;

#pragma mark libusb_callback implementations
void tx_callback(struct libusb_transfer *xfer) noexcept{
    std::shared_ptr<USBDevice> dev = *(std::shared_ptr<USBDevice> *)xfer->user_data;

    if(xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        switch(xfer->status) {
            case LIBUSB_TRANSFER_COMPLETED: //shut up compiler
            case LIBUSB_TRANSFER_ERROR:
                // funny, this happens when we disconnect the device while waiting for a transfer, sometimes
                info("Device %d-%d TX aborted due to error or disconnect", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_TIMED_OUT:
                error("TX transfer timed out for device %d-%d", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_CANCELLED:
                debug("Device %d-%d TX transfer cancelled", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_STALL:
                error("TX transfer stalled for device %d-%d", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_NO_DEVICE:
                // other times, this happens, and also even when we abort the transfer after device removal
                info("Device %d-%d TX aborted due to disconnect", dev->_bus, dev->_address);
                break;
            case LIBUSB_TRANSFER_OVERFLOW:
                error("TX transfer overflow for device %d-%d", dev->_bus, dev->_address);
                break;
                // and nothing happens (this never gets called) if the device is freed after a disconnect! (bad)
            default:
                // this should never be reached.
                break;
        }
        dev->kill();
    }

    //remove transfer
    {
        guardWrite(dev->_tx_xfers_Guard);
        dev->_tx_xfers.erase(xfer);
    }

    safeFree(xfer->buffer);
    {
        std::shared_ptr<USBDevice> *userarg = (std::shared_ptr<USBDevice> *)xfer->user_data;xfer->user_data = NULL;
        safeDelete(userarg);
    }
    libusb_free_transfer(xfer);
}

void usb_get_vendormode_callback(struct libusb_transfer *transfer) noexcept{
    std::shared_ptr<USBDevice> usbdev = *(std::shared_ptr<USBDevice> *)transfer->user_data;
    const unsigned char *data = NULL;
    char* desired_mode_char = NULL;
    int desired_mode = 0;
    int guessed_mode = 0;
    int ret = 0;

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED || transfer->actual_length != 4) {
        goto skip_switch;
    }
    
    data = libusb_control_transfer_get_data(transfer); //error-free function
    desired_mode_char = getenv(ENV_DEVICE_MODE);
    desired_mode = desired_mode_char ? atoi(desired_mode_char) : 3;
    guessed_mode = usbdev->guess_mode();
    
    if(desired_mode >= 1 && desired_mode <= 5 &&
       guessed_mode > 0 && // do not switch mode if guess failed
       guessed_mode != desired_mode) {
        info("Switching device %i-%i mode to %i", usbdev->_bus, usbdev->_address, desired_mode);
        
        
        libusb_fill_control_setup(transfer->buffer, (uint8_t)LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_DEVICE, APPLE_VEND_SPECIFIC_SET_MODE, 0, desired_mode, 1);
        libusb_fill_control_transfer(transfer, usbdev->_usbdev, transfer->buffer, usb_set_vendormode_callback, transfer->user_data, 1000);
        if ((ret = libusb_submit_transfer(transfer)) < 0) {
            error("Could not request transfer for device %d-%d (%d)", usbdev->_bus, usbdev->_address, ret);
            goto skip_switch;
        }
        return;
    }
skip_switch:
    debug("Skipping mode switch!");
    transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
    {
        std::shared_ptr<USBDevice> *floating_usbdev = (std::shared_ptr<USBDevice> *)transfer->user_data;transfer->user_data = NULL;
        delete floating_usbdev; floating_usbdev = NULL;
    }
    safeFreeCustom(transfer, libusb_free_transfer);
    try {
        usbdev->configureUSB();
    } catch (tihmstar::exception &e) {
        error("[usb_get_vendormode_callback] Failed with error=%d (%s)",e.code(),e.what());
        e.dump();

        usbdev->_parent->del_constructing(usbdev->_bus, usbdev->_address);
        usbdev->kill();
    }
}

void usb_set_vendormode_callback(struct libusb_transfer *transfer) noexcept{
    std::shared_ptr<USBDevice> usbdev = *(std::shared_ptr<USBDevice> *)transfer->user_data;
    transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
    {
        std::shared_ptr<USBDevice> *floating_usbdev = (std::shared_ptr<USBDevice> *)transfer->user_data;transfer->user_data = NULL;
        delete floating_usbdev; floating_usbdev = NULL;
    }
    safeFreeCustom(transfer, libusb_free_transfer);
    try {
        usbdev->configureUSB();
    } catch (tihmstar::exception &e) {
        error("[usb_set_vendormode_callback] Failed with error=%d (%s)",e.code(),e.what());
        e.dump();

        usbdev->_parent->del_constructing(usbdev->_bus, usbdev->_address);
        usbdev->kill();
    }
}

void usb_get_langid_callback(struct libusb_transfer *transfer) noexcept{
    std::shared_ptr<USBDevice> usbdev = *(std::shared_ptr<USBDevice> *)transfer->user_data;
    
    try {
        uint16_t langid = 0;
        const unsigned char *data = NULL;
        int ret = 0;

        retassure(usbdev != nullptr, "USBDevice was dead when usb_get_langid_callback fired");

        transfer->flags |= LIBUSB_TRANSFER_FREE_BUFFER;
        retassure(transfer->status == LIBUSB_TRANSFER_COMPLETED, "Failed to request lang ID for device %d-%d (%i)", usbdev->_bus, usbdev->_address, transfer->status);

        data = libusb_control_transfer_get_data(transfer); //error-free function
        langid = (uint16_t)(data[2] | (data[3] << 8));
        info("Got lang ID %u for device %d-%d", langid, usbdev->_bus, usbdev->_address);

        /* re-use the same transfer */
        libusb_fill_control_setup(transfer->buffer, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR,
                                  (uint16_t)((LIBUSB_DT_STRING << 8) | usbdev->_devdesc.iSerialNumber),
                                  langid, 1024 + LIBUSB_CONTROL_SETUP_SIZE);

        libusb_fill_control_transfer(transfer, usbdev->_usbdev, transfer->buffer, usb_get_serial_callback, transfer->user_data, 1000);

        retassure((ret = libusb_submit_transfer(transfer)) >= 0, "Could not request transfer for device %d-%d (%d)", usbdev->_bus, usbdev->_address, ret);
    } catch (tihmstar::exception &e) {
        error("[usb_get_langid_callback] Failed with error=%d (%s)",e.code(),e.what());
        e.dump();

        usbdev->_parent->del_constructing(usbdev->_bus, usbdev->_address);
        //at this point we are the only owner of usbdev (the floating shared_ptr to it)
        //if anything goes wrong, this is the last chance to delete the object without leaking it
        {
            std::shared_ptr<USBDevice> *floating_usbdev = (std::shared_ptr<USBDevice> *)transfer->user_data;transfer->user_data = NULL;
            delete floating_usbdev; floating_usbdev = NULL;
        }

        safeFreeCustom(transfer, libusb_free_transfer);
    }
}

void usb_get_serial_callback(struct libusb_transfer *transfer) noexcept{
    std::shared_ptr<USBDevice> usbdev = *(std::shared_ptr<USBDevice> *)transfer->user_data;
    try {
        const unsigned char *data = NULL;

        retassure(usbdev != nullptr, "USBDevice was dead when usb_get_serial_callback fired");

        retassure(transfer->status == LIBUSB_TRANSFER_COMPLETED, "Failed to request serial for device %d-%d (%i)", usbdev->_bus, usbdev->_address, transfer->status);

        /* De-unicode, taken from libusb */
        assure(data = libusb_control_transfer_get_data(transfer));

        {
            unsigned int di = 0, si = 2;
            for (; si < data[0] && di < sizeof(usbdev->_serial)-1; si += 2) {
                if ((data[si] & 0x80) || (data[si + 1])) /* non-ASCII */
                    usbdev->_serial[di++] = '?';
                else if (data[si] == '\0')
                    break;
                else
                    usbdev->_serial[di++] = data[si];
            }
            //should already be zero terminated at the correct offset (hopefully)
            //just doing sanity zero termination
            usbdev->_serial[sizeof(usbdev->_serial)-1] = 0;

            /* new style UDID: add hyphen between first 8 and following 16 digits */
            if (di == 24) {
                memmove(&usbdev->_serial[9], &usbdev->_serial[8], 16);
                usbdev->_serial[8] = '-';
                usbdev->_serial[di+1] = '\0';
            }
        }

        info("Got serial '%s' for device %d-%d", usbdev->_serial, usbdev->_bus, usbdev->_address);

        // Spin up NUM_RX_LOOPS parallel usb data retrieval loops
        // Old usbmuxds used only 1 rx loop, but that leaves the
        // USB port sleeping most of the time
        {
            int rx_loops = 0;
            for (; rx_loops < NUM_RX_LOOPS; rx_loops++) {
                try {
                    USBDevice::usb_start_rx_loop(usbdev, usbdev->_ep_in, rx_callback, new USBDevice_receiver(usbdev.get()));
                } catch (tihmstar::exception &e) {
                    warning("Failed to start RX loop number %d", NUM_RX_LOOPS - rx_loops);
                }
            }
            // Ensure we have at least 1 RX loop going
            retassure(rx_loops, "Failed to start any RX loop for device %d-%d", usbdev->_bus, usbdev->_address);
            if (rx_loops != NUM_RX_LOOPS) {
                warning("Failed to start all %d RX loops. Going on with %d loops. This may have negative impact on device read speed.", NUM_RX_LOOPS, rx_loops);
            } else {
                debug("All %d RX loops started successfully", NUM_RX_LOOPS);
            }
        }

        usbdev->mux_init();

    } catch (tihmstar::exception &e) {
        error("[usb_get_serial_callback] Failed with error=%d (%s)",e.code(),e.what());
        e.dump();
    }
    usbdev->_parent->del_constructing(usbdev->_bus, usbdev->_address); //device is either done or failed at this point
    
    //at this point we are the only owner of usbdev (the floating shared_ptr to it)
    //if anything goes wrong, this is the last chance to delete the object without leaking it
    {
        std::shared_ptr<USBDevice> *floating_usbdev = (std::shared_ptr<USBDevice> *)transfer->user_data;transfer->user_data = NULL;
        delete floating_usbdev; floating_usbdev = NULL;
    }
    safeFreeCustom(transfer, libusb_free_transfer);
}

void rx_callback(struct libusb_transfer *xfer) noexcept{
    std::shared_ptr<USBDevice> dev = *(std::shared_ptr<USBDevice> *)xfer->user_data;
//    debug("RX callback dev %d-%d len %d status %d", dev->_bus, dev->_address, xfer->actual_length, xfer->status);
    if(xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        try {
            dev->_arrived_xfer.post(xfer);
            return;
        } catch (tihmstar::exception &e) {
            debug("Failed to post _arrived_xfer with err:\n%s",e.dumpStr().c_str());
        }
    }
    switch(xfer->status) {
        case LIBUSB_TRANSFER_ERROR:
            // funny, this happens when we disconnect the device while waiting for a transfer, sometimes
            info("Device %d-%d RX aborted due to error or disconnect", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_TIMED_OUT:
            error("RX transfer timed out for device %d-%d", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            debug("Device %d-%d RX transfer cancelled", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_STALL:
            error("RX transfer stalled for device %d-%d", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            // other times, this happens, and also even when we abort the transfer after device removal
            info("Device %d-%d RX aborted due to disconnect", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_OVERFLOW:
            error("RX transfer overflow for device %d-%d", dev->_bus, dev->_address);
            break;
        case LIBUSB_TRANSFER_COMPLETED: //shut up compiler
        default:
            // this should never be reached.
            break;
    }
error:
    //remove transfer
    {
        guardWrite(dev->_rx_xfers_Guard);
        dev->_rx_xfers.erase(xfer);
    }

    debug("freing rx xfer for USBDevice(%s)",dev->_serial);
    safeFree(xfer->buffer);
    {
        std::shared_ptr<USBDevice> *cbargref = (std::shared_ptr<USBDevice> *)xfer->user_data; xfer->user_data = NULL;
        safeDelete(cbargref);
    }
    libusb_free_transfer(xfer);

    dev->kill();
}

#pragma mark USBDevice
USBDevice::USBDevice(Muxer *mux, USBDeviceManager *parent, uint16_t pid)
: Device(mux, MUXCONN_USB), _selfref{}, _parent(parent)
, _pid(pid)
, _bus(0), _address(0)
, _interface(0), _ep_in(-1), _ep_out(-1)
, _devdesc{}
, _wMaxPacketSize(0), _speed(0)
, _state{}, _usbdev(NULL), _nextPort(0)
, _muxdev{}, _usbLck{}
, _rx_xfers{}, _tx_xfers{}
, _killExtraRXLoops{0}
, _usb_mtu(USB_MTU)
{
    retassure(_muxdev.pktbuf = (uint8_t*)malloc(DEV_MRU), "Failed to alloc pktbuf");
    _conReaperThread = std::thread([this]{
        reaper_runloop();
    });

}

USBDevice::~USBDevice(){
    deconstruct();
    debug("deleting device '%s'",_serial);
    _conReaperThread.join();
    
    {
        std::unique_lock<std::mutex> ul(_parent->_childrenLck);
        _parent->_children.erase(this);
        _parent->_childrenEvent.notifyAll();
        _parent = NULL;
    }
    
    safeFree(_muxdev.pktbuf);
    //free resources
    if (_usbdev){
        libusb_release_interface(_usbdev, _interface);
        safeFreeCustom(_usbdev, libusb_close);
    }
    assert(isDeviceReadyForDestruction());
#ifdef XCODE
    debug("USBDevice(%p) destroyed",this);
#endif
}

#pragma mark private
bool USBDevice::isDeviceReadyForDestruction(){
    return _rx_xfers.size() == 0 && _tx_xfers.size() == 0;
}

void USBDevice::reaper_runloop(){
    while (true) {
        uint16_t conport = 0;
        try {
            conport = _reapConnections.wait();
        } catch (...) {
            break;
        }
        {
            guardWrite(_conns_Guard);
            auto cp = _conns.find(conport);
            if (cp != _conns.end()){
                cp->second->deconstruct();
            }
            _conns.erase(conport);
            _conns_close_event.notifyAll();
        }
    }
}

int USBDevice::guess_mode(){
    int res, j;
    int has_valeria = 0, has_cdc_ncm = 0, has_usbmux = 0;
    struct libusb_config_descriptor *config;

    if(_devdesc.bNumConfigurations == 1) {
        // CDC-NCM Direct
        return 5;
    }

    if(_devdesc.bNumConfigurations <= 4) {
        // Assume this is initial mode
        return 1;
    }

    if(_devdesc.bNumConfigurations == 6) {
        // USB Ethernet + CDC-NCM
        return 4;
    }

    if(_devdesc.bNumConfigurations != 5) {
        // No known modes with more then 5 configurations
        return 0;
    }

    if((res = libusb_get_config_descriptor_by_value(libusb_get_device(_usbdev), 5, &config)) != 0) {
        usbmuxd_log(LL_NOTICE, "Could not get configuration 5 descriptor for device %i-%i: %s", _bus, _address, libusb_error_name(res));
        return 0;
    }

    // Require both usbmux and one of the other interfaces to determine this is a valid configuration
    for(j = 0 ; j < config->bNumInterfaces ; j++) {
        const struct libusb_interface_descriptor *intf = &config->interface[j].altsetting[0];
        if(intf->bInterfaceClass == INTERFACE_CLASS &&
           intf->bInterfaceSubClass == 42 &&
           intf->bInterfaceProtocol == 255) {
            has_valeria = 1;
        }
        // https://github.com/torvalds/linux/blob/72a85e2b0a1e1e6fb4ee51ae902730212b2de25c/include/uapi/linux/usb/cdc.h#L22
        // 2 for Communication class, 0xd for CDC NCM subclass
        if(intf->bInterfaceClass == 2 &&
           intf->bInterfaceSubClass == 0xd) {
            has_cdc_ncm = 1;
        }
        if(intf->bInterfaceClass == INTERFACE_CLASS &&
           intf->bInterfaceSubClass == INTERFACE_SUBCLASS &&
           intf->bInterfaceProtocol == INTERFACE_PROTOCOL) {
            has_usbmux = 1;
        }
    }

    libusb_free_config_descriptor(config);

    if(has_valeria && has_usbmux) {
        debug("Found Valeria and Apple USB Multiplexor in device %i-%i configuration 5", _bus, _address);
        return 2;
    }

    if(has_cdc_ncm && has_usbmux) {
        debug("Found CDC-NCM and Apple USB Multiplexor in device %i-%i configuration 5", _bus, _address);
        return 3;
    }

    return 0;
}

void USBDevice::configureUSB(){
    /**
     * From libusb:
     *     Asking for the zero'th index is special - it returns a string
     *     descriptor that contains all the language IDs supported by the
     *     device.
     **/
    struct libusb_config_descriptor *config = NULL;
    std::shared_ptr<USBDevice> *transferdevref = nullptr;
    unsigned char *transfer_buffer = NULL;
    struct libusb_transfer *transfer = NULL;
    cleanup([&]{
        safeFreeCustom(transfer, libusb_free_transfer);
        safeFree(transfer_buffer);
        safeDelete(transferdevref);
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
                _interface = intf->bInterfaceNumber;
                _ep_out = intf->endpoint[0].bEndpointAddress;
                _ep_in = intf->endpoint[1].bEndpointAddress;
                debug("Found interface %d with endpoints %02x/%02x for device %d-%d", _interface, _ep_out, _ep_in, _bus, _address);
                goto found_device;
            } else if((intf->endpoint[1].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_OUT &&
                      (intf->endpoint[0].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_IN) {
                _interface = intf->bInterfaceNumber;
                _ep_out = intf->endpoint[1].bEndpointAddress;
                _ep_in = intf->endpoint[0].bEndpointAddress;
                warning("Found interface %d with swapped endpoints %02x/%02x for device %d-%d", _interface, _ep_out, _ep_in, _bus, _address);
                goto found_device;
            } else {
                warning("Endpoint type mismatch for interface %d of device %d-%d", intf->bInterfaceNumber, _bus, _address);
            }
        }
        reterror("Could not find a suitable USB interface for device %d-%d", _bus, _address);
    found_device:;
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

    /*
        transfer gets own reference, which it needs to free once done
     */
    transferdevref = new std::shared_ptr<USBDevice>(_selfref.lock());

    retassure(transfer = libusb_alloc_transfer(0), "Failed to allocate transfer for device %d-%d", _bus, _address);

    retassure(transfer_buffer = (unsigned char *)malloc(1024 + LIBUSB_CONTROL_SETUP_SIZE + 8), "Failed to allocate transfer buffer for device %d-%d: %d", _bus, _address, err);
    memset(transfer_buffer, '\0', 1024 + LIBUSB_CONTROL_SETUP_SIZE + 8);

    libusb_fill_control_setup(transfer_buffer, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR, LIBUSB_DT_STRING << 8, 0, 1024 + LIBUSB_CONTROL_SETUP_SIZE);

    libusb_fill_control_transfer(transfer, _usbdev, transfer_buffer, usb_get_langid_callback, transferdevref, 1000);

    retassure(!(err = libusb_submit_transfer(transfer)), "Could not request transfer for device %d-%d (%d)", _bus, _address, err);

    transferdevref = nullptr; //don't cleanup
    transfer = NULL; //transfer in process, needs to be freed by callback
    transfer_buffer = NULL; //transfer in process, needs to be freed by callback
}


#pragma mark protected
void USBDevice::addReceiver(tihmstar::Manager *receiver){
    _receivers.insert(receiver);
}

#pragma mark inheritence provider
void USBDevice::kill() noexcept{
    debug("[Killing] USBDevice %s",_serial);
    {
        std::shared_ptr<USBDevice> selfref = _selfref.lock();
        if (selfref)
            _parent->_reapDevices.post(selfref);
    }
}

void USBDevice::deconstruct() noexcept{
    debug("[Deconstructing] USBDevice '%s'",_serial);
    {
        std::shared_ptr<USBDevice> selfref = _selfref.lock();
        if (selfref)
            _mux->delete_device(selfref);
    }
    _arrived_xfer.kill();
    _data_in_event.kill();

    //cancel all rx transfers
    {
        guardRead(_rx_xfers_Guard);
        for (auto xfer : _rx_xfers) {
            int rsp = libusb_cancel_transfer(xfer);
#ifdef XCODE
            debug("cancelling _rx_xfers(%p)=%d",xfer,rsp);
#endif
        }
    }

    //cancel all tx transfers
    {
        guardRead(_tx_xfers_Guard);
        for (auto xfer : _tx_xfers) {
#ifdef XCODE
            debug("cancelling _tx_xfers(%p)",xfer);
#endif
            libusb_cancel_transfer(xfer);
        }
    }
    
    //cancel all TCP connections
    {
        {
            guardRead(_conns_Guard);
            for (auto c : _conns) {
                _reapConnections.post(c.first);
            }
        }
        while (true) {
            uint64_t wevent = 0;
            {
                guardRead(_conns_Guard);
                if (_conns.size() == 0) break;
                wevent = _conns_close_event.getNextEvent();
            }
            _conns_close_event.waitForEvent(wevent);
        }
    }
    while (_receivers.size()) {
        auto r = *_receivers.begin();
        _receivers.erase(r);
        delete r;
    }
    
    assert(_receivers.size() == 0);
    
    _reapConnections.kill();
}

void USBDevice::start_connect(uint16_t dport, std::shared_ptr<Client> cli){
    std::shared_ptr<TCP> conn;
    assure(_conns.size() < 0xfff0); //we can't handle more connections than we have ports!

    {
        guardWrite(_conns_Guard);
        bool didIterAllPorts = false;
        while (_conns.find(_nextPort) != _conns.end() || _nextPort == 0){
            if (_nextPort == 0) {
                retassure(!didIterAllPorts, "Failed to find available port!");
                didIterAllPorts = true;
            }
            _nextPort++;
        }
        try {
            conn = std::make_shared<TCP>(_nextPort,dport,_selfref.lock(),cli, _usb_mtu);
        } catch (...) {
            throw;
        }
        _conns[_nextPort] = conn;
    }

    try {
        conn->connect();
    } catch (tihmstar::exception &e) {
        error("failed to connect client dport=%d error=%s code=%d",dport,e.what(),e.code());
        throw;
    }
}

void USBDevice::closeConnection(uint16_t sport){
    _reapConnections.post(sport);
}


#pragma mark members
uint32_t USBDevice::usb_location(){
    return (_bus << 16) | _address;
}

uint64_t USBDevice::getSpeed(){
    return _speed;
}

uint16_t USBDevice::getPid(){
    return _pid;
}

void USBDevice::initDevice(){
    /**
     * From libusb:
     *     Asking for the zero'th index is special - it returns a string
     *     descriptor that contains all the language IDs supported by the
     *     device.
     **/
    std::shared_ptr<USBDevice> *transferdevref = nullptr;
    unsigned char *transfer_buffer = NULL;
    struct libusb_transfer *transfer = NULL;
    cleanup([&]{
        safeFreeCustom(transfer, libusb_free_transfer);
        safeFree(transfer_buffer);
        safeDelete(transferdevref);
    });
    int err = 0;

    /*
        transfer gets own reference, which it needs to free once done
     */
    transferdevref = new std::shared_ptr<USBDevice>(_selfref.lock());

    retassure(transfer = libusb_alloc_transfer(0), "Failed to allocate transfer for device %d-%d", _bus, _address);

    retassure(transfer_buffer = (unsigned char *)malloc(1024 + LIBUSB_CONTROL_SETUP_SIZE + 8), "Failed to allocate transfer buffer for device %d-%d: %d", _bus, _address, err);
    memset(transfer_buffer, '\0', 1024 + LIBUSB_CONTROL_SETUP_SIZE + 8);

    libusb_fill_control_setup(transfer_buffer, (uint8_t)LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_DEVICE, APPLE_VEND_SPECIFIC_GET_MODE, 0, 0, 4);

    libusb_fill_control_transfer(transfer, _usbdev, transfer_buffer, usb_get_vendormode_callback, transferdevref, 1000);

    retassure(!(err = libusb_submit_transfer(transfer)), "Could not request transfer for device %d-%d (%d)", _bus, _address, err);

    transferdevref = nullptr; //don't cleanup
    transfer = NULL; //transfer in process, needs to be freed by callback
    transfer_buffer = NULL; //transfer in process, needs to be freed by callback
}

void USBDevice::mux_init(){
    mux_version_header vh = {};
    
    vh.major = htonl(2);
    vh.minor = htonl(0);
    vh.padding = 0;
    
    send_packet(MUX_PROTO_VERSION, &vh, sizeof(vh));
}

void USBDevice::send_packet(enum mux_protocol proto, const void *data, size_t length, tcphdr *header){
    /*
     buf allocated and guaranteed transfered to usb_send without failing in between.
     usb_send will always make sure buf is freed, even in case of failure.
     Don't free buf in this function in any case!
     */
    unsigned char *buf = NULL; //unchecked
    size_t buflen = 0;
    mux_header *mhdr = NULL; //unchecked
    int mux_header_size = 0;

    mux_header_size = ((_muxdev.version < 2) ? sizeof(struct mux_header_v1) : sizeof(struct mux_header_v2));

    buflen = mux_header_size + length;
    if (header) {
        buflen += sizeof(tcphdr);
    }

    assure(buflen>length); //sanity check

    retassure(buflen <= USB_MTU, "Tried to send packet larger than USB MTU (hdr %zu data %zu total %zu) to device %s", buflen, length, buflen, _serial);

    buf = (unsigned char *)malloc(buflen); //unchecked
    mhdr = (mux_header *)buf;
    mhdr->protocol = htonl(proto);
    mhdr->length = htonl(buflen);

    {
        std::unique_lock<std::mutex> ul(_usbLck);
        if (_muxdev.version >= 2) {
            mhdr->v2.magic = htonl(0xfeedface);
            if (proto == MUX_PROTO_SETUP) {
                _muxdev.tx_seq = 0;
                _muxdev.rx_seq = 0xffff;
            }
            mhdr->v2.tx_seq = htons(_muxdev.tx_seq);
            mhdr->v2.rx_seq = htons(_muxdev.rx_seq);
//            debug("----- MUX UPDATE SEND _muxdev.tx_seq=%d _muxdev.rx_seq=%d",_muxdev.tx_seq,_muxdev.rx_seq);
            _muxdev.tx_seq++;
        }
        if (header) {
            memcpy(buf + mux_header_size, header, sizeof(tcphdr));
            memcpy(buf + mux_header_size + sizeof(tcphdr), data, length);
        }else{
            memcpy(buf + mux_header_size, data, length);
        }

        try {
            unsigned char *sendbuf = buf; buf = NULL; //freed by usb_send in any case
            usb_send(sendbuf, buflen);
        } catch (tihmstar::exception &e) {
#ifdef XCODE
            debug("failed to send packet to usbdevice(%p) error=%s code=%d",this,e.what(),e.code());
#endif
            kill();
            throw;
        }
    }
}

/*
 always frees buf
 */
void USBDevice::usb_send(void *buf, size_t length){
    struct libusb_transfer *xfer = NULL;
    std::shared_ptr<USBDevice> *txcbargref = nullptr;
    cleanup([&]{
        safeDelete(txcbargref);
        safeFree(buf);
        if (xfer) {
            {
                guardWrite(_tx_xfers_Guard);
                _tx_xfers.erase(xfer);
            }
            safeFree(xfer->buffer);
            {
                std::shared_ptr<USBDevice> *userdata = (std::shared_ptr<USBDevice> *)xfer->user_data; xfer->user_data = NULL;
                safeDelete(userdata);
            }
            libusb_free_transfer(xfer);
        }
    });
    int ret = 0;

    assure(length<=INT_MAX); //sanity check
    assure(xfer = libusb_alloc_transfer(0));
    xfer->user_data = NULL;

    txcbargref = new std::shared_ptr<USBDevice>(_selfref.lock());
    libusb_fill_bulk_transfer(xfer, _usbdev, _ep_out, (unsigned char *)buf, (int)length, tx_callback, txcbargref, 0);
    buf = NULL;
    txcbargref = nullptr;

    {
        guardWrite(_tx_xfers_Guard);
        _tx_xfers.insert(xfer);
    }
    retassure((ret = libusb_submit_transfer(xfer)) >=0, "Failed to submit TX transfer len %zu to device %d-%d: %d", length, _bus, _address, ret);
    xfer = NULL;
    if (length % _wMaxPacketSize == 0 && length >= _wMaxPacketSize) {
        debug("Send ZLP");
        // Send Zero Length Packet
        assure(buf = malloc(1));
        assure(xfer = libusb_alloc_transfer(0));
        xfer->user_data = NULL;

        txcbargref = new std::shared_ptr<USBDevice>(_selfref.lock());
        libusb_fill_bulk_transfer(xfer, _usbdev, _ep_out, (unsigned char *)buf, 0, tx_callback, txcbargref, 0);
        buf = NULL;
        txcbargref = nullptr;

        {
            guardWrite(_tx_xfers_Guard);
            _tx_xfers.insert(xfer);
        }
        retassure((ret = libusb_submit_transfer(xfer)) >=0, "Failed to submit TX ZLP transfer to device %d-%d: %d", _bus, _address, ret);
        xfer = NULL;
    }
}

void USBDevice::device_data_input(unsigned char *buffer, uint32_t length){
    mux_header *mhdr = NULL;
    unsigned char *payload = NULL;
    uint32_t payload_length = 0;
    int mux_header_size = 0;

    if(!length)
        return;

    // sanity check (should never happen with current USB implementation)
    retassure((length <= USB_MRU) && (length <= DEV_MRU),"Too much data received from USB (%u), file a bug", length);

//    debug("Mux data input for device %s: len %u", _serial, length);
    mhdr = (mux_header *)buffer;

    {
        std::unique_lock<std::mutex> ul(_usbLck);
        mux_header_size = ((_muxdev.version < 2) ? sizeof(struct mux_header_v1) : sizeof(struct mux_header_v2));
#ifdef XCODE
        assert(ntohl(mhdr->length) <= USB_MRU);
#endif
        retassure(ntohl(mhdr->length) == length, "Incoming packet size mismatch (dev %s, expected %d, got %u)", _serial, ntohl(mhdr->length), length);
        if (_muxdev.version >= 2) {
            cleanup([&]{
                _data_in_event.notifyAll();
            });
            uint16_t txseq = ntohs(mhdr->v2.tx_seq);
//            debug("----- MUX txseq=%d -- _muxdev.tx_seq=%d _muxdev.rx_seq=%d",txseq,_muxdev.tx_seq,_muxdev.rx_seq);
            if ((uint16_t)(_muxdev.rx_seq+1) != txseq) {
                while ((uint16_t)(_muxdev.rx_seq+1) < txseq || (uint16_t)(_muxdev.rx_seq+1+_rx_xfers.size()) < txseq + _rx_xfers.size()) {
                    uint64_t wevent = _data_in_event.getNextEvent();
                    ul.unlock();
                    _data_in_event.waitForEvent(wevent);
                    ul.lock();
                }
            }
            if ((uint16_t)(_muxdev.rx_seq+1) != txseq){
                debug("Discarding duplicated MUX packet txseq=%d -- _muxdev.tx_seq=%d _muxdev.rx_seq=%d",txseq,_muxdev.tx_seq,_muxdev.rx_seq);
                return;
            }
            _muxdev.rx_seq = txseq;
        }
        
        // handle broken up transfers
        if(_muxdev.pktlen) {
            if (_muxdev.version < 2){
                error("Mux v1 doesn't support broken up transfers!");
                reterror("Mux v1 doesn't support broken up transfers!");
            }

            //check rx/tx
            retassure((length + _muxdev.pktlen) <= DEV_MRU, "Incoming split packet is too large (%u so far), dropping!", length + _muxdev.pktlen);

            memcpy(_muxdev.pktbuf + _muxdev.pktlen, buffer, length);

            if((length < USB_MRU) || (ntohl(mhdr->length) == (length + _muxdev.pktlen))) {
                buffer = _muxdev.pktbuf;
                length += _muxdev.pktlen;
                _muxdev.pktlen = 0;
                debug("Gathered mux data from buffer (total size: %u)", length);
            } else {
                _muxdev.pktlen += (uint32_t)length;
                debug("Appended mux data to buffer (total size: %u)", _muxdev.pktlen);
                return;
            }
        }else{
            if((length == USB_MRU) && (length < ntohl(mhdr->length))) {
                memcpy(_muxdev.pktbuf, buffer, length);
                _muxdev.pktlen = (uint32_t)length;
                debug("Copied mux data to buffer (size: %u)", _muxdev.pktlen);
                return;
            }
        }
    }

    switch(ntohl(mhdr->protocol)) {
        case MUX_PROTO_VERSION:
            retassure(length >= (mux_header_size + sizeof(struct mux_version_header)), "Incoming version packet is too small (%u)", length);
        {
            mux_version_header vh = *(struct mux_version_header *)((char*)mhdr+mux_header_size);
            vh.major = ntohl(vh.major);
            vh.minor = ntohl(vh.minor);
            device_version_input(&vh);
        }
            break;
        case MUX_PROTO_CONTROL:
            payload = (unsigned char *)(mhdr+1);
            payload_length = length - mux_header_size;
            device_control_input(payload, payload_length);
            break;
        case MUX_PROTO_TCP:
            retassure(length >= (mux_header_size + sizeof(struct tcphdr)), "Incoming TCP packet is too small (%u)", length);
        {
            tcphdr *tcp_header = reinterpret_cast<tcphdr*>((uint8_t*)mhdr+mux_header_size);
            payload = reinterpret_cast<std::uint8_t*>(tcp_header+1);
            payload_length = length - sizeof(tcphdr) - mux_header_size;
            uint16_t dport = htons(tcp_header->th_dport);
            std::shared_ptr<TCP> connect = nullptr;
            {
                guardRead(_conns_Guard);
                auto conn = _conns.find(dport);
                if (conn != _conns.end()){
                    connect = conn->second;
                }
            }
#ifdef DEBUG
            if (tcp_header->th_flags & TH_RST && payload_length > 1) {
                /*
                    Debug stuff here
                 */
                debug("[DEV TCP] sport=%u dport=%u log '%.*s'",htons(tcp_header->th_sport), htons(tcp_header->th_dport),payload_length-1,(char*)payload+1);
            }
#endif
            if (!connect){
                try {
                    TCP::send_RST(this, tcp_header);
                } catch (...) {
                    //
                }
                error("no connection found with snum=%d",dport);
            }else{
               try {
                   connect->handle_input(tcp_header, payload, payload_length);
               } catch (tihmstar::exception &e) {
                   error("failed to handle input on snum=%d device(%d)=%s with error=%d (%s)",dport,_id,_serial,e.code(),e.what());
                   throw;
               }
            }
            return;
        }
            break;
        default:
            error("Incoming packet for device %s has unknown protocol 0x%x)", _serial, ntohl(mhdr->protocol));
            break;
    }
}

void USBDevice::device_version_input(struct mux_version_header *vh){
    retassure(_state == MUXDEV_INIT, "Version packet from already initialized device %s", _serial);

    if(vh->major != 2 && vh->major != 1) {
        error("Device %s has unknown version %d.%d", _serial, vh->major, vh->minor);
        kill();
        return;
    }
    _muxdev.version = vh->major;

    if (_muxdev.version >= 2) {
        send_packet(MUX_PROTO_SETUP, "\x07", 1);
    }

    info("Connected to v%d.%d device %s on location 0x%x", _muxdev.version, vh->minor, _serial, usb_location());
    _state = MUXDEV_ACTIVE;

    //this device is now set up and ready to be used by muxer
    _mux->add_device(_selfref.lock());
}

void USBDevice::device_control_input(unsigned char *payload, uint32_t payload_length){
    char* buf = NULL;
    cleanup([&]{
        safeFree(buf);
    });
    
    if (payload_length > 0) {
        const char *type = "UNK";
        switch (payload[0]) {
            case 3:
                type = "ERROR";
                break;
            case 4:
                type = "2WARNING2";
                break;
            case 5:
                type = "WARNING";
                break;
            case 7:
                type = "INFO";
                break;
            default:
                break;
        }
        if (payload_length > 1) {
            asprintf(&buf, "%s: %.*s",type,payload_length-1,payload+1);
            printf("%s: %s\n", __func__, buf);
        } else {
            warning("%s: got a type %d packet without payload", __func__, (int)payload[0]);
        }
    }
}

#pragma mark static
// Start a read-callback loop for this device
void USBDevice::usb_start_rx_loop(std::shared_ptr<USBDevice> dev, uint8_t ep, libusb_transfer_cb_fn callback, tihmstar::Manager *receiver, int timeout){
    void *buf = NULL;
    struct libusb_transfer *xfer = NULL;
    std::shared_ptr<USBDevice> *devrefarg = nullptr;
    cleanup([&](){ //cleanup only code
        safeDelete(devrefarg);
        safeFree(buf);
        if (xfer) {
            {
                guardWrite(dev->_rx_xfers_Guard);
                dev->_rx_xfers.erase(xfer);
            }
            safeFree(xfer->buffer);
            {
                std::shared_ptr<USBDevice>* ud = (std::shared_ptr<USBDevice>*)xfer->user_data;xfer->user_data = NULL;
                safeDelete(ud);
            }
            libusb_free_transfer(xfer);
        }
        if (receiver) {
            dev->kill();
        }
        safeDelete(receiver);
    });
    int ret = 0;

    assure(buf = malloc(USB_MRU));
    assure(xfer = libusb_alloc_transfer(0));
    xfer->user_data = NULL;

    devrefarg = new std::shared_ptr<USBDevice>{dev};
    libusb_fill_bulk_transfer(xfer, dev->_usbdev, ep, (unsigned char *)buf, USB_MRU, callback, devrefarg, timeout);
    buf = NULL; //owned by xfer now
    devrefarg = nullptr; //owned by xfer now

    {
        guardWrite(dev->_rx_xfers_Guard);
        dev->_rx_xfers.insert(xfer); //transfer ownsership of transfer to device
    }
    retassure(!((ret = libusb_submit_transfer(xfer)),ret),"Failed to submit RX transfer to device %d-%d: %d", dev->_bus, dev->_address, ret);
    xfer = NULL;
    dev->addReceiver(receiver);receiver = NULL;
}
