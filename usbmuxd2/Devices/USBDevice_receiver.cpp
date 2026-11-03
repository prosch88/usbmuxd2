//
//  USBDevice_receiver.cpp
//  usbmuxd2
//
//  Created by tihmstar on 09.09.23.
//
#include <libgeneral/macros.h>

#include "USBDevice_receiver.hpp"
#include "USBDevice.hpp"
#include "../Manager/USBDeviceManager.hpp"

USBDevice_receiver::USBDevice_receiver(USBDevice *parent)
: _parent(parent)
{
    startLoop();
}

USBDevice_receiver::~USBDevice_receiver(){
    stopLoop();
}

bool USBDevice_receiver::loopEvent(){
    bool success = true;
    bool removeTransfer = false;
    USBDevice *dev = static_cast<USBDevice*>(_parent);

    struct libusb_transfer *xfer = dev->_arrived_xfer.wait();
    try {
        dev->device_data_input(xfer->buffer, xfer->actual_length);
        success = true;
    } catch (tihmstar::exception &e) {
        success = false;
        error("failed to device_data_input_kis usbdev=%s:\n%s",_parent->_serial,e.dumpStr().c_str());
    }
    
    if (success) {
        uint32_t needsRxRemoved = dev->_killExtraRXLoops;
        while (!removeTransfer && needsRxRemoved > 0) {
            removeTransfer = dev->_killExtraRXLoops.compare_exchange_strong(needsRxRemoved, needsRxRemoved-1);
            needsRxRemoved = dev->_killExtraRXLoops;
        }
    }
            
    if (!success || removeTransfer){
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
    }else{
        libusb_submit_transfer(xfer);
    }

    if (!success) {
        dev->kill();
    }
    return success && !removeTransfer;
}
