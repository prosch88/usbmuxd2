//
//  USBDeviceManager.cpp
//  usbmuxd2
//
//  Created by tihmstar on 20.07.23.
//
#include <libgeneral/macros.h>

#include "USBDeviceManager.hpp"
#include "../Devices/USBDevice.hpp"
#include "../Devices/USBDeviceKIS.hpp"
#include "../MUXException.hpp"

#include <unistd.h>
#include <string.h>

#pragma mark libusb_callback implementations

int usb_hotplug_cb(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data) noexcept{
    int err = 0;
    USBDeviceManager *devmgr = (USBDeviceManager*)user_data;
    
    switch (event) {
        case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
            try {
                debug("Adding device");
                devmgr->device_add(device);
            } catch (tihmstar::exception &e) {
                uint8_t bus = libusb_get_bus_number(device);
                uint8_t address = libusb_get_device_address(device);
                creterror("failed to add device on bus=0x%02x address=0x%02x error=%s code=%d",bus,address,e.what(),e.code());
            }
            break;
        case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
        {
            uint8_t bus = libusb_get_bus_number(device);
            uint8_t address = libusb_get_device_address(device);
            devmgr->_mux->delete_device(bus, address);
        }
            break;
        default:
            error("Unhandled event %d", event);
            break;
    }
    
error:
    return 0;
}

#pragma mark USBDeviceManager
USBDeviceManager::USBDeviceManager(Muxer *parent)
: DeviceManager(parent)
, _ctx(NULL), _usb_hotplug_cb_handle(0)
{
    bool didInit = false;
    cleanup([&]{
        if (!didInit) this->~USBDeviceManager();
    });
    int err = 0;
    info("USBDeviceManager libusb 1.0");
    retassure(libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG), "libusb does not support hotplug events");

    assure(!libusb_init(&_ctx));
    info("Registering for libusb hotplug events");

    retassure(!(err = libusb_hotplug_register_callback(_ctx, static_cast<libusb_hotplug_event>(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT), LIBUSB_HOTPLUG_ENUMERATE, VID_APPLE, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, usb_hotplug_cb, this, &_usb_hotplug_cb_handle)),"ERROR: Could not register for libusb hotplug events (%d)", err);
    didInit = true;
    _devReaperThread = std::thread([this]{
        reaper_runloop();
    });
}

USBDeviceManager::~USBDeviceManager(){
    if (_usb_hotplug_cb_handle) {
        libusb_hotplug_deregister_callback(_ctx, _usb_hotplug_cb_handle); _usb_hotplug_cb_handle = 0;
    }
    if (_children.size()) {
        debug("waiting for usb children to die...");
        std::unique_lock<std::mutex> ul(_childrenLck);
        while (size_t s = _children.size()) {
            for (auto c : _children) c->kill();
            uint64_t wevent = _childrenEvent.getNextEvent();
            ul.unlock();
            debug("Need to kill %zu more usb children",s);
            _childrenEvent.waitForEvent(wevent);
            ul.lock();
        }
    }
    _reapDevices.kill();
    _devReaperThread.join();

    stopLoop();
    safeFreeCustom(_ctx, libusb_exit);
}

#pragma mark inheritance override
bool USBDeviceManager::loopEvent(){
    int err = 0;
    retassure(!(err = libusb_handle_events(_ctx)), "libusb_handle_events_completed failed: %d", err);
    return true;
}

void USBDeviceManager::stopAction() noexcept{
    int err = 0;
    /*
        re-registering the event handler, triggers an event
     */
    if ((err = libusb_hotplug_register_callback(NULL, static_cast<libusb_hotplug_event>(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT), LIBUSB_HOTPLUG_ENUMERATE, VID_APPLE, LIBUSB_HOTPLUG_MATCH_ANY, 0, usb_hotplug_cb, this, &_usb_hotplug_cb_handle))){
        error("Could not re-register for libusb hotplug events (%d)", err);
    }
    libusb_hotplug_deregister_callback(_ctx, _usb_hotplug_cb_handle);
    _usb_hotplug_cb_handle = 0;
}

#pragma mark private members
void USBDeviceManager::add_constructing(uint8_t bus, uint8_t addr){
    uint16_t dev = (uint16_t)(bus<<8) | addr;
    {
        guardWrite(_constructingGuard);
        _constructing.insert(dev);
    }
}

void USBDeviceManager::del_constructing(uint8_t bus, uint8_t addr){
    uint16_t dev = (uint16_t)(bus<<8) | addr;
    {
        guardWrite(_constructingGuard);
        _constructing.erase(dev);
    }
}

bool USBDeviceManager::is_constructing(uint8_t bus, uint8_t addr){
    bool ret = false;
    uint16_t dev = (uint16_t)(bus<<8) | addr;
    {
        guardRead(_constructingGuard);
        ret = _constructing.find(dev) != _constructing.end();
    }
    return ret;
}

void USBDeviceManager::device_add(libusb_device *dev){
    libusb_device_handle *handle = NULL;
    struct libusb_config_descriptor *config = NULL;
    cleanup([&]{
        safeFreeCustom(config, libusb_free_config_descriptor);
        safeFreeCustom(handle, libusb_close);
    });
    int err = 0;
    uint8_t bus = 0;
    uint8_t address = 0;
    struct libusb_device_descriptor devdesc = {};
    std::shared_ptr<USBDevice> newDevice;
    
    bus = libusb_get_bus_number(dev);
    assure((address = libusb_get_device_address(dev))>0);

    if (_mux->have_usb_device(bus, address) || is_constructing(bus, address)) {
        //device already found
        return;
    }
    
    retassure(!(err = libusb_get_device_descriptor(dev, &devdesc)), "Could not get device descriptor for device %d-%d: %d", bus, address, err);

    retassure(devdesc.idVendor == VID_APPLE, "USBDevice is not an Apple device");
    retassure((devdesc.idProduct >= PID_RANGE_LOW && devdesc.idProduct <= PID_RANGE_MAX) || devdesc.idProduct == PID_KIS, "USBDevice is Apple, but PID is not in observe range");

    info("Found new device with v/p %04x:%04x at %d-%d", devdesc.idVendor, devdesc.idProduct, bus, address);

    // No blocking operation can follow: it may be run in the libusb hotplug callback and libusb will refuse any
    // blocking call
    retassure(!(err = libusb_open(dev, &handle)),"Could not open device %d-%d: %d", bus, address, err);

    if (devdesc.idProduct == PID_KIS) {
        newDevice = std::make_shared<USBDeviceKIS>(_mux,this,devdesc.idProduct);
    }else{
        newDevice = std::make_shared<USBDevice>(_mux,this,devdesc.idProduct);
    }
    newDevice->_selfref = newDevice;
    {
        std::unique_lock<std::mutex> ul(_childrenLck);
        _children.insert(newDevice.get());
    }
    
    newDevice->_devdesc = devdesc;
    newDevice->_bus = bus;
    newDevice->_address = address;
    newDevice->_usbdev = handle; handle = NULL; //transfering ownership here!
    
    try {
        newDevice->initDevice();
    } catch (...) {
        newDevice->kill();
        throw;
    }
    
    add_constructing(bus, address);
}

void USBDeviceManager::reaper_runloop(){
    while (true) {
        std::shared_ptr<USBDevice>dev;
        try {
            dev = _reapDevices.wait();
        } catch (...) {
            break;
        }
        //make device go out of scope so it can die in piece
        dev->deconstruct();
    }
}


#pragma mark public


