//
//  descriptor.h
//  usbmuxd2
//
//  Created by erd on 20.11.24.
//

#ifndef descriptor_h
#define descriptor_h

#include <libusb-1.0/libusb.h>

#ifdef __cplusplus
extern "C"{
#endif

int mylibusb_parse_configuration(struct libusb_config_descriptor *config, const uint8_t *buffer, int size);

#ifdef __cplusplus
}
#endif

#endif /* descriptor_h */
