/* -*- Mode: C; indent-tabs-mode:t ; c-basic-offset:8 -*- */
/*
 * USB descriptor handling functions for libusb
 * Copyright © 2007 Daniel Drake <dsd@gentoo.org>
 * Copyright © 2001 Johannes Erdfelt <johannes@erdfelt.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */


#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>
#include "libusbi.h"

#define usbi_err(a...)

#define DESC_HEADER_LENGTH    2

/** @defgroup libusb_desc USB descriptors
 * This page details how to examine the various standard USB descriptors
 * for detected devices
 */

static inline uint16_t ReadLittleEndian16(const uint8_t p[2])
{
    return (uint16_t)((uint16_t)p[1] << 8 |
              (uint16_t)p[0]);
}

static inline uint32_t ReadLittleEndian32(const uint8_t p[4])
{
    return (uint32_t)((uint32_t)p[3] << 24 |
              (uint32_t)p[2] << 16 |
              (uint32_t)p[1] << 8 |
              (uint32_t)p[0]);
}

static void clear_endpoint(struct libusb_endpoint_descriptor *endpoint)
{
    free((void *)endpoint->extra);
}

int parse_endpoint(struct libusb_endpoint_descriptor *endpoint, const uint8_t *buffer, int size)
{
    const struct usbi_descriptor_header *header;
    const uint8_t *begin;
    void *extra;
    int parsed = 0;

    if (size < DESC_HEADER_LENGTH) {
//        usbi_err(ctx, "short endpoint descriptor read %d/%d",
//             size, DESC_HEADER_LENGTH);
        return LIBUSB_ERROR_IO;
    }

    header = (const struct usbi_descriptor_header *)buffer;
    if (header->bDescriptorType != LIBUSB_DT_ENDPOINT) {
//        usbi_err(ctx, "unexpected descriptor 0x%x (expected 0x%x)",
//            header->bDescriptorType, LIBUSB_DT_ENDPOINT);
        return parsed;
    } else if (header->bLength < LIBUSB_DT_ENDPOINT_SIZE) {
//        usbi_err(ctx, "invalid endpoint bLength (%u)", header->bLength);
        return LIBUSB_ERROR_IO;
    } else if (header->bLength > size) {
//        usbi_warn(ctx, "short endpoint descriptor read %d/%u",
//              size, header->bLength);
        return parsed;
    }

    endpoint->bLength = buffer[0];
    endpoint->bDescriptorType = buffer[1];
    endpoint->bEndpointAddress = buffer[2];
    endpoint->bmAttributes = buffer[3];
    endpoint->wMaxPacketSize = ReadLittleEndian16(&buffer[4]);
    endpoint->bInterval = buffer[6];
    if (header->bLength >= LIBUSB_DT_ENDPOINT_AUDIO_SIZE) {
        endpoint->bRefresh = buffer[7];
        endpoint->bSynchAddress = buffer[8];
    }

    buffer += header->bLength;
    size -= header->bLength;
    parsed += header->bLength;

    /* Skip over the rest of the Class Specific or Vendor Specific */
    /*  descriptors */
    begin = buffer;
    while (size >= DESC_HEADER_LENGTH) {
        header = (const struct usbi_descriptor_header *)buffer;
        if (header->bLength < DESC_HEADER_LENGTH) {
//            usbi_err(ctx, "invalid extra ep desc len (%u)",
//                 header->bLength);
            return LIBUSB_ERROR_IO;
        } else if (header->bLength > size) {
//            usbi_warn(ctx, "short extra ep desc read %d/%u",
//                  size, header->bLength);
            return parsed;
        }

        /* If we find another "proper" descriptor then we're done  */
        if (header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
            header->bDescriptorType == LIBUSB_DT_INTERFACE ||
            header->bDescriptorType == LIBUSB_DT_CONFIG ||
            header->bDescriptorType == LIBUSB_DT_DEVICE)
            break;

//        usbi_dbg(ctx, "skipping descriptor 0x%x", header->bDescriptorType);
        buffer += header->bLength;
        size -= header->bLength;
        parsed += header->bLength;
    }

    /* Copy any unknown descriptors into a storage area for drivers */
    /*  to later parse */
    ptrdiff_t len = buffer - begin;
    if (len <= 0)
        return parsed;

    extra = malloc((size_t)len);
    if (!extra)
        return LIBUSB_ERROR_NO_MEM;

    memcpy(extra, begin, (size_t)len);
    endpoint->extra = extra;
    endpoint->extra_length = (int)len;

    return parsed;
}

static void clear_interface(struct libusb_interface *usb_interface)
{
    int i;

    if (usb_interface->altsetting) {
        for (i = 0; i < usb_interface->num_altsetting; i++) {
            struct libusb_interface_descriptor *ifp =
                (struct libusb_interface_descriptor *)
                usb_interface->altsetting + i;

            free((void *)ifp->extra);
            if (ifp->endpoint) {
                uint8_t j;

                for (j = 0; j < ifp->bNumEndpoints; j++)
                    clear_endpoint((struct libusb_endpoint_descriptor *)
                               ifp->endpoint + j);
            }
            free((void *)ifp->endpoint);
        }
    }
    free((void *)usb_interface->altsetting);
    usb_interface->altsetting = NULL;
}

int mylibusb_parse_interface(struct libusb_interface *usb_interface, const uint8_t *buffer, int size){
    int r;
    int parsed = 0;
    int interface_number = -1;
    const struct usbi_descriptor_header *header;
    const struct usbi_interface_descriptor *if_desc;
    struct libusb_interface_descriptor *ifp;
    const uint8_t *begin;

    while (size >= LIBUSB_DT_INTERFACE_SIZE) {
        struct libusb_interface_descriptor *altsetting;

        altsetting = realloc((void *)usb_interface->altsetting,
            sizeof(*altsetting) * (size_t)(usb_interface->num_altsetting + 1));
        if (!altsetting) {
            r = LIBUSB_ERROR_NO_MEM;
            goto err;
        }
        usb_interface->altsetting = altsetting;

        ifp = altsetting + usb_interface->num_altsetting;
        ifp->bLength = buffer[0];
        ifp->bDescriptorType = buffer[1];
        ifp->bInterfaceNumber = buffer[2];
        ifp->bAlternateSetting = buffer[3];
        ifp->bNumEndpoints = buffer[4];
        ifp->bInterfaceClass = buffer[5];
        ifp->bInterfaceSubClass = buffer[6];
        ifp->bInterfaceProtocol = buffer[7];
        ifp->iInterface = buffer[8];
        if (ifp->bDescriptorType != LIBUSB_DT_INTERFACE) {
//            usbi_err(ctx, "unexpected descriptor 0x%x (expected 0x%x)",
//                 ifp->bDescriptorType, LIBUSB_DT_INTERFACE);
            return parsed;
        } else if (ifp->bLength < LIBUSB_DT_INTERFACE_SIZE) {
            usbi_err(ctx, "invalid interface bLength (%u)",
                 ifp->bLength);
            r = LIBUSB_ERROR_IO;
            goto err;
        } else if (ifp->bLength > size) {
//            usbi_warn(ctx, "short intf descriptor read %d/%u",
//                 size, ifp->bLength);
            return parsed;
        } else if (ifp->bNumEndpoints > USB_MAXENDPOINTS) {
            usbi_err(ctx, "too many endpoints (%u)", ifp->bNumEndpoints);
            r = LIBUSB_ERROR_IO;
            goto err;
        }

        usb_interface->num_altsetting++;
        ifp->extra = NULL;
        ifp->extra_length = 0;
        ifp->endpoint = NULL;

        if (interface_number == -1)
            interface_number = ifp->bInterfaceNumber;

        /* Skip over the interface */
        buffer += ifp->bLength;
        parsed += ifp->bLength;
        size -= ifp->bLength;

        begin = buffer;

        /* Skip over any interface, class or vendor descriptors */
        while (size >= DESC_HEADER_LENGTH) {
            header = (const struct usbi_descriptor_header *)buffer;
            if (header->bLength < DESC_HEADER_LENGTH) {
                usbi_err(ctx,
                     "invalid extra intf desc len (%u)",
                     header->bLength);
                r = LIBUSB_ERROR_IO;
                goto err;
            } else if (header->bLength > size) {
//                usbi_warn(ctx,
//                      "short extra intf desc read %d/%u",
//                      size, header->bLength);
                return parsed;
            }

            /* If we find another "proper" descriptor then we're done */
            if (header->bDescriptorType == LIBUSB_DT_INTERFACE ||
                header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
                header->bDescriptorType == LIBUSB_DT_CONFIG ||
                header->bDescriptorType == LIBUSB_DT_DEVICE)
                break;

            buffer += header->bLength;
            parsed += header->bLength;
            size -= header->bLength;
        }

        /* Copy any unknown descriptors into a storage area for */
        /*  drivers to later parse */
        ptrdiff_t len = buffer - begin;
        if (len > 0) {
            void *extra = malloc((size_t)len);

            if (!extra) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            memcpy(extra, begin, (size_t)len);
            ifp->extra = extra;
            ifp->extra_length = (int)len;
        }

        if (ifp->bNumEndpoints > 0) {
            struct libusb_endpoint_descriptor *endpoint;
            uint8_t i;

            endpoint = calloc(ifp->bNumEndpoints, sizeof(*endpoint));
            if (!endpoint) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            ifp->endpoint = endpoint;
            for (i = 0; i < ifp->bNumEndpoints; i++) {
                r = parse_endpoint(endpoint + i, buffer, size);
                if (r < 0)
                    goto err;
                if (r == 0) {
                    ifp->bNumEndpoints = i;
                    break;
                }

                buffer += r;
                parsed += r;
                size -= r;
            }
        }

        /* We check to see if it's an alternate to this one */
        if_desc = (const struct usbi_interface_descriptor *)buffer;
        if (size < LIBUSB_DT_INTERFACE_SIZE ||
            if_desc->bDescriptorType != LIBUSB_DT_INTERFACE ||
            if_desc->bInterfaceNumber != interface_number)
            return parsed;
    }

    return parsed;
err:
    clear_interface(usb_interface);
    return r;
}

static void clear_configuration(struct libusb_config_descriptor *config)
{
    uint8_t i;

    if (config->interface) {
        for (i = 0; i < config->bNumInterfaces; i++)
            clear_interface((struct libusb_interface *)
                    config->interface + i);
    }
    free((void *)config->interface);
    free((void *)config->extra);
}

int mylibusb_parse_configuration(struct libusb_config_descriptor *config, const uint8_t *buffer, int size){
    uint8_t i;
    int r;
    const struct usbi_descriptor_header *header;
    struct libusb_interface *usb_interface;

    if (size < LIBUSB_DT_CONFIG_SIZE) {
        usbi_err(ctx, "short config descriptor read %d/%d",
             size, LIBUSB_DT_CONFIG_SIZE);
        return LIBUSB_ERROR_IO;
    }

    config->bLength = buffer[0];
    config->bDescriptorType = buffer[1];
    config->wTotalLength = ReadLittleEndian16(&buffer[2]);
    config->bNumInterfaces = buffer[4];
    config->bConfigurationValue = buffer[5];
    config->iConfiguration = buffer[6];
    config->bmAttributes = buffer[7];
    config->MaxPower = buffer[8];
    if (config->bDescriptorType != LIBUSB_DT_CONFIG) {
        usbi_err(ctx, "unexpected descriptor 0x%x (expected 0x%x)",
             config->bDescriptorType, LIBUSB_DT_CONFIG);
        return LIBUSB_ERROR_IO;
    } else if (config->bLength < LIBUSB_DT_CONFIG_SIZE) {
        usbi_err(ctx, "invalid config bLength (%u)", config->bLength);
        return LIBUSB_ERROR_IO;
    } else if (config->bLength > size) {
        usbi_err(ctx, "short config descriptor read %d/%u",
             size, config->bLength);
        return LIBUSB_ERROR_IO;
    } else if (config->bNumInterfaces > USB_MAXINTERFACES) {
        usbi_err(ctx, "too many interfaces (%u)", config->bNumInterfaces);
        return LIBUSB_ERROR_IO;
    }

    usb_interface = calloc(config->bNumInterfaces, sizeof(*usb_interface));
    if (!usb_interface)
        return LIBUSB_ERROR_NO_MEM;

    config->interface = usb_interface;

    buffer += config->bLength;
    size -= config->bLength;

    for (i = 0; i < config->bNumInterfaces; i++) {
        const uint8_t *begin;

        /* Skip over the rest of the Class Specific or Vendor */
        /*  Specific descriptors */
        begin = buffer;
        while (size >= DESC_HEADER_LENGTH) {
            header = (const struct usbi_descriptor_header *)buffer;
            if (header->bLength < DESC_HEADER_LENGTH) {
                usbi_err(ctx,
                     "invalid extra config desc len (%u)",
                     header->bLength);
                r = LIBUSB_ERROR_IO;
                goto err;
            } else if (header->bLength > size) {
//                usbi_warn(ctx,
//                      "short extra config desc read %d/%u",
//                      size, header->bLength);
                config->bNumInterfaces = i;
                return size;
            }

            /* If we find another "proper" descriptor then we're done */
            if (header->bDescriptorType == LIBUSB_DT_ENDPOINT ||
                header->bDescriptorType == LIBUSB_DT_INTERFACE ||
                header->bDescriptorType == LIBUSB_DT_CONFIG ||
                header->bDescriptorType == LIBUSB_DT_DEVICE)
                break;

//            usbi_dbg(ctx, "skipping descriptor 0x%x", header->bDescriptorType);
            buffer += header->bLength;
            size -= header->bLength;
        }

        /* Copy any unknown descriptors into a storage area for */
        /*  drivers to later parse */
        ptrdiff_t len = buffer - begin;
        if (len > 0) {
            uint8_t *extra = realloc((void *)config->extra,
                         (size_t)(config->extra_length) + (size_t)len);

            if (!extra) {
                r = LIBUSB_ERROR_NO_MEM;
                goto err;
            }

            memcpy(extra + config->extra_length, begin, (size_t)len);
            config->extra = extra;
            config->extra_length += (int)len;
        }

        r = mylibusb_parse_interface(usb_interface + i, buffer, (int)size);
        if (r < 0)
            goto err;
        if (r == 0) {
            config->bNumInterfaces = i;
            break;
        }

        buffer += r;
        size -= r;
    }

    return size;

err:
    clear_configuration(config);
    return r;
}


/** \ingroup libusb_desc
 * Free a superspeed endpoint companion descriptor obtained from
 * libusb_get_ss_endpoint_companion_descriptor().
 * It is safe to call this function with a NULL ep_comp parameter, in which
 * case the function simply returns.
 *
 * \param ep_comp the superspeed endpoint companion descriptor to free
 */

static int parse_bos(struct libusb_context *ctx,
    struct libusb_bos_descriptor **bos,
    const uint8_t *buffer, int size)
{
    struct libusb_bos_descriptor *_bos;
    const struct usbi_bos_descriptor *bos_desc;
    const struct usbi_descriptor_header *header;
    uint8_t i;

    if (size < LIBUSB_DT_BOS_SIZE) {
        usbi_err(ctx, "short bos descriptor read %d/%d",
             size, LIBUSB_DT_BOS_SIZE);
        return LIBUSB_ERROR_IO;
    }

    bos_desc = (const struct usbi_bos_descriptor *)buffer;
    if (bos_desc->bDescriptorType != LIBUSB_DT_BOS) {
        usbi_err(ctx, "unexpected descriptor 0x%x (expected 0x%x)",
             bos_desc->bDescriptorType, LIBUSB_DT_BOS);
        return LIBUSB_ERROR_IO;
    } else if (bos_desc->bLength < LIBUSB_DT_BOS_SIZE) {
        usbi_err(ctx, "invalid bos bLength (%u)", bos_desc->bLength);
        return LIBUSB_ERROR_IO;
    } else if (bos_desc->bLength > size) {
        usbi_err(ctx, "short bos descriptor read %d/%u",
             size, bos_desc->bLength);
        return LIBUSB_ERROR_IO;
    }

    _bos = calloc(1, sizeof(*_bos) + bos_desc->bNumDeviceCaps * sizeof(void *));
    if (!_bos)
        return LIBUSB_ERROR_NO_MEM;

    _bos->bLength = buffer[0];
    _bos->bDescriptorType = buffer[1];
    _bos->wTotalLength = ReadLittleEndian16(&buffer[2]);
    _bos->bNumDeviceCaps = buffer[4];
    buffer += _bos->bLength;
    size -= _bos->bLength;

    /* Get the device capability descriptors */
    for (i = 0; i < _bos->bNumDeviceCaps; i++) {
        if (size < LIBUSB_DT_DEVICE_CAPABILITY_SIZE) {
            usbi_warn(ctx, "short dev-cap descriptor read %d/%d",
                  size, LIBUSB_DT_DEVICE_CAPABILITY_SIZE);
            break;
        }
        header = (const struct usbi_descriptor_header *)buffer;
        if (header->bDescriptorType != LIBUSB_DT_DEVICE_CAPABILITY) {
            usbi_warn(ctx, "unexpected descriptor 0x%x (expected 0x%x)",
                  header->bDescriptorType, LIBUSB_DT_DEVICE_CAPABILITY);
            break;
        } else if (header->bLength < LIBUSB_DT_DEVICE_CAPABILITY_SIZE) {
            usbi_err(ctx, "invalid dev-cap bLength (%u)",
                 header->bLength);
            libusb_free_bos_descriptor(_bos);
            return LIBUSB_ERROR_IO;
        } else if (header->bLength > size) {
            usbi_warn(ctx, "short dev-cap descriptor read %d/%u",
                  size, header->bLength);
            break;
        }

        _bos->dev_capability[i] = malloc(header->bLength);
        if (!_bos->dev_capability[i]) {
            libusb_free_bos_descriptor(_bos);
            return LIBUSB_ERROR_NO_MEM;
        }
        memcpy(_bos->dev_capability[i], buffer, header->bLength);
        buffer += header->bLength;
        size -= header->bLength;
    }
    _bos->bNumDeviceCaps = i;
    *bos = _bos;

    return LIBUSB_SUCCESS;
}

/// @cond DEV
/** \internal \ingroup libusb_desc
 * We use this private struct only to parse a SuperSpeedPlus device capability
 * descriptor according to section 9.6.2.5 of the USB 3.1 specification.
 * We don't expose it.
 */
struct internal_ssplus_capability_descriptor {
    /** The length of the descriptor. Must be equal to LIBUSB_BT_SSPLUS_USB_DEVICE_CAPABILITY_SIZE */
    uint8_t  bLength;

    /** The type of the descriptor */
    uint8_t  bDescriptorType;

    /** Must be equal to LIBUSB_BT_SUPERSPEED_PLUS_CAPABILITY */
    uint8_t  bDevCapabilityType;

    /** Unused */
    uint8_t  bReserved;

    /** Contains the number of SublinkSpeedIDs */
    uint32_t bmAttributes;

    /** Contains the ssid, minRxLaneCount, and minTxLaneCount */
    uint16_t wFunctionalitySupport;

    /** Unused */
    uint16_t wReserved;
};
/// @endcond

