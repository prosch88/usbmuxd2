/*
 * Internal header for libusb
 * Copyright © 2007-2009 Daniel Drake <dsd@gentoo.org>
 * Copyright © 2001 Johannes Erdfelt <johannes@erdfelt.com>
 * Copyright © 2019 Nathan Hjelm <hjelmn@cs.umm.edu>
 * Copyright © 2019-2020 Google LLC. All rights reserved.
 * Copyright © 2020 Chris Dickens <christopher.a.dickens@gmail.com>
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

#ifndef LIBUSBI_H
#define LIBUSBI_H

#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include "libusb.h"

/* Not all C standard library headers define static_assert in assert.h
 * Additionally, Visual Studio treats static_assert as a keyword.
 */
#if !defined(__cplusplus) && !defined(static_assert) && !defined(_MSC_VER)
#define static_assert(cond, msg) _Static_assert(cond, msg)
#endif

#ifdef NDEBUG
#define ASSERT_EQ(expression, value)    (void)expression
#define ASSERT_NE(expression, value)    (void)expression
#else
#define ASSERT_EQ(expression, value)    assert(expression == value)
#define ASSERT_NE(expression, value)    assert(expression != value)
#endif

#define container_of(ptr, type, member) \
    ((type *)((uintptr_t)(ptr) - (uintptr_t)offsetof(type, member)))

#ifndef ARRAYSIZE
#define ARRAYSIZE(array) (sizeof(array) / sizeof(array[0]))
#endif

#ifndef CLAMP
#define CLAMP(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#endif

#ifndef MIN
#define MIN(a, b)    ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)    ((a) > (b) ? (a) : (b))
#endif

/* The following is used to silence warnings for unused variables */
#if defined(UNREFERENCED_PARAMETER) && !defined(__GNUC__)
#define UNUSED(var)    UNREFERENCED_PARAMETER(var)
#else
#define UNUSED(var)    do { (void)(var); } while(0)
#endif

/* Macro to align a value up to the next multiple of the size of a pointer */
#define PTR_ALIGN(v) \
    (((v) + (sizeof(void *) - 1)) & ~(sizeof(void *) - 1))

/* Atomic operations
 *
 * Useful for reference counting or when accessing a value without a lock
 *
 * The following atomic operations are defined:
 *   usbi_atomic_load() - Atomically read a variable's value
 *   usbi_atomic_store() - Atomically write a new value value to a variable
 *   usbi_atomic_inc() - Atomically increment a variable's value and return the new value
 *   usbi_atomic_dec() - Atomically decrement a variable's value and return the new value
 *
 * All of these operations are ordered with each other, thus the effects of
 * any one operation is guaranteed to be seen by any other operation.
 */
#ifdef _MSC_VER
typedef volatile LONG usbi_atomic_t;
#define usbi_atomic_load(a)    (*(a))
#define usbi_atomic_store(a, v)    (*(a)) = (v)
#define usbi_atomic_inc(a)    InterlockedIncrement((a))
#define usbi_atomic_dec(a)    InterlockedDecrement((a))
#else
#if defined(__HAIKU__) && defined(__GNUC__) && !defined(__clang__)
/* The Haiku port of libusb has some C++ files and GCC does not define
 * anything in stdatomic.h when compiled in C++11 (only in C++23).
 * This appears to be a bug in gcc's stdatomic.h, and should be fixed either
 * in gcc or in Haiku. Until then, use the gcc builtins. */
typedef long usbi_atomic_t;
#define usbi_atomic_load(a)    __atomic_load_n((a), __ATOMIC_SEQ_CST)
#define usbi_atomic_store(a, v)        __atomic_store_n((a), (v), __ATOMIC_SEQ_CST)
#define usbi_atomic_inc(a)     __atomic_add_fetch((a), 1, __ATOMIC_SEQ_CST)
#define usbi_atomic_dec(a)     __atomic_sub_fetch((a), 1, __ATOMIC_SEQ_CST)
#else
#include <stdatomic.h>
typedef atomic_long usbi_atomic_t;
#define usbi_atomic_load(a)    atomic_load((a))
#define usbi_atomic_store(a, v)    atomic_store((a), (v))
#define usbi_atomic_inc(a)    (atomic_fetch_add((a), 1) + 1)
#define usbi_atomic_dec(a)    (atomic_fetch_add((a), -1) - 1)
#endif
#endif

/* Internal abstractions for event handling and thread synchronization */
#if defined(PLATFORM_POSIX)
#include "os/events_posix.h"
#include "os/threads_posix.h"
#elif defined(PLATFORM_WINDOWS)
#include "os/events_windows.h"
#include "os/threads_windows.h"
#endif

/* Inside the libusb code, mark all public functions as follows:
 *   return_type API_EXPORTED function_name(params) { ... }
 * But if the function returns a pointer, mark it as follows:
 *   DEFAULT_VISIBILITY return_type * LIBUSB_CALL function_name(params) { ... }
 * In the libusb public header, mark all declarations as:
 *   return_type LIBUSB_CALL function_name(params);
 */
#define API_EXPORTED LIBUSB_CALL DEFAULT_VISIBILITY
#define API_EXPORTEDV LIBUSB_CALLV DEFAULT_VISIBILITY

#ifdef __cplusplus
extern "C" {
#endif

#define USB_MAXENDPOINTS    32
#define USB_MAXINTERFACES    32
#define USB_MAXCONFIG        8

/* Backend specific capabilities */
#define USBI_CAP_HAS_HID_ACCESS            0x00010000
#define USBI_CAP_SUPPORTS_DETACH_KERNEL_DRIVER    0x00020000

/* Maximum number of bytes in a log line */
#define USBI_MAX_LOG_LEN    1024
/* Terminator for log lines */
#define USBI_LOG_LINE_END    "\n"

struct list_head {
    struct list_head *prev, *next;
};

/* Get an entry from the list
 *  ptr - the address of this list_head element in "type"
 *  type - the data type that contains "member"
 *  member - the list_head element in "type"
 */
#define list_entry(ptr, type, member) \
    container_of(ptr, type, member)

#define list_first_entry(ptr, type, member) \
    list_entry((ptr)->next, type, member)

#define list_next_entry(ptr, type, member) \
    list_entry((ptr)->member.next, type, member)

/* Get each entry from a list
 *  pos - A structure pointer has a "member" element
 *  head - list head
 *  member - the list_head element in "pos"
 *  type - the type of the first parameter
 */
#define list_for_each_entry(pos, head, member, type)            \
    for (pos = list_first_entry(head, type, member);        \
         &pos->member != (head);                \
         pos = list_next_entry(pos, type, member))

#define list_for_each_entry_safe(pos, n, head, member, type)        \
    for (pos = list_first_entry(head, type, member),        \
         n = list_next_entry(pos, type, member);        \
         &pos->member != (head);                \
         pos = n, n = list_next_entry(n, type, member))

/* Helper macros to iterate over a list. The structure pointed
 * to by "pos" must have a list_head member named "list".
 */
#define for_each_helper(pos, head, type) \
    list_for_each_entry(pos, head, list, type)

#define for_each_safe_helper(pos, n, head, type) \
    list_for_each_entry_safe(pos, n, head, list, type)

#define list_empty(entry) ((entry)->next == (entry))

static inline void list_init(struct list_head *entry)
{
    entry->prev = entry->next = entry;
}

static inline void list_add(struct list_head *entry, struct list_head *head)
{
    entry->next = head->next;
    entry->prev = head;

    head->next->prev = entry;
    head->next = entry;
}

static inline void list_add_tail(struct list_head *entry,
    struct list_head *head)
{
    entry->next = head;
    entry->prev = head->prev;

    head->prev->next = entry;
    head->prev = entry;
}

static inline void list_del(struct list_head *entry)
{
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
    entry->next = entry->prev = NULL;
}

static inline void list_cut(struct list_head *list, struct list_head *head)
{
    if (list_empty(head)) {
        list_init(list);
        return;
    }

    list->next = head->next;
    list->next->prev = list;
    list->prev = head->prev;
    list->prev->next = list;

    list_init(head);
}

static inline void list_splice_front(struct list_head *list, struct list_head *head)
{
    list->next->prev = head;
    list->prev->next = head->next;
    head->next->prev = list->prev;
    head->next = list->next;
}

static inline void *usbi_reallocf(void *ptr, size_t size)
{
    void *ret = realloc(ptr, size);

    if (!ret)
        free(ptr);
    return ret;
}

#if !defined(USEC_PER_SEC)
#define USEC_PER_SEC    1000000L
#endif

#if !defined(NSEC_PER_SEC)
#define NSEC_PER_SEC    1000000000L
#endif

#define TIMEVAL_IS_VALID(tv)                        \
    ((tv)->tv_sec >= 0 &&                        \
     (tv)->tv_usec >= 0 && (tv)->tv_usec < USEC_PER_SEC)

#define TIMESPEC_IS_SET(ts)    ((ts)->tv_sec || (ts)->tv_nsec)
#define TIMESPEC_CLEAR(ts)    (ts)->tv_sec = (ts)->tv_nsec = 0
#define TIMESPEC_CMP(a, b, CMP)                        \
    (((a)->tv_sec == (b)->tv_sec)                    \
     ? ((a)->tv_nsec CMP (b)->tv_nsec)                \
     : ((a)->tv_sec CMP (b)->tv_sec))
#define TIMESPEC_SUB(a, b, result)                    \
    do {                                \
        (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;        \
        (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec;    \
        if ((result)->tv_nsec < 0L) {                \
            --(result)->tv_sec;                \
            (result)->tv_nsec += NSEC_PER_SEC;        \
        }                            \
    } while (0)

#if defined(PLATFORM_WINDOWS)
#define TIMEVAL_TV_SEC_TYPE    long
#else
#define TIMEVAL_TV_SEC_TYPE    time_t
#endif

/* Some platforms don't have this define */
#ifndef TIMESPEC_TO_TIMEVAL
#define TIMESPEC_TO_TIMEVAL(tv, ts)                    \
    do {                                \
        (tv)->tv_sec = (TIMEVAL_TV_SEC_TYPE) (ts)->tv_sec;    \
        (tv)->tv_usec = (ts)->tv_nsec / 1000L;            \
    } while (0)
#endif

#ifdef ENABLE_LOGGING

#if defined(_MSC_VER) && (_MSC_VER < 1900)
#include <stdio.h>
#define snprintf usbi_snprintf
#define vsnprintf usbi_vsnprintf
int usbi_snprintf(char *dst, size_t size, const char *format, ...);
int usbi_vsnprintf(char *dst, size_t size, const char *format, va_list args);
#define LIBUSB_PRINTF_WIN32
#endif /* defined(_MSC_VER) && (_MSC_VER < 1900) */

void usbi_log(struct libusb_context *ctx, enum libusb_log_level level,
    const char *function, const char *format, ...) PRINTF_FORMAT(4, 5);

#define _usbi_log(ctx, level, ...) usbi_log(ctx, level, __func__, __VA_ARGS__)

#define usbi_err(ctx, ...)    _usbi_log(ctx, LIBUSB_LOG_LEVEL_ERROR, __VA_ARGS__)
#define usbi_warn(ctx, ...)    _usbi_log(ctx, LIBUSB_LOG_LEVEL_WARNING, __VA_ARGS__)
#define usbi_info(ctx, ...)    _usbi_log(ctx, LIBUSB_LOG_LEVEL_INFO, __VA_ARGS__)
#define usbi_dbg(ctx ,...)          _usbi_log(ctx, LIBUSB_LOG_LEVEL_DEBUG, __VA_ARGS__)

#else /* ENABLE_LOGGING */

#define usbi_err(ctx, ...)    do { (void)(ctx); } while(0)
#define usbi_warn(ctx, ...)    do { (void)(ctx); } while(0)
#define usbi_info(ctx, ...)    do { (void)(ctx); } while(0)
#define usbi_dbg(ctx, ...)    do { (void)(ctx); } while(0)

#endif /* ENABLE_LOGGING */

#define DEVICE_CTX(dev)        ((dev)->ctx)
#define HANDLE_CTX(handle)    ((handle) ? DEVICE_CTX((handle)->dev) : NULL)
#define ITRANSFER_CTX(itransfer) \
    ((itransfer)->dev ? DEVICE_CTX((itransfer)->dev) : NULL)
#define TRANSFER_CTX(transfer) \
    (ITRANSFER_CTX(LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer)))

#define IS_EPIN(ep)        (0 != ((ep) & LIBUSB_ENDPOINT_IN))
#define IS_EPOUT(ep)        (!IS_EPIN(ep))
#define IS_XFERIN(xfer)        (0 != ((xfer)->endpoint & LIBUSB_ENDPOINT_IN))
#define IS_XFEROUT(xfer)    (!IS_XFERIN(xfer))


enum usbi_transfer_state_flags {
    /* Transfer successfully submitted by backend */
    USBI_TRANSFER_IN_FLIGHT = 1U << 0,

    /* Cancellation was requested via libusb_cancel_transfer() */
    USBI_TRANSFER_CANCELLING = 1U << 1,

    /* Operation on the transfer failed because the device disappeared */
    USBI_TRANSFER_DEVICE_DISAPPEARED = 1U << 2,
};

enum usbi_transfer_timeout_flags {
    /* Set by backend submit_transfer() if the OS handles timeout */
    USBI_TRANSFER_OS_HANDLES_TIMEOUT = 1U << 0,

    /* The transfer timeout has been handled */
    USBI_TRANSFER_TIMEOUT_HANDLED = 1U << 1,

    /* The transfer timeout was successfully processed */
    USBI_TRANSFER_TIMED_OUT = 1U << 2,
};

#define TRANSFER_PRIV_TO_USBI_TRANSFER(transfer_priv) \
    ((struct usbi_transfer *)            \
     ((unsigned char *)(transfer_priv)    \
      + PTR_ALIGN(sizeof(*transfer_priv))))

#define USBI_TRANSFER_TO_TRANSFER_PRIV(itransfer) \
    ((unsigned char *)            \
     ((unsigned char *)(itransfer)    \
      - PTR_ALIGN(usbi_backend.transfer_priv_size)))

#define USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer)    \
    ((struct libusb_transfer *)            \
     ((unsigned char *)(itransfer)            \
      + PTR_ALIGN(sizeof(struct usbi_transfer))))

#define LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer)    \
    ((struct usbi_transfer *)            \
     ((unsigned char *)(transfer)            \
      - PTR_ALIGN(sizeof(struct usbi_transfer))))

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif



#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
#define LIBUSB_FLEXIBLE_ARRAY        /* [] - valid C99 code */
#else
#define LIBUSB_FLEXIBLE_ARRAY    0    /* [0] - non-standard, but usually working code */
#endif /* __STDC_VERSION__ */


/* All standard descriptors have these 2 fields in common */
struct usbi_descriptor_header {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
} LIBUSB_PACKED;

struct usbi_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} LIBUSB_PACKED;

struct usbi_configuration_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} LIBUSB_PACKED;

struct usbi_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} LIBUSB_PACKED;

struct usbi_string_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wData[LIBUSB_FLEXIBLE_ARRAY];
} LIBUSB_PACKED;

struct usbi_bos_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumDeviceCaps;
} LIBUSB_PACKED;

#ifdef _MSC_VER
#pragma pack(pop)
#endif

union usbi_config_desc_buf {
        struct usbi_configuration_descriptor desc;
        uint8_t buf[LIBUSB_DT_CONFIG_SIZE];
        uint16_t align;         /* Force 2-byte alignment */
};

union usbi_string_desc_buf {
        struct usbi_string_descriptor desc;
        uint8_t buf[255];       /* Some devices choke on size > 255 */
        uint16_t align;         /* Force 2-byte alignment */
};

union usbi_bos_desc_buf {
        struct usbi_bos_descriptor desc;
        uint8_t buf[LIBUSB_DT_BOS_SIZE];
        uint16_t align;         /* Force 2-byte alignment */
};

enum usbi_hotplug_flags {
    /* This callback is interested in device arrivals */
    USBI_HOTPLUG_DEVICE_ARRIVED = LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,

    /* This callback is interested in device removals */
    USBI_HOTPLUG_DEVICE_LEFT = LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,

    /* IMPORTANT: The values for the below entries must start *after*
     * the highest value of the above entries!!!
     */

    /* The vendor_id field is valid for matching */
    USBI_HOTPLUG_VENDOR_ID_VALID = (1U << 3),

    /* The product_id field is valid for matching */
    USBI_HOTPLUG_PRODUCT_ID_VALID = (1U << 4),

    /* The dev_class field is valid for matching */
    USBI_HOTPLUG_DEV_CLASS_VALID = (1U << 5),

    /* This callback has been unregistered and needs to be freed */
    USBI_HOTPLUG_NEEDS_FREE = (1U << 6),
};

struct usbi_hotplug_callback {
    /* Flags that control how this callback behaves */
    uint8_t flags;

    /* Vendor ID to match (if flags says this is valid) */
    uint16_t vendor_id;

    /* Product ID to match (if flags says this is valid) */
    uint16_t product_id;

    /* Device class to match (if flags says this is valid) */
    uint8_t dev_class;

    /* Callback function to invoke for matching event/device */
    libusb_hotplug_callback_fn cb;

    /* Handle for this callback (used to match on deregister) */
    libusb_hotplug_callback_handle handle;

    /* User data that will be passed to the callback function */
    void *user_data;

    /* List this callback is registered in (ctx->hotplug_cbs) */
    struct list_head list;
};

struct usbi_hotplug_message {
    /* The hotplug event that occurred */
    libusb_hotplug_event event;

    /* The device for which this hotplug event occurred */
    struct libusb_device *device;

    /* List this message is contained in (ctx->hotplug_msgs) */
    struct list_head list;
};

/* shared data and functions */

extern const struct usbi_os_backend usbi_backend;

#define for_each_context(c) \
    for_each_helper(c, &active_contexts_list, struct libusb_context)

#define for_each_device(ctx, d) \
    for_each_helper(d, &(ctx)->usb_devs, struct libusb_device)

#define for_each_device_safe(ctx, d, n) \
    for_each_safe_helper(d, n, &(ctx)->usb_devs, struct libusb_device)

#define for_each_open_device(ctx, h) \
    for_each_helper(h, &(ctx)->open_devs, struct libusb_device_handle)

#define __for_each_transfer(list, t) \
    for_each_helper(t, (list), struct usbi_transfer)

#define for_each_transfer(ctx, t) \
    __for_each_transfer(&(ctx)->flying_transfers, t)

#define __for_each_transfer_safe(list, t, n) \
    for_each_safe_helper(t, n, (list), struct usbi_transfer)

#define for_each_transfer_safe(ctx, t, n) \
    __for_each_transfer_safe(&(ctx)->flying_transfers, t, n)

#define __for_each_completed_transfer_safe(list, t, n) \
    list_for_each_entry_safe(t, n, (list), completed_list, struct usbi_transfer)

#define for_each_event_source(ctx, e) \
    for_each_helper(e, &(ctx)->event_sources, struct usbi_event_source)

#define for_each_removed_event_source(ctx, e) \
    for_each_helper(e, &(ctx)->removed_event_sources, struct usbi_event_source)

#define for_each_removed_event_source_safe(ctx, e, n) \
    for_each_safe_helper(e, n, &(ctx)->removed_event_sources, struct usbi_event_source)

#define for_each_hotplug_cb(ctx, c) \
    for_each_helper(c, &(ctx)->hotplug_cbs, struct usbi_hotplug_callback)

#define for_each_hotplug_cb_safe(ctx, c, n) \
    for_each_safe_helper(c, n, &(ctx)->hotplug_cbs, struct usbi_hotplug_callback)

#ifdef __cplusplus
}
#endif

#endif
