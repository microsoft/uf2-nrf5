#ifndef PLATFORM_INCLUDES
#define PLATFORM_INCLUDES

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define PROCESSOR_WORD_TYPE uint32_t
#define DEVICE_USB 1
#define DEVICE_WEBUSB 1
#define USB_EP_FLAG_NO_AUTO_ZLP 0x01

#define USB_MAX_PKT_SIZE 64
#define DEVICE_USB_ENDPOINTS 7 // TODO check
#define USB_DEFAULT_VID 0x1915
#define USB_DEFAULT_PID 0x521f
#define DEVICE_DMESG_BUFFER_SIZE 0

#endif
