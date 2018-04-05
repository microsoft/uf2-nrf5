#include "GhostFAT.h"

namespace codal {

CodalUSB usb;

static const DeviceDescriptor device_desc = {
    0x12,   // bLength
    0x01,   // bDescriptorType
    0x0210, // bcdUSBL

    // Class etc specified per-interface
    0x00, 0x00, 0x00,

    0x40,            // bMaxPacketSize0
    USB_DEFAULT_VID, //
    USB_DEFAULT_PID, //
    0x0202,          // bcdDevice - no HF2 yet
    // 0x4202,          // bcdDevice - leave unchanged for the HF2 to work
    0x01, // iManufacturer
    0x02, // iProduct
    0x03, // SerialNumber
    0x01  // bNumConfigs
};

// TODO extract these from uf2_info()?
static const char *string_descriptors[] = {
    "Example Corp.",
    "NRF52 Bootloader",
    "123456",
};

class FlashFAT : public GhostFAT {
  protected:
  public:
    FlashFAT();
};

FlashFAT fat;

FlashFAT::FlashFAT() {
    usb.stringDescriptors = string_descriptors;
    usb.deviceDescriptor = &device_desc;
    usb.add(*this);
    usb.start();
}

} // namespace codal
