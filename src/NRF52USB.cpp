/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "CodalUSB.h"

#if CONFIG_ENABLED(DEVICE_USB)
#include "samd21.h"
#include "CodalDmesg.h"
#include "system_interrupt.h"

static UsbDeviceDescriptor *usb_endpoints;
static uint8_t usb_num_endpoints;

#define NVM_USB_PAD_TRANSN_POS 45
#define NVM_USB_PAD_TRANSN_SIZE 5
#define NVM_USB_PAD_TRANSP_POS 50
#define NVM_USB_PAD_TRANSP_SIZE 5
#define NVM_USB_PAD_TRIM_POS 55
#define NVM_USB_PAD_TRIM_SIZE 3

#undef ENABLE
#undef DISABLE

void usb_configure(uint8_t numEndpoints)
{
    usb_assert(usb_num_endpoints == 0);
    usb_assert(numEndpoints > 0);


    usb_num_endpoints = numEndpoints;
    usb_endpoints = new UsbDeviceDescriptor[usb_num_endpoints];
    memset(usb_endpoints, 0, usb_num_endpoints * sizeof(UsbDeviceDescriptor));

    uint32_t pad_transn, pad_transp, pad_trim;

    /* Enable USB clock */
    PM->APBBMASK.reg |= PM_APBBMASK_USB;

    /* Set up the USB DP/DN pins */
    PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA24G_USB_DM / 2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA24G_USB_DM / 2].reg |= MUX_PA24G_USB_DM
                                                     << (4 * (PIN_PA24G_USB_DM & 0x01u));
    PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[PIN_PA25G_USB_DP / 2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
    PORT->Group[0].PMUX[PIN_PA25G_USB_DP / 2].reg |= MUX_PA25G_USB_DP
                                                     << (4 * (PIN_PA25G_USB_DP & 0x01u));

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(6) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;

    /* Reset */
    USB->HOST.CTRLA.bit.SWRST = 1;
    while (USB->HOST.SYNCBUSY.bit.SWRST)
    {
        /* Sync wait */
    }

    /* Load Pad Calibration */
    pad_transn = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSN_POS / 32)) >>
                  (NVM_USB_PAD_TRANSN_POS % 32)) &
                 ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

    if (pad_transn == 0x1F)
    {
        pad_transn = 5;
    }

    USB->HOST.PADCAL.bit.TRANSN = pad_transn;

    pad_transp = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSP_POS / 32)) >>
                  (NVM_USB_PAD_TRANSP_POS % 32)) &
                 ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

    if (pad_transp == 0x1F)
    {
        pad_transp = 29;
    }

    USB->HOST.PADCAL.bit.TRANSP = pad_transp;
    pad_trim = (*((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRIM_POS / 32)) >>
                (NVM_USB_PAD_TRIM_POS % 32)) &
               ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

    if (pad_trim == 0x7)
    {
        pad_trim = 3;
    }

    USB->HOST.PADCAL.bit.TRIM = pad_trim;

    /* Set the configuration */
    /* Set mode to Device mode */
    USB->HOST.CTRLA.bit.MODE = 0;
    /* Enable Run in Standby */
    USB->HOST.CTRLA.bit.RUNSTDBY = true;
    /* Set the descriptor address */
    USB->HOST.DESCADD.reg = (uint32_t)(&usb_endpoints[0]);
    /* Set speed configuration to Full speed */
    USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
    /* Attach to the USB host */
    USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH;

    memset(usb_endpoints, 0, usb_num_endpoints * sizeof(UsbDeviceDescriptor));

    USB->DEVICE.INTENCLR.reg = USB_DEVICE_INTFLAG_MASK;
    USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;

    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_USB);

    USB->HOST.CTRLA.bit.ENABLE = true;
}

extern "C" void USB_Handler(void)
{
    CodalUSB *cusb = CodalUSB::usbInstance;

#if 0
    DMESG("USB devint=%x ep0int=%x ep1int=%x", USB->DEVICE.INTFLAG.reg,
          USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg, USB->DEVICE.DeviceEndpoint[1].EPINTFLAG.reg);
#endif

    if (USB->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST)
    {
        DMESG("USB EORST");
        /* Clear the flag */
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
        /* Set Device address as 0 */
        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;

        cusb->initEndpoints();
        return;
    }

    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP)
    {
        // clear the flag
        USBSetup setup;
        int len = cusb->ctrlOut->read(&setup, sizeof(setup));
        usb_assert(len == sizeof(setup));
        USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
        cusb->setupRequest(setup);
        return;
    }

    cusb->interruptHandler();
}

void usb_set_address(uint16_t wValue)
{
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue;
}

int UsbEndpointIn::clearStall()
{
    DMESG("clear stall IN %d", ep);
	if (USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) {
        // Remove stall request
        USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
        if (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
            USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
            // The Stall has occurred, then reset data toggle
            USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
        }
    }
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::reset()
{
    DMESG("reset IN %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::stall()
{
    DMESG("stall IN %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall()
{
    DMESG("clear stall OUT %d", ep);
	if (USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) {
        // Remove stall request
        USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
        if (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
            USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
            // The Stall has occurred, then reset data toggle
            USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
        }
    }
    return DEVICE_OK;
}

int UsbEndpointOut::reset()
{
    DMESG("reset OUT %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    return DEVICE_OK;
}

int UsbEndpointOut::stall()
{
    DMESG("stall OUT %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
    return DEVICE_OK;
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    flags = 0;

    if (type == USB_EP_TYPE_INTERRUPT)
        flags = USB_EP_FLAG_NO_AUTO_ZLP;

    UsbDeviceEndpoint *dep = &USB->DEVICE.DeviceEndpoint[ep];

    // Atmel type 0 is disabled, so types are shifted by 1
    dep->EPCFG.reg =
        USB_DEVICE_EPCFG_EPTYPE1(type + 1) | (dep->EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk);
    /* Set maximum packet size as 64 bytes */
    usb_endpoints[ep].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
    dep->EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;

    dep->EPINTENCLR.reg = USB_DEVICE_EPINTFLAG_MASK;

    // dep->EPINTENSET.reg =
    //    USB_DEVICE_EPINTENSET_TRCPT1 | USB_DEVICE_EPINTENSET_TRFAIL1 |
    //    USB_DEVICE_EPINTENSET_STALL1;
}

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;

    UsbDeviceEndpoint *dep = &USB->DEVICE.DeviceEndpoint[ep];

    dep->EPCFG.reg =
        USB_DEVICE_EPCFG_EPTYPE0(type + 1) | (dep->EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk);
    /* Set maximum packet size as 64 bytes */
    usb_endpoints[ep].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
    usb_endpoints[ep].DeviceDescBank[0].ADDR.reg = (uint32_t)buf;
    dep->EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;

    // dep->EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0 | USB_DEVICE_EPINTENSET_TRFAIL0 |
    //                      USB_DEVICE_EPINTENSET_STALL0 | USB_DEVICE_EPINTENSET_RXSTP;
    dep->EPINTENCLR.reg = USB_DEVICE_EPINTFLAG_MASK;
    enableIRQ();
    startRead();
}

int UsbEndpointOut::disableIRQ()
{
    USB->DEVICE.DeviceEndpoint[ep].EPINTENCLR.reg = 
        ep == 0 ? USB_DEVICE_EPINTENCLR_RXSTP 
                : USB_DEVICE_EPINTENCLR_TRCPT0;
    return DEVICE_OK;
}

int UsbEndpointOut::enableIRQ()
{
    USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = 
        ep == 0 ? USB_DEVICE_EPINTENSET_RXSTP 
                : USB_DEVICE_EPINTENSET_TRCPT0;
    return DEVICE_OK;
}

void UsbEndpointOut::startRead()
{
    UsbDeviceDescriptor *epdesc = (UsbDeviceDescriptor *)usb_endpoints + ep;

    epdesc->DeviceDescBank[0].ADDR.reg = (uint32_t)buf;
    epdesc->DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    epdesc->DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Start the reception by clearing the bank 0 ready bit */
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    int packetSize = 0;

    usb_assert(this != NULL);

    uint32_t flag = ep == 0 ? USB_DEVICE_EPINTFLAG_RXSTP : USB_DEVICE_EPINTFLAG_TRCPT0;

    /* Check for Transfer Complete 0 flag */
    if (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & flag)
    {
        packetSize = usb_endpoints[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT;

        // DMESG("USBRead(%d) => %d bytes", ep, packetSize);

        // Note that we shall discard any excessive data
        if (packetSize > maxlen)
            packetSize = maxlen;

        memcpy(dst, buf, packetSize);

        /* Clear the Transfer Complete 0 flag */
        USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = flag;

        startRead();
    }

    return packetSize;
}

static void writeEP(UsbDeviceDescriptor *epdesc, uint8_t ep, int len)
{
    epdesc->DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = len;
    epdesc->DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Clear the transfer complete flag  */
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    /* Set the bank as ready */
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;

    /* Wait for transfer to complete */
    while (!(USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1))
    {
    }
}

int UsbEndpointIn::write(const void *src, int len)
{
    uint32_t data_address;

    // this happens when someone tries to write before USB is initialized
    usb_assert(this != NULL);

    UsbDeviceDescriptor *epdesc = (UsbDeviceDescriptor *)usb_endpoints + ep;

    int epSize = 1 << (epdesc->DeviceDescBank[1].PCKSIZE.bit.SIZE + 3);
    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (wLength)
    {
        if (len >= wLength) {
            len = wLength;
            // see https://stackoverflow.com/questions/3739901/when-do-usb-hosts-require-a-zero-length-in-packet-at-the-end-of-a-control-read-t
            zlp = 0;
        }
        wLength = 0;
    }

    if (len > epSize)
    {
        data_address = (uint32_t)src;
        // data must be in RAM!
        usb_assert(data_address >= HMCRAMC0_ADDR);
    }
    else
    {
        /* Copy to local buffer */
        memcpy(buf, src, len);
        data_address = (uint32_t)buf;
    }

    epdesc->DeviceDescBank[1].ADDR.reg = data_address;

    writeEP(epdesc, ep, len);

    // It seems AUTO_ZLP has issues with 64 byte control endpoints.
    // We just send ZLP manually if needed.
    if (zlp && len && (len & (epSize - 1)) == 0) {
        writeEP(epdesc, ep, 0);
    }

    return DEVICE_OK;
}

#endif
