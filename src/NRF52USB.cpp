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

#define nrfusb_errata_166() true

void nrfusb_enable(void) {
    /* Prepare for READY event receiving */
    nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_READY_MASK);
    /* Enable the peripheral */
    nrf_usbd_enable();
    /* Waiting for peripheral to enable, this should take a few us */
    while (0 == (NRF_USBD_EVENTCAUSE_READY_MASK & nrf_usbd_eventcause_get())) {
        /* Empty loop */
    }
    nrf_usbd_eventcause_clear(NRF_USBD_EVENTCAUSE_READY_MASK);

    if (nrfusb_errata_166()) {
        *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7E3;
        *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = 0x40;
        __ISB();
        __DSB();
    }

    nrf_usbd_isosplit_set(NRF_USBD_ISOSPLIT_Half);
}

void nrfusb_disable(void) {
    /* Stop just in case */
    nrfusb_stop();

    /* Disable all parts */
    nrf_usbd_int_disable(nrf_usbd_int_enable_get());
    nrf_usbd_disable();
}

void nrfusb_start() {
    /* Power should be already enabled - wait just in case if user calls
     * app_usbd_start just after app_usbd_enable without waiting for the event. */
    while (!nrf_power_usbregstatus_outrdy_get()) {
        /* Wait for the power but terminate the function if USBD power disappears */
        if (!nrf_power_usbregstatus_vbusdet_get())
            return;
    }

    uint32_t ints_to_enable =
        NRF_USBD_INT_USBRESET_MASK | NRF_USBD_INT_STARTED_MASK | NRF_USBD_INT_ENDEPIN0_MASK |
        NRF_USBD_INT_EP0DATADONE_MASK | NRF_USBD_INT_ENDEPOUT0_MASK | NRF_USBD_INT_USBEVENT_MASK |
        NRF_USBD_INT_EP0SETUP_MASK | NRF_USBD_INT_DATAEP_MASK | NRF_USBD_INT_ACCESSFAULT_MASK;

    /* Enable all required interrupts */
    nrf_usbd_int_enable(ints_to_enable);

    /* Enable interrupt globally */
    nrf_drv_common_irq_enable(USBD_IRQn, USBD_CONFIG_IRQ_PRIORITY);

    /* Enable pullups */
    nrf_usbd_pullup_enable();
}

void nrfusb_stop(void) {
    if (nrf_drv_common_irq_enable_check(USBD_IRQn)) {
        /* Abort transfers */
        // usbd_ep_abort_all();

        /* Disable pullups */
        nrf_usbd_pullup_disable();

        /* Disable interrupt globally */
        nrf_drv_common_irq_disable(USBD_IRQn);

        /* Disable all interrupts */
        nrf_usbd_int_disable(~0U);
    }
}

static void power_usb_event_handler(nrf_drv_power_usb_evt_t event) {
    switch (event) {
    case NRF_DRV_POWER_USB_EVT_DETECTED:
        NRF_LOG_INFO("USB power detected");
        // if (!nrfusb_is_enabled())
        { nrfusb_enable(); }
        break;

    case NRF_DRV_POWER_USB_EVT_REMOVED:
        NRF_LOG_INFO("USB power removed");
        nrfusb_stop();
        break;

    case NRF_DRV_POWER_USB_EVT_READY:
        NRF_LOG_INFO("USB ready");
        nrfusb_start();
        break;

    default:
        ASSERT(false);
    }
}

static void usb_start() {
    if (USBD_POWER_DETECTION) {
        static const nrf_drv_power_usbevt_config_t config = {.handler = power_usb_event_handler};

        ret_code_t ret;
        ret = nrf_drv_power_usbevt_init(&config);
        APP_ERROR_CHECK(ret);
    } else {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        power_usb_event_handler(NRF_DRV_POWER_USB_EVT_DETECTED);
        power_usb_event_handler(NRF_DRV_POWER_USB_EVT_READY);
    }
}

void usb_configure(uint8_t numEndpoints) {
    usb_assert(usb_num_endpoints == 0);
    usb_assert(numEndpoints > 0);

    usb_start();
}

extern "C" void USBD_IRQHandler(void)
{
    const uint32_t enabled = nrf_usbd_int_enable_get();
    uint32_t to_process = enabled;
    uint32_t active = 0;

    /* Check all enabled interrupts */
    while (to_process)
    {
        uint8_t event_nr = __CLZ(__RBIT(to_process));
        if (nrf_usbd_event_get_and_clear((nrf_usbd_event_t)nrf_drv_bitpos_to_event(event_nr)))
        {
            active |= 1UL << event_nr;
        }
        to_process &= ~(1UL << event_nr);
    }

    if (nrf_drv_usbd_errata_104())
    {
        /* Event correcting */
        if ((0 == m_dma_pending) && (0 != (active & (USBD_INTEN_SOF_Msk))))
        {
            uint8_t usbi, uoi, uii;
            /* Testing */
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7A9;
            uii = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != uii)
            {
                uii &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AA;
            uoi = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != uoi)
            {
                uoi &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AB;
            usbi = (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            if (0 != usbi)
            {
                usbi &= (uint8_t)(*((volatile uint32_t *)(NRF_USBD_BASE + 0x804)));
            }
            /* Processing */
            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AC;
            uii &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != uii)
            {
                uint8_t rb;
                m_simulated_dataepstatus |= ((uint32_t)uii) << USBD_EPIN_BITPOS_0;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7A9;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = uii;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            NRF_DRV_USBD_LOG_PROTO1_FIX_PRINTF("   uii: 0x%.2x (0x%.2x)", uii, rb);
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AD;
            uoi &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != uoi)
            {
                uint8_t rb;
                m_simulated_dataepstatus |= ((uint32_t)uoi) << USBD_EPOUT_BITPOS_0;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AA;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = uoi;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            NRF_DRV_USBD_LOG_PROTO1_FIX_PRINTF("   uoi: 0x%.2u (0x%.2x)", uoi, rb);
            }

            *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AE;
            usbi &= (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            if (0 != usbi)
            {
                uint8_t rb;
                if (usbi & 0x01)
                {
                    active |= USBD_INTEN_EP0SETUP_Msk;
                }
                if (usbi & 0x10)
                {
                    active |= USBD_INTEN_USBRESET_Msk;
                }
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x800)) = 0x7AB;
                *((volatile uint32_t *)(NRF_USBD_BASE + 0x804)) = usbi;
                rb = (uint8_t)*((volatile uint32_t *)(NRF_USBD_BASE + 0x804));
            NRF_DRV_USBD_LOG_PROTO1_FIX_PRINTF("   usbi: 0x%.2u (0x%.2x)", usbi, rb);
            }

            if (0 != (m_simulated_dataepstatus &
                ~((1U << USBD_EPOUT_BITPOS_0) | (1U << USBD_EPIN_BITPOS_0))))
            {
                active |= enabled & NRF_USBD_INT_DATAEP_MASK;
            }
            if (0 != (m_simulated_dataepstatus &
                ((1U << USBD_EPOUT_BITPOS_0) | (1U << USBD_EPIN_BITPOS_0))))
            {
                if (0 != (enabled & NRF_USBD_INT_EP0DATADONE_MASK))
                {
                    m_simulated_dataepstatus &=
                        ~((1U << USBD_EPOUT_BITPOS_0) | (1U << USBD_EPIN_BITPOS_0));
                    active |= NRF_USBD_INT_EP0DATADONE_MASK;
                }
            }
        }
    }

    /* Process the active interrupts */
    bool setup_active = 0 != (active & NRF_USBD_INT_EP0SETUP_MASK);
    active &= ~NRF_USBD_INT_EP0SETUP_MASK;

    while (active)
    {
        uint8_t event_nr = __CLZ(__RBIT(active));
        m_isr[event_nr]();
        active &= ~(1UL << event_nr);
    }
    usbd_dmareq_process();

    if (setup_active)
    {
        m_isr[USBD_INTEN_EP0SETUP_Pos]();
    }
}


extern "C" void USB_Handler(void) {
    CodalUSB *cusb = CodalUSB::usbInstance;

#if 0
    DMESG("USB devint=%x ep0int=%x ep1int=%x", USB->DEVICE.INTFLAG.reg,
          USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg, USB->DEVICE.DeviceEndpoint[1].EPINTFLAG.reg);
#endif

    if (USB->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST) {
        DMESG("USB EORST");
        /* Clear the flag */
        USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
        /* Set Device address as 0 */
        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;

        cusb->initEndpoints();
        return;
    }

    if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP) {
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

void usb_set_address(uint16_t wValue) { USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue; }

int UsbEndpointIn::clearStall() {
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

int UsbEndpointIn::reset() {
    DMESG("reset IN %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::stall() {
    DMESG("stall IN %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall() {
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

int UsbEndpointOut::reset() {
    DMESG("reset OUT %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    return DEVICE_OK;
}

int UsbEndpointOut::stall() {
    DMESG("stall OUT %d", ep);
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
    return DEVICE_OK;
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size) {
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

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size) {
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

int UsbEndpointOut::disableIRQ() {
    USB->DEVICE.DeviceEndpoint[ep].EPINTENCLR.reg =
        ep == 0 ? USB_DEVICE_EPINTENCLR_RXSTP : USB_DEVICE_EPINTENCLR_TRCPT0;
    return DEVICE_OK;
}

int UsbEndpointOut::enableIRQ() {
    USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg =
        ep == 0 ? USB_DEVICE_EPINTENSET_RXSTP : USB_DEVICE_EPINTENSET_TRCPT0;
    return DEVICE_OK;
}

void UsbEndpointOut::startRead() {
    UsbDeviceDescriptor *epdesc = (UsbDeviceDescriptor *)usb_endpoints + ep;

    epdesc->DeviceDescBank[0].ADDR.reg = (uint32_t)buf;
    epdesc->DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    epdesc->DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Start the reception by clearing the bank 0 ready bit */
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
}

int UsbEndpointOut::read(void *dst, int maxlen) {
    int packetSize = 0;

    usb_assert(this != NULL);

    uint32_t flag = ep == 0 ? USB_DEVICE_EPINTFLAG_RXSTP : USB_DEVICE_EPINTFLAG_TRCPT0;

    /* Check for Transfer Complete 0 flag */
    if (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & flag) {
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

static void writeEP(UsbDeviceDescriptor *epdesc, uint8_t ep, int len) {
    epdesc->DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = len;
    epdesc->DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Clear the transfer complete flag  */
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
    /* Set the bank as ready */
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;

    /* Wait for transfer to complete */
    while (!(USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1)) {
    }
}

int UsbEndpointIn::write(const void *src, int len) {
    uint32_t data_address;

    // this happens when someone tries to write before USB is initialized
    usb_assert(this != NULL);

    UsbDeviceDescriptor *epdesc = (UsbDeviceDescriptor *)usb_endpoints + ep;

    int epSize = 1 << (epdesc->DeviceDescBank[1].PCKSIZE.bit.SIZE + 3);
    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (wLength) {
        if (len >= wLength) {
            len = wLength;
            // see
            // https://stackoverflow.com/questions/3739901/when-do-usb-hosts-require-a-zero-length-in-packet-at-the-end-of-a-control-read-t
            zlp = 0;
        }
        wLength = 0;
    }

    if (len > epSize) {
        data_address = (uint32_t)src;
        // data must be in RAM!
        usb_assert(data_address >= HMCRAMC0_ADDR);
    } else {
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
