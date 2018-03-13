/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "nrf_usb_uf2.h"

#include <string.h>
#include "boards.h"
#include "nrf_dfu_req_handler.h"
#include "nrf_dfu_transport.h"
#include "slip.h"

#include "nrf_gpio.h"
#include "nrf_drv_power.h"
#include "nrf_drv_usbd.h"

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_msc.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_util_platform.h"

#include "nrf_log.h"

#include "nrf_block_dev_uf2.h"


#define SERIAL_NUMBER_STRING_SIZE       (16)

#define MSC_INTERFACE          0
#define MSC_ENDPOINT 1
#define UF2_BLOCK_COUNT 8000



/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

DFU_TRANSPORT_REGISTER(nrf_dfu_transport_t const dfu_trans) =
{
    .init_func = usb_uf2_transport_init,
    .close_func = usb_uf2_transport_close
};

uint16_t g_extern_serial_number[SERIAL_NUMBER_STRING_SIZE + 1];

static void serial_number_string_create(void)
{
    g_extern_serial_number[0] =
        (uint16_t)APP_USBD_DESCRIPTOR_STRING << 8 | sizeof(g_extern_serial_number);

    uint32_t dev_id_hi = NRF_FICR->DEVICEID[1];
    uint32_t dev_id_lo = NRF_FICR->DEVICEID[0];

    uint64_t device_id = (((uint64_t)dev_id_hi) << 32) | dev_id_lo;

    for (size_t i = 1; i < SERIAL_NUMBER_STRING_SIZE + 1; ++i)
    {
        char tmp[2];
        (void)snprintf(tmp, sizeof(tmp), "%x", (unsigned int)(device_id & 0xF));
        g_extern_serial_number[i] = tmp[0];
        device_id >>= 4;
    }
}




/**
 * @brief Mass storage class user event handler
 * */
static void msc_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_msc_user_event_t     event);


APP_TIMER_DEF(uf2_timer_id);

NRF_BLOCK_DEV_UF2_DEFINE(m_block_dev_uf2,
    NRF_BLOCK_DEV_UF2_CONFIG(VOLUME_LABEL),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "UF2", "1.00"));


#define BLOCKDEV_LIST() (                                   \
    NRF_BLOCKDEV_BASE_ADDR(m_block_dev_uf2, block_dev)     \
)

/**
 * @brief Endpoint list passed to @ref APP_USBD_MSC_GLOBAL_DEF
 * */
#define ENDPOINT_LIST() APP_USBD_MSC_ENDPOINT_LIST(1, 1)

/**
 * @brief Mass storage class work buffer size
 * */
#define MSC_WORKBUFFER_SIZE (1024)

/*lint -save -e26 -e64 -e123 -e505 -e651*/
/**
 * @brief Mass storage class instance
 * */
APP_USBD_MSC_GLOBAL_DEF(m_app_msc,
                        0,
                        msc_user_ev_handler,
                        ENDPOINT_LIST(),
                        BLOCKDEV_LIST(),
                        MSC_WORKBUFFER_SIZE);

/*lint -restore*/

/**
 * @brief  USB connection status
 * */
static bool m_usb_connected = false;

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void msc_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_msc_user_event_t     event)
{
}


static void power_usb_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch (event)
    {
        case NRF_DRV_POWER_USB_EVT_DETECTED:
        {
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }

            nrf_gpio_pin_set(BSP_LED_0);
            nrf_gpio_pin_clear(BSP_LED_1);
        } break;

        case NRF_DRV_POWER_USB_EVT_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            nrf_gpio_pin_set(BSP_LED_1);
            nrf_gpio_pin_clear(BSP_LED_0);
        } break;

        case NRF_DRV_POWER_USB_EVT_READY:
        {
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
        } break;

        default:
            ASSERT(false);
    }
}


static void usb_start(void)
{
    if (USBD_POWER_DETECTION)
    {
        static const nrf_drv_power_usbevt_config_t config =
        {
            .handler = power_usb_event_handler
        };

        ret_code_t ret;
        ret = nrf_drv_power_usbevt_init(&config);
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
        m_usb_connected = true;
    }
}

/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(BSP_LED_0);
    nrf_gpio_cfg_output(BSP_LED_1);
    nrf_gpio_pin_clear(BSP_LED_0);
    nrf_gpio_pin_set(BSP_LED_1);
}


static void usbd_dfu_transport_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        default:
            break;
    }
}


static void usbd_sched_event_handler(void * p_event_data, uint16_t event_size)
{
    app_usbd_event_execute(p_event_data);
}


static void usbd_event_handler(app_usbd_internal_evt_t const * const p_event)
{
    uint32_t ret;
    ret = app_sched_event_put(p_event, sizeof(app_usbd_internal_evt_t), usbd_sched_event_handler);
    ASSERT(NRF_SUCCESS == ret);
}

void uf2_timer(void * p_context);

void uf2_timer_start(int ms)
{
    app_timer_start(uf2_timer_id, APP_TIMER_TICKS(ms), NULL);
}

uint32_t usb_uf2_transport_init(void)
{
    /* Execute event directly in interrupt handler */
    static const app_usbd_config_t usbd_config =
    {
        .ev_handler    = usbd_event_handler,
        .ev_state_proc = usbd_dfu_transport_ev_handler
    };

    uint32_t err_code;

    serial_number_string_create();

    leds_init();

    NRF_LOG_DEBUG("Initializing power driver.");

    err_code = nrf_drv_power_init(NULL);
    VERIFY_SUCCESS(err_code);

    err_code = app_timer_create(&uf2_timer_id, APP_TIMER_MODE_SINGLE_SHOT, uf2_timer);
    VERIFY_SUCCESS(err_code);

    err_code = app_usbd_init(&usbd_config);
    VERIFY_SUCCESS(err_code);

    app_usbd_class_inst_t const * class_inst_msc = app_usbd_msc_class_inst_get(&m_app_msc);
    err_code = app_usbd_class_append(class_inst_msc);
    VERIFY_SUCCESS(err_code);

    //app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    //err_code                                    = app_usbd_class_append(class_cdc_acm);
    //VERIFY_SUCCESS(err_code);

    NRF_LOG_DEBUG("Starting USB");

    usb_start();
    

    NRF_LOG_DEBUG("USB Transport initialized");

    return err_code;
}


uint32_t usb_uf2_transport_close(void)
{
    return NRF_SUCCESS;
}
