/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "zlib.h"
// Pico
#include "pico/stdlib.h"

// For memcpy
#include <string.h>

// Include descriptor struct definitions
#include "usb_common.h"
// USB register definitions from pico-sdk
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk
#include "hardware/structs/usb.h"
// For interrupt enable and numbers
#include "hardware/irq.h"
// For resetting the USB controller
#include "hardware/resets.h"

// Device descriptors
#include "dev_lowlevel.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "apa104.pio.h"
//#include "apa104.h"
#include "led_param.h"
#include "hardware/gpio.h" //for gpio irq
#include "cmd_parser.h"
#include "pico/sem.h"
struct semaphore led_frame_sem;
bool b_clear_led_frame = false;
bool b_set_current_gain = false;
#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

uint32_t test_pattern = COLOR_WHITE;
int32_t led_select = -1;//-1 means all

int32_t led_lighting_mode = LED_NORMAL_MODE; //分區或是全部點亮

//move to led_param.h
//#define LED_NUM         1000
//#define LED_CHANNELS    3
//#define LED_PORTS       8
uint8_t led_patterns[LED_PORTS][LED_NUM][LED_CHANNELS] = {0};
//int LED_WIDTH = 40;
//int LED_HEIGHT = 24;
uint32_t led_total_width = LED_WIDTH;
uint32_t led_total_height = LED_HEIGHT;
int32_t led_area_startx = 0;
int32_t led_area_starty = 0;
int32_t led_area_width = LED_WIDTH;
int32_t led_area_height = LED_HEIGHT;


void gpio_callback(uint gpio, uint32_t events);
int32_t gpio_irq_enable(uint32_t gpio, void* callback, uint32_t condition);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    printf("Timer %d fired!\n", (int) id);
    //timer_fired = true;
    gpio_irq_enable(15, &gpio_callback, GPIO_IRQ_EDGE_RISE );
    // Can return a value here in us to fire in the future
    return 0;
}


void gpio_callback(uint gpio, uint32_t events) {
    gpio_set_irq_enabled_with_callback(15, NULL, false, NULL);
    printf("%s\n", __func__);
    if(test_pattern == COLOR_WHITE){
    	test_pattern = COLOR_RED;
	printf("RED!\n");
    }else if(test_pattern == COLOR_RED){
    	test_pattern = COLOR_GREEN;
	printf("GREEN!\n");
    }else if(test_pattern == COLOR_GREEN){
    	test_pattern = COLOR_BLUE;
	printf("BLUE!\n");
    }else if(test_pattern == COLOR_BLUE){
    	test_pattern = COLOR_WHITE;
	printf("WHITE!\n");
    }
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

}

int32_t gpio_irq_enable(uint32_t gpio, void* callback, uint32_t condition) {
    gpio_set_irq_enabled_with_callback(gpio, condition, true, callback);
}

// add for LED apa104 control
const int PIN_TX_0 = 4;
const int PIN_TX_1 = 5;
const int PIN_TX_2 = 6;
const int PIN_TX_3 = 7;

const int PIN_TX_4 = 16;
const int PIN_TX_5 = 17;
const int PIN_TX_6 = 18;
const int PIN_TX_7 = 19;
//PIO pio_0 = pio0
//PIO pio_1 = pio1

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 1, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
    pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 0, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 1, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 2, pixel_grb << 8u);
    pio_sm_put_blocking(pio1, 3, pixel_grb << 8u);
}

static inline void put_pixel_by_panel(uint8_t panel_id, uint32_t pixel_grb) {
    switch(panel_id){
        case 0:
            pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
            break;
        case 1:
            pio_sm_put_blocking(pio0, 1, pixel_grb << 8u);
            break;
        case 2:
            pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
            break;
        case 3:
            pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
            break;
        case 4:
            pio_sm_put_blocking(pio1, 0, pixel_grb << 8u);
            break;
        case 5:
            pio_sm_put_blocking(pio1, 1, pixel_grb << 8u);
            break;
        case 6:
            pio_sm_put_blocking(pio1, 2, pixel_grb << 8u);
            break;
        case 7:
            pio_sm_put_blocking(pio1, 3, pixel_grb << 8u);
            break;
        default:
            printf("error!no such panel id: %d\n", panel_id);
            break;
    }
}


static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

static int32_t pio_initial() {

    //pio0 4 sm port
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, PIN_TX_0, 800000, false);
    ws2812_program_init(pio, sm+1, offset, PIN_TX_1, 800000, false);
    ws2812_program_init(pio, sm+2, offset, PIN_TX_2, 800000, false);
    ws2812_program_init(pio, sm+3, offset, PIN_TX_3, 800000, false);
    
    //pio0 4 sm port
    PIO pio_1 = pio1;
    int sm_1 = 0;
    uint offset_1 = pio_add_program(pio1, &ws2812_program);
    ws2812_program_init(pio_1, sm_1, offset_1, PIN_TX_4, 800000, false);
    ws2812_program_init(pio_1, sm_1+1, offset_1, PIN_TX_5, 800000, false);
    ws2812_program_init(pio_1, sm_1+2, offset_1, PIN_TX_6, 800000, false);
    ws2812_program_init(pio_1, sm_1+3, offset_1, PIN_TX_7, 800000, false);
    return 0;
}

// Function prototypes for our device specific endpoint handlers defined
// later on
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep1_out_handler(uint8_t *buf, uint16_t len);
void ep2_in_handler(uint8_t *buf, uint16_t len);

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_handler,
                        .endpoint_control = NULL, // NA for EP0
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_handler,
                        .endpoint_control = NULL, // NA for EP0,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep1_out,
                        .handler = &ep1_out_handler,
                        // EP1 starts at offset 0 for endpoint control
                        .endpoint_control = &usb_dpram->ep_ctrl[0].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].out,
                        // First free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_in,
                        .handler = &ep2_in_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].in,
                        // Second free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of that endpoint. Returns NULL
 * if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration*
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen(str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf) {
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep) {
    printf("Set up endpoint 0x%x with buffer address 0x%p\n", ep->descriptor->bEndpointAddress, ep->data_buffer);

    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) {
        return;
    }

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 *
 */
void usb_setup_endpoints() {
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init() {
    // Reset usb controller
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear any previous state in dpram just in case
    memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>

    // Enable USB interrupt at processor
    irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable an interrupt per EP0 transaction
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; // <2>

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    // Set up endpoints (endpoint control registers)
    // described by device configuration
    usb_setup_endpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len) {
    // We are asserting that the length is <= 64 bytes for simplicity of the example.
    // For multi packet transfers see the tinyusb port.
    assert(len <= 64);

    //printf("Start transfer of len %d on ep addr 0x%x\n", len, ep->descriptor->bEndpointAddress);

    // Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        // Need to copy the data from the user buffer to the usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        // Mark as full
        val |= USB_BUF_CTRL_FULL;
    }

    // Set pid and flip for next transfer
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 *
 */
void usb_handle_device_descriptor(void) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, sizeof(struct usb_device_descriptor));
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *) buf, dev_config.interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        const struct usb_endpoint_configuration *ep = dev_config.endpoints;

        // Copy all the endpoint descriptors starting from EP1
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }

    }

    // Send data
    // Get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back to 0.
 *
 */
void usb_bus_reset(void) {
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], len);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the device address in
 * hardware is done in ep0_in_handler. This is because we have to acknowledge the request first
 * as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // Set address is a bit of a strange case because we have to send a 0 length status packet first with
    // address 0
    dev_addr = (pkt->wValue & 0xff);
    printf("Set address %d\r\n", dev_addr);
    // Will set address in the callback phase
    should_set_address = true;
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one configuration so simply
 * sends a zero length status packet back to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt) {
    // Only one configuration so just acknowledge the request
    printf("Device Enumerated\r\n");
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    configured = true;
}

/**
 * @brief Respond to a setup packet from the host.
 *
 */
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
            printf("Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor();
                    printf("GET DEVICE DESCRIPTOR\r\n");
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR\r\n");
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR\r\n");
                    break;

                default:
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    // Get the transfer length for this endpoint
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    // Call that endpoints buffer done handler
    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 * direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    //printf("EP %d (in = %d) done\n", ep_num, in);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 * buffers have been sent / received. Notify each endpoint where this
 * is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 *
 */
/// \tag::isr_setup_packet[]
void isr_usbctrl(void) {
    // USB interrupt handler
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }
/// \end::isr_setup_packet[]

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        printf("BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}

/**
 * @brief EP0 in transfer complete. Either finish the SET_ADDRESS process, or receive a zero
 * length status packet from the host.
 *
 * @param buf the data that was sent
 * @param len the length that was sent
 */
void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        // Set actual device address in hardware
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep0_out_handler(uint8_t *buf, uint16_t len) {
    ;
}

uint8_t tmp_buf[64];
// Device specific functions
void ep1_out_handler(uint8_t *buf, uint16_t len) {
    //printf("RX %d bytes from host\n", len);

    	
    //memcpy(tmp_buf, buf, len);	
    printf("got : \n %s\n", buf);
    cmd_parser(buf);
    memset(buf, 0, 64);
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);
    // Send data back to host
    //struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP2_IN_ADDR);
    //usb_start_transfer(ep, buf, len);
}

void ep2_in_handler(uint8_t *buf, uint16_t len) {
    printf("Sent %d bytes to host\n", len);
    // Get ready to rx again from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);
}

void set_current_gain(int gain_value){
    gpio_init(4);
    gpio_init(5);
    gpio_set_dir(4, true);
    gpio_set_dir(5, true);
    gpio_put(4, 0);
    gpio_put(5, 0);
    sleep_us(250);

    gpio_put(4, 1);
    gpio_put(5, 1);
    sleep_us(20);

    gpio_put(4, 0);
    gpio_put(5, 0);
    sleep_us(250);

    //gpio_put(4, 1);
    //gpio_put(5, 1);
    //sleep_us(80);


    pio_initial();
    for(int j = 0; j < 8; j++){
        int pattern = 0xff0000;//led_cmd[0][j][1] << 16 | led_cmd[0][j][0] << 8 | led_cmd[0][j][2] << 0;
        put_pixel_by_panel(j, pattern);
    }
    
    sleep_ms(1);

}


int main(void) {
    int i, j, k;
    uint8_t tmp_color = 0x01;
    stdio_init_all();
    printf("USB Device Low-Level hardware example\n");
    usb_device_init();
    sem_init(&led_frame_sem, 1, 1);
    pio_initial();
    int ret = gpio_get_dir(15);
    printf("gpio 15 dir is %d\n", ret); //default is in
    gpio_pull_up(15);
    gpio_irq_enable(15, &gpio_callback, GPIO_IRQ_EDGE_RISE );
    // Wait until configured
    while (!configured) {
        tight_loop_contents();
    }
    for(j = 0; j < LED_PORTS; j++){    
        for(i = 0; i < LED_NUM; i++){
            for(k = 0; k < LED_CHANNELS; k++){
                led_patterns[j][i][k] = 0x10; 
            }
        }
    }
    // Get ready to rx from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);

    // Everything is interrupt driven so just loop here
    while (1) {
                    

        //tight_loop_contents(); //marked this busy loop
        sem_acquire_blocking(&led_frame_sem);
        if(b_set_current_gain == true){
            set_current_gain(0xff);
            b_set_current_gain = false;
            sem_release(&led_frame_sem);
            printf("set current gain\n");
            continue;
        }

        if(b_clear_led_frame == true){
	        for(i = 0; i < 40; i++){
	            for(j = 0; j < 24; j++){
                    for(k = 0; k < LED_PORTS; k++){
                        test_pattern = 0x000000;
	    	            put_pixel_by_panel(k, test_pattern);
                    }
	            }
	        }
            sleep_ms(10);
                
            b_clear_led_frame = false;
            sem_release(&led_frame_sem);
            printf("clear frame\n");
            continue;
        }
        sem_release(&led_frame_sem);
 
	    //test pattern
        if(led_lighting_mode == LED_NORMAL_MODE){
	        for(i = 0; i < LED_WIDTH; i++){
	            for(j = 0; j < LED_HEIGHT; j++){
                    for(k = 0; k < LED_PORTS; k++){
                        test_pattern = led_patterns[k][((j*LED_WIDTH) + i)][0] << 16 |  //g
                                        led_patterns[k][((j*LED_WIDTH) + i)][1] << 8 |  //r
                                        led_patterns[k][((j*LED_WIDTH) + i)][2] ;       //b
	    	            put_pixel_by_panel(k, test_pattern);
                    }
	            }
	        }
            sleep_ms(10);
        }else{
            /*printf("led_total_width = %d\n", led_total_width);
            printf("led_total_height = %d\n", led_total_height);
            printf("led_area_startx = %d\n", led_area_startx);
            printf("led_area_starty = %d\n", led_area_starty);
            printf("led_area_width = %d\n", led_area_width);
            printf("led_area_height = %d\n", led_area_height);*/
	        for(j = 0; j < led_total_height; j++){
	            for(i = 0; i < led_total_width; i++){
                    for(k = 0; k < LED_PORTS; k++){
                        if((j >= led_area_starty)&&(j < (led_area_starty + led_area_height))){
                            if(j % 2 == 0){
                                if((i >= led_area_startx)&&(i < (led_area_startx + led_area_width))){
                                    test_pattern = led_patterns[k][((j*led_total_width) + i)][0] << 16 |  //g
                                                led_patterns[k][((j*led_total_width) + i)][1] << 8 |  //r
                                                led_patterns[k][((j*led_total_width) + i)][2] ;       //b
                                }else{
                                    test_pattern = 0x000000;
                                }
                            }else{
                                if((i >= (led_total_width - led_area_startx-led_area_width))
                                        &&(i < (led_total_width - led_area_startx))){
                                    test_pattern = led_patterns[k][((j*led_total_width) + i)][0] << 16 |  //g
                                                led_patterns[k][((j*led_total_width) + i)][1] << 8 |  //r
                                                led_patterns[k][((j*led_total_width) + i)][2] ;       //b
                                }else{
                                    test_pattern = 0x000000;
                                }

                            }
                        }else{
                            test_pattern = 0x000000;
                        
                        }                       
                             
                            /*test_pattern = led_patterns[k][((j*led_total_width) + i)][0] << 16 |  //g
                                            led_patterns[k][((j*led_total_width) + i)][1] << 8 |  //r
                                            led_patterns[k][((j*led_total_width) + i)][2] ;       //b*/

	    	            put_pixel_by_panel(k, test_pattern);

                    }
	            }
	        }        
	        sleep_ms(10);
        }
    }

    return 0;
}
