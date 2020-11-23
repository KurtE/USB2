/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
//WMXZ: cloned from rawhid and modified for mtp

#include "avr/pgmspace.h" // for PROGMEM, DMAMEM, FASTRUN
#include "core_pins.h" // for yield(), millis()
#include <string.h>    // for memcpy()

#include "usb_dev.h"
#include "usb1_mtp.h"
//#include "HardwareSerial.h"

#define printf(...) //Serial.printf(__VA_ARGS__)

//#include "debug/printf.h"

#if defined(USB1_TEST) && defined(__IMXRT1062__) && defined(MTP_INTERFACE)

#define TX_NUM   4
static transfer_t tx_transfer[TX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t txbuffer[MTP_TX_SIZE * TX_NUM];
static uint8_t tx_head = 0;

#define RX_NUM 8
static transfer_t rx_transfer[RX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[MTP_RX_SIZE * RX_NUM];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static uint8_t rx_list[RX_NUM + 1];

extern volatile uint8_t usb_configuration;

static uint32_t mtp_TXcount = 0;
//static uint32_t mtp_RXcount = 0;

static void rx_queue_transfer(int i);
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t) {mtp_TXcount++;}
static void rx_event_event(transfer_t *t);

// Events end point
static transfer_t tx_event_transfer[1] __attribute__ ((used, aligned(32)));
static uint8_t tx_event_buffer[MTP_EVENT_SIZE] __attribute__ ((used, aligned(32)));

static transfer_t rx_event_transfer[1] __attribute__ ((used, aligned(32)));
static uint8_t rx_event_buffer[MTP_EVENT_SIZE] __attribute__ ((used, aligned(32)));



void usb_mtp_configure(void)
{
	digitalWriteFast(33, HIGH);

	printf("usb_mtp_configure\n");
	memset(tx_transfer, 0, sizeof(tx_transfer));
	memset(rx_transfer, 0, sizeof(rx_transfer));
	memset(rx_event_transfer, 0, sizeof(rx_event_transfer));
	//memset(tx_event_transfer, 0, sizeof(tx_event_transfer));
	tx_head = 0;
	rx_head = 0;
	rx_tail = 0;
	usb_config_tx(MTP_TX_ENDPOINT, MTP_TX_SIZE, 0, tx_event);
	usb_config_rx(MTP_RX_ENDPOINT, MTP_RX_SIZE, 0, rx_event);

	// Not sure if you do two calls here for this type? .
	usb_config_rx(MTP_EVENT_ENDPOINT, MTP_EVENT_SIZE, 0, rx_event_event);
	//usb_config_tx(MTP_EVENT_ENDPOINT, MTP_EVENT_SIZE, 0, tx_event_event);

	//usb_config_rx(MTP_RX_ENDPOINT, MTP_RX_SIZE, 0, NULL); // why does this not work?
	int i;
	for (i = 0; i < RX_NUM; i++) rx_queue_transfer(i);
	digitalWriteFast(33, LOW);

	// Lets do eents
	usb_prepare_transfer(rx_event_transfer + 0, rx_event_buffer, MTP_EVENT_SIZE, MTP_EVENT_ENDPOINT);
	usb_receive(MTP_EVENT_ENDPOINT, rx_event_transfer + 0);

}

/*************************************************************************/
/**                               Receive                               **/
/*************************************************************************/

static void rx_queue_transfer(int i)
{
	void *buffer = rx_buffer + i * MTP_RX_SIZE;
	arm_dcache_delete(buffer, MTP_RX_SIZE);
	//memset(buffer, )
	NVIC_DISABLE_IRQ(IRQ_USB1);
	usb_prepare_transfer(rx_transfer + i, buffer, MTP_RX_SIZE, i);
	usb_receive(MTP_RX_ENDPOINT, rx_transfer + i);
	NVIC_ENABLE_IRQ(IRQ_USB1);
}

static void rx_event(transfer_t *t)
{
	digitalWriteFast(34, HIGH);
	int i = t->callback_param;
	//printf("rx event i=%d\n", i);
	// received a packet with data
	uint32_t head = rx_head;
	if (++head > RX_NUM) head = 0;
	rx_list[head] = i;
	rx_head = head;
	digitalWriteFast(34, LOW);
}

int usb_mtp_recv(void *buffer, uint32_t timeout)
{
	digitalWriteFast(35, HIGH);
	uint32_t wait_begin_at = systick_millis_count;
	uint32_t tail = rx_tail;

	while (1) {
		if (!usb_configuration) {
			digitalToggleFast(39);
			digitalWriteFast(35, LOW);
			return -1; // usb not enumerated by host
		}
		if (tail != rx_head) break;
		if (systick_millis_count - wait_begin_at > timeout)  {
			digitalToggleFast(39);
			digitalWriteFast(35, LOW);
			return 0;
		}
		yield();
	}

	if (++tail > RX_NUM) tail = 0;
	uint32_t i = rx_list[tail];
	rx_tail = tail;

	memcpy(buffer,  rx_buffer + i * MTP_RX_SIZE, MTP_RX_SIZE);
	rx_queue_transfer(i);

	digitalWriteFast(35, LOW);
	return MTP_RX_SIZE;
}

static int mtp_xfer_wait(transfer_t *xfer,uint32_t timeout)
{
	uint32_t wait_begin_at = systick_millis_count;
	while (1) {
		if (!usb_configuration) {
			digitalToggleFast(39);
			return -1; // usb not enumerated by host
		}
		uint32_t status = usb_transfer_status(xfer);
		if (!(status & 0x80)) break; // transfer descriptor ready
		if (systick_millis_count - wait_begin_at > timeout) {
			digitalToggleFast(39);
			return 0;
		}
		yield();
	}
	return 1;
}

int usb_mtp_send(const void *buffer,  int len, uint32_t timeout)
{
	digitalWriteFast(36, HIGH);
	transfer_t *xfer = tx_transfer + tx_head;

    int ret;
	if((ret=mtp_xfer_wait(xfer,timeout)) <= 0) {
		digitalWriteFast(36, LOW);
		return ret;
	}
	uint8_t *txdata = txbuffer + (tx_head * MTP_TX_SIZE);
	memcpy(txdata, buffer, len);
	arm_dcache_flush_delete(txdata, len );
	usb_prepare_transfer(xfer, txdata, len, 0);
	usb_transmit(MTP_TX_ENDPOINT, xfer);
	if (++tx_head >= TX_NUM) tx_head = 0;
	digitalWriteFast(36, LOW);
	return len;
}

int usb_mtp_available(void)
{
	if (!usb_configuration) return 0;
	if (rx_head != rx_tail) return MTP_RX_SIZE;
	return 0;
}

/*************************************************************************/
/**                               Event                                 **/
/*************************************************************************/
extern void usb2_phex(uint8_t n);
extern void usb2_print(const char *psz);

void rx_event_event(transfer_t *t) {
	// bugbug: not sure if any correct way to get pointer to data and size
	// so just dump our one...
	usb2_print("rx_event_event");
	for (uint16_t i=0; i < MTP_EVENT_SIZE; i++) {
		usb2_phex(rx_event_buffer[i]);
		if ((i & 0xf ) == 0xf) usb2_print("\n");
	}
	usb2_print("\n");
}

int usb_mtp_eventSend(const void *buffer, uint32_t timeout)
{
	transfer_t *xfer = tx_event_transfer;

    int ret;
	if((ret=mtp_xfer_wait(xfer,timeout)) <= 0) return ret;

	uint8_t *txdata = tx_event_buffer;
	memcpy(txdata, buffer, MTP_EVENT_SIZE);
	usb_prepare_transfer(xfer, txdata, MTP_EVENT_SIZE,  MTP_EVENT_ENDPOINT);
	usb_transmit(MTP_EVENT_ENDPOINT, xfer);
	return MTP_EVENT_SIZE;
}

int usb_mtp_eventRecv(void *buffer, uint32_t timeout)
{
	transfer_t *xfer = rx_event_transfer;

    int ret;
	if((ret=mtp_xfer_wait(xfer,timeout)) <= 0) return ret;

	memcpy(buffer, rx_event_buffer, MTP_EVENT_SIZE);
	memset(xfer, 0, sizeof(rx_event_transfer));
	usb_prepare_transfer(xfer, rx_event_buffer, MTP_EVENT_SIZE, MTP_EVENT_ENDPOINT);
	usb_receive(MTP_EVENT_ENDPOINT, xfer);

	return MTP_EVENT_SIZE;
}


#else
void usb_mtp_configure(void) {}
#endif // MTP_INTERFACE
