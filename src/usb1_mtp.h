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
#pragma once

#include "usb_desc.h"

#define USB1_TEST
#if defined(USB1_TEST) && defined(__IMXRT1062__) && defined(MTP_INTERFACE)

#define MTP_TX_SIZE MTP_TX_SIZE_480
#define MTP_RX_SIZE MTP_RX_SIZE_480

#include <inttypes.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
int usb_mtp_haveRX(void);
int usb_mtp_canTX(void);

void usb_mtp_configure(void);
int usb_mtp_recv(void *buffer, uint32_t timeout);
int usb_mtp_send(const void *buffer, int len, uint32_t timeout);
int usb_mtp_available(void);

int usb_mtp_recvEevent(const void *buffer, uint32_t len, uint32_t timeout);
int usb_mtp_sendEevent(const void *buffer, uint32_t len, uint32_t timeout);

#ifdef __cplusplus
}
#endif


// C++ interface
#ifdef __cplusplus
class usb1_mtp_class
{
public:
	int available(void) {return usb_mtp_available(); }
//	int read(void *buffer, uint16_t timeout) { return usb_mtp_read(buffer, timeout); }
	int recv(void *buffer, uint16_t timeout) { return usb_mtp_recv(buffer, timeout); }
	int send(const void *buffer, int len, uint16_t timeout) { return usb_mtp_send(buffer, len, timeout); }
//	int sendEvent(const void *buffer, uint16_t timeout) { return usb_mtp_eventSend(buffer, timeout); }
//	int recvEvent(void *buffer, uint16_t timeout) { return usb_mtp_eventRecv(buffer, timeout); }
};

//extern usb1_mtp_class MTP;

#endif // __cplusplus

#else
#ifdef __cplusplus
extern "C" {
#endif
void usb_mtp_configure(void);
#ifdef __cplusplus
}
#endif

#endif // MTP_INTERFACE
