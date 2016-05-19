/*
  * Copyrigth (c) 2015 Mandl
 *  Copyright (c) 2014 Sean Cross
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 *
 */

#ifndef HW_MT62XX_UART_H
#define HW_MT62XX_UART_H

#include "hw/sysbus.h"
#include "qemu/typedefs.h"

#define TYPE_mt6262_UART "mt62xx-uart"
#define mt6262_UART(obj) \
        OBJECT_CHECK(mt6262UARTState, (obj), TYPE_mt6262_UART)


typedef struct mt6262UARTState {
	SysBusDevice parent_obj;

	MemoryRegion iomem;
	CharDriverState *chr;
	qemu_irq irq;

	uint8_t utcr0;
	uint16_t brd;
	uint8_t utcr3;
	uint8_t utsr0;
	uint8_t utsr1;

	uint8_t tx_fifo[8];
	uint8_t tx_start;
	uint8_t tx_len;
	uint16_t rx_fifo[12]; /* value + error flags in high bits */
	uint8_t rx_start;
	uint8_t rx_len;

	uint64_t char_transmit_time; /* time to transmit a char in ticks*/
	bool wait_break_end;
	QEMUTimer *rx_timeout_timer;
	QEMUTimer *tx_timer;

	uint32_t RBR;
	uint32_t THR;
	uint32_t IER;
	uint32_t IIR;
	uint32_t FCR;
	uint32_t LCR;
	uint32_t MCR;
	uint32_t LSR;
	uint32_t MSR;
	uint32_t SCR;

	uint32_t SPEED;

	/* The following are active when LCR[7] = 1 */
	uint32_t DLL;
	uint32_t DLH;

	/* The following are active when LCR = 0xbf */
	uint32_t EFR;
	uint32_t XON1;
	uint32_t XON2;
	uint32_t XOFF1;
	uint32_t XOFF2;
} mt6262UARTState;



#endif /* HW_MT62XX_UART_H */
