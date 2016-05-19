/*
 * Copyrigth (c) 2015 Mandl
 * Copyright (c) 2014 Sean Cross
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "trace.h"
#include "sysemu/char.h"
#include "qom/object.h"
#include "hw/char/mt62xx_uart.h"

/* UART Ports */
#define UART_RBR 0x00
#define UART_THR 0x00
#define UART_IER 0x04
#define UART_IIR 0x08
#define UART_FCR 0x08
#define UART_LCR 0x0c
#define UART_MCR 0x10
#define UART_LSR 0x14
#define UART_MSR 0x18
#define UART_SCR 0x1c

#define UART_AUTOBAUD_EN = 0x20
//#define UART_HIGHSPEED = 0x24
#define UART_SAMPLE_COUNT = 0x28
#define UART_SAMPLE_POINT = 0x2c
#define UART_AUTOBAUD_REG = 0x30
#define UART_RATE_FIX_REG = 0x34
#define UART_AUTOBAUDSAMPLE = 0x38
#define UART_GUARD = 0x3c
#define UART_ESCAPE_DAT = 0x40
#define UART_ESCAPE_EN = 0x44
#define UART_SLEEP_EN = 0x48
#define UART_VFIFO_EN = 0x4c
#define UART_RXTRIG = 0x50

#define UART_SPEED 0x24

/* The following are active when LCR[7] = 1 */
#define UART_DLL 0x100
#define UART_DLH 0x104

/* The following are active when LCR = 0xbf */
#define UART_EFR   0x208
#define UART_XON1  0x210
#define UART_XON2  0x214
#define UART_XOFF1 0x218
#define UART_XOFF2 0x21c

#define UTCR0 0x00
#define UTCR1 0x04
#define UTCR2 0x08
#define UTCR3 0x0c
#define UTDR  0x14
#define UTSR0 0x1c
#define UTSR1 0x20

#define UTCR0_PE  (1 << 0) /* Parity enable */
#define UTCR0_OES (1 << 1) /* Even parity */
#define UTCR0_SBS (1 << 2) /* 2 stop bits */
#define UTCR0_DSS (1 << 3) /* 8-bit data */

#define UTCR3_RXE (1 << 0) /* Rx enable */
#define UTCR3_TXE (1 << 1) /* Tx enable */
#define UTCR3_BRK (1 << 2) /* Force Break */
#define UTCR3_RIE (1 << 3) /* Rx int enable */
#define UTCR3_TIE (1 << 4) /* Tx int enable */
#define UTCR3_LBM (1 << 5) /* Loopback */

#define UTSR0_TFS (1 << 0) /* Tx FIFO nearly empty */
#define UTSR0_RFS (1 << 1) /* Rx FIFO nearly full */
#define UTSR0_RID (1 << 2) /* Receiver Idle */
#define UTSR0_RBB (1 << 3) /* Receiver begin break */
#define UTSR0_REB (1 << 4) /* Receiver end break */
#define UTSR0_EIF (1 << 5) /* Error in FIFO */

#define UTSR1_RNE (1 << 0) /* Receive FIFO not empty */
#define UTSR1_TNF (1 << 5) /* Transmit FIFO not full */
#define UTSR1_PRE (1 << 3) /* Parity error */
#define UTSR1_FRE (1 << 4) /* Frame error */
//#define UTSR1_ROR (1 << 5) /* Receive Over Run */

#define RX_FIFO_PRE (1 << 8)
#define RX_FIFO_FRE (1 << 9)
#define RX_FIFO_ROR (1 << 10)






static void mt6262_uart_update_status(mt6262UARTState *s) {
	uint16_t utsr1 = 0;

	if (s->tx_len != 8) {
		utsr1 |= UTSR1_TNF;
	}

	if (s->rx_len != 0) {
		uint16_t ent = s->rx_fifo[s->rx_start];

		utsr1 |= UTSR1_RNE;
		if (ent & RX_FIFO_PRE) {
			s->utsr1 |= UTSR1_PRE;
		}
		if (ent & RX_FIFO_FRE) {
			s->utsr1 |= UTSR1_FRE;
		}
		//if (ent & RX_FIFO_ROR) {
		//	s->utsr1 |= UTSR1_ROR;
		//}
	}

	s->utsr1 = utsr1;
}

static void mt6262_uart_update_int_status(mt6262UARTState *s) {
	uint16_t utsr0 = s->utsr0 & (UTSR0_REB | UTSR0_RBB | UTSR0_RID);
	int i;

	if ((s->utcr3 & UTCR3_TXE) && (s->utcr3 & UTCR3_TIE) && s->tx_len <= 4) {
		utsr0 |= UTSR0_TFS;
	}

	if ((s->utcr3 & UTCR3_RXE) && (s->utcr3 & UTCR3_RIE) && s->rx_len > 4) {
		utsr0 |= UTSR0_RFS;
	}

	for (i = 0; i < s->rx_len && i < 4; i++)
		if (s->rx_fifo[(s->rx_start + i) % 12] & ~0xff) {
			utsr0 |= UTSR0_EIF;
			break;
		}

	s->utsr0 = utsr0;
	qemu_set_irq(s->irq, utsr0);
}

static void mt6262_uart_rx_to(void *opaque) {
	mt6262UARTState *s = opaque;

	if (s->rx_len) {
		s->utsr0 |= UTSR0_RID;
		mt6262_uart_update_int_status(s);
	}
}

static void mt6262_uart_rx_push(mt6262UARTState *s, uint16_t c) {
	if ((s->utcr3 & UTCR3_RXE) == 0) {
		/* rx disabled */
		return;
	}

	if (s->wait_break_end) {
		s->utsr0 |= UTSR0_REB;
		s->wait_break_end = false;
	}

	if (s->rx_len < 12) {
		s->rx_fifo[(s->rx_start + s->rx_len) % 12] = c;
		s->rx_len++;
	} else
		s->rx_fifo[(s->rx_start + 11) % 12] |= RX_FIFO_ROR;
}

static int mt6262_uart_can_receive(void *opaque) {
	mt6262UARTState *s = opaque;

	if (s->rx_len == 12) {
		return 0;
	}
	/* It's best not to get more than 2/3 of RX FIFO, so advertise that much */
	if (s->rx_len < 8) {
		return 8 - s->rx_len;
	}
	return 1;
}

static void mt6262_uart_receive(void *opaque, const uint8_t *buf, int size) {
	mt6262UARTState *s = opaque;
	int i;
	//printf("get char");

	for (i = 0; i < size; i++) {
		mt6262_uart_rx_push(s, buf[i]);
	}

	/* call the timeout receive callback in 3 char transmit time */
	timer_mod(s->rx_timeout_timer,
			qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 3);

	mt6262_uart_update_status(s);
	mt6262_uart_update_int_status(s);
}

static void mt6262_uart_event(void *opaque, int event) {
	mt6262UARTState *s = opaque;
	if (event == CHR_EVENT_BREAK) {
		s->utsr0 |= UTSR0_RBB;
		mt6262_uart_rx_push(s, RX_FIFO_FRE);
		s->wait_break_end = true;
		mt6262_uart_update_status(s);
		mt6262_uart_update_int_status(s);
	}
}

static void mt6262_uart_tx(void *opaque) {
	mt6262UARTState *s = opaque;
	uint64_t new_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

	if (s->utcr3 & UTCR3_LBM) /* loopback */
	{
		mt6262_uart_receive(s, &s->tx_fifo[s->tx_start], 1);
	} else if (s->chr) {
		qemu_chr_fe_write(s->chr, &s->tx_fifo[s->tx_start], 1);
	}

	s->tx_start = (s->tx_start + 1) % 8;
	s->tx_len--;
	if (s->tx_len) {
		timer_mod(s->tx_timer, new_xmit_ts + s->char_transmit_time);
	}
	mt6262_uart_update_status(s);
	mt6262_uart_update_int_status(s);
}

static uint64_t mt6262_uart_read(void *opaque, hwaddr addr, unsigned size) {
	mt6262UARTState *s = opaque;
	uint16_t ret;

	switch (addr) {

	case UART_RBR:
		if (s->rx_len != 0) {
			ret = s->rx_fifo[s->rx_start];
			s->rx_start = (s->rx_start + 1) % 12;
			s->rx_len--;
			mt6262_uart_update_status(s);
			mt6262_uart_update_int_status(s);
			return ret;
		}
		return 0;
	case UART_DLL:
		if (s->LCR & (1 << 7))
			return s->DLL;
		return '\0';

	case UART_IER:
		if (s->LCR & (1 << 7))
			return s->DLH;
		return s->IER;

	case UART_IIR:
	case UART_EFR:
		if (s->LCR == 0xbf)
			return s->EFR;
		return s->IIR;

	case UART_MSR:
	case UART_XOFF1:
		if (s->LCR == 0xbf)
			return s->XOFF1;
		return s->MSR;

	case UART_LSR:
	case UART_XON2:
		if (s->LCR == 0xbf)
			return s->XON2;
		return s->utsr1;

	case UART_MCR:
	case UART_XON1:
		if (s->LCR == 0xbf)
			return s->XON1;
		return s->MCR;

	case UART_SCR:
	case UART_XOFF2:
		if (s->LCR == 0xbf)
			return s->XOFF2;
		return s->SCR;

	default:
		printf("%s: Bad register 0x" TARGET_FMT_plx "\n", __func__, addr);
		return 0;
	}
}

static void mt6262_uart_write(void *opaque, hwaddr addr, uint64_t value,
		unsigned size) {

	mt6262UARTState *s = opaque;

	switch (addr) {
	case UART_RBR:
		if (s->LCR & (1 << 7))
			s->DLL = value;
		else

		if (s->tx_len != 8) {
			s->tx_fifo[(s->tx_start + s->tx_len) % 8] = value;
			s->tx_len++;
			mt6262_uart_update_status(s);
			mt6262_uart_update_int_status(s);
			if (s->tx_len == 1) {
				mt6262_uart_tx(s);
			}
		}

		break;

	case UART_IER:
	case UART_DLH:
		if (s->LCR & (1 << 7))
			s->DLH = value;
		else
			s->IER = value;
		break;

	case UART_MCR:
	case UART_XON1:
		if (s->LCR == 0xbf)
			s->XON1 = value;
		else
			s->MCR = value;

	case UART_LCR:
		s->LCR = value;
		break;

	case UART_IIR:
	case UART_EFR:
		if (s->LCR == 0xbf)
			s->EFR = value;
		else
			s->IIR = value;
		break;

	case UART_MSR:
	case UART_XOFF1:
		if (s->LCR == 0xbf)
			s->XOFF1 = value;
		else
			s->MSR = value;
		break;

	case UART_SCR:
	case UART_XOFF2:
		if (s->LCR == 0xbf)
			s->XOFF2 = value;
		else
			s->SCR = value;
		break;

	case UART_XON2:
	case UART_LSR:
		if (s->LCR == 0xbf)
			s->XON2 = value;
		else
			s->LSR = value;
		break;

	default:
		printf("%s: Bad register 0x%04x -> 0x%04x\n", __func__, (int) addr,
				(int) value);
		break;
	}
}

static const MemoryRegionOps mt6262_uart_ops = { .read = mt6262_uart_read,
		.write = mt6262_uart_write, .endianness = DEVICE_NATIVE_ENDIAN, };

static int mt6262_uart_init(SysBusDevice *dev) {
	mt6262UARTState *s = mt6262_UART(dev);

	memory_region_init_io(&s->iomem, OBJECT(s), &mt6262_uart_ops, s, "uart",
			0x10000);
	sysbus_init_mmio(dev, &s->iomem);
	sysbus_init_irq(dev, &s->irq);

	s->rx_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mt6262_uart_rx_to,
			s);
	s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mt6262_uart_tx, s);

	if (s->chr) {
		qemu_chr_add_handlers(s->chr, mt6262_uart_can_receive,
				mt6262_uart_receive, mt6262_uart_event, s);
	}

	return 0;
}

static void mt6262_uart_reset(DeviceState *dev) {
	mt6262UARTState *s = mt6262_UART(dev);

	s->utcr0 = UTCR0_DSS; /* 8 data, no parity */
	s->brd = 23; /* 9600 */
	/* enable send & recv - this actually violates spec */
	s->utcr3 = UTCR3_TXE | UTCR3_RXE;

	s->rx_len = s->tx_len = 0;

	//mt6262_uart_update_parameters(s);
	mt6262_uart_update_status(s);
	mt6262_uart_update_int_status(s);
}

static int mt6262_uart_post_load(void *opaque, int version_id) {
	mt6262UARTState *s = opaque;

	//mt6262_uart_update_parameters(s);
	mt6262_uart_update_status(s);
	mt6262_uart_update_int_status(s);

	/* tx and restart timer */
	if (s->tx_len) {
		mt6262_uart_tx(s);
	}

	/* restart rx timeout timer */
	if (s->rx_len) {
		timer_mod(s->rx_timeout_timer,
				qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
						+ s->char_transmit_time * 3);
	}

	return 0;
}

static const VMStateDescription vmstate_mt6262_uart_regs = { .name =
		"mt6262-uart", .version_id = 0, .minimum_version_id = 0,
		.minimum_version_id_old = 0, .post_load = mt6262_uart_post_load,
		.fields = (VMStateField[] )
				{
				VMSTATE_UINT8(utcr0, mt6262UARTState),
				VMSTATE_UINT16(brd, mt6262UARTState),
				VMSTATE_UINT8(utcr3, mt6262UARTState),
				VMSTATE_UINT8(utsr0, mt6262UARTState),
				VMSTATE_UINT8_ARRAY(tx_fifo, mt6262UARTState, 8),
				VMSTATE_UINT8(tx_start, mt6262UARTState),
				VMSTATE_UINT8(tx_len, mt6262UARTState),
				VMSTATE_UINT16_ARRAY(rx_fifo, mt6262UARTState, 12),
				VMSTATE_UINT8(rx_start, mt6262UARTState),
				VMSTATE_UINT8(rx_len, mt6262UARTState),
				VMSTATE_BOOL(wait_break_end, mt6262UARTState),
				VMSTATE_END_OF_LIST(), } , };

static Property mt6262_uart_properties[] = {
DEFINE_PROP_CHR("chardev", mt6262UARTState, chr),
DEFINE_PROP_END_OF_LIST(), };

static void mt6262_uart_class_init(ObjectClass *klass, void *data) {
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

	k->init = mt6262_uart_init;
	dc->desc = "mt6262 UART controller";
	dc->reset = mt6262_uart_reset;
	dc->vmsd = &vmstate_mt6262_uart_regs;
	dc->props = mt6262_uart_properties;
}

static const TypeInfo mt6262_uart_info = { .name = TYPE_mt6262_UART, .parent =
TYPE_SYS_BUS_DEVICE, .instance_size = sizeof(mt6262UARTState), .class_init =
		mt6262_uart_class_init, };

static void mt6262_register_types(void) {
	type_register_static(&mt6262_uart_info);

}

type_init(mt6262_register_types)
