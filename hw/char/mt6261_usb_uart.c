/*
 *  QEMU model of the LatticeMico32 UART block.
 *
 *  Copyright (c) 2010 Michael Walle <michael@walle.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Specification available at:
 *   http://www.latticesemi.com/documents/mico32uart.pdf
 */


#include "hw/hw.h"
#include "hw/sysbus.h"
#include "trace.h"
#include "sysemu/char.h"
#include "qemu/error-report.h"

enum {
    R_RXTX = 0,
    R_IER,
    R_IIR,
    R_LCR,
    R_MCR,
    R_LSR,
    R_MSR,
    R_DIV,
    R_MAX
};

enum {
    IER_RBRI = (1<<0),
    IER_THRI = (1<<1),
    IER_RLSI = (1<<2),
    IER_MSI  = (1<<3),
};

enum {
    IIR_STAT = (1<<0),
    IIR_ID0  = (1<<1),
    IIR_ID1  = (1<<2),
};

enum {
    LCR_WLS0 = (1<<0),
    LCR_WLS1 = (1<<1),
    LCR_STB  = (1<<2),
    LCR_PEN  = (1<<3),
    LCR_EPS  = (1<<4),
    LCR_SP   = (1<<5),
    LCR_SB   = (1<<6),
};

enum {
    MCR_DTR  = (1<<0),
    MCR_RTS  = (1<<1),
};

enum {
    LSR_DR   = (1<<0),
    LSR_OE   = (1<<1),
    LSR_PE   = (1<<2),
    LSR_FE   = (1<<3),
    LSR_BI   = (1<<4),
    LSR_THRE = (1<<5),
    LSR_TEMT = (1<<6),
};

enum {
    MSR_DCTS = (1<<0),
    MSR_DDSR = (1<<1),
    MSR_TERI = (1<<2),
    MSR_DDCD = (1<<3),
    MSR_CTS  = (1<<4),
    MSR_DSR  = (1<<5),
    MSR_RI   = (1<<6),
    MSR_DCD  = (1<<7),
};

#define TYPE_mt6261_usb_uart "mt6261_usb_uart"
#define mt6261_usb_uart(obj) OBJECT_CHECK(mt6261_usb_uartState, (obj), TYPE_mt6261_usb_uart)

struct mt6261_usb_uartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharDriverState *chr;
    qemu_irq irq;

    uint32_t regs[R_MAX];
};
typedef struct mt6261_usb_uartState mt6261_usb_uartState;

static void uart_update_irq(mt6261_usb_uartState *s)
{
    unsigned int irq;

    if ((s->regs[R_LSR] & (LSR_OE | LSR_PE | LSR_FE | LSR_BI))
            && (s->regs[R_IER] & IER_RLSI)) {
        irq = 1;
        s->regs[R_IIR] = IIR_ID1 | IIR_ID0;
    } else if ((s->regs[R_LSR] & LSR_DR) && (s->regs[R_IER] & IER_RBRI)) {
        irq = 1;
        s->regs[R_IIR] = IIR_ID1;
    } else if ((s->regs[R_LSR] & LSR_THRE) && (s->regs[R_IER] & IER_THRI)) {
        irq = 1;
        s->regs[R_IIR] = IIR_ID0;
    } else if ((s->regs[R_MSR] & 0x0f) && (s->regs[R_IER] & IER_MSI)) {
        irq = 1;
        s->regs[R_IIR] = 0;
    } else {
        irq = 0;
        s->regs[R_IIR] = IIR_STAT;
    }

    //trace_6261_usb_uart_irq_state(irq);
    qemu_set_irq(s->irq, irq);
}

static uint64_t uart_read(void *opaque, hwaddr addr,
                          unsigned size)
{
    mt6261_usb_uartState *s = opaque;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr) {
    case R_RXTX:
        r = s->regs[R_RXTX];
        s->regs[R_LSR] &= ~LSR_DR;
        uart_update_irq(s);
        qemu_chr_accept_input(s->chr);
        break;
    case R_IIR:
    case R_LSR:
    case R_MSR:
        r = s->regs[addr];
        break;
    case R_IER:
    case R_LCR:
    case R_MCR:
    case R_DIV:
        error_report("6261_usb_uart: read access to write only register 0x"
                TARGET_FMT_plx, addr << 2);
        break;
    default:
        error_report("6261_usb_uart: read access to unknown register 0x"
                TARGET_FMT_plx, addr << 2);
        break;
    }

    //trace_6261_usb_uart_memory_read(addr << 2, r);
    return r;
}

static void uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned size)
{
    mt6261_usb_uartState *s = opaque;
    unsigned char ch = value;

    //trace_6261_usb_uart_memory_write(addr, value);

    addr >>= 2;
    switch (addr) {
    case R_RXTX:
        if (s->chr) {
            qemu_chr_fe_write_all(s->chr, &ch, 1);
        }
        break;
    case R_IER:
    case R_LCR:
    case R_MCR:
    case R_DIV:
        s->regs[addr] = value;
        break;
    case R_IIR:
    case R_LSR:
    case R_MSR:
        error_report("6261_usb_uart: write access to read only register 0x"
                TARGET_FMT_plx, addr << 2);
        break;
    default:
        error_report("6261_usb_uart: write access to unknown register 0x"
                TARGET_FMT_plx, addr << 2);
        break;
    }
    uart_update_irq(s);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void uart_rx(void *opaque, const uint8_t *buf, int size)
{
    mt6261_usb_uartState *s = opaque;

    if (s->regs[R_LSR] & LSR_DR) {
        s->regs[R_LSR] |= LSR_OE;
    }

    s->regs[R_LSR] |= LSR_DR;
    s->regs[R_RXTX] = *buf;

    uart_update_irq(s);
}

static int uart_can_rx(void *opaque)
{
    mt6261_usb_uartState *s = opaque;

    return !(s->regs[R_LSR] & LSR_DR);
}

static void uart_event(void *opaque, int event)
{
}

static void uart_reset(DeviceState *d)
{
    mt6261_usb_uartState *s = mt6261_usb_uart(d);
    int i;

    for (i = 0; i < R_MAX; i++) {
        s->regs[i] = 0;
    }

    /* defaults */
    s->regs[R_LSR] = LSR_THRE | LSR_TEMT;
}

static int mt6261_usb_uart_init(SysBusDevice *dev)
{
    mt6261_usb_uartState *s = mt6261_usb_uart(dev);

    sysbus_init_irq(dev, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &uart_ops, s,
                          "uart", R_MAX * 4);
    sysbus_init_mmio(dev, &s->iomem);

    s->chr = qemu_char_get_next_serial();
    if (s->chr) {
        qemu_chr_add_handlers(s->chr, uart_can_rx, uart_rx, uart_event, s);
    }

    return 0;
}

static const VMStateDescription vmstate_mt6261_usb_uart = {
    .name = "lm32-uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, mt6261_usb_uartState, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void mt6261_usb_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = mt6261_usb_uart_init;
    dc->reset = uart_reset;
    dc->vmsd = &vmstate_mt6261_usb_uart;
}

static const TypeInfo mt6261_usb_uart_info = {
    .name          = TYPE_mt6261_usb_uart,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(mt6261_usb_uartState),
    .class_init    = mt6261_usb_uart_class_init,
};

static void mt6261_usb_uart_register_types(void)
{
    type_register_static(&mt6261_usb_uart_info);
}

type_init(mt6261_usb_uart_register_types)
