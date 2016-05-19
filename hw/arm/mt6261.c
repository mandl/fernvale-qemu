/*
 * mt6262 emulation
 * Copyrigth (c) 2015 Mandl
 * Copyright (c) 2014 Sean Cross
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

/*
 *  ./arm-softmmu/qemu-system-arm -machine mt6262 -serial telnet:localhost:1235,server
 *
 *  ./arm-softmmu/qemu-system-arm -machine mt6262 -serial stdio -s -S
 *
 */

#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "hw/boards.h"
#include "hw/char/serial.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/loader.h"
#include "sysemu/blockdev.h"
#include "hw/block/flash.h"
#include "exec/address-spaces.h"
#include <sys/stat.h>
#include "sysemu/char.h"
#include <termios.h>
#include "hw/char/mt62xx_uart.h"

//#define FERNLY_IO_DEBUG
#define PRINT_NORMAL 1
#define mt6262_DEBUG_UART "/dev/ttyUSB0"

#define mt6262_READY_CHAR 'k'
static int mt6262_fd;

// ROM Files
#define IRAM_FILE "INT_SYSRAM"
#define PSRAM_FILE "EXT_RAM"

#define BOOTROM_FILE "boot_rom.bin"
#define FIRMROM_FILE "firmware.bin"

#define MY_ROM_FILE "/home/mandl/Entwicklung/mt6223_blink/main.bin"

//#define BOOT_ROM
//#define DA_TOOL
#define MY_ROM_PS_RAM

//#define PROGRAM_START 0x70007000

//#define PROGRAM_START 0xfff00000

#define PROGRAM_START 0x10020000

// Internal RAM mt6261
#define FV_IRAM_SIZE 8* 1024 * 128   // 64k
#define FV_IRAM_BASE 0x70000000
#define IRAM_FILE_OFFSET 0x7000

#define MT6223_UART_BASE 0xa0080000

// External PSRAM
#define FV_PSRAM_SIZE 8 * 1024 * 1024
#define FV_PSRAM_BASE 0x10020000

// Boot Rom
#define FV_BOOTROM_SIZE 8 * 1024 * 64
#define FV_BOOTROM_BASE 0xfff00000

// Firmware Rom
#define FV_FIRMROM_SIZE 8 * 1024 * 64 * 2
#define FV_FIRMROM_BASE 0x00000000

static void *iram;
static void *psram;
static void *bootrom;
//static void *firmrom;

static uint32_t IRQ_MASK = 0xFFFFFFFF;

static uint32_t IRQ_Sensitive_Register;

static uint32_t EINT_MASK = 0x00;
static uint32_t EINT_Status = 0x00;

static void mt6262_cpu_reset(void *opaque) {
	ARMCPU *cpu = opaque;
	CPUARMState *env = &cpu->env;

	cpu_reset(CPU(cpu));

	/* Place the PC at the start of the main program */

	env->regs[15] = PROGRAM_START;

}

static int my_getresponse(int fd, char *buf, int len) {
	int offset = 0;
	ssize_t ret;
	uint8_t byte;
	fd_set fds;
	struct timeval tv;

#ifdef FERNLY_IO_DEBUG
	fprintf(stderr, "[[ reading : ");
#endif

	while (len > 1) {

		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		ret = select(fd + 1, &fds, NULL, NULL, &tv);
		if (ret == -1) {
			if (errno == EINTR) {
//                fprintf(stderr, " [interrupted select - retrying] ");
				continue;
			}
			perror("Couldn't select");
			return -1;
		}
		if (!ret) {
			fprintf(stderr, " [no response - board crashed?] ");
			return -1;
		}

		ret = read(fd, &byte, 1);

		if (-1 == ret) {
			if (errno != EINTR) {
				perror("couldn't read");
				return ret;
			}
//            fprintf(stderr, " [interrupted read - retrying] ");
			continue;
		}

		if (byte == mt6262_READY_CHAR) {
			buf[offset++] = '\0';
#ifdef FERNLY_IO_DEBUG
			fprintf(stderr, " read line: (%s)]]\n", buf);
#endif
			if (strstr(buf, "Handled IRQ")) {
				printf(">>> HANDLED IRQ <<<\n");
			}
			if (strstr(buf, "Handled FIQ")) {
				printf(">>> HANDLED IRQ <<<\n");
			}

			return offset;
		}

		else
			buf[offset++] = byte;
		len--;
	}

#ifdef FERNLY_IO_DEBUG
	fprintf(stderr, " read line: (%s)]]\n", buf);
#endif
	return -1;
}

static int my_sendcmd(int fd, char *buf, int len) {
	int ret;

#ifdef FERNLY_IO_DEBUG
	fprintf(stderr, "[[ writing [%s]... ", buf);
#endif
	while (1) {
		ret = write(fd, buf, len);
		if (-1 == ret) {
			perror("Unable to write to serial port");
			if (errno == EINTR)
				continue;
			return -1;
		} else
			break;
	}

#ifdef FERNLY_IO_DEBUG
	fprintf(stderr, "done.\n");
#endif
	return ret + 2;
}

static void getBaseName(char *name, uint32_t base) {

	switch (base)

	{
	case (0xA0000000):
		strcpy(name, "VERSION");
		break;

	case (0xA0010000):
		strcpy(name, "CONFIG");
		break;
	case (0xA0020000):
		strcpy(name, "GPIO");
		break;
	case (0xA0030000):
		strcpy(name, "RGU");
		break;
	case (0xA0050000):
		strcpy(name, "EMI");
		break;
	case (0xA0060000):
		strcpy(name, "CIRQ");
		break;
	case (0xA0070000):
		strcpy(name, "DMA");
		break;
	case (0xA0080000):
		strcpy(name, "UART1");
		break;
	case (0xA0090000):
		strcpy(name, "UART2");
		break;
	case (0xA00A0000):
		strcpy(name, "UART3");
		break;
	case (0xA00B0000):
		strcpy(name, "BTIF");
		break;
	case (0xA00C0000):
		strcpy(name, "GPT");
		break;
	case (0xA00D0000):
		strcpy(name, "KP");
		break;
	case (0xA00E0000):
		strcpy(name, "PWM");
		break;
	case (0xA00F0000):
		strcpy(name, "SIM");
		break;
	case (0xA0100000):
		strcpy(name, "SIM2");
		break;
	case (0xA0110000):
		strcpy(name, "SEJ");
		break;
	case (0xA0120000):
		strcpy(name, "I2C");
		break;
	case (0xA0130000):
		strcpy(name, "MSDC");
		break;
	case (0xA0140000):
		strcpy(name, "SFI");
		break;
	case (0xA0170000):
		strcpy(name, "MIXED");
		break;

	case (0xA0180000):
		strcpy(name, "MCU_TOPSM");
		break;
	case (0xA01C0000):
		strcpy(name, "EFUSE");
		break;
	case (0xA01E0000):
		strcpy(name, "SPI");
		break;
	case (0xA01F0000):
		strcpy(name, "OSTIMER");
		break;
	case (0xA0210000):
		strcpy(name, "ANALOG_MAP_");
		break;
	case (0xA0220000):
		strcpy(name, "MCU_MBIST");
		break;
	case (0xA0260000):
		strcpy(name, "FSPI_MAS");
		break;
	case (0xA0270000):
		strcpy(name, "MSDC2");
		break;
	case (0xA0280000):
		strcpy(name, "PWM2");
		break;
	case (0xA0290000):
		strcpy(name, "SPI_SLAVE");
		break;
	case (0xA02A0000):
		strcpy(name, "I2C_18V");
		break;

	case (0xA0400000):
		strcpy(name, "ROT_DMA");
		break;
	case (0xA0410000):
		strcpy(name, "CRZ");
		break;
	case (0xA0420000):
		strcpy(name, "CAMERA");
		break;

	case (0xA0430000):
		strcpy(name, "SCAM");
		break;
	case (0xA0440000):
		strcpy(name, "G2D");
		break;
	case (0xA0450000):
		strcpy(name, "LCD");
		break;
	case (0xA0460000):
		strcpy(name, "MMSYS_MBIST");
		break;
	case (0xA0470000):
		strcpy(name, "MM_COLOR");
		break;
	case (0xA0480000):
		strcpy(name, "MMSYS_CONFIG");
		break;

	case (0xA0500000):
		strcpy(name, "ARM_CONFG");
		break;
	case (0xA0510000):
		strcpy(name, "BOOT_ENG");
		break;
	case (0xA0520000):
		strcpy(name, "CDCMP");
		break;
	case (0xA0530000):
		strcpy(name, "L1_CACHE");
		break;
	case (0xA0540000):
		strcpy(name, "MPU");
		break;

	case (0xA0700000):
		strcpy(name, "PMU");
		break;
	case (0xA0710000):
		strcpy(name, "RTC");
		break;
	case (0xA0720000):
		strcpy(name, "ABBSYS");
		break;
	case (0xA0730000):
		strcpy(name, "ANA_CFGSYS");
		break;
	case (0xA0740000):
		strcpy(name, "PWM_2CH");
		break;
	case (0xA0750000):
		strcpy(name, "ACCDET");
		break;
	case (0xA0760000):
		strcpy(name, "ADIE_CIRQ");
		break;
	case (0xA0790000):
		strcpy(name, "AUXADC");
		break;

	case (0xA0900000):
		strcpy(name, "USB");
		break;
	case (0xA0910000):
		strcpy(name, "USB_SIFSLV");
		break;
	case (0xA0920000):
		strcpy(name, "DMA_AHB");
		break;

	case (0xA3300000):
		strcpy(name, "BT_CONFG");
		break;
	case (0xA3310000):
		strcpy(name, "BT_CIRQ");
		break;
	case (0xA3320000):
		strcpy(name, "BT_DMA");
		break;
	case (0xA3330000):
		strcpy(name, "BT_BTIF");
		break;
	case (0xA3340000):
		strcpy(name, "BT_PKV");
		break;
	case (0xA3350000):
		strcpy(name, "BT_TIM");
		break;
	case (0xA3360000):
		strcpy(name, "BT_RF");
		break;
	case (0xA3370000):
		strcpy(name, "BT_MODEM");
		break;
	case (0xA3380000):
		strcpy(name, "BT_DBGIF");
		break;
	case (0xA3390000):
		strcpy(name, "BT_MBIST_CONFG_");
		break;

	case (0x82000000):
		strcpy(name, "IDMA");
		break;
	case (0x82200000):
		strcpy(name, "DPRAM_CPU");
		break;
	case (0x82800000):
		strcpy(name, "AHB2DSPIO");
		break;
	case (0x82C00000):
		strcpy(name, "MD2GCONFG");
		break;
	case (0x82C10000):
		strcpy(name, "MD2G_MBIST_CONFG");
		break;
	case (0x82C30000):
		strcpy(name, "APC");
		break;
	case (0x82C70000):
		strcpy(name, "CSD_ACC");
		break;
	case (0x82CA0000):
		strcpy(name, "SHARE");
		break;
	case (0x82CB0000):
		strcpy(name, "IRDMA");
		break;
	case (0x82CC0000):
		strcpy(name, "PATCH");
		break;
	case (0x82CD0000):
		strcpy(name, "AFE");
		break;
	case (0x82CE0000):
		strcpy(name, "BFE");
		break;

	case (0x83000000):
		strcpy(name, "MDCONFIG");
		break;
	case (0x83008000):
		strcpy(name, "MODEM_MBIST_CONFIG");
		break;
	case (0x83010000):
		strcpy(name, "MODEM2G_TOPSM");
		break;
	case (0x83020000):
		strcpy(name, "TDMA");
		break;
	case (0x83030000):
		strcpy(name, "SHAREG2");
		break;
	case (0x83040000):
		strcpy(name, "DIVIDER");
		break;
	case (0x83050000):
		strcpy(name, "FCS");
		break;
	case (0x83060000):
		strcpy(name, "GCU");
		break;
	case (0x83070000):
		strcpy(name, "BSI");
		break;
	case (0x83080000):
		strcpy(name, "BPI");
		break;
	default:
		strcpy(name, "???");
		break;
	}
}
static uint64_t mt6262_live_mem_read(void *opaque, hwaddr addr, unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	char cmd[128];
	uint32_t ret;
	int len;
	char name[20];

	getBaseName(name, base);
	printf("%-10s: ", name);

	/* Write command out */
	switch (size) {
	case 1:
		len = snprintf(cmd, sizeof(cmd) - 1, "ro%08x", offset);
		printf("READ  BYTE 0x%08x =", offset);
		break;
	case 2:
		len = snprintf(cmd, sizeof(cmd) - 1, "rt%08x", offset);
		printf("READ  WORD 0x%08x =", offset);
		break;
	case 4:
		len = snprintf(cmd, sizeof(cmd) - 1, "rf%08x", offset);
		printf("READ  DWORD 0x%08x =", offset);
		break;
	default:
		printf("READ Unrecognized size %d at offset %d = \n", size, offset);
		len = snprintf(cmd, sizeof(cmd) - 1, "rf%08x", offset);
		break;
	}

	fflush(stdout);
	my_sendcmd(mt6262_fd, cmd, len);

	/* Read the response */
	len = my_getresponse(mt6262_fd, cmd, sizeof(cmd));
	if (len == -1) {
		perror("Unable to read response");
		return -1;
	}

	ret = strtoul(cmd, NULL, 16);

	if (size == 1)
		printf(" 0x%02x... ok\n", 0xff & ret);
	else if (size == 2)
		printf(" 0x%04x... ok\n", 0xffff & ret);
	else if (size == 4)
		printf(" 0x%08x... ok\n", 0xffffffff & ret);

	return ret;
}

static void mt6262_live_mem_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	char cmd[128];
	int len;
	uint32_t value = val;
	char name[20];

	getBaseName(name, base);
	printf("%-10s: ", name);

	/* Write command out */
	switch (size) {
	case 1:
		len = snprintf(cmd, sizeof(cmd) - 1, "wo%08x%02x", offset,
				0xff & value);
#ifdef PRINT_NORMAL
		printf("WRITE BYTE 0x%08x = 0x%02x...", offset, 0xff & (value));
#else
		printf("writeb( 0x%02x , 0x%08x ); // ", 0xff & (value), offset);
#endif
		break;
	case 2:
		len = snprintf(cmd, sizeof(cmd) - 1, "wt%08x%04x", offset,
				0xffff & value);
#ifdef PRINT_NORMAL
		printf("WRITE WORD0x%08x = 0x%04x...", offset,

		0xffff & value);
#else
		printf("writew( 0x%04x , 0x%08x ); // ", 0xffff & (value), offset);
#endif
		break;
	case 4:
	default:
		len = snprintf(cmd, sizeof(cmd) - 1, "wf%08x%08x", offset, value);
#ifdef PRINT_NORMAL
		printf("WRITE DWORD 0x%08x = 0x%08x...", offset, value);
#else
		printf("writel( 0x%08x , 0x%08x ); // ", value, offset);
#endif
		break;
	}

	fflush(stdout);
	my_sendcmd(mt6262_fd, cmd, len);

	/* Read the line back */
	len = my_getresponse(mt6262_fd, cmd, sizeof(cmd));
	if (len == -1) {
		perror("Unable to read line");
		return;
	}

	printf(" ok\n");
}

/* These are used for zero write combining */
static int f00d_count = 0;
static uint32_t f00d_start;
static uint32_t f00d_end;

static uint64_t mt6262_f00d_read(void *opaque, hwaddr addr, unsigned size) {
	uint32_t base = 0xf00d0000;
	uint32_t offset = base + addr;

	/* We're either not writing zeroes, or not continuing a run.  Flush. */
	if (f00d_count) {
		char cmd[128];
		int len;

		len = snprintf(cmd, sizeof(cmd) - 1, "z%08x%08x", f00d_start, f00d_end);
		my_sendcmd(mt6262_fd, cmd, len);

		/* Read the line back */
		len = my_getresponse(mt6262_fd, cmd, sizeof(cmd));

		f00d_count = 0;
	}
	return mt6262_live_mem_read(NULL, offset, size);
}

static void mt6262_f00d_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;
	char cmd[128];
	int len;

	/* Flush out the zeroes since we've reached the end of a run */
	if (val == 0) {

		/* Start a new run */
		if (f00d_count == 0) {
			f00d_count++;
			f00d_start = offset;
			f00d_end = offset + 4;
			return;
		}

		/* If this is a continuation of a run, buffer the write and return */
		if (offset == f00d_end) {
			f00d_count++;
			f00d_end = offset + 4;
			return;
		}
	}

	/* We're either not writing zeroes, or not continuing a run.  Flush. */
	if (f00d_count) {
		len = snprintf(cmd, sizeof(cmd) - 1, "z%08x%08x", f00d_start, f00d_end);
		my_sendcmd(mt6262_fd, cmd, len);

		/* Read the line back */
		len = my_getresponse(mt6262_fd, cmd, sizeof(cmd));

		f00d_count = 0;
	}

	if (val == 0) {
		f00d_count++;
		f00d_start = offset;
		f00d_end = offset + 4;
		return;
	}

	if (size == 4) {
		len = snprintf(cmd, sizeof(cmd) - 1, "wf%08x%08x", offset, value);
		my_sendcmd(mt6262_fd, cmd, len);

		/* Read the line back */
		len = my_getresponse(mt6262_fd, cmd, sizeof(cmd));
	} else
		mt6262_live_mem_write(opaque, addr, val, size);
}

static const MemoryRegionOps mt6262_f00d_ops = { .read = mt6262_f00d_read,
		.write = mt6262_f00d_write, .endianness = DEVICE_NATIVE_ENDIAN, };

static const MemoryRegionOps mt6262_live_mem_ops = { .read =
		mt6262_live_mem_read, .write = mt6262_live_mem_write, .endianness =
		DEVICE_NATIVE_ENDIAN, };

static void mt6262_hook_memory(uint32_t base, const char *name,
		const MemoryRegionOps *ops, int size) {
	MemoryRegion *hook = g_new(MemoryRegion, 1);
	MemoryRegion *address_space = get_system_memory();

	memory_region_init_io(hook, NULL, ops, (void *) (intptr_t) base, name,
			size);
	memory_region_add_subregion(address_space, base, hook);
}

static void mt6262_hook_live(uint32_t base, const char *name) {
	mt6262_hook_memory(base, name, &mt6262_live_mem_ops, 0x10000);
}

/*static void
 mt6262_hook_f00d(uint32_t base, const char *name)
 {
 MemoryRegion *hook = g_new(MemoryRegion, 1);
 MemoryRegion *address_space = get_system_memory();

 memory_region_init_rom_device(hook, NULL, &mt6262_f00d_ops,
 (void *) (intptr_t) base, name, 0x420000); // Up to 0xf04f0000
 memory_region_add_subregion(address_space, base, hook);
 }*/

// ***************************    EMI  ***************************************************
static uint32_t EMI_0x10000 = 0; // EMI Remap
static uint32_t EMI_0x160 = 0;
static uint32_t EMI_0x348 = 0;

static uint64_t mt6262_emi_read(void *opaque, hwaddr addr, unsigned size) {
	uint64_t val;
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;

	switch (addr) {
	case 10000:
		val = EMI_0x10000;
		break;
	case 0x160:
		val = EMI_0x160;
		break;
	case 0x348:
		val = EMI_0x348;
		break;

	default:
		//printf(" read emi register  0x%08x  \n", offset);
		val = 0;
		break;
	}
	printf(" read emi register  0x%08x  \n", offset);
	fflush(stdout);
	return val;
}

static void mt6262_emi_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;

	printf(" Write emi register  0x%08x = 0x%08x\n", offset, value);

	switch (addr) {

	case 0x10000:
		EMI_0x10000 = val;
		break;
	case 0x160:
		EMI_0x160 = val;
		break;
	case 0x348:
		EMI_0x348 = val;
		break;

	default:
		//printf(" Write emi register  0x%08x = 0x%08x\n", offset, value);
		break;
	}
	fflush(stdout);
}

static const MemoryRegionOps mt6262_emi_ops = { .read = mt6262_emi_read,
		.write = mt6262_emi_write, .endianness = DEVICE_NATIVE_ENDIAN, };

//*********************************** IRQ ********************************************++
static uint64_t mt6262_irq_read(void *opaque, hwaddr addr, unsigned size) {
	uint64_t val;
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;

	switch (addr) {

	// EINT Status Register
	case 0x100:
		printf("read irq EINT_Status register 0x%08x\n", EINT_MASK);
		val = EINT_Status;
		break;

	case 0x104:
		printf("read irq EINT_MASK register 0x%08x\n", EINT_MASK);
		val = EINT_MASK;
		break;

	case 0x1c:
		printf("read irq IRQ_MASK register 0x%08x\n", IRQ_MASK);
		val = 0;
		break;

	case 0x30:
		printf("read irq IRQ Sensitive Register register 0x%08x\n",
				IRQ_Sensitive_Register);
		val = IRQ_Sensitive_Register;
		break;

	default:

		printf("read irq register  0x%08x\n", offset);
		val = 0;
		break;
	}

	return val;
}

static void mt6262_irq_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;
	switch (addr) {
	case 0x20:
		printf("write irq IRQ Mask Clear Register 0x%08x\n", value);
		IRQ_MASK &= (~value);
		printf("IRQ Mask = 0x%08x\n", IRQ_MASK);
		break;

	case 0x24:
		printf("write irq IRQ Mask Set Register 0x%08x\n", value);
		IRQ_MASK |= value;
		printf("IRQ Mask = 0x%08x\n", IRQ_MASK);

		break;
	case 0x1c:
		printf("write irq IRQ MASK register 0x%08x\n", value);
		IRQ_MASK = value;
		break;

	case 0x30:
		printf("write irq IRQ Sensitive Register register 0x%08x\n", value);
		IRQ_Sensitive_Register = value;
		break;

	default:
		printf("write irq register  0x%08x = 0x%08x\n", offset, value);
		break;
	}
}

static const MemoryRegionOps mt6262_irq_ops = { .read = mt6262_irq_read,
		.write = mt6262_irq_write, .endianness = DEVICE_NATIVE_ENDIAN, };

//******************************** USB *********************************

static void mt6262_usb_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;

	switch (addr) {
	case 0x24:
		printf("send char           0x%08x =  0x%08x\n", offset, value);
		break;

	default:
		printf("USB register write  0x%08x =  0x%08x\n", offset, value);
		break;
	}

}

static uint64_t mt6262_usb_read(void *opaque, hwaddr addr, unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value;

	switch (addr) {
	case 0x0:
		printf("usb register read  0x%08x = 0x0a\n", offset);
		value = 0xa;
		break;
	case 0x2:
		//printf(".");
		value = 0x0;
		break;
	case 0x4:
		//printf(".");
		value = 0x0;
		break;

	case 0x13:
		printf("usb register read  0x%08x = 0x40\n", offset);
		value = 0x40;
		break;

	default:
		printf("usb register read  0x%08x = 0x00\n", offset);
		value = 0x0;
		break;
	}
	return value;
}

static const MemoryRegionOps mt6262_usb_ops = { .read = mt6262_usb_read,
		.write = mt6262_usb_write, .endianness = DEVICE_NATIVE_ENDIAN, };

//******************************** efuse *********************************

static void mt6262_efuse_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;

	printf("efuse register  0x%08x =  0x%08x\n", offset, value);

}

static const MemoryRegionOps mt6262_efuse_ops = { .read = mt6262_live_mem_read,
		.write = mt6262_efuse_write, .endianness = DEVICE_NATIVE_ENDIAN, };

//******************************** RTC *********************************

static void mt6262_rtc_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;
	switch (addr) {
	case 0x0:
		printf("RTC register  0x%08x = 0x%08x\n", offset, value);
		break;

	default:
		mt6262_live_mem_write(opaque, addr, val, size);

		break;
	}
}

static const MemoryRegionOps mt6262_rtc_ops = { .read = mt6262_live_mem_read,
		.write = mt6262_rtc_write, .endianness = DEVICE_NATIVE_ENDIAN, };

// ********************************** SEJ ******************************************

static void mt6262_sej_write(void *opaque, hwaddr addr, uint64_t val,
		unsigned size) {
	uint32_t base = (uint32_t) (intptr_t) opaque;
	uint32_t offset = base + addr;
	uint32_t value = val;

	printf("sej  0x%08x = 0x%08x\n", offset, value);

}

static const MemoryRegionOps mt6262_sej_ops = { .read = mt6262_live_mem_read,
		.write = mt6262_sej_write, .endianness = DEVICE_NATIVE_ENDIAN, };

//*****************************************************************************************

/* Make unassigned access nonfatal */
static void mt6262_do_unassigned_access(CPUState *cpu, hwaddr addr,
		bool is_write, bool is_exec, int opaque, unsigned size) {
	if (is_exec)
		printf("!!! UNASSIGNED EXEC: \n");
//	else if (is_write)
//		printf("!!! UNASSIGNED WRITE: \n");
//	else
//		printf("!!! UNASSIGNED READ: \n");
	return;
}

static void mt6262_init(QEMUMachineInitArgs *args) {
	const char *cpu_model = args->cpu_model;
	ARMCPU *cpu;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *iram_region = g_new(MemoryRegion, 1);
	MemoryRegion *psram_region = g_new(MemoryRegion, 1);
	MemoryRegion *bootrom_region = g_new(MemoryRegion, 1);
	//MemoryRegion *firmrom_region = g_new(MemoryRegion, 1);

	CPUClass *cc;

	mt6262_fd = open(mt6262_DEBUG_UART, O_RDWR);
	if (-1 == mt6262_fd) {
		perror("Unable to open debug uart " mt6262_DEBUG_UART);
		exit(1);
	}

	sleep(2); //required to make flush work, for some reason
	tcflush(mt6262_fd, TCIOFLUSH);

	if (!cpu_model) {
		cpu_model = "arm926";
	}
	cpu = cpu_arm_init(cpu_model);
	if (!cpu) {
		fprintf(stderr, "Unable to find CPU definition\n");
		exit(1);
	}

	/* Hook unassigned memory accesses and send them to live mt6262 */
	cc = CPU_GET_CLASS(cpu);
	cc->do_unassigned_access = mt6262_do_unassigned_access;

	iram = malloc(FV_IRAM_SIZE);
	memory_region_init_ram_ptr(iram_region, NULL, "mt6262.iram",
	FV_IRAM_SIZE, iram);
	vmstate_register_ram_global(iram_region);
	memory_region_add_subregion(address_space_mem, FV_IRAM_BASE, iram_region);

	psram = malloc(FV_PSRAM_SIZE);
	memory_region_init_ram_ptr(psram_region, NULL, "mt6262.psram",
	FV_PSRAM_SIZE, psram);
	vmstate_register_ram_global(psram_region);
	memory_region_add_subregion(address_space_mem, FV_PSRAM_BASE, psram_region);

	bootrom = malloc(FV_BOOTROM_SIZE);
	memory_region_init_ram_ptr(bootrom_region, NULL, "mt6262.bootrom",
	FV_BOOTROM_SIZE, bootrom);
	vmstate_register_ram_global(bootrom_region);
	memory_region_add_subregion(address_space_mem, FV_BOOTROM_BASE,
			bootrom_region);

//	firmrom = malloc(FV_FIRMROM_SIZE);
//	memory_region_init_ram_ptr(firmrom_region, NULL, "mt6262.firmrom",
//	FV_FIRMROM_SIZE, firmrom);
//	vmstate_register_ram_global(firmrom_region);
//	memory_region_add_subregion(address_space_mem, FV_FIRMROM_BASE,
//			firmrom_region);

#ifdef DA_TOOL

	FILE *tmp = fopen(PSRAM_FILE, "rb");
	if (!tmp) {
		fprintf(stderr, "Unable to open file EXT_RAM\n");
	} else {

		struct stat st;
		stat(PSRAM_FILE, &st);
		int size = st.st_size;
		fprintf(stderr, "File size PSRAM_FILE %d\n", size);
		if (!fread(psram, size, 1, tmp))
		fprintf(stderr, "Unable to read in PSRAM_FILE file\n");

		fclose(tmp);
	}

	FILE *tmp2 = fopen(IRAM_FILE, "rb");
	if (!tmp2) {
		fprintf(stderr, "Unable to open file IRAM_FILE\n");

	} else {
		struct stat st;
		stat(IRAM_FILE, &st);
		int size = st.st_size;
		fprintf(stderr, "File size IRAM_FILE %d\n", size);
		if (!fread(iram + IRAM_FILE_OFFSET, size, 1, tmp2))
		fprintf(stderr, "Unable to read in IRAM_FILE file\n");

		fclose(tmp2);

	}
#endif

#ifdef BOOT_ROM
	FILE *tmp3 = fopen(BOOTROM_FILE, "rb");
	if (!tmp3) {
		fprintf(stderr, "Unable to open file BOOTROM_FILE\n");

	} else {
		struct stat st;
		stat(BOOTROM_FILE, &st);
		int size = st.st_size;
		fprintf(stderr, "File size BOOTROM_FILE %d\n", size);
		if (!fread(bootrom, size, 1, tmp3))
			fprintf(stderr, "Unable to read in BOOTROM_FILE file\n");

		fclose(tmp3);

	}
#endif

#ifdef MY_ROM_PS_RAM
	FILE *tmp4 = fopen(MY_ROM_FILE, "rb");
		if (!tmp4) {
			fprintf(stderr, "Unable to open file EXT_RAM\n");
		} else {

			struct stat st;
			stat(MY_ROM_FILE, &st);
			int size = st.st_size;
			fprintf(stderr, "File size MY_ROM_FILE %d\n", size);
			if (!fread(psram, size, 1, tmp4))
			fprintf(stderr, "Unable to read in MY_ROM_FILE file\n");

			fclose(tmp4);
		}
#endif

	/* Add serial port */
	{
		DeviceState *dev = qdev_create(NULL, TYPE_mt6262_UART);
		qdev_prop_set_chr(dev, "chardev", serial_hds[0]);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MT6223_UART_BASE);
	}

	mt6262_hook_memory(0xa0900000, "MTK6261_USB_BASE", &mt6262_usb_ops,
			0x10000);

	mt6262_hook_live(0x80000000, "MTK6261_CONFG_BASE");
	//mt6262_hook_live(0xa0900000, "MTK6261_USB_BASE");
	mt6262_hook_live(0x80200000, "MTK6261_XYZ_BASE");

	mt6262_hook_live(0xA00C0000, "MTK6261_GPT_BASE");

	mt6262_hook_live(0xA0010000, "MTK6261_CONFIG_BASE");

	mt6262_hook_live(0xA0030000, "MTK6261_RGU_BASE");
	mt6262_hook_live(0xA0040000, "MTK6261_CACHE_BASE");

	mt6262_hook_memory(0xA01C0000, "MTK6261_EFUSE_BASE", &mt6262_efuse_ops,
			0x10000);

	mt6262_hook_memory(0xA0050000, "MTK6261_EMI_BASE", &mt6262_emi_ops,
			0x10000);

	mt6262_hook_live(0xA0110000, "MTK6261_SEJ_BASE");

	mt6262_hook_live(0xA0140000, "MTK6261_SFI_BASE"); // Serial Flash

	mt6262_hook_live(0xA0540000, "MTK6261_MPU_BASE"); // MPU Unit

	mt6262_hook_live(0xA0530000, "MTK6261_MPU_L1CACHE");

	mt6262_hook_live(0xA0710000, "MTK6261_RTC");

	mt6262_hook_live(0xA0020000, "MTK6261_GPIO");

	mt6262_hook_live(0xA0120000, "MTK6261_I2C");


	//mt6262_hook_f00d(0xf00d0000, "f00d");

	qemu_register_reset(mt6262_cpu_reset, cpu);
}

static QEMUMachine mt6262_machine = { .name = "mt6262", .desc =
		"6262 (ARM7EJ-S)", .init = mt6262_init, };

static void mt6262_machine_init(void) {
	qemu_register_machine(&mt6262_machine);
}

machine_init(mt6262_machine_init);

