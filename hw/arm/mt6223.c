/*
 * mt6223 emulation
 * Copyrigth (c) 2015 Mandl
 * Copyright (c) 2014 Sean Cross
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
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
#define READ_MAIN_BIN

#define PRINT_NORMAL 1
#define FERNVALE_DEBUG_UART "/dev/ttyUSB0"
#define FERNVALE_DEBUG_PROMPT "fernly>"
#define FERNVALE_READY_CHAR 'k'
static int fernvale_fd;

//0x80010040,0x2,MTK6223_EMI_REMAP
//
// External RAM    = 0x8000000
//
// Extrernal Flash = 0x0000000

// Internal RAM MT6223
#define FV_IRAM_SIZE 8* 1024 * 40   // 40k
#define FV_IRAM_BASE 0x40000000

#define MT6223_UART_BASE 0x80130000

// External RAM
#define FV_PSRAM_SIZE 8 * 1024 * 1024
#define FV_PSRAM_BASE 0x0000000

// Extrernal Flash
#define FV_ROM_SIZE 8 * 1024 * 1024
#define FV_ROM_BASE 0x8000000

static void *iram;
static void *psram;
static void *rom;

static bool toogle = 0;
static uint32_t IRQ_MASK = 0xFFFFFFFF;


static uint32_t IRQ_Sensitive_Register;

static uint32_t EINT_MASK = 0x00;
static uint32_t EINT_Status = 0x00;

#define TYPE_FERNVALE_UART "mt6223-uart"
#define FERNVALE_UART(obj) \
        OBJECT_CHECK(FernvaleUARTState, (obj), TYPE_FERNVALE_UART)

static void
fernvale_cpu_reset(void *opaque)
{
  ARMCPU *cpu = opaque;
  CPUARMState *env = &cpu->env;

  cpu_reset(CPU(cpu));

  /* Place the PC at the start of the main program */
  //env->regs[15] = 0x0;
  env->regs[15] = 0x8000000;

}

static int
my_getresponse(int fd, char *buf, int len)
{
  int offset = 0;
  ssize_t ret;
  uint8_t byte;
  fd_set fds;
  struct timeval tv;

#ifdef FERNLY_IO_DEBUG
  fprintf(stderr, "[[ reading : ");
#endif

  while (len > 1)
    {

      FD_ZERO(&fds);
      FD_SET(fd, &fds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      ret = select(fd + 1, &fds, NULL, NULL, &tv);
      if (ret == -1)
        {
          if (errno == EINTR)
            {
//                fprintf(stderr, " [interrupted select - retrying] ");
              continue;
            }
          perror("Couldn't select");
          return -1;
        }
      if (!ret)
        {
          fprintf(stderr, " [no response - board crashed?] ");
          return -1;
        }

      ret = read(fd, &byte, 1);

      if (-1 == ret)
        {
          if (errno != EINTR)
            {
              perror("couldn't read");
              return ret;
            }
//            fprintf(stderr, " [interrupted read - retrying] ");
          continue;
        }

      if (byte == FERNVALE_READY_CHAR)
        {
          buf[offset++] = '\0';
#ifdef FERNLY_IO_DEBUG
          fprintf(stderr, " read line: (%s)]]\n", buf);
#endif
          if (strstr(buf, "Handled IRQ"))
            {
              printf(">>> HANDLED IRQ <<<\n");
            }
          if (strstr(buf, "Handled FIQ"))
            {
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

static int
my_sendcmd(int fd, char *buf, int len)
{
  int ret;

#ifdef FERNLY_IO_DEBUG
  fprintf(stderr, "[[ writing [%s]... ", buf);
#endif
  while (1)
    {
      ret = write(fd, buf, len);
      if (-1 == ret)
        {
          perror("Unable to write to serial port");
          if (errno == EINTR)
            continue;
          return -1;
        }
      else
        break;
    }

#ifdef FERNLY_IO_DEBUG
  fprintf(stderr, "done.\n");
#endif
  return ret + 2;
}

static uint64_t
fernvale_live_mem_read(void *opaque, hwaddr addr, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  char cmd[128];
  uint32_t ret;
  int len;

  /* Write command out */
  switch (size)
    {
  case 1:
    len = snprintf(cmd, sizeof(cmd) - 1, "ro%08x", offset);
    printf("// READ BYTE Fernvale Live 0x%08x =", offset);
    break;
  case 2:
    len = snprintf(cmd, sizeof(cmd) - 1, "rt%08x", offset);
    printf("// READ WORD Fernvale Live 0x%08x =", offset);
    break;
  case 4:
    len = snprintf(cmd, sizeof(cmd) - 1, "rf%08x", offset);
    printf("// READ DWORD Fernvale Live 0x%08x =", offset);
    break;
  default:
    printf("READ Unrecognized size %d at offset %d = \n", size, offset);
    len = snprintf(cmd, sizeof(cmd) - 1, "rf%08x", offset);
    break;
    }

  fflush(stdout);
  my_sendcmd(fernvale_fd, cmd, len);

  /* Read the response */
  len = my_getresponse(fernvale_fd, cmd, sizeof(cmd));
  if (len == -1)
    {
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

static void
fernvale_live_mem_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  char cmd[128];
  int len;
  uint32_t value = val;

  /* Write command out */
  switch (size)
    {
  case 1:
    len = snprintf(cmd, sizeof(cmd) - 1, "wo%08x%02x", offset, 0xff & value);
#ifdef PRINT_NORMAL
    printf("// WRITE BYTE Fernvale Live 0x%08x = 0x%02x...", offset,
        0xff & (value));
#else
    printf("writeb( 0x%02x , 0x%08x ); // ", 0xff & (value), offset);
#endif
    break;
  case 2:
    len = snprintf(cmd, sizeof(cmd) - 1, "wt%08x%04x", offset, 0xffff & value);
#ifdef PRINT_NORMAL
    printf("// WRITE WORD Fernvale Live 0x%08x = 0x%04x...", offset,

    0xffff & value);
#else
    printf("writew( 0x%04x , 0x%08x ); // ", 0xffff & (value), offset);
#endif
    break;
  case 4:
  default:
    len = snprintf(cmd, sizeof(cmd) - 1, "wf%08x%08x", offset, value);
#ifdef PRINT_NORMAL
    printf("// WRITE DWORD Fernvale Live 0x%08x = 0x%08x...", offset, value);
#else
    printf("writel( 0x%08x , 0x%08x ); // ", value, offset);
#endif
    break;
    }

  fflush(stdout);
  my_sendcmd(fernvale_fd, cmd, len);

  /* Read the line back */
  len = my_getresponse(fernvale_fd, cmd, sizeof(cmd));
  if (len == -1)
    {
      perror("Unable to read line");
      return;
    }

  printf(" ok\n");
}

/* These are used for zero write combining */
static int f00d_count = 0;
static uint32_t f00d_start;
static uint32_t f00d_end;

static uint64_t
fernvale_f00d_read(void *opaque, hwaddr addr, unsigned size)
{
  uint32_t base = 0xf00d0000;
  uint32_t offset = base + addr;

  /* We're either not writing zeroes, or not continuing a run.  Flush. */
  if (f00d_count)
    {
      char cmd[128];
      int len;

      len = snprintf(cmd, sizeof(cmd) - 1, "z%08x%08x", f00d_start, f00d_end);
      my_sendcmd(fernvale_fd, cmd, len);

      /* Read the line back */
      len = my_getresponse(fernvale_fd, cmd, sizeof(cmd));

      f00d_count = 0;
    }
  return fernvale_live_mem_read(NULL, offset, size);
}

static void
fernvale_f00d_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;
  char cmd[128];
  int len;

  /* Flush out the zeroes since we've reached the end of a run */
  if (val == 0)
    {

      /* Start a new run */
      if (f00d_count == 0)
        {
          f00d_count++;
          f00d_start = offset;
          f00d_end = offset + 4;
          return;
        }

      /* If this is a continuation of a run, buffer the write and return */
      if (offset == f00d_end)
        {
          f00d_count++;
          f00d_end = offset + 4;
          return;
        }
    }

  /* We're either not writing zeroes, or not continuing a run.  Flush. */
  if (f00d_count)
    {
      len = snprintf(cmd, sizeof(cmd) - 1, "z%08x%08x", f00d_start, f00d_end);
      my_sendcmd(fernvale_fd, cmd, len);

      /* Read the line back */
      len = my_getresponse(fernvale_fd, cmd, sizeof(cmd));

      f00d_count = 0;
    }

  if (val == 0)
    {
      f00d_count++;
      f00d_start = offset;
      f00d_end = offset + 4;
      return;
    }

  if (size == 4)
    {
      len = snprintf(cmd, sizeof(cmd) - 1, "wf%08x%08x", offset, value);
      my_sendcmd(fernvale_fd, cmd, len);

      /* Read the line back */
      len = my_getresponse(fernvale_fd, cmd, sizeof(cmd));
    }
  else
    fernvale_live_mem_write(opaque, addr, val, size);
}

static const MemoryRegionOps fernvale_f00d_ops =
  { .read = fernvale_f00d_read, .write = fernvale_f00d_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

static const MemoryRegionOps fernvale_live_mem_ops =
  { .read = fernvale_live_mem_read, .write = fernvale_live_mem_write,
      .endianness = DEVICE_NATIVE_ENDIAN, };

static void
fernvale_hook_memory(uint32_t base, const char *name,
    const MemoryRegionOps *ops, int size)
{
  MemoryRegion *hook = g_new(MemoryRegion, 1);
  MemoryRegion *address_space = get_system_memory();

  memory_region_init_io(hook, NULL, ops, (void *) (intptr_t) base, name, size);
  memory_region_add_subregion(address_space, base, hook);
}

static void
fernvale_hook_live(uint32_t base, const char *name)
{
  fernvale_hook_memory(base, name, &fernvale_live_mem_ops, 0x10000);
}

static void
fernvale_hook_f00d(uint32_t base, const char *name)
{
  MemoryRegion *hook = g_new(MemoryRegion, 1);
  MemoryRegion *address_space = get_system_memory();

  memory_region_init_rom_device(hook, NULL, &fernvale_f00d_ops,
      (void *) (intptr_t) base, name, 0x420000); // Up to 0xf04f0000
  memory_region_add_subregion(address_space, base, hook);
}

// ***************************    EMI
static uint64_t
fernvale_emi_read(void *opaque, hwaddr addr, unsigned size)
{
  uint64_t val;
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;

  switch (offset)
    {
  case 0x80010040:
    val = 0x3;
    break;

  default:
    printf(" emi register  0x%08x  \n", offset);
    val = 0;
    break;
    }

  return val;
}

static void
fernvale_emi_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;

  switch (offset)
    {

  default:
    printf(" Write emi register  0x%08x = 0x%08x\n", offset, value);
    break;
    }
}

static const MemoryRegionOps fernvale_emi_ops =
  { .read = fernvale_emi_read, .write = fernvale_emi_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

//*********************************** IRQ ********************************************++
static uint64_t
fernvale_irq_read(void *opaque, hwaddr addr, unsigned size)
{
  uint64_t val;
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;

  switch (addr)
    {

   // EINT Status Register
   case 0x100:
   printf("read irq EINT_Status register 0x%08x\n",EINT_MASK);
   val=EINT_Status;
   break;

   case 0x104:
   printf("read irq EINT_MASK register 0x%08x\n",EINT_MASK);
   val=EINT_MASK;
   break;

   case 0x1c:
   printf("read irq IRQ_MASK register 0x%08x\n",IRQ_MASK);
   val = 0;
   break;

   case 0x30:
   printf("read irq IRQ Sensitive Register register 0x%08x\n",IRQ_Sensitive_Register);
   val = IRQ_Sensitive_Register;
   break;

  default:

    printf("read irq register  0x%08x\n", offset);
    val = 0;
    break;
    }

  return val;
}

static void
fernvale_irq_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;
  switch (addr)
    {
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

static const MemoryRegionOps fernvale_irq_ops =
  { .read = fernvale_irq_read, .write = fernvale_irq_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };
//******************************** Config *********************************

static void
fernvale_config_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;
  switch (addr)
    {
  case 0x100:
    printf("config register  0x%08x =  0x%08x\n", offset, value);
    break;
  case 0x110:
    printf("config register  0x%08x =  0x%08x\n", offset, value);
    break;
  case 0x118:
    printf("config register  0x%08x =  0x%08x\n", offset, value);
    break;
  default:
    fernvale_live_mem_write(opaque, addr, val, size);

    break;
    }
}

static const MemoryRegionOps fernvale_config_ops =
  { .read = fernvale_live_mem_read, .write = fernvale_config_write,
      .endianness = DEVICE_NATIVE_ENDIAN, };

//******************************** rgu *********************************

static void
fernvale_rgu_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;
  switch (addr)
    {
  case 0x0:
    printf("RGU register  0x%08x=0x%08x\n", offset, value);
    break;

  default:
    fernvale_live_mem_write(opaque, addr, val, size);

    break;
    }
}

static const MemoryRegionOps fernvale_rgu_ops =
  { .read = fernvale_live_mem_read, .write = fernvale_rgu_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

//******************************** RTC *********************************

static void
fernvale_rtc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
  uint32_t base = (uint32_t) (intptr_t) opaque;
  uint32_t offset = base + addr;
  uint32_t value = val;
  switch (addr)
    {
  case 0x0:
    printf("RTC register  0x%08x = 0x%08x\n", offset, value);
    break;

  default:
    fernvale_live_mem_write(opaque, addr, val, size);

    break;
    }
}

static const MemoryRegionOps fernvale_rtc_ops =
  { .read = fernvale_live_mem_read, .write = fernvale_rtc_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

// ********************************** TDMA ******************************************

static uint64_t
fernvale_tdma_read(void *opaque, hwaddr addr, unsigned size)
{
  uint64_t val;

  switch (addr)
    {
  case 0x0:
    toogle = !toogle;
    if (toogle)
      val = 0;
    else
      val = 0x98a;
    //val = 0x134d;
    break;

  default:

    val = fernvale_live_mem_read(opaque, addr, size);

    break;
    }

  return val;
}

static const MemoryRegionOps fernvale_tdma_ops =
  { .read = fernvale_tdma_read, .write = fernvale_live_mem_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

/* Make unassigned access nonfatal */
static void
fernvale_do_unassigned_access(CPUState *cpu, hwaddr addr, bool is_write,
    bool is_exec, int opaque, unsigned size)
{
  if (is_exec)
    printf("!!! UNASSIGNED EXEC: ");
  else if (is_write)
    printf("!!! UNASSIGNED WRITE: ");
  else
    printf("!!! UNASSIGNED READ: ");
  return;
}

static void
mt6223_init(QEMUMachineInitArgs *args)
{
  const char *cpu_model = args->cpu_model;
  ARMCPU *cpu;
  MemoryRegion *address_space_mem = get_system_memory();
  MemoryRegion *iram_region = g_new(MemoryRegion, 1);
  MemoryRegion *psram_region = g_new(MemoryRegion, 1);
  MemoryRegion *rom_region = g_new(MemoryRegion, 1);

  //DriveInfo *dinfo;
  //int flash_size;
  CPUClass *cc;

  fernvale_fd = open(FERNVALE_DEBUG_UART, O_RDWR);
  if (-1 == fernvale_fd)
    {
      perror("Unable to open debug uart " FERNVALE_DEBUG_UART);
      exit(1);
    }

  sleep(2); //required to make flush work, for some reason
  tcflush(fernvale_fd, TCIOFLUSH);

  if (!cpu_model)
    {
      cpu_model = "arm926";
    }
  cpu = cpu_arm_init(cpu_model);
  if (!cpu)
    {
      fprintf(stderr, "Unable to find CPU definition\n");
      exit(1);
    }

  /* Hook unassigned memory accesses and send them to live Fernvale */
  cc = CPU_GET_CLASS(cpu);
  cc->do_unassigned_access = fernvale_do_unassigned_access;

  iram = malloc(FV_IRAM_SIZE);
  memory_region_init_ram_ptr(iram_region, NULL, "mt6223.iram", FV_IRAM_SIZE,
      iram);
  vmstate_register_ram_global(iram_region);
  memory_region_add_subregion(address_space_mem, FV_IRAM_BASE, iram_region);

  rom = malloc(FV_ROM_SIZE);
  memory_region_init_ram_ptr(rom_region, NULL, "mt6223.rom", FV_ROM_SIZE, rom);
  vmstate_register_ram_global(rom_region);
  memory_region_add_subregion(address_space_mem, FV_ROM_BASE, rom_region);

  psram = malloc(FV_PSRAM_SIZE);
  memory_region_init_ram_ptr(psram_region, NULL, "mt6223.psram", FV_PSRAM_SIZE,
      psram);
  vmstate_register_ram_global(psram_region);
  memory_region_add_subregion(address_space_mem, FV_PSRAM_BASE, psram_region);

  /* Register SPI NOR flash
   dinfo = drive_get(IF_PFLASH, 0, 0);
   if (!dinfo) {
   fprintf(stderr, "No flash image specified.  "
   "Specify with -pflash [spi ROM]\n");
   exit(1);
   }

   flash_size = bdrv_getlength(dinfo->bdrv);
   if (flash_size != 8*1024*1024) {
   fprintf(stderr, "Invalid flash image size (expected 8MB)\n");
   exit(1);
   }

   pflash_cfi01_register(0x10000000, NULL,
   "fernvale.spi", flash_size,
   dinfo->bdrv, 0x10000,
   (flash_size + 0xffff) >> 16,
   2, 0x00BF, 0x236D, 0x0000, 0x0000,
   0);
   */

  printf("filename %s\n", (args->kernel_filename));

  //printf("filename cmd %s\n",args->kernel_cmdline);

#ifdef READ_MAIN_BIN

  FILE *tmp = fopen("dumpgps3.bin", "rb");
  if (!tmp)
    {
      fprintf(stderr, "Unable to open file main.bin\n");
    }
  else
    {

      struct stat st;
      stat("dumpgps3.bin", &st);
      int size = st.st_size;
      fprintf(stderr, "File size %d\n", size);
      if (!fread(rom, size, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");

      fseek(tmp, 0x4AF1EC, SEEK_SET);   // 0x84AF1EC
      if (!fread(psram + 0x200, 0x15A0, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");

      // internal ram
      fseek(tmp, 0x4BB4EC, SEEK_SET);   // 0x84BB4EC
      if (!fread(iram, 0x6520, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");

      fseek(tmp, 0x4B078C, SEEK_SET);   // 0x84B078C
      if (!fread(psram + 0x3F368, 0xAD08, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");

      // DSP dma
      fseek(tmp, 0x4BB494, SEEK_SET);   // 0x84BB494
      if (!fread(psram + 0x3FC000, 0x58, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");
      fseek(tmp, 0x4BB4EC, SEEK_SET);   // 0x84BB4EC
      if (!fread(psram + 0x3FE000, 0x58, 1, tmp))
        fprintf(stderr, "Unable to read in RAM file\n");

      fclose(tmp);
    }

#else

  // Load ROM
  FILE *tmp = fopen("bird.bin", "rb");
  if (!tmp)
    {
      fprintf(stderr, "Unable to open file bird.bin\n");
    }
  else
    {
      struct stat st;
      stat("bird.bin", &st);
      int size = st.st_size;
      fprintf(stderr, "File size bird.bin %d\n", size);
      if (!fread(rom, size, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      // Copy interrupt vectors to ram 0x0000
      fseek(tmp, 0x0, SEEK_SET);
      if (!fread(psram, 170, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      // Copy some vars to ram 0x2944
      fseek(tmp, 0x21Df5C, SEEK_SET);
      if (!fread(psram + 0x2944, 0xA5844, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      // Copy Code to internal ram 0x40000000
      fseek(tmp, 0x2C37D0, SEEK_SET);
      if (!fread(iram, 0x58C8, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      // Copy some vars to ram 0x200
      fseek(tmp, 0x21CE30, SEEK_SET);
      if (!fread(psram + 0x200, 0x112C, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      // Setup RAM for DSP DMA 0xFE000 ???
      fseek(tmp, 0x2C37A0, SEEK_SET);
      if (!fread(psram + 0xFE000, 0x40, 1, tmp))
      fprintf(stderr, "Unable to read in RAM file\n");

      fclose(tmp);

    }
#endif

  /* Add serial port */
    {
      DeviceState *dev = qdev_create(NULL, TYPE_mt6262_UART);
      qdev_prop_set_chr(dev, "chardev", serial_hds[0]);
      qdev_init_nofail(dev);
      sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MT6223_UART_BASE);
    }

  // APB Peripherals
  fernvale_hook_live(0x80000000, "MTK6223_CONFG_BASE");
  //fernvale_hook_live( 0x80010000,"MTK6223_EMI_BASE");
  //fernvale_hook_live(   0x80020000,"MTK6223_CIRQ_BASE");
  //fernvale_hook_live(0x80030000, "MTK6223_DMA_BASE");
  //fernvale_hook_live(0x80040000, "MTK6223_RGU_BASE");
  fernvale_hook_live(0x80060000, "MTK6223_GCU_BASE");
  fernvale_hook_live(0x80070000, "MTK6223_I2C_BASE");
  fernvale_hook_live(0x80090000, "MTK6223_SWDBG_BASE");
  fernvale_hook_live(0x80100000, "MTK6223_GPT_BASE");
  fernvale_hook_live(0x80110000, "MTK6223_KP_BASE");
  fernvale_hook_live(0x80120000, "MTK6223_GPIO_BASE");
  //fernvale_hook_live(0x80130000, "MTK6223_UART1_BASE");
  fernvale_hook_live(0x80140000, "MTK6223_SIM_BASE");
  fernvale_hook_live(0x80150000, "MTK6223_PWM_BASE");
  fernvale_hook_live(0x80160000, "MTK6223_ALTER_BASE");
  fernvale_hook_live(0x80170000, "MTK6223_SEJ_BASE");
  fernvale_hook_live(0x80180000, "MTK6223_UART2_BASE");
  fernvale_hook_live(0x801b0000, "MTK6223_UART3_BASE");
//fernvale_hook_live(0x80200000, "MTK6223_TDMA_BASE");
  //fernvale_hook_live(0x80210000, "MTK6223_RTC_BASE");
  fernvale_hook_live(0x80220000, "MTK6223_BSI_BASE");
  fernvale_hook_live(0x80230000, "MTK6223_BPI_BASE");
  fernvale_hook_live(0x80240000, "MTK6223_AFC_BASE");
  fernvale_hook_live(0x80250000, "MTK6223_APC_BASE");
  fernvale_hook_live(0x80260000, "MTK6223_FCS_BASE");
  fernvale_hook_live(0x80270000, "MTK6223_AUXADC_BASE");
  fernvale_hook_live(0x80280000, "MTK6223_DIVIDER_BASE");
  fernvale_hook_live(0x80290000, "MTK6223_CSD_ACC_BASE");
  fernvale_hook_live(0x80300000, "MTK6223_SHARE1_BASE");
  fernvale_hook_live(0x80310000, "MTK6223_PATCH1_BASE");
  fernvale_hook_live(0x80320000, "MTK6223_SHARE2_BASE");
  fernvale_hook_live(0x80330000, "MTK6223_PATCH2_BASE");
  fernvale_hook_live(0x80400000, "MTK6223_AFE_BASE");
  fernvale_hook_live(0x80410000, "MTK6223_BFE_BASE");
  fernvale_hook_live(0x80500000, "MTK6223_MIXED_BASE");

  fernvale_hook_live(0x90000000, "MTK6223_LCD");

  // DSP dual port ram
  fernvale_hook_live(0x50000000, "MTK_DPRAM_CPU_BAS");
  fernvale_hook_live(0x58000000, "MTK_DPRAM2_CPU_BAS");

  // DSP memory  // IDMA port
  fernvale_hook_memory(0x60000000, "MTK_6223_DSP_MEM_BASE",
      &fernvale_live_mem_ops, 0xFFFFFF);
  fernvale_hook_memory(0x68000000, "MTK_6223_DSP2_MEM_BASE",
      &fernvale_live_mem_ops, 0xFFFFFF);

  fernvale_hook_memory(0x80010000, "MTK6223_EMI_BASE", &fernvale_emi_ops,
      0x10000);
  fernvale_hook_memory(0x80020000, "MTK6223_CIRQ_BASE", &fernvale_irq_ops,
      0x10000);
  fernvale_hook_memory(0x80000000, "MTK6223_CONFG_BASE", &fernvale_config_ops,
      0x10000);

  fernvale_hook_memory(0x80200000, "MTK6223_TDMA_BASE", &fernvale_tdma_ops,
      0x10000);

  fernvale_hook_memory(0x80040000, "MTK6223_RGU_BASE", &fernvale_rgu_ops,
      0x10000);

  fernvale_hook_memory(0x80210000, "MTK6223_RTC_BASE", &fernvale_rtc_ops,
      0x10000);

  fernvale_hook_f00d(0xf00d0000, "f00d");

  qemu_register_reset(fernvale_cpu_reset, cpu);
}



static QEMUMachine mt6223_machine =
  { .name = "mt6223", .desc = "6223 (ARM7EJ-S)", .init = mt6223_init, };

static void
mt6223_machine_init(void)
{
  qemu_register_machine(&mt6223_machine);
}

machine_init(mt6223_machine_init);


