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

#define FV_UART_BASE 0x80130000

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
      DeviceState *dev = qdev_create(NULL, TYPE_FERNVALE_UART);
      qdev_prop_set_chr(dev, "chardev", serial_hds[0]);
      qdev_init_nofail(dev);
      sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FV_UART_BASE);
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

#define UTSR1_RNE (1 << 1) /* Receive FIFO not empty */
#define UTSR1_TNF (1 << 2) /* Transmit FIFO not full */
#define UTSR1_PRE (1 << 3) /* Parity error */
#define UTSR1_FRE (1 << 4) /* Frame error */
#define UTSR1_ROR (1 << 5) /* Receive Over Run */

#define RX_FIFO_PRE (1 << 8)
#define RX_FIFO_FRE (1 << 9)
#define RX_FIFO_ROR (1 << 10)

typedef struct FernvaleUARTState
{
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
} FernvaleUARTState;

static void
fernvale_uart_update_status(FernvaleUARTState *s)
{
  uint16_t utsr1 = 0;

  if (s->tx_len != 8)
    {
      utsr1 |= UTSR1_TNF;
    }

  if (s->rx_len != 0)
    {
      uint16_t ent = s->rx_fifo[s->rx_start];

      utsr1 |= UTSR1_RNE;
      if (ent & RX_FIFO_PRE)
        {
          s->utsr1 |= UTSR1_PRE;
        }
      if (ent & RX_FIFO_FRE)
        {
          s->utsr1 |= UTSR1_FRE;
        }
      if (ent & RX_FIFO_ROR)
        {
          s->utsr1 |= UTSR1_ROR;
        }
    }

  s->utsr1 = utsr1;
}

static void
fernvale_uart_update_int_status(FernvaleUARTState *s)
{
  uint16_t utsr0 = s->utsr0 & (UTSR0_REB | UTSR0_RBB | UTSR0_RID);
  int i;

  if ((s->utcr3 & UTCR3_TXE) && (s->utcr3 & UTCR3_TIE) && s->tx_len <= 4)
    {
      utsr0 |= UTSR0_TFS;
    }

  if ((s->utcr3 & UTCR3_RXE) && (s->utcr3 & UTCR3_RIE) && s->rx_len > 4)
    {
      utsr0 |= UTSR0_RFS;
    }

  for (i = 0; i < s->rx_len && i < 4; i++)
    if (s->rx_fifo[(s->rx_start + i) % 12] & ~0xff)
      {
        utsr0 |= UTSR0_EIF;
        break;
      }

  s->utsr0 = utsr0;
  qemu_set_irq(s->irq, utsr0);
}

static void
fernvale_uart_rx_to(void *opaque)
{
  FernvaleUARTState *s = opaque;

  if (s->rx_len)
    {
      s->utsr0 |= UTSR0_RID;
      fernvale_uart_update_int_status(s);
    }
}

static void
fernvale_uart_rx_push(FernvaleUARTState *s, uint16_t c)
{
  if ((s->utcr3 & UTCR3_RXE) == 0)
    {
      /* rx disabled */
      return;
    }

  if (s->wait_break_end)
    {
      s->utsr0 |= UTSR0_REB;
      s->wait_break_end = false;
    }

  if (s->rx_len < 12)
    {
      s->rx_fifo[(s->rx_start + s->rx_len) % 12] = c;
      s->rx_len++;
    }
  else
    s->rx_fifo[(s->rx_start + 11) % 12] |= RX_FIFO_ROR;
}

static int
fernvale_uart_can_receive(void *opaque)
{
  FernvaleUARTState *s = opaque;

  if (s->rx_len == 12)
    {
      return 0;
    }
  /* It's best not to get more than 2/3 of RX FIFO, so advertise that much */
  if (s->rx_len < 8)
    {
      return 8 - s->rx_len;
    }
  return 1;
}

static void
fernvale_uart_receive(void *opaque, const uint8_t *buf, int size)
{
  FernvaleUARTState *s = opaque;
  int i;

  for (i = 0; i < size; i++)
    {
      fernvale_uart_rx_push(s, buf[i]);
    }

  /* call the timeout receive callback in 3 char transmit time */
  timer_mod(s->rx_timeout_timer,
      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 3);

  fernvale_uart_update_status(s);
  fernvale_uart_update_int_status(s);
}

static void
fernvale_uart_event(void *opaque, int event)
{
  FernvaleUARTState *s = opaque;
  if (event == CHR_EVENT_BREAK)
    {
      s->utsr0 |= UTSR0_RBB;
      fernvale_uart_rx_push(s, RX_FIFO_FRE);
      s->wait_break_end = true;
      fernvale_uart_update_status(s);
      fernvale_uart_update_int_status(s);
    }
}

static void
fernvale_uart_tx(void *opaque)
{
  FernvaleUARTState *s = opaque;
  uint64_t new_xmit_ts = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

  if (s->utcr3 & UTCR3_LBM) /* loopback */
    {
      fernvale_uart_receive(s, &s->tx_fifo[s->tx_start], 1);
    }
  else if (s->chr)
    {
      qemu_chr_fe_write(s->chr, &s->tx_fifo[s->tx_start], 1);
    }

  s->tx_start = (s->tx_start + 1) % 8;
  s->tx_len--;
  if (s->tx_len)
    {
      timer_mod(s->tx_timer, new_xmit_ts + s->char_transmit_time);
    }
  fernvale_uart_update_status(s);
  fernvale_uart_update_int_status(s);
}

static uint64_t
fernvale_uart_read(void *opaque, hwaddr addr, unsigned size)
{
  FernvaleUARTState *s = opaque;

  switch (addr)
    {

  case UART_RBR:
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
    return 0x20;

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

static void
fernvale_uart_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
  FernvaleUARTState *s = opaque;

  switch (addr)
    {
  case UART_RBR:
    if (s->LCR & (1 << 7))
      s->DLL = value;
    else
      putchar(value);
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

static const MemoryRegionOps fernvale_uart_ops =
  { .read = fernvale_uart_read, .write = fernvale_uart_write, .endianness =
      DEVICE_NATIVE_ENDIAN, };

static int
mt6223_uart_init(SysBusDevice *dev)
{
  FernvaleUARTState *s = FERNVALE_UART(dev);

  memory_region_init_io(&s->iomem, OBJECT(s), &fernvale_uart_ops, s, "uart",
      0x10000);
  sysbus_init_mmio(dev, &s->iomem);
  sysbus_init_irq(dev, &s->irq);

  s->rx_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fernvale_uart_rx_to,
      s);
  s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fernvale_uart_tx, s);

  if (s->chr)
    {
      qemu_chr_add_handlers(s->chr, fernvale_uart_can_receive,
          fernvale_uart_receive, fernvale_uart_event, s);
    }

  return 0;
}

static void
mt6223_uart_reset(DeviceState *dev)
{
  FernvaleUARTState *s = FERNVALE_UART(dev);

  s->utcr0 = UTCR0_DSS; /* 8 data, no parity */
  s->brd = 23; /* 9600 */
  /* enable send & recv - this actually violates spec */
  s->utcr3 = UTCR3_TXE | UTCR3_RXE;

  s->rx_len = s->tx_len = 0;

  //fernvale_uart_update_parameters(s);
  fernvale_uart_update_status(s);
  fernvale_uart_update_int_status(s);
}

static int
fernvale_uart_post_load(void *opaque, int version_id)
{
  FernvaleUARTState *s = opaque;

  //fernvale_uart_update_parameters(s);
  fernvale_uart_update_status(s);
  fernvale_uart_update_int_status(s);

  /* tx and restart timer */
  if (s->tx_len)
    {
      fernvale_uart_tx(s);
    }

  /* restart rx timeout timer */
  if (s->rx_len)
    {
      timer_mod(s->rx_timeout_timer,
          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->char_transmit_time * 3);
    }

  return 0;
}

static const VMStateDescription vmstate_fernvale_uart_regs =
  { .name = "mt6223-uart", .version_id = 0, .minimum_version_id = 0,
      .minimum_version_id_old = 0, .post_load = fernvale_uart_post_load,
      .fields = (VMStateField[]
            )
              {
              VMSTATE_UINT8(utcr0, FernvaleUARTState),
            VMSTATE_UINT16(brd, FernvaleUARTState),
          VMSTATE_UINT8(utcr3, FernvaleUARTState),
        VMSTATE_UINT8(utsr0, FernvaleUARTState),
      VMSTATE_UINT8_ARRAY(tx_fifo, FernvaleUARTState, 8),
    VMSTATE_UINT8(tx_start, FernvaleUARTState),
  VMSTATE_UINT8(tx_len, FernvaleUARTState),
VMSTATE_UINT16_ARRAY(rx_fifo, FernvaleUARTState, 12),
VMSTATE_UINT8(rx_start, FernvaleUARTState),
VMSTATE_UINT8(rx_len, FernvaleUARTState),
VMSTATE_BOOL(wait_break_end, FernvaleUARTState),
VMSTATE_END_OF_LIST(), } , };

static Property mt6223_uart_properties[] =
  {
  DEFINE_PROP_CHR("chardev", FernvaleUARTState, chr),
DEFINE_PROP_END_OF_LIST(), };

static void
mt6223_uart_class_init(ObjectClass *klass, void *data)
{
  DeviceClass *dc = DEVICE_CLASS(klass);
  SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

  k->init = mt6223_uart_init;
  dc->desc = "Fernvale UART controller";
  dc->reset = mt6223_uart_reset;
  dc->vmsd = &vmstate_fernvale_uart_regs;
  dc->props = mt6223_uart_properties;
}

static const TypeInfo fernvale_uart_info =
  { .name = TYPE_FERNVALE_UART, .parent = TYPE_SYS_BUS_DEVICE, .instance_size =
      sizeof(FernvaleUARTState), .class_init = mt6223_uart_class_init, };

static QEMUMachine mt6223_machine =
  { .name = "mt6223", .desc = "6223 (ARM7EJ-S)", .init = mt6223_init, };

static void
mt6223_machine_init(void)
{
  qemu_register_machine(&mt6223_machine);
}

machine_init(mt6223_machine_init);

static void
mt6223_register_types(void)
{
  type_register_static(&fernvale_uart_info);
#if 0
  type_register_static(&mv88w8618_pit_info);
  type_register_static(&mv88w8618_flashcfg_info);
  type_register_static(&mv88w8618_eth_info);
  type_register_static(&mv88w8618_wlan_info);
  type_register_static(&musicpal_lcd_info);
  type_register_static(&musicpal_gpio_info);
  type_register_static(&musicpal_key_info);
  type_register_static(&musicpal_misc_info);
#endif
}

type_init(mt6223_register_types)
