/**
 ********************************************************************
 * @file    hal_i2c.c
 * @brief   DJI PSDK I2C HAL for Jetson Orin NX (WeAct-N002)
 *
 *  Hardware mapping (Orin NX on WeAct-N002):
 *    I2C  : /dev/i2c-7  (I2C1_SCL=pin189, I2C1_SDA=pin190)
 *    NRST : GPIO2 = PAC.06 = gpiochip0 line 144
 *           sysfs number = gpiochip0_base(348) + line(144) = 492
 *           JetPack5+ sysfs path: /sys/class/gpio/PAC.06/
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hal_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <gpiod.h>

/* Private constants ---------------------------------------------------------*/
#define I2C_DEVICE_RESET_LOW_US    (25  * 1000)   /* NRST low pulse : 25 ms  */
#define I2C_DEVICE_BOOT_US         (200 * 1000)   /* chip boot delay: 200 ms */

/*
 * NRST = GPIO2 = PAC.06
 *   gpiochip0 (tegra234-gpio, 164 lines), base = 348
 *   line = 492 - 348 = 144
 *   JetPack5+ sysfs ID string: "PAC.06"
 *   sysfs numeric: 492
 */
#define I2C_RESET_GPIOCHIP_PATH    "/dev/gpiochip0"
#define I2C_RESET_LINE_NUM         (144)
#define I2C_RESET_SYSFS_GPIO_NUM   (492)
#define I2C_RESET_SYSFS_GPIO_ID    "PAC.06"       /* JetPack5+ path */

/* Hold gpiod handles so NRST stays driven HIGH between calls */
static struct gpiod_chip *s_resetChip = NULL;
static struct gpiod_line *s_resetLine = NULL;

/* Private types -------------------------------------------------------------*/
typedef struct {
    int32_t i2cFd;
} T_I2cHandleStruct;

/* Private functions declaration ---------------------------------------------*/
static void HalI2c_ResetDevice(void);
static void HalI2c_ResetDeviceSysfs(void);
static void HalI2c_ScanBus(int32_t fd);
static int  HalI2c_ProbeAddr(int32_t fd, uint16_t addr);

/* ========================================================================== */
/* Exported functions                                                          */
/* ========================================================================== */

T_DjiReturnCode HalI2c_Init(T_DjiHalI2cConfig i2cConfig, T_DjiI2cHandle *i2cHandle)
{
    (void)i2cConfig;

    T_I2cHandleStruct *i2CHandleStruct = NULL;

    printf("[HAL_I2C] Init: dev=%s  NRST=%s line %d (sysfs %s / gpio%d)\r\n",
           LINUX_I2C_DEV1,
           I2C_RESET_GPIOCHIP_PATH, I2C_RESET_LINE_NUM,
           I2C_RESET_SYSFS_GPIO_ID, I2C_RESET_SYSFS_GPIO_NUM);

    /* --- Reset SDK CC chip via gpiod ----------------------------------- */
    HalI2c_ResetDevice();

    /* If gpiod failed (line busy / permission), fall back to sysfs */
    if (!s_resetLine) {
        printf("[HAL_I2C] gpiod failed → sysfs fallback (%s / gpio%d)\r\n",
               I2C_RESET_SYSFS_GPIO_ID, I2C_RESET_SYSFS_GPIO_NUM);
        HalI2c_ResetDeviceSysfs();
    }

    /* --- Open I2C bus -------------------------------------------------- */
    i2CHandleStruct = malloc(sizeof(T_I2cHandleStruct));
    if (i2CHandleStruct == NULL) {
        printf("[HAL_I2C] ERROR: malloc failed\r\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    i2CHandleStruct->i2cFd = open(LINUX_I2C_DEV1, O_RDWR);
    if (i2CHandleStruct->i2cFd < 0) {
        printf("[HAL_I2C] ERROR: open(%s) failed: %s\r\n",
               LINUX_I2C_DEV1, strerror(errno));
        free(i2CHandleStruct);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    printf("[HAL_I2C] Opened %s  fd=%d\r\n", LINUX_I2C_DEV1, i2CHandleStruct->i2cFd);

    /* --- Scan bus ------------------------------------------------------ */
    HalI2c_ScanBus(i2CHandleStruct->i2cFd);

    int probe = HalI2c_ProbeAddr(i2CHandleStruct->i2cFd, 0x2A);
    printf("[HAL_I2C] Probe 0x2A (SDK CC) → %s\r\n",
           probe == 0 ? "ACK ✓  chip present" : "NACK ✗  chip NOT responding");

    *i2cHandle = i2CHandleStruct;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_DeInit(T_DjiI2cHandle i2cHandle)
{
    T_I2cHandleStruct *i2CHandleStruct = (T_I2cHandleStruct *) i2cHandle;

    if (i2CHandleStruct == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    close(i2CHandleStruct->i2cFd);
    free(i2CHandleStruct);

    if (s_resetLine) {
        gpiod_line_release(s_resetLine);
        s_resetLine = NULL;
    }
    if (s_resetChip) {
        gpiod_chip_close(s_resetChip);
        s_resetChip = NULL;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_WriteData(T_DjiI2cHandle i2cHandle, uint16_t devAddress,
                                 const uint8_t *buf, uint32_t len, uint32_t *realLen)
{
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages;
    int32_t ret;
    T_I2cHandleStruct *i2CHandleStruct = (T_I2cHandleStruct *) i2cHandle;

    if (i2CHandleStruct == NULL || buf == NULL || realLen == NULL || len == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    messages.addr  = devAddress;
    messages.flags = 0;
    messages.len   = len;
    messages.buf   = (uint8_t *) buf;
    data.msgs  = &messages;
    data.nmsgs = 1;

    ret = ioctl(i2CHandleStruct->i2cFd, I2C_RDWR, &data);
    if (ret < 0) {
        printf("[HAL_I2C] Write ERR: addr=0x%02X len=%u errno=%d (%s)\r\n",
               devAddress, len, errno, strerror(errno));
        *realLen = 0;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    printf("[HAL_I2C] Write OK : addr=0x%02X len=%u\r\n", devAddress, len);
    *realLen = (uint32_t)ret;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalI2c_ReadData(T_DjiI2cHandle i2cHandle, uint16_t devAddress,
                                uint8_t *buf, uint32_t len, uint32_t *realLen)
{
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages;
    int32_t ret;
    T_I2cHandleStruct *i2CHandleStruct = (T_I2cHandleStruct *) i2cHandle;

    if (i2CHandleStruct == NULL || buf == NULL || realLen == NULL || len == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    messages.addr  = devAddress;
    messages.flags = I2C_M_RD;
    messages.len   = len;
    messages.buf   = buf;
    data.msgs  = &messages;
    data.nmsgs = 1;

    ret = ioctl(i2CHandleStruct->i2cFd, I2C_RDWR, &data);
    if (ret < 0) {
        printf("[HAL_I2C] Read  ERR: addr=0x%02X len=%u errno=%d (%s)\r\n",
               devAddress, len, errno, strerror(errno));
        *realLen = 0;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    printf("[HAL_I2C] Read  OK : addr=0x%02X len=%u\r\n", devAddress, len);
    *realLen = (uint32_t)ret;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ========================================================================== */
/* Private functions                                                           */
/* ========================================================================== */

/*
 * HalI2c_ResetDevice  — gpiod version (preferred)
 *
 * NRST = GPIO2 = PAC.06 = gpiochip0 line 144
 * Line is kept driven HIGH after reset so the chip stays out of reset.
 */
static void HalI2c_ResetDevice(void)
{
    int ret;

    /* Re-use existing handle: just send another pulse */
    if (s_resetChip && s_resetLine) {
        gpiod_line_set_value(s_resetLine, 0);
        usleep(I2C_DEVICE_RESET_LOW_US);
        gpiod_line_set_value(s_resetLine, 1);
        usleep(I2C_DEVICE_BOOT_US);
        printf("[HAL_I2C] NRST pulse done (re-used handle)\r\n");
        return;
    }

    s_resetChip = gpiod_chip_open(I2C_RESET_GPIOCHIP_PATH);
    if (!s_resetChip) {
        printf("[HAL_I2C] gpiod_chip_open(%s) failed: %s\r\n",
               I2C_RESET_GPIOCHIP_PATH, strerror(errno));
        return;
    }
    printf("[HAL_I2C] gpiod: opened %s  label=\"%s\"  lines=%u\r\n",
           I2C_RESET_GPIOCHIP_PATH,
           gpiod_chip_label(s_resetChip),
           gpiod_chip_num_lines(s_resetChip));

    s_resetLine = gpiod_chip_get_line(s_resetChip, I2C_RESET_LINE_NUM);
    if (!s_resetLine) {
        printf("[HAL_I2C] gpiod_chip_get_line(%d) failed\r\n", I2C_RESET_LINE_NUM);
        gpiod_chip_close(s_resetChip);
        s_resetChip = NULL;
        return;
    }
    printf("[HAL_I2C] gpiod: line %d = \"%s\"\r\n",
           I2C_RESET_LINE_NUM,
           gpiod_line_name(s_resetLine) ? gpiod_line_name(s_resetLine) : "(none)");

    /* Request as output, initial value HIGH (not in reset) */
    ret = gpiod_line_request_output(s_resetLine, "psdk_i2c_rst", 1);
    if (ret < 0) {
        printf("[HAL_I2C] gpiod_line_request_output failed: %s\r\n", strerror(errno));
        printf("[HAL_I2C] Check if line is held by another process:\r\n");
        printf("[HAL_I2C]   sudo cat /sys/kernel/debug/gpio | grep PAC.06\r\n");
        gpiod_chip_close(s_resetChip);
        s_resetChip = NULL;
        s_resetLine = NULL;
        return;
    }

    /* Pulse NRST LOW then HIGH */
    gpiod_line_set_value(s_resetLine, 0);
    printf("[HAL_I2C] NRST LOW  (%d ms)...\r\n", I2C_DEVICE_RESET_LOW_US / 1000);
    usleep(I2C_DEVICE_RESET_LOW_US);

    gpiod_line_set_value(s_resetLine, 1);
    printf("[HAL_I2C] NRST HIGH, waiting %d ms for boot...\r\n",
           I2C_DEVICE_BOOT_US / 1000);
    usleep(I2C_DEVICE_BOOT_US);

    printf("[HAL_I2C] gpiod reset done (%s line %d = PAC.06 = gpio%d)\r\n",
           I2C_RESET_GPIOCHIP_PATH, I2C_RESET_LINE_NUM, I2C_RESET_SYSFS_GPIO_NUM);
}

/*
 * HalI2c_ResetDeviceSysfs  — sysfs fallback
 *
 * JetPack 5.x uses GPIO-ID path (/sys/class/gpio/PAC.06/).
 * The numeric export (492) is still needed to create the entry first.
 * GPIO stays exported and driven HIGH after reset (NOT unexported).
 */
static void HalI2c_ResetDeviceSysfs(void)
{
    char path[80];
    int  fd;

    printf("[HAL_I2C] sysfs reset: export gpio%d → /sys/class/gpio/%s/\r\n",
           I2C_RESET_SYSFS_GPIO_NUM, I2C_RESET_SYSFS_GPIO_ID);

    /* Export — EBUSY is fine (already exported) */
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) {
        char num[8];
        snprintf(num, sizeof(num), "%d", I2C_RESET_SYSFS_GPIO_NUM);
        write(fd, num, strlen(num));
        close(fd);
        usleep(100 * 1000);   /* wait for sysfs entry to appear */
    }

    /* Direction: out  — use GPIO ID path (JetPack 5+) */
    snprintf(path, sizeof(path), "/sys/class/gpio/%s/direction",
             I2C_RESET_SYSFS_GPIO_ID);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        /* Fallback to numeric path (JetPack 4) */
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction",
                 I2C_RESET_SYSFS_GPIO_NUM);
        fd = open(path, O_WRONLY);
    }
    if (fd < 0) {
        printf("[HAL_I2C] sysfs: cannot open direction: %s\r\n", strerror(errno));
        return;
    }
    write(fd, "out", 3);
    close(fd);

    /* Value path */
    snprintf(path, sizeof(path), "/sys/class/gpio/%s/value",
             I2C_RESET_SYSFS_GPIO_ID);
    if (access(path, W_OK) != 0) {
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value",
                 I2C_RESET_SYSFS_GPIO_NUM);
    }

    /* LOW */
    fd = open(path, O_WRONLY);
    if (fd < 0) { printf("[HAL_I2C] sysfs: cannot open value: %s\r\n", strerror(errno)); return; }
    write(fd, "0", 1);
    close(fd);
    printf("[HAL_I2C] sysfs: NRST LOW (%d ms)...\r\n", I2C_DEVICE_RESET_LOW_US / 1000);
    usleep(I2C_DEVICE_RESET_LOW_US);

    /* HIGH — keep driven, do NOT unexport */
    fd = open(path, O_WRONLY);
    if (fd < 0) { return; }
    write(fd, "1", 1);
    close(fd);
    printf("[HAL_I2C] sysfs: NRST HIGH, boot wait %d ms...\r\n",
           I2C_DEVICE_BOOT_US / 1000);
    usleep(I2C_DEVICE_BOOT_US);

    printf("[HAL_I2C] sysfs reset done (PAC.06 / gpio%d stays HIGH)\r\n",
           I2C_RESET_SYSFS_GPIO_NUM);
}

static int HalI2c_ProbeAddr(int32_t fd, uint16_t addr)
{
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg msg;
    uint8_t dummy = 0;

    msg.addr  = addr;
    msg.flags = 0;
    msg.len   = 0;
    msg.buf   = &dummy;
    data.msgs  = &msg;
    data.nmsgs = 1;

    return (ioctl(fd, I2C_RDWR, &data) < 0) ? -1 : 0;
}

static void HalI2c_ScanBus(int32_t fd)
{
    int found = 0;
    printf("[HAL_I2C] Scanning %s (0x08-0x77)...\r\n", LINUX_I2C_DEV1);
    printf("[HAL_I2C]      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    for (int row = 0; row <= 0x70; row += 16) {
        printf("[HAL_I2C] %02x: ", row);
        for (int col = 0; col < 16; col++) {
            int addr = row + col;
            if (addr < 0x08 || addr > 0x77) { printf("   "); continue; }
            if (HalI2c_ProbeAddr(fd, (uint16_t)addr) == 0) {
                printf("%02x ", addr);
                found++;
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    if (found == 0) {
        printf("[HAL_I2C] NO devices on bus. Check: power(3V3), SCL/SDA wiring, NRST\r\n");
    } else {
        printf("[HAL_I2C] Found %d device(s)\r\n", found);
    }
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/