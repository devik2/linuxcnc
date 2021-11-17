/********************************************************************
* Description:  hal_pi_stcnc.c
*               Driver for the RPi connected STM32G4 SPI board
*
* Author: Martin Devera (C) 2021
* License: GPL Version 2
*
* some code taken from::
*
* Jeff Epler <jepler@unpythonic.net> (hm2_spi.c)
* Copyright 2014
* Author: Michael Haberler (hal_pi_gpio.c)
* Copyright (c) 2012.
* Author: Mike McCauley (mikem@open.com.au)
* Copyright (C) 2011 Mike McCauley
* see http://www.open.com.au/mikem/bcm2835/
* Copyright (c) 2012 Ben Croston - cpuinfo.*
*
********************************************************************/


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_bitops.h"
#include "rtapi_app.h"		/* RTAPI realtime module decls */
                                /* this also includes config.h */
#include "hal.h"		/* HAL public API decls */
#include "cpuinfo.h"

#define RTAPI_BIT(nr)           (1UL << (nr))

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

MODULE_AUTHOR("Martin Devera");
MODULE_DESCRIPTION("Driver for RPi STM32G4 CNC board");
MODULE_LICENSE("GPL");

// port direction bits, 1=output
static char *spidev_path = "/dev/spidev0.0";
RTAPI_MP_STRING(spidev_path, "path to spi device");

// exclude pins from usage
static int spidev_rate = 10000; 
RTAPI_MP_INT(spidev_rate, "SPI rate in kHz");

#define SRV_N 4 // servo count

struct status_blk_t {
        uint16_t seq;
        uint16_t us;
        uint32_t srv_pos[SRV_N]; // actual servo pos
        uint16_t spe_pos;       // spindle pos (indexing)
        uint16_t _pad1;
        uint16_t ins;           // input pin statuses

} __attribute__ ((aligned (4))) status_blk;

struct cmd_blk_t {
        uint8_t type;
        uint8_t scnt;   // servo cnt in srv_pos
        uint16_t seq;   // to detect missing cmd
        uint32_t srv_pos[SRV_N]; // requested servo pos
        uint16_t spe_spd;       // requested spindle speed 0...0xffff
        uint8_t spe_rev;        // reverse spindle
        uint8_t _pad1;
        uint16_t outs;          // requested output pins states

} __attribute__ ((aligned (4))) cmd_blk;

struct stcnc_data_t {
	hal_bit_t *led;
	hal_u32_t *spe_spd;
	hal_u32_t dbg,*dbgp;
	hal_s32_t *srv0;
} static *stcnc_data;

static int spi;
struct spi_ioc_transfer xfer;
static int comp_id;		/* component ID */

static int spidev_open_and_configure(char *dev, int khz) 
{
    int fd = open(dev, O_RDWR),v;
    if (fd < 0) return -errno;

    v = 0;
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &v) < 0) goto fail_errno;
    if (ioctl(fd, SPI_IOC_WR_MODE, &v) < 0) goto fail_errno;
    v = 1000*khz;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &v) < 0) goto fail_errno;
    v = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &v) < 0) goto fail_errno;
    return fd;

fail_errno:
    close(fd);
    return -errno;
}

static void update(void *arg, long period)
{
    cmd_blk.seq++;
    cmd_blk.type = 1;
    cmd_blk.scnt = 4;
    cmd_blk.spe_spd = *stcnc_data->spe_spd;
    cmd_blk.srv_pos[0] = *stcnc_data->srv0;
    xfer.tx_buf = (uint32_t)&cmd_blk;
    xfer.rx_buf = (uint32_t)&status_blk;
    stcnc_data->dbg = ioctl(spi, SPI_IOC_MESSAGE(1), &xfer)+1;
}

int rtapi_app_main(void)
{
//    int n;
    int rev,retval;

    if ((rev = get_rpi_revision()) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
		      "unrecognized Raspberry revision, see /proc/cpuinfo\n");
      return -EINVAL;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "RPi rev %d", rev);

    switch (rev) {
    case 5:
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry4\n");
      break;
    default:
	rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_PI_STCNC: ERROR: board revision %d not supported\n", rev);
	return -EINVAL;
    }
    comp_id = hal_init("hal_pi_stcnc");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_STCNC: ERROR: hal_init() failed\n");
	return -1;
    }
    spi = spidev_open_and_configure(spidev_path,spidev_rate);
    if (spi < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_STCNC: ERROR: SPI open failed: %d\n",spi);
	hal_exit(comp_id);
	return -1;
    }
    stcnc_data = hal_malloc(sizeof(*stcnc_data));
    if (!stcnc_data) goto err;

    retval = hal_export_funct("hal_pi_stcnc.update", update, 0,
			      0, 0, comp_id);
    if (retval < 0) {
err:
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_STCNC: ERROR: funct/pin export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    if (hal_pin_bit_new("hal_pi_stcnc.led",HAL_IN,
			    &stcnc_data->led,comp_id) < 0) goto err;
    if (hal_pin_u32_new("hal_pi_stcnc.spe_spd",HAL_IN,
			    &stcnc_data->spe_spd,comp_id) < 0) goto err;
    if (hal_pin_s32_new("hal_pi_stcnc.pos.0",HAL_IN,
			    &stcnc_data->srv0,comp_id) < 0) goto err;
    if (hal_pin_u32_new("hal_pi_stcnc.dbg",HAL_OUT,
			    &stcnc_data->dbgp,comp_id) < 0) goto err;
    stcnc_data->dbgp = &stcnc_data->dbg;
    stcnc_data->dbg = 7;

    memset(&xfer,0,sizeof(xfer));
    xfer.len = sizeof(cmd_blk);
    xfer.speed_hz = 1000*spidev_rate;
    xfer.tx_nbits = xfer.rx_nbits = 8;
    xfer.bits_per_word = 8;

    memset(&cmd_blk,0,sizeof(cmd_blk));

    rtapi_print_msg(RTAPI_MSG_INFO,
	"HAL_PI_STCNC: installed driver\n");
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
