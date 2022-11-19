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

#define SRV_N 6		// servo count
#define FLG_PZERO 1	// zero all servo hw pos

struct status_blk_t {
        uint16_t seq;
        uint16_t us;
        int32_t srv_pos[SRV_N]; // actual servo pos
        int16_t spe_pos;       // spindle pos (indexing)
        int16_t spe_spd;	// rpm
        uint16_t ins;           // input pin statuses
	uint8_t flags;
        uint8_t spe_load;
	uint32_t csum;

} __attribute__ ((aligned (4))) status_blk;

#define P_OUTS_EN_OFF 0		// servos en
#define P_OUTS_PWEN_OFF 8	// aux power en (sensors)
struct cmd_blk_t {
        uint8_t type;
        uint8_t scnt;   // servo cnt in srv_pos
        uint16_t seq;   // to detect missing cmd
        int32_t srv_pos[SRV_N]; // requested servo pos
        uint16_t spe_spd;       // requested spindle speed 0...0xffff
        uint8_t spe_rev;        // reverse spindle
	uint8_t leds;           // indicator leds status
        uint16_t outs;          // requested output pins states
	uint8_t flags;
        uint8_t _pad2;
	uint32_t csum;

} __attribute__ ((aligned (4))) cmd_blk;

#define LED_N 5
struct stcnc_data_t {
	hal_bit_t *led[LED_N];
	hal_bit_t *spe_rev;
	hal_float_t *spe_spd;
	hal_float_t *spe_rps;
	hal_float_t *spe_pos;
	hal_bit_t *spe_index_enable;
	hal_u32_t dbg[3];
	hal_float_t *srv_pos[SRV_N];
	hal_float_t *srv_fb[SRV_N];
	hal_bit_t *srv_en[SRV_N];
	hal_bit_t *outs[12];
	hal_bit_t *ins[16];

	hal_float_t srv_scale[SRV_N];
} static *stcnc_data;

static int spi;
struct spi_ioc_transfer xfer;
static int comp_id;		/* component ID */
static int inited;
static int prev_spe_pos;
static int16_t pos_offset;

static int spidev_open_and_configure(char *dev, int khz) 
{
    int fd = open(dev, O_RDWR),v;
    if (fd < 0) return -errno;

    v = 0;
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &v) < 0) goto fail_errno;
    v = SPI_MODE_0;
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

static uint32_t comp_sum(uint32_t *p,int cnt)
{
    uint32_t x = 0;
    for (int i=0;i<cnt;i++) x += p[i];
    return x;
}

static void update(void *arg, long period)
{
    int i;
    cmd_blk.seq++;
    cmd_blk.type = 1;
    cmd_blk.scnt = 4;
    cmd_blk.outs = 0;
    cmd_blk.spe_spd = *stcnc_data->spe_spd;
    cmd_blk.spe_rev = *stcnc_data->spe_rev;
    cmd_blk.leds = 0;
    cmd_blk.flags = inited ? 0 : FLG_PZERO;
    for (i=0;i<LED_N;i++) {
	    if (*stcnc_data->led[i]) cmd_blk.leds |= 1<<i;
    }
    cmd_blk.outs = 0;
    for (i=0;i<8;i++) {
	    if (*stcnc_data->outs[i]) cmd_blk.outs |= 0x100<<i;
    }
    hal_float_t sc[SRV_N];
    for (i=0;i<SRV_N;i++) {
	    sc[i] = stcnc_data->srv_scale[i] * 1024;
	    cmd_blk.srv_pos[i] = *stcnc_data->srv_pos[i] * sc[i];
	    if (*stcnc_data->srv_en[i]) cmd_blk.outs |= 1<<i;
    }
    cmd_blk.csum = comp_sum((uint32_t*)&cmd_blk,sizeof(cmd_blk)/4-1);
    xfer.tx_buf = (uint32_t)&cmd_blk;
    xfer.rx_buf = (uint32_t)&status_blk;
    i = ioctl(spi, SPI_IOC_MESSAGE(1), &xfer);
    if (i<=0) { 
	    stcnc_data->dbg[0]++;
	    return;
    }
    uint32_t sum = comp_sum((uint32_t*)&status_blk,sizeof(status_blk)/4-1);
    if (sum != status_blk.csum) {
	    stcnc_data->dbg[1]++;
	    return;
    }
    for (i=0;i<SRV_N;i++) {
	    *stcnc_data->srv_fb[i] = status_blk.srv_pos[i] / sc[i];
    }
    for (i=0;i<16;i++) {
	    *stcnc_data->ins[i] = status_blk.ins & (1<<i) ? 1 : 0;
    }
    *stcnc_data->spe_rps = status_blk.spe_spd;
    *stcnc_data->spe_pos = (status_blk.spe_pos+pos_offset) * (1.0/16);
    uint32_t pb = status_blk.spe_pos & 15;
    uint32_t pa = prev_spe_pos & 15;
    if (((pa == 15 && pb == 0) || (pa == 0 && pb == 15)) 
		    && *stcnc_data->spe_index_enable) {
	    *stcnc_data->spe_index_enable = 0;
	    pos_offset = -status_blk.spe_pos;
    }
    prev_spe_pos = status_blk.spe_pos;
    if (status_blk.flags & FLG_PZERO) inited = 1;
}

int rtapi_app_main(void)
{
    int rev,retval,n;

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
			      1, 0, comp_id);
    if (retval < 0) {
err:
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_STCNC: ERROR: funct/pin export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    for (n=0;n<LED_N;n++) {
	    if (hal_pin_bit_newf(HAL_IN,stcnc_data->led+n,
			    comp_id,"hal_pi_stcnc.led.%d",n) < 0) goto err;
    }
    if (hal_pin_float_new("hal_pi_stcnc.spe_spd",HAL_IN,
			    &stcnc_data->spe_spd,comp_id) < 0) goto err;
    if (hal_pin_bit_new("hal_pi_stcnc.spe_rev",HAL_IN,
			    &stcnc_data->spe_rev,comp_id) < 0) goto err;
    if (hal_pin_bit_new("hal_pi_stcnc.spe_index_enable",HAL_IO,
		    &stcnc_data->spe_index_enable,comp_id) < 0) goto err;
    if (hal_pin_float_new("hal_pi_stcnc.spe_rps",HAL_OUT,
			    &stcnc_data->spe_rps,comp_id) < 0) goto err;
    if (hal_pin_float_new("hal_pi_stcnc.spe_pos",HAL_OUT,
			    &stcnc_data->spe_pos,comp_id) < 0) goto err;
    for (n=0;n<SRV_N;n++) {
	stcnc_data->srv_scale[n] = 1.0;
    	if (hal_param_float_newf(HAL_RW,stcnc_data->srv_scale+n,
			comp_id,"hal_pi_stcnc.srv_scale.%d",n) < 0) goto err;
    	if (hal_pin_float_newf(HAL_IN,stcnc_data->srv_pos+n,
			comp_id,"hal_pi_stcnc.srv_pos.%d",n) < 0) goto err;
    	if (hal_pin_float_newf(HAL_OUT,stcnc_data->srv_fb+n,
			comp_id,"hal_pi_stcnc.srv_fb.%d",n) < 0) goto err;
    	if (hal_pin_bit_newf(HAL_IN,stcnc_data->srv_en+n,
			comp_id,"hal_pi_stcnc.en.%d",n) < 0) goto err;
    }
    for (n=0;n<16;n++) {
    	if (n<12 && hal_pin_bit_newf(HAL_IN,stcnc_data->outs+n,
			comp_id,"hal_pi_stcnc.outs.%d",n) < 0) goto err;
    	if (hal_pin_bit_newf(HAL_OUT,stcnc_data->ins+n,
			comp_id,"hal_pi_stcnc.ins.%d",n) < 0) goto err;
    }
    if (hal_param_u32_new("hal_pi_stcnc.dbg0",HAL_RO, stcnc_data->dbg,comp_id) < 0) goto err;
    if (hal_param_u32_new("hal_pi_stcnc.dbg1",HAL_RO, stcnc_data->dbg+1,comp_id) < 0) goto err;
    if (hal_param_u32_new("hal_pi_stcnc.dbg2",HAL_RO, stcnc_data->dbg+2,comp_id) < 0) goto err;

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
