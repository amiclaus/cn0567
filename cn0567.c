/***************************************************************************//**
 *   @file   cn0567.c
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include "cn0567.h"
#include "cn0567_config.h"
#include "error.h"
#include "timer.h"
#include "gpio.h"
#include "delay.h"
#include "util.h"
#include "uart_extra.h"
#include "power.h"
#include "irq_extra.h"
#include "spi_extra.h"
#include "app_config.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Set device to get timestamp for low frequency oscillator calibration.
 * @param [in] dev - The device structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t cn0567_calibrate_lfo_set_ts(struct cn0567_dev *dev)
{
	int32_t ret;
	uint16_t reg_data;

	/** Enable clock calibration circuitry */
	if(dev->chip_id == 0x02c2) {
		ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
					&reg_data);
		if(ret != SUCCESS)
			return FAILURE;
		reg_data |= BITM_OSC1M_OSC_CLK_CAL_ENA;
		ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
					 reg_data);
	}

	/** Enable GPIO0 input */
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_GPIO_CFG,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data |= (1 & BITM_GPIO_CFG_GPIO_PIN_CFG0);
	ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_GPIO_CFG,
				 reg_data);
	if(ret != SUCCESS)
		return FAILURE;

	/** Enable GPIO0 as time stamp input */
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_GPIO_EXT,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data |= ((0 << BITP_GPIO_EXT_TIMESTAMP_GPIO) &
		     BITM_GPIO_EXT_TIMESTAMP_GPIO);
	return adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_GPIO_EXT,
				  reg_data);
}

/**
 * @brief Get time stamp for low frequency oscillator calibration.
 * @param [in] dev - The device structure.
 * @param [out] ts_val - Pointer to the timestamp value container.
 * @param [in] ts_gpio - Descriptor for the counter start/stop GPIO.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t cn0567_calibrate_lfo_get_timestamp(struct cn0567_dev *dev,
		uint32_t *ts_val, struct gpio_desc *ts_gpio)
{
	int32_t ret;
	uint16_t reg_data;

	/** Start time stamp calibration */
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC32K,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data |= BITM_OSC32K_CAPTURE_TIMESTAMP;
	ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_OSC32K,
				 reg_data);
	if(ret != SUCCESS)
		return FAILURE;

	/** Give first time stamp trigger */
	ret = gpio_set_value(ts_gpio, GPIO_HIGH);
	if(ret != SUCCESS)
		return FAILURE;
	mdelay(1);
	ret = gpio_set_value(ts_gpio, GPIO_LOW);
	if(ret != SUCCESS)
		return FAILURE;

	/** Start time stamp calibration */
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC32K,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data |= BITM_OSC32K_CAPTURE_TIMESTAMP;
	ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_OSC32K,
				 reg_data);
	if(ret != SUCCESS)
		return FAILURE;

	mdelay(10);
	ret = gpio_set_value(ts_gpio, GPIO_HIGH);
	if(ret != SUCCESS)
		return FAILURE;
	mdelay(1);
	ret = gpio_set_value(ts_gpio, GPIO_LOW);
	if(ret != SUCCESS)
		return FAILURE;

	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC32K,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	if(reg_data & BITM_OSC32K_CAPTURE_TIMESTAMP)
		return FAILURE;

	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_STAMP_H,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	*ts_val = (reg_data << 16) & 0xFFFF0000;
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_STAMP_L,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	*ts_val |= reg_data;

	return SUCCESS;
}

/**
 * @brief Do low frequency oscillator calibration with respect to an external
 *        reference.
 * @param [in] dev - The device structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t cn0567_calibrate_lfo(struct cn0567_dev *dev)
{
	int32_t ret;
	uint32_t ts_val_current, ts_val_last = 0, ts_val;
	uint16_t reg_data, cal_value;
	int8_t rdy = 0;
	struct gpio_desc *ts_gpio;
	struct gpio_init_param ts_param;

	ret = cn0567_calibrate_lfo_set_ts(dev);
	if(ret != SUCCESS)
		return FAILURE;

	/** Setup platform GPIO for time stamp trigger */
	ts_param.number = 15;
	ts_param.extra = NULL;
	ret = gpio_get(&ts_gpio, &ts_param);
	if(ret != SUCCESS)
		return FAILURE;
	ret = gpio_direction_output(ts_gpio, GPIO_LOW);
	if(ret != SUCCESS)
		return FAILURE;

	/** Delay to correctly initialize GPIO circuitry in device. */
	mdelay(1);

	while (1) {
		ret = cn0567_calibrate_lfo_get_timestamp(dev, &ts_val_current,
				ts_gpio);
		if(ret != SUCCESS) {
			return FAILURE;
		}

		if(ts_val_current < ts_val_last) {
			ts_val_last = 0;
			continue;
		}
		ts_val = ts_val_current - ts_val_last;
		ts_val_last = ts_val_current;

		ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
					&reg_data);
		if(ret != SUCCESS) {
			return FAILURE;
		}
		cal_value = reg_data & BITM_OSC1M_OSC_1M_FREQ_ADJ;
		if(ts_val < (10000 - (10000 * 0.005)))
			cal_value++;
		else if(ts_val > (10000 + (10000 * 0.005)))
			cal_value--;
		else
			rdy = 1;
		if(rdy == 1)
			break;
		if((cal_value == 0) || (cal_value == BITM_OSC1M_OSC_1M_FREQ_ADJ))
			break;
		reg_data &= ~BITM_OSC1M_OSC_1M_FREQ_ADJ;
		reg_data |= cal_value & BITM_OSC1M_OSC_1M_FREQ_ADJ;
		ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
					 reg_data);
		if(ret != SUCCESS) {
			return FAILURE;
		}
	};

	ret = gpio_remove(ts_gpio);
	if(ret != SUCCESS) {
		return FAILURE;
	}

	if(rdy == 1)
		return SUCCESS;
	else
		return FAILURE;
}

/**
 * @brief Do high frequency oscillator calibration with respect to the low
 *        frequency oscillator.
 * @param [in] dev - The device structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t cn0567_calibrate_hfo(struct cn0567_dev *dev)
{
	int32_t ret;
	uint16_t reg_data;

	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC32M_CAL,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data |= BITM_OSC32M_CAL_OSC_32M_CAL_START;

	do {
		ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC32M_CAL,
					&reg_data);
		if(ret != SUCCESS)
			return FAILURE;
	} while(reg_data & BITM_OSC32M_CAL_OSC_32M_CAL_START);

	/** Disable clock calibration circuitry */
	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
				&reg_data);
	if(ret != SUCCESS)
		return FAILURE;
	reg_data &= ~BITM_OSC1M_OSC_CLK_CAL_ENA;
	ret = adpd410x_reg_write(dev->adpd4100_handler, ADPD410X_REG_OSC1M,
				 reg_data);

	return SUCCESS;
}

/**
 * @brief Initial process of the application.
 * @param [out] device - Pointer to the application handler.
 * @param [in] init_param - Pointer to the application initialization structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t cn0567_init(struct cn0567_dev **device)
{
	int32_t ret;
	struct cn0567_dev *dev;
	int8_t i;
	uint16_t data;
	uint32_t sw_flash_buffer[63];
	uint16_t reg_addr, reg_val;

	ret = pwr_setup();
	if(ret != SUCCESS)
		return FAILURE;

	dev = (struct cn0567_dev *)calloc(1, sizeof *dev);
	if(!dev)
		return FAILURE;

	ret = adpd410x_setup(&dev->adpd4100_handler,
			     &adpd4100_param);

	ret = adpd410x_reg_read(dev->adpd4100_handler, ADPD410X_REG_CHIP_ID,
				&dev->chip_id);

	ret = adpd410x_set_sampling_freq(dev->adpd4100_handler,
					 CN0567_CODE_ODR_DEFAULT);
	if(ret != SUCCESS)
		goto error_cn;

	ret = adpd410x_set_last_timeslot(dev->adpd4100_handler, ADPD410X_ACTIVE_TIMESLOTS - 1);
	if(ret != SUCCESS)
		goto error_cn;

	for (i = 0; i < ADPD410X_ACTIVE_TIMESLOTS; i++) {
		ret = adpd410x_timeslot_setup(dev->adpd4100_handler, i,
					      ts_init_tab + i);
		if(ret != SUCCESS)
			goto error_cn;

		/** Precondition VC1 and VC2 to TIA_VREF+250mV */
		ret = adpd410x_reg_read(dev->adpd4100_handler,
					ADPD410X_REG_CATHODE(i), &data);
		if(ret != SUCCESS)
			goto error_cn;
		data &= ~(BITM_CATHODE_A_VC2_SEL | BITM_CATHODE_A_VC1_SEL);
		data |= (2 << BITP_CATHODE_A_VC2_SEL) & BITM_CATHODE_A_VC2_SEL;
		data |= (2 << BITP_CATHODE_A_VC1_SEL) & BITM_CATHODE_A_VC1_SEL;
		ret = adpd410x_reg_write(dev->adpd4100_handler,
					 ADPD410X_REG_CATHODE(i), data);
		if(ret != SUCCESS)
			goto error_cn;

		/** Set the two channels trim option */
		ret = adpd410x_reg_read(dev->adpd4100_handler,
					ADPD410X_REG_AFE_TRIM(i), &data);
		if(ret != SUCCESS)
			goto error_cn;
		data |= (1 << BITP_AFE_TRIM_A_CH1_TRIM_INT |
			 1 << BITP_AFE_TRIM_A_CH2_TRIM_INT);
		ret = adpd410x_reg_write(dev->adpd4100_handler,
					 ADPD410X_REG_AFE_TRIM(i), data);
		if(ret != SUCCESS)
			goto error_cn;

		/**
		 * Set to ~32us integrator offset to line up zero crossing of
		 * BPF
		 */
		ret = adpd410x_reg_read(dev->adpd4100_handler,
					ADPD410X_REG_INTEG_OFFSET(i),
					&data);
		if(ret != SUCCESS)
			goto error_cn;
		data = 0x03FC & BITM_INTEG_OFFSET_A_INTEG_OFFSET;
		ret = adpd410x_reg_write(dev->adpd4100_handler,
					 ADPD410X_REG_INTEG_OFFSET(i), data);
		if(ret != SUCCESS)
			goto error_cn;

		/** Set to ~32us LED offset */
		ret = adpd410x_reg_read(dev->adpd4100_handler,
					ADPD410X_REG_LED_PULSE(i), &data);
		if(ret != SUCCESS)
			goto error_cn;
		data = 0x0220;
		ret = adpd410x_reg_write(dev->adpd4100_handler,
					 ADPD410X_REG_LED_PULSE(i), data);
		if(ret != SUCCESS)
			goto error_cn;
	}

	ret = cn0567_calibrate_lfo(dev);
	if(ret != SUCCESS)
		goto error_cn;

	ret = cn0567_calibrate_hfo(dev);
	if(ret != SUCCESS)
		goto error_cn;

	/** General configuration */
	sw_flash_buffer[0]   = 0x00f8000;
	sw_flash_buffer[1]   = 0x00f0006;
	sw_flash_buffer[2]   = 0x0000048;
	sw_flash_buffer[3]   = 0x001000f;
	sw_flash_buffer[4]   = 0x00d4e20;
	sw_flash_buffer[5]   = 0x00e0000;
	sw_flash_buffer[6]   = 0x0100300;
	/** Optical path 1 */
	sw_flash_buffer[7]   = 0x1020005;
	sw_flash_buffer[8]   = 0x1057070;
	sw_flash_buffer[9]   = 0x1060000;
	/** Optical path 2 */
	sw_flash_buffer[10]   = 0x1220050;
	sw_flash_buffer[11]   = 0x1250000;
	sw_flash_buffer[12]   = 0x1260030;
	/** Optical path 3 */
	sw_flash_buffer[13]   = 0x1420500;
	sw_flash_buffer[14]   = 0x14580B0;
	sw_flash_buffer[15]   = 0x1460000;
	/** Optical path 4 */
	sw_flash_buffer[16]   = 0x1625000;
	sw_flash_buffer[17]   = 0x1650000;
	sw_flash_buffer[18]   = 0x16680B0;
	/** AFE Path */
	sw_flash_buffer[19]   = 0x10140DA;
	sw_flash_buffer[20]   = 0x12140DA;
	sw_flash_buffer[21]   = 0x14140DA;
	sw_flash_buffer[22]   = 0x16140DA;
	/** CH2 enable */
	sw_flash_buffer[23]   = 0x1004000;
	sw_flash_buffer[24]   = 0x1204000;
	sw_flash_buffer[25]   = 0x1404000;
	sw_flash_buffer[26]   = 0x1604000;
	/** Precondition PDs to TIA_VREF */
	sw_flash_buffer[27]   = 0x1031000;
	sw_flash_buffer[28]   = 0x1231000;
	sw_flash_buffer[29]   = 0x1431000;
	sw_flash_buffer[30]   = 0x1631000;
	/** AFE gain */
	sw_flash_buffer[31]   = 0x1042A92;
	sw_flash_buffer[32]   = 0x1242A92;
	sw_flash_buffer[33]   = 0x1442A92;
	sw_flash_buffer[34]   = 0x1642A92;
	/** 1 integrate for every 32 pulses */
	sw_flash_buffer[35]   = 0x1070120;
	sw_flash_buffer[36]   = 0x1270120;
	sw_flash_buffer[37]   = 0x1470120;
	sw_flash_buffer[38]   = 0x1670120;
	/** 2us LED pulse with 32us offset */
	sw_flash_buffer[39]   = 0x1090220;
	sw_flash_buffer[40]   = 0x1290220;
	sw_flash_buffer[41]   = 0x1490220;
	sw_flash_buffer[42]   = 0x1690220;
	/** 3us AFE width double sided */
	sw_flash_buffer[43]   = 0x10A0003;
	sw_flash_buffer[44]   = 0x12A0003;
	sw_flash_buffer[45]   = 0x14A0003;
	sw_flash_buffer[46]   = 0x16A0003;
	/** ~32us integrator offset to line up zero crossing of BPF */
	sw_flash_buffer[47]   = 0x10B03FC;
	sw_flash_buffer[48]   = 0x12B03FC;
	sw_flash_buffer[49]   = 0x14B03FC;
	sw_flash_buffer[50]   = 0x16B03FC;
	/** 4 pulse chop */
	sw_flash_buffer[51]   = 0x10D00AA;
	sw_flash_buffer[52]   = 0x12D00AA;
	sw_flash_buffer[53]   = 0x14D00AA;
	sw_flash_buffer[54]   = 0x16D00AA;
	/** 2048 digital offset */
	sw_flash_buffer[55]   = 0x10f8000;
	sw_flash_buffer[56]   = 0x12f8000;
	sw_flash_buffer[57]   = 0x14f8000;
	sw_flash_buffer[58]   = 0x16f8000;
	/** 4 byte wide data */
	sw_flash_buffer[59]   = 0x1100004;
	sw_flash_buffer[60]   = 0x1300004;
	sw_flash_buffer[61]   = 0x1500004;
	sw_flash_buffer[62]   = 0x1700004;

	for (i = 0; i < 63; i++) {
			reg_addr = (sw_flash_buffer[i] & 0xFFFF0000) >> 16;
			reg_val  = sw_flash_buffer[i] & 0x0000FFFF;
			ret = adpd410x_reg_write(dev->adpd4100_handler, reg_addr,
						 reg_val);
			if (ret != SUCCESS)
				return FAILURE;
		}

	*device = dev;

	return ret;

error_cn:
	free(dev);

	return FAILURE;
}

/**
 * @brief Free memory allocated by cn0567_init().
 * @param [in] dev - The device structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t cn0567_remove(struct cn0567_dev *dev)
{
	int32_t ret;

	if(!dev)
		return FAILURE;

	ret = adpd410x_remove(dev->adpd4100_handler);
	if(ret != SUCCESS)
		return FAILURE;

	free(dev);

	return SUCCESS;
}
