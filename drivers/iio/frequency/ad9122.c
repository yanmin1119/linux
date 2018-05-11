/*
 * AD9122 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9122.h"
#include "cf_axi_dds.h"

const char clk_names[CLK_NUM][10] = {"data_clk", "dac_clk", "ref_clk"};

static const unsigned char ad9122_reg_defaults[][2] = {
	{AD9122_REG_COMM, 0x00},
	{AD9122_REG_COMM, AD9122_COMM_RESET},
	{AD9122_REG_COMM, 0x00},
	{AD9122_REG_POWER_CTRL, AD9122_POWER_CTRL_PD_AUX_ADC},
	{AD9122_REG_DATA_FORMAT, AD9122_DATA_FORMAT_BINARY},
	{AD9122_REG_INTERRUPT_EN_1, 0x00},
	{AD9122_REG_INTERRUPT_EN_2, 0x00},
	{AD9122_REG_CLK_REC_CTRL, AD9122_CLK_REC_CTRL_DACCLK_CROSS_CORRECTION |
				  AD9122_CLK_REC_CTRL_REFCLK_CROSS_CORRECTION |
				  0xF},
	{AD9122_REG_PLL_CTRL_1, AD9122_PLL_CTRL_1_PLL_MANUAL_EN},
	{AD9122_REG_PLL_CTRL_2, AD9122_PLL_CTRL_2_PLL_LOOP_BANDWIDTH(3) |
				AD9122_PLL_CTRL_2_PLL_CHARGE_PUMP_CURRENT(0x11)},
	{AD9122_REG_PLL_CTRL_3, AD9122_PLL_CTRL_3_N2(3) |
				AD9122_PLL_CTRL_3_PLL_CROSS_CTRL_EN |
				AD9122_PLL_CTRL_3_N0(1) |
				AD9122_PLL_CTRL_3_N1(2)},
	{AD9122_REG_SYNC_CTRL_1, AD9122_SYNC_CTRL_1_DATA_FIFO_RATE_TOGGLE |
				 AD9122_SYNC_CTRL_1_RISING_EDGE_SYNC},
	{AD9122_REG_SYNC_CTRL_2, 0x00},
	{AD9122_REG_DCI_DELAY, 0x00},
	{AD9122_REG_FIFO_CTRL, AD9122_FIFO_CTRL_FIFO_PHA_OFFSET(4)},
	{AD9122_REG_FIFO_STATUS_1, 0x00},
	{AD9122_REG_DATAPATH_CTRL, AD9122_DATAPATH_CTRL_BYPASS_PREMOD |
				   AD9122_DATAPATH_CTRL_BYPASS_INV_SINC |
				   AD9122_DATAPATH_CTRL_BYPASS_NCO},
	{AD9122_REG_HB1_CTRL, AD9122_HB1_CTRL_BYPASS_HB1},
	{AD9122_REG_HB2_CTRL, AD9122_HB2_CTRL_BYPASS_HB2},
	{AD9122_REG_HB3_CTRL, AD9122_HB3_CTRL_BYPASS_HB3},
	{AD9122_REG_FTW_7_0, 0x00},
	{AD9122_REG_FTW_15_8, 0x00},
	{AD9122_REG_FTW_23_16, 0x00},
	{AD9122_REG_FTW_31_24, 0x00},
	{AD9122_REG_NCO_PHA_OFFSET_LSB, 0x00},
	{AD9122_REG_NCO_PHA_OFFSET_MSB, 0x00},
	{AD9122_REG_NCO_FTW_UPDATE, 0x00},
	{AD9122_REG_I_PHA_ADJ_LSB, 0x00},
	{AD9122_REG_I_PHA_ADJ_MSB, 0x00},
	{AD9122_REG_Q_PHA_ADJ_LSB, 0x00},
	{AD9122_REG_Q_PHA_ADJ_MSB, 0x00},
	{AD9122_REG_I_DAC_OFFSET_LSB, 0x00},
	{AD9122_REG_I_DAC_OFFSET_MSB, 0x00},
	{AD9122_REG_Q_DAC_OFFSET_LSB, 0x00},
	{AD9122_REG_Q_DAC_OFFSET_MSB, 0x00},
	{AD9122_REG_I_DAC_FS_ADJ, 0xF9},
	{AD9122_REG_I_DAC_CTRL, 0x01},
	{AD9122_REG_I_AUX_DAC_DATA, 0x00},
	{AD9122_REG_I_AUX_DAC_CTRL, 0x00},
	{AD9122_REG_Q_DAC_FS_ADJ, 0xF9},
	{AD9122_REG_Q_DAC_CTRL, 0x01},
	{AD9122_REG_Q_AUX_DAC_DATA, 0x00},
	{AD9122_REG_Q_AUX_DAC_CTRL, 0x00},
	{AD9122_REG_DIE_TEMP_RANGE_CTRL, AD9122_DIE_TEMP_RANGE_CTRL_REF_CURRENT(1)},
	{AD9122_REG_FIFO_STATUS_1, AD9122_FIFO_STATUS_1_FIFO_SOFT_ALIGN_REQ},
};

struct ad9122_sed {
	unsigned short i0;
	unsigned short q0;
	unsigned short i1;
	unsigned short q1;
};

static struct ad9122_sed dac_sed_pattern[5] = {
	{
		.i0 = 0x5555,
		.q0 = 0xAAAA,
		.i1 = 0xAAAA,
		.q1 = 0x5555,
	},
	{
		.i0 = 0,
		.q0 = 0,
		.i1 = 0xFFFF,
		.q1 = 0xFFFF,
	},
	{
		.i0 = 0,
		.q0 = 0,
		.i1 = 0,
		.q1 = 0,
	},
	{
		.i0 = 0xFFFF,
		.q0 = 0xFFFF,
		.i1 = 0xFFFF,
		.q1 = 0xFFFF,
	},
	{
		.i0 = 0x1248,
		.q0 = 0xEDC7,
		.i1 = 0xEDC7,
		.q1 = 0x1248,
	}
};

static int ad9122_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[2];
	int ret;

	buf[0] = 0x80 | reg;

	ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);
	if (ret < 0)
		return ret;

	return buf[1];
}

static int ad9122_write(struct spi_device *spi,
			 unsigned reg, unsigned val)
{
	unsigned char buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;
	ret = spi_write_then_read(spi, buf, 2, NULL, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9122_find_dci(unsigned long *err_field, unsigned entries)
{
	int dci, cnt, start, max_start, max_cnt;
	bool valid = false;
	char str[33];
	int ret;

	for(dci = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		dci < entries; dci++) {
		if (test_bit(dci, err_field) == 0) {
			if (start == -1)
				start = dci;
			cnt++;
			str[dci] = 'o';
			valid = true;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
			str[dci] = '-';
		}
	}
	str[dci] = 0;

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}


	ret = max_start + ((max_cnt - 1) / 2);

	str[ret] = '|';

	printk("%s DCI %d\n",str, ret);

	if (!valid)
		return -EIO;

	return ret;
}

static int ad9122_tune_dci(struct cf_axi_converter *conv)
{
	unsigned reg;
	int i = 0, dci;
	unsigned long err_bfield = 0;

	if (!conv->pcore_set_sed_pattern)
		return -ENODEV;

	for (dci = 0; dci < 4; dci++) {
		ad9122_write(conv->spi, AD9122_REG_DCI_DELAY, dci);
		for (i = 0; i < ARRAY_SIZE(dac_sed_pattern); i++) {

			ad9122_write(conv->spi, AD9122_REG_SED_CTRL, 0);

			conv->pcore_set_sed_pattern(conv->indio_dev,
				(dac_sed_pattern[i].i1 << 16) | dac_sed_pattern[i].i0,
				(dac_sed_pattern[i].q1 << 16) | dac_sed_pattern[i].q0);

			ad9122_write(conv->spi, AD9122_REG_COMPARE_I0_LSBS,
				dac_sed_pattern[i].i0 & 0xFF);
			ad9122_write(conv->spi, AD9122_REG_COMPARE_I0_MSBS,
				dac_sed_pattern[i].i0 >> 8);

			ad9122_write(conv->spi, AD9122_REG_COMPARE_Q0_LSBS,
				dac_sed_pattern[i].q0 & 0xFF);
			ad9122_write(conv->spi, AD9122_REG_COMPARE_Q0_MSBS,
				dac_sed_pattern[i].q0 >> 8);

			ad9122_write(conv->spi, AD9122_REG_COMPARE_I1_LSBS,
				dac_sed_pattern[i].i1 & 0xFF);
			ad9122_write(conv->spi, AD9122_REG_COMPARE_I1_MSBS,
				dac_sed_pattern[i].i1 >> 8);

			ad9122_write(conv->spi, AD9122_REG_COMPARE_Q1_LSBS,
				dac_sed_pattern[i].q1 & 0xFF);
			ad9122_write(conv->spi, AD9122_REG_COMPARE_Q1_MSBS,
				dac_sed_pattern[i].q1 >> 8);


			ad9122_write(conv->spi, AD9122_REG_SED_CTRL,
				    AD9122_SED_CTRL_SED_COMPARE_EN);

 			ad9122_write(conv->spi, AD9122_REG_EVENT_FLAG_2,
 				    AD9122_EVENT_FLAG_2_AED_COMPARE_PASS |
				    AD9122_EVENT_FLAG_2_AED_COMPARE_FAIL |
				    AD9122_EVENT_FLAG_2_SED_COMPARE_FAIL);

			ad9122_write(conv->spi, AD9122_REG_SED_CTRL,
				AD9122_SED_CTRL_SED_COMPARE_EN |
				AD9122_SED_CTRL_AUTOCLEAR_EN);

			msleep(100);

			reg = ad9122_read(conv->spi, AD9122_REG_SED_CTRL);

			if(!(reg & (AD9122_SED_CTRL_SAMPLE_ERR_DETECTED | AD9122_SED_CTRL_COMPARE_PASS)))
			{
				return -1;
			}

			if (reg & AD9122_SED_CTRL_SAMPLE_ERR_DETECTED)
				set_bit(dci, &err_bfield);
		}
	}

	dci = ad9122_find_dci(&err_bfield, 4);
	if (dci < 0) {
		dev_err(&conv->spi->dev, "Failed DCI calibration");
		ad9122_write(conv->spi, AD9122_REG_DCI_DELAY, 0);
	}  else {
		ad9122_write(conv->spi, AD9122_REG_DCI_DELAY, dci);
	}

	ad9122_write(conv->spi, AD9122_REG_SED_CTRL, 0);

	return dci;
}

static int ad9122_get_fifo_status(struct cf_axi_converter *conv)
{
	unsigned stat;

	stat = ad9122_read(conv->spi, AD9122_REG_FIFO_STATUS_1);
	if (stat & (AD9122_FIFO_STATUS_1_FIFO_WARNING_1 |
		AD9122_FIFO_STATUS_1_FIFO_WARNING_2)) {
		return -1;
	}
	return 0;
}

static int ad9122_sync(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	int ret, timeout;

	timeout = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, AD9122_REG_FIFO_STATUS_1);
		if (ret < 0)
			return ret;

	} while (timeout-- && !(ret & AD9122_FIFO_STATUS_1_FIFO_SOFT_ALIGN_ACK));

	ad9122_write(spi, AD9122_REG_FIFO_STATUS_1, 0x0);
	ad9122_write(spi, AD9122_REG_SYNC_CTRL_1,
		     AD9122_SYNC_CTRL_1_SYNC_EN |
		     AD9122_SYNC_CTRL_1_RISING_EDGE_SYNC);

	timeout = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, AD9122_REG_SYNC_STATUS_1);
		if (ret < 0)
			return ret;

	} while (timeout-- && !(ret & AD9122_SYNC_STATUS_1_SYNC_LOCKED));

	return 0;
}

static int ad9122_setup(struct cf_axi_converter *conv, unsigned mode)
{
	struct spi_device *spi = conv->spi;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad9122_reg_defaults); i++)
			ad9122_write(spi, ad9122_reg_defaults[i][0],
					     ad9122_reg_defaults[i][1]);

	return ad9122_sync(conv);
}


static int ad9122_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < CLK_NUM; i++) {
		clk = clk_get(&conv->spi->dev, &clk_names[i][0]);
		if (IS_ERR(clk)) {
			return -EPROBE_DEFER;
		}

		ret = clk_prepare_enable(clk);
		if (ret < 0)
			return ret;

		conv->clk[i] = clk;
	}
	return 0;
}

static unsigned long long ad9122_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DATA]);
}

static void ad9122_update_avail_intp_modes(struct cf_axi_converter *conv,
					 unsigned long dat_freq)
{
	unsigned long dac_freq;
	long r_dac_freq;
	int intp, i;

	for (i = 0, intp = 1; intp <= 8; intp *= 2) {
		dac_freq = dat_freq * intp;
		if (dac_freq > AD9122_MAX_DAC_RATE) {
			break;
		}
		r_dac_freq = clk_round_rate(conv->clk[CLK_DAC], dac_freq);
		if (r_dac_freq != dac_freq)
			continue;
		else
			conv->intp_modes[i++] = dac_freq;
	}

	conv->intp_modes[i] = 0;
}

static void ad9122_update_avail_fcent_modes(struct cf_axi_converter *conv,
					  unsigned long dat_freq)
{
	int i;

	if (conv->interp_factor == 1) {
		conv->cs_modes[0] = 0;
		conv->cs_modes[1] = -1;
		return;
	}

	for (i = 0; i < (conv->interp_factor * 2); i++) {
		conv->cs_modes[i] = (dat_freq * i) / 2;
	}

	conv->cs_modes[i] = -1;
}

static int ad9122_set_data_clk(struct cf_axi_converter *conv, unsigned long freq)
{
	unsigned long dac_freq;
	long dat_freq, r_dac_freq;
	int ret;

	dat_freq = clk_round_rate(conv->clk[CLK_DATA], freq);
	if (dat_freq < 0 || dat_freq > AD9122_MAX_DAC_RATE) {
		dev_err(&conv->spi->dev,
			"CLK_DATA: Error or requested rate exceeds maximum %ld (%lu)",
			dat_freq, AD9122_MAX_DAC_RATE);
		return -EINVAL;
	}

	dac_freq = dat_freq * conv->interp_factor;
	if (dac_freq > AD9122_MAX_DAC_RATE) {
		dev_err(&conv->spi->dev,
			"CLK_DAC: Requested Rate exceeds maximum %lu (%lu)",
			dac_freq, AD9122_MAX_DAC_RATE);
		return -EINVAL;
	}

	r_dac_freq = clk_round_rate(conv->clk[CLK_DAC], dac_freq);
	if (r_dac_freq != dac_freq) {
		dev_err(&conv->spi->dev,
			"CLK_DAC: Requested Rate exceeds mismatch %ld (%lu)",
			r_dac_freq, dac_freq);
		return -EINVAL;
	}

	ret = clk_set_rate(conv->clk[CLK_DATA], dat_freq);
	if (ret < 0)
		return ret;

	ret = clk_set_rate(conv->clk[CLK_DAC], dac_freq);
	if (ret < 0)
		return ret;

	ad9122_update_avail_fcent_modes(conv, dat_freq);
	ad9122_update_avail_intp_modes(conv, dat_freq);

	return 0;
}

static unsigned int ad9122_validate_interp_factor(unsigned fact)
{
	switch (fact) {
		case 1:
		case 2:
		case 4:
		case 8:
			return fact;
		default:
			return 1;
	}
}

static int __ad9122_set_interpol(struct cf_axi_converter *conv, unsigned interp,
			       unsigned fcent_shift, unsigned long data_rate)
{
	unsigned char hb1, hb2, hb3, tmp;
	int ret, cached;

	hb1 = AD9122_HB1_CTRL_BYPASS_HB1;
	hb2 = AD9122_HB2_CTRL_BYPASS_HB2;
	hb3 = AD9122_HB3_CTRL_BYPASS_HB3;

	switch (interp) {
		case 1:
			break;
		case 2:
			if (fcent_shift > 3)
				return -EINVAL;
			hb1 = AD9122_HB1_INTERP(fcent_shift);
			break;
		case 4:
			if (fcent_shift > 7)
				return -EINVAL;
			hb1 = AD9122_HB1_INTERP(fcent_shift % 4);
			hb2 = AD9122_HB23_INTERP(fcent_shift);
			break;
		case 8:
			if (fcent_shift > 15)
				return -EINVAL;
			hb1 = AD9122_HB1_INTERP(fcent_shift % 4);
			hb2 = AD9122_HB23_INTERP(fcent_shift % 8);
			hb3 = AD9122_HB23_INTERP(fcent_shift / 2);
			break;
		default:
			return -EINVAL;
	}

	cached = conv->interp_factor;
	conv->interp_factor = interp;
	ret = ad9122_set_data_clk(conv, data_rate ?
				 data_rate : ad9122_get_data_clk(conv));

	if (ret < 0) {
		conv->interp_factor = cached;

		return ret;
	}

	tmp = ad9122_read(conv->spi, AD9122_REG_DATAPATH_CTRL);
	switch (hb1) {
		case AD9122_HB1_INTERP(1):
		case AD9122_HB1_INTERP(3):
			tmp &= ~AD9122_DATAPATH_CTRL_BYPASS_PREMOD;
			break;
		default:
			tmp |= AD9122_DATAPATH_CTRL_BYPASS_PREMOD;
	}

	ad9122_write(conv->spi, AD9122_REG_DATAPATH_CTRL, tmp);
	ad9122_write(conv->spi, AD9122_REG_HB1_CTRL, hb1);
	ad9122_write(conv->spi, AD9122_REG_HB2_CTRL, hb2);
	ad9122_write(conv->spi, AD9122_REG_HB3_CTRL, hb3);
	conv->fcenter_shift = fcent_shift;

	return 0;
}

static int ad9122_set_interpol_freq(struct cf_axi_converter *conv,
				   unsigned long freq)
{

	return __ad9122_set_interpol(conv, freq / ad9122_get_data_clk(conv),
			       conv->fcenter_shift, 0);
}

static int ad9122_set_interpol_fcent_freq(struct cf_axi_converter *conv,
					 unsigned long freq)
{

	return __ad9122_set_interpol(conv, conv->interp_factor,
		(freq * 2) / ad9122_get_data_clk(conv), 0);
}

static unsigned long ad9122_get_interpol_freq(struct cf_axi_converter *conv)
{
	return ad9122_get_data_clk(conv) * conv->interp_factor;
}

static unsigned long
ad9122_get_interpol_fcent_freq(struct cf_axi_converter *conv)
{
	return (ad9122_get_data_clk(conv) * conv->fcenter_shift) / 2;
}

static ssize_t ad9122_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	long readin;
	int ret;

	ret = kstrtol(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
	case AD9122_REG_I_DAC_OFFSET_MSB:
	case AD9122_REG_Q_DAC_OFFSET_MSB:
		if (readin < 0 || readin > 0xFFFF) {
			ret = -EINVAL;
			goto out;
		}
		break;
	case AD9122_REG_I_PHA_ADJ_MSB:
	case AD9122_REG_Q_PHA_ADJ_MSB:
		if (readin < -512 || readin > 511) {
			ret = -EINVAL;
			goto out;
		}
		break;
	default:
		if (readin < 0 || readin > 0x3FF) {
			ret = -EINVAL;
			goto out;
		}
		break;
	}

	ret = ad9122_write(conv->spi, (u32)this_attr->address, readin >> 8);
	if (ret < 0)
		goto out;

	ret = ad9122_write(conv->spi, (u32)this_attr->address - 1, readin & 0xFF);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9122_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	unsigned val;

	mutex_lock(&indio_dev->mlock);
	ret = ad9122_read(conv->spi, (u32)this_attr->address);
	if (ret < 0)
		goto out;
	val = ret << 8;

	ret = ad9122_read(conv->spi, (u32)this_attr->address - 1);
	if (ret < 0)
		goto out;
	val |= ret & 0xFF;

	switch ((u32)this_attr->address) {
	case AD9122_REG_I_PHA_ADJ_MSB:
	case AD9122_REG_Q_PHA_ADJ_MSB:
		val = sign_extend32(val, 9);
		break;
	}

	ret = sprintf(buf, "%d\n", val);
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9122_interpolation_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	long readin;
	unsigned pwr;
	int ret;

	ret = kstrtol(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	pwr = ad9122_read(conv->spi, AD9122_REG_POWER_CTRL);
	ad9122_write(conv->spi, AD9122_REG_POWER_CTRL, pwr |
			AD9122_POWER_CTRL_PD_I_DAC |
			AD9122_POWER_CTRL_PD_Q_DAC);

	switch ((u32)this_attr->address) {
	case 0:
		ret = ad9122_set_interpol_freq(conv, readin);
		break;
	case 1:
		ret = ad9122_set_interpol_fcent_freq(conv, readin);
		break;
	default:
		ret = -EINVAL;
	}

	if (conv->pcore_sync)
		conv->pcore_sync(indio_dev);
	ad9122_write(conv->spi, AD9122_REG_POWER_CTRL, pwr);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9122_interpolation_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int i, ret = 0;


	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case 0:
		ret = sprintf(buf, "%lu\n", ad9122_get_interpol_freq(conv));
		break;
	case 1:
		ret = sprintf(buf, "%lu\n", ad9122_get_interpol_fcent_freq(conv));
		break;
	case 2:
		for (i = 0; conv->intp_modes[i] != 0; i++)
			ret += sprintf(buf + ret, "%ld ", conv->intp_modes[i]);

		ret += sprintf(buf + ret, "\n");
		break;
	case 3:
		for (i = 0; conv->cs_modes[i] != -1; i++)
			ret += sprintf(buf + ret, "%ld ", conv->cs_modes[i]);

		ret += sprintf(buf + ret, "\n");
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(out_voltage0_phase, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_I_PHA_ADJ_MSB);

static IIO_DEVICE_ATTR(out_voltage1_phase, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_Q_PHA_ADJ_MSB);

static IIO_DEVICE_ATTR(out_voltage0_calibbias, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_I_DAC_OFFSET_MSB);

static IIO_DEVICE_ATTR(out_voltage1_calibbias, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_Q_DAC_OFFSET_MSB);

static IIO_DEVICE_ATTR(out_voltage0_calibscale, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_I_DAC_CTRL);

static IIO_DEVICE_ATTR(out_voltage1_calibscale, S_IRUGO | S_IWUSR,
 			ad9122_show,
 			ad9122_store,
 			AD9122_REG_Q_DAC_CTRL);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation_frequency, S_IRUGO | S_IWUSR,
			ad9122_interpolation_show,
			ad9122_interpolation_store,
			0);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation_frequency_available, S_IRUGO,
			ad9122_interpolation_show,
			NULL,
			2);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation_center_shift_frequency,
		        S_IRUGO | S_IWUSR,
			ad9122_interpolation_show,
			ad9122_interpolation_store,
			1);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation_center_shift_frequency_available,
		        S_IRUGO,
			ad9122_interpolation_show,
			NULL,
			3);

static struct attribute *ad9122_attributes[] = {
	&iio_dev_attr_out_voltage0_phase.dev_attr.attr, /* I */
	&iio_dev_attr_out_voltage0_calibscale.dev_attr.attr,
	&iio_dev_attr_out_voltage0_calibbias.dev_attr.attr,
	&iio_dev_attr_out_voltage1_phase.dev_attr.attr, /* Q */
	&iio_dev_attr_out_voltage1_calibscale.dev_attr.attr,
	&iio_dev_attr_out_voltage1_calibbias.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation_frequency.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation_center_shift_frequency.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation_frequency_available.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation_center_shift_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9122_attribute_group = {
	.attrs = ad9122_attributes,
};

static int ad9122_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*val = ad9122_get_data_clk(conv);
		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9122_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long rate;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		rate = ad9122_get_data_clk(conv);
		ret = ad9122_set_data_clk(conv, val);
		if (ret < 0) {
			return ret;
		}

		if (val != rate) {
			ad9122_tune_dci(conv);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9122_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct cf_axi_converter *conv;
	unsigned id, rate, datapath_ctrl;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->reset_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (!IS_ERR(conv->reset_gpio)) {
		ret = gpiod_direction_output(conv->reset_gpio, 1);
	}

	id = ad9122_read(spi, AD9122_REG_CHIP_ID);
	if (id != CHIPID_AD9122) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
 		ret = -ENODEV;
 		goto out;
	}

	conv->write = ad9122_write;
	conv->read = ad9122_read;
	conv->setup = ad9122_tune_dci;
	conv->get_fifo_status = ad9122_get_fifo_status;
	conv->get_data_clk = ad9122_get_data_clk;
	conv->write_raw = ad9122_write_raw;
	conv->read_raw = ad9122_read_raw;
	conv->attrs = &ad9122_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9122;

	ret = ad9122_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	ret = ad9122_setup(conv, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	of_property_read_u32(np, "dac-interp-factor", &conv->interp_factor);
	conv->interp_factor = ad9122_validate_interp_factor(conv->interp_factor);
	of_property_read_u32(np, "dac-fcenter-shift", &conv->fcenter_shift);

	datapath_ctrl = AD9122_DATAPATH_CTRL_BYPASS_PREMOD |
			AD9122_DATAPATH_CTRL_BYPASS_NCO;

	if (!of_property_read_bool(np, "dac-invsinc-en"))
	    datapath_ctrl |= AD9122_DATAPATH_CTRL_BYPASS_INV_SINC;

	ad9122_write(spi, AD9122_REG_DATAPATH_CTRL, datapath_ctrl);

	ret = of_property_read_u32(np, "dac-data-rate", &rate);
	if (ret)
		rate = 0;

	ret = __ad9122_set_interpol(conv, conv->interp_factor,
				  conv->fcenter_shift, rate);
	if (ret)
		goto out;

	spi_set_drvdata(spi, conv);

	return 0;
out:
	return ret;
}

static int ad9122_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct spi_device_id ad9122_id[] = {
	{"ad9122", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9122_id);

static struct spi_driver ad9122_driver = {
	.driver = {
		.name	= "ad9122",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9122_probe,
	.remove		= ad9122_remove,
	.id_table	= ad9122_id,
};
module_spi_driver(ad9122_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9122 ADC");
MODULE_LICENSE("GPL v2");
