/*
 * AD9467 SPI DAC driver for DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>
#include <linux/clkdev.h>

#define DCO_DEBUG

static int ad9467_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = 0x80 | (reg >> 8);
		buf[1] = reg & 0xFF;

		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
		if (ret < 0)
			return ret;

		return buf[2];
	}
	return -ENODEV;
}

static int ad9467_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg >> 8;
		buf[1] = reg & 0xFF;
		buf[2] = val;
		ret = spi_write_then_read(spi, buf, 3, NULL, 0);
		if (ret < 0)
			return ret;

		if ((reg == ADC_REG_TRANSFER) && (val == TRANSFER_SYNC) &&
			(spi_get_device_id(spi)->driver_data == CHIPID_AD9265))
			ad9467_spi_write(spi, ADC_REG_TRANSFER, 0);

		return 0;
	}

	return -ENODEV;
}

static int ad9467_outputmode_set(struct spi_device *spi, unsigned mode)
{
	int ret;

	ret = ad9467_spi_write(spi, ADC_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;
	ret = ad9467_spi_write(spi, ADC_REG_TEST_IO, TESTMODE_OFF);
	if (ret < 0)
		return ret;

	return ad9467_spi_write(spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
}

static int ad9467_testmode_set(struct iio_dev *indio_dev,
			     unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad9467_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 1 << chan);
	ad9467_spi_write(conv->spi, ADC_REG_TEST_IO, mode);
	ad9467_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 0x3);
	ad9467_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
	conv->testmode[chan] = mode;
	return 0;
}

static int ad9467_test_and_outputmode_set(struct iio_dev *indio_dev,
			     unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret;

	if (mode == TESTMODE_OFF)
		ret = ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
				       conv->adc_output_mode);
	else
		ret = ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
			conv->adc_output_mode & ~OUTPUT_MODE_TWOS_COMPLEMENT);

	if (ret < 0)
		return ret;

	return ad9467_testmode_set(indio_dev, chan, mode);
}

static int ad9467_dco_calibrate(struct iio_dev *indio_dev, unsigned chan)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, dco, cnt, start, max_start, max_cnt;
	unsigned stat, inv_range = 0, dco_en = 0, do_inv;
	unsigned char err_field[66];
	unsigned chan_ctrl0, chan_ctrl1;

	switch (conv->id) {
	case CHIPID_AD9250:
	case CHIPID_AD9683:
	case CHIPID_AD9625:
		return 0;
	case CHIPID_AD9434: /* TODO */
		return 0;
	case CHIPID_AD9265:
		dco_en = 0;
		break;
	default:
		dco_en = DCO_DELAY_ENABLE;
	}

	ret = ad9467_outputmode_set(conv->spi,
			conv->adc_output_mode & ~OUTPUT_MODE_TWOS_COMPLEMENT);
	if (ret < 0)
		return ret;

	chan_ctrl0 = axiadc_read(st, ADI_REG_CHAN_CNTRL(0));
	chan_ctrl1 = axiadc_read(st, ADI_REG_CHAN_CNTRL(1));

	do {
		ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE, OUTPUT_EVEN_ODD_MODE_EN |
				(inv_range ? INVERT_DCO_CLK : 0));

		if (chan == 2) {
			ad9467_testmode_set(indio_dev, 1, TESTMODE_PN23_SEQ);
			axiadc_write(st, ADI_REG_CHAN_CNTRL(1), ADI_ENABLE | ADI_PN23_TYPE);
			axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);
		}

		ad9467_testmode_set(indio_dev, 0, TESTMODE_PN9_SEQ);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(0), ADI_ENABLE);
		axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);

		for(dco = 0; dco <= 32; dco++) {
			ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_DELAY,
				dco > 0 ? ((dco - 1) | dco_en) : 0);
			ad9467_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);

			axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);
			if (chan == 2)
				axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);

			mdelay(1);

			stat = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
			if (chan == 2)
				stat |= axiadc_read(st, ADI_REG_CHAN_STATUS(0));

			err_field[dco + (inv_range * 33)] = !!(stat & (ADI_PN_ERR | ADI_PN_OOS));
		}

		for(dco = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
			dco <= (32 + (inv_range * 33)); dco++) {
			if (err_field[dco] == 0) {
				if (start == -1)
					start = dco;
				cnt++;
			} else {
				if (cnt > max_cnt) {
					max_cnt = cnt;
					max_start = start;
				}
				start = -1;
				cnt = 0;
			}
		}

		if (cnt > max_cnt) {
			max_cnt = cnt;
			max_start = start;
		}

		if ((inv_range == 0) &&
			((max_cnt < 3) || (err_field[32] == 0))) {
			do_inv = 1;
			inv_range = 1;
		} else {
			do_inv = 0;
		}

	} while (do_inv);

	dco = max_start + (max_cnt / 2);

#ifdef DCO_DEBUG
	for(cnt = 0; cnt <= (32  + (inv_range * 33)); cnt++)
		if (cnt == dco)
			printk("|");
		else
			printk("%c", err_field[cnt] ? '-' : 'o');
#endif
	if (dco > 32) {
		dco -= 33;
		ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN | INVERT_DCO_CLK);
		cnt = 1;
	} else {
		ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN);
		cnt = 0;
	}

#ifdef DCO_DEBUG
	printk(" %s DCO 0x%X CLK %lu Hz\n", cnt ? "INVERT" : "",
	       dco > 0 ? ((dco - 1) | dco_en) : 0, conv->adc_clk);
#endif

	ad9467_testmode_set(indio_dev, 0, TESTMODE_OFF);
	ad9467_testmode_set(indio_dev, 1, TESTMODE_OFF);
	ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_DELAY,
		      dco > 0 ? ((dco - 1) | dco_en) : 0);
	ad9467_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);

	axiadc_write(st, ADI_REG_CHAN_CNTRL(0), chan_ctrl0);
	axiadc_write(st, ADI_REG_CHAN_CNTRL(1), chan_ctrl1);

	ret = ad9467_outputmode_set(conv->spi, conv->adc_output_mode);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9265_scale_table[][2] = {
	{1250, 0x00}, {1500, 0x40}, {1750, 0x80}, {2000, 0xC0},
};

static int ad9467_scale_table[][2] = {
	{2000, 0}, {2100, 6}, {2200, 7},
	{2300, 8}, {2400, 9}, {2500, 10},
};

static int ad9643_scale_table[][2] = {
	{2087, 0x0F}, {2065, 0x0E}, {2042, 0x0D}, {2020, 0x0C}, {1997, 0x0B},
	{1975, 0x0A}, {1952, 0x09}, {1930, 0x08}, {1907, 0x07}, {1885, 0x06},
	{1862, 0x05}, {1840, 0x04}, {1817, 0x03}, {1795, 0x02}, {1772, 0x01},
	{1750, 0x00}, {1727, 0x1F}, {1704, 0x1E}, {1681, 0x1D}, {1658, 0x1C},
	{1635, 0x1B}, {1612, 0x1A}, {1589, 0x19}, {1567, 0x18}, {1544, 0x17},
	{1521, 0x16}, {1498, 0x15}, {1475, 0x14}, {1452, 0x13}, {1429, 0x12},
	{1406, 0x11}, {1383, 0x10},

};

static int ad9434_scale_table[][2] = {
	{1600, 0x1C}, {1580, 0x1D}, {1550, 0x1E}, {1520, 0x1F}, {1500, 0x00},
	{1470, 0x01}, {1440, 0x02}, {1420, 0x03}, {1390, 0x04}, {1360, 0x05},
	{1340, 0x06}, {1310, 0x07}, {1280, 0x08}, {1260, 0x09}, {1230, 0x0A},
	{1200, 0x0B}, {1180, 0x0C},
};

static void ad9467_convert_scale_table(struct axiadc_converter *conv)
{
	int i;

	for (i = 0; i < conv->chip_info->num_scales; i++)
		conv->chip_info->scale_table[i][0] =
			(conv->chip_info->scale_table[i][0] * 1000000ULL) >>
			conv->chip_info->channel[0].scan_type.realbits;

}

static const char testmodes[][16] = {
	[TESTMODE_OFF]			= "off",
	[TESTMODE_MIDSCALE_SHORT]	= "midscale_short",
	[TESTMODE_POS_FULLSCALE]		= "pos_fullscale",
	[TESTMODE_NEG_FULLSCALE]		= "neg_fullscale",
	[TESTMODE_ALT_CHECKERBOARD]	= "checkerboard",
	[TESTMODE_PN23_SEQ]		= "pn_long",
	[TESTMODE_PN9_SEQ]		= "pn_short",
	[TESTMODE_ONE_ZERO_TOGGLE]	= "one_zero_toggle",
	[TESTMODE_RAMP]			= "ramp",
};

static ssize_t ad9467_show_scale_available(struct iio_dev *indio_dev,
				   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i, len = 0;

	for (i = 0; i < conv->chip_info->num_scales; i++)
		len += sprintf(buf + len, "0.%06u ",
			       conv->chip_info->scale_table[i][0]);

	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t ad9467_testmode_mode_available(struct iio_dev *indio_dev,
				   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	size_t len = 0;
	int i;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		len += sprintf(buf + len, "%s ", testmodes[i]);
	}
	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t axiadc_testmode_read(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return sprintf(buf, "%s\n",
		testmodes[conv->testmode[chan->channel]]);
}

static ssize_t axiadc_testmode_write(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode, i;
	int ret;

	mode = 0;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (sysfs_streq(buf, testmodes[i])) {
			mode = i;
			break;
		}
	}

	mutex_lock(&indio_dev->mlock);
	ret = ad9467_testmode_set(indio_dev, chan->channel, mode);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	{
		.name = "test_mode",
		.read = axiadc_testmode_read,
		.write = axiadc_testmode_write,
	},
	{
		.name = "test_mode_available",
		.read = ad9467_testmode_mode_available,
		.shared = true,
	},
	{
		.name = "scale_available",
		.read = ad9467_show_scale_available,
		.shared = true,
	},
	{ },
};

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT | 		\
			IIO_CHAN_INFO_CALIBSCALE_SEPARATE_BIT |		\
			IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |		\
			IIO_CHAN_INFO_CALIBPHASE_SEPARATE_BIT |		\
			IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY_SEPARATE_BIT |		\
			IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
			.ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}



#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT |	 		\
			IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9467] = {
		.name = "AD9467",
		.max_rate = 250000000UL,
		.scale_table = ad9467_scale_table,
		.num_scales = ARRAY_SIZE(ad9467_scale_table),
		.max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 1,
		.channel[0] = AIM_CHAN(0, 0, 16, 's'),
	},
	[ID_AD9643] = {
		.name = "AD9643",
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = TESTMODE_RAMP,
		.num_channels = 2,
		.channel[0] = AIM_CHAN(0, 0, 14, 's'),
		.channel[1] = AIM_CHAN(1, 1, 14, 's'),
	},
	[ID_AD9250] = {
		.name = "AD9250",
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = TESTMODE_RAMP,
		.num_channels = 2,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 's'),
		.channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 's'),
	},
	[ID_AD9683] = {
		.name = "AD9683",
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = TESTMODE_RAMP,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 's'),
	},
	[ID_AD9625] = {
		.name = "AD9625",
		.max_rate = 2500000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = TESTMODE_RAMP,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 's'),
	},
	[ID_AD9265] = {
		.name = "AD9265",
		.max_rate = 125000000UL,
		.scale_table = ad9265_scale_table,
		.num_scales = ARRAY_SIZE(ad9265_scale_table),
		.max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 's'),
	},
	[ID_AD9434] = {
		.name = "AD9434",
		.max_rate = 500000000UL,
		.scale_table = ad9434_scale_table,
		.num_scales = ARRAY_SIZE(ad9434_scale_table),
		.max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 's'),
	},
};

static int ad9250_setup(struct spi_device *spi, unsigned m, unsigned l)
{
	int ret;
	unsigned pll_stat;
	static int sel = 0;

	ret = ad9467_spi_write(spi, 0x5f, (0x16 | 0x1)); // trail bits, ilas normal & pd
	ret |= ad9467_spi_write(spi, 0x5e, m << 4 | l); // m=2, l=2
	ret |= ad9467_spi_write(spi, 0x66, sel++); // lane id
	ret |= ad9467_spi_write(spi, 0x67, sel++); // lane id
	ret |= ad9467_spi_write(spi, 0x6e, 0x80 | (l - 1)); // scr, 2-lane
	ret |= ad9467_spi_write(spi, 0x70, 0x1f); // no. of frames per multi frame
	ret |= ad9467_spi_write(spi, 0x3a, 0x1e); // sysref enabled
	ret |= ad9467_spi_write(spi, 0x5f, (0x16 | 0x0)); // enable
	ret |= ad9467_spi_write(spi, 0x14, 0x00); // offset binary
	ret |= ad9467_spi_write(spi, 0x0d, 0x00); // test patterns

	ret |= ad9467_spi_write(spi, 0xff, 0x01);
	ret |= ad9467_spi_write(spi, 0xff, 0x00);

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "PLL %s, JESD204B Link %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED",
		 pll_stat & 0x01 ? "Ready" : "Fail");

	return ret;
}

static int ad9625_setup(struct spi_device *spi)
{
	unsigned pll_stat;
	int ret;

	ret = ad9467_spi_write(spi, 0x000, 0x3c);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	ret |= ad9467_spi_write(spi, 0x000, 0x18);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	ret |= ad9467_spi_write(spi, 0x008, 0x00);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	ret |= ad9467_spi_write(spi, 0x05f, 0x15);
	ret |= ad9467_spi_write(spi, 0x080, 0x00);
	ret |= ad9467_spi_write(spi, 0x120, 0x11);
	ret |= ad9467_spi_write(spi, 0x00d, 0x00);
	ret |= ad9467_spi_write(spi, 0x014, 0x00);
	ret |= ad9467_spi_write(spi, 0x05f, 0x14);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	ret |= ad9467_spi_write(spi, 0xff, 0x00);

	mdelay(10);

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "PLL %s, JESD204B Link %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED",
		 pll_stat & 0x01 ? "Ready" : "Fail");

	return ret;
}
static struct attribute *ad9467_attributes[] = {
	NULL,
};

static const struct attribute_group ad9467_attribute_group = {
	.attrs = ad9467_attributes,
};

static int ad9467_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned vref_val, mask;
	int i;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		vref_val = ad9467_spi_read(conv->spi, ADC_REG_VREF);

		switch (conv->id) {
		case CHIPID_AD9467:
			mask = AD9467_REG_VREF_MASK;
			break;
		case CHIPID_AD9643:
			mask = AD9643_REG_VREF_MASK;
			break;
		case CHIPID_AD9250:
		case CHIPID_AD9683:
		case CHIPID_AD9625:
			mask = AD9250_REG_VREF_MASK;
			break;
		case CHIPID_AD9265:
			mask = AD9265_REG_VREF_MASK;
			break;
		default:
			mask = 0xFFFF;
		}

		vref_val &= mask;

		for (i = 0; i < conv->chip_info->num_scales; i++)
			if (vref_val == conv->chip_info->scale_table[i][1])
				break;

		*val =  0;
		*val2 = conv->chip_info->scale_table[i][0];

		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9467_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;

		for (i = 0; i < conv->chip_info->num_scales; i++)
			if (val2 == conv->chip_info->scale_table[i][0]) {
				ad9467_spi_write(conv->spi, ADC_REG_VREF,
					conv->chip_info->scale_table[i][1]);
				ad9467_spi_write(conv->spi, ADC_REG_TRANSFER,
					      TRANSFER_SYNC);
				return 0;
			}

		return -EINVAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				"Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		if (conv->adc_clk != r_clk) {
			conv->adc_clk = r_clk;
			ret = ad9467_dco_calibrate(indio_dev,
						   conv->chip_info->num_channels);
		}

		return 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9467_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, i;

	ret = ad9467_dco_calibrate(indio_dev, conv->chip_info->num_channels);
	if (ret < 0)
		return ret;

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_IQCOR_ENB | ADI_ENABLE);

	return 0;
}

static int ad9467_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct clk *clk = NULL;
	int ret;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	spi->mode = SPI_MODE_0 | SPI_3WIRE;

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	conv->clk = clk;
	conv->adc_clk = clk_get_rate(clk);


	conv->id = ad9467_spi_read(spi, ADC_REG_CHIP_ID);
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
 		ret = -ENODEV;
 		goto out;
	}

	switch (conv->id) {
	case CHIPID_AD9467:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9467];
		conv->adc_output_mode = AD9467_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9643:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9643];
		conv->adc_output_mode = AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ad9467_spi_write(spi, ADC_REG_OUTPUT_PHASE, OUTPUT_EVEN_ODD_MODE_EN);
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9250:
		ret = ad9250_setup(spi, 2, 2);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize\n");
			ret = -EIO;
			goto out;
		}

		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9250];
		conv->adc_output_mode = AD9250_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9683:

		ret = ad9250_setup(spi, 1, 1);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize\n");
			ret = -EIO;
			goto out;
		}
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9683];
		conv->adc_output_mode = AD9683_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9625:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9625];
		conv->adc_output_mode = AD9625_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9265:
		ret = ad9625_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize\n");
			ret = -EIO;
			goto out;
		}

		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9265];
		conv->adc_output_mode = AD9265_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9434:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9434];
		conv->adc_output_mode = AD9434_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		ret = -ENODEV;
		goto out;
	}

	if (ret < 0)
		goto out;


	ad9467_convert_scale_table(conv);

	conv->write = ad9467_spi_write;
	conv->read = ad9467_spi_read;
	conv->write_raw = ad9467_write_raw;
	conv->read_raw = ad9467_read_raw;
	conv->post_setup = ad9467_post_setup;
	conv->testmode_set = ad9467_test_and_outputmode_set;
	conv->attrs = &ad9467_attribute_group;
	conv->spi = spi;

	spi_set_drvdata(spi, conv);

	return 0;

out:
	clk_disable_unprepare(clk);

	return ret;
}

static int ad9467_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	clk_disable_unprepare(conv->clk);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9467_id[] = {
	{"ad9467", CHIPID_AD9467},
	{"ad9643", CHIPID_AD9643},
	{"ad9250", CHIPID_AD9250},
	{"ad9265", CHIPID_AD9265},
	{"ad9683", CHIPID_AD9683},
	{"ad9434", CHIPID_AD9434},
	{"ad9625", CHIPID_AD9625},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9467_id);

static struct spi_driver ad9467_driver = {
	.driver = {
		.name	= "ad9467",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9467_probe,
	.remove		= ad9467_remove,
	.id_table	= ad9467_id,
};
module_spi_driver(ad9467_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC");
MODULE_LICENSE("GPL v2");
