/*
 * AXI_ADC ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

static int axiadc_spi_read(struct axiadc_state *st, unsigned reg)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	return conv->read(conv->spi, reg);
}

static int axiadc_spi_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	return conv->write(conv->spi, reg, val);
}

static void axiadc_toggle_scale_offset_en(struct axiadc_state *st)
{
// 	unsigned val = axiadc_read(st, AXIADC_PCORE_ADC_CTRL);
// 	val &= ~AXIADC_SCALE_OFFSET_EN;
// 	axiadc_write(st, AXIADC_PCORE_ADC_CTRL, val);
// 	axiadc_write(st, AXIADC_PCORE_ADC_CTRL, val | AXIADC_SCALE_OFFSET_EN);
}

// static int axiadc_debugfs_open(struct inode *inode, struct file *file)
// {
// 	if (inode->i_private)
// 		file->private_data = inode->i_private;
//
// 	return 0;
// }
//
//
//
// static ssize_t axiadc_debugfs_pncheck_read(struct file *file, char __user *userbuf,
// 			      size_t count, loff_t *ppos)
// {
// 	struct iio_dev *indio_dev = file->private_data;
// 	struct axiadc_state *st = iio_priv(indio_dev);
// 	char buf[80];
// 	ssize_t len;
// 	unsigned stat;
//
// 	stat = axiadc_read(st, AXIADC_PCORE_ADC_STAT);
//
// 	switch (st->id) {
// 	case CHIPID_AD9467:
// 		len = sprintf(buf, "%s %s\n", (stat & AXIADC_PCORE_ADC_STAT_PN_OOS) ?
// 			"Out of Sync :" : "In Sync :",
// 			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR) ?
// 			"PN Error" : "No Error");
// 		break;
// 	case CHIPID_AD9643:
// 		len = sprintf(buf, "CH0 %s %s\nCH1 %s %s\n", (stat & AXIADC_PCORE_ADC_STAT_PN_OOS0) ?
// 			"Out of Sync :" : "In Sync :",
// 			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR0) ?
// 			"PN Error" : "No Error",
// 			(stat & AXIADC_PCORE_ADC_STAT_PN_OOS1) ?
// 			"Out of Sync :" : "In Sync :",
// 			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR1) ?
// 			"PN Error" : "No Error");
// 		break;
// 	default:
// 		len = 0;
// 	}
//
// 	axiadc_write(st, AXIADC_PCORE_ADC_STAT, AXIADC_PCORE_ADC_STAT_MASK);
//
// 	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
// }
//
// static ssize_t axiadc_debugfs_pncheck_write(struct file *file,
// 		     const char __user *userbuf, size_t count, loff_t *ppos)
// {
// 	struct iio_dev *indio_dev = file->private_data;
// 	struct axiadc_state *st = iio_priv(indio_dev);
// 	unsigned mode = TESTMODE_OFF;
// 	char buf[80], *p = buf;
//
// 	count = min_t(size_t, count, (sizeof(buf)-1));
// 	if (copy_from_user(p, userbuf, count))
// 		return -EFAULT;
//
// 	p[count] = 0;
//
// 	if (sysfs_streq(p, "PN9"))
// 		mode = TESTMODE_PN9_SEQ;
// 	else if (sysfs_streq(p, "PN23"))
// 		mode = TESTMODE_PN23_SEQ;
// 	else if (sysfs_streq(p, "CALIB"))
// 		axiadc_dco_calibrate(indio_dev, st->chip_info->num_channels);
// 	else
// 		mode = TESTMODE_OFF;
//
// 	mutex_lock(&indio_dev->mlock);
// 	axiadc_testmode_set(indio_dev, 0, mode);
// 	axiadc_testmode_set(indio_dev, 1, mode);
//
// // 	axiadc_write(st, AXIADC_PCORE_PN_ERR_CTRL, (mode == TESTMODE_PN23_SEQ) ?
// // 		  AXIADC_PN23_EN : AXIADC_PN9_EN);
//
// 	mdelay(1); /* FIXME */
//
// 	axiadc_write(st, AXIADC_PCORE_ADC_STAT, AXIADC_PCORE_ADC_STAT_MASK);
// 	mutex_unlock(&indio_dev->mlock);
//
// 	return count;
// }
//
// static const struct file_operations axiadc_debugfs_pncheck_fops = {
// 	.open = axiadc_debugfs_open,
// 	.read = axiadc_debugfs_pncheck_read,
// 	.write = axiadc_debugfs_pncheck_write,
// };

static int axiadc_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			axiadc_write(st, reg & 0xFFFF, writeval);
			ret = 0;
		} else {
			ret = axiadc_spi_write(st, reg, writeval);
			axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		}
	} else {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			*readval = axiadc_read(st, reg & 0xFFFF);
		} else {
			ret = axiadc_spi_read(st, reg);
			if (ret < 0)
				return ret;
			*readval = ret;
		}
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int axiadc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int ret, sign;
	unsigned tmp, phase = 0;
	unsigned long long llval;

	switch (m) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
	case IIO_CHAN_INFO_CALIBSCALE:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_2(chan->channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (!((phase + chan->channel) % 2)) {
			tmp = ADI_TO_IQCOR_COEFF_1(tmp);
		} else {
			tmp = ADI_TO_IQCOR_COEFF_2(tmp);
		}

		if (tmp & 0x8000)
			sign = -1;
		else
			sign = 1;

		if (tmp & 0x4000)
			*val = 1 * sign;
		else
			*val = 0;

		tmp &= ~0xC000;

		llval = tmp * 1000000ULL;
		do_div(llval, 0x4000);
		if (*val == 0)
			*val2 = llval * sign;
		else
			*val2 = llval;

		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_1(chan->channel));
		tmp = ADI_TO_DCFILT_OFFSET(tmp);
		*val = sign_extend32(tmp, 14);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = conv->read_raw(indio_dev, chan, val, val2, m);
		if (ret < 0 || !*val) {
			tmp = ADI_TO_CLK_FREQ(axiadc_read(st, ADI_REG_CLK_FREQ));
			llval = tmp * 100000000ULL /* FIXME */ * ADI_TO_CLK_RATIO(axiadc_read(st, ADI_REG_CLK_RATIO));
			*val = llval >> 16;
		}

		if (chan->extend_name) {
			tmp = axiadc_read(st,
				ADI_REG_CHAN_USR_CNTRL_2(chan->channel));

			llval = ADI_TO_USR_DECIMATION_M(tmp) * conv->adc_clk;
			do_div(llval, ADI_TO_USR_DECIMATION_N(tmp));
			*val = llval;
		}

	default:
		return conv->read_raw(indio_dev, chan, val, val2, m);

	}

	return -EINVAL;
}

static int axiadc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	unsigned fract, tmp, phase = 0;
	unsigned long long llval;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
	case IIO_CHAN_INFO_CALIBSCALE:
		/*  format is 1.1.14 (sign, integer and fractional bits) */
		switch (val) {
		case 1:
			fract = 0x4000;
			break;
		case -1:
			fract = 0xC000;
			break;
		case 0:
			fract = 0;
			if (val2 < 0) {
				fract = 0x8000;
				val2 *= -1;
			}
			break;
		default:
			return -EINVAL;
		}

		llval = (unsigned long long)val2 * 0x4000UL;
		do_div(llval, 1000000);
		fract |= llval;

		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_2(chan->channel));

		if (!((chan->channel + phase) % 2)) {
			tmp &= ~ADI_IQCOR_COEFF_1(~0);
			tmp |= ADI_IQCOR_COEFF_1(fract);
		} else {
			tmp &= ~ADI_IQCOR_COEFF_2(~0);
			tmp |= ADI_IQCOR_COEFF_2(fract);
		}

		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(chan->channel), tmp);

		axiadc_toggle_scale_offset_en(st);

		return 0;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_1(chan->channel));
		tmp &= ~ADI_DCFILT_OFFSET(~0);
		tmp |= ADI_DCFILT_OFFSET((short)val);

		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(chan->channel), tmp);
		axiadc_toggle_scale_offset_en(st);
		return 0;
	default:
		return conv->write_raw(indio_dev, chan, val, val2, mask);
	}
}

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static int axiadc_channel_setup(struct iio_dev *indio_dev,
				struct iio_chan_spec *adc_channels,
				unsigned adc_chan_num)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, cnt, usr_ctrl;

	st->max_usr_channel = ADI_USR_CHANMAX(axiadc_read(st, ADI_REG_USR_CNTRL_1));
	st->max_usr_channel = 0; /* FIXME */

	for (i = 0, cnt = 0; i < adc_chan_num; i++)
		st->channels[cnt++] = adc_channels[i];

	for (i = 0; i < st->max_usr_channel; i++) {
		usr_ctrl = axiadc_read(st, ADI_REG_CHAN_USR_CNTRL_1(cnt));
		st->channels[cnt].type = IIO_VOLTAGE;
		st->channels[cnt].indexed = 1,
		st->channels[cnt].channel = cnt;
		st->channels[cnt].scan_index = cnt;
		st->channels[cnt].info_mask = IIO_CHAN_INFO_SAMP_FREQ_SEPARATE_BIT;
		st->channels[cnt].extend_name = "user_logic";
		st->channels[cnt].scan_type.sign = (usr_ctrl & ADI_USR_DATATYPE_SIGNED) ? 's' : 'u';
		st->channels[cnt].scan_type.realbits = ADI_TO_USR_DATATYPE_BITS(usr_ctrl);
		st->channels[cnt].scan_type.storagebits = ADI_TO_USR_DATATYPE_TOTAL_BITS(usr_ctrl);
		st->channels[cnt].scan_type.shift = ADI_TO_USR_DATATYPE_SHIFT(usr_ctrl);
		st->channels[cnt].scan_type.endianness = (usr_ctrl & ADI_USR_DATATYPE_BE) ? IIO_BE : IIO_LE;
		cnt++;
	}

	indio_dev->channels = st->channels;
	indio_dev->num_channels = cnt;
	indio_dev->masklength = cnt;

	return 0;
}

static const struct iio_info axiadc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &axiadc_read_raw,
	.write_raw = &axiadc_write_raw,
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

static int axiadc_attach_spi_client(struct device *dev, void *data)
{
	struct axiadc_spidev *axiadc_spidev = data;

	if ((axiadc_spidev->of_nspi == dev->of_node) && dev->driver) {
		axiadc_spidev->dev_spi = dev;
		return 1;
	}

	return 0;
}



/**
 * axiadc_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int axiadc_of_probe(struct platform_device *op)
{
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	struct axiadc_state *st;
	struct resource r_mem; /* IO mem resources */
	struct axiadc_spidev axiadc_spidev;
	struct axiadc_converter *conv;
	resource_size_t remap_size, phys_addr;
	int ret;

	dev_dbg(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	/* Defer driver probe until matching spi
	 * converter driver is registered
	 */
	axiadc_spidev.of_nspi = of_parse_phandle(op->dev.of_node,
						 "spibus-connected", 0);
	if (!axiadc_spidev.of_nspi) {
		dev_err(&op->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, &axiadc_spidev,
			       axiadc_attach_spi_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(axiadc_spidev.dev_spi->driver->owner))
		return -ENODEV;

	get_device(axiadc_spidev.dev_spi);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		return ret;
	}

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev_spi = axiadc_spidev.dev_spi;

	dev_set_drvdata(dev, indio_dev);
	mutex_init(&st->lock);

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}

	st->rx_chan = of_dma_request_slave_channel(op->dev.of_node, "rx");
	if (!st->rx_chan) {
		ret = -EPROBE_DEFER;
		dev_err(dev, "failed to find rx dma device\n");
		goto failed2;
	}

	conv = to_converter(st->dev_spi);
	iio_device_set_drvdata(indio_dev, conv);

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);
	st->max_count = AXIADC_MAX_DMA_SIZE;

	st->dma_align = ADI_DMA_BUSWIDTH(axiadc_read(st, ADI_REG_DMA_BUSWIDTH));


	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	axiadc_channel_setup(indio_dev, conv->chip_info->channel,
			     conv->chip_info->num_channels);

	st->iio_info = axiadc_info;
	st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	ret = conv->post_setup(indio_dev);

	init_completion(&st->dma_complete);

	axiadc_configure_ring(indio_dev);

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto failed4;

	*indio_dev->buffer->scan_mask = (1UL << conv->chip_info->num_channels) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed4;

	dev_info(dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p,"
		 " DMA-%d probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)phys_addr, st->regs,
		 st->rx_chan->chan_id, conv->chip_info->name,
		 axiadc_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER");

// 	if (iio_get_debugfs_dentry(indio_dev))
// 		debugfs_create_file("pseudorandom_err_check", 0644,
// 					iio_get_debugfs_dentry(indio_dev),
// 					indio_dev, &axiadc_debugfs_pncheck_fops);

	return 0;

failed4:
	axiadc_unconfigure_ring(indio_dev);
failed3:
	dma_release_channel(st->rx_chan);
failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);
	iio_device_free(indio_dev);
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * axiadc_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int axiadc_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	axiadc_unconfigure_ring(indio_dev);

	dma_release_channel(st->rx_chan);

	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	iio_device_free(indio_dev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,cf-ad9467-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9643-core-1.00.a", },
	{ .compatible = "xlnx,axi-adc-2c-1.00.a", },
	{ .compatible =	"xlnx,axi-adc-1c-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9250-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9265-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9683-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9625-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9434-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe		= axiadc_of_probe,
	.remove		= axiadc_of_remove,
};

module_platform_driver(axiadc_of_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices ADI-AIM");
MODULE_LICENSE("GPL v2");
