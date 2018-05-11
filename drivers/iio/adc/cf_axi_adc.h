/*
 * ADI-AIM ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#ifndef ADI_AXI_ADC_H_
#define ADI_AXI_ADC_H_

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */


/* ADC COMMON */


#define ADI_REG_RSTN			0x0040
#define ADI_RSTN				(1 << 0)

#define ADI_REG_CNTRL			0x0044
#define ADI_R1_MODE			(1 << 2)
#define ADI_DDR_EDGESEL			(1 << 1)
#define ADI_PIN_MODE			(1 << 0)

#define ADI_REG_CLK_FREQ			0x0054
#define ADI_CLK_FREQ(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_FREQ(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_CLK_RATIO		0x0058
#define ADI_CLK_RATIO(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_RATIO(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_STATUS			0x005C
#define ADI_MUX_PN_ERR			(1 << 3)
#define ADI_MUX_PN_OOS			(1 << 2)
#define ADI_MUX_OVER_RANGE		(1 << 1)
#define ADI_STATUS			(1 << 0)

#define ADI_REG_DELAY_CNTRL		0x0060
#define ADI_DELAY_SEL			(1 << 17)
#define ADI_DELAY_RWN			(1 << 16)
#define ADI_DELAY_ADDRESS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_DELAY_ADDRESS(x)		(((x) >> 8) & 0xFF)
#define ADI_DELAY_WDATA(x)		(((x) & 0x1F) << 0)
#define ADI_TO_DELAY_WDATA(x)		(((x) >> 0) & 0x1F)

#define ADI_REG_DELAY_STATUS		0x0064
#define ADI_DELAY_LOCKED			(1 << 9)
#define ADI_DELAY_STATUS			(1 << 8)
#define ADI_DELAY_RDATA(x)		(((x) & 0x1F) << 0)
#define ADI_TO_DELAY_RDATA(x)		(((x) >> 0) & 0x1F)

#define ADI_REG_DRP_CNTRL		0x0070
#define ADI_DRP_SEL			(1 << 29)
#define ADI_DRP_RWN			(1 << 28)
#define ADI_DRP_ADDRESS(x)		(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)		(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS		0x0074
#define ADI_DRP_STATUS			(1 << 16)
#define ADI_DRP_RDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_RDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DMA_CNTRL		0x0080
#define ADI_DMA_STREAM			(1 << 1)
#define ADI_DMA_START			(1 << 0)

#define ADI_REG_DMA_COUNT		0x0084
#define ADI_DMA_COUNT(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_COUNT(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_DMA_STATUS		0x0088
#define ADI_DMA_OVF			(1 << 2)
#define ADI_DMA_UNF			(1 << 1)
#define ADI_DMA_STATUS			(1 << 0)

#define ADI_REG_DMA_BUSWIDTH		0x008C
#define ADI_DMA_BUSWIDTH(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_BUSWIDTH(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_USR_CNTRL_1		0x00A0
#define ADI_USR_CHANMAX(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)		(((x) >> 0) & 0xFF)

/* ADC CHANNEL */

#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_IQCOR_ENB			(1 << 9)
#define ADI_DCFILT_ENB			(1 << 8)
#define ADI_FORMAT_SIGNEXT		(1 << 6)
#define ADI_FORMAT_TYPE			(1 << 5)
#define ADI_FORMAT_ENABLE		(1 << 4)
#define ADI_PN23_TYPE			(1 << 1)
#define ADI_ENABLE			(1 << 0)

#define ADI_REG_CHAN_STATUS(c)		(0x0404 + (c) * 0x40)
#define ADI_PN_ERR			(1 << 2)
#define ADI_PN_OOS			(1 << 1)
#define ADI_OVER_RANGE			(1 << 0)

#define ADI_REG_CHAN_CNTRL_1(c)		(0x0410 + (c) * 0x40)
#define ADI_DCFILT_OFFSET(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DCFILT_OFFSET(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DCFILT_COEFF(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DCFILT_COEFF(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)
#define ADI_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_USR_CNTRL_1(c)		(0x0420 + (c) * 0x40)
#define ADI_USR_DATATYPE_BE			(1 << 25)
#define ADI_USR_DATATYPE_SIGNED			(1 << 24)
#define ADI_USR_DATATYPE_SHIFT(x)		(((x) & 0xFF) << 16)
#define ADI_TO_USR_DATATYPE_SHIFT(x)		(((x) >> 16) & 0xFF)
#define ADI_USR_DATATYPE_TOTAL_BITS(x)		(((x) & 0xFF) << 8)
#define ADI_TO_USR_DATATYPE_TOTAL_BITS(x)	(((x) >> 8) & 0xFF)
#define ADI_USR_DATATYPE_BITS(x)			(((x) & 0xFF) << 0)
#define ADI_TO_USR_DATATYPE_BITS(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_CHAN_USR_CNTRL_2(c)		(0x0424 + (c) * 0x40)
#define ADI_USR_DECIMATION_M(x)			(((x) & 0xFFFF) << 16)
#define ADI_TO_USR_DECIMATION_M(x)		(((x) >> 16) & 0xFFFF)
#define ADI_USR_DECIMATION_N(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_USR_DECIMATION_N(x)		(((x) >> 0) & 0xFFFF)



/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877
 */

#define ADC_REG_CHIP_PORT_CONF		0x00
#define ADC_REG_CHIP_ID			0x01
#define ADC_REG_CHIP_GRADE		0x02
#define ADC_REG_CHAN_INDEX		0x05
#define ADC_REG_TRANSFER		0xFF
#define ADC_REG_MODES			0x08
#define ADC_REG_TEST_IO			0x0D
#define ADC_REG_ADC_INPUT		0x0F
#define ADC_REG_OFFSET			0x10
#define ADC_REG_OUTPUT_MODE		0x14
#define ADC_REG_OUTPUT_ADJUST		0x15
#define ADC_REG_OUTPUT_PHASE		0x16
#define ADC_REG_OUTPUT_DELAY		0x17
#define ADC_REG_VREF			0x18
#define ADC_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* ADC_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ			0x6
#define TESTMODE_ONE_ZERO_TOGGLE		0x7
#define TESTMODE_RAMP			0xF

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

/* ADC_REG_OUTPUT_DELAY */
#define DCO_DELAY_ENABLE 		0x80


/*
 * Analog Devices AD9467 16-Bit, 200/250 MSPS ADC
 */

#define CHIPID_AD9467			0x50
#define AD9467_DEF_OUTPUT_MODE		0x08
#define AD9467_REG_VREF_MASK		0x0F

/*
 * Analog Devices AD9643 Dual 14-Bit, 170/210/250 MSPS ADC
 */

#define CHIPID_AD9643			0x82
#define AD9643_REG_VREF_MASK		0x1F
#define AD9643_DEF_OUTPUT_MODE		0x00

/*
 * Analog Devices AD9250 Dual 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9250			0xB9
#define AD9250_REG_VREF_MASK		0x1F
#define AD9250_DEF_OUTPUT_MODE		0x00

/*
 * Analog Devices AD9683 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9683			0xC3
#define AD9683_DEF_OUTPUT_MODE		0x00
#define AD9683_AXIADC_PCORE_DATA_SEL	0x28
#define AD9683_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9625 12-Bit, 2500 MSPS ADC, JESD204B
 */

#define CHIPID_AD9625			0x41
#define AD9625_DEF_OUTPUT_MODE		0x00
#define AD9625_AXIADC_PCORE_DATA_SEL	0x24
#define AD9625_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9265 16-Bit, 125/105/80 MSPS ADC
 */

#define CHIPID_AD9265			0x64
#define AD9265_DEF_OUTPUT_MODE		0x40
#define AD9265_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9434 12-Bit, 370/500 MSPS ADC
 */

#define CHIPID_AD9434			0x6A
#define AD9434_DEF_OUTPUT_MODE		0x00
#define AD9434_REG_VREF_MASK		0xC0


/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

#include <linux/spi/spi.h>

#define AXIADC_MAX_PCORE_TSIZE		(524288)
#define AXIADC_MAX_DMA_SIZE		(4 * 1024 * 1024) /* Randomly picked */


enum {
	ID_AD9467,
	ID_AD9643,
	ID_AD9250,
	ID_AD9265,
	ID_AD9683,
	ID_AD9625,
	ID_AD9434,
};

struct axiadc_chip_info {
	char				name[8];
	unsigned			num_channels;
	int			(*scale_table)[2];
	int				num_scales;
	int				max_testmode;
	unsigned long			max_rate;
	struct iio_chan_spec		channel[2];
};

struct axiadc_state {
	struct device 			*dev_spi;
	struct mutex			lock;
	struct completion		dma_complete;
	struct dma_chan			*rx_chan;
	struct iio_info			iio_info;
	void __iomem			*regs;
	void				*buf_virt;
	dma_addr_t			buf_phys;
	size_t				read_offs;
	int				compl_stat;
	unsigned				max_usr_channel;
	unsigned			adc_def_output_mode;
	unsigned			ring_lenght;
	unsigned			rcount;
	unsigned			max_count;
	unsigned			id;
	unsigned			pcore_version;
	unsigned char		testmode[2];
	unsigned long 		adc_clk;
	unsigned			dma_align;
	struct iio_chan_spec	channels[16];
};

struct ad9361_rf_phy;

struct axiadc_converter {
	struct spi_device 	*spi;
	struct clk 		*clk;
	struct ad9361_rf_phy *phy;
	unsigned			id;
	unsigned			adc_output_mode;
	unsigned 		testmode[2];
	unsigned long 		adc_clk;
	const struct axiadc_chip_info	*chip_info;
	int		(*read)(struct spi_device *spi, unsigned reg);
	int		(*write)(struct spi_device *spi,
				 unsigned reg, unsigned val);
	int		(*setup)(struct spi_device *spi, unsigned mode);

	struct iio_chan_spec const	*channels;
	int				num_channels;
	const struct attribute_group	*attrs;
	struct iio_dev 	*indio_dev;
	int (*read_raw)(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask);

	int (*write_raw)(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int val,
			 int val2,
			 long mask);

	int (*post_setup)(struct iio_dev *indio_dev);
};



static inline struct axiadc_converter *to_converter(struct device *dev)
{
	struct axiadc_converter *conv = spi_get_drvdata(to_spi_device(dev));

	if (conv)
		return conv;

	return ERR_PTR(-ENODEV);
};

struct axiadc_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
};

/*
 * IO accessors
 */

static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

int axiadc_configure_ring(struct iio_dev *indio_dev);
void axiadc_unconfigure_ring(struct iio_dev *indio_dev);

#endif /* ADI_AXI_ADC_H_ */
