/*
 * AD9361 Agile RF Transceiver
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>

#include <linux/of.h>
#include <linux/of_gpio.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"
#include "ad9361.h"

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

enum ad9361_clocks {
	BB_REFCLK,
	RX_REFCLK,
	TX_REFCLK,
	BBPLL_CLK,
	ADC_CLK,
	R2_CLK,
	R1_CLK,
	CLKRF_CLK,
	RX_SAMPL_CLK,
	DAC_CLK,
	T2_CLK,
	T1_CLK,
	CLKTF_CLK,
	TX_SAMPL_CLK,
	RX_RFPLL,
	TX_RFPLL,
	NUM_AD9361_CLKS,
};

enum {
	ID_AD9361,
};

struct ad9361_rf_phy {
	struct spi_device 	*spi;
	struct clk 		*clk_refin;
	struct clk 		*clks[NUM_AD9361_CLKS];
	struct clk_onecell_data	clk_data;
	struct ad9361_phy_platform_data *pdata;
	struct bin_attribute 	bin;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	u8 			prev_ensm_state;
	u8			curr_ensm_state;
	struct rx_gain_info rx_gain[RXGAIN_TBLS_END];
	enum rx_gain_table_name current_table;
	bool 			ensm_pin_ctl_en;

	bool			auto_cal_en;
	u64			last_rx_quad_cal_freq;
	u64			last_tx_quad_cal_freq;
	unsigned long		flags;
	unsigned long		cal_threshold_freq;
	u32			current_rx_bw_Hz;
	u32			current_tx_bw_Hz;
	u32			rxbbf_div;
	u32			ensm_conf1;
	u32			rate_governor;
	bool			bypass_rx_fir;
	bool			bypass_tx_fir;
	bool			rx_eq_2tx;
	u8			tx_fir_int;
	u8			tx_fir_ntaps;
	u8			rx_fir_dec;
	u8			rx_fir_ntaps;
	u8			agc_mode[2];
	bool			rfdc_track_en;
	bool			bbdc_track_en;
	bool			quad_track_en;
};

struct refclk_scale {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct ad9361_rf_phy	*phy;
	u32			mult;
	u32			div;
	enum ad9361_clocks 	source;
};

const char ad9361_ensm_states[][10] = { "sleep", "", "", "", "", "alert", "tx", "tx flush",
	"rx", "rx_flush", "fdd", "fdd_flush"};


static int ad9361_reset(struct ad9361_rf_phy *phy)
{
	if (gpio_is_valid(phy->pdata->gpio_resetb)) {
		gpio_set_value(phy->pdata->gpio_resetb, 0);
		udelay(2);
		gpio_set_value(phy->pdata->gpio_resetb, 1);
		return 0;
	}

	return -ENODEV;
}

static int ad9361_spi_readm(struct spi_device *spi, u32 reg,
			   u8 *rbuf, u32 num)
{
	u8 buf[2];
	int ret;
	u16 cmd;

	if (num > MAX_MBYTE_SPI)
		return -EINVAL;

	cmd = AD_READ | AD_CNT(num) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	ret = spi_write_then_read(spi, &buf[0], 2, rbuf, num);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	{
		int i;
		for (i = 0; i < num; i++)
			dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n",
				__func__, reg--, rbuf[i]);
	}
#endif

	return 0;
}

static int ad9361_spi_read(struct spi_device *spi, u32 reg)
{
	u8 buf;
	int ret;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	return buf;
}

static int __ad9361_spi_readf(struct spi_device *spi, u32 reg,
				 u32 mask, u32 offset)
{
	u8 buf;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	buf &= mask;
	buf >>= offset;

	return buf;
}

#define ad9361_spi_readf(spi, reg, mask) \
	__ad9361_spi_readf(spi, reg, mask, __ffs(mask))

static int ad9361_spi_write(struct spi_device *spi,
			 u32 reg, u32 val)
{
	u8 buf[3];
	int ret;
	u16 cmd;

	cmd = AD_WRITE | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, buf[2]);
#endif

	return 0;
}

static int __ad9361_spi_writef(struct spi_device *spi, u32 reg,
				 u32 mask, u32 offset, u32 val)
{
	u8 buf;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	buf &= ~mask;
	buf |= ((val << offset) & mask);

	return ad9361_spi_write(spi, reg, buf);
}

#define ad9361_spi_writef(spi, reg, mask, val) \
	__ad9361_spi_writef(spi,reg, mask, __ffs(mask), val)

static int ad9361_spi_writem(struct spi_device *spi,
			 u32 reg, u8 *tbuf, u32 num)
{
	u8 buf[10];
	int ret;
	u16 cmd;

	if (num > MAX_MBYTE_SPI)
		return -EINVAL;

	cmd = AD_WRITE | AD_CNT(num) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	memcpy(&buf[2], tbuf, num);

	ret = spi_write_then_read(spi, buf, num + 2, NULL, 0);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	{
		int i;
		for (i = 0; i < num; i++)
			printk("%s: reg 0x%X val 0x%X\n", __func__, reg--, tbuf[i]);
	}
#endif

	return 0;
}

static int ad9361_check_cal_done(struct ad9361_rf_phy *phy, u32 reg,
				 u32 mask, bool done_state)
{
	u32 timeout = 500;
	u32 state;

	do {
		state = ad9361_spi_readf(phy->spi, reg, mask);
		if (state == done_state)
			return 0;

		msleep_interruptible(1);
	} while (timeout--);

	dev_err(&phy->spi->dev, "Calibration TIMEOUT (0x%X, 0x%X)", reg, mask);

	return -ETIMEDOUT;
}

static int ad9361_run_calibration(struct ad9361_rf_phy *phy, u32 mask)
{
	int ret = ad9361_spi_write(phy->spi, REG_CALIBRATION_CTRL, mask);
	if (ret < 0)
		return ret;

	dev_dbg(&phy->spi->dev, "%s: CAL Mask 0x%X", __func__, mask);

	return ad9361_check_cal_done(phy, REG_CALIBRATION_CTRL, mask, 0);
}

enum rx_gain_table_name ad9361_gt_tableindex(u64 freq)
{
	if (freq <= 1300000000ULL)
		return TBL_200_1300_MHZ;

	if (freq <= 4000000000ULL)
		return TBL_1300_4000_MHZ;

	return TBL_4000_6000_MHZ;
}

/* PLL operates between 47 .. 6000 MHz which is > 2^32 */

unsigned long ad9361_to_clk(u64 freq)
{
	return (unsigned long)(freq >> 1);
}

u64 ad9361_from_clk(unsigned long freq)
{
	return ((u64)freq << 1);
}

static int ad9361_load_gt(struct ad9361_rf_phy *phy, u64 freq, u32 dest)
{
	struct spi_device *spi = phy->spi;
	const u8 (*tab)[3];
	u32 band, index_max, i;

	dev_dbg(&phy->spi->dev, "%s: frequency %llu", __func__, freq);

	band = ad9361_gt_tableindex(freq);

	dev_dbg(&phy->spi->dev, "%s: frequency %llu (band %d)",
		__func__, freq, band);

	/* check if table is present */
	if (phy->current_table == band)
		return 0;

	ad9361_spi_writef(spi, REG_AGC_CONFIG_2,
			  AGC_USE_FULL_GAIN_TABLE, !phy->pdata->split_gt);

	if (phy->pdata->split_gt) {
		tab = &split_gain_table[band][0];
		index_max = SIZE_SPLIT_TABLE;
	} else {
		tab = &full_gain_table[band][0];
		index_max = SIZE_FULL_TABLE;
	}

	ad9361_spi_write(spi, REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK |
			RECEIVER_SELECT(dest)); /* Start Gain Table Clock */

	for (i = 0; i < index_max; i++) {
		ad9361_spi_write(spi, REG_GAIN_TABLE_ADDRESS, i); /* Gain Table Index */
		ad9361_spi_write(spi, REG_GAIN_TABLE_WRITE_DATA1, tab[i][0]); /* Ext LNA, Int LNA, & Mixer Gain Word */
		ad9361_spi_write(spi, REG_GAIN_TABLE_WRITE_DATA2, tab[i][1]); /* TIA & LPF Word */
		ad9361_spi_write(spi, REG_GAIN_TABLE_WRITE_DATA3, tab[i][2]); /* DC Cal bit & Dig Gain Word */
		ad9361_spi_write(spi, REG_GAIN_TABLE_CONFIG,
				START_GAIN_TABLE_CLOCK |
				WRITE_GAIN_TABLE |
				RECEIVER_SELECT(dest)); /* Gain Table Index */
		ad9361_spi_write(spi, REG_GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay 3 ADCCLK/16 cycles */
		ad9361_spi_write(spi, REG_GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	}

	ad9361_spi_write(spi, REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK |
			RECEIVER_SELECT(dest)); /* Clear Write Bit */
	ad9361_spi_write(spi, REG_GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	ad9361_spi_write(spi, REG_GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	ad9361_spi_write(spi, REG_GAIN_TABLE_CONFIG, 0); /* Stop Gain Table Clock */

	phy->current_table = band;

	return 0;
}

static int ad9361_setup_ext_lna(struct ad9361_rf_phy *phy, u32 gain_mdB,
				u32 bypass_loss_mdB)
{
	ad9361_spi_write(phy->spi, REG_EXT_LNA_HIGH_GAIN,
			EXT_LNA_HIGH_GAIN(gain_mdB / 500));

	return ad9361_spi_write(phy->spi, REG_EXT_LNA_LOW_GAIN,
			EXT_LNA_LOW_GAIN(bypass_loss_mdB / 500));
}

static int ad9361_load_mixer_gm_subtable(struct ad9361_rf_phy *phy)
{
	int i, addr;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_CONFIG,
			 START_GM_SUB_TABLE_CLOCK); /* Start Clock */

	for (i = 0, addr = ARRAY_SIZE(gm_st_ctrl); i < ARRAY_SIZE(gm_st_ctrl); i++) {
		ad9361_spi_write(phy->spi, REG_GAIN_TABLE_ADDRESS, --addr); /* Gain Table Index */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_BIAS_WRITE, 0); /* Bias */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_GAIN_WRITE, gm_st_gain[i]); /* Gain */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_CTRL_WRITE, gm_st_ctrl[i]); /* Control */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_CONFIG,
				 WRITE_GM_SUB_TABLE | START_GM_SUB_TABLE_CLOCK); /* Write Words */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
		ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	}

	ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_CONFIG, START_GM_SUB_TABLE_CLOCK); /* Clear Write */
	ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	ad9361_spi_write(phy->spi, REG_GM_SUB_TABLE_CONFIG, 0); /* Stop Clock */

	return 0;
}

static int ad9361_set_tx_atten(struct ad9361_rf_phy *phy, u32 atten_mdb,
			       bool tx1, bool tx2)
{
	u8 buf[2];
	int ret = 0;

	dev_dbg(&phy->spi->dev, "%s : attenuation %u mdB tx1=%d tx2=%d",
		__func__, atten_mdb, tx1, tx2);

	if (atten_mdb > 89750) /* 89.75 dB */
		return -EINVAL;

	atten_mdb /= 250; /* Scale to 0.25dB / LSB */

	buf[0] = atten_mdb >> 8;
	buf[1] = atten_mdb & 0xFF;

	if (tx1)
		ret = ad9361_spi_writem(phy->spi, REG_TX1_ATTEN_1, buf, 2);

	if (tx2)
		ret = ad9361_spi_writem(phy->spi, REG_TX2_ATTEN_1, buf, 2);

	ad9361_spi_writef(phy->spi, REG_TX2_DIG_ATTEN,
			  IMMEDIATELY_UPDATE_TPC_ATTEN, 1);

	return ret;
}

static int ad9361_get_tx_atten(struct ad9361_rf_phy *phy, u32 tx_num)
{
	u8 buf[2];
	int ret = 0;
	u32 code;

	ret = ad9361_spi_readm(phy->spi, (tx_num == 1) ?
			REG_TX1_ATTEN_1 : REG_TX2_ATTEN_1, buf, 2);

	if (ret < 0)
		return ret;

	code = (buf[0] << 8) | buf[1];

	code *= 250;

	return code;
}

u32 ad9361_rfvco_tableindex(unsigned long freq)
{
	if (freq < 50000000UL)
		return LUT_FTDD_40;

	if (freq <= 70000000UL)
		return LUT_FTDD_60;

	return LUT_FTDD_80;
}

static int ad9361_rfpll_vco_init(struct ad9361_rf_phy *phy,
				 bool tx, u64 vco_freq,
				 unsigned long ref_clk)
{
	struct spi_device *spi = phy->spi;
	const struct SynthLUT (*tab);
	int i = 0;
	u32 range, offs = 0;

	range = ad9361_rfvco_tableindex(ref_clk);

	dev_dbg(&phy->spi->dev, "%s : vco_freq %llu : ref_clk %lu : range %d",
		__func__, vco_freq, ref_clk, range);

	do_div(vco_freq, 1000000UL); /* vco_freq in MHz */

	if (phy->pdata->fdd) {
		tab = &SynthLUT_FDD[range][0];
	} else {
		tab = &SynthLUT_TDD[range][0];

	}

	if (tx)
		offs = REG_TX_VCO_OUTPUT - REG_RX_VCO_OUTPUT;

	while (i < SYNTH_LUT_SIZE && tab[i].VCO_MHz > vco_freq)
		i++;

	dev_dbg(&phy->spi->dev, "%s : freq %d MHz : index %d",
		__func__, tab[i].VCO_MHz, i);


	ad9361_spi_write(spi, REG_RX_VCO_OUTPUT + offs,
			       VCO_OUTPUT_LEVEL(tab[i].VCO_Output_Level) |
			       PORB_VCO_LOGIC);
	ad9361_spi_writef(spi, REG_RX_ALC_VARACTOR + offs,
			       VCO_VARACTOR(~0), tab[i].VCO_Varactor);
	ad9361_spi_write(spi, REG_RX_VCO_BIAS_1 + offs,
			       VCO_BIAS_REF(tab[i].VCO_Bias_Ref) |
			       VCO_BIAS_TCF(tab[i].VCO_Bias_Tcf));

	ad9361_spi_write(spi, REG_RX_FORCE_VCO_TUNE_1 + offs,
			       VCO_CAL_OFFSET(tab[i].VCO_Cal_Offset));
	ad9361_spi_write(spi, REG_RX_VCO_VARACTOR_CTRL_1 + offs,
			       VCO_VARACTOR_REFERENCE(
			       tab[i].VCO_Varactor_Reference));

	ad9361_spi_write(spi, REG_RX_VCO_CAL_REF, VCO_CAL_REF_TCF(0));

	ad9361_spi_write(spi, REG_RX_VCO_VARACTOR_CTRL_0,
				VCO_VARACTOR_OFFSET(0) |
				VCO_VARACTOR_REFERENCE_TCF(7));

	ad9361_spi_writef(spi, REG_RX_CP_CURRENT + offs, CHARGE_PUMP_CURRENT(~0),
			       tab[i].Charge_Pump_Current);
	ad9361_spi_write(spi, REG_RX_LOOP_FILTER_1 + offs,
			 LOOP_FILTER_C2(tab[i].LF_C2) |
			 LOOP_FILTER_C1(tab[i].LF_C1));
	ad9361_spi_write(spi, REG_RX_LOOP_FILTER_2 + offs,
			 LOOP_FILTER_R1(tab[i].LF_R1) |
			 LOOP_FILTER_C3(tab[i].LF_C3));
	ad9361_spi_write(spi, REG_RX_LOOP_FILTER_3 + offs,
			 LOOP_FILTER_R3(tab[i].LF_R3));

	return 0;
}

static int ad9361_get_split_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	u32 val, tbl_addr, lna_index, tia_index, mixer_index;
	int rc = 0;

	rx_gain->lmt_index = ad9361_spi_readf(spi, idx_reg,
					     FULL_TABLE_GAIN_INDEX(~0));
	tbl_addr = ad9361_spi_read(spi, REG_GAIN_TABLE_ADDRESS);

	ad9361_spi_write(spi, REG_GAIN_TABLE_ADDRESS, rx_gain->lmt_index);

	val = ad9361_spi_read(spi, REG_GAIN_TABLE_READ_DATA1);
	lna_index = TO_LNA_GAIN(val);
	mixer_index = TO_MIXER_GM_GAIN(val);

	tia_index = ad9361_spi_readf(spi, REG_GAIN_TABLE_READ_DATA2, TIA_GAIN);

	rx_gain->lmt_gain = lna_table[lna_index] +
				mixer_table[mixer_index] +
				tia_table[tia_index];

	ad9361_spi_write(spi, REG_GAIN_TABLE_ADDRESS, tbl_addr);

	/* Read LPF Index */
	rx_gain->lpf_gain = ad9361_spi_readf(spi, idx_reg + 1, LPF_GAIN_RX(~0));

	/* Read Digital Gain */
	rx_gain->digital_gain = ad9361_spi_readf(spi, idx_reg + 2,
						DIGITAL_GAIN_RX(~0));

	rx_gain->gain_db = rx_gain->lmt_gain + rx_gain->lpf_gain +
				rx_gain->digital_gain;
	return rc;
}

static int ad9361_get_full_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	u32 val;
	enum rx_gain_table_name tbl;
	struct rx_gain_info *gain_info;
	int rc = 0, rx_gain_db;

	tbl = ad9361_gt_tableindex(
		ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));

	val = ad9361_spi_readf(spi, idx_reg, FULL_TABLE_GAIN_INDEX(~0));

	gain_info = &phy->rx_gain[tbl];
	if (val > gain_info->idx_step_offset) {
		val = val - gain_info->idx_step_offset;
		rx_gain_db = gain_info->starting_gain_db +
			((val) * gain_info->gain_step_db);
	} else {
		rx_gain_db = gain_info->starting_gain_db;
	}

	rx_gain->gain_db = rx_gain_db;

	return rc;
}
static int ad9361_get_rx_gain(struct ad9361_rf_phy *phy,
		u32 rx_id, struct rf_rx_gain *rx_gain)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	u32 val, idx_reg;
	u8 gain_ctl_shift, rx_enable_mask;
	u8 fast_atk_shift;
	int rc = 0;

	if (rx_id == 1) {
		gain_ctl_shift = RX1_GAIN_CTRL_SHIFT;
		idx_reg = REG_GAIN_RX1;
		rx_enable_mask = RX_CHANNEL_ENABLE(RX_1);
		fast_atk_shift = RX1_FAST_ATK_SHIFT;

	} else if (rx_id == 2) {
		gain_ctl_shift = RX2_GAIN_CTRL_SHIFT;
		idx_reg = REG_GAIN_RX2;
		rx_enable_mask = RX_CHANNEL_ENABLE(RX_2);
		fast_atk_shift = RX2_FAST_ATK_SHIFT;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", rx_id);
		rc = -EINVAL;
		goto out;
	}

	val = ad9361_spi_readf(spi, REG_RX_ENABLE_FILTER_CTRL, rx_enable_mask);

	if (!val) {
		dev_err(dev, "Rx%d is not enabled\n", rx_gain->ant);
		rc = -EAGAIN;
		goto out;
	}

	val = ad9361_spi_read(spi, REG_AGC_CONFIG_1);

	val = (val >> gain_ctl_shift) & RX_GAIN_CTL_MASK;

	if (val == RX_GAIN_CTL_AGC_FAST_ATK) {
		/* In fast attack mode check whether Fast attack state machine
		 * has locked gain, if not then we can not read gain.
		 */
		val = ad9361_spi_read(spi, REG_FAST_ATTACK_STATE);
		val = (val >> fast_atk_shift) & FAST_ATK_MASK;
		if (val != FAST_ATK_GAIN_LOCKED) {
			dev_err(dev, "Failed to read gain, state m/c at %x\n",
				val);
			rc = -EAGAIN;
			goto out;
		}
	}

	if (phy->pdata->split_gt)
		rc = ad9361_get_split_table_gain(phy, idx_reg, rx_gain);
	else
		rc = ad9361_get_full_table_gain(phy, idx_reg, rx_gain);

out:
	return rc;
}

static void ad9361_ensm_force_state(struct ad9361_rf_phy *phy, u8 ensm_state)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	u8 dev_ensm_state;
	int rc;
	u32 val;

	dev_ensm_state = ad9361_spi_readf(spi, REG_STATE, ENSM_STATE(~0));

	phy->prev_ensm_state = dev_ensm_state;

	if (dev_ensm_state == ensm_state) {
		dev_dbg(dev, "Nothing to do, device is already in %d state\n",
			ensm_state);
		goto out;
	}

	dev_dbg(dev, "Device is in %x state, forcing to %x\n", dev_ensm_state,
			ensm_state);

	val = ad9361_spi_read(spi, REG_ENSM_CONFIG_1);

	/* Enable control through SPI writes, and take out from
	 * Alert
	 */
	if (val & ENABLE_ENSM_PIN_CTRL) {
		val &= ~ENABLE_ENSM_PIN_CTRL;
		phy->ensm_pin_ctl_en = 1;
	} else {
		phy->ensm_pin_ctl_en = 0;
	}

	if (dev_ensm_state & dev_ensm_state)
		val &= ~(TO_ALERT);

	switch (ensm_state) {

	case ENSM_STATE_TX:
		val |= FORCE_TX_ON;
		break;
	case ENSM_STATE_RX:
		val |= FORCE_RX_ON;
		break;
	case ENSM_STATE_FDD:
		val |= (FORCE_TX_ON | FORCE_RX_ON);
		break;
	case ENSM_STATE_ALERT:
		val &= ~(FORCE_TX_ON | FORCE_RX_ON);
		val |= TO_ALERT | FORCE_ALERT_STATE;
		break;
	default:
		dev_err(dev, "No handling for forcing %d ensm state\n",
		ensm_state);
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONFIG_1, val);
	if (rc)
		dev_err(dev, "Failed to restore state\n");

out:
	return;

}

static void ad9361_ensm_restore_prev_state(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc;
	u32 val;

	val = ad9361_spi_read(spi, REG_ENSM_CONFIG_1);

	/* We are restoring state only, so clear State bits first
	 * which might have set while forcing a particular state
	 */
	val &= ~(FORCE_TX_ON | FORCE_RX_ON |
			TO_ALERT | FORCE_ALERT_STATE);

	switch (phy->prev_ensm_state) {

	case ENSM_STATE_TX:
		val |= FORCE_TX_ON;
		break;
	case ENSM_STATE_RX:
		val |= FORCE_RX_ON;
		break;
	case ENSM_STATE_FDD:
		val |= (FORCE_TX_ON | FORCE_RX_ON);
		break;
	case ENSM_STATE_ALERT:
		val |= TO_ALERT;
		break;
	case ENSM_STATE_INVALID:
		dev_dbg(dev, "No need to restore, ENSM state wasn't saved\n");
		goto out;
	default:
		dev_dbg(dev, "Could not restore to %d ENSM state\n",
		phy->prev_ensm_state);
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONFIG_1, val);
	if (rc) {
		dev_err(dev, "Failed to write ENSM_CONFIG_1");
		goto out;
	}

	if (phy->ensm_pin_ctl_en) {
		val |= ENABLE_ENSM_PIN_CTRL;
		rc = ad9361_spi_write(spi, REG_ENSM_CONFIG_1, val);
		if (rc)
			dev_err(dev, "Failed to write ENSM_CONFIG_1");
	}

out:
	return;
}

static int set_split_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	int rc = 0;

	if ((rx_gain->lmt_index > MAX_LMT_INDEX) ||
			(rx_gain->lpf_gain > MAX_LPF_GAIN) ||
			(rx_gain->digital_gain > MAX_DIG_GAIN)) {
		dev_err(dev, "LMT_INDEX missing or greater than max value %d",
				MAX_LMT_INDEX);
		dev_err(dev, "LPF_GAIN missing or greater than max value %d",
				MAX_LPF_GAIN);
		dev_err(dev, "DIGITAL_GAIN cannot be more than %d",
				MAX_DIG_GAIN);
		rc = -EINVAL;
		goto out;
	}
	if (rx_gain->gain_db > 0)
		dev_dbg(dev, "Ignoring rx_gain value in split table mode.");
	if (rx_gain->lmt_index == 0 && rx_gain->lpf_gain == 0 &&
			rx_gain->digital_gain == 0) {
		dev_err(dev,
		"In split table mode, All LMT/LPF/digital gains cannot be 0");
		rc = -EINVAL;
		goto out;
	}

	ad9361_spi_writef(spi, idx_reg, RX_FULL_TBL_IDX_MASK, rx_gain->lmt_index);
	ad9361_spi_writef(spi, idx_reg + 1, RX_LPF_IDX_MASK, rx_gain->lpf_gain);

	if (phy->pdata->gain_ctrl.dig_gain_en) {
		ad9361_spi_writef(spi, idx_reg + 2, RX_DIGITAL_IDX_MASK, rx_gain->digital_gain);

	} else if (rx_gain->digital_gain > 0) {
		dev_err(dev, "Digital gain is disabled and cannot be set");
	}
out:
	return rc;
}

static int set_full_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	enum rx_gain_table_name tbl;
	struct rx_gain_info *gain_info;
	u32 val;
	int rc = 0;

	if (rx_gain->lmt_index != ~0 || rx_gain->lpf_gain != ~0 ||
			rx_gain->digital_gain > 0)
		dev_dbg(dev,
			"Ignoring lmt/lpf/digital gains in Single Table mode");

	tbl = ad9361_gt_tableindex(
		ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));

	gain_info = &phy->rx_gain[tbl];
	if ((rx_gain->gain_db < gain_info->starting_gain_db) ||
		(rx_gain->gain_db > gain_info->max_gain_db)) {

		dev_err(dev, "Invalid gain %d, supported range [%d - %d]\n",
			rx_gain->gain_db, gain_info->starting_gain_db,
			gain_info->max_gain_db);
		rc = -EINVAL;
		goto out;

	}

	val = ((rx_gain->gain_db - gain_info->starting_gain_db) /
		gain_info->gain_step_db) + gain_info->idx_step_offset;
	ad9361_spi_writef(spi, idx_reg, RX_FULL_TBL_IDX_MASK, val);

out:
	return rc;
}

static int ad9361_set_rx_gain(struct ad9361_rf_phy *phy,
		u32 rx_id, struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	u32 val, idx_reg;
	u8 gain_ctl_shift, ensm_state;
	int rc = 0;

	if (rx_id == 1) {
		gain_ctl_shift = RX1_GAIN_CTRL_SHIFT;
		idx_reg = REG_RX1_MANUAL_LMT_FULL_GAIN;

	} else if (rx_id == 2) {
		gain_ctl_shift = RX2_GAIN_CTRL_SHIFT;
		idx_reg = REG_RX2_MANUAL_LMT_FULL_GAIN;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", rx_id);
		rc = -EINVAL;
		goto out;

	}

	val = ad9361_spi_read(spi, REG_AGC_CONFIG_1);
	val = (val >> gain_ctl_shift) & RX_GAIN_CTL_MASK;

	if (val != RX_GAIN_CTL_MGC) {
		dev_dbg(dev, "Rx gain can be set in MGC mode only\n");
		goto out;
	}

	if (phy->pdata->fdd)
		ensm_state = ENSM_STATE_FDD;
	else
		ensm_state = ENSM_STATE_RX;

	/* RX must be enabled while changing Gain */
	ad9361_ensm_force_state(phy, ensm_state);

	if (phy->pdata->split_gt)
		rc = set_split_table_gain(phy, idx_reg, rx_gain);
	else
		rc = set_full_table_gain(phy, idx_reg, rx_gain);

	/* Restore is done intentionally before checking rc, because
	 * we need to restore PHY to previous state even if write failed
	 */
	ad9361_ensm_restore_prev_state(phy);

	if (rc) {
		dev_err(dev, "Unable to write gain tbl idx reg: %d\n", idx_reg);
		goto out;
	}

out:
	return rc;

}

void ad9361_init_gain_info(struct rx_gain_info *rx_gain,
	enum rx_gain_table_type type, int starting_gain,
	int max_gain, int gain_step, int max_idx, int idx_offset)
{
	rx_gain->tbl_type = type;
	rx_gain->starting_gain_db = starting_gain;
	rx_gain->max_gain_db = max_gain;
	rx_gain->gain_step_db = gain_step;
	rx_gain->max_idx = max_idx;
	rx_gain->idx_step_offset = idx_offset;
}

int ad9361_init_gain_tables(struct ad9361_rf_phy *phy)
{
	struct rx_gain_info *rx_gain;

	/* Intialize Meta data according to default gain tables
	 * of AD9631. Changing/Writing of gain tables is not
	 * supported yet.
	 */
	rx_gain = &phy->rx_gain[TBL_200_1300_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, 1, 77, 1,
		SIZE_FULL_TABLE, 0);

	rx_gain = &phy->rx_gain[TBL_1300_4000_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, -4, 71, 1,
		SIZE_FULL_TABLE, 1);

	rx_gain = &phy->rx_gain[TBL_4000_6000_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, -10, 62, 1,
		SIZE_FULL_TABLE, 4);

	return 0;
}

int ad9361_en_dis_tx(struct ad9361_rf_phy *phy, u32 tx_if, u32 enable)
{
	if (tx_if == 2 && !phy->pdata->rx2tx2)
		return -EINVAL;

	return ad9361_spi_writef(phy->spi, REG_TX_ENABLE_FILTER_CTRL,
			TX_CHANNEL_ENABLE(tx_if), enable);
}

int ad9361_en_dis_rx(struct ad9361_rf_phy *phy, u32 rx_if, u32 enable)
{
	if (rx_if == 2 && !phy->pdata->rx2tx2)
		return -EINVAL;

	return ad9361_spi_writef(phy->spi, REG_RX_ENABLE_FILTER_CTRL,
			  RX_CHANNEL_ENABLE(rx_if), enable);
}


int ad9361_set_gain_ctrl_mode(struct ad9361_rf_phy *phy,
		struct rf_gain_ctrl *gain_ctrl)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc = 0;
	u32 gain_ctl_shift, mode;
	u8 val;

	rc = ad9361_spi_readm(spi, REG_AGC_CONFIG_1, &val, 1);
	if (rc) {
		dev_err(dev, "Unable to read AGC config1 register: %x\n",
			REG_AGC_CONFIG_1);
		goto out;
	}

	switch (gain_ctrl->mode) {
	case RF_GAIN_MGC:
		mode = RX_GAIN_CTL_MGC;
		break;
	case RF_GAIN_FASTATTACK_AGC:
		mode = RX_GAIN_CTL_AGC_FAST_ATK;
		break;
	case RF_GAIN_SLOWATTACK_AGC:
		mode = RX_GAIN_CTL_AGC_SLOW_ATK;
		break;
	case RF_GAIN_HYBRID_AGC:
		mode = RX_GAIN_CTL_AGC_SLOW_ATK_HYBD;
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

	if (gain_ctrl->ant == 1) {
		gain_ctl_shift = RX1_GAIN_CTRL_SHIFT;
	} else if (gain_ctrl->ant == 2) {
		gain_ctl_shift = RX2_GAIN_CTRL_SHIFT;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", gain_ctrl->ant);
		rc = -EINVAL;
		goto out;
	}

	rc = ad9361_en_dis_rx(phy, gain_ctrl->ant, RX_DISABLE);
	if (rc) {
		dev_err(dev, "Unable to disable rx%d\n", gain_ctrl->ant);
		goto out;
	}

	val &= ~(RX_GAIN_CTL_MASK << gain_ctl_shift);
	val |= mode << gain_ctl_shift;
	if (mode == RX_GAIN_CTL_AGC_SLOW_ATK_HYBD)
		val |= SLOW_ATTACK_HYBRID_MODE;
	else
		val &= ~SLOW_ATTACK_HYBRID_MODE;

	rc = ad9361_spi_write(spi, REG_AGC_CONFIG_1, val);
	if (rc) {
		dev_err(dev, "Unable to write AGC config1 register: %x\n",
				REG_AGC_CONFIG_1);
		goto out;
	}

	rc = ad9361_en_dis_rx(phy, gain_ctrl->ant, RX_ENABLE);
out:
	return rc;
}

static int ad9361_read_rssi(struct ad9361_rf_phy *phy, struct rf_rssi *rssi)
{
	struct spi_device *spi = phy->spi;
	u8 reg_val_buf[6];
	int rc;

	rc = ad9361_spi_readm(spi, REG_PREAMBLE_LSB,
			reg_val_buf, ARRAY_SIZE(reg_val_buf));
	if (rssi->ant == 1) {
		rssi->symbol = RSSI_RESOLUTION *
				((reg_val_buf[5] << RSSI_LSB_SHIFT) +
				 (reg_val_buf[1] & RSSI_LSB_MASK1));
		rssi->preamble = RSSI_RESOLUTION *
				((reg_val_buf[4] << RSSI_LSB_SHIFT) +
				 (reg_val_buf[0] & RSSI_LSB_MASK1));
	} else if (rssi->ant == 2) {
		rssi->symbol = RSSI_RESOLUTION *
				((reg_val_buf[3] << RSSI_LSB_SHIFT) +
				 ((reg_val_buf[1] & RSSI_LSB_MASK2) >> 1));
		rssi->preamble = RSSI_RESOLUTION *
				((reg_val_buf[2] << RSSI_LSB_SHIFT) +
				 ((reg_val_buf[0] & RSSI_LSB_MASK2) >> 1));
	} else
		rc = -EFAULT;

	rssi->multiplier = RSSI_MULTIPLIER;

	return rc;
}

static int ad9361_rx_adc_setup(struct ad9361_rf_phy *phy, unsigned long bbpll_freq,
			 unsigned long adc_sampl_freq_Hz)
{

	unsigned long scale_snr_1e3, maxsnr, sqrt_inv_rc_tconst_1e3, tmp_1e3,
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3, sqrt_term_1e3,
		min_sqrt_term_1e3, bb_bw_Hz;
	u64 tmp, invrc_tconst_1e6;
	u8 data[40];
	u32 i;
	int ret;

	u8 c3_msb = ad9361_spi_read(phy->spi, REG_RX_BBF_C3_MSB);
	u8 c3_lsb = ad9361_spi_read(phy->spi, REG_RX_BBF_C3_LSB);
	u8 r2346 = ad9361_spi_read(phy->spi, REG_RX_BBF_R2346);

	/*
	 * BBBW = (BBPLL / RxTuneDiv) * ln(2) / (1.4 * 2PI )
	 * We assume ad9361_rx_bb_analog_filter_calib() is always run prior
	 */

	tmp = bbpll_freq * 10000ULL;
	do_div(tmp, 126906UL * phy->rxbbf_div);
	bb_bw_Hz = tmp;

	dev_dbg(&phy->spi->dev, "%s : BBBW %lu : ADCfreq %lu",
		__func__, bb_bw_Hz, adc_sampl_freq_Hz);

	dev_dbg(&phy->spi->dev, "c3_msb 0x%X : c3_lsb 0x%X : r2346 0x%X : ",
		c3_msb, c3_lsb, r2346);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 28000000UL);

	if (adc_sampl_freq_Hz < 80000000)
		scale_snr_1e3 = 1000;
	else
		scale_snr_1e3 = 1585; /* pow(10, scale_snr_dB/10); */

 	if (bb_bw_Hz >= 18000000) {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz) * (1000 + (10 * (bb_bw_Hz - 18000000) / 1000000)));

		do_div(invrc_tconst_1e6, 1000UL);

	} else {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz));
	}

	do_div(invrc_tconst_1e6, 1000000000UL);

	if (invrc_tconst_1e6 > ULONG_MAX)
		dev_err(&phy->spi->dev, "invrc_tconst_1e6 > ULONG_MAX");

	sqrt_inv_rc_tconst_1e3 = int_sqrt((u32)invrc_tconst_1e6);
	maxsnr = 640/160;
	scaled_adc_clk_1e6 = DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 640);
	inv_scaled_adc_clk_1e3 = DIV_ROUND_CLOSEST(640000000,
			DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 1000));
	tmp_1e3 = DIV_ROUND_CLOSEST(980000 + 20 * max_t(u32, 1000U,
			DIV_ROUND_CLOSEST(inv_scaled_adc_clk_1e3, maxsnr)), 1000);
	sqrt_term_1e3 = int_sqrt(scaled_adc_clk_1e6);
	min_sqrt_term_1e3 = min_t(u32, 1000U,
			int_sqrt(maxsnr * scaled_adc_clk_1e6));

	dev_dbg(&phy->spi->dev, "invrc_tconst_1e6 %llu, sqrt_inv_rc_tconst_1e3 %lu\n",
		invrc_tconst_1e6, sqrt_inv_rc_tconst_1e3);
	dev_dbg(&phy->spi->dev, "scaled_adc_clk_1e6 %lu, inv_scaled_adc_clk_1e3 %lu\n",
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3);
	dev_dbg(&phy->spi->dev, "tmp_1e3 %lu, sqrt_term_1e3 %lu, min_sqrt_term_1e3 %lu\n",
		tmp_1e3, sqrt_term_1e3, min_sqrt_term_1e3);

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0x24;
	data[4] = 0x24;
	data[5] = 0;
	data[6] = 0;

	tmp = -50000000 + 8ULL * scale_snr_1e3 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3;
	do_div(tmp, 100000000UL);
	data[7] = min_t(u64, 124U, tmp);

	tmp = (invrc_tconst_1e6 >> 1) + 20 * inv_scaled_adc_clk_1e3 *
		data[7] / 80 * 1000ULL;
	do_div(tmp, invrc_tconst_1e6);
	data[8] = min_t(u64, 255U, tmp);

	tmp = (-500000 + 77ULL * sqrt_inv_rc_tconst_1e3 * min_sqrt_term_1e3);
	do_div(tmp, 1000000UL);
	data[10] = min_t(u64, 127U, tmp);

	data[9] = min_t(u32, 127U, ((800 * data[10]) / 1000));
	tmp = ((invrc_tconst_1e6 >> 1) + (20 * inv_scaled_adc_clk_1e3 *
		data[10] * 1000ULL));
	do_div(tmp, invrc_tconst_1e6 * 77);
	data[11] = min_t(u64, 255U, tmp);
	data[12] = min_t(u32, 127U, (-500000 + 80 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3) / 1000000UL);

	tmp = -3*(long)(invrc_tconst_1e6 >> 1) + inv_scaled_adc_clk_1e3 *
		data[12] * (1000ULL * 20 / 80);
	do_div(tmp, invrc_tconst_1e6);
	data[13] = min_t(u64, 255, tmp);

	data[14] = 21 * (inv_scaled_adc_clk_1e3 / 10000);
	data[15] = min_t(u32, 127U, (500 + 1025 * data[7]) / 1000);
	data[16] = min_t(u32, 127U, (data[15] * tmp_1e3) / 1000);
	data[17] = data[15];
	data[18] = min_t(u32, 127U, (500 + 975 * data[10]) / 1000);
	data[19] = min_t(u32, 127U, (data[18] * tmp_1e3) / 1000);
	data[20] = data[18];
	data[21] = min_t(u32, 127U, (500 + 975 * data[12]) / 1000);
	data[22] = min_t(u32, 127, (data[21] * tmp_1e3) / 1000);
	data[23] = data[21];
	data[24] = 0x2E;
	data[25] = (128 + min_t(u32, 63000U, DIV_ROUND_CLOSEST(63 *
		scaled_adc_clk_1e6, 1000)) / 1000);
	data[26] = min_t(u32, 63U,63 * scaled_adc_clk_1e6 / 1000000 *
		(920 + 80 * inv_scaled_adc_clk_1e3 / 1000) / 1000);
	data[27] = min_t(u32, 63,(32 * sqrt_term_1e3) / 1000);
	data[28] = data[25];
	data[29] = data[26];
	data[30] = data[27];
	data[31] = data[25];
	data[32] = data[26];
	data[33] = min_t(u32, 63U, 63 * sqrt_term_1e3 / 1000);
	data[34] = min_t(u32, 127U, 64 * sqrt_term_1e3 / 1000);
	data[35] = 0x40;
	data[36] = 0x40;
	data[37] = 0x2C;
	data[38] = 0x00;
	data[39] = 0x00;

 	for (i = 0; i < 40; i++) {
		ret = ad9361_spi_write(phy->spi, 0x200 + i, data[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9361_rx_tia_calib(struct ad9361_rf_phy *phy, unsigned long bb_bw_Hz)
{
	unsigned long Cbbf, R2346;
	u64 CTIA_fF;

	u8 reg1EB = ad9361_spi_read(phy->spi, REG_RX_BBF_C3_MSB);
	u8 reg1EC = ad9361_spi_read(phy->spi, REG_RX_BBF_C3_LSB);
	u8 reg1E6 = ad9361_spi_read(phy->spi, REG_RX_BBF_R2346);
	u8 reg1DB, reg1DF, reg1DD, reg1DC, reg1DE, temp;

	dev_dbg(&phy->spi->dev, "%s : bb_bw_Hz %lu",
		__func__, bb_bw_Hz);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 20000000UL);

	Cbbf = (reg1EB * 160) + (reg1EC * 10) + 140; /* fF */
	R2346 = 18300 * RX_BBF_R2346(reg1E6);

	CTIA_fF = Cbbf * R2346 * 560ULL;
	do_div(CTIA_fF, 3500000UL);

	if (bb_bw_Hz <= 3000000UL)
		reg1DB = 0xE0;
	else if (bb_bw_Hz <= 10000000UL)
		reg1DB = 0x60;
	else
		reg1DB = 0x20;

	if (CTIA_fF > 2920ULL) {
		reg1DC = 0x40;
		reg1DE = 0x40;
		temp = min(127U, DIV_ROUND_CLOSEST((u32)CTIA_fF - 400, 320U));
		reg1DD = temp;
		reg1DF = temp;
	} else {
		temp = DIV_ROUND_CLOSEST((u32)CTIA_fF - 400, 40U) + 0x40;
		reg1DC = temp;
		reg1DE = temp;
		reg1DD = 0;
		reg1DF = 0;
	}

	ad9361_spi_write(phy->spi, REG_RX_TIA_CONFIG, reg1DB);
	ad9361_spi_write(phy->spi, REG_TIA1_C_LSB, reg1DC);
	ad9361_spi_write(phy->spi, REG_TIA1_C_MSB, reg1DD);
	ad9361_spi_write(phy->spi, REG_TIA2_C_LSB, reg1DE);
	ad9361_spi_write(phy->spi, REG_TIA2_C_MSB, reg1DF);

	return 0;
}

/* BASEBAND RX ANALOG FILTER CALIBRATION */

static int ad9361_rx_bb_analog_filter_calib(struct ad9361_rf_phy *phy,
					    unsigned long rx_bb_bw,
					    unsigned long bbpll_freq)
{
	unsigned long target;
	u8 tmp;
	int ret;

	dev_dbg(&phy->spi->dev, "%s : rx_bb_bw %lu bbpll_freq %lu",
		__func__, rx_bb_bw, bbpll_freq);

	rx_bb_bw = clamp(rx_bb_bw, 200000UL, 28000000UL);

	/* 1.4 * BBBW * 2PI / ln(2) */
	target =  126906UL * (rx_bb_bw / 10000UL);
	phy->rxbbf_div = min_t(unsigned long, 511UL, DIV_ROUND_UP(bbpll_freq, target));

	/* Set RX baseband filter divide value */
	ad9361_spi_write(phy->spi, REG_RX_BBF_TUNE_DIVIDE, phy->rxbbf_div);
	ad9361_spi_writef(phy->spi, REG_RX_BBF_TUNE_CONFIG, BIT(0), phy->rxbbf_div >> 8);

	/* Write the BBBW into registers 0x1FB and 0x1FC */
	ad9361_spi_write(phy->spi, REG_RX_BBBW_MHZ, rx_bb_bw / 1000000UL);

	tmp = DIV_ROUND_CLOSEST((rx_bb_bw % 1000000UL) * 128, 1000000UL);
	ad9361_spi_write(phy->spi, REG_RX_BBBW_KHZ, min_t(u8, 127, tmp));

	ad9361_spi_write(phy->spi, REG_RX_MIX_LO_CM, RX_MIX_LO_CM(0x3F)); /* Set Rx Mix LO CM */
	ad9361_spi_write(phy->spi, REG_RX_MIX_GM_CONFIG, RX_MIX_GM_PLOAD(3)); /* Set GM common mode */

	/* Enable the RX BBF tune circuit by writing 0x1E2=0x02 and 0x1E3=0x02 */
	ad9361_spi_write(phy->spi, REG_RX1_TUNE_CTRL, RX1_TUNE_RESAMPLE);
	ad9361_spi_write(phy->spi, REG_RX2_TUNE_CTRL, RX2_TUNE_RESAMPLE);

	/* Start the RX Baseband Filter calibration in register 0x016[7] */
	/* Calibration is complete when register 0x016[7] self clears */
	ret = ad9361_run_calibration(phy, RX_BB_TUNE_CAL);

	/* Disable the RX baseband filter tune circuit, write 0x1E2=3, 0x1E3=3 */
	ad9361_spi_write(phy->spi, REG_RX1_TUNE_CTRL,
			RX1_TUNE_RESAMPLE | RX1_PD_TUNE);
	ad9361_spi_write(phy->spi, REG_RX2_TUNE_CTRL,
			RX2_TUNE_RESAMPLE | RX2_PD_TUNE);

	return ret;
}

/* BASEBAND TX ANALOG FILTER CALIBRATION */

static int ad9361_tx_bb_analog_filter_calib(struct ad9361_rf_phy *phy,
					    unsigned long tx_bb_bw,
					    unsigned long bbpll_freq)
{
	unsigned long target, txbbf_div;
	int ret;

	dev_dbg(&phy->spi->dev, "%s : tx_bb_bw %lu bbpll_freq %lu",
		__func__, tx_bb_bw, bbpll_freq);

	tx_bb_bw = clamp(tx_bb_bw, 625000UL, 20000000UL);

	/* 1.6 * BBBW * 2PI / ln(2) */
	target =  132345 * (tx_bb_bw / 10000UL);
	txbbf_div = min_t(unsigned long, 511UL, DIV_ROUND_UP(bbpll_freq, target));

	/* Set TX baseband filter divide value */
	ad9361_spi_write(phy->spi, REG_TX_BBF_TUNE_DIVIDER, txbbf_div);
	ad9361_spi_writef(phy->spi, REG_TX_BBF_TUNE_MODE,
			  TX_BBF_TUNE_DIVIDER, txbbf_div >> 8);

	/* Enable the TX baseband filter tune circuit by setting 0x0CA=0x22. */
	ad9361_spi_write(phy->spi, REG_TX_TUNE_CTRL, TUNER_RESAMPLE | TUNE_CTRL(1));

	/* Start the TX Baseband Filter calibration in register 0x016[6] */
	/* Calibration is complete when register 0x016[] self clears */
	ret = ad9361_run_calibration(phy, TX_BB_TUNE_CAL);

	/* Disable the TX baseband filter tune circuit by writing 0x0CA=0x26. */
	ad9361_spi_write(phy->spi, REG_TX_TUNE_CTRL,
			 TUNER_RESAMPLE | TUNE_CTRL(1) | PD_TUNE);

	return ret;
}

/* BASEBAND TX SECONDARY FILTER */

static int ad9361_tx_bb_second_filter_calib(struct ad9361_rf_phy *phy,
					   unsigned long tx_rf_bw)
{
	u64 cap;
	unsigned long corner, res, div;
	u32 reg_conf, reg_res;
	int ret, i;

	dev_dbg(&phy->spi->dev, "%s : tx_rf_bw %lu",
		__func__, tx_rf_bw);

	tx_rf_bw = clamp(tx_rf_bw, 1060000UL, 40000000UL);

	/* BBBW * 5PI */
	corner = 15708 * (tx_rf_bw / 20000UL);

	for (i = 0, res = 1; i < 4; i++) {
		div = corner * res;
		cap = (500000000ULL) + (div >> 1);
		do_div(cap, div);
		cap -= 12ULL;
		if (cap < 64ULL)
			break;

		res <<= 1;
	}

	if (cap > 63ULL)
		cap = 63ULL;

	if(tx_rf_bw <= 9000000UL )
		reg_conf = 0x59;
	else if (tx_rf_bw <= 24000000UL)
		reg_conf = 0x56;
	else
		reg_conf = 0x57;

	switch (res) {
	case 1:
		reg_res = 0x0C;
		break;
	case 2:
		reg_res = 0x04;
		break;
	case 4:
		reg_res = 0x03;
		break;
	case 8:
		reg_res = 0x01;
		break;
	default:
		reg_res = 0x01;
	}

	ret = ad9361_spi_write(phy->spi, REG_CONFIG0, reg_conf);
	ret |= ad9361_spi_write(phy->spi, REG_RESISTOR, reg_res);
	ret |= ad9361_spi_write(phy->spi, REG_CAPACITOR, (u8)cap);

	return ret;
}

/* RF SYNTHESIZER CHARGE PUMP CALIBRATION */

static int ad9361_txrx_synth_cp_calib(struct ad9361_rf_phy *phy,
					   unsigned long ref_clk_hz, bool tx)
{
	u32 offs = tx ? 0x40 : 0;
	u32 vco_cal_cnt;
	int ret;

	dev_dbg(&phy->spi->dev, "%s : ref_clk_hz %lu : is_tx %d",
		__func__, ref_clk_hz, tx);

	ad9361_spi_write(phy->spi, REG_RX_LO_GEN_POWER_MODE + offs, 0x00);
	ad9361_spi_write(phy->spi, REG_RX_VCO_LDO + offs, 0x0B);
	ad9361_spi_write(phy->spi, REG_RX_VCO_PD_OVERRIDES + offs, 0x02);
	ad9361_spi_write(phy->spi, REG_RX_CP_CURRENT + offs, 0x80);
	ad9361_spi_write(phy->spi, REG_RX_CP_CONFIG + offs, 0x00);

	/* see Table 70 Example Calibration Times for RF VCO Cal */
	if (phy->pdata->fdd) {
		vco_cal_cnt = 0x8E;
	} else {
		if (ref_clk_hz >= 50000000UL)
			vco_cal_cnt = 0x86;
		else
			vco_cal_cnt = 0x82;
	}

	ad9361_spi_write(phy->spi, REG_RX_VCO_CAL + offs, vco_cal_cnt);

	/* Enable FDD mode during calibrations */

	if (!phy->pdata->fdd)
		ad9361_spi_write(phy->spi, REG_PARALLEL_PORT_CONF_3, LVDS_MODE);

	ad9361_spi_write(phy->spi, REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE);
	ad9361_spi_write(phy->spi, REG_ENSM_CONFIG_1,
			FORCE_ALERT_STATE |
			TO_ALERT);
	ad9361_spi_write(phy->spi, REG_ENSM_MODE, FDD_MODE);

	ad9361_spi_write(phy->spi, REG_RX_CP_CONFIG + offs, CP_CAL_ENABLE);

	ret = ad9361_check_cal_done(phy, REG_RX_CAL_STATUS + offs,
				    CP_CAL_VALID, 1);
	ad9361_spi_write(phy->spi, REG_RX_CP_CONFIG + offs, 0x0);

	return ret;
}

/* BASEBAND DC OFFSET CALIBRATION */
static int ad9361_bb_dc_offset_calib(struct ad9361_rf_phy *phy)
{
	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(phy->spi, REG_BB_DC_OFFSET_COUNT, 0x3F);
	ad9361_spi_write(phy->spi, REG_BB_DC_OFFSET_SHIFT, BB_DC_M_SHIFT(0xF));
	ad9361_spi_write(phy->spi, REG_BB_DC_OFFSET_ATTEN, BB_DC_OFFSET_ATTEN(1));

	return ad9361_run_calibration(phy, BBDC_CAL);
}

/* RF DC OFFSET CALIBRATION */

static int ad9361_rf_dc_offset_calib(struct ad9361_rf_phy *phy,
				     u64 rx_freq)
{
	struct spi_device *spi = phy->spi;

	dev_dbg(&phy->spi->dev, "%s : rx_freq %llu",
		__func__, rx_freq);

// 	ad9361_spi_write(spi, REG_ENSM_CONFIG_1,
// 			ENABLE_ENSM_PIN_CTRL |
// 			FORCE_ALERT_STATE |
// 			TO_ALERT);

	ad9361_spi_write(spi, REG_WAIT_COUNT, 0x20);

	if(rx_freq <= 4000000000ULL) {
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_COUNT, 0x32);
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_CONFIG_1,
				 RF_DC_CALIBRATION_COUNT(4) | DAC_FS(2));
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_ATTEN,
				 RF_DC_OFFSET_ATTEN(5));
	} else {
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_COUNT, 0x28);
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_CONFIG_1,
				 RF_DC_CALIBRATION_COUNT(4) | DAC_FS(3));
		ad9361_spi_write(spi, REG_RF_DC_OFFSET_ATTEN,
				 RF_DC_OFFSET_ATTEN(6));
	}

	ad9361_spi_write(spi, REG_DC_OFFSET_CONFIG2,
			 USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL |
			 DC_OFFSET_UPDATE(3));

	ad9361_spi_write(spi, REG_INVERT_BITS,
			 INVERT_RX1_RF_DC_CGOUT_WORD |
			 INVERT_RX2_RF_DC_CGOUT_WORD);

	return ad9361_run_calibration(phy, RFDC_CAL);
}

/* TX QUADRATURE CALIBRATION */

static int ad9361_tx_quad_calib(struct ad9361_rf_phy *phy,
					   unsigned long bw)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	unsigned long clktf, clkrf;
	int txnco_word, rxnco_word;
	u8 rx_phase = 0;
	const u8 (*tab)[3];
	u32 index_max, i , lpf_tia_mask;
	/*
	 * Find NCO frequency that matches this equation:
	 * BW / 4 = Rx NCO freq = Tx NCO freq:
	 * Rx NCO = ClkRF * (rxNCO <1:0> + 1) / 32
	 * Tx NCO = ClkTF * (txNCO <1:0> + 1) / 32
	 */

	clkrf = clk_get_rate(phy->clks[CLKRF_CLK]);
	clktf = clk_get_rate(phy->clks[CLKTF_CLK]);

	dev_dbg(&phy->spi->dev, "%s : bw %lu clkrf %lu clktf %lu",
		__func__, bw, clkrf, clktf);

	txnco_word = DIV_ROUND_CLOSEST(bw * 8, clktf) - 1;
	txnco_word = clamp_t(int, txnco_word, 0, 3);

 	dev_dbg(dev, "Tx NCO frequency: %lu (BW/4: %lu) txnco_word %d\n",
		clktf * (txnco_word + 1) / 32, bw / 4, txnco_word);

	rxnco_word = txnco_word;

	if (clkrf == (2 * clktf)) {
		rx_phase = 0x0E;
		switch (txnco_word) {
		case 0:
			txnco_word++;
			break;
		case 1:
			rxnco_word--;
			break;
		case 2:
			rxnco_word-=2;
			txnco_word--;
			break;
		case 3:
			rxnco_word-=2;	/* REVISIT */
			rx_phase = 0x08;
			break;
		}
	} else if (clkrf == clktf) {
		switch (txnco_word) {
		case 0:
		case 3:
			rx_phase = 0x15;
			break;
		case 2:
			rx_phase = 0x1F;
			break;
		case 1:
			if (ad9361_spi_readf(spi,
				REG_TX_ENABLE_FILTER_CTRL, 0x3F) == 0x22)
				rx_phase = 0x15; 	/* REVISIT */
			else
				rx_phase = 0x1A;
			break;
		}
	} else
		dev_err(dev, "Error in %s line %d clkrf %lu clktf %lu\n",
			__func__, __LINE__, clkrf, clktf);

	ad9361_spi_write(spi, REG_QUAD_CAL_NCO_FREQ_PHASE_OFFSET,
			 RX_NCO_FREQ(rxnco_word) | RX_NCO_PHASE_OFFSET(rx_phase));
	ad9361_spi_writef(spi, REG_KEXP_2, TX_NCO_FREQ(~0), txnco_word);

	ad9361_spi_write(spi, REG_QUAD_CAL_CTRL,
			 SETTLE_MAIN_ENABLE | DC_OFFSET_ENABLE |
			 GAIN_ENABLE | PHASE_ENABLE | M_DECIM(3));
	ad9361_spi_write(spi, REG_QUAD_CAL_COUNT, 0xFF);
	ad9361_spi_write(spi, REG_KEXP_1, KEXP_TX(1) | KEXP_TX_COMP(3) |
			 KEXP_DC_I(3) | KEXP_DC_Q(3));
	ad9361_spi_write(spi, REG_MAG_FTEST_THRESH, 0x01);
	ad9361_spi_write(spi, REG_MAG_FTEST_THRESH_2, 0x01);

	if (phy->pdata->split_gt) {
		tab = &split_gain_table[phy->current_table][0];
		index_max = SIZE_SPLIT_TABLE;
		lpf_tia_mask = 0x20;
	} else {
		tab = &full_gain_table[phy->current_table][0];
		index_max = SIZE_FULL_TABLE;
		lpf_tia_mask = 0x3F;
	}

	for (i = 0; i < index_max; i++)
		if ((tab[i][1] & lpf_tia_mask) == 0x20) {
			ad9361_spi_write(spi, REG_TX_QUAD_FULL_LMT_GAIN, i);
			break;
		}

	if (i >= index_max)
		dev_err(dev, "failed to find suitable LPF TIA value in gain table\n");

	ad9361_spi_write(spi, REG_QUAD_SETTLE_COUNT, 0xF0);
	ad9361_spi_write(spi, REG_TX_QUAD_LPF_GAIN, 0x00);

	return ad9361_run_calibration(phy, TX_QUAD_CAL);
}

static int ad9361_rx_quad_calib(struct ad9361_rf_phy *phy,
					   unsigned long bw)
{
	return -EINVAL; /* TODO */
}

static int ad9361_tracking_control(struct ad9361_rf_phy *phy, bool bbdc_track,
				   bool rfdc_track, bool rxquad_track)
{
	struct spi_device *spi = phy->spi;
	u32 qtrack = 0;

	dev_dbg(&spi->dev, "%s : bbdc_track=%d, rfdc_track=%d, rxquad_track=%d",
		__func__, bbdc_track, rfdc_track, rxquad_track);

	ad9361_spi_write(spi, REG_CALIBRATION_CONFIG_2,
			 CALIBRATION_CONFIG2_DFLT  | K_EXP_PHASE(0x15));
	ad9361_spi_write(spi, REG_CALIBRATION_CONFIG_3,
			 PREVENT_POS_LOOP_GAIN | K_EXP_AMPLITUDE(0x15));

	ad9361_spi_writef(spi, REG_DC_OFFSET_CONFIG2,
			  DC_OFFSET_UPDATE(~0), 0x5); /* Gain change + Rx exit */
	ad9361_spi_writef(spi, REG_DC_OFFSET_CONFIG2,
			  ENABLE_BB_DC_OFFSET_TRACKING, bbdc_track);
	ad9361_spi_writef(spi, REG_DC_OFFSET_CONFIG2,
			  ENABLE_RF_OFFSET_TRACKING, rfdc_track);

	if (rxquad_track)
		qtrack = ENABLE_TRACKING_MODE_CH1 |
			(phy->pdata->rx2tx2 ? ENABLE_TRACKING_MODE_CH2 : 0);

	ad9361_spi_write(spi, REG_CALIBRATION_CONFIG_1,
			 ENABLE_PHASE_CORR | ENABLE_GAIN_CORR |
			 FREE_RUN_MODE | ENABLE_CORR_WORD_DECIMATION |
			 qtrack);

	return 0;
}


/* REFERENCE CLOCK DELAY UNIT COUNTER REGISTER */
static int ad9361_set_ref_clk_cycles(struct ad9361_rf_phy *phy,
				    unsigned long ref_clk_hz)
{
	dev_dbg(&phy->spi->dev, "%s : ref_clk_hz %lu",
		__func__, ref_clk_hz);

	return ad9361_spi_write(phy->spi, REG_REFERENCE_CLOCK_CYCLES,
		REFERENCE_CLOCK_CYCLES_PER_US((ref_clk_hz / 1000000UL) - 1));
}

static int ad9361_set_dcxo_tune(struct ad9361_rf_phy *phy,
				    u32 coarse, u32 fine)
{
	dev_dbg(&phy->spi->dev, "%s : coarse %u fine %u",
		__func__, coarse, fine);

	ad9361_spi_write(phy->spi, REG_DCXO_COARSE_TUNE,
			DCXO_TUNE_COARSE(coarse));
	ad9361_spi_write(phy->spi, REG_DCXO_FINE_TUNE_LOW,
			DCXO_TUNE_FINE_LOW(fine));
	return ad9361_spi_write(phy->spi, REG_DCXO_FINE_TUNE_HIGH,
			DCXO_TUNE_FINE_HIGH(fine));
}

/* val
 * 0	(RX1A_N &  RX1A_P) and (RX2A_N & RX2A_P) enabled; balanced
 * 1	(RX1B_N &  RX1B_P) and (RX2B_N & RX2B_P) enabled; balanced
 * 2	(RX1C_N &  RX1C_P) and (RX2C_N & RX2C_P) enabled; balanced
 *
 * 3	RX1A_N and RX2A_N enabled; unbalanced
 * 4	RX1A_P and RX2A_P enabled; unbalanced
 * 5	RX1B_N and RX2B_N enabled; unbalanced
 * 6	RX1B_P and RX2B_P enabled; unbalanced
 * 7	RX1C_N and RX2C_N enabled; unbalanced
 * 8	RX1C_P and RX2C_P enabled; unbalanced
 */

static int ad9361_rf_port_setup(struct ad9361_rf_phy *phy,
				    u32 rx_inputs, u32 txb)
{
	u32 val;

	if (rx_inputs > 8)
		return -EINVAL;

	if (rx_inputs < 3)
		val = 3 <<  (rx_inputs * 2);
	else
		val = 1 <<  (rx_inputs - 3);

	if (txb)
		val |= TX_OUTPUT; /* Select TX1B, TX2B */

	dev_dbg(&phy->spi->dev, "%s : INPUT_SELECT 0x%X",
		__func__, val);

	return ad9361_spi_write(phy->spi, REG_INPUT_SELECT, val);
}

/*
 * Setup the Parallel Port (Digital Data Interface)
 */
static int ad9361_pp_port_setup(struct ad9361_rf_phy *phy, bool restore_c3)
{
	struct spi_device *spi = phy->spi;
	struct ad9361_phy_platform_data *pd = phy->pdata;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	if (!pd->fdd) /* REVISIT */
		pd->port_ctrl.pp_conf[2] |= HALF_DUPLEX_MODE;

	if (restore_c3) {
		return ad9361_spi_write(spi, REG_PARALLEL_PORT_CONF_3,
					pd->port_ctrl.pp_conf[2]);
	}

	ad9361_spi_write(spi, REG_PARALLEL_PORT_CONF_1, pd->port_ctrl.pp_conf[0]);
	ad9361_spi_write(spi, REG_PARALLEL_PORT_CONF_2, pd->port_ctrl.pp_conf[1]);
	ad9361_spi_write(spi, REG_PARALLEL_PORT_CONF_3, pd->port_ctrl.pp_conf[2]);
	ad9361_spi_write(spi, REG_RX_CLOCK_DATA_DELAY, pd->port_ctrl.rx_clk_data_delay);
	ad9361_spi_write(spi, REG_TX_CLOCK_DATA_DELAY, pd->port_ctrl.tx_clk_data_delay);

	ad9361_spi_write(spi, REG_LVDS_BIAS_CTRL, pd->port_ctrl.lvds_bias_ctrl);
//	ad9361_spi_write(spi, REG_DIGITAL_IO_CTRL, pd->port_ctrl.digital_io_ctrl);
	ad9361_spi_write(spi, REG_LVDS_INVERT_CTRL1, pd->port_ctrl.lvds_invert[0]);
	ad9361_spi_write(spi, REG_LVDS_INVERT_CTRL2, pd->port_ctrl.lvds_invert[1]);

	return 0;
}

static int ad9361_gc_setup(struct ad9361_rf_phy *phy, struct gain_control *ctrl)
{
	struct spi_device *spi = phy->spi;
	u32 reg, tmp1, tmp2;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	reg = DEC_PWR_FOR_GAIN_LOCK_EXIT | DEC_PWR_FOR_LOCK_LEVEL |
		DEC_PWR_FOR_LOW_PWR;

	if (ctrl->rx1_mode == RF_GAIN_HYBRID_AGC ||
		ctrl->rx2_mode == RF_GAIN_HYBRID_AGC)
		reg |= SLOW_ATTACK_HYBRID_MODE;

	reg |= RX1_GAIN_CTRL_SETUP(ctrl->rx1_mode) |
		RX2_GAIN_CTRL_SETUP(ctrl->rx2_mode);

	phy->agc_mode[0] = ctrl->rx1_mode;
	phy->agc_mode[1] = ctrl->rx2_mode;

	ad9361_spi_write(spi, REG_AGC_CONFIG_1, reg); // Gain Control Mode Select

	/* AGC_USE_FULL_GAIN_TABLE handled in ad9361_load_gt() */
	ad9361_spi_writef(spi, REG_AGC_CONFIG_2, MAN_GAIN_CTRL_RX1,
			  ctrl->mgc_rx1_ctrl_inp_en);
	ad9361_spi_writef(spi, REG_AGC_CONFIG_2, MAN_GAIN_CTRL_RX2,
			  ctrl->mgc_rx2_ctrl_inp_en);
	ad9361_spi_writef(spi, REG_AGC_CONFIG_2, DIG_GAIN_EN,
			  ctrl->dig_gain_en);

	ctrl->adc_ovr_sample_size = clamp_t(u8, ctrl->adc_ovr_sample_size, 1U, 8U);
	reg = ADC_OVERRANGE_SAMPLE_SIZE(ctrl->adc_ovr_sample_size - 1);

	if (phy->pdata->split_gt &&
		(ctrl->mgc_rx1_ctrl_inp_en || ctrl->mgc_rx2_ctrl_inp_en)) {
		switch (ctrl->mgc_split_table_ctrl_inp_gain_mode) {
		case 1:
			reg &= ~INCDEC_LMT_GAIN;
			break;
		case 2:
			reg |= INCDEC_LMT_GAIN;
			break;
		default:
		case 0:
			reg |= USE_AGC_FOR_LMTLPF_GAIN;
			break;
		}
	}

	ctrl->mgc_inc_gain_step = clamp_t(u8, ctrl->mgc_inc_gain_step, 1U, 8U);
	reg |= MANUAL_INCR_STEP_SIZE(ctrl->mgc_inc_gain_step - 1);
	ad9361_spi_write(spi, REG_AGC_CONFIG_3, reg); // Incr Step Size, ADC Overrange Size

	if (phy->pdata->split_gt) {
		reg = SIZE_SPLIT_TABLE - 1;
	} else {
		reg = SIZE_FULL_TABLE - 1;
	}
	ad9361_spi_write(spi, REG_MAX_LMT_FULL_GAIN, reg); // Max Full/LMT Gain Table Index
	ad9361_spi_write(spi, REG_RX1_MANUAL_LMT_FULL_GAIN, reg); // Rx1 Full/LMT Gain Index
	ad9361_spi_write(spi, REG_RX2_MANUAL_LMT_FULL_GAIN, reg); // Rx2 Full/LMT Gain Index

	ctrl->analog_settling_time = clamp_t(u8, ctrl->analog_settling_time, 0U, 31U);
	ctrl->mgc_dec_gain_step = clamp_t(u8, ctrl->mgc_dec_gain_step, 1U, 8U);
	reg = PEAK_OVERLOAD_WAIT_TIME(ctrl->analog_settling_time);
	reg |= MANUAL_CTRL_IN_DECR_GAIN_STP_SIZE(ctrl->mgc_dec_gain_step);
	ad9361_spi_write(spi, REG_PEAK_WAIT_TIME, reg); // Decr Step Size, Peak Overload Time

	if (ctrl->dig_gain_en)
		ad9361_spi_write(spi, REG_DIGITAL_GAIN,
				MAXIMUM_DIGITAL_GAIN(ctrl->max_dig_gain) |
				DIG_GAIN_STP_SIZE(ctrl->dig_gain_step_size));

	if (ctrl->adc_large_overload_thresh >= ctrl->adc_small_overload_thresh) {
		ad9361_spi_write(spi, REG_ADC_SMALL_OVERLOAD_THRESH,
				 ctrl->adc_small_overload_thresh); // ADC Small Overload Threshold
		ad9361_spi_write(spi, REG_ADC_LARGE_OVERLOAD_THRESH,
				 ctrl->adc_large_overload_thresh); // ADC Large Overload Threshold
	} else {
		ad9361_spi_write(spi, REG_ADC_SMALL_OVERLOAD_THRESH,
				 ctrl->adc_large_overload_thresh); // ADC Small Overload Threshold
		ad9361_spi_write(spi, REG_ADC_LARGE_OVERLOAD_THRESH,
				 ctrl->adc_small_overload_thresh); // ADC Large Overload Threshold
	}

	reg = (ctrl->lmt_overload_high_thresh / 16) - 1;
	reg = clamp(reg, 0U, 63U);
	ad9361_spi_write(spi, REG_LARGE_LMT_OVERLOAD_THRESH, reg);
	reg = (ctrl->lmt_overload_low_thresh / 16) - 1;
	reg = clamp(reg, 0U, 63U);
	ad9361_spi_writef(spi, REG_SMALL_LMT_OVERLOAD_THRESH,
			  SMALL_LMT_OVERLOAD_THRESH(~0), reg);

	if (phy->pdata->split_gt) {
		/* REVIST */
		ad9361_spi_write(spi, REG_RX1_MANUAL_LPF_GAIN, 0x58); // Rx1 LPF Gain Index
		ad9361_spi_write(spi, REG_RX2_MANUAL_LPF_GAIN, 0x18); // Rx2 LPF Gain Index
		ad9361_spi_write(spi, REG_FAST_INITIAL_LMT_GAIN_LIMIT, 0x27); // Initial LMT Gain Limit
	}

	ad9361_spi_write(spi, REG_RX1_MANUAL_DIGITALFORCED_GAIN, 0x00); // Rx1 Digital Gain Index
	ad9361_spi_write(spi, REG_RX2_MANUAL_DIGITALFORCED_GAIN, 0x00); // Rx2 Digital Gain Index

	reg = clamp_t(u8, ctrl->low_power_thresh, 0U, 64U) * 2;
	ad9361_spi_write(spi, REG_FAST_LOW_POWER_THRESH, reg); // Low Power Threshold
	ad9361_spi_write(spi, REG_TX_SYMBOL_ATTEN_CONFIG, 0x00); // Tx Symbol Gain Control

	ad9361_spi_writef(spi, REG_DEC_POWER_MEASURE_DURATION_0,
			  USE_HB1_OUT_FOR_DEC_PWR_MEAS, 1); // Power Measurement Duration

	ad9361_spi_writef(spi, REG_DEC_POWER_MEASURE_DURATION_0,
			  ENABLE_DEC_PWR_MEAS, 1); // Power Measurement Duration

	reg = ilog2(ctrl->dec_pow_measuremnt_duration / 16);
	ad9361_spi_writef(spi, REG_DEC_POWER_MEASURE_DURATION_0,
			  DEC_POWER_MEASUREMENT_DURATION(~0), reg); // Power Measurement Duration

	/* AGC */

	reg = clamp_t(u8, ctrl->agc_attack_delay_us, 0U, 31U);
	ad9361_spi_writef(spi, REG_AGC_ATTACK_DELAY,
			  AGC_ATTACK_DELAY(~0), reg);

	/* For Fast AGC upper bits may be used */
	reg = clamp_t(u8, ctrl->agc_settling_delay, 0U, 31U);
	ad9361_spi_write(spi, REG_FAST_CONFIG_2_SETTLING_DELAY, reg);

	tmp1 = reg = clamp_t(u8, ctrl->agc_inner_thresh_high, 0U, 127U);
	ad9361_spi_writef(spi, REG_AGC_LOCK_LEVEL,
			  AGC_LOCK_LEVEL_FAST_AGC_INNER_HIGH_THRESH_SLOW(~0),
			  reg);

	tmp2 = reg = clamp_t(u8, ctrl->agc_inner_thresh_low, 0U, 127U);
	reg |= (ctrl->adc_lmt_small_overload_prevent_gain_inc ?
		PREVENT_GAIN_INC : 0);
	ad9361_spi_write(spi, REG_AGC_INNER_LOW_THRESH, reg);

	reg = AGC_OUTER_HIGH_THRESH(tmp1 - ctrl->agc_outer_thresh_high) |
		AGC_OUTER_LOW_THRESH(ctrl->agc_outer_thresh_low - tmp2);
	ad9361_spi_write(spi, REG_OUTER_POWER_THRESHS, reg);

	reg = AGC_OUTER_HIGH_THRESH_EXED_STP_SIZE(ctrl->agc_outer_thresh_high_dec_steps) |
		AGC_OUTER_LOW_THRESH_EXED_STP_SIZE(ctrl->agc_outer_thresh_low_inc_steps);
	ad9361_spi_write(spi, REG_GAIN_STP_2, reg);

	reg = ((ctrl->immed_gain_change_if_large_adc_overload) ?
		IMMED_GAIN_CHANGE_IF_LG_ADC_OVERLOAD : 0) |
		((ctrl->immed_gain_change_if_large_lmt_overload) ?
		IMMED_GAIN_CHANGE_IF_LG_LMT_OVERLOAD : 0) |
		AGC_INNER_HIGH_THRESH_EXED_STP_SIZE(ctrl->agc_inner_thresh_high_dec_steps) |
		AGC_INNER_LOW_THRESH_EXED_STP_SIZE(ctrl->agc_inner_thresh_low_inc_steps);
	ad9361_spi_write(spi, REG_GAIN_STP1, reg);

	reg = LARGE_ADC_OVERLOAD_EXED_COUNTER(ctrl->adc_large_overload_exceed_counter) |
		SMALL_ADC_OVERLOAD_EXED_COUNTER(ctrl->adc_small_overload_exceed_counter);
	ad9361_spi_write(spi, REG_ADC_OVERLOAD_COUNTERS, reg);

	ad9361_spi_writef(spi, REG_GAIN_STP_CONFIG_2, LARGE_LPF_GAIN_STEP(~0),
			 LARGE_LPF_GAIN_STEP(ctrl->adc_large_overload_inc_steps));

	reg = LARGE_LMT_OVERLOAD_EXED_COUNTER(ctrl->lmt_overload_large_exceed_counter) |
		SMALL_LMT_OVERLOAD_EXED_COUNTER(ctrl->lmt_overload_small_exceed_counter);
	ad9361_spi_write(spi, REG_LMT_OVERLOAD_COUNTERS, reg);

	ad9361_spi_writef(spi, REG_GAIN_STP_CONFIG1,
			DEC_STP_SIZE_FOR_LARGE_LMT_OVERLOAD(~0),
			ctrl->lmt_overload_large_inc_steps);

	reg = DIG_SATURATION_EXED_COUNTER(ctrl->dig_saturation_exceed_counter) |
		(ctrl->sync_for_gain_counter_en ? ENABLE_SYNC_FOR_GAIN_COUNTER : 0) |
		(ctrl->gain_update_counter > 131070 ? DOUBLE_GAIN_COUNTER : 0);
	ad9361_spi_write(spi, REG_DIGITAL_SAT_COUNTER, reg);

	tmp1 = (ctrl->gain_update_counter > 131070) ?
		ctrl->gain_update_counter / 4 :
		ctrl->gain_update_counter / 2;

	ad9361_spi_write(spi, REG_GAIN_UPDATE_COUNTER1, tmp1 & 0xFF);
	ad9361_spi_write(spi, REG_GAIN_UPDATE_COUNTER2, tmp1 >> 8);

	return 0;
}

  //************************************************************
  // Setup AuxDAC
  //************************************************************
static int ad9361_auxdac_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x018, 0x00); // AuxDAC1 Word[9:2]
	ad9361_spi_write(spi, 0x019, 0x00); // AuxDAC2 Word[9:2]
	ad9361_spi_write(spi, 0x01a, 0x00); // AuxDAC1 Config and Word[1:0]
	ad9361_spi_write(spi, 0x01b, 0x00); // AuxDAC2 Config and Word[1:0]
	ad9361_spi_write(spi, 0x023, 0xff); // AuxDAC Manaul/Auto Control
	/* FIXME eLNA control */
	ad9361_spi_write(spi, 0x026, 0x00); // AuxDAC Manual Select Bit/GPO Manual Select
	ad9361_spi_write(spi, 0x030, 0x00); // AuxDAC1 Rx Delay
	ad9361_spi_write(spi, 0x031, 0x00); // AuxDAC1 Tx Delay
	ad9361_spi_write(spi, 0x032, 0x00); // AuxDAC2 Rx Delay
	ad9361_spi_write(spi, 0x033, 0x00); // AuxDAC2 Tx Delay

	return 0;
}
  //************************************************************
  // Setup AuxADC
  //************************************************************

static int ad9361_auxadc_setup(struct ad9361_rf_phy *phy,
			       struct auxadc_control *ctrl,
			       unsigned long bbpll_freq)
{
	struct spi_device *spi = phy->spi;
	u32 val;

	/* FIXME this function needs to be called whenever BBPLL changes */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	val = DIV_ROUND_CLOSEST(ctrl->temp_time_inteval_ms *
		(bbpll_freq / 1000UL), (1 << 29));

	ad9361_spi_write(spi, REG_TEMP_OFFSET, ctrl->offset);
	ad9361_spi_write(spi, REG_START_TEMP_READING, 0x00);
	ad9361_spi_write(spi, REG_TEMP_SENSE2,
			 MEASUREMENT_TIME_INTERVAL(val) |
			 (ctrl->periodic_temp_measuremnt ?
			 TEMP_SENSE_PERIODIC_ENABLE : 0));
	ad9361_spi_write(spi, REG_TEMP_SENSOR_CONFIG,
			 TEMP_SENSOR_DECIMATION(
			 ilog2(ctrl->temp_sensor_decimation) - 8));
	ad9361_spi_write(spi, REG_AUXADC_CLOCK_DIVIDER,
			 bbpll_freq / ctrl->auxadc_clock_rate);
	ad9361_spi_write(spi, REG_AUXADC_CONFIG,
			 AUX_ADC_DECIMATION(
			ilog2(ctrl->auxadc_decimation) - 8));

	return 0;
}

static int ad9361_get_temp(struct ad9361_rf_phy *phy)
{
	u32 val = ad9361_spi_read(phy->spi, REG_TEMPERATURE);

	return DIV_ROUND_CLOSEST(val * 1000, 1140);
}

  //************************************************************
  // Setup Control Outs
  //************************************************************

static int ad9361_ctrl_outs_setup(struct ad9361_rf_phy *phy,
				  struct ctrl_outs_control *ctrl)
{
	struct spi_device *spi = phy->spi;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, REG_CTRL_OUTPUT_POINTER, ctrl->index); // Ctrl Out index
	return ad9361_spi_write(spi, REG_CTRL_OUTPUT_ENABLE, ctrl->en_mask); // Ctrl Out [7:0] output enable
}
  //************************************************************
  // Setup GPO
  //************************************************************

static int ad9361_gpo_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x020, 0x00); // GPO Auto Enable Setup in RX and TX
	ad9361_spi_write(spi, 0x027, 0x03); // GPO Manual and GPO auto value in ALERT
	ad9361_spi_write(spi, 0x028, 0x00); // GPO_0 RX Delay
	ad9361_spi_write(spi, 0x029, 0x00); // GPO_1 RX Delay
	ad9361_spi_write(spi, 0x02a, 0x00); // GPO_2 RX Delay
	ad9361_spi_write(spi, 0x02b, 0x00); // GPO_3 RX Delay
	ad9361_spi_write(spi, 0x02c, 0x00); // GPO_0 TX Delay
	ad9361_spi_write(spi, 0x02d, 0x00); // GPO_1 TX Delay
	ad9361_spi_write(spi, 0x02e, 0x00); // GPO_2 TX Delay
	ad9361_spi_write(spi, 0x02f, 0x00); // GPO_3 TX Delay

	return 0;
}

static int ad9361_rssi_setup(struct ad9361_rf_phy *phy,
			     struct rssi_control *ctrl,
			     bool is_update)
{
	struct spi_device *spi = phy->spi;
	u32 total_weight, weight[4], total_dur = 0, temp;
	u8 dur_buf[4] = {0};
	int val, ret, i, j = 0;
	u32 rssi_delay;
	u32 rssi_wait;
	u32 rssi_duration;
	unsigned long rate;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	if (ctrl->rssi_unit_is_rx_samples) {
		if (is_update)
			return 0; /* no update required */

		rssi_delay = ctrl->rssi_delay;
		rssi_wait = ctrl->rssi_wait;
		rssi_duration = ctrl->rssi_duration;
	} else {
		/* update sample based on RX rate */
		rate = DIV_ROUND_CLOSEST(
			clk_get_rate(phy->clks[RX_SAMPL_CLK]), 1000);
		/* units are in us */
		rssi_delay = DIV_ROUND_CLOSEST(ctrl->rssi_delay * rate, 1000);
		rssi_wait = DIV_ROUND_CLOSEST(ctrl->rssi_wait * rate, 1000);
		rssi_duration = DIV_ROUND_CLOSEST(
			ctrl->rssi_duration * rate, 1000);
	}

	if (ctrl->restart_mode == EN_AGC_PIN_IS_PULLED_HIGH)
		rssi_delay = 0;

	rssi_delay = clamp(rssi_delay / 8, 0U, 255U);
	rssi_wait = clamp(rssi_wait / 4, 0U, 255U);

	do {
		for (i = 14; rssi_duration > 0 && i >= 0 ; i--) {
			val = 1 << i;
			if (rssi_duration >= val) {
				dur_buf[j++] = i;
				total_dur += val;
				rssi_duration -= val;
				break;
			}
		}

	} while (j < 4 && rssi_duration > 0);

	for (i = 0, total_weight = 0; i < 4; i++)
		total_weight += weight[i] =
			DIV_ROUND_CLOSEST(RSSI_MAX_WEIGHT *
				(1 << dur_buf[i]), total_dur);

	/* total of all weights must be 0xFF */
	val = total_weight - 0xFF;
	weight[j - 1] -= val;

	ad9361_spi_write(spi, REG_MEASURE_DURATION_01,
			 (dur_buf[1] << 4) | dur_buf[0]); // RSSI Measurement Duration 0, 1
	ad9361_spi_write(spi, REG_MEASURE_DURATION_23,
			 (dur_buf[3] << 4) | dur_buf[2]); // RSSI Measurement Duration 2, 3
	ad9361_spi_write(spi, REG_RSSI_WEIGHT_0, weight[0]); // RSSI Weighted Multiplier 0
	ad9361_spi_write(spi, REG_RSSI_WEIGHT_1, weight[1]); // RSSI Weighted Multiplier 1
	ad9361_spi_write(spi, REG_RSSI_WEIGHT_2, weight[2]); // RSSI Weighted Multiplier 2
	ad9361_spi_write(spi, REG_RSSI_WEIGHT_3, weight[3]); // RSSI Weighted Multiplier 3
	ad9361_spi_write(spi, REG_RSSI_DELAY, rssi_delay); // RSSI Delay
	ad9361_spi_write(spi, REG_RSSI_WAIT_TIME, rssi_wait); // RSSI Wait

	temp = RSSI_MODE_SELECT(ctrl->restart_mode);
	if (ctrl->restart_mode == SPI_WRITE_TO_REGISTER)
		temp |= START_RSSI_MEAS;

	ret = ad9361_spi_write(spi, REG_RSSI_CONFIG, temp); // RSSI Mode Select

	if (ret < 0)
		dev_err(&phy->spi->dev, "Unable to write rssi config\n");

	return 0;
}

static int ad9361_ensm_set_state(struct ad9361_rf_phy *phy, u8 ensm_state)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc = 0;
	u32 val;

	if (phy->curr_ensm_state == ensm_state) {
		dev_dbg(dev, "Nothing to do, device is already in %d state\n",
			ensm_state);
		goto out;
	}

	dev_dbg(dev, "Device is in %x state, moving to %x\n", phy->curr_ensm_state,
			ensm_state);

	val = phy->ensm_conf1;

	switch (ensm_state) {
	case ENSM_STATE_TX:
		val |= FORCE_TX_ON;
		if (phy->pdata->fdd)
			rc = -EINVAL;
		else if (phy->curr_ensm_state != ENSM_STATE_ALERT)
			rc = -EINVAL;
		break;
	case ENSM_STATE_RX:
		val |= FORCE_RX_ON;
		if (phy->pdata->fdd)
			rc = -EINVAL;
		else if (phy->curr_ensm_state != ENSM_STATE_ALERT)
			rc = -EINVAL;
		break;
	case ENSM_STATE_FDD:
		val |= (FORCE_TX_ON | FORCE_RX_ON);
		if (!phy->pdata->fdd)
			rc = -EINVAL;
		break;
	case ENSM_STATE_ALERT:
		val &= ~(FORCE_TX_ON | FORCE_RX_ON);
		val |= TO_ALERT | FORCE_ALERT_STATE;
		break;
	case ENSM_STATE_SLEEP_WAIT:
		break;
	default:
		dev_err(dev, "No handling for forcing %d ensm state\n",
		ensm_state);
		goto out;
	}

	if (rc) {
		dev_err(dev, "Invalid ENSM state transition in %s mode\n",
			phy->pdata->fdd ? "FDD" : "TDD");
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONFIG_1, val);
	if (rc)
		dev_err(dev, "Failed to restore state\n");

	phy->curr_ensm_state = ensm_state;

out:
	return rc;

}

static int ad9361_set_trx_clock_chain(struct ad9361_rf_phy *phy,
				      unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	struct device *dev = &phy->spi->dev;
	int ret, i, j, n;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	if (!rx_path_clks || !tx_path_clks)
		return -EINVAL;

	ret = clk_set_rate(phy->clks[BBPLL_CLK], rx_path_clks[BBPLL_FREQ]);
	if (ret < 0)
		return ret;

	for (i = ADC_CLK, j = DAC_CLK, n = ADC_FREQ;
		i <= RX_SAMPL_CLK; i++, j++, n++) {
		ret = clk_set_rate(phy->clks[i], rx_path_clks[n]);
		if (ret < 0) {
			dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
				ret);
			return ret;
		}
		ret = clk_set_rate(phy->clks[j], tx_path_clks[n]);
		if (ret < 0) {
			dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
				ret);
			return ret;
		}
	}
	return 0;
}

static int ad9361_get_trx_clock_chain(struct ad9361_rf_phy *phy, unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	int i, j, n;
	unsigned long bbpll_freq;

	if (!rx_path_clks && !tx_path_clks)
		return -EINVAL;

	bbpll_freq = clk_get_rate(phy->clks[BBPLL_CLK]);

	if (rx_path_clks)
		rx_path_clks[BBPLL_FREQ] = bbpll_freq;

	if (tx_path_clks)
		tx_path_clks[BBPLL_FREQ] = bbpll_freq;

	for (i = ADC_CLK, j = DAC_CLK, n = ADC_FREQ;
		i <= RX_SAMPL_CLK; i++, j++, n++) {
		if (rx_path_clks)
			rx_path_clks[n] = clk_get_rate(phy->clks[i]);
		if (tx_path_clks)
			tx_path_clks[n] = clk_get_rate(phy->clks[j]);
	}

	return 0;
}

static int ad9361_calculate_rf_clock_chain(struct ad9361_rf_phy *phy,
				      unsigned long tx_sample_rate,
				      u32 low_power,
				      unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	unsigned long clktf, clkrf, adc_rate = 0, dac_rate = 0, tmp;
	u64 bbpll_rate;
	int i, index_rx = -1, index_tx = -1;
	u32 div, tx_intdec, rx_intdec;
	const char clk_dividers[][4] = {
		{12,3,2,2},
		{8,2,2,2},
		{6,3,1,2},
		{4,2,2,1},
		{3,3,1,1},
		{2,2,1,1},
		{1,1,1,1},
	};


	if (phy->bypass_rx_fir)
		rx_intdec = 1;
	else
		rx_intdec = phy->rx_fir_dec;

	if (phy->bypass_tx_fir)
		tx_intdec = 1;
	else
		tx_intdec = phy->tx_fir_int;


	dev_dbg(&phy->spi->dev, "%s: requested rate %lu TXFIR int %d RXFIR dec %d mode %s",
		__func__, tx_sample_rate, tx_intdec, rx_intdec,
		low_power ? "Medium PWR" : "Highest OSR");

	if (tx_sample_rate > (phy->pdata->rx2tx2 ? 61440000UL : 122880000UL))
		return -EINVAL;

	clktf = tx_sample_rate * tx_intdec;
	clkrf = tx_sample_rate * rx_intdec * (phy->rx_eq_2tx ? 2 : 1);

	for (i = low_power; i < 7; i++) {
		adc_rate = clkrf * clk_dividers[i][0];
		dac_rate = clktf * clk_dividers[i][0];
		if ((adc_rate <= MAX_ADC_CLK) && (adc_rate >= MIN_ADC_CLK)) {
			tmp = adc_rate / dac_rate;
			if (adc_rate <= MAX_DAC_CLK) {
				index_rx = i;
				index_tx = i - ((tmp == 1) ? 0 : tmp);
				dac_rate = adc_rate; /* ADC_CLK */
				break;
			} else {
				dac_rate = adc_rate / 2;  /* ADC_CLK/2 */
				index_tx = i + 2 - ((tmp == 1) ? 0 : tmp);
				index_rx = i;
				break;
			}
		}
	}

	if ((index_tx < 0 || index_tx > 6 || index_rx < 0 || index_rx > 6) && low_power < 7) {
		return ad9361_calculate_rf_clock_chain(phy, tx_sample_rate,
			++low_power, rx_path_clks, tx_path_clks);
	} else if ((index_tx < 0 || index_tx > 6 || index_rx < 0 || index_rx > 6)) {
		dev_err(&phy->spi->dev, "%s: Failed to find suitable dividers: %s",
		__func__, (adc_rate < MIN_ADC_CLK) ? "ADC clock below limit" : "BBPLL rate above limit");

		return -EINVAL;
	}

	/* Calculate target BBPLL rate */
	div = MAX_BBPLL_DIV;

	do {
		bbpll_rate = (u64)adc_rate * div;
		div >>= 1;

	} while ((bbpll_rate > MAX_BBPLL_FREQ) && (div >= MIN_BBPLL_DIV));

	rx_path_clks[BBPLL_FREQ] = bbpll_rate;
	rx_path_clks[ADC_FREQ] = adc_rate;
	rx_path_clks[R2_FREQ] = rx_path_clks[ADC_FREQ] / clk_dividers[index_rx][1];
	rx_path_clks[R1_FREQ] = rx_path_clks[R2_FREQ] / clk_dividers[index_rx][2];
	rx_path_clks[CLKRF_FREQ] = rx_path_clks[R1_FREQ] / clk_dividers[index_rx][3];
	rx_path_clks[RX_SAMPL_FREQ] = rx_path_clks[CLKRF_FREQ] / 	rx_intdec;

	tx_path_clks[BBPLL_FREQ] = bbpll_rate;
	tx_path_clks[DAC_FREQ] = dac_rate;
	tx_path_clks[T2_FREQ] = tx_path_clks[DAC_FREQ] / clk_dividers[index_tx][1];
	tx_path_clks[T1_FREQ] =tx_path_clks[T2_FREQ] / clk_dividers[index_tx][2];
	tx_path_clks[CLKTF_FREQ] = tx_path_clks[T1_FREQ] / clk_dividers[index_tx][3];
	tx_path_clks[TX_SAMPL_FREQ] = tx_path_clks[CLKTF_FREQ] / 	tx_intdec;

	dev_dbg(&phy->spi->dev, "%s: %lu %lu %lu %lu %lu %lu",
		__func__, rx_path_clks[BBPLL_FREQ], rx_path_clks[ADC_FREQ],
		rx_path_clks[R2_FREQ], rx_path_clks[R1_FREQ],
		rx_path_clks[CLKRF_FREQ], rx_path_clks[RX_SAMPL_FREQ]);

	dev_dbg(&phy->spi->dev, "%s: %lu %lu %lu %lu %lu %lu",
		__func__, tx_path_clks[BBPLL_FREQ], tx_path_clks[ADC_FREQ],
		tx_path_clks[R2_FREQ], tx_path_clks[R1_FREQ],
		tx_path_clks[CLKRF_FREQ], tx_path_clks[RX_SAMPL_FREQ]);

	return 0;
}

static int ad9361_setup(struct ad9361_rf_phy *phy)
{
	unsigned long refin_Hz, ref_freq, bbpll_freq;
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	struct ad9361_phy_platform_data *pd = phy->pdata;
	int ret;
	u32 real_rx_bandwidth = pd->rf_rx_bandwidth_Hz / 2;
	u32 real_tx_bandwidth = pd->rf_tx_bandwidth_Hz / 2;

	dev_dbg(dev, "%s", __func__);

	if (pd->port_ctrl.pp_conf[2] & FDD_RX_RATE_2TX_RATE)
		phy->rx_eq_2tx = true;

	phy->ensm_conf1 =
		(pd->ensm_pin_level_mode ? LEVEL_MODE : 0) |
		(pd->ensm_pin_ctrl ? ENABLE_ENSM_PIN_CTRL : 0) |
		TO_ALERT;

	ad9361_spi_write(spi, REG_SPI_CONF, SOFT_RESET | _SOFT_RESET); /* RESET */
	ad9361_spi_write(spi, REG_SPI_CONF, 0x0);

	ad9361_spi_write(spi, REG_CTRL, CTRL_ENABLE);
	ad9361_spi_write(spi, REG_BANDGAP_CONFIG0, MASTER_BIAS_TRIM(0x0E)); /* Enable Master Bias */
	ad9361_spi_write(spi, REG_BANDGAP_CONFIG1, BANDGAP_TEMP_TRIM(0x0E)); /* Set Bandgap Trim */

	ad9361_set_dcxo_tune(phy, pd->dcxo_coarse, pd->dcxo_fine);

	refin_Hz = clk_get_rate(phy->clk_refin);

	if (refin_Hz < 40000000UL)
		ref_freq = 2 * refin_Hz;
	else if (refin_Hz < 80000000UL)
		ref_freq = refin_Hz;
	else if (refin_Hz < 160000000UL)
		ref_freq = refin_Hz / 2;
	else if (refin_Hz < 320000000UL)
		ref_freq = refin_Hz / 4;
	else
		return -EINVAL;

	ret = clk_set_rate(phy->clks[RX_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set RX Synth ref clock rate (%d)\n", ret);
		return ret;
	}

	ret = clk_set_rate(phy->clks[TX_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set TX Synth ref clock rate (%d)\n", ret);
		return ret;
	}

 	ret = clk_prepare_enable(phy->clks[RX_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable RX Synth ref clock (%d)\n", ret);
		return ret;
	}

 	ret = clk_prepare_enable(phy->clks[TX_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable TX Synth ref clock (%d)\n", ret);
		return ret;
	}

	ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_1, RX_REF_RESET_BAR, 1);
	ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_2,
			  TX_REF_DOUBLER_FB_DELAY(~0), 3); /* FB DELAY */
	ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_2,
			  RX_REF_DOUBLER_FB_DELAY(~0), 3); /* FB DELAY */

	ad9361_spi_write(spi, REG_CLOCK_ENABLE,
			DIGITAL_POWER_UP | CLOCK_ENABLE_DFLT | BBPLL_ENABLE |
			(pd->use_extclk ? XO_BYPASS : 0)); /* Enable Clocks */

	ret = clk_set_rate(phy->clks[BB_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[BB_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable BB ref clock rate (%d)\n",
			ret);
		return ret;
	}

	ret = ad9361_set_trx_clock_chain(phy, pd->rx_path_clks,
				   pd->tx_path_clks);
	if (ret < 0)
		return ret;

	ad9361_en_dis_tx(phy, 1, TX_ENABLE);
	ad9361_en_dis_rx(phy, 1, RX_ENABLE);

	if (pd->rx2tx2) {
		ad9361_en_dis_tx(phy, 2, TX_ENABLE);
		ad9361_en_dis_rx(phy, 2, RX_ENABLE);
	}

	ret = ad9361_rf_port_setup(phy, pd->rf_rx_input_sel,
				   pd->rf_tx_output_sel);
	if (ret < 0)
		return ret;

	ret = ad9361_pp_port_setup(phy, false);
	if (ret < 0)
		return ret;

	ret = ad9361_auxdac_setup(phy);
	if (ret < 0)
		return ret;

	bbpll_freq = clk_get_rate(phy->clks[BBPLL_CLK]);

	ret = ad9361_auxadc_setup(phy, &pd->auxadc_ctrl, bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_ctrl_outs_setup(phy, &pd->ctrl_outs_ctrl);
	if (ret < 0)
		return ret;

	ret = ad9361_gpo_setup(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_set_ref_clk_cycles(phy, refin_Hz);
	if (ret < 0)
		return ret;

	ret = ad9361_setup_ext_lna(phy, pd->elna_ctrl.gain_mdB,
			pd->elna_ctrl.bypass_loss_mdB);
	if (ret < 0)
		return ret;

	ret = ad9361_txrx_synth_cp_calib(phy, ref_freq, false); /* RXCP */
	if (ret < 0)
		return ret;

	ret = ad9361_txrx_synth_cp_calib(phy, ref_freq, true); /* TXCP */
	if (ret < 0)
		return ret;

	ret = clk_set_rate(phy->clks[RX_RFPLL], ad9361_to_clk(pd->rx_synth_freq));
	if (ret < 0) {
		dev_err(dev, "Failed to set RX Synth rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[RX_RFPLL]);
	if (ret < 0)
		return ret;

	ret = clk_set_rate(phy->clks[TX_RFPLL], ad9361_to_clk(pd->tx_synth_freq));
	if (ret < 0) {
		dev_err(dev, "Failed to set TX Synth rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[TX_RFPLL]);
	if (ret < 0)
		return ret;

	ret = ad9361_load_mixer_gm_subtable(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_gc_setup(phy, &pd->gain_ctrl);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_bb_analog_filter_calib(phy,
				real_rx_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_analog_filter_calib(phy,
				real_tx_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_tia_calib(phy, real_rx_bandwidth);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_second_filter_calib(phy, pd->rf_tx_bandwidth_Hz);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_adc_setup(phy,
				bbpll_freq,
				clk_get_rate(phy->clks[ADC_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_bb_dc_offset_calib(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_rf_dc_offset_calib(phy,
			ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));
	if (ret < 0)
		return ret;

	ret = ad9361_tx_quad_calib(phy, real_tx_bandwidth);
	if (ret < 0)
		return ret;

	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
			phy->rfdc_track_en, phy->quad_track_en);
	if (ret < 0)
		return ret;

	if (!pd->fdd)
		ad9361_run_calibration(phy, TXMON_CAL);

	ad9361_pp_port_setup(phy, true);

	ad9361_spi_write(phy->spi, REG_ENSM_MODE, pd->fdd ? FDD_MODE : 0x00);

	if (pd->fdd)
		ad9361_spi_write(phy->spi, REG_ENSM_CONFIG_2,
			DUAL_SYNTH_MODE |
			(pd->ensm_pin_ctrl ? FDD_EXTERNAL_CTRL_ENABLE : 0)); /* Dual Synth */
	 else    /* For now in TDD always use Dual Synth */
		ad9361_spi_write(phy->spi, REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE);

	ret = ad9361_set_tx_atten(phy, pd->tx_atten, true, true);
	if (ret < 0)
		return ret;

	ret = ad9361_rssi_setup(phy, &pd->rssi_ctrl, false);
	if (ret < 0)
		return ret;

	phy->curr_ensm_state = ad9361_spi_readf(spi, REG_STATE, ENSM_STATE(~0));
	ad9361_ensm_set_state(phy, pd->fdd ? ENSM_STATE_FDD : ENSM_STATE_RX);

	phy->current_rx_bw_Hz = pd->rf_rx_bandwidth_Hz;
	phy->current_tx_bw_Hz = pd->rf_tx_bandwidth_Hz;
	phy->auto_cal_en = true;
	phy->cal_threshold_freq = 100000000ULL; /* 100 MHz */

	return 0;

}

static int ad9361_do_calib_run(struct ad9361_rf_phy *phy, u32 cal)
{
	int ret;

	ret = ad9361_tracking_control(phy, false, false, false);
	if (ret < 0)
		return ret;

	ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);

	switch (cal) {
	case TX_QUAD_CAL:
		ret = ad9361_tx_quad_calib(phy, phy->current_tx_bw_Hz / 2);
		break;
	case RX_QUAD_CAL:
		ret = ad9361_rx_quad_calib(phy, phy->current_rx_bw_Hz / 2);
		break;
	case RFDC_CAL:
		ret = ad9361_rf_dc_offset_calib(phy,
			ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));
		break;
	default:
		ret = -EINVAL;
		break;
	}

	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
			phy->rfdc_track_en, phy->quad_track_en);
	ad9361_ensm_restore_prev_state(phy);

	return ret;
}

static int ad9361_update_rf_bandwidth(struct ad9361_rf_phy *phy,
				     u32 rf_rx_bw, u32 rf_tx_bw)
{
	unsigned long bbpll_freq;
	u32 real_rx_bandwidth = rf_rx_bw / 2;
	u32 real_tx_bandwidth = rf_tx_bw / 2;
	int ret;

	bbpll_freq = clk_get_rate(phy->clks[BBPLL_CLK]);

	ret = ad9361_tracking_control(phy, false, false, false);
	if (ret < 0)
		return ret;


	ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);

	ret = ad9361_rx_bb_analog_filter_calib(phy,
				real_rx_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_analog_filter_calib(phy,
				real_tx_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_tia_calib(phy, real_rx_bandwidth);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_second_filter_calib(phy, rf_tx_bw);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_adc_setup(phy,
				bbpll_freq,
				clk_get_rate(phy->clks[ADC_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_tx_quad_calib(phy, real_tx_bandwidth);
	if (ret < 0)
		return ret;

	phy->current_rx_bw_Hz = rf_rx_bw;
	phy->current_tx_bw_Hz = rf_tx_bw;

	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
			phy->rfdc_track_en, phy->quad_track_en);
	if (ret < 0)
		return ret;

	ad9361_ensm_restore_prev_state(phy);

	return 0;
}

static int ad9361_load_fir_filter_coef(struct ad9361_rf_phy *phy,
				       enum fir_dest dest, int gain_dB,
				       u32 ntaps, short *coef)
{
	struct spi_device *spi = phy->spi;
	u32 val, offs = 0, fir_conf = 0;

	dev_dbg(&phy->spi->dev, "%s: TAPS %d, gain %d, dest %d",
		__func__, ntaps, gain_dB, dest);

	if (coef == NULL || !ntaps || ntaps > 128 || ntaps % 16) {
		dev_err(&phy->spi->dev,
			"%s: Invalid parameters: TAPS %d, gain %d, dest 0x%X",
			__func__, ntaps, gain_dB, dest);

		return -EINVAL;
	}

	if (dest & FIR_IS_RX) {
		val = 3 - (gain_dB + 12) / 6;
		ad9361_spi_write(spi, REG_RX_FILTER_GAIN, val & 0x3);
		offs = REG_RX_FILTER_COEF_ADDR - REG_TX_FILTER_COEF_ADDR;
		phy->rx_fir_ntaps = ntaps;
	} else {
		if (gain_dB == -6)
			fir_conf = TX_FIR_GAIN_6DB;
		phy->tx_fir_ntaps = ntaps;
	}

	val = ntaps / 16 - 1;

	fir_conf |= FIR_NUM_TAPS(val) | FIR_SELECT(dest) | FIR_START_CLK;

	ad9361_spi_write(spi, REG_TX_FILTER_CONF + offs, fir_conf);

	for (val = 0; val < ntaps; val++) {
		ad9361_spi_write(spi, REG_TX_FILTER_COEF_ADDR + offs, val);
		ad9361_spi_write(spi, REG_TX_FILTER_COEF_WRITE_DATA_1 + offs,
				 coef[val] & 0xFF);
		ad9361_spi_write(spi, REG_TX_FILTER_COEF_WRITE_DATA_2 + offs,
				 coef[val] >> 8);
		ad9361_spi_write(spi, REG_TX_FILTER_CONF + offs,
				 fir_conf | FIR_WRITE);
		ad9361_spi_write(spi, REG_TX_FILTER_COEF_READ_DATA_2 + offs, 0);
		ad9361_spi_write(spi, REG_TX_FILTER_COEF_READ_DATA_2 + offs, 0);
	}

	ad9361_spi_write(spi, REG_TX_FILTER_CONF + offs, fir_conf);
	fir_conf &= ~FIR_START_CLK;
	ad9361_spi_write(spi, REG_TX_FILTER_CONF + offs, fir_conf);

	return 0;
}

static int ad9361_parse_fir(struct ad9361_rf_phy *phy,
				 char *data, u32 size)
{
	char *line;
	int i = 0, ret, txc, rxc;
	int tx = -1, tx_gain, tx_int;
	int rx = -1, rx_gain, rx_dec;
	short coef_tx[128];
	short coef_rx[128];
	char *ptr = data;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size) {
			break;
		}

		if (line[0] == '#')
			continue;

		if (tx < 0) {
			ret = sscanf(line, "TX %d GAIN %d INT %d",
				     &tx, &tx_gain, &tx_int);
			if (ret == 3)
				continue;
			else
				tx = -1;
		}
		if (rx < 0) {
			ret = sscanf(line, "RX %d GAIN %d DEC %d",
				     &rx, &rx_gain, &rx_dec);
			if (ret == 3)
				continue;
			else
				tx = -1;
		}
		ret = sscanf(line, "%d,%d", &txc, &rxc);
		if (ret == 1) {
			coef_tx[i] = coef_rx[i] = (short)txc;
			i++;
			continue;
		} else if (ret == 2) {
			coef_tx[i] = (short)txc;
			coef_rx[i] = (short)rxc;
			i++;
			continue;
		}
	}

	switch (tx) {
	case FIR_TX1:
	case FIR_TX2:
	case FIR_TX1_TX2:
		ret = ad9361_load_fir_filter_coef(phy, tx, tx_gain, i, coef_tx);
		phy->tx_fir_int = tx_int;
		break;
	default:
		ret = -EINVAL;
	}

	switch (rx | FIR_IS_RX) {
	case FIR_RX1:
	case FIR_RX2:
	case FIR_RX1_RX2:
		ret = ad9361_load_fir_filter_coef(phy, rx | FIR_IS_RX,
						  rx_gain, i, coef_rx);
		phy->rx_fir_dec = rx_dec;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		return ret;


	return size;
}


static int ad9361_validate_enable_fir(struct ad9361_rf_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	int ret;
	unsigned long rx[6], tx[6];
	u32 max;

	dev_dbg(dev, "%s: TX FIR EN=%d/TAPS%d/INT%d, RX FIR EN=%d/TAPS%d/DEC%d",
		__func__, !phy->bypass_tx_fir, phy->tx_fir_ntaps, phy->tx_fir_int,
		!phy->bypass_rx_fir, phy->rx_fir_ntaps, phy->rx_fir_dec);

	if (!phy->bypass_tx_fir) {
		if (!(phy->tx_fir_int == 1 || phy->tx_fir_int == 2 ||
			phy->tx_fir_int == 4)) {
			dev_err(dev,
				"%s: Invalid: Interpolation %d in filter config",
				__func__, phy->tx_fir_int);
			return -EINVAL;
		}


		if (phy->tx_fir_int == 1 && phy->tx_fir_ntaps > 64) {
			dev_err(dev,
				"%s: Invalid: TAPS > 64 and Interpolation = 1",
				__func__);
			return -EINVAL;
		}
	}

	if (!phy->bypass_rx_fir) {
		if (!(phy->rx_fir_dec == 1 || phy->rx_fir_dec == 2 ||
			phy->rx_fir_dec == 4)) {
			dev_err(dev,
				"%s: Invalid: Decimation %d in filter config",
				__func__, phy->rx_fir_dec);

			return -EINVAL;
		}
	}

	ret = ad9361_calculate_rf_clock_chain(phy,
			clk_get_rate(phy->clks[TX_SAMPL_CLK]),
			phy->rate_governor, rx, tx);

	if (ret < 0) {
		dev_err(dev,
			"%s: Calculating filter rates failed %d",
			__func__, ret);

		return ret;
	}

	if (!phy->bypass_tx_fir) {
		max = ((rx[ADC_FREQ] / 2) / tx[TX_SAMPL_FREQ]) * 16;
		if (phy->tx_fir_ntaps > max) {
			dev_err(dev,
				"%s: Invalid: ratio ADC/2 / TX_SAMPL * 16 > TAPS"
				"(max %d, adc %lu, tx %lu)",
				__func__, max, rx[ADC_FREQ], tx[TX_SAMPL_FREQ]);
			return -EINVAL;
		}
	}

	if (!phy->bypass_rx_fir) {
		max = ((rx[ADC_FREQ] / 2) / rx[RX_SAMPL_FREQ]) * 16;
		if (phy->rx_fir_ntaps > max) {
			dev_err(dev,
				"%s: Invalid: ratio ADC/2 / RX_SAMPL * 16 > TAPS (max %d)",
				__func__, max);
			return -EINVAL;
		}
	}

	ret = ad9361_set_trx_clock_chain(phy, rx, tx);
	if (ret < 0)
		return ret;

	/*
	 * Workaround for clock framework since clocks don't change the we
	 * manually need to enable the filter
	 */

	if (phy->rx_fir_dec == 1 || phy->bypass_rx_fir) {
		ad9361_spi_writef(phy->spi, REG_RX_ENABLE_FILTER_CTRL,
			RX_FIR_ENABLE_DECIMATION(~0), !phy->bypass_rx_fir);
	}

	if (phy->tx_fir_int == 1 || phy->bypass_tx_fir) {
		ad9361_spi_writef(phy->spi, REG_TX_ENABLE_FILTER_CTRL,
			TX_FIR_ENABLE_INTERPOLATION(~0), !phy->bypass_tx_fir);
	}

	return ad9361_update_rf_bandwidth(phy, phy->current_rx_bw_Hz,
			phy->current_tx_bw_Hz);
}

static void ad9361_work_func(struct work_struct *work)
{
	struct ad9361_rf_phy *phy =
		container_of(work, struct ad9361_rf_phy, work);
	int ret;

	dev_dbg(&phy->spi->dev, "%s:", __func__);

	ret = ad9361_do_calib_run(phy, TX_QUAD_CAL);
	if (ret < 0)
		dev_err(&phy->spi->dev,
			"%s: TX QUAD cal failed", __func__);

	complete_all(&phy->complete);
	clear_bit(0, &phy->flags);
}

/*
 * AD9361 Clocks
 */

#define to_clk_priv(_hw) container_of(_hw, struct refclk_scale, hw)

static inline int ad9361_set_muldiv(struct refclk_scale *priv, u32 mul, u32 div)
{
	priv->mult = mul;
	priv->div = div;
	return 0;
}

static int ad9361_get_clk_scaler(struct clk_hw *hw)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	u32 tmp, tmp1;

	switch (clk_priv->source) {
	case BB_REFCLK:
		tmp = ad9361_spi_read(spi, REG_CLOCK_CTRL);
		tmp &= 0x3;
		break;
	case RX_REFCLK:
		tmp = ad9361_spi_readf(spi, REG_REF_DIVIDE_CONFIG_1,
					RX_REF_DIVIDER_MSB);
		tmp1 = ad9361_spi_readf(spi, REG_REF_DIVIDE_CONFIG_2,
					RX_REF_DIVIDER_LSB);
		tmp = (tmp << 1) | tmp1;
		break;
	case TX_REFCLK:
		tmp = ad9361_spi_readf(spi, REG_REF_DIVIDE_CONFIG_2,
				       TX_REF_DIVIDER(~0));
		break;
	case ADC_CLK:
		tmp = ad9361_spi_read(spi, REG_BBPLL);
		return ad9361_set_muldiv(clk_priv, 1, 1 << (tmp & 0x7));
	case R2_CLK:
		tmp = ad9361_spi_readf(spi, REG_RX_ENABLE_FILTER_CTRL,
				       DEC3_ENABLE_DECIMATION(~0));
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case R1_CLK:
		tmp = ad9361_spi_readf(spi, REG_RX_ENABLE_FILTER_CTRL, RHB2_EN);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case CLKRF_CLK:
		tmp = ad9361_spi_readf(spi, REG_RX_ENABLE_FILTER_CTRL, RHB1_EN);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case RX_SAMPL_CLK:
		tmp = ad9361_spi_readf(spi, REG_RX_ENABLE_FILTER_CTRL,
				       RX_FIR_ENABLE_DECIMATION(~0));

		if (!tmp)
			tmp = 1; /* bypass filter */
		else
			tmp = (1 << (tmp - 1));

		return ad9361_set_muldiv(clk_priv, 1, tmp);
	case DAC_CLK:
		tmp = ad9361_spi_readf(spi, REG_BBPLL, BIT(3));
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case T2_CLK:
		tmp = ad9361_spi_readf(spi, REG_TX_ENABLE_FILTER_CTRL,
				       THB3_ENABLE_INTERP(~0));
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case T1_CLK:
		tmp = ad9361_spi_readf(spi, REG_TX_ENABLE_FILTER_CTRL, THB2_EN);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case CLKTF_CLK:
		tmp = ad9361_spi_readf(spi, REG_TX_ENABLE_FILTER_CTRL, THB1_EN);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case TX_SAMPL_CLK:
		tmp = ad9361_spi_readf(spi, REG_TX_ENABLE_FILTER_CTRL,
				       TX_FIR_ENABLE_INTERPOLATION(~0));

		if (!tmp)
			tmp = 1; /* bypass filter */
		else
			tmp = (1 << (tmp - 1));

		return ad9361_set_muldiv(clk_priv, 1, tmp);
	default:
		return -EINVAL;
	}

	/* REFCLK Scaler */
	switch (tmp) {
	case 0:
		ad9361_set_muldiv(clk_priv, 1, 1);
		break;
	case 1:
		ad9361_set_muldiv(clk_priv, 1, 2);
		break;
	case 2:
		ad9361_set_muldiv(clk_priv, 1, 4);
		break;
	case 3:
		ad9361_set_muldiv(clk_priv, 2, 1);
		break;
	default:
		return -EINVAL;

	}

	return 0;
}

static int ad9361_to_refclk_scaler(struct refclk_scale *clk_priv)
{
	/* REFCLK Scaler */
	switch (((clk_priv->mult & 0xF) << 4) | (clk_priv->div & 0xF)) {
	case 0x11:
		return 0;
	case 0x12:
		return 1;
	case 0x14:
		return 2;
	case 0x21:
		return 3;
	default:
		return -EINVAL;
	}
};

static int ad9361_set_clk_scaler(struct clk_hw *hw, bool set)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	u32 tmp;
	int ret;

	switch (clk_priv->source) {
	case BB_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set)
			return ad9361_spi_writef(spi, REG_CLOCK_CTRL, 0x3, ret);
		break;

	case RX_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set) {
			tmp = ret;
			ret = ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_1,
						RX_REF_DIVIDER_MSB, tmp >> 1);
			ret |= ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_2,
						 RX_REF_DIVIDER_LSB, tmp & 1);
			return ret;
		}
		break;
	case TX_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set)
			return ad9361_spi_writef(spi, REG_REF_DIVIDE_CONFIG_2,
						0x0C, ret);
		break;
	case ADC_CLK:
		tmp = ilog2((u8)clk_priv->div);
		if (clk_priv->mult != 1 || tmp > 6 || tmp < 1)
			return -EINVAL;

		if (set)
			return ad9361_spi_writef(spi, REG_BBPLL, 0x7, tmp);
		break;
	case R2_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 3 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_RX_ENABLE_FILTER_CTRL,
						 DEC3_ENABLE_DECIMATION(~0),
						 clk_priv->div - 1);
		break;
	case R1_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_RX_ENABLE_FILTER_CTRL,
						 RHB2_EN, clk_priv->div - 1);
		break;
	case CLKRF_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_RX_ENABLE_FILTER_CTRL,
						 RHB1_EN, clk_priv->div - 1);
		break;
	case RX_SAMPL_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 4 ||
			clk_priv->div < 1 || clk_priv->div == 3)
			return -EINVAL;

		if (clk_priv->phy->bypass_rx_fir)
			tmp = 0;
		else
			tmp = ilog2(clk_priv->div) + 1;

		if (set)
			return ad9361_spi_writef(spi, REG_RX_ENABLE_FILTER_CTRL,
						 RX_FIR_ENABLE_DECIMATION(~0), tmp);
		break;
	case DAC_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_BBPLL,
						 BIT(3), clk_priv->div - 1);
		break;
	case T2_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 3 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_TX_ENABLE_FILTER_CTRL,
						 THB3_ENABLE_INTERP(~0),
						 clk_priv->div - 1);
		break;
	case T1_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_TX_ENABLE_FILTER_CTRL,
						 THB2_EN, clk_priv->div - 1);
		break;
	case CLKTF_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, REG_TX_ENABLE_FILTER_CTRL,
						 THB1_EN, clk_priv->div - 1);
		break;
	case TX_SAMPL_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 4 ||
			clk_priv->div < 1 || clk_priv->div == 3)
			return -EINVAL;

		if (clk_priv->phy->bypass_tx_fir)
			tmp = 0;
		else
			tmp = ilog2(clk_priv->div) + 1;

		if (set)
			return ad9361_spi_writef(spi, REG_TX_ENABLE_FILTER_CTRL,
					TX_FIR_ENABLE_INTERPOLATION(~0), tmp);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned long ad9361_clk_factor_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	u64 rate;

	ad9361_get_clk_scaler(hw);
	rate = (parent_rate * clk_priv->mult) / clk_priv->div;

	return (unsigned long)rate;
}

static long ad9361_clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	int ret;

	if (rate >= *prate) {
		clk_priv->mult = DIV_ROUND_CLOSEST(rate, *prate);
		clk_priv->div = 1;

	} else {
		clk_priv->div = DIV_ROUND_CLOSEST(*prate, rate);
		clk_priv->mult = 1;
	}

	ret = ad9361_set_clk_scaler(hw, false);
	if (ret < 0)
		return ret;

	return (*prate / clk_priv->div) * clk_priv->mult;
}

static int ad9361_clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	if (rate >= parent_rate) {
		clk_priv->mult = DIV_ROUND_CLOSEST(rate, parent_rate);
		clk_priv->div = 1;
	} else {
		clk_priv->div = DIV_ROUND_CLOSEST(parent_rate, rate);
		clk_priv->mult = 1;
	}

	return ad9361_set_clk_scaler(hw, true);
}

struct clk_ops refclk_scale_ops = {
	.round_rate = ad9361_clk_factor_round_rate,
	.set_rate = ad9361_clk_factor_set_rate,
	.recalc_rate = ad9361_clk_factor_recalc_rate,
};

/*
 * BBPLL
 */

static unsigned long ad9361_bbpll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	u64 rate;
	unsigned long fract, integer;
	u8 buf[4];

	ad9361_spi_readm(clk_priv->spi, REG_INTEGER_BB_FREQ_WORD, &buf[0],
			REG_INTEGER_BB_FREQ_WORD - REG_FRACT_BB_FREQ_WORD_1 + 1);

	fract = (buf[3] << 16) | (buf[2] << 8) | buf[1];
	integer = buf[0];

	rate = ((u64)parent_rate * fract);
	do_div(rate, BBPLL_MODULUS);
	rate += (u64)parent_rate * integer;

	return (unsigned long)rate;
}

static long ad9361_bbpll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	u64 tmp;
	u32 fract, integer;

	if (rate > MAX_BBPLL_FREQ)
		return MAX_BBPLL_FREQ;

	if (rate < MIN_BBPLL_FREQ)
		return MIN_BBPLL_FREQ;

	tmp = do_div(rate, *prate);
	tmp = tmp * BBPLL_MODULUS + (*prate >> 1);
	do_div(tmp, *prate);

	integer = rate;
	fract = tmp;

	tmp = *prate * (u64)fract;
	do_div(tmp, BBPLL_MODULUS);
	tmp += *prate * integer;

	return tmp;
}

static int ad9361_bbpll_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	u64 tmp;
	u32 fract, integer;
	int icp_val;
	u8 lf_defaults[3] = {0x35, 0x5B, 0xE8};

	dev_dbg(&spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	/*
	 * Setup Loop Filter and CP Current
	 * Scale is 150uA @ (1280MHz BBPLL, 40MHz REFCLK)
	 */
	tmp = (rate >> 7) * 150ULL;
	do_div(tmp, (parent_rate >> 7) * 32UL + (tmp >> 1));

	/* 25uA/LSB, Offset 25uA */
	icp_val = DIV_ROUND_CLOSEST((u32)tmp, 25U) - 1;

	icp_val = clamp(icp_val, 1, 64);

	ad9361_spi_write(spi, REG_CP_CURRENT, icp_val);
	ad9361_spi_writem(spi, REG_LOOP_FILTER_3, lf_defaults,
			  ARRAY_SIZE(lf_defaults));

	/* Allow calibration to occur and set cal count to 1024 for max accuracy */
	ad9361_spi_write(spi, REG_VCO_CTRL,
			 FREQ_CAL_ENABLE | FREQ_CAL_COUNT_LENGTH(3));
	/* Set calibration clock to REFCLK/4 for more accuracy */
	ad9361_spi_write(spi, REG_SDM_CTRL, 0x10);

	/* Calculate and set BBPLL frequency word */
	tmp = do_div(rate, parent_rate);
	tmp = tmp *(u64) BBPLL_MODULUS + (parent_rate >> 1);
	do_div(tmp, parent_rate);

	integer = rate;
	fract = tmp;

	ad9361_spi_write(spi, REG_INTEGER_BB_FREQ_WORD, integer);
	ad9361_spi_write(spi, REG_FRACT_BB_FREQ_WORD_3, fract);
	ad9361_spi_write(spi, REG_FRACT_BB_FREQ_WORD_2, fract >> 8);
	ad9361_spi_write(spi, REG_FRACT_BB_FREQ_WORD_1, fract >> 16);

	ad9361_spi_write(spi, REG_SDM_CTRL_1, INIT_BB_FO_CAL | BBPLL_RESET_BAR); /* Start BBPLL Calibration */
	ad9361_spi_write(spi, REG_SDM_CTRL_1, BBPLL_RESET_BAR); /* Clear BBPLL start calibration bit */

	ad9361_spi_write(spi, REG_VCO_PROGRAM_1, 0x86); /* Increase BBPLL KV and phase margin */
	ad9361_spi_write(spi, REG_VCO_PROGRAM_2, 0x01); /* Increase BBPLL KV and phase margin */
	ad9361_spi_write(spi, REG_VCO_PROGRAM_2, 0x05); /* Increase BBPLL KV and phase margin */

	return ad9361_check_cal_done(clk_priv->phy, REG_CH_1_OVERFLOW,
				     BBPLL_LOCK, 1);
}

struct clk_ops bbpll_clk_ops = {
	.round_rate = ad9361_bbpll_round_rate,
	.set_rate = ad9361_bbpll_set_rate,
	.recalc_rate = ad9361_bbpll_recalc_rate,
};

/*
 * RFPLL
 */

static u64 ad9361_calc_rfpll_freq(u64 parent_rate,
				   u64 integer,
				   u64 fract, u32 vco_div)
{
	u64 rate;

	rate = parent_rate * fract;
	do_div(rate, RFPLL_MODULUS);
	rate += parent_rate * integer;

	return rate >> (vco_div + 1);
}

static int ad9361_calc_rfpll_divder(u64 freq,
			     u64 parent_rate, u32 *integer,
			     u32 *fract, int *vco_div, u64 *vco_freq)
{
	u64 tmp;
	int div;

	if (freq > MAX_CARRIER_FREQ_HZ || freq < MIN_CARRIER_FREQ_HZ)
		return -EINVAL;

	div = -1;

	while (freq < MIN_VCO_FREQ_HZ) {
		freq <<= 1;
		div++;
	}

	*vco_div = div;
	*vco_freq = freq;
	tmp = do_div(freq, parent_rate);
	tmp = tmp * RFPLL_MODULUS + (parent_rate >> 1);
	do_div(tmp, parent_rate);
	*integer = freq;
	*fract = tmp;

	return 0;
}

static unsigned long ad9361_rfpll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	unsigned long fract, integer;
	u8 buf[5];
	u32 reg, div_mask, vco_div;

	switch (clk_priv->source) {
	case RX_RFPLL:
		reg = REG_RX_FRACT_BYTE_2;
		div_mask = RX_VCO_DIVIDER(~0);
		break;
	case TX_RFPLL:
		reg = REG_TX_FRACT_BYTE_2;
		div_mask = TX_VCO_DIVIDER(~0);
		break;
	default:
		return -EINVAL;
	}

	ad9361_spi_readm(clk_priv->spi, reg, &buf[0], ARRAY_SIZE(buf));

	vco_div = ad9361_spi_readf(clk_priv->spi, REG_RFPLL_DIVIDERS, div_mask);

	fract = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	integer = buf[3] << 8 | buf[4];

	return ad9361_to_clk(ad9361_calc_rfpll_freq(parent_rate, integer,
					      fract, vco_div));
}

static long ad9361_rfpll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	u64 vco;
	u32 fract, integer;
	int vco_div, ret;

	ret = ad9361_calc_rfpll_divder(ad9361_from_clk(rate), *prate, &integer, &fract, &vco_div, &vco);
	if (ret < 0)
		return ret;

	return ad9361_to_clk(ad9361_calc_rfpll_freq(*prate, integer, fract, vco_div));
}

static int ad9361_rfpll_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct ad9361_rf_phy *phy = clk_priv->phy;
	u64 vco;
	u8 buf[5];
	u32 reg, div_mask, lock_reg, fract, integer;
	int vco_div, ret;

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	ret = ad9361_calc_rfpll_divder(ad9361_from_clk(rate), parent_rate,
				&integer, &fract, &vco_div, &vco);
	if (ret < 0)
		return ret;

	switch (clk_priv->source) {
	case RX_RFPLL:
		reg = REG_RX_FRACT_BYTE_2;
		lock_reg = REG_RX_CP_OVERRANGE_VCO_LOCK;
		div_mask = RX_VCO_DIVIDER(~0);
		break;
	case TX_RFPLL:
		reg = REG_TX_FRACT_BYTE_2;
		lock_reg = REG_TX_CP_OVERRANGE_VCO_LOCK;
		div_mask = TX_VCO_DIVIDER(~0);
		break;
	default:
		return -EINVAL;

	}

	ad9361_rfpll_vco_init(phy, div_mask == TX_VCO_DIVIDER(~0),
			      vco, parent_rate);

	buf[0] = fract >> 16;
	buf[1] = fract >> 8;
	buf[2] = fract & 0xFF;
	buf[3] = integer >> 8;
	buf[4] = integer & 0xFF;

	ad9361_spi_writem(clk_priv->spi, reg, buf, 5);
	ad9361_spi_writef(clk_priv->spi, REG_RFPLL_DIVIDERS, div_mask, vco_div);

	/* Load Gain Table */
	if (clk_priv->source == RX_RFPLL) {
		ret = ad9361_load_gt(phy, ad9361_from_clk(rate), GT_RX1 + GT_RX2);
		if (ret < 0)
			return ret;
	}

	/* For RX LO we typically have the tracking option enabled
	 * so for now do nothing here.
	 */
	if (phy->auto_cal_en && (clk_priv->source == TX_RFPLL))
		if (abs(phy->last_tx_quad_cal_freq - ad9361_from_clk(rate)) >
			phy->cal_threshold_freq) {

			set_bit(0, &phy->flags);
			INIT_COMPLETION(phy->complete);
			schedule_work(&phy->work);
			phy->last_tx_quad_cal_freq = ad9361_from_clk(rate);
		}

	return ad9361_check_cal_done(phy, lock_reg, BIT(1), 1);
}

struct clk_ops rfpll_clk_ops = {
	.round_rate = ad9361_rfpll_round_rate,
	.set_rate = ad9361_rfpll_set_rate,
	.recalc_rate = ad9361_rfpll_recalc_rate,
};

static struct clk *ad9361_clk_register(struct ad9361_rf_phy *phy, const char *name,
		const char *parent_name, unsigned long flags,
		u32 source)
{
	struct refclk_scale *clk_priv;
	struct clk_init_data init;
	struct clk *clk;

	clk_priv = kmalloc(sizeof(*clk_priv), GFP_KERNEL);
	if (!clk_priv) {
		pr_err("%s: could not allocate fixed factor clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* struct refclk_scale assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	init.name = name;

	switch (source) {
	case BBPLL_CLK:
		init.ops = &bbpll_clk_ops;
		break;
	case RX_RFPLL:
	case TX_RFPLL:
		init.ops = &rfpll_clk_ops;
		break;
	default:
		init.ops = &refclk_scale_ops;
	}

	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clk = clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clk_data.clks[source] = clk;

	if (IS_ERR(clk))
		kfree(clk_priv);

	return clk;
}

static int register_clocks(struct ad9361_rf_phy *phy)
{
	u32 flags = 0;

	phy->clk_data.clks = devm_kzalloc(&phy->spi->dev,
					 sizeof(*phy->clk_data.clks) *
					 NUM_AD9361_CLKS, GFP_KERNEL);
	if (!phy->clk_data.clks) {
		dev_err(&phy->spi->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	if (phy->pdata->debug_mode)
		flags = CLK_GET_RATE_NOCACHE;

	phy->clk_data.clk_num = NUM_AD9361_CLKS;

	/* Scaled Reference Clocks */
	phy->clks[TX_REFCLK] = ad9361_clk_register(phy,
					"tx_refclk", "ad9361_ext_refclk",
					flags | CLK_IGNORE_UNUSED,
					TX_REFCLK);

	phy->clks[RX_REFCLK] = ad9361_clk_register(phy,
					"rx_refclk", "ad9361_ext_refclk",
					flags | CLK_IGNORE_UNUSED,
					RX_REFCLK);

	phy->clks[BB_REFCLK] = ad9361_clk_register(phy,
					"bb_refclk", "ad9361_ext_refclk",
					flags | CLK_IGNORE_UNUSED,
					BB_REFCLK);

	/* Base Band PLL Clock */
	phy->clks[BBPLL_CLK] = ad9361_clk_register(phy,
					"bbpll_clk", "bb_refclk",
					flags | CLK_IGNORE_UNUSED,
					BBPLL_CLK);

	phy->clks[ADC_CLK] = ad9361_clk_register(phy,
					"adc_clk", "bbpll_clk",
					flags | CLK_IGNORE_UNUSED,
					ADC_CLK);

	phy->clks[R2_CLK] = ad9361_clk_register(phy,
					"r2_clk", "adc_clk",
					flags | CLK_IGNORE_UNUSED,
					R2_CLK);

	phy->clks[R1_CLK] = ad9361_clk_register(phy,
					"r1_clk", "r2_clk",
					flags | CLK_IGNORE_UNUSED,
					R1_CLK);

	phy->clks[CLKRF_CLK] = ad9361_clk_register(phy,
					"clkrf_clk", "r1_clk",
					flags | CLK_IGNORE_UNUSED,
					CLKRF_CLK);

	phy->clks[RX_SAMPL_CLK] = ad9361_clk_register(phy,
					"rx_sampl_clk", "clkrf_clk",
					flags | CLK_IGNORE_UNUSED,
					RX_SAMPL_CLK);


	phy->clks[DAC_CLK] = ad9361_clk_register(phy,
					"dac_clk", "adc_clk",
					flags | CLK_IGNORE_UNUSED,
					DAC_CLK);

	phy->clks[T2_CLK] = ad9361_clk_register(phy,
					"t2_clk", "dac_clk",
					flags | CLK_IGNORE_UNUSED,
					T2_CLK);

	phy->clks[T1_CLK] = ad9361_clk_register(phy,
					"t1_clk", "t2_clk",
					flags | CLK_IGNORE_UNUSED,
					T1_CLK);

	phy->clks[CLKTF_CLK] = ad9361_clk_register(phy,
					"clktf_clk", "t1_clk",
					flags | CLK_IGNORE_UNUSED,
					CLKTF_CLK);

	phy->clks[TX_SAMPL_CLK] = ad9361_clk_register(phy,
					"tx_sampl_clk", "clktf_clk",
					flags | CLK_IGNORE_UNUSED,
					TX_SAMPL_CLK);

	phy->clks[RX_RFPLL] = ad9361_clk_register(phy,
					"rx_rfpll", "rx_refclk",
					flags | CLK_IGNORE_UNUSED,
					RX_RFPLL);

	phy->clks[TX_RFPLL] = ad9361_clk_register(phy,
					"tx_rfpll", "tx_refclk",
					flags | CLK_IGNORE_UNUSED,
					TX_RFPLL);


	return 0;
}

// static int ad9361_scale_table[][2] = {
// 	{2000, 0}, {2100, 6}, {2200, 7},
// 	{2300, 8}, {2400, 9}, {2500, 10},
// };
//
// static void ad9361_convert_scale_table(struct axiadc_converter *conv)
// {
// 	int i;
//
// 	for (i = 0; i < conv->chip_info->num_scales; i++)
// 		conv->chip_info->scale_table[i][0] =
// 			(conv->chip_info->scale_table[i][0] * 1000000ULL) >>
// 			conv->chip_info->channel[0].scan_type.realbits;
//
// }
//
// static ssize_t ad9361_show_scale_available(struct iio_dev *indio_dev,
// 				   uintptr_t private,
// 					   const struct iio_chan_spec *chan,
// 					   char *buf)
// {
// 	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
// 	int i, len = 0;
//
// 	for (i = 0; i < conv->chip_info->num_scales; i++)
// 		len += sprintf(buf + len, "0.%06u ",
// 			       conv->chip_info->scale_table[i][0]);
//
// 	len += sprintf(buf + len, "\n");
//
// 	return len;
// }
//
//
// static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
// 	{
// 		.name = "scale_available",
// 		.read = ad9361_show_scale_available,
// 		.shared = true,
// 	},
// 	{ },
// };

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask =  IIO_CHAN_INFO_CALIBSCALE_SEPARATE_BIT |		\
			IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |		\
			IIO_CHAN_INFO_CALIBPHASE_SEPARATE_BIT |		\
			IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
			/*.ext_info = axiadc_ext_info,*/			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9361] = {
		.name = "AD9361",
		.max_rate = 250000000UL,
		.max_testmode = 0,
		.num_channels = 4,
		.channel[0] = AIM_CHAN(0, 0, 12, 's'),
		.channel[1] = AIM_CHAN(1, 1, 12, 's'),
		.channel[2] = AIM_CHAN(2, 2, 12, 's'),
		.channel[3] = AIM_CHAN(3, 3, 12, 's'),
	},
};
static struct attribute *ad9361_attributes[] = {
	NULL,
};

static const struct attribute_group ad9361_attribute_group = {
	.attrs = ad9361_attributes,
};

static int ad9361_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9361_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (mask) {
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

		return 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9361_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	axiadc_write(st, ADI_REG_CNTRL,
		     (conv->phy->pdata->rx2tx2) ? 0 : ADI_R1_MODE);

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_ENABLE);
	}

	return 0;
}

static int ad9361_register_axi_converter(struct ad9361_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->id = ad9361_spi_read(spi, REG_PRODUCT_ID) & PRODUCT_ID_MASK;
	if (conv->id != PRODUCT_ID_9361) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
  		ret = -ENODEV;
  		goto out;
	}

	conv->chip_info = &axiadc_chip_info_tbl[ID_AD9361];
	conv->adc_output_mode = OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->write = ad9361_spi_write;
	conv->read = ad9361_spi_read;
	conv->write_raw = ad9361_write_raw;
	conv->read_raw = ad9361_read_raw;
	conv->post_setup = ad9361_post_setup;
	conv->attrs = &ad9361_attribute_group;
	conv->spi = spi;
	conv->phy = phy;

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
out:
	spi_set_drvdata(spi, NULL);
	return ret;

}

enum ad9361_iio_dev_attr {
	AD9361_RF_RX_BANDWIDTH,
	AD9361_RF_TX_BANDWIDTH,
	AD9361_ENSM_MODE,
	AD9361_ENSM_MODE_AVAIL,
	AD9361_CALIB_MODE,
	AD9361_CALIB_MODE_AVAIL,
	AD9361_RX_PATH_FREQ,
	AD9361_TX_PATH_FREQ,
	AD9361_TRX_RATE_GOV,
	AD9361_TRX_RATE_GOV_AVAIL,
	AD9361_FIR_RX_ENABLE,
	AD9361_FIR_TX_ENABLE,
	AD9361_FIR_TRX_ENABLE,
	AD9361_BBDC_OFFS_ENABLE,
	AD9361_RFDC_OFFS_ENABLE,
	AD9361_QUAD_ENABLE,
};

static ssize_t ad9361_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	long readin;
	int ret = 0;
	u32 val;
	bool res;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
	case AD9361_RF_RX_BANDWIDTH:
		ret = kstrtol(buf, 10, &readin);
		if (ret)
			break;

		if (phy->current_rx_bw_Hz != readin)
			ret = ad9361_update_rf_bandwidth(phy, readin,
					phy->current_tx_bw_Hz);
		else
			ret = 0;
		break;
	case AD9361_RF_TX_BANDWIDTH:
		ret = kstrtol(buf, 10, &readin);
		if (ret)
			break;

		if (phy->current_tx_bw_Hz != readin)
			ret = ad9361_update_rf_bandwidth(phy,
					phy->current_rx_bw_Hz, readin);
		else
			ret = 0;
		break;
	case AD9361_ENSM_MODE:
		if (sysfs_streq(buf, "tx"))
			val = ENSM_STATE_TX;
		else if (sysfs_streq(buf, "rx"))
			val = ENSM_STATE_RX;
		else if (sysfs_streq(buf, "alert"))
			val = ENSM_STATE_ALERT;
		else if (sysfs_streq(buf, "fdd"))
			val = ENSM_STATE_FDD;
		else if (sysfs_streq(buf, "sleep"))
			val = ENSM_STATE_SLEEP_WAIT;
//		else if (sysfs_streq(buf, "pinctrl")
		else
			break;
		ret = ad9361_ensm_set_state(phy, val);
		break;
	case AD9361_TRX_RATE_GOV:
		if (sysfs_streq(buf, "highest_osr"))
			phy->rate_governor = 0;
		else if (sysfs_streq(buf, "low_power"))
			phy->rate_governor = 1;
		else
			ret = -EINVAL;
		break;
	case AD9361_FIR_TRX_ENABLE:
		ret = strtobool(buf, &res);
		if (ret < 0)
			break;

		if ((phy->bypass_rx_fir == phy->bypass_tx_fir) == !res)
			break;

		phy->bypass_rx_fir = phy->bypass_tx_fir = !res;

		ret = ad9361_validate_enable_fir(phy);
		if (ret < 0) {
			phy->bypass_rx_fir = true;
			phy->bypass_tx_fir = true;
		}

		break;
	case AD9361_FIR_RX_ENABLE:
		ret = strtobool(buf, &res);
		if (ret < 0)
			break;

		if(phy->bypass_rx_fir == !res)
			break;

		phy->bypass_rx_fir = !res;

		ret = ad9361_validate_enable_fir(phy);
		if (ret < 0) {
			phy->bypass_rx_fir = true;
		}

		break;
	case AD9361_FIR_TX_ENABLE:
		ret = strtobool(buf, &res);
		if (ret < 0)
			break;

		if(phy->bypass_tx_fir == !res)
			break;

		phy->bypass_tx_fir = !res;

		ret = ad9361_validate_enable_fir(phy);
		if (ret < 0) {
			phy->bypass_tx_fir = true;
		}


		break;
	case AD9361_CALIB_MODE:
		val = 0;
		if (sysfs_streq(buf, "auto"))
			phy->auto_cal_en = true;
		else if (sysfs_streq(buf, "manual"))
			phy->auto_cal_en = false;
		else if (sysfs_streq(buf, "rx_quad"))
			val = RX_QUAD_CAL;
		else if (sysfs_streq(buf, "tx_quad"))
			val = TX_QUAD_CAL;
		else if (sysfs_streq(buf, "rf_dc_offs"))
			val = RFDC_CAL;
		else
			break;

		if (val)
			ret = ad9361_do_calib_run(phy, val);

		break;
	case AD9361_BBDC_OFFS_ENABLE:
		ret = strtobool(buf, &phy->bbdc_track_en);
		if (ret < 0)
			break;
		ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				phy->rfdc_track_en, phy->quad_track_en);
		break;
	case AD9361_RFDC_OFFS_ENABLE:
		ret = strtobool(buf, &phy->rfdc_track_en);
		if (ret < 0)
			break;
		ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				phy->rfdc_track_en, phy->quad_track_en);
		break;
	case AD9361_QUAD_ENABLE:
		ret = strtobool(buf, &phy->quad_track_en);
		if (ret < 0)
			break;
		ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				phy->rfdc_track_en, phy->quad_track_en);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	unsigned long clk[6];


	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9361_RF_RX_BANDWIDTH:
		ret = sprintf(buf, "%u\n", phy->current_rx_bw_Hz);
		break;
	case AD9361_RF_TX_BANDWIDTH:
		ret = sprintf(buf, "%u\n", phy->current_tx_bw_Hz);
		break;
	case AD9361_ENSM_MODE:
		ret = sprintf(buf, "%s\n",
			      ad9361_ensm_states[ad9361_spi_readf
			      (phy->spi, REG_STATE, ENSM_STATE(~0))]);
		break;
	case AD9361_ENSM_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", phy->pdata->fdd ?
				"sleep alert fdd pinctrl" :
				"sleep alert rx tx pinctrl");
		break;
	case AD9361_TX_PATH_FREQ:
		ad9361_get_trx_clock_chain(phy, NULL, clk);
		ret = sprintf(buf, "BBPLL:%lu DAC:%lu T2:%lu T1:%lu TF:%lu TXSAMP:%lu\n",
			      clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
		break;
	case AD9361_RX_PATH_FREQ:
		ad9361_get_trx_clock_chain(phy, clk, NULL);
		ret = sprintf(buf, "BBPLL:%lu ADC:%lu R2:%lu R1:%lu RF:%lu RXSAMP:%lu\n",
			      clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
		break;
	case AD9361_TRX_RATE_GOV:
		ret = sprintf(buf, "%s\n", phy->rate_governor ?
				 "low_power" : "highest_osr");
		break;
	case AD9361_TRX_RATE_GOV_AVAIL:
		ret = sprintf(buf, "%s\n", "low_power highest_osr");
		break;
	case AD9361_FIR_RX_ENABLE:
		ret = sprintf(buf, "%d\n", !phy->bypass_rx_fir);
		break;
	case AD9361_FIR_TX_ENABLE:
		ret = sprintf(buf, "%d\n", !phy->bypass_tx_fir);
		break;
	case AD9361_FIR_TRX_ENABLE:
		ret = sprintf(buf, "%d\n", !phy->bypass_tx_fir && !phy->bypass_rx_fir);
		break;
	case AD9361_CALIB_MODE_AVAIL:
		ret = sprintf(buf, "auto manual tx_quad rf_dc_offs rx_quad\n");
		break;
	case AD9361_CALIB_MODE:
		ret = sprintf(buf, "%s\n", phy->auto_cal_en ? "auto" : "manual");
		break;
	case AD9361_BBDC_OFFS_ENABLE:
		ret = sprintf(buf, "%d\n", phy->bbdc_track_en);
		break;
	case AD9361_RFDC_OFFS_ENABLE:
		ret = sprintf(buf, "%d\n", phy->rfdc_track_en);
		break;
	case AD9361_QUAD_ENABLE:
		ret = sprintf(buf, "%d\n", phy->quad_track_en);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(in_voltage_rf_bandwidth, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_RF_RX_BANDWIDTH);

static IIO_DEVICE_ATTR(out_voltage_rf_bandwidth, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_RF_TX_BANDWIDTH);

static IIO_DEVICE_ATTR(ensm_mode, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_ENSM_MODE);

static IIO_DEVICE_ATTR(ensm_mode_available, S_IRUGO,
			ad9361_phy_show,
			NULL,
			AD9361_ENSM_MODE_AVAIL);

static IIO_DEVICE_ATTR(calib_mode, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_CALIB_MODE);

static IIO_DEVICE_ATTR(calib_mode_available, S_IRUGO,
			ad9361_phy_show,
			NULL,
			AD9361_CALIB_MODE_AVAIL);

static IIO_DEVICE_ATTR(rx_path_rates, S_IRUGO,
			ad9361_phy_show,
			NULL,
			AD9361_RX_PATH_FREQ);

static IIO_DEVICE_ATTR(tx_path_rates, S_IRUGO,
			ad9361_phy_show,
			NULL,
			AD9361_TX_PATH_FREQ);

static IIO_DEVICE_ATTR(trx_rate_governor, S_IRUGO,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_TRX_RATE_GOV);

static IIO_DEVICE_ATTR(trx_rate_governor_available, S_IRUGO,
			ad9361_phy_show,
			NULL,
			AD9361_TRX_RATE_GOV_AVAIL);

static IIO_DEVICE_ATTR(in_voltage_filter_fir_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_FIR_RX_ENABLE);

static IIO_DEVICE_ATTR(out_voltage_filter_fir_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_FIR_TX_ENABLE);

static IIO_DEVICE_ATTR(in_out_voltage_filter_fir_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_FIR_TRX_ENABLE);

static IIO_DEVICE_ATTR(in_voltage_bb_dc_offset_tracking_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_BBDC_OFFS_ENABLE);

static IIO_DEVICE_ATTR(in_voltage_rf_dc_offset_tracking_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_RFDC_OFFS_ENABLE);

static IIO_DEVICE_ATTR(in_voltage_quadrature_tracking_en, S_IRUGO | S_IWUSR,
			ad9361_phy_show,
			ad9361_phy_store,
			AD9361_QUAD_ENABLE);

static struct attribute *ad9361_phy_attributes[] = {
	&iio_dev_attr_in_voltage_filter_fir_en.dev_attr.attr,
	&iio_dev_attr_out_voltage_filter_fir_en.dev_attr.attr,
	&iio_dev_attr_in_out_voltage_filter_fir_en.dev_attr.attr,
	&iio_dev_attr_in_voltage_rf_bandwidth.dev_attr.attr,
	&iio_dev_attr_out_voltage_rf_bandwidth.dev_attr.attr,
	&iio_dev_attr_ensm_mode.dev_attr.attr,
	&iio_dev_attr_ensm_mode_available.dev_attr.attr,
	&iio_dev_attr_calib_mode.dev_attr.attr,
	&iio_dev_attr_calib_mode_available.dev_attr.attr,
	&iio_dev_attr_tx_path_rates.dev_attr.attr,
	&iio_dev_attr_rx_path_rates.dev_attr.attr,
	&iio_dev_attr_trx_rate_governor.dev_attr.attr,
	&iio_dev_attr_trx_rate_governor_available.dev_attr.attr,
	&iio_dev_attr_in_voltage_bb_dc_offset_tracking_en.dev_attr.attr,
	&iio_dev_attr_in_voltage_rf_dc_offset_tracking_en.dev_attr.attr,
	&iio_dev_attr_in_voltage_quadrature_tracking_en.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9361_phy_attribute_group = {
	.attrs = ad9361_phy_attributes,
};


static int ad9361_phy_reg_access(struct iio_dev *indio_dev,
			      u32 reg, u32 writeval,
			      u32 *readval)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad9361_spi_write(phy->spi, reg, writeval);
	} else {
		*readval =  ad9361_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9361_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	u64 readin;
	unsigned long tmp;
	int ret = 0;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:
		tmp = clk_set_rate(phy->clks[RX_RFPLL],
				   ad9361_to_clk(readin));
		break;

	case 1:
		tmp = clk_set_rate(phy->clks[TX_RFPLL],
				   ad9361_to_clk(readin));
		if (test_bit(0, &phy->flags))
			wait_for_completion(&phy->complete);

		break;

	default:
		ret = -EINVAL;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	u64 val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:
		val = ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL]));
		break;

	case 1:
		val = ad9361_from_clk(clk_get_rate(phy->clks[TX_RFPLL]));
		break;

	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

#define _AD9361_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9361_phy_lo_read, \
	.write = ad9361_phy_lo_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info ad9361_phy_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_AD9361_EXT_LO_INFO("frequency", 0),
	{ },
};

static int ad9361_set_agc_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, u32 mode)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	struct rf_gain_ctrl gc = {0};

	gc.ant = chan->channel + 1;
	gc.mode = phy->agc_mode[chan->channel] = mode;

	return ad9361_set_gain_ctrl_mode(phy, &gc);
}

static int ad9361_get_agc_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	return phy->agc_mode[chan->channel];
}


static const char * const ad9361_agc_modes[] =
 	{"manual", "fast_attack", "slow_attack", "hybrid"};

static const struct iio_enum ad9361_agc_modes_available = {
	.items = ad9361_agc_modes,
	.num_items = ARRAY_SIZE(ad9361_agc_modes),
	.get = ad9361_get_agc_mode,
	.set = ad9361_set_agc_mode,

};

static ssize_t ad9361_phy_rx_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
//	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	u64 readin;
	int ret = 0;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:

		break;

	case 1:

		break;

	default:
		ret = -EINVAL;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_rx_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	struct rf_rssi rssi = {0};
	int val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	rssi.ant = chan->channel + 1;
	rssi.duration = 1;
	ret = ad9361_read_rssi(phy, &rssi);
	val = rssi.symbol;

	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "-%u.%02u dB\n",
			val / rssi.multiplier, val % rssi.multiplier);
}

#define _AD9361_EXT_RX_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9361_phy_rx_read, \
	.write = ad9361_phy_rx_write, \
	.private = _ident, \
}


static const struct iio_chan_spec_ext_info ad9361_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE("gain_control_mode", &ad9361_agc_modes_available),
	IIO_ENUM("gain_control_mode", false, &ad9361_agc_modes_available),
	_AD9361_EXT_RX_INFO("rssi", 1),
	{ },
};

static int ad9361_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			ret = ad9361_get_tx_atten(phy, chan->channel + 1);
			if (ret < 0)
				return -EINVAL;

			*val = -1 * (ret / 1000);
			*val2 = (ret % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			struct rf_rx_gain rx_gain = {0};
			ret = ad9361_get_rx_gain(phy, chan->channel + 1, &rx_gain);
			*val = rx_gain.gain_db;
			*val2 = 0;
		}
		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output)
			*val = (int)clk_get_rate(phy->clks[TX_SAMPL_CLK]);
		else
			*val = (int)clk_get_rate(phy->clks[RX_SAMPL_CLK]);
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		*val = ad9361_get_temp(phy);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int ad9361_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	u32 code;
	unsigned long rx[6], tx[6];
	int ret;


	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));
			ret = ad9361_set_tx_atten(phy, code,
				chan->channel == 0, chan->channel == 1);
		} else {
			struct rf_rx_gain rx_gain = {0};
			rx_gain.gain_db = val;
			ret = ad9361_set_rx_gain(phy, chan->channel + 1, &rx_gain);
		}
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (phy->rx_eq_2tx && (chan->output == 0)) {
			ret = 0;
			break;
		}

		ret = ad9361_calculate_rf_clock_chain(phy, val,
			phy->rate_governor, rx, tx);
		if (ret < 0)
			goto out;
		ad9361_set_trx_clock_chain(phy, rx, tx);
		ret = ad9361_update_rf_bandwidth(phy, phy->current_rx_bw_Hz,
						phy->current_tx_bw_Hz);
		break;

	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_chan_spec ad9361_phy_chan[] = {
{
	.type = IIO_TEMP,
	.indexed = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
}, {	/* RX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.extend_name = "RX_LO",
	.ext_info = ad9361_phy_ext_info,
}, {	/* TX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 1,
	.extend_name = "TX_LO",
	.ext_info = ad9361_phy_ext_info,
}, {	/* TX1 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
}, {	/* RX1 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = ad9361_phy_rx_ext_info,

}, {	/* TX2 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
}, {	/* RX2 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = ad9361_phy_rx_ext_info,
}};

static const struct iio_info ad9361_phy_info = {
	.read_raw = &ad9361_phy_read_raw,
	.write_raw = &ad9361_phy_write_raw,
	.debugfs_reg_access = &ad9361_phy_reg_access,
	.attrs = &ad9361_phy_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
struct ad9361_dport_config {
	u8 reg;
	u8 offset;
	char name[40];
};

static const struct ad9361_dport_config ad9361_dport_config[] = {
	{1, 7, "adi,pp-tx-swap-enable"},
	{1, 6, "adi,pp-rx-swap-enable"},
	{1, 5, "adi,tx-channel-swap-enable"},
	{1, 4, "adi,rx-channel-swap-enable"},
	{1, 3, "adi,rx-frame-pulse-mode-enable"},
//	{1, 2, "adi,2t2r-timing-enable"},
	{1, 1, "adi,invert-data-bus-enable"},
	{1, 0, "adi,invert-data-clk-enable"},
	{2, 7, "adi,fdd-alt-word-order-enable"},
	{2, 2, "adi,invert-rx-frame-enable"},
	{3, 7, "adi,fdd-rx-rate-2tx-enable"},
	{3, 6, "adi,swap-ports-enable"},
	{3, 5, "adi,single-data-rate-enable"},
	{3, 4, "adi,lvds-mode-enable"},
	{3, 3, "adi,half-duplex-mode-enable"},
	{3, 2, "adi,single-port-mode-enable"},
	{3, 1, "adi,full-port-enable"},
	{3, 0, "adi,full-duplex-swap-bits-enable"},
};

static int __ad9361_of_get_u32(struct device_node *np, const char *propname,
			     u32 defval, void *out_value, u32 size)
{
	int ret;
	u32 tmp = defval;

	ret = of_property_read_u32(np, propname, &tmp);

	if (out_value) {
		switch (size){
		case 1:
			*(u8*)out_value = tmp;
			break;
		case 2:
			*(u16*)out_value = tmp;
			break;
		case 4:
			*(u32*)out_value = tmp;
			break;
		default:
			ret = -EINVAL;
		}
	}

	return ret;
}
#define ad9361_of_get_u32(dnp, name, def, outp) \
	__ad9361_of_get_u32(dnp, name, def, outp, sizeof(*outp))

static void ad9361_of_get_bool(struct device_node *np, const char *propname,
			     bool *out_value)
{
	*out_value = of_property_read_bool(np, propname);
}

static struct ad9361_phy_platform_data *ad9361_phy_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9361_phy_platform_data *pdata;
	u32 tmp;
	u64 tmpl;
	u32 array[6] = {0};
	int ret, i;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	ad9361_of_get_bool(np, "adi,frequency-division-duplex-mode-enable",
			   &pdata->fdd);

	ad9361_of_get_bool(np, "adi,ensm-enable-pin-level-mode-enable",
			   &pdata->ensm_pin_level_mode);

	ad9361_of_get_bool(np, "adi,ensm-enable-txnrx-control-enable",
			   &pdata->ensm_pin_ctrl);

	ad9361_of_get_bool(np, "adi,debug-mode-enable",
			   &pdata->debug_mode);

	for (i = 0; i < ARRAY_SIZE(ad9361_dport_config); i++)
		pdata->port_ctrl.pp_conf[ad9361_dport_config[i].reg - 1] |=
			(of_property_read_bool(np, ad9361_dport_config[i].name)
			<< ad9361_dport_config[i].offset);

	tmp = 0;
	of_property_read_u32(np, "adi,delay-rx-data", &tmp);
	pdata->port_ctrl.pp_conf[1] |= (tmp & 0x3);

	tmp = 0;
	of_property_read_u32(np, "adi,rx-data-clock-delay", &tmp);
	pdata->port_ctrl.rx_clk_data_delay = (tmp & 0xF) << 4;
	tmp = 0;
	of_property_read_u32(np, "adi,rx-data-delay", &tmp);
	pdata->port_ctrl.rx_clk_data_delay |= (tmp & 0xF);

	tmp = 0;
	of_property_read_u32(np, "adi,tx-fb-clock-delay", &tmp);
	pdata->port_ctrl.tx_clk_data_delay = (tmp & 0xF) << 4;
	tmp = 0;
	of_property_read_u32(np, "adi,tx-data-delay", &tmp);
	pdata->port_ctrl.tx_clk_data_delay |= (tmp & 0xF);

	tmp = 75;
	of_property_read_u32(np, "adi,lvds-bias-mV", &tmp);
	pdata->port_ctrl.lvds_bias_ctrl = (tmp / 75) & 0x7;
	pdata->port_ctrl.lvds_bias_ctrl |= (of_property_read_bool(np,
			"adi,lvds-rx-onchip-termination-enable") << 5);

	ad9361_of_get_bool(np, "adi,2rx-2tx-mode-enable", &pdata->rx2tx2);

	ad9361_of_get_bool(np, "adi,frequency-division-duplex-mode-enable",
			   &pdata->fdd);

	ad9361_of_get_bool(np, "adi,split-gain-table-mode-enable",
			   &pdata->split_gt);

// 	pdata->ensm_pin_ctl_en = of_property_read_bool(np,
// 			"adi,ensm-state-pincontrol-enable");

	ad9361_of_get_u32(np, "adi,rx-rf-port-input-select", 0,
			  &pdata->rf_rx_input_sel);
	ad9361_of_get_u32(np, "adi,tx-rf-port-input-select", 0,
			  &pdata->rf_tx_output_sel);

	tmpl = 2400000000ULL;
	of_property_read_u64(np, "adi,rx-synthesizer-frequency-hz", &tmpl);
	pdata->rx_synth_freq = tmpl;

	tmpl = 2440000000ULL;
 	of_property_read_u64(np, "adi,tx-synthesizer-frequency-hz", &tmpl);
	pdata->tx_synth_freq = tmpl;

	ret = of_property_read_u32_array(np, "adi,dcxo-coarse-and-fine-tune",
			      array, 2);
	if (ret < 0) {
		if (ret == -EINVAL)
			pdata->use_extclk = true;
		else
			return NULL;
	}

	pdata->dcxo_coarse = array[0];
	pdata->dcxo_fine = array[1];

	ret = of_property_read_u32_array(np, "adi,rx-path-clock-frequencies",
			pdata->rx_path_clks, ARRAY_SIZE(pdata->rx_path_clks));
	if (ret < 0)
		return NULL;

	ret = of_property_read_u32_array(np, "adi,tx-path-clock-frequencies",
			pdata->tx_path_clks, ARRAY_SIZE(pdata->tx_path_clks));
	if (ret < 0)
		return NULL;

	ad9361_of_get_u32(np, "adi,rf-rx-bandwidth-hz", 18000000UL,
			  &pdata->rf_rx_bandwidth_Hz);
	ad9361_of_get_u32(np, "adi,rf-tx-bandwidth-hz", 18000000UL,
			  &pdata->rf_tx_bandwidth_Hz);
	ad9361_of_get_u32(np, "adi,tx-attenuation-mdB", 10000, &pdata->tx_atten);

	/* Gain Control */

	ad9361_of_get_u32(np, "adi,gc-rx1-mode", 0, &pdata->gain_ctrl.rx1_mode);
	ad9361_of_get_u32(np, "adi,gc-rx2-mode", 0, &pdata->gain_ctrl.rx2_mode);
	ad9361_of_get_u32(np, "adi,gc-adc-ovr-sample-size", 4,
			  &pdata->gain_ctrl.adc_ovr_sample_size);
	ad9361_of_get_u32(np, "adi,gc-adc-small-overload-thresh", 47,
			  &pdata->gain_ctrl.adc_small_overload_thresh);
	ad9361_of_get_u32(np, "adi,gc-adc-large-overload-thresh", 58,
			  &pdata->gain_ctrl.adc_large_overload_thresh);
	ad9361_of_get_u32(np, "adi,gc-lmt-overload-high-thresh", 800,
			  &pdata->gain_ctrl.lmt_overload_high_thresh);
	ad9361_of_get_u32(np, "adi,gc-lmt-overload-low-thresh", 704,
			  &pdata->gain_ctrl.lmt_overload_low_thresh);
	ad9361_of_get_u32(np, "adi,gc-analog-settling-time", 8,
			  &pdata->gain_ctrl.analog_settling_time);
	ad9361_of_get_u32(np, "adi,gc-dec-pow-measurement-duration", 8192,
			  &pdata->gain_ctrl.dec_pow_measuremnt_duration);
	ad9361_of_get_u32(np, "adi,gc-low-power-thresh", 24,
			  &pdata->gain_ctrl.low_power_thresh);
	ad9361_of_get_bool(np, "adi,gc-dig-gain-enable",
			  &pdata->gain_ctrl.dig_gain_en);
	ad9361_of_get_u32(np, "adi,gc-max-dig-gain", 15,
			  &pdata->gain_ctrl.max_dig_gain);

	ad9361_of_get_bool(np, "adi,mgc-rx1-ctrl-inp-enable",
			   &pdata->gain_ctrl.mgc_rx1_ctrl_inp_en);
	ad9361_of_get_bool(np, "adi,mgc-rx2-ctrl-inp-enable",
			   &pdata->gain_ctrl.mgc_rx1_ctrl_inp_en);
	ad9361_of_get_u32(np, "adi,mgc-inc-gain-step", 2,
			  &pdata->gain_ctrl.mgc_inc_gain_step);
	ad9361_of_get_u32(np, "adi,mgc-dec-gain-step", 2,
			  &pdata->gain_ctrl.mgc_dec_gain_step);
	ad9361_of_get_u32(np, "adi,mgc-split-table-ctrl-inp-gain-mode", 0,
			  &pdata->gain_ctrl.mgc_split_table_ctrl_inp_gain_mode);
	ad9361_of_get_u32(np, "adi,agc-attack-delay-us", 10,
			  &pdata->gain_ctrl.agc_attack_delay_us);
	ad9361_of_get_u32(np, "adi,agc-settling-delay", 10,
			  &pdata->gain_ctrl.agc_settling_delay);
	ad9361_of_get_u32(np, "adi,agc-outer-thresh-high", 5,
			  &pdata->gain_ctrl.agc_outer_thresh_high);
	ad9361_of_get_u32(np, "adi,agc-outer-thresh-high-dec-steps", 2,
			  &pdata->gain_ctrl.agc_outer_thresh_high_dec_steps);
	ad9361_of_get_u32(np, "adi,agc-inner-thresh-high", 10,
			  &pdata->gain_ctrl.agc_inner_thresh_high);
	ad9361_of_get_u32(np, "adi,agc-inner-thresh-high-dec-steps", 1,
			  &pdata->gain_ctrl.agc_inner_thresh_high_dec_steps);
	ad9361_of_get_u32(np, "adi,agc-inner-thresh-low", 12,
			  &pdata->gain_ctrl.agc_inner_thresh_low);
	ad9361_of_get_u32(np, "adi,agc-inner-thresh-low-inc-steps", 1,
			  &pdata->gain_ctrl.agc_inner_thresh_low_inc_steps);
	ad9361_of_get_u32(np, "adi,agc-outer-thresh-low", 18,
			  &pdata->gain_ctrl.agc_outer_thresh_low);
	ad9361_of_get_u32(np, "adi,agc-outer-thresh-low-inc-steps", 2,
			  &pdata->gain_ctrl.agc_outer_thresh_low_inc_steps);
	ad9361_of_get_u32(np, "adi,agc-adc-small-overload-exceed-counter", 10,
			  &pdata->gain_ctrl.adc_small_overload_exceed_counter);
	ad9361_of_get_u32(np, "adi,agc-adc-large-overload-exceed-counter", 10,
			  &pdata->gain_ctrl.adc_large_overload_exceed_counter);
	ad9361_of_get_u32(np, "adi,agc-adc-large-overload-inc-steps", 2,
			  &pdata->gain_ctrl.adc_large_overload_inc_steps);
	ad9361_of_get_bool(np, "adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable",
			   &pdata->gain_ctrl.adc_lmt_small_overload_prevent_gain_inc);
	ad9361_of_get_u32(np, "adi,agc-lmt-overload-large-exceed-counter", 10,
			  &pdata->gain_ctrl.lmt_overload_large_exceed_counter);
	ad9361_of_get_u32(np, "adi,agc-lmt-overload-small-exceed-counter", 10,
			  &pdata->gain_ctrl.lmt_overload_small_exceed_counter);
	ad9361_of_get_u32(np, "adi,agc-lmt-overload-large-inc-steps", 2,
			  &pdata->gain_ctrl.lmt_overload_large_inc_steps);
	ad9361_of_get_u32(np, "adi,agc-dig-saturation-exceed-counter", 3,
			  &pdata->gain_ctrl.dig_saturation_exceed_counter);
	ad9361_of_get_u32(np, "adi,agc-dig-gain-step-size", 4,
			  &pdata->gain_ctrl.dig_gain_step_size);
	ad9361_of_get_bool(np, "adi,agc-sync-for-gain-counter-enable",
			   &pdata->gain_ctrl.sync_for_gain_counter_en);
	ad9361_of_get_u32(np, "adi,agc-gain-update-counter", 30698,
			  &pdata->gain_ctrl.gain_update_counter);
	ad9361_of_get_bool(np, "adi,agc-immed-gain-change-if-large-adc-overload-enable",
			   &pdata->gain_ctrl.immed_gain_change_if_large_adc_overload);
	ad9361_of_get_bool(np, "adi,agc-immed-gain-change-if-large-lmt-overload-enable",
			   &pdata->gain_ctrl.immed_gain_change_if_large_lmt_overload);

	/* RSSI Control */

	ad9361_of_get_u32(np, "adi,rssi-restart-mode", 3,
			  &pdata->rssi_ctrl.restart_mode);
	ad9361_of_get_bool(np, "adi,rssi-unit-is-rx-samples-enable",
			   &pdata->rssi_ctrl.rssi_unit_is_rx_samples);
	ad9361_of_get_u32(np, "adi,rssi-delay", 1,
			  &pdata->rssi_ctrl.rssi_delay);
	ad9361_of_get_u32(np, "adi,rssi-wait", 1,
			  &pdata->rssi_ctrl.rssi_wait);
	ad9361_of_get_u32(np, "adi,rssi-duration", 1000,
			  &pdata->rssi_ctrl.rssi_duration);

	/* Control Outs Control */

	ad9361_of_get_u32(np, "adi,ctrl-outs-index", 0,
			  &pdata->ctrl_outs_ctrl.index);
	ad9361_of_get_u32(np, "adi,ctrl-outs-enable-mask", 0xFF,
			  &pdata->ctrl_outs_ctrl.en_mask);

	/* eLNA Control */

	ad9361_of_get_u32(np, "adi,elna-gain-mdB", 0,
			  &pdata->elna_ctrl.gain_mdB);
	ad9361_of_get_u32(np, "adi,ctrl-bypass-loss-mdB", 0,
			  &pdata->elna_ctrl.bypass_loss_mdB);
	ad9361_of_get_bool(np, "adi,elna-rx1-gpo0-control-enable",
			   &pdata->elna_ctrl.elna_1_control_en);
	ad9361_of_get_bool(np, "adi,elna-rx2-gpo1-control-enable",
			   &pdata->elna_ctrl.elna_2_control_en);

	ret = of_get_gpio(np, 0);
	if (ret < 0)
		pdata->gpio_resetb = -1;
	else
		pdata->gpio_resetb = ret;

	/* AuxADC Temp Sense Control */

	ad9361_of_get_u32(np, "adi,temp-sense-measurement-interval-ms", 1000,
			  &pdata->auxadc_ctrl.temp_time_inteval_ms);
	ad9361_of_get_u32(np, "adi,temp-sense-offset-signed", 0xBD,
			  &pdata->auxadc_ctrl.offset); /* signed */
	ad9361_of_get_bool(np, "adi,temp-sense-periodic-measurement-enable",
			   &pdata->auxadc_ctrl.periodic_temp_measuremnt);
	ad9361_of_get_u32(np, "adi,temp-sense-decimation", 256,
			  &pdata->auxadc_ctrl.temp_sensor_decimation);
	ad9361_of_get_u32(np, "adi,aux-adc-rate", 40000000UL,
			  &pdata->auxadc_ctrl.auxadc_clock_rate);
	ad9361_of_get_u32(np, "adi,aux-adc-decimation", 256,
			  &pdata->auxadc_ctrl.auxadc_decimation);

	return pdata;
}
#else
static
struct ad9361_phy_platform_data *ad9361_phy_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static ssize_t
ad9361_fir_bin_write(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	return ad9361_parse_fir(phy, buf, count);
}

static ssize_t
ad9361_fir_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	if (off)
		return 0;

	return sprintf(buf, "FIR Rx: %d,%d Tx: %d,%d\n",
		       phy->rx_fir_ntaps, phy->rx_fir_dec,
			phy->tx_fir_ntaps, phy->tx_fir_int);
}

static int ad9361_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9361_rf_phy *phy;
	struct clk *clk = NULL;
	int ret, rev;

	dev_info(&spi->dev, "%s : enter", __func__);

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	indio_dev = iio_device_alloc(sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;

	phy->pdata = ad9361_phy_parse_dt(&spi->dev);
	if (phy->pdata == NULL)
		return -EINVAL;

	if (gpio_is_valid(phy->pdata->gpio_resetb)) {
		ret = devm_gpio_request_one(&spi->dev, phy->pdata->gpio_resetb,
			GPIOF_OUT_INIT_HIGH, "AD9361 RESETB");
	} /*else {
		ret = -ENODEV;
	}*/

	if (ret) {
		dev_err(&spi->dev, "fail to request RESET GPIO-%d",
			phy->pdata->gpio_resetb);
		ret = -ENODEV;
		goto out;
	}

	phy->spi = spi;
	phy->clk_refin = clk;

	phy->current_table = RXGAIN_TBLS_END;
	phy->bypass_tx_fir = true;
	phy->bypass_rx_fir = true;
	phy->rate_governor = 1;
	phy->rfdc_track_en = true;
	phy->bbdc_track_en = true;
	phy->quad_track_en = true;


	if (ad9361_reset(phy)) {
		ad9361_spi_write(spi, REG_SPI_CONF, SOFT_RESET | _SOFT_RESET); /* RESET */
		ad9361_spi_write(spi, REG_SPI_CONF, 0x0);
	}

	ret = ad9361_spi_read(spi, REG_PRODUCT_ID);
	if ((ret & PRODUCT_ID_MASK) != PRODUCT_ID_9361) {
		dev_err(&spi->dev, "%s : Unsupported PRODUCT_ID 0x%X",
			__func__, ret);
		ret = -ENODEV;
		goto out;
	}

	rev = ret & REV_MASK;

	if (spi_get_device_id(spi)->driver_data == 9643)
		phy->pdata->rx2tx2 = false;

	INIT_WORK(&phy->work, ad9361_work_func);
	init_completion(&phy->complete);

	ret = register_clocks(phy);
	if (ret < 0)
		goto out;

	ad9361_init_gain_tables(phy);

	ret = ad9361_setup(phy);
	if (ret < 0)
		goto out;

	of_clk_add_provider(spi->dev.of_node,
			    of_clk_src_onecell_get, &phy->clk_data);

	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "filter_fir_config";
	phy->bin.attr.mode = S_IWUSR;
	phy->bin.write = ad9361_fir_bin_write;
	phy->bin.read = ad9361_fir_bin_read;
	phy->bin.size = 4096;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ad9361-phy";
	indio_dev->info = &ad9361_phy_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9361_phy_chan;
	indio_dev->num_channels = ARRAY_SIZE(ad9361_phy_chan) -
		(phy->pdata->rx2tx2 ? 0 : 2);

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out;
	ret = ad9361_register_axi_converter(phy);
	if (ret < 0)
		goto out1;
	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		goto out1;

	dev_info(&spi->dev, "%s : AD9361 Rev %d successfully initialized",
		 __func__, rev);

	return 0;

out1:
	iio_device_unregister(indio_dev);

out:
	clk_disable_unprepare(clk);
	iio_device_free(indio_dev);
	spi_set_drvdata(spi, NULL);

	return ret;
}

static int ad9361_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9361_rf_phy *phy = conv->phy;

	sysfs_remove_bin_file(&phy->indio_dev->dev.kobj, &phy->bin);
	iio_device_unregister(phy->indio_dev);
	clk_disable_unprepare(conv->clk);
	iio_device_free(phy->indio_dev);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9361_id[] = {
	{"ad9361", 9361}, /* 2RX2TX */
	{"ad9364", 9364}, /* 1RX1TX */
	{}
};
MODULE_DEVICE_TABLE(spi, ad9361_id);

static struct spi_driver ad9361_driver = {
	.driver = {
		.name	= "ad9361",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9361_probe,
	.remove		= ad9361_remove,
	.id_table	= ad9361_id,
};
module_spi_driver(ad9361_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9361 ADC");
MODULE_LICENSE("GPL v2");
