/*
 * Linear Technology LTC4306 and LTC4305 I2C multiplexer/switch
 *
 * Copyright (C) 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * Based on: i2c-mux-pca954x.c
 *
 * Datasheet: http://cds.linear.com/docs/en/datasheet/4306.pdf
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#define LTC4306_MAX_NCHANS 4

#define LTC_REG_STATUS	0x0
#define LTC_REG_CONFIG	0x1
#define LTC_REG_MODE	0x2
#define LTC_REG_SWITCH	0x3

#define LTC_DOWNSTREAM_ACCL_EN BIT(6)
#define LTC_UPSTREAM_ACCL_EN BIT(7)

#define LTC_GPIO_ALL_INPUT	0xC0

enum ltc_type {
	ltc_4305,
	ltc_4306,
};

struct chip_desc {
	u8 nchans;
	u8 num_gpios;
};

struct ltc4306 {
	struct i2c_client *client;
	struct gpio_chip gpiochip;
	const struct chip_desc *chip;

	u8 deselect;
	u8 regs[LTC_REG_SWITCH + 1];

};

/* Provide specs for the PCA954x types we know about */
static const struct chip_desc chips[] = {
	[ltc_4305] = {
		.nchans = 2,
	},
	[ltc_4306] = {
		.nchans = LTC4306_MAX_NCHANS,
		.num_gpios = 2,
	},
};

static int ltc4306_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ltc4306 *data = gpiochip_get_data(chip);
	int ret = 0;

	if (gpiochip_line_is_open_drain(chip, offset) ||
		(data->regs[LTC_REG_MODE] & BIT(7 - offset))) {
		/* Open Drain or Input */
		ret = i2c_smbus_read_byte_data(data->client, LTC_REG_CONFIG);
		if (ret < 0)
			return ret;

		return !!(ret & BIT(1 - offset));
	} else {
		return !!(data->regs[LTC_REG_CONFIG] & BIT(5 - offset));
	}
}

static void ltc4306_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct ltc4306 *data = gpiochip_get_data(chip);

	if (value)
		data->regs[LTC_REG_CONFIG] |= BIT(5 - offset);
	else
		data->regs[LTC_REG_CONFIG] &= ~BIT(5 - offset);


	i2c_smbus_write_byte_data(data->client, LTC_REG_CONFIG,
				 data->regs[LTC_REG_CONFIG]);
}

static int ltc4306_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct ltc4306 *data = gpiochip_get_data(chip);

	data->regs[LTC_REG_MODE] |= BIT(7 - offset);

	return i2c_smbus_write_byte_data(data->client, LTC_REG_MODE,
					 data->regs[LTC_REG_MODE]);
}

static int ltc4306_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int value)
{
	struct ltc4306 *data = gpiochip_get_data(chip);

	ltc4306_gpio_set(chip, offset, value);
	data->regs[LTC_REG_MODE] &= ~BIT(7 - offset);

	return i2c_smbus_write_byte_data(data->client, LTC_REG_MODE,
					 data->regs[LTC_REG_MODE]);
}

static int ltc4306_gpio_set_single_ended(struct gpio_chip *chip,
				     unsigned int offset,
				     enum single_ended_mode mode)
{
	struct ltc4306 *data = gpiochip_get_data(chip);

	switch (mode) {
	case LINE_MODE_OPEN_DRAIN:
		data->regs[LTC_REG_MODE] &= ~BIT(4 - offset);
		break;
	case LINE_MODE_PUSH_PULL:
		data->regs[LTC_REG_MODE] |= BIT(4 - offset);
		break;
	default:
		return -ENOTSUPP;
	}

	return i2c_smbus_write_byte_data(data->client, LTC_REG_MODE,
					 data->regs[LTC_REG_MODE]);
}

static int ltc4306_gpio_init(struct ltc4306 *data)
{
	if (!data->chip->num_gpios)
		return 0;

	data->gpiochip.label = dev_name(&data->client->dev);
	data->gpiochip.base = -1;
	data->gpiochip.ngpio = data->chip->num_gpios;
	data->gpiochip.parent = &data->client->dev;
	data->gpiochip.can_sleep = true;
	data->gpiochip.direction_input = ltc4306_gpio_direction_input;
	data->gpiochip.direction_output = ltc4306_gpio_direction_output;
	data->gpiochip.get = ltc4306_gpio_get;
	data->gpiochip.set = ltc4306_gpio_set;
	data->gpiochip.set_single_ended = ltc4306_gpio_set_single_ended;
	data->gpiochip.owner = THIS_MODULE;

	/* gpiolib assumes all GPIOs default input */
	data->regs[LTC_REG_MODE] |= LTC_GPIO_ALL_INPUT;
	i2c_smbus_write_byte_data(data->client, LTC_REG_MODE,
					 data->regs[LTC_REG_MODE]);

	return devm_gpiochip_add_data(&data->client->dev,
				      &data->gpiochip, data);

}

/*
 * Write to chip register. Don't use i2c_transfer()/i2c_smbus_xfer()
 * as they will try to lock the adapter a second time.
 */
static int ltc4306_reg_write(struct i2c_adapter *adap,
			     struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	if (adap->algo->master_xfer) {
		struct i2c_msg msg;
		char buf[2];

		msg.addr = client->addr;
		msg.flags = 0;
		msg.len = 2;
		buf[0] = reg;
		buf[1] = val;
		msg.buf = buf;
		ret = __i2c_transfer(adap, &msg, 1);
	} else {
		union i2c_smbus_data data;

		data.byte = val;
		ret = adap->algo->smbus_xfer(adap, client->addr,
					     client->flags,
					     I2C_SMBUS_WRITE,
					     reg,
					     I2C_SMBUS_BYTE_DATA, &data);
	}

	return ret;
}

static int ltc4306_select_chan(struct i2c_mux_core *muxc, u32 chan)
{
	struct ltc4306 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;
	u8 regval;
	int ret = 0;

	regval = BIT(7 - chan);

	/* Only select the channel if its different from the last channel */
	if (data->regs[LTC_REG_SWITCH] != regval) {
		ret = ltc4306_reg_write(muxc->parent, client,
					LTC_REG_SWITCH, regval);
		data->regs[LTC_REG_SWITCH] = ret < 0 ? 0 : regval;
	}

	return ret;
}

static int ltc4306_deselect_mux(struct i2c_mux_core *muxc, u32 chan)
{
	struct ltc4306 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;

	if (!(data->deselect & BIT(chan)))
		return 0;

	/* Deselect active channel */
	data->regs[LTC_REG_SWITCH] = 0;

	return ltc4306_reg_write(muxc->parent, client,
				 LTC_REG_SWITCH, data->regs[LTC_REG_SWITCH]);
}

static const struct i2c_device_id ltc4306_id[] = {
	{ "ltc4305", ltc_4305 },
	{ "ltc4306", ltc_4306 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4306_id);

static const struct of_device_id ltc4306_of_match[] = {
	{ .compatible = "lltc,ltc4305", .data = &chips[ltc_4305] },
	{ .compatible = "lltc,ltc4306", .data = &chips[ltc_4306] },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4306_of_match);

static int ltc4306_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	struct device_node *of_node = client->dev.of_node;
	bool idle_disconnect_dt = false;
	struct gpio_desc *gpio;
	struct i2c_mux_core *muxc;
	struct ltc4306 *data;
	const struct of_device_id *match;
	int num, ret;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	muxc = i2c_mux_alloc(adap, &client->dev,
			     LTC4306_MAX_NCHANS, sizeof(*data), 0,
			     ltc4306_select_chan, ltc4306_deselect_mux);
	if (!muxc)
		return -ENOMEM;
	data = i2c_mux_priv(muxc);

	i2c_set_clientdata(client, muxc);
	data->client = client;

	/* Enable the mux if a enable GPIO is specified. */
	gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	/* Write the mux register at addr to verify
	 * that the mux is in fact present. This also
	 * initializes the mux to disconnected state.
	 */
	if (i2c_smbus_write_byte_data(client, LTC_REG_SWITCH, 0) < 0) {
		dev_warn(&client->dev, "probe failed\n");
		return -ENODEV;
	}

	match = of_match_device(of_match_ptr(ltc4306_of_match), &client->dev);
	if (match)
		data->chip = of_device_get_match_data(&client->dev);
	else
		data->chip = &chips[id->driver_data];

	if (of_node) {
		idle_disconnect_dt =
			of_property_read_bool(of_node,
					      "i2c-mux-idle-disconnect");
		if (of_property_read_bool(of_node,
					  "ltc,downstream-accelerators-enable"))
			data->regs[LTC_REG_CONFIG] |= LTC_DOWNSTREAM_ACCL_EN;

		if (of_property_read_bool(of_node,
					  "ltc,upstream-accelerators-enable"))
			data->regs[LTC_REG_CONFIG] |= LTC_UPSTREAM_ACCL_EN;

		if (i2c_smbus_write_byte_data(client, LTC_REG_CONFIG,
					      data->regs[LTC_REG_CONFIG]) < 0) {
			dev_warn(&client->dev, "probe failed\n");
			return -ENODEV;
		}

	}

	ret = ltc4306_gpio_init(data);
	if (ret < 0)
		return ret;

	/* Now create an adapter for each channel */
	for (num = 0; num < data->chip->nchans; num++) {

		data->deselect |= idle_disconnect_dt << num;

		ret = i2c_mux_add_adapter(muxc, 0, num, 0);
		if (ret) {
			dev_err(&client->dev,
				"failed to register multiplexed adapter %d\n",
				num);
			goto virt_reg_failed;
		}
	}

	dev_info(&client->dev,
		 "registered %d multiplexed busses for I2C switch %s\n",
		 num, client->name);

	return 0;

virt_reg_failed:
	i2c_mux_del_adapters(muxc);
	return ret;
}

static int ltc4306_remove(struct i2c_client *client)
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);

	i2c_mux_del_adapters(muxc);
	return 0;
}

static struct i2c_driver ltc4306_driver = {
	.driver		= {
		.name	= "ltc4306",
		.of_match_table = of_match_ptr(ltc4306_of_match),
	},
	.probe		= ltc4306_probe,
	.remove		= ltc4306_remove,
	.id_table	= ltc4306_id,
};

module_i2c_driver(ltc4306_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC4306, LTC4305 I2C mux/switch driver");
MODULE_LICENSE("GPL v2");
