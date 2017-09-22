/*
 * Copyright 2017 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef LINUX_IIO_HW_CONSUMER_H
#define LINUX_IIO_HW_CONSUMER_H

struct device;
struct iio_hw_consumer;

struct iio_hw_consumer *iio_hw_consumer_alloc(struct device *dev);
void iio_hw_consumer_free(struct iio_hw_consumer *hwc);
int iio_hw_consumer_enable(struct iio_hw_consumer *hwc);
void iio_hw_consumer_disable(struct iio_hw_consumer *hwc);

#endif
