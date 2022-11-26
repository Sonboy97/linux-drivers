// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define AP3216C_REGMAP_NAME	"ap3216c_regmap"
#define AP3216C_DRV_NAME	"ap3216c"


#define AP3216C_SYSTEMCONG	0x00	/* System Configuration */
#define AP3216C_INTSTATUS	0X01	/* Interrupt Status */
#define AP3216C_INTCLEAR	0X02	/* INT Clear Manner */
#define AP3216C_IRDATALOW	0x0A	/* IR Data Low */
#define AP3216C_IRDATAHIGH	0x0B	/* IR Data High */
#define AP3216C_ALSDATALOW	0x0C	/* ALS Data Low */
#define AP3216C_ALSDATAHIGH	0X0D	/* ALS Data High */
#define AP3216C_PSDATALOW	0X0E	/* PS Data Low */
#define AP3216C_PSDATAHIGH	0X0F	/* PS Data High */

#define AP3216C_ALSCONFIG	0X10	/* ALS Configuration */
#define AP3216C_PSCONFIG	0X20	/* PS Configuration */
#define AP3216C_PSLEDCONFIG 0X21	/* PS LED Driver */

enum ap3216c_idx {
	AP3216C_ALS,
	AP3216C_PS,
	AP3216C_IR,
};

struct ap3216c_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct mutex lock;
	struct regmap *regmap;
};

static const struct regmap_config ap3216c_regmap_config = {
	.name = AP3216C_REGMAP_NAME,
	.reg_bits = 8,
	.val_bits = 8,
};

static const unsigned long ap3216c_scan_masks[] = {
	BIT(AP3216C_ALS) | BIT(AP3216C_PS) | BIT(AP3216C_IR),
	0
};

static const struct iio_chan_spec ap3216c_channels[] = {
	/* ALS */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = AP3216C_ALS,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},

	/* PS */
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = AP3216C_PS,
		.scan_type = {
			.sign = 'u',
			.realbits = 10,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},

	/* IR */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = AP3216C_IR,
		.scan_type = {
			.sign = 'u',
			.realbits = 10,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
};

static int ap3216c_read_als(struct ap3216c_data *data, int *val)
{
	unsigned char reg[2];
	int ret;

	ret = regmap_bulk_read(data->regmap, AP3216C_ALSDATALOW, reg, 2);
	if (ret)
		return ret;

	*val = ((int)reg[1] << 8) | reg[0];

	return ret;
}

static int ap3216c_read_ps(struct ap3216c_data *data, int *val)
{
	unsigned char reg[2];
	int ret;

	ret = regmap_bulk_read(data->regmap, AP3216C_PSDATALOW, reg, 2);
	if (ret)
		return ret;

	*val = ((int)(reg[1] & 0X3F) << 4) | (reg[0] & 0X0F);

	return ret;
}

static int ap3216c_read_ir(struct ap3216c_data *data, int *val)
{
	unsigned char reg[2];
	int ret;

	ret = regmap_bulk_read(data->regmap, AP3216C_IRDATALOW, reg, 2);
	if (ret)
		return ret;

	*val = ((int)reg[1] << 2) | (reg[0] & 0X03);

	return ret;
}

#define AP3216C_SCALE(range)  (int)(((long long)range * 1000000) / 65535)
static const int ap3216c_als_scale[] = {AP3216C_SCALE(23360), AP3216C_SCALE(5840), 
										AP3216C_SCALE(1460), AP3216C_SCALE(365)};

static int ap3216c_read_als_scale(struct ap3216c_data *data, int *val)
{
	unsigned int reg;
	int ret;	

	ret = regmap_read(data->regmap, AP3216C_ALSCONFIG, &reg);
	if (ret)
		return ret;

	*val = ap3216c_als_scale[((reg & 0X30) >> 4)];

	return ret;
}

static int ap3216c_write_als_scale(struct ap3216c_data *data, int val)
{
	int i, ret = 0;	

	for (i=0; i<ARRAY_SIZE(ap3216c_als_scale); ++i)
	{
		if (ap3216c_als_scale[i] == val)
			break;
	}

	if(i == ARRAY_SIZE(ap3216c_als_scale))
	{
		ret = -EINVAL;
	}
	else
	{
		ret = regmap_write(data->regmap, AP3216C_ALSCONFIG, (unsigned char)(i<<4));
	}
		
	return ret;
}

static int ap3216c_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_INTENSITY:
			switch (chan->channel2) {
			case IIO_MOD_LIGHT_BOTH:
				mutex_lock(&data->lock);
				ret = ap3216c_read_als(data, val);
				mutex_unlock(&data->lock);
				if (ret)
					return ret;
				ret = IIO_VAL_INT;
				break;
			case IIO_MOD_LIGHT_IR:
				mutex_lock(&data->lock);
				ret = ap3216c_read_ir(data, val);
				mutex_unlock(&data->lock);
				if (ret)
					return ret;
				ret = IIO_VAL_INT;
				break;
			default:
				ret = -EINVAL;
				break;
			}
			break;
		case IIO_PROXIMITY:
			mutex_lock(&data->lock);
			ret = ap3216c_read_ps(data, val);
			mutex_unlock(&data->lock);
			if (ret)
				return ret;
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_INTENSITY:
			mutex_lock(&data->lock);
			ret = ap3216c_read_als_scale(data, val2);
			mutex_unlock(&data->lock);
			if (ret)
				return ret;
			*val = 0;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct iio_info ap3216c_info = {
	.read_raw = ap3216c_read_raw,
};

static int ap3216c_init(struct ap3216c_data *data)
{
	struct device *dev = &data->client->dev;
	int als, ps, ir;
	int ret;

	ret = regmap_write(data->regmap, AP3216C_SYSTEMCONG, 0x04); // reset ap3216c
	if (ret)
		return ret;
	
	mdelay(50); // >=10ms

	ret = regmap_write(data->regmap, AP3216C_SYSTEMCONG, 0x03); // ALS and PS+IR functions once
	if (ret)
		return ret;

	ret = ap3216c_write_als_scale(data, ap3216c_als_scale[0]); // 23360 lux / Resolution bit
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, AP3216C_PSLEDCONFIG, 0x13); // LED 1 pulse, LED driver ratio 100%
	if (ret)
		return ret;

	ap3216c_read_als(data, &als);
	ap3216c_read_ps(data, &ps);
	ap3216c_read_ir(data, &ir);
	dev_info(dev, "ALS=%d, PS=%d, IR=%d\n", als, ps, ir);

	return 0;
}

static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ap3216c_data *data;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name = AP3216C_DRV_NAME;
	indio_dev->info = &ap3216c_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ap3216c_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap3216c_channels);
	indio_dev->available_scan_masks = ap3216c_scan_masks;

	data = iio_priv(indio_dev);
	data->indio_dev = indio_dev;
	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, indio_dev);

	data->regmap = devm_regmap_init_i2c(client, &ap3216c_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(data->regmap);
	}

	ret = ap3216c_init(data);
	if (ret)
		return ret;

	return iio_device_register(indio_dev);
}

static int ap3216c_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id ap3216c_of_match[] = {
	{ .compatible = "lsc,ap3216c" },
	{ },
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static struct i2c_device_id ap3216c_id[] = {
	{ "ap3216c", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static struct i2c_driver ap3216c_driver = {
	.driver = {
        .owner = THIS_MODULE,
		.name	= "ap3216c",
		.of_match_table = of_match_ptr(ap3216c_of_match),
	},
	.probe	    = ap3216c_probe,
	.remove		= ap3216c_remove,
	.id_table	= ap3216c_id,
};

module_i2c_driver(ap3216c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sonboy");
MODULE_DESCRIPTION("Driver for AP3216C");
