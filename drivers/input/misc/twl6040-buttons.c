/*
 * twl6040-buttons.c - TWL6040 Buttons POC driver
 *
 * Author:      Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * Copyright:   (C) 2012 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/mfd/twl6040.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c/twl6030-gpadc.h>

struct buttons_info {
	struct device *dev;
	struct input_dev *input_dev;
	struct mutex mutex;
	int irq;

	bool enabled;

	struct twl6040 *twl6040;
};

static irqreturn_t twl6040_hook_irq_handler(int irq, void *data)
{
	struct buttons_info *info = data;
	struct twl6030_gpadc_request req;
	int ret, channel;

	dev_err(info->dev, "%s\n", __func__);

	channel = 2;
	req.channels = (1 << channel);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	req.type = TWL6030_GPADC_WAIT;

	ret = twl6030_gpadc_conversion(&req);
	if (ret > 0) {
		dev_err(info->dev,
			"Got something (on channel %d): 0x%x (raw: 0x%x)\n",
			channel, req.rbuf[channel], req.buf[channel].raw_code);
	} else if (ret == 0) {
		dev_err(info->dev, "No data (on channel %d)\n", channel);
	} else {
		dev_err(info->dev, "Error (on channel %d): %d\n", channel, ret);
	}

	return IRQ_HANDLED;
}

static void twl6040_buttons_enable(struct buttons_info *info)
{
	twl6040_power(info->twl6040, 1);

	info->enabled = true;
}

static void twl6040_buttons_disable(struct buttons_info *info)
{
	twl6040_power(info->twl6040, 0);

	info->enabled = false;
}

static void twl6040_buttons_setup(struct buttons_info *info)
{
	struct twl6040 *twl6040 = info->twl6040;
	u8 val;

	twl6040_buttons_enable(info);

	/* Let's hack, shall we? */
	dev_err(info->dev, "%s: hacking the thing\n", __func__);
	/* AMICBCTL bit0, bit1*/
	val = 0x01 | 0x02;
	twl6040_set_bits(twl6040, TWL6040_REG_AMICBCTL, val);

	/*
	 * HKCTL1
	 */
	val = TWL6040_HKEN;
	val |= TWL6040_HKRATE(2);
	twl6040_set_bits(twl6040, TWL6040_REG_HKCTL1, val);

	/*
	 * HKCTL2
	 */
//	val = 0x01 | 0x02;
//	twl6040_set_bits(twl6040, TWL6040_REG_HKCTL2, val);

}

static void twl6040_buttons_close(struct input_dev *input)
{
	struct buttons_info *info = input_get_drvdata(input);

	mutex_lock(&info->mutex);

	if (info->enabled)
		twl6040_buttons_disable(info);

	mutex_unlock(&info->mutex);
}

#ifdef CONFIG_PM_SLEEP
static int twl6040_buttons_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct buttons_info *info = platform_get_drvdata(pdev);

	mutex_lock(&info->mutex);

	if (info->enabled)
		twl6040_buttons_disable(info);

	mutex_unlock(&info->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(twl6040_buttons_pm_ops, twl6040_buttons_suspend, NULL);

static int __devinit twl6040_buttons_probe(struct platform_device *pdev)
{
	struct buttons_info *info;
	int ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "couldn't allocate memory\n");
		return -ENOMEM;
	}

	info->dev = &pdev->dev;

	info->twl6040 = dev_get_drvdata(pdev->dev.parent);

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(info->dev, "invalid irq\n");
		ret = -EINVAL;
		goto err_kzalloc;
	}

	mutex_init(&info->mutex);

	info->input_dev = input_allocate_device();
	if (info->input_dev == NULL) {
		dev_err(info->dev, "couldn't allocate input device\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	input_set_drvdata(info->input_dev, info);

	info->input_dev->name = "twl6040:buttons";
	info->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	info->input_dev->keybit[BIT_WORD(KEY_1)] = BIT_MASK(KEY_1);
	info->input_dev->keybit[BIT_WORD(KEY_2)] = BIT_MASK(KEY_2);
	info->input_dev->keybit[BIT_WORD(KEY_3)] = BIT_MASK(KEY_3);
	info->input_dev->id.version = 1;
	info->input_dev->dev.parent = pdev->dev.parent;
	info->input_dev->close = twl6040_buttons_close;
	__set_bit(FF_RUMBLE, info->input_dev->ffbit);

	ret = input_register_device(info->input_dev);
	if (ret < 0) {
		dev_err(info->dev, "couldn't register input device\n");
		goto err_ialloc;
	}

	platform_set_drvdata(pdev, info);

	ret = request_threaded_irq(info->irq, NULL, twl6040_hook_irq_handler, 0,
				   "twl6040_irq_hook", info);
	if (ret) {
		dev_err(info->dev, "HOOK IRQ request failed: %d\n", ret);
		goto err_irq;
	}

	twl6040_buttons_setup(info);
	return 0;

err_irq:
	input_unregister_device(info->input_dev);
	info->input_dev = NULL;
err_ialloc:
	input_free_device(info->input_dev);
err_kzalloc:
	kfree(info);
	return ret;
}

static int __devexit twl6040_buttons_remove(struct platform_device *pdev)
{
	struct buttons_info *info = platform_get_drvdata(pdev);

	input_unregister_device(info->input_dev);
	free_irq(info->irq, info);
	kfree(info);

	return 0;
}

static struct platform_driver twl6040_buttons_driver = {
	.probe		= twl6040_buttons_probe,
	.remove		= __devexit_p(twl6040_buttons_remove),
	.driver		= {
		.name	= "twl6040-buttons",
		.owner	= THIS_MODULE,
		.pm	= &twl6040_buttons_pm_ops,
	},
};
module_platform_driver(twl6040_buttons_driver);

MODULE_ALIAS("platform:twl6040-buttons");
MODULE_DESCRIPTION("TWL6040 buttons driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
