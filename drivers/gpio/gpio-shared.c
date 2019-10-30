// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <dt-bindings/gpio/gpio.h>

enum gpio_shared_mode {
	GPIO_SHARED_AND = 0,
	GPIO_SHARED_OR,
};

struct gpio_client {
	unsigned requested:1;
	int value;
};

struct gpio_shared_priv {
	struct device *dev;
	struct gpio_desc *root_gpio;

	struct gpio_chip gpio_chip;
	enum gpio_shared_mode share_mode;
	int root_value;

	struct mutex mutex; /* protecting the counters */
	int high_count;
	int low_count;

	/* root gpio calbacks */
	int (*root_get)(const struct gpio_desc *desc);
	void (*root_set)(struct gpio_desc *desc, int value);

	struct gpio_client *clients;
};

static int gpio_shared_aggregate_root_value(struct gpio_shared_priv *priv)
{
	int value = 0;
	int i;

	for (i = 0; i < priv->gpio_chip.ngpio; i++) {
		if (priv->clients[i].requested) {
			if (priv->share_mode == GPIO_SHARED_AND)
				value &= priv->clients[i].value;
			else
				value |= priv->clients[i].value;
		}
	}

	return value;
}

static int gpio_shared_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	struct gpio_shared_priv *priv = gpiochip_get_data(chip);
	int ret = 0;

	if (priv->clients[offset].requested) {
		ret = -EBUSY;
		goto out;
	}

	mutex_lock(&priv->mutex);
	priv->clients[offset].requested = 1;
	priv->clients[offset].value = priv->root_value;

out:
	mutex_unlock(&priv->mutex);
	return ret;
}

static void gpio_shared_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	struct gpio_shared_priv *priv = gpiochip_get_data(chip);

	priv->clients[offset].requested = 0;
}

static void gpio_shared_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct gpio_shared_priv *priv = gpiochip_get_data(chip);
	int root_value;

	mutex_lock(&priv->mutex);
	priv->clients[offset].value = value;

	root_value = gpio_shared_aggregate_root_value(priv);
	if (priv->root_value != root_value) {
		priv->root_set(priv->root_gpio, root_value);

		/* Update the root's and client's value for the change */
		priv->root_value = root_value;
	}
	mutex_unlock(&priv->mutex);
}

static int gpio_shared_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct gpio_shared_priv *priv = gpiochip_get_data(chip);
	int value;

	mutex_lock(&priv->mutex);
	value = priv->clients[offset].value;
	mutex_unlock(&priv->mutex);

	return value;
}

static int gpio_shared_gpio_direction_out(struct gpio_chip *chip,
					  unsigned int offset, int value)
{
	gpio_shared_gpio_set(chip, offset, value);

	return 0;
}

static const struct gpio_chip gpio_shared_template_chip = {
	.owner			= THIS_MODULE,
	.request		= gpio_shared_gpio_request,
	.free			= gpio_shared_gpio_free,
	.set			= gpio_shared_gpio_set,
	.get			= gpio_shared_gpio_get,
	.direction_output	= gpio_shared_gpio_direction_out,
	.base			= -1,
};

static const struct of_device_id gpio_shared_of_match[] = {
	{ .compatible = "gpio-shared", },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_shared_of_match);

static int gpio_shared_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_shared_priv *priv;
	u32 val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->gpio_chip = gpio_shared_template_chip;
	priv->gpio_chip.label = dev_name(dev);
	priv->gpio_chip.parent = dev;
	priv->gpio_chip.of_node = dev->of_node;

	ret = of_property_read_u32(dev->of_node, "branch-count", &val);
	if (ret) {
		dev_err(dev, "branch-count is not provided\n");
		return ret;
	}

	priv->gpio_chip.ngpio = val;

	priv->clients = devm_kcalloc(dev, priv->gpio_chip.ngpio,
				     sizeof(*priv->clients), GFP_KERNEL);
	if (!priv->clients)
		return -ENOMEM;

	priv->root_gpio = devm_gpiod_get(dev, "root", GPIOD_ASIS);
	if (IS_ERR(priv->root_gpio)) {
		ret = PTR_ERR(priv->root_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get root GPIO\n");

		return ret;
	}

	/* If the root GPIO is input, change it to output */
	if (gpiod_get_direction(priv->root_gpio))
		gpiod_direction_output(priv->root_gpio, 0);

	priv->gpio_chip.can_sleep = gpiod_cansleep(priv->root_gpio);
	if (priv->gpio_chip.can_sleep) {
		priv->root_get = gpiod_get_value_cansleep;
		priv->root_set = gpiod_set_value_cansleep;
	} else {
		priv->root_get = gpiod_get_value;
		priv->root_set = gpiod_set_value;
	}

	priv->root_value = priv->root_get(priv->root_gpio);

	ret = of_property_read_u32(dev->of_node, "hold-active-state", &val);
	if (ret)
		val = GPIO_ACTIVE_LOW;

	if (val == GPIO_ACTIVE_HIGH)
		priv->share_mode = GPIO_SHARED_OR;

	dev_set_drvdata(dev, priv);
	mutex_init(&priv->mutex);

	return devm_gpiochip_add_data(dev, &priv->gpio_chip, priv);
}

static int gpio_shared_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver gpio_shared_driver = {
	.driver = {
		.name	= "gpio-shared",
		.of_match_table = gpio_shared_of_match,
	},
	.probe		= gpio_shared_probe,
	.remove		= gpio_shared_remove,
};

module_platform_driver(gpio_shared_driver);

MODULE_ALIAS("platform:gpio-shared");
MODULE_DESCRIPTION("Generic shared GPIO driver");
MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_LICENSE("GPL v2");
