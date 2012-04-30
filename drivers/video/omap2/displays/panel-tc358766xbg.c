/*
 * Toshiba TC358766XBG DSI-to-DP bridge
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#define USE_1280x720

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#include <video/omapdss.h>
#include <video/omap-panel-tc358766xbg.h>
#include <video/mipi_display.h>

#include "panel-tc358766xbg.h"

#define TC_ADDR 0x68

#define W(reg, val) \
	tc358766xbg_write_reg_dsi(dssdev, reg, val); \
	//printk("WRITE %20s (0x%04x) = 0x%08x\t%08x\n", # reg, reg, val, cpu_to_be32(val));

#define WI(reg, val) \
	tc358766xbg_write_reg_dsi(dssdev, reg, be32_to_cpu(val)); \
	//printk("WRITE %20s (0x%04x) = 0x%08x\n", # reg, reg, be32_to_cpu(val));
#define R(reg) \
	tc358766xbg_read_reg_dsi(dssdev, reg, &value); \
	printk("READ  %20s (0x%04x) = 0x%08x\n", # reg, reg, value);

static int start_link(struct omap_dss_device *dssdev);

#ifdef USE_1280x720
static const struct omap_video_timings tc358766xbg_timings = {
	.x_res		= 1280,
	.y_res		= 720,
	.pixel_clock    = 74250,
	.hfp            = 702,	/* 0 - 4095 */
	.hsw            = 8,	/* 0 - 255 */
	.hbp            = 2,	/* 0 - 4095 */
	.vfp            = 5,	/* 0 - 4095 */
	.vsw            = 5,	/* 0 - 255 */
	.vbp            = 20,	/* 0 - 4095 */
};

static const struct omap_dss_dsi_videomode_data vm_data = {
	.hsa			= 0,	/* 0 - 255 */
	.hfp			= 528,	/* 0 - 4095 */
	.hbp			= 3,	/* 0 - 4095 */
	.vsa			= 5,	/* 0 - 255 */
	.vfp			= 5,	/* 0 - 255 */
	.vbp			= 20,	/* 0 - 255 */

	.vp_de_pol		= true,
	.vp_vsync_pol		= true,	/* true = active high */
	.vp_hsync_pol		= true,	/* true = active high */
	.vp_hsync_end		= false,
	.vp_vsync_end		= false,

	.blanking_mode		= 1,
	.hsa_blanking_mode	= 1,
	.hfp_blanking_mode	= 1,
	.hbp_blanking_mode	= 1,

	.ddr_clk_always_on	= true,

	.window_sync		= 4,
};
#else
static const struct omap_video_timings tc358766xbg_timings = {
	.x_res		= 640,
	.y_res		= 480,
	.pixel_clock    = 25200,
	.hfp            = 166,	/* 0 - 4095 */
	.hsw            = 8,	/* 0 - 255 */
	.hbp            = 2,	/* 0 - 4095 */
	.vfp            = 10,	/* 0 - 4095 */
	.vsw            = 2,	/* 0 - 255 */
	.vbp            = 33,	/* 0 - 4095 */
};

static const struct omap_dss_dsi_videomode_data vm_data = {
	.hsa			= 0,	/* 0 - 255 */
	.hfp			= 1656,	/* 0 - 4095 */
	.hbp			= 3,	/* 0 - 4095 */
	.vsa			= 2,	/* 0 - 255 */
	.vfp			= 10,	/* 0 - 255 */
	.vbp			= 33,	/* 0 - 255 */

	.vp_de_pol		= true,
	.vp_vsync_pol		= true,	/* true = active high */
	.vp_hsync_pol		= true,	/* true = active high */
	.vp_hsync_end		= false,
	.vp_vsync_end		= false,

	.blanking_mode		= 1,
	.hsa_blanking_mode	= 1,
	.hfp_blanking_mode	= 1,
	.hbp_blanking_mode	= 1,

	.ddr_clk_always_on	= true,

	.window_sync		= 4,
};
#endif

/* device private data structure */
struct tc358766xbg_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int config_channel;
	int pixel_channel;

	struct i2c_adapter *adapter;
};

static struct tc358766xbg_board_data *get_board_data(struct omap_dss_device *dssdev)
{
	return (struct tc358766xbg_board_data *)dssdev->data;
}


static int tc358766xbg_read_dsi(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	return dsi_vc_generic_read_2(dssdev, d2d->config_channel, reg & 0x00ff,
		(reg & 0xff00) >> 8, buf, len);
}

static int tc358766xbg_read_reg_dsi(struct omap_dss_device *dssdev, u16 reg,
		u32 *value)
{
	u8 buf[4];
	int r;

	r = tc358766xbg_read_dsi(dssdev, reg, buf, 4);
	if (r)
		return r;

	*value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return 0;
}


#if 0
static int tc358766xbg_read_reg_dsi(struct omap_dss_device *dssdev, u16 reg,
		u32 *value)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[4];
	int r;

	r = dsi_vc_generic_read_2(dssdev, d2d->config_channel, reg & 0x00ff,
		(reg & 0xff00) >> 8, buf, 4);
	if (r < 0) {
		dev_err(&dssdev->dev, "gen read failed\n");
		return r;
	}

	*value = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);

	return 0;
}
#endif

static int tc358766xbg_write_dsi(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	int i;
	int padded_len = (len + 3) & ~3;

	u8 cmd[2 + 4*4] = {
		reg & 0xff,
		(reg >> 8) & 0xff,
	};

	for (i = 0; i < padded_len; ++i)
		cmd[2 + i] = buf[i];

	r = dsi_vc_generic_write_nosync(dssdev, d2d->config_channel, cmd,
			padded_len + 2);
	if (r)
		dev_err(&dssdev->dev, "reg write reg(%x) failed: %d\n",
			       reg, r);
	mdelay(1);

	return r;
}

static int tc358766xbg_write_reg_dsi(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	u8 buf[] = {
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};

	return tc358766xbg_write_dsi(dssdev, reg, buf, 4);
}

static int tc358766xbg_write_i2c(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	struct i2c_adapter *adapter = d2d->adapter;
	int r;
	int i;
	int padded_len = (len + 3) & ~3;

	u8 cmd[2 + 4*4] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	struct i2c_msg msgs[] = {
		{
			.addr   = TC_ADDR,
			.flags  = 0,
			.len    = padded_len + 2,
			.buf    = cmd,
		},
	};

	for (i = 0; i < padded_len; ++i)
		cmd[2 + i] = buf[i];


	r = i2c_transfer(adapter, msgs, 1);
	if (r != 1)
		return -EIO;

	return 0;
}

static int tc358766xbg_write_reg_i2c(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	u8 buf[] = {
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};

	return tc358766xbg_write_i2c(dssdev, reg, buf, 4);
}

static int tc358766xbg_read_i2c(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	struct i2c_adapter *adapter = d2d->adapter;
	int r;

	u8 cmd[2] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	struct i2c_msg msgs[] = {
		{
			.addr   = TC_ADDR,
			.flags  = 0,
			.len    = 2,
			.buf    = cmd,
		}, {
			.addr   = TC_ADDR,
			.flags  = I2C_M_RD,
			.len    = len,
			.buf    = buf,
		}
	};

	r = i2c_transfer(adapter, msgs, 2);
	if (r != 2)
		return -EIO;

	return 0;
}

static int tc358766xbg_read_reg_i2c(struct omap_dss_device *dssdev, u16 reg,
		u32 *value)
{
	u8 buf[4];
	int r;

	r = tc358766xbg_read_i2c(dssdev, reg, buf, 4);
	if (r)
		return r;

	*value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return 0;
}

static int tc358766xbg_write(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	return tc358766xbg_write_dsi(dssdev, reg, buf, len);
}

static int tc358766xbg_write_reg(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	return tc358766xbg_write_reg_dsi(dssdev, reg, value);
}

static int tc358766xbg_read(struct omap_dss_device *dssdev, u16 reg,
		u8 *buf, int len)
{
	return tc358766xbg_read_dsi(dssdev, reg, buf, len);
}

static int tc358766xbg_read_reg(struct omap_dss_device *dssdev, u16 reg,
		u32 *value)
{
	return tc358766xbg_read_reg_dsi(dssdev, reg, value);
}




static void tc358766xbg_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void tc358766xbg_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int tc358766xbg_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (tc358766xbg_timings.x_res != timings->x_res ||
			tc358766xbg_timings.y_res != timings->y_res ||
			tc358766xbg_timings.pixel_clock != timings->pixel_clock ||
			tc358766xbg_timings.hsw != timings->hsw ||
			tc358766xbg_timings.hfp != timings->hfp ||
			tc358766xbg_timings.hbp != timings->hbp ||
			tc358766xbg_timings.vsw != timings->vsw ||
			tc358766xbg_timings.vfp != timings->vfp ||
			tc358766xbg_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void tc358766xbg_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = tc358766xbg_timings.x_res;
	*yres = tc358766xbg_timings.y_res;
}

static int tc358766xbg_hw_reset(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_board_data *board_data = get_board_data(dssdev);

	if (board_data == NULL || board_data->reset_gpio == -1)
		return 0;

	gpio_set_value_cansleep(board_data->reset_gpio, 1);
	udelay(100);
	/* reset the panel */
	gpio_set_value_cansleep(board_data->reset_gpio, 0);
	/* assert reset */
	udelay(100);
	gpio_set_value_cansleep(board_data->reset_gpio, 1);

	/* wait after releasing reset */
	msleep(100);

	return 0;
}

static ssize_t taal_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u32 val;

	dsi_bus_lock(dssdev);

	dsi_disable_video_output(dssdev, d2d->pixel_channel);

	tc358766xbg_read_reg_dsi(dssdev, SYSCTRL, &val);

	dsi_enable_video_output(dssdev, d2d->pixel_channel);

	dsi_bus_unlock(dssdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}


static ssize_t wr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	u16 reg;
	u32 value;
	int r;
	char s1[64], s2[64];

	msleep(50);

	r = sscanf(buf, "%s %s", s1, s2);
	if (r != 2) {
		printk("sscanf failed: %d\n", r);
		return -EINVAL;
	}

	r = kstrtou16(s1, 0, &reg);
	if (r) {
		printk("parse reg failed\n");
		return r;
	}

	r = kstrtou32(s2, 0, &value);
	if (r) {
		printk("parse value failed\n");
		return r;
	}

	r = tc358766xbg_write_reg_i2c(dssdev, reg, value);
	if (r) {
		printk("i2c write failed: %d\n", r);
		return r;
	}

	printk("\nreg write 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t rd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	u16 reg;
	u32 value;
	int r;

	msleep(50);

	r = kstrtou16(buf, 0, &reg);
	if (r) {
		printk("parse failed\n");
		return r;
	}

	r = tc358766xbg_read_reg_i2c(dssdev, reg, &value);
	if (r) {
		printk("i2c read failed: %d\n", r);
		return r;
	}

	printk("\nreg read 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t dwr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u16 reg;
	u32 value;
	int r;
	char s1[64], s2[64];

	msleep(50);

	r = sscanf(buf, "%s %s", s1, s2);
	if (r != 2) {
		printk("sscanf failed: %d\n", r);
		return -EINVAL;
	}

	r = kstrtou16(s1, 0, &reg);
	if (r) {
		printk("parse reg failed\n");
		return r;
	}

	r = kstrtou32(s2, 0, &value);
	if (r) {
		printk("parse value failed\n");
		return r;
	}

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = tc358766xbg_write_reg_dsi(dssdev, reg, value);
	/*
	if (r) {
		printk("write failed\n");
	} else {
		r = dsi_vc_send_bta_sync(dssdev, d2d->config_channel);
		if (r)
			printk("bta failed\n");
	}
*/
	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	if (r) {
		printk("reg write failed\n");
		return r;
	}

	printk("\nreg write 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t drd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u16 reg;
	u32 value;
	int r;

	msleep(50);

	r = kstrtou16(buf, 0, &reg);
	if (r) {
		printk("parse failed\n");
		return r;
	}

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = tc358766xbg_read_reg_dsi(dssdev, reg, &value);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	if (r) {
		printk("dsi read failed: %d\n", r);
		return r;
	}

	printk("\nreg read 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t reg_dump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u32 value;
	int count = 0;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

#define DUMP(reg) \
	tc358766xbg_read_reg(dssdev, reg, &value); \
	count += sprintf(buf + count, "%20s = 0x%08x\n", # reg, value);

	DUMP(DSI_STARTDSI);
	DUMP(DSI_INTSTATUS);
	DUMP(VPCTRL0);
	DUMP(HTIM01);
	DUMP(HTIM02);
	DUMP(VTIM01);
	DUMP(VTIM02);
	DUMP(VFUEN0);
	DUMP(SYSCTRL);
	DUMP(SYSSTAT);
	DUMP(DP0Ctl);
	DUMP(DP0_VIDMNGEN0);
	DUMP(DP0_VIDMNGEN1);
	DUMP(DP0_VIDMNGENSTATUS);
	DUMP(DP0_VIDSYNCDELAY);
	DUMP(DP0_TOTALVAL);
	DUMP(DP0_STARTVAL);
	DUMP(DP0_ACTIVEVAL);
	DUMP(DP0_SYNCVAL);
	DUMP(DP0_MISC);
	DUMP(DP0_AUXCFG0);
	DUMP(DP0_AUXCFG1);
	DUMP(DP0_AUXADDR);
	DUMP(DP0_AUXWDATA(0));
	DUMP(DP0_AUXRDATA(0));
	DUMP(DP0_AUXRDATA(1));
	DUMP(DP0_AUXSTATUS);
	DUMP(DP0_AUXI2CADR);
	DUMP(DP0_SRCCTRL);
	DUMP(DP0_LTSTAT);
	DUMP(DP0_LTLOOPCTRL);
	DUMP(DP0_SNKLTCTRL);
	DUMP(DP_PHY_CTRL);
	DUMP(DP0_PLLCTRL);
	DUMP(DP1_PLLCTRL);
	DUMP(PXL_PLLCTRL);
	DUMP(PXL_PLLPARAM);
	DUMP(SYS_PLLPARAM);
	DUMP(D2DPTSTCTL);


	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	return count;
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, taal_num_errors_show, NULL);
static DEVICE_ATTR(wr, S_IWUSR, NULL, wr_store);
static DEVICE_ATTR(rd, S_IWUSR, NULL, rd_store);
static DEVICE_ATTR(dwr, S_IWUSR, NULL, dwr_store);
static DEVICE_ATTR(drd, S_IWUSR, NULL, drd_store);
static DEVICE_ATTR(reg_dump, S_IRUGO, reg_dump_show, NULL);

static struct attribute *taal_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_wr.attr,
	&dev_attr_rd.attr,
	&dev_attr_dwr.attr,
	&dev_attr_drd.attr,
	&dev_attr_reg_dump.attr,
	NULL,
};

static struct attribute_group taal_attr_group = {
	.attrs = taal_attrs,
};

static irqreturn_t taal_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u32 old_mask;
	u32 stat;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	dsi_disable_video_output(dssdev, d2d->pixel_channel);

	tc358766xbg_read_reg(dssdev, INTCTL_G, &old_mask);
	tc358766xbg_write_reg(dssdev, INTCTL_G, 0);

	tc358766xbg_read_reg(dssdev, INTSTS_G, &stat);

	printk("*** IRQ status 0x%x\n", stat);

	if (stat & (1 << 2)) {
		u32 value;

		tc358766xbg_read_reg(dssdev, GPIOI, &value);
		printk("GPIOI %x (L->H)\n", value);
		if (value & 1)
			start_link(dssdev);
	}

	if (stat & (1 << 3)) {
		u32 value;

		tc358766xbg_read_reg(dssdev, GPIOI, &value);
		printk("GPIOI %x (H->L)\n", value);
	}

	if (stat & (1 << 16)) {
		u32 value;

		tc358766xbg_read_reg(dssdev, SYSSTAT, &value);
		if (value != 0) {
			printk("SYSSTAT ERROR %x\n", value);
		}
	}

	tc358766xbg_write_reg(dssdev, INTSTS_G, stat);
	tc358766xbg_write_reg(dssdev, INTCTL_G, old_mask);

	dsi_enable_video_output(dssdev, d2d->pixel_channel);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	return IRQ_HANDLED;
}

static int tc358766xbg_probe(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_board_data *board_data = get_board_data(dssdev);
	struct tc358766xbg_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "tc358766xbg_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = tc358766xbg_timings;
	dssdev->panel.dsi_vm_data = vm_data;
	dssdev->panel.dsi_pix_fmt = OMAP_DSS_DSI_FMT_RGB888;

	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err0;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	if (board_data != NULL && board_data->reset_gpio != -1) {
		r = gpio_request_one(board_data->reset_gpio,
				GPIOF_OUT_INIT_HIGH, "tc358766xbg resx");

		if (r) {
			dev_err(&dssdev->dev, "failed to request gpio\n");
			goto err0;
		}
	}

	// XXX
	d2d->adapter = i2c_get_adapter(5);
	if (!d2d->adapter) {
		printk("can't get i2c adapter\n");
		return -EIO;
	}

#define INT_IRQ 171

	r = gpio_request_one(INT_IRQ, GPIOF_IN, "tc358766xbg int");
	if (r) {
		printk("request gpio int failed\n");
		return r;
	}

	r = request_threaded_irq(gpio_to_irq(INT_IRQ), NULL, taal_te_isr,
			IRQF_DISABLED | IRQF_TRIGGER_RISING,
			"tc int", dssdev);
	if (r) {
		printk("request irq failed\n");
		return r;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->pixel_channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel for"
			" transmitting pixel data\n");
		goto err0;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->pixel_channel, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID for pixel data"
			" virtual channel\n");
		goto err1;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->config_channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel for"
			"configuring bridge\n");
		goto err1;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->config_channel, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID for config"
			" channel\n");
		goto err2;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &taal_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		//goto err_vc_id;
	}

	dev_dbg(&dssdev->dev, "tc358766xbg_probe done\n");

	return 0;
err2:
	omap_dsi_release_vc(dssdev, d2d->config_channel);
err1:
	omap_dsi_release_vc(dssdev, d2d->pixel_channel);
err0:
	kfree(d2d);

	return r;
}

static void tc358766xbg_remove(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_board_data *board_data = get_board_data(dssdev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (board_data != NULL && board_data->reset_gpio != -1)
		gpio_free(board_data->reset_gpio);

	gpio_free(INT_IRQ);
	free_irq(gpio_to_irq(INT_IRQ), dssdev);

	sysfs_remove_group(&dssdev->dev.kobj, &taal_attr_group);
	omap_dsi_release_vc(dssdev, d2d->pixel_channel);
	omap_dsi_release_vc(dssdev, d2d->config_channel);

	kfree(d2d);
}

static void read_lt_status(struct omap_dss_device *dssdev)
{
	u32 value;

	tc358766xbg_read_reg(dssdev, DP0_LTSTAT, &value);

	printk("\tLT status %x, ", (value >> 8) & 0x1f);
	if ((value & (1 << 0)) != 0) printk("CRDone0 ");
	if ((value & (1 << 1)) != 0) printk("EQDone0 ");
	if ((value & (1 << 2)) != 0) printk("SymLck0 ");
	if ((value & (1 << 3)) != 0) printk("ILAlign ");
	if ((value & (1 << 4)) != 0) printk("CRDone1 ");
	if ((value & (1 << 5)) != 0) printk("EQDone1 ");
	if ((value & (1 << 6)) != 0) printk("SymLck1 ");
	if ((value & (1 << 13)) != 0) printk("LoopDone ");
	printk("\n");
}

struct auxstatus
{
	bool busy:1;
	bool timeout:1;
	unsigned reserved1:2;
	unsigned status:4;
	unsigned bytes:8;
	bool err_sync:1;
	bool err_stop:1;
	bool err_align:1;
	unsigned reserved2:1;
	unsigned retry:3;
	unsigned reserved3:9;
} __attribute__ ((__packed__));

static int aux_nat_read(struct omap_dss_device *dssdev, u32 addr, u8 *buf,
		int len)
{
	const u32 native_read_cmd = 0x9;
	int r;
	int retries = 10;
	struct auxstatus auxstat;
	int n;

	r = tc358766xbg_write_reg(dssdev, DP0_AUXADDR, addr);
	r = tc358766xbg_write_reg(dssdev, DP0_AUXCFG0,
			((len - 1) << 8) | native_read_cmd);

	while (--retries > 0) {
		mdelay(1);

		r = tc358766xbg_read(dssdev, DP0_AUXSTATUS,
				(u8*)&auxstat, 4);

		if (auxstat.busy == false)
			break;
	}

	if (retries == 0) {
		printk("failed\n");
		return -EIO;
	}

	if (auxstat.timeout) {
		printk("timeout\n");
		return -EIO;
	}

	if (auxstat.err_sync || auxstat.err_stop || auxstat.err_align) {
		printk("AUX error on read: ");

		if (auxstat.err_sync)
			printk("ErrSync ");
		if (auxstat.err_stop)
			printk("ErrStop ");
		if (auxstat.err_align)
			printk("ErrAlign ");

		printk("\n");

		return -EIO;
	}

	if (auxstat.status != 0) {
		printk("bad aux status %x\n", auxstat.status);
		return -EIO;
	}

	if (len != auxstat.bytes) {
		printk("bad read len %d\n", auxstat.bytes);
		return -EIO;
	}

	for (n = 0; n < (len + 3) / 4; ++n)
	{
		r = tc358766xbg_read(dssdev, DP0_AUXRDATA(n),
				buf + n * 4, 4); //min(4, len - n * 4));
	}

	return 0;
}

static int aux_nat_write(struct omap_dss_device *dssdev, u32 addr, u8 *buf,
		int len)
{
	const u32 native_write_cmd = 0x8;
	int r;
	int retries = 10;
	struct auxstatus auxstat;
	int n;
	int i;

	printk("AUX WRITE 0x%04x = ", addr);

	for (i = 0; i < len; ++i)
		printk("%02x ", buf[i]);

	printk("\n");

	r = tc358766xbg_write_reg(dssdev, DP0_AUXADDR, addr);

	for (n = 0; n < (len + 3) / 4; ++n)
	{
		r = tc358766xbg_write(dssdev, DP0_AUXWDATA(n),
				buf + n * 4, min(4, len - n * 4));
	}

	r = tc358766xbg_write_reg(dssdev, DP0_AUXCFG0,
			((len - 1) << 8) | native_write_cmd);

	while (--retries > 0) {
		mdelay(1);

		r = tc358766xbg_read(dssdev, DP0_AUXSTATUS,
				(u8*)&auxstat, 4);

		if (auxstat.busy == false)
			break;
	}

	if (retries == 0) {
		printk("failed\n");
		return -EIO;
	}

	if (auxstat.timeout) {
		printk("timeout\n");
		return -EIO;
	}

	if (auxstat.err_sync || auxstat.err_stop || auxstat.err_align) {
		printk("AUX error on write: ");

		if (auxstat.err_sync)
			printk("ErrSync ");
		if (auxstat.err_stop)
			printk("ErrStop ");
		if (auxstat.err_align)
			printk("ErrAlign ");

		printk("\n");

		return -EIO;
	}

	if (auxstat.status != 0) {
		printk("bad aux status %x\n", auxstat.status);
		return -EIO;
	}

	if (len != auxstat.bytes) {
		printk("bad write len %d\n", auxstat.bytes);
		return -EIO;
	}

	return 0;
}

static int aux_write_1(struct omap_dss_device *dssdev, u32 addr, u8 b1)
{
	u8 buf[] = { b1 };

	return aux_nat_write(dssdev, addr, buf, 1);
}

static int aux_write_2(struct omap_dss_device *dssdev, u32 addr, u8 b1, u8 b2)
{
	u8 buf[] = { b1, b2 };

	return aux_nat_write(dssdev, addr, buf, 2);
}

static void aux_dump(struct omap_dss_device *dssdev, u32 addr, int len)
{
	u8 buf[4*4];
	int i;
	int r;

	r = aux_nat_read(dssdev, addr, buf, len);

	if (r) {
		printk("AUX READ  0x%04x FAILED\n", addr);
		return;
	}

	printk("AUX READ  0x%04x = ", addr);

	for (i = 0; i < len; ++i)
		printk("%02x ", buf[i]);

	printk("\n");
}

static int do_link_training(struct omap_dss_device *dssdev)
{
	u32 value;

	W(DP0_AUXCFG1,	0x32 |	// timer
			(0x7 << 8) |  // threshold
			(1 << 16)); // filter enable

	// MAX_LINK_RATE, 0x6 = 1.62 Gbps
	aux_dump(dssdev, 0x1, 1);

	// MAX_LANE_COUNT (0x84, 4 lanes)
	aux_dump(dssdev, 0x2, 1);

	// LINK_BW_SET & LANE_COUNT_SET
	aux_write_2(dssdev, 0x100, 0x6, 0x2); // 1.62Gbps, two lane

	aux_dump(dssdev, 0x100, 2);

	// MAIN_LINK_CHANNEL_CODING_SET
	aux_write_1(dssdev, 0x108, 0x1); // SET_ANSI 8B10B

	aux_dump(dssdev, 0x108, 1);


	// clear LT irq
	tc358766xbg_write_reg(dssdev, INTSTS_G, (1 << 1));


	W(DP0_LTLOOPCTRL,
			(0xd << 0) | // Loop timer delay
			(0x6 << 24) | // LoopIter
			(0xf << 28)); // DeferIter

	// Data to be written to Sink LT Control register at address 0x00102
	W(DP0_SNKLTCTRL, 0x21);
	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(1 << 8) | // training pattern 1
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	W(DP0Ctl, 0x1); // dp_en


	{
		int i;

		for (i = 0; i < 10; ++i) {
			tc358766xbg_read_reg(dssdev, INTSTS_G, &value);
			if (value & (1 << 1)) // LT0
				break;
			mdelay(1);
		}

		if (i == 10)
			printk("LT timeout 1\n");
	}


	R(DP0_LTSTAT);
	read_lt_status(dssdev);

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);


	// clear LT irq
	tc358766xbg_write_reg(dssdev, INTSTS_G, (1 << 1));

	// Data to be written to Sink LT Control register at address 0x00102
	W(DP0_SNKLTCTRL, 0x22);
	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(2 << 8) | // training pattern 2
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL


	{
		int i;

		for (i = 0; i < 10; ++i) {
			tc358766xbg_read_reg(dssdev, INTSTS_G, &value);
			if (value & (1 << 1)) // LT0
				break;
			mdelay(1);
		}

		if (i == 10)
			printk("LT timeout 2\n");
	}

	// clear LT irq
	tc358766xbg_write_reg(dssdev, INTSTS_G, (1 << 1));


	R(DP0_LTSTAT);
	read_lt_status(dssdev);

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);

	// TRAINING_PATTERN_SET
	aux_write_1(dssdev, 0x102, 0);

	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(1 << 12)); // EN810B

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);
	printk("        Expected = 41 00 77 00 01\n");

	return 0;
}

static int dsi_config(struct omap_dss_device *dssdev)
{
	printk("dsi config\n");

	W(PPI_TX_RX_TA, (4 << 16) | 4); // TXTAGOCNT & TXTASURECNT
	W(PPI_LPTXCNT, 4);	// LPTXTIMECNT

	W(PPI_D0S_CLRSIPOCOUNT, 7);
	W(PPI_D1S_CLRSIPOCOUNT, 7);
	W(PPI_D2S_CLRSIPOCOUNT, 7);
	W(PPI_D3S_CLRSIPOCOUNT, 7);

	W(PPI_LANEENABLE, 0x1f);	// PPI enable 4 + 1 lanes

	W(DSI_LANEENABLE, 0x1f);	// DSI enable 4 + 1 lanes

	W(PPI_STARTPPI, 1);
	W(DSI_STARTDSI, 1);

	return 0;
}

static int main_config(struct omap_dss_device *dssdev)
{
	printk("main config\n");

	W(SYS_PLLPARAM,	(1 << 0) | // LS_CLK_DIV = 2
			(0 << 4) | // use LS CLK
			(1 << 8)); // RefClk freq = 19.2

	W(DP0_PLLCTRL,	(1 << 0) |	// Enable PLL
			(1 << 2));	// PLL UPDATE

	msleep(100);

	return 0;
}

struct dp_timings
{
	unsigned x_res, hfp, hsw, hbp;
	unsigned y_res, vfp, vsw, vbp;
	bool hs_pol, vs_pol;
};

static struct dp_timings dp_timings_1280x720 =
{
	.x_res = 1280,
	.hfp = 110,
	.hsw = 40,
	.hbp = 220,
	.hs_pol = true,

	.y_res = 720,
	.vfp = 5,
	.vsw = 5,
	.vbp = 20,
	.vs_pol = true,
};

static struct dp_timings dp_timings_640x480 =
{
	.x_res = 640,
	.hfp = 16,
	.hsw = 96,
	.hbp = 48,
	.hs_pol = true,

	.y_res = 480,
	.vfp = 10,
	.vsw = 2,
	.vbp = 33,
	.vs_pol = true,
};


static int config_dp0_timings(struct omap_dss_device *dssdev,
		struct dp_timings *t)
{
	unsigned htot, hstart;
	unsigned vtot, vstart;
	unsigned thresh_dly, vid_sync_dly;

	htot = t->x_res + t->hfp + t->hsw + t->hbp;
	hstart = t->hsw + t->hbp;

	vtot = t->y_res + t->vfp + t->vsw + t->vbp;
	vstart = t->vsw + t->vbp;

	thresh_dly = 31;
	vid_sync_dly = t->hsw + t->hbp + t->x_res;

	W(DP0_VIDSYNCDELAY, (thresh_dly << 16) | vid_sync_dly);
	W(DP0_TOTALVAL, (vtot << 16) | htot);
	W(DP0_STARTVAL, (vstart << 16) | hstart);
	W(DP0_ACTIVEVAL, (t->y_res << 16) | t->x_res);
	W(DP0_SYNCVAL,	((t->vs_pol ? 1 : 0) << 31) | (t->vsw << 16) |
			((t->hs_pol ? 1 : 0) << 15) | t->hsw);

	return 0;
}

static int config_vtgen(struct omap_dss_device *dssdev,
		struct dp_timings *t)
{
	W(HTIM01, (t->hbp << 16) | t->hsw);
	W(HTIM02, (t->hfp << 16) | t->x_res);
	W(VTIM01, (t->vbp << 16) | t->vsw);
	W(VTIM02, (t->vfp << 16) | t->y_res);
	W(VFUEN0, 1); // VFUEN

	return 0;
}

static unsigned long dp_pclk; // XXX

struct dp_divs
{
	unsigned fbd;
	unsigned pre_div;
	unsigned ext_pre_div;
	unsigned ext_post_div;
};

static struct dp_divs dp_divs_1280x720 = {
	.fbd = 116,
	.pre_div = 15,
	.ext_post_div = 1,
	.ext_pre_div = 7,
};

static struct dp_divs dp_divs_640x480 = {
	.fbd = 3,
	.pre_div = 8,
	.ext_post_div = 1,
	.ext_pre_div = 1,
};

static int config_pxl_pll(struct omap_dss_device *dssdev)
{
	unsigned long refclk = 67200000; // HSCLK / 4
	struct dp_divs *d;
	unsigned long vco;
	u32 v;

#ifdef USE_1280x720
	d = &dp_divs_1280x720;
#else
	d = &dp_divs_640x480;
#endif

	vco = (refclk / 1000) * d->fbd / d->ext_pre_div / d->pre_div;
	vco *= 1000;

	dp_pclk = vco / d->ext_post_div;

	printk("DP PCLK %lu Hz\n", dp_pclk);

	v = 0;
	v |= d->fbd == 128 ? 0 : d->fbd;
	v |= (d->pre_div == 16 ? 0 : d->pre_div) << 8;
	v |= 3 << 14;			// IN_SEL = HSCKBY4
	v |= d->ext_post_div << 16;
	v |= d->ext_pre_div << 20;
	v |= (vco >= 300 ? 1 : 0) << 24;

	W(PXL_PLLPARAM,	v);

	W(PXL_PLLCTRL,	(1 << 0) |	// Enable PLL
			(1 << 2));	// PLL UPDATE
	msleep(100);

	return 0;
}

static unsigned calc_vsdelay(struct omap_dss_device *dssdev,
		struct dp_timings *dpt)
{
	unsigned long byteclk = 67200000;
	//struct omap_video_timings *t = &dssdev->panel.timings;
	struct omap_dss_dsi_videomode_data *dt = &dssdev->panel.dsi_vm_data;
	unsigned bytespp = 3;
	unsigned datalanes = 4;

	unsigned d_hact, d_h;
	unsigned h;
	unsigned vsdelay;

	d_hact = dpt->x_res * bytespp / datalanes;
	d_h = d_hact + dt->hsa + dt->hbp + dt->hfp;

	h = dpt->x_res + dpt->hsw + dpt->hbp;

	vsdelay = (dp_pclk / 1000) * d_h / (byteclk / 1000);

	 if (vsdelay < h)
		 vsdelay = 0;
	 else
		 vsdelay -= h;

	printk("VSDELAY %u\n", vsdelay);

	return vsdelay;
}

static int start_link(struct omap_dss_device *dssdev)
{
	u32 value;
	static bool started;
	struct dp_timings *dpt;

#ifdef USE_1280x720
	dpt = &dp_timings_1280x720;
#else
	dpt = &dp_timings_640x480;
#endif

	if (started)
		return 0;

	started = true;

	// DP PLL input = 268.8/2/7 = 19.2MHz

	W(DP0_SRCCTRL,	(1 << 7) | // lane skew
			(0 << 1) | // 1.62 Gbps
			(1 << 2) | // two main channel lanes
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	//main_config(dssdev);

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25)); // AUX PHY BGR Enable

	msleep(100);

	config_pxl_pll(dssdev);


	R(DP_PHY_CTRL);

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25) | // AUX PHY BGR Enable
			(1 << 8)); // PHY Main Channel0 Reset

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25)); // AUX PHY BGR Enable

	msleep(100);
	R(DP_PHY_CTRL);


	do_link_training(dssdev);


	msleep(100);

	W(D2DPTSTCTL,	2 | // color bar
			(1 << 4) | // i2c filter
			(0x63 << 8) |	// B
			(0x14 << 16) |	// G
			(0x78 << 24));	// R



	W(VPCTRL0,	(calc_vsdelay(dssdev, dpt) << 20) | // VSDELAY)
			(1 << 8) |	// RGB666/RGB888
			(1 << 4)); // enable/disable VTGen


	config_vtgen(dssdev, dpt);
	config_dp0_timings(dssdev, dpt);

	W(DP0_MISC,	(0 << 0) | // sync_m
			(1 << 5) | // 8 bits per color component RGB888
			(0x3f << 16) |	// tu_size
			(0x3d << 24));	// max_tu_symbol


	msleep(100);
	W(DP0Ctl,	(1 << 0) | // dp_en
			(1 << 6)); // vid_mn_gen
	W(DP0Ctl,	(1 << 0) | // dp_en
			(1 << 1) | // vid_en
			(1 << 6)); // vid_mn_gen

	//W(SYSCTRL, 3);	// DP0_VidSrc = Color bar
	W(SYSCTRL, 1);		// DP0_VidSrc = DSI

	return 0;
}

static int tc358766xbg_power_on(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u32 value;

	dev_dbg(&dssdev->dev, "power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	/* reset tc358766xbg bridge */
	tc358766xbg_hw_reset(dssdev);

	omapdss_dsi_vc_enable_hs(dssdev, d2d->pixel_channel, true);



	main_config(dssdev);

	dsi_config(dssdev);


	// XXX this makes the max return data size 8...?
	//dsi_vc_set_max_rx_packet_size(dssdev, d2d->config_channel, 7);

	/*
	msleep(50);
	R(0x66c);
	tc358766xbg_write_reg_dsi(dssdev, 0x66c, 0x12345678);
	msleep(50);
	R(0x66c);
	*/

	value = 0;
	tc358766xbg_read_reg_dsi(dssdev, IDREG, &value);
	printk("%x\n", value);
	tc358766xbg_read_reg_dsi(dssdev, IDREG, &value);
	printk("%x\n", value);


	tc358766xbg_read_reg(dssdev, SYSSTAT, &value);
	if (value != 0)
		printk("SYSSTAT ERROR %x\n", value);


	/* GPIOs */
	//W(INTGP0LCNT, 0x2); // GPIO0 irq low count
	W(GPIOM, 0);	// pins are GPIOs
	W(GPIOC, 0);	// GPIOs are inputs
	W(INTCTL_G, 0);
	W(INTSTS_G, 0xffffffff);
	W(INTCTL_G,	(1 << 16) | // SYS ERR
			(1 << 2) | (1 << 3)); // enable GPIO interrupts


	W(DP0_AUXI2CADR, 0xA5);

	R(IDREG);

	tc358766xbg_read_reg(dssdev, GPIOI, &value);
	printk("GPIOI %x\n", value);
	/*
	if (value & 1)
		start_link(dssdev);
*/



	omapdss_dsi_vc_enable_hs(dssdev, d2d->config_channel, true);


	//tc358766xbg_read_reg_dsi(dssdev, D0W_DPHYCONTTX);
	//tc358766xbg_read_reg_dsi(dssdev, D0W_DPHYCONTTX);

#if 0
	/* configure D2L chip DSI-RX configuration registers */
	r = tc358766xbg_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	tc358766xbg_read_reg_dsi(dssdev, IDREG);
	omapdss_dsi_vc_enable_hs(dssdev, d2d->config_channel, true);
	tc358766xbg_read_reg_dsi(dssdev, IDREG);
	tc358766xbg_read_reg_dsi(dssdev, SYSSTAT);

	dsi_vc_send_bta_sync(dssdev, d2d->config_channel);
	dsi_vc_send_bta_sync(dssdev, d2d->config_channel);
	dsi_vc_send_bta_sync(dssdev, d2d->config_channel);

	tc358766xbg_read_reg_dsi(dssdev, IDREG);
	tc358766xbg_read_reg_dsi(dssdev, SYSSTAT);
#endif
	dsi_enable_video_output(dssdev, d2d->pixel_channel);



	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

//err_write_init:
//	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void tc358766xbg_power_off(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	dsi_disable_video_output(dssdev, d2d->pixel_channel);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void tc358766xbg_disable(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		tc358766xbg_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tc358766xbg_enable(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = tc358766xbg_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static struct omap_dss_driver tc358766xbg_driver = {
	.probe		= tc358766xbg_probe,
	.remove		= tc358766xbg_remove,

	.enable		= tc358766xbg_enable,
	.disable	= tc358766xbg_disable,

	.get_resolution	= tc358766xbg_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= tc358766xbg_get_timings,
	.set_timings	= tc358766xbg_set_timings,
	.check_timings	= tc358766xbg_check_timings,

	.driver         = {
		.name   = "tc358766xbg",
		.owner  = THIS_MODULE,
	},
};

static int __init tc358766xbg_init(void)
{
	omap_dss_register_driver(&tc358766xbg_driver);
	return 0;
}

static void __exit tc358766xbg_exit(void)
{
	omap_dss_unregister_driver(&tc358766xbg_driver);
}

module_init(tc358766xbg_init);
module_exit(tc358766xbg_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TC358766XBG DSI-2-LVDS Driver");
MODULE_LICENSE("GPL");
