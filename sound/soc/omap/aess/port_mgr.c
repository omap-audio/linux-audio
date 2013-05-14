/*
 * ALSA SoC OMAP ABE port manager
 *
 * Author: Liam Girdwood <lrg@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/device.h>

#include "omap-aess-priv.h"
#include "abe_port.h"

/* ports can either be enabled or disabled */
enum port_state {
	PORT_DISABLED = 0,
	PORT_ENABLED,
};

/* structure used for client port info */
struct omap_abe_port {

	/* logical and physical port IDs that correspond this port */
	int logical_id;
	int physical_id;

	/* enabled or disabled */
	enum port_state state;

	/* logical port ref count */
	int users;

	struct list_head list;
	struct omap_aess *aess;
	struct snd_pcm_substream *substream;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_lstate;
	struct dentry *debugfs_lphy;
	struct dentry *debugfs_lusers;
#endif
};

/* this must match logical ID numbers in port_mgr.h */
static const char *lport_name[] = {
		"dmic0", "dmic1", "dmic2", "pdmdl1", "pdmdl2", "mcasp",
		"pdmul1", "bt_vx_dl", "bt_vx_ul", "mm_ext_ul", "mm_ext_dl",
		"mm_dl1", "mm_ul1", "mm_ul2", "vx_dl", "vx_ul", "tones",
		"mm_dl_lp"
};

static DEFINE_MUTEX(port_mgr_mutex);
static struct omap_aess *the_aess = NULL;
static int users = 0;

/*
 * Get the Physical port ID based on the logical port ID
 *
 * FE and BE ports have unique ID's within the driver but share
 * ID's within the ABE. This maps a driver port ID to an ABE port ID.
 */
static int get_physical_id(int logical_id)
{
	switch (logical_id) {
	/* backend ports */
	case OMAP_ABE_BE_PORT_DMIC0:
	case OMAP_ABE_BE_PORT_DMIC1:
	case OMAP_ABE_BE_PORT_DMIC2:
		return OMAP_ABE_DMIC_PORT;
	case OMAP_ABE_BE_PORT_PDM_DL1:
	case OMAP_ABE_BE_PORT_PDM_DL2:
		return OMAP_ABE_PDM_DL_PORT;
	case OMAP_ABE_BE_PORT_MCASP:
		return OMAP_ABE_MCASP_DL_PORT;
	case OMAP_ABE_BE_PORT_PDM_UL1:
		return OMAP_ABE_PDM_UL_PORT;
	case OMAP_ABE_BE_PORT_BT_VX_DL:
		return OMAP_ABE_BT_VX_DL_PORT;
	case OMAP_ABE_BE_PORT_BT_VX_UL:
		return OMAP_ABE_BT_VX_UL_PORT;
	case OMAP_ABE_BE_PORT_MM_EXT_DL:
		return OMAP_ABE_MM_EXT_OUT_PORT;
	case OMAP_ABE_BE_PORT_MM_EXT_UL:
		return OMAP_ABE_MM_EXT_IN_PORT;
	/* front end ports */
	case OMAP_ABE_FE_PORT_MM_DL1:
	case OMAP_ABE_FE_PORT_MM_DL_LP:
		return OMAP_ABE_MM_DL_PORT;
	case OMAP_ABE_FE_PORT_MM_UL1:
		return OMAP_ABE_MM_UL_PORT;
	case OMAP_ABE_FE_PORT_MM_UL2:
		return OMAP_ABE_MM_UL2_PORT;
	case OMAP_ABE_FE_PORT_VX_DL:
		return OMAP_ABE_VX_DL_PORT;
	case OMAP_ABE_FE_PORT_VX_UL:
		return OMAP_ABE_VX_UL_PORT;
	case OMAP_ABE_FE_PORT_TONES:
		return OMAP_ABE_TONES_DL_PORT;
	}
	return -EINVAL;
}

static struct omap_abe_port *find_logical_port(struct omap_aess *aess,
					       int logical_id)
{
	struct omap_abe_port *p;

	list_for_each_entry(p, &aess->ports, list) {
		if (p->logical_id == logical_id)
			return p;
	}
	return NULL;
}

static int port_is_open(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *p = find_logical_port(aess, logical_id);

	if (p && p->state == PORT_ENABLED)
		return 1;

	return 0;
}

/*
 * Get the number of enabled users of the physical port shared by this client.
 * Locks held by callers.
 */
static int port_get_num_users(struct omap_aess *aess,
			      struct omap_abe_port *port)
{
	struct omap_abe_port *p;
	int users = 0;

	list_for_each_entry(p, &aess->ports, list) {
		if (p->physical_id == port->physical_id &&
		    p->state == PORT_ENABLED)
			users++;
	}
	return users;
}

/*
 * Check whether the physical port is enabled for this PHY port ID.
 * Locks held by callers.
 */
int omap_abe_port_is_enabled(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);
	struct omap_abe_port *p;
	unsigned long flags;

	if (!port) {
		dev_err(aess->dev, "Port %s is not open.\n",
			lport_name[logical_id]);
		return 0;
	}

	spin_lock_irqsave(&aess->lock, flags);

	list_for_each_entry(p, &aess->ports, list) {
		if (p->physical_id == port->physical_id && p->state == PORT_ENABLED) {
			spin_unlock_irqrestore(&aess->lock, flags);
			return 1;
		}
	}

	spin_unlock_irqrestore(&aess->lock, flags);
	return 0;
}
EXPORT_SYMBOL(omap_abe_port_is_enabled);

/*
 * omap_abe_port_enable - enable ABE logical port
 *
 * aess -  AESS.
 * logical_id - logical ABE port ID to be enabled.
 */
int omap_abe_port_enable(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);
	unsigned long flags;
	int ret = 0;

	if (!port) {
		dev_err(aess->dev, "Port %s is not open, can not be enabled\n",
			lport_name[logical_id]);
		return -EINVAL;
	}

	/* only enable the physical port iff it is disabled */
	pr_debug("port %s increment count %d\n",
		 lport_name[port->logical_id], port->users);

	spin_lock_irqsave(&aess->lock, flags);
	if (port->users == 0 && port_get_num_users(aess, port) == 0) {

		/* enable the physical port */
		pr_debug("port %s phy port %d enabled\n",
			 lport_name[port->logical_id], port->physical_id);
		omap_aess_enable_data_transfer(aess, port->physical_id);
	}

	port->state = PORT_ENABLED;
	port->users++;
	spin_unlock_irqrestore(&aess->lock, flags);
	return ret;
}
EXPORT_SYMBOL(omap_abe_port_enable);

/*
 * omap_abe_port_disable - disable ABE logical port
 *
 * aess -  ABE.
 * logical_id - logical ABE port ID to be disabled.
 */
int omap_abe_port_disable(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);
	unsigned long flags;
	int ret = 0;

	if (!port) {
		dev_err(aess->dev, "Port %s is not open, can not be disabled\n",
			lport_name[logical_id]);
		return -EINVAL;
	}

	/* only disable the port iff no other users are using it */
	pr_debug("port %s decrement count %d\n",
		 lport_name[port->logical_id], port->users);

	spin_lock_irqsave(&aess->lock, flags);

	if (port->users == 1 && port_get_num_users(aess, port) == 1) {
		/* disable the physical port */
		pr_debug("port %s phy port %d disabled\n",
			 lport_name[port->logical_id], port->physical_id);

		omap_aess_disable_data_transfer(aess, port->physical_id);
	}

	port->state = PORT_DISABLED;
	port->users--;
	spin_unlock_irqrestore(&aess->lock, flags);
	return ret;
}
EXPORT_SYMBOL(omap_abe_port_disable);

/*
 * omap_abe_port_open - open ABE logical port
 *
 * @abe -  ABE.
 * @logical_id - logical ABE port ID to be opened.
 */
int omap_abe_port_open(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *port;
	unsigned long flags;

#ifdef CONFIG_DEBUG_FS
	char debug_fs_name[32];
#endif

	if (logical_id < 0 || logical_id >= OMAP_ABE_PORT_ID_LAST) {
		pr_err("invalid logical port %d\n", logical_id);
		return -EINVAL;
	}

	if (port_is_open(aess, logical_id)) {
		pr_err("logical port %d already open\n", logical_id);
		return -EBUSY;
	}

	port = find_logical_port(aess, logical_id);
	if (port) {
		dev_info(aess->dev, "Port %s is already open\n",
			 lport_name[logical_id]);
		return 0;
	}

	port = kzalloc(sizeof(struct omap_abe_port), GFP_KERNEL);
	if (port == NULL)
		return -ENOMEM;

	port->logical_id = logical_id;
	port->physical_id = get_physical_id(logical_id);
	port->state = PORT_DISABLED;
	port->aess = aess;

	spin_lock_irqsave(&aess->lock, flags);
	list_add(&port->list, &aess->ports);
	spin_unlock_irqrestore(&aess->lock, flags);

#ifdef CONFIG_DEBUG_FS
	sprintf(debug_fs_name, "%s_state", lport_name[logical_id]);
	port->debugfs_lstate = debugfs_create_u32(debug_fs_name, 0644,
			aess->debugfs_root, &port->state);
	sprintf(debug_fs_name, "%s_phy", lport_name[logical_id]);
	port->debugfs_lphy = debugfs_create_u32(debug_fs_name, 0644,
			aess->debugfs_root, &port->physical_id);
	sprintf(debug_fs_name, "%s_users", lport_name[logical_id]);
	port->debugfs_lusers = debugfs_create_u32(debug_fs_name, 0644,
			aess->debugfs_root, &port->users);
#endif

	pr_debug("opened port %s\n", lport_name[logical_id]);
	return 0;
}
EXPORT_SYMBOL(omap_abe_port_open);

/*
 * omap_abe_port_close - close ABE logical port
 *
 * @port - logical ABE port to be closed (and disabled).
 */
void omap_abe_port_close(struct omap_aess *aess, int logical_id)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);
	unsigned long flags;

	if (!port) {
		dev_err(aess->dev, "Port %s is not open, can not be closed\n",
			lport_name[logical_id]);
		return;
	}

	/* disable the port */
	omap_abe_port_disable(aess, logical_id);

	spin_lock_irqsave(&aess->lock, flags);
	list_del(&port->list);
	spin_unlock_irqrestore(&aess->lock, flags);

	pr_debug("closed port %s\n", lport_name[port->logical_id]);

	kfree(port);
	
}
EXPORT_SYMBOL(omap_abe_port_close);

void omap_abe_port_set_substream(struct omap_aess *aess, int logical_id,
				 struct snd_pcm_substream *substream)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);

	if (!port) {
		dev_err(aess->dev, "Port %s is not open.\n",
			lport_name[logical_id]);
		return;
	}

	port->substream = substream;
}
EXPORT_SYMBOL(omap_abe_port_set_substream);

struct snd_pcm_substream *omap_abe_port_get_substream(struct omap_aess *aess,
						      int logical_id)
{
	struct omap_abe_port *port = find_logical_port(aess, logical_id);

	if (!port) {
		dev_err(aess->dev, "Port %s is not open.\n",
			lport_name[logical_id]);
		return NULL;
	}

	return port->substream;
}
EXPORT_SYMBOL(omap_abe_port_get_substream);

void omap_abe_port_mgr_init(struct omap_aess *aess)
{
	the_aess = aess;

#ifdef CONFIG_DEBUG_FS
	aess->debugfs_root = debugfs_create_dir("abe_port", NULL);
	if (!aess->debugfs_root)
		dev_warn(aess->dev, "Failed to create debugfs directory\n");
#endif
}

void omap_abe_port_mgr_cleanup(struct omap_aess *aess)
{
	the_aess = NULL;

	if (users != 0)
		dev_warn(aess->dev, "Port manager use count is not 0 (%d)\n",
			users);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(aess->debugfs_root);
#endif
}

struct omap_aess *omap_abe_port_mgr_get(void)
{
	mutex_lock(&port_mgr_mutex);

	if (!the_aess)
		pr_err("%s: AESS has not been initialized!\n", __func__);
	else
		users++;

	mutex_unlock(&port_mgr_mutex);
	return the_aess;
}
EXPORT_SYMBOL(omap_abe_port_mgr_get);

void omap_abe_port_mgr_put(struct omap_aess *aess)
{
	mutex_lock(&port_mgr_mutex);

	if (users == 0)
		goto out;

	users--;
out:
	mutex_unlock(&port_mgr_mutex);
}
EXPORT_SYMBOL(omap_abe_port_mgr_put);

MODULE_LICENSE("GPL");
