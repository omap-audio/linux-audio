/*
 * drivers/gpu/drm/omapdrm/omap_drv.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
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

#include <linux/wait.h>
#include <linux/sort.h>
#include <linux/of.h>
#include <linux/bitops.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>

#include "omap_dmm_tiler.h"
#include "omap_drv.h"

#define DRIVER_NAME		MODULE_NAME
#define DRIVER_DESC		"OMAP DRM"
#define DRIVER_DATE		"20110917"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

#define MAX_NR_DISPLAYS		8
static int display_order[MAX_NR_DISPLAYS];
static int display_order_nelm;
module_param_array_named(displays, display_order, int, &display_order_nelm,
			 0444);
MODULE_PARM_DESC(displays,
		 "ID array to specify the order of the active displays");

/*
 * mode config funcs
 */

/* Notes about mapping DSS and DRM entities:
 *    CRTC:        overlay
 *    encoder:     manager.. with some extension to allow one primary CRTC
 *                 and zero or more video CRTC's to be mapped to one encoder?
 *    connector:   dssdev.. manager can be attached/detached from different
 *                 devices
 */

static void omap_fb_output_poll_changed(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	DBG("dev=%p", dev);
	if (priv->fbdev)
		drm_fb_helper_hotplug_event(priv->fbdev);
}

struct omap_atomic_state_commit {
	struct work_struct work;
	struct drm_device *dev;
	struct drm_atomic_state *state;
	u32 crtcs;
};

static void omap_atomic_wait_for_completion(struct drm_device *dev,
					    struct drm_atomic_state *old_state)
{
	struct drm_crtc_state *old_crtc_state;
	struct drm_crtc *crtc;
	unsigned int i;
	int ret;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		if (!crtc->state->enable)
			continue;

		ret = omap_crtc_wait_pending(crtc);

		if (!ret)
			dev_warn(dev->dev,
				 "atomic complete timeout (pipe %u)!\n", i);
	}
}

static void omap_atomic_complete(struct omap_atomic_state_commit *commit)
{
	struct drm_device *dev = commit->dev;
	struct omap_drm_private *priv = dev->dev_private;
	struct drm_atomic_state *old_state = commit->state;

	/* Apply the atomic update. */
	priv->dispc_ops->runtime_get();

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	/* With the current dss dispc implementation we have to enable
	 * the new modeset before we can commit planes. The dispc ovl
	 * configuration relies on the video mode configuration been
	 * written into the HW when the ovl configuration is
	 * calculated.
	 *
	 * This approach is not ideal because after a mode change the
	 * plane update is executed only after the first vblank
	 * interrupt. The dispc implementation should be fixed so that
	 * it is able use uncommitted drm state information.
	 */
	drm_atomic_helper_commit_modeset_enables(dev, old_state);
	omap_atomic_wait_for_completion(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state, 0);

	omap_atomic_wait_for_completion(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);

	priv->dispc_ops->runtime_put();

	drm_atomic_state_free(old_state);

	/* Complete the commit, wake up any waiter. */
	spin_lock(&priv->commit.lock);
	priv->commit.pending &= ~commit->crtcs;
	spin_unlock(&priv->commit.lock);

	wake_up_all(&priv->commit.wait);

	kfree(commit);
}

static void omap_atomic_work(struct work_struct *work)
{
	struct omap_atomic_state_commit *commit =
		container_of(work, struct omap_atomic_state_commit, work);

	omap_atomic_complete(commit);
}

static bool omap_atomic_is_pending(struct omap_drm_private *priv,
				   struct omap_atomic_state_commit *commit)
{
	bool pending;

	spin_lock(&priv->commit.lock);
	pending = priv->commit.pending & commit->crtcs;
	spin_unlock(&priv->commit.lock);

	return pending;
}

static int omap_atomic_check(struct drm_device *dev,
			     struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	return ret;
}

static int omap_atomic_commit(struct drm_device *dev,
			      struct drm_atomic_state *state, bool nonblock)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct omap_atomic_state_commit *commit;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	int i, ret;

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;

	/* Allocate the commit object. */
	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (commit == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	INIT_WORK(&commit->work, omap_atomic_work);
	commit->dev = dev;
	commit->state = state;

	/* Wait until all affected CRTCs have completed previous commits and
	 * mark them as pending.
	 */
	for_each_crtc_in_state(state, crtc, crtc_state, i)
		commit->crtcs |= drm_crtc_mask(crtc);

	wait_event(priv->commit.wait, !omap_atomic_is_pending(priv, commit));

	spin_lock(&priv->commit.lock);
	priv->commit.pending |= commit->crtcs;
	spin_unlock(&priv->commit.lock);

	/* Swap the state, this is the point of no return. */
	drm_atomic_helper_swap_state(state, true);

	if (nonblock)
		schedule_work(&commit->work);
	else
		omap_atomic_complete(commit);

	return 0;

error:
	drm_atomic_helper_cleanup_planes(dev, state);
	return ret;
}

static const struct drm_mode_config_funcs omap_mode_config_funcs = {
	.fb_create = omap_framebuffer_create,
	.output_poll_changed = omap_fb_output_poll_changed,
	.atomic_check = omap_atomic_check,
	.atomic_commit = omap_atomic_commit,
};

static int get_connector_type(struct omap_dss_device *dssdev)
{
	switch (dssdev->type) {
	case OMAP_DISPLAY_TYPE_HDMI:
		return DRM_MODE_CONNECTOR_HDMIA;
	case OMAP_DISPLAY_TYPE_DVI:
		return DRM_MODE_CONNECTOR_DVID;
	case OMAP_DISPLAY_TYPE_DSI:
		return DRM_MODE_CONNECTOR_DSI;
	case OMAP_DISPLAY_TYPE_DPI:
	case OMAP_DISPLAY_TYPE_DBI:
		return DRM_MODE_CONNECTOR_DPI;
	case OMAP_DISPLAY_TYPE_VENC:
		/* TODO: This could also be composite */
		return DRM_MODE_CONNECTOR_SVIDEO;
	case OMAP_DISPLAY_TYPE_SDI:
		return DRM_MODE_CONNECTOR_LVDS;
	default:
		return DRM_MODE_CONNECTOR_Unknown;
	}
}

static void omap_disconnect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		dssdev->driver->disconnect(dssdev);
		priv->dssdevs[i] = NULL;
		omap_dss_put_device(dssdev);
	}

	priv->num_dssdevs = 0;
}

static int omap_compare_dssdevs(const void *a, const void *b)
{
	const struct omap_dss_device *dssdev1 = *(struct omap_dss_device **)a;
	const struct omap_dss_device *dssdev2 = *(struct omap_dss_device **)b;
	int  id1, id2;

	id1 = of_alias_get_id(dssdev1->dev->of_node, "display");
	id2 = of_alias_get_id(dssdev2->dev->of_node, "display");

	if (id1 > id2)
		return 1;
	else if (id1 < id2)
		return -1;
	return 0;
}

static void omap_collect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	struct omap_dss_device *dssdevs[ARRAY_SIZE(priv->dssdevs)];
	struct omap_dss_device *dssdev = NULL;
	int num_dssdevs = 0;
	unsigned long dssdev_mask = 0;
	int i;

	/* No displays should be enabled */
	if (display_order_nelm == 1 && display_order[0] < 0)
		return;

	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		set_bit(num_dssdevs, &dssdev_mask);
		dssdevs[num_dssdevs++] = dssdev;
		if (num_dssdevs == ARRAY_SIZE(dssdevs)) {
			/* To balance the 'for_each_dss_dev' loop */
			omap_dss_put_device(dssdev);
			break;
		}
	}

	/* Sort the list by DT aliases */
	sort(dssdevs, num_dssdevs, sizeof(dssdevs[0]), omap_compare_dssdevs,
	     NULL);

	/* Do ordering based on the display_order parameter array */
	for (i = 0; i < display_order_nelm; i++) {
		int old_index = display_order[i];

		if ((old_index >= 0 && old_index < num_dssdevs) &&
		    (dssdev_mask & BIT(old_index))) {
			priv->dssdevs[priv->num_dssdevs++] = dssdevs[old_index];
			clear_bit(old_index, &dssdev_mask);
		} else {
			dev_err(ddev->dev,
				"Ignoring invalid displays module parameter\n");
			priv->num_dssdevs = 0;
			break;
		}
	}

	/* if the target list is empty, copy the collected dssdevs, if any */
	if (priv->num_dssdevs == 0) {
		for (i = 0; i < num_dssdevs; i++)
			priv->dssdevs[i] = dssdevs[i];

		priv->num_dssdevs = num_dssdevs;
	} else {
		u32 idx;

		/* check if we have dssdev which is not carried over */
		for_each_set_bit(idx, &dssdev_mask, ARRAY_SIZE(dssdevs))
			omap_dss_put_device(dssdevs[idx]);
	}
}

static int omap_connect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	u32 working = 0;
	int r, i, j;

	if (!omapdss_stack_is_ready())
		return -EPROBE_DEFER;

	omap_collect_dssdevs(ddev);

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		r = dssdev->driver->connect(dssdev);
		if (r == -EPROBE_DEFER)
			goto cleanup;
		else if (r)
			dev_warn(dssdev->dev, "could not connect display: %s\n",
				 dssdev->name);
		else
			working |= BIT(i);
	}

	/* Remove the dssdevs if their connect failed */
	j = 0;
	for (i = 0; i < priv->num_dssdevs; i++) {
		if (working & BIT(i)) {
			if (j != i)
				priv->dssdevs[j] = priv->dssdevs[i];
			j++;
		} else {
			omap_dss_put_device(priv->dssdevs[i]);
		}
	}

	priv->num_dssdevs = j;

	return 0;

cleanup:
	/*
	 * if we are deferring probe, we disconnect the devices we previously
	 * connected
	 */
	omap_disconnect_dssdevs(ddev);

	return r;
}

static int omap_modeset_init_properties(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;

	static const struct drm_prop_enum_list trans_key_mode_list[] = {
		{ 0, "disable"},
		{ 1, "gfx-dst"},
		{ 2, "vid-src"},
	};

	/* plane properties */

	priv->zorder_prop = drm_property_create_range(dev, 0, "zorder", 0, 3);
	if (!priv->zorder_prop)
		return -ENOMEM;

	priv->global_alpha_prop = drm_property_create_range(dev, 0,
		"global_alpha", 0, 255);
	if (!priv->global_alpha_prop)
		return -ENOMEM;

	priv->pre_mult_alpha_prop = drm_property_create_bool(dev, 0,
		"pre_mult_alpha");
	if (!priv->pre_mult_alpha_prop)
		return -ENOMEM;

	/* crtc properties */

	priv->background_color_prop = drm_property_create_range(dev, 0,
		"background", 0, 0xffffff);
	if (!priv->background_color_prop)
		return -ENOMEM;

	/* crtc properties */

	priv->trans_key_mode_prop = drm_property_create_enum(dev, 0,
		"trans-key-mode",
		trans_key_mode_list, ARRAY_SIZE(trans_key_mode_list));
	if (!priv->trans_key_mode_prop)
		return -ENOMEM;

	priv->trans_key_prop = drm_property_create_range(dev, 0, "trans-key",
		0, 0xffffff);
	if (!priv->trans_key_prop)
		return -ENOMEM;

	priv->alpha_blender_prop = drm_property_create_bool(dev, 0,
		"alpha_blender");
	if (!priv->alpha_blender_prop)
		return -ENOMEM;

	return 0;
}

static int omap_modeset_init(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct omap_dss_device *dssdev = NULL;
	int num_ovls = priv->dispc_ops->get_num_ovls();
	int num_mgrs = priv->dispc_ops->get_num_mgrs();
	int num_crtcs, crtc_idx, plane_idx;
	int ret, i;
	u32 plane_crtc_mask;
	u32 min_w, min_h, max_w, max_h;

	drm_mode_config_init(dev);

	ret = omap_modeset_init_properties(dev);
	if (ret < 0)
		return ret;

	/*
	 * This function creates exactly one connector, encoder, crtc,
	 * and primary plane per each connected dss-device. Each
	 * connector->encoder->crtc chain is expected to be separate
	 * and each crtc is connect to a single dss-channel. If the
	 * configuration does not match the expectations or exceeds
	 * the available resources, the configuration is rejected.
	 */
	num_crtcs = priv->num_dssdevs;
	if (num_crtcs > num_mgrs || num_crtcs > num_ovls ||
	    num_crtcs > ARRAY_SIZE(priv->crtcs) ||
	    num_crtcs > ARRAY_SIZE(priv->planes) ||
	    num_crtcs > ARRAY_SIZE(priv->encoders) ||
	    num_crtcs > ARRAY_SIZE(priv->connectors)) {
		dev_err(dev->dev, "%s(): Too many connected displays\n",
			__func__);
		return -EINVAL;
	}

	/* All planes can be put to any CRTC */
	plane_crtc_mask = (1 << num_crtcs) - 1;

	dssdev = NULL;

	crtc_idx = 0;
	plane_idx = 0;
	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];
		struct drm_connector *connector;
		struct drm_encoder *encoder;
		struct drm_plane *plane;
		struct drm_crtc *crtc;

		encoder = omap_encoder_init(dev, dssdev);
		if (!encoder)
			return -ENOMEM;

		connector = omap_connector_init(dev,
				get_connector_type(dssdev), dssdev, encoder);
		if (!connector)
			return -ENOMEM;

		plane = omap_plane_init(dev, plane_idx, DRM_PLANE_TYPE_PRIMARY,
					plane_crtc_mask);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		crtc = omap_crtc_init(dev, plane, dssdev);
		if (IS_ERR(crtc))
			return PTR_ERR(crtc);

		drm_mode_connector_attach_encoder(connector, encoder);
		encoder->possible_crtcs = (1 << crtc_idx);

		priv->crtcs[priv->num_crtcs++] = crtc;
		priv->planes[priv->num_planes++] = plane;
		priv->encoders[priv->num_encoders++] = encoder;
		priv->connectors[priv->num_connectors++] = connector;

		plane_idx++;
		crtc_idx++;
	}

	/*
	 * Create normal planes for the remaining overlays:
	 */
	for (; plane_idx < num_ovls; plane_idx++) {
		struct drm_plane *plane;

		if (WARN_ON(priv->num_planes >= ARRAY_SIZE(priv->planes)))
			return -EINVAL;

		plane = omap_plane_init(dev, plane_idx, DRM_PLANE_TYPE_OVERLAY,
			plane_crtc_mask);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		priv->planes[priv->num_planes++] = plane;
	}

	DBG("registered %d planes, %d crtcs, %d encoders and %d connectors\n",
		priv->num_planes, priv->num_crtcs, priv->num_encoders,
		priv->num_connectors);

	priv->dispc_ops->get_min_max_size(&min_w, &min_h, &max_w, &max_h);

	dev->mode_config.min_width = min_w;
	dev->mode_config.min_height = min_h;

	/* note: eventually will need some cpu_is_omapXYZ() type stuff here
	 * to fill in these limits properly on different OMAP generations..
	 */
	dev->mode_config.max_width = max_w;
	dev->mode_config.max_height = max_h;

	dev->mode_config.funcs = &omap_mode_config_funcs;

	drm_mode_config_reset(dev);

	omap_drm_irq_install(dev);

	return 0;
}

/*
 * Enable the HPD in external components if supported
 */
static void omap_modeset_enable_external_hpd(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (dssdev->driver->enable_hpd)
			dssdev->driver->enable_hpd(dssdev);
	}
}

/*
 * Disable the HPD in external components if supported
 */
static void omap_modeset_disable_external_hpd(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (dssdev->driver->disable_hpd)
			dssdev->driver->disable_hpd(dssdev);
	}
}

/*
 * drm ioctl funcs
 */


static int ioctl_get_param(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct drm_omap_param *args = data;

	DBG("%p: param=%llu", dev, args->param);

	switch (args->param) {
	case OMAP_PARAM_CHIPSET_ID:
		args->value = priv->omaprev;
		break;
	default:
		DBG("unknown parameter %lld", args->param);
		return -EINVAL;
	}

	return 0;
}

static int ioctl_set_param(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_param *args = data;

	switch (args->param) {
	default:
		DBG("unknown parameter %lld", args->param);
		return -EINVAL;
	}

	return 0;
}

#define OMAP_BO_USER_MASK	0x00ffffff	/* flags settable by userspace */

static int ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_new *args = data;
	u32 flags = args->flags & OMAP_BO_USER_MASK;

	VERB("%p:%p: size=0x%08x, flags=%08x", dev, file_priv,
	     args->size.bytes, flags);

	return omap_gem_new_handle(dev, file_priv, args->size, flags,
				   &args->handle);
}

static int ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	int ret;

	VERB("%p:%p: handle=%d, op=%x", dev, file_priv, args->handle, args->op);

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	ret = omap_gem_op_sync(obj, args->op);

	if (!ret)
		ret = omap_gem_op_start(obj, args->op);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

	VERB("%p:%p: handle=%d", dev, file_priv, args->handle);

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	/* XXX flushy, flushy */
	ret = 0;

	if (!ret)
		ret = omap_gem_op_finish(obj, args->op);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_info *args = data;
	struct drm_gem_object *obj;
	int ret = 0;

	VERB("%p:%p: handle=%d", dev, file_priv, args->handle);

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	args->size = omap_gem_mmap_size(obj);
	args->offset = omap_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static const struct drm_ioctl_desc ioctls[DRM_COMMAND_END - DRM_COMMAND_BASE] = {
	DRM_IOCTL_DEF_DRV(OMAP_GET_PARAM, ioctl_get_param,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_SET_PARAM, ioctl_set_param,
			  DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_NEW, ioctl_gem_new,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_CPU_PREP, ioctl_gem_cpu_prep,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_CPU_FINI, ioctl_gem_cpu_fini,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_INFO, ioctl_gem_info,
			  DRM_AUTH | DRM_RENDER_ALLOW),
};

/*
 * drm driver funcs
 */

static int dev_open(struct drm_device *dev, struct drm_file *file)
{
	file->driver_priv = NULL;

	DBG("open: dev=%p, file=%p", dev, file);

	return 0;
}

/**
 * lastclose - clean up after all DRM clients have exited
 * @dev: DRM device
 *
 * Take care of cleaning up after all DRM clients have exited.  In the
 * mode setting case, we want to restore the kernel's initial mode (just
 * in case the last client left us in a bad state).
 */
static void dev_lastclose(struct drm_device *dev)
{
	int i;

	/* we don't support vga_switcheroo.. so just make sure the fbdev
	 * mode is active
	 */
	struct omap_drm_private *priv = dev->dev_private;
	int ret;

	DBG("lastclose: dev=%p", dev);

	/* need to restore default rotation state.. not sure
	 * if there is a cleaner way to restore properties to
	 * default state?  Maybe a flag that properties should
	 * automatically be restored to default state on
	 * lastclose?
	 */
	for (i = 0; i < priv->num_crtcs; i++) {
		struct drm_crtc *crtc = priv->crtcs[i];

		if (!crtc->primary->rotation_property)
			continue;

		drm_object_property_set_value(&crtc->base,
					      crtc->primary->rotation_property,
					      DRM_ROTATE_0);
	}

	for (i = 0; i < priv->num_planes; i++) {
		struct drm_plane *plane = priv->planes[i];

		if (!plane->rotation_property)
			continue;

		drm_object_property_set_value(&plane->base,
					      plane->rotation_property,
					      DRM_ROTATE_0);
	}

	if (priv->fbdev) {
		ret = drm_fb_helper_restore_fbdev_mode_unlocked(priv->fbdev);
		if (ret)
			DBG("failed to restore crtc mode");
	}
}

static const struct vm_operations_struct omap_gem_vm_ops = {
	.fault = omap_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations omapdriver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.unlocked_ioctl = drm_ioctl,
	.release = drm_release,
	.mmap = omap_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
};

static struct drm_driver omap_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM  | DRIVER_PRIME |
		DRIVER_ATOMIC | DRIVER_RENDER,
	.open = dev_open,
	.lastclose = dev_lastclose,
	.get_vblank_counter = drm_vblank_no_hw_counter,
	.enable_vblank = omap_irq_enable_vblank,
	.disable_vblank = omap_irq_disable_vblank,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init = omap_debugfs_init,
	.debugfs_cleanup = omap_debugfs_cleanup,
#endif
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = omap_gem_prime_export,
	.gem_prime_import = omap_gem_prime_import,
	.gem_free_object = omap_gem_free_object,
	.gem_vm_ops = &omap_gem_vm_ops,
	.dumb_create = omap_gem_dumb_create,
	.dumb_map_offset = omap_gem_dumb_map_offset,
	.dumb_destroy = drm_gem_dumb_destroy,
	.ioctls = ioctls,
	.num_ioctls = DRM_OMAP_NUM_IOCTLS,
	.fops = &omapdriver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int pdev_probe(struct platform_device *pdev)
{
	struct omap_drm_platform_data *pdata = pdev->dev.platform_data;
	struct omap_drm_private *priv;
	struct drm_device *ddev;
	unsigned int i;
	int ret;

	DBG("%s", pdev->name);

	if (omapdss_is_initialized() == false)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Allocate and initialize the DRM device. */
	ddev = drm_dev_alloc(&omap_drm_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	ddev->dev_private = priv;
	platform_set_drvdata(pdev, ddev);

	omap_crtc_pre_init();

	ret = omap_connect_dssdevs(ddev);
	if (ret)
		goto err_crtc_uninit;

	/* Initialize the driver private structure. */
	priv->dispc_ops = dispc_get_ops();
	priv->omaprev = pdata->omaprev;
	priv->wq = alloc_ordered_workqueue("omapdrm", 0);

	init_waitqueue_head(&priv->commit.wait);
	spin_lock_init(&priv->commit.lock);
	spin_lock_init(&priv->list_lock);
	INIT_LIST_HEAD(&priv->obj_list);

	/* Get memory bandwidth limits */
	if (priv->dispc_ops->get_memory_bandwidth_limit)
		priv->max_bandwidth =
				priv->dispc_ops->get_memory_bandwidth_limit();

	/* Get memory bandwidth and pixelcolc limits */
	if (priv->dispc_ops->get_memory_and_clock_limits)
		priv->dispc_ops->get_memory_and_clock_limits(
				&priv->max_bandwidth, &priv->max_pixelclock);

	omap_gem_init(ddev);

	ret = omap_modeset_init(ddev);
	if (ret) {
		dev_err(&pdev->dev, "omap_modeset_init failed: ret=%d\n", ret);
		goto err_gem_deinit;
	}

	/* Initialize vblank handling, start with all CRTCs disabled. */
	ret = drm_vblank_init(ddev, priv->num_crtcs);
	if (ret) {
		dev_err(&pdev->dev, "could not init vblank\n");
		goto err_cleanup_modeset;
	}

	for (i = 0; i < priv->num_crtcs; i++)
		drm_crtc_vblank_off(priv->crtcs[i]);

	priv->fbdev = omap_fbdev_init(ddev);

	drm_kms_helper_poll_init(ddev);
	omap_modeset_enable_external_hpd(ddev);

	if (priv->dispc_ops->has_writeback()) {
		ret = omap_wb_init(ddev);
		if (ret)
			dev_warn(&pdev->dev, "failed to initialize writeback\n");
		else
			priv->wb_initialized = true;
	}

	/*
	 * Register the DRM device with the core and the connectors with
	 * sysfs.
	 */
	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto err_cleanup_helpers;

	return 0;

err_cleanup_helpers:
	if (priv->wb_initialized)
		omap_wb_cleanup(ddev);
	omap_modeset_disable_external_hpd(ddev);
	drm_kms_helper_poll_fini(ddev);
	if (priv->fbdev)
		omap_fbdev_free(ddev);
err_cleanup_modeset:
	drm_mode_config_cleanup(ddev);
	omap_drm_irq_uninstall(ddev);
err_gem_deinit:
	omap_gem_deinit(ddev);
	destroy_workqueue(priv->wq);
	omap_disconnect_dssdevs(ddev);
err_crtc_uninit:
	omap_crtc_pre_uninit();
	drm_dev_unref(ddev);
	return ret;
}

static int pdev_remove(struct platform_device *pdev)
{
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct omap_drm_private *priv = ddev->dev_private;

	DBG("");

	drm_dev_unregister(ddev);

	if (priv->wb_initialized)
		omap_wb_cleanup(ddev);

	omap_modeset_disable_external_hpd(ddev);
	drm_kms_helper_poll_fini(ddev);

	if (priv->fbdev)
		omap_fbdev_free(ddev);

	drm_mode_config_cleanup(ddev);

	omap_drm_irq_uninstall(ddev);
	omap_gem_deinit(ddev);

	destroy_workqueue(priv->wq);

	omap_disconnect_dssdevs(ddev);
	omap_crtc_pre_uninit();

	drm_dev_unref(ddev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int omap_drm_suspend_all_displays(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (!dssdev->driver)
			continue;

		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			dssdev->driver->disable(dssdev);
			dssdev->activate_after_resume = true;
		} else {
			dssdev->activate_after_resume = false;
		}
	}

	return 0;
}

static int omap_drm_resume_all_displays(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (!dssdev->driver)
			continue;

		if (dssdev->activate_after_resume) {
			dssdev->driver->enable(dssdev);
			dssdev->activate_after_resume = false;
		}
	}

	return 0;
}

static int omap_drm_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_kms_helper_poll_disable(drm_dev);

	drm_modeset_lock_all(drm_dev);
	omap_drm_suspend_all_displays(drm_dev);
	drm_modeset_unlock_all(drm_dev);

	return 0;
}

static int omap_drm_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_modeset_lock_all(drm_dev);
	omap_drm_resume_all_displays(drm_dev);
	drm_modeset_unlock_all(drm_dev);

	drm_kms_helper_poll_enable(drm_dev);

	return omap_gem_resume(dev);
}
#endif

static SIMPLE_DEV_PM_OPS(omapdrm_pm_ops, omap_drm_suspend, omap_drm_resume);

static struct platform_driver pdev = {
	.driver = {
		.name = DRIVER_NAME,
		.pm = &omapdrm_pm_ops,
	},
	.probe = pdev_probe,
	.remove = pdev_remove,
};

static struct platform_driver * const drivers[] = {
	&omap_dmm_driver,
	&pdev,
};

static int __init omap_drm_init(void)
{
	DBG("init");

	return platform_register_drivers(drivers, ARRAY_SIZE(drivers));
}

static void __exit omap_drm_fini(void)
{
	DBG("fini");

	platform_unregister_drivers(drivers, ARRAY_SIZE(drivers));
}

/* need late_initcall() so we load after dss_driver's are loaded */
late_initcall(omap_drm_init);
module_exit(omap_drm_fini);

MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_DESCRIPTION("OMAP DRM Display Driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL v2");
