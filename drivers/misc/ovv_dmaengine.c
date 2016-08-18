/*
 * ovv - to move vrfb to dmaengine
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/omap-dma.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <video/omapvrfb.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

#define IMAGE_RES_X		640
#define IMAGE_RES_Y		480
#define IMAGE_BPP		4

#define MAX_PIXELS_PER_LINE	2048
#define MAC_VRFB_CTXS		2
#define VRFB_TX_TIMEOUT		1000
#define OMAP_DMA_NO_DEVICE	0

#define NR_OF_BUFFERS		3

enum dma_channel_state {
	DMA_CHAN_NOT_ALLOTED,
	DMA_CHAN_ALLOTED,
};

enum dss_rotation {
	dss_rotation_0_degree	= 0,
	dss_rotation_90_degree	= 1,
	dss_rotation_180_degree	= 2,
	dss_rotation_270_degree = 3,
};

struct vid_vrfb_dma {
	int dev_id;
	int dma_ch;
	int req_status;
	int tx_status;
	wait_queue_head_t wait;

	struct dma_chan *chan;
	struct dma_interleaved_template *xt;
};

struct omap_vout_device {

	/* we don't allow to change image fmt/size once buffer has
	 * been allocated
	 */
	int buffer_allocated;
	/* allow to reuse previously allocated buffer which is big enough */
	int buffer_size;
	/* keep buffer info across opens */
	unsigned long buf_virt_addr[NR_OF_BUFFERS];
	unsigned long buf_phy_addr[NR_OF_BUFFERS];
	u32 buf_size[NR_OF_BUFFERS];

	struct vrfb vrfb_context[MAC_VRFB_CTXS];

	struct v4l2_pix_format pix;

	enum dss_rotation rotation;
	bool mirror;

	int bpp; /* bytes per pixel */
	int vrfb_bpp; /* bytes per pixel with respect to VRFB */

	struct vid_vrfb_dma vrfb_dma_tx;
};

struct videobuf_buffer {
	int i; /* index of the buffer from to copy, 0 mostly */
};

#define BIZBASZ_PATTERN_ID_MASK		0x3
#define BIZBASZ_PATTERN_ID(val)		((val) & 0x3)
#define BIZBASZ_PATTERN(id, i)		(((i) << 2) | ((id) & 0x3))
static void ovv_init_pattern(struct omap_vout_device *vout, int id)
{
	int i;
	int size = vout->buf_size[id] / IMAGE_BPP;
	u32 *buffer = (u32 *)vout->buf_virt_addr[id];

	for (i = 0; i < size; i++)
		buffer[i] = BIZBASZ_PATTERN(id ? 2 : 1, i);
}

static int ovv_validate_pattern(struct omap_vout_device *vout, int id)
{
	int i;
	int ret = 0;
	int size = vout->buf_size[id] / IMAGE_BPP;
	u32 *buffer = (u32 *)vout->buf_virt_addr[id];
	bool prev_ok = true;

	for (i = 0; i < size; i++) {
		bool ok = true;
		if (buffer[i] != BIZBASZ_PATTERN(id ? 2 : 1, i)) {
			ret++;
			ok = false;
		}
		if (prev_ok != ok) {
			pr_err("%s: [%d] is now %s (0x%08x vs 0x%08x)\n",
			       __func__, i, ok ? "OK" : "NOK", buffer[i], BIZBASZ_PATTERN(id, i ? 2 : 1));
			prev_ok = ok;
		}
	}

	return ret;
}

static int ovv_comppare_destinations(struct omap_vout_device *vout)
{
	int i;
	int ret = 0;
	int size = vout->buf_size[1] / IMAGE_BPP;
	u32 *buffer1 = (u32 *)vout->buf_virt_addr[1];
	u32 *buffer2 = (u32 *)vout->buf_virt_addr[2];
	bool prev_ok = true;

	for (i = 0; i < size; i++) {
		bool ok = true;
		if (buffer1[i] != buffer2[i]) {
// 			pr_err("%s: no match at %d: 0x%08x vs 0x%08x\n",
// 				__func__, i, buffer1[i], buffer2[i]);
			ret++;
			ok = false;
		}
		if (prev_ok != ok) {
			pr_err("%s: [%d (pixel: %dx%d)] is now %s (0x%08x vs 0x%08x)\n",
			       __func__, i, i / MAX_PIXELS_PER_LINE,
			       i % MAX_PIXELS_PER_LINE,
			       ok ? "OK" : "NOK", buffer1[i], buffer2[i]);
			prev_ok = ok;
		}
	}

	return ret;
}

static void ovv_dump_50_item_from(struct omap_vout_device *vout, int start)
{
	int i;
	u32 *buffer0 = (u32 *)vout->buf_virt_addr[0];
	u32 *buffer1 = (u32 *)vout->buf_virt_addr[1];
	u32 *buffer2 = (u32 *)vout->buf_virt_addr[2];

	pr_err("%s: dump from %d:\n", __func__, start);
	pr_err("------------------------------------\n");
	pr_err("    SRC    |    DST1    |    DST2\n");
	pr_err("------------------------------------\n");
	for (i = 0; i < 50; i++) {
		pr_err("0x%08x | 0x%08x | 0x%08x\n", buffer0[start + i],
		       buffer1[start + i], buffer2[start + i]);
	}
}

static inline int calc_rotation(const struct omap_vout_device *vout)
{
	if (!vout->mirror)
		return vout->rotation;

	switch (vout->rotation) {
	case dss_rotation_90_degree:
		return dss_rotation_270_degree;
	case dss_rotation_270_degree:
		return dss_rotation_90_degree;
	case dss_rotation_180_degree:
		return dss_rotation_0_degree;
	default:
		return dss_rotation_180_degree;
	}
}

static struct omap_vout_device vout;

static inline int is_rotation_enabled(const struct omap_vout_device *vout)
{
	return vout->rotation || vout->mirror;
}

static void dmaengine_callback(void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;

	pr_err("%s: ENTER\n", __func__);
	t->tx_status = 1;
	wake_up_interruptible(&t->wait);
}

static void omap_vout_vrfb_dma_tx_callback(int lch, u16 ch_status, void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;

	pr_err("%s: ENTER\n", __func__);
	t->tx_status = 1;
	wake_up_interruptible(&t->wait);
}

static int omap_vout_prepare_vrfb(struct omap_vout_device *vout,
				  struct videobuf_buffer *vb)
{
	dma_addr_t dmabuf;
	struct vid_vrfb_dma *tx;
	enum dss_rotation rotation;
	u32 dest_frame_index = 0, src_element_index = 0;
	u32 dest_element_index = 0, src_frame_index = 0;
	u32 elem_count = 0, frame_count = 0, pixsize = 2;

	if (!is_rotation_enabled(vout))
		return 0;

	dmabuf = vout->buf_phy_addr[vb->i];
	/* If rotation is enabled, copy input buffer into VRFB
	 * memory space using DMA. We are copying input buffer
	 * into VRFB memory space of desired angle and DSS will
	 * read image VRFB memory for 0 degree angle
	 */
	pixsize = vout->bpp * vout->vrfb_bpp;
	/*
	 * DMA transfer in double index mode
	 */

	/* Frame index */
	dest_frame_index = ((MAX_PIXELS_PER_LINE * pixsize) -
			(vout->pix.width * vout->bpp)) + 1;

	/* Source and destination parameters */
	src_element_index = 0;
	src_frame_index = 0;
	dest_element_index = 1;
	/* Number of elements per frame */
	elem_count = vout->pix.width * vout->bpp;
	frame_count = vout->pix.height;
	tx = &vout->vrfb_dma_tx;
	tx->tx_status = 0;
	omap_set_dma_transfer_params(tx->dma_ch, OMAP_DMA_DATA_TYPE_S32,
			(elem_count / 4), frame_count, OMAP_DMA_SYNC_ELEMENT,
			tx->dev_id, 0x0);
	/* src_port required only for OMAP1 */
	omap_set_dma_src_params(tx->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
			dmabuf, src_element_index, src_frame_index);
	/*set dma source burst mode for VRFB */
	omap_set_dma_src_burst_mode(tx->dma_ch, OMAP_DMA_DATA_BURST_16);

	/* dest_port required only for OMAP1 */
	omap_set_dma_dest_params(tx->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
			vout->vrfb_context[vb->i].paddr[0], dest_element_index,
			dest_frame_index);
	/*set dma dest burst mode for VRFB */
	omap_set_dma_dest_burst_mode(tx->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	omap_start_dma(tx->dma_ch);
	wait_event_interruptible_timeout(tx->wait, tx->tx_status == 1,
					 VRFB_TX_TIMEOUT);

	if (tx->tx_status == 0) {
		omap_stop_dma(tx->dma_ch);
		return -EINVAL;
	}
	/* Store buffers physical address into an array. Addresses
	 * from this array will be used to configure DSS */
//	rotation = calc_rotation(vout);
// 	vout->queued_buf_addr[vb->i] = (u8 *)
// 		vout->vrfb_context[vb->i].paddr[rotation];
	return 0;
}

static int omap_vout_prepare_vrfb2(struct omap_vout_device *vout,
				   struct videobuf_buffer *vb)
{
	struct dma_async_tx_descriptor *tx;
	enum dma_ctrl_flags flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;
	struct dma_chan *chan = vout->vrfb_dma_tx.chan;
	struct dma_device *dmadev = chan->device;
	struct dma_interleaved_template *xt = vout->vrfb_dma_tx.xt;
	dma_cookie_t cookie;
	enum dma_status status;
	enum dss_rotation rotation;
	size_t dst_icg;
	u32 pixsize;

	if (!is_rotation_enabled(vout))
		return 0;

	/* If rotation is enabled, copy input buffer into VRFB
	 * memory space using DMA. We are copying input buffer
	 * into VRFB memory space of desired angle and DSS will
	 * read image VRFB memory for 0 degree angle
	 */

	pixsize = vout->bpp * vout->vrfb_bpp;
	dst_icg = ((MAX_PIXELS_PER_LINE * pixsize) -
		  (vout->pix.width * vout->bpp)) + 1;

	xt->src_start = vout->buf_phy_addr[vb->i];
	xt->dst_start = vout->vrfb_context[vb->i].paddr[0];

	xt->numf = vout->pix.height;
	xt->frame_size = 1;
	xt->sgl[0].size = vout->pix.width * vout->bpp;
	xt->sgl[0].icg = dst_icg;

	xt->dir = DMA_MEM_TO_MEM;
	xt->src_sgl = false;
	xt->src_inc = true;
	xt->dst_sgl = true;
	xt->dst_inc = true;

	tx = dmadev->device_prep_interleaved_dma(chan, xt, flags);
	if (tx == NULL) {
		pr_err("%s: DMA interleaved prep error\n", __func__);
		return -EINVAL;
	}

	tx->callback = dmaengine_callback;
	tx->callback_param = &vout->vrfb_dma_tx;

	cookie = dmaengine_submit(tx);
	if (dma_submit_error(cookie)) {
		pr_err("%s: dmaengine_submit failed (%d)\n", __func__, cookie);
		return -EINVAL;
	}

	vout->vrfb_dma_tx.tx_status = 0;
	dma_async_issue_pending(chan);

	wait_event_interruptible_timeout(vout->vrfb_dma_tx.wait,
					 vout->vrfb_dma_tx.tx_status == 1,
					 VRFB_TX_TIMEOUT);

	status = dma_async_is_tx_complete(chan, cookie, NULL, NULL);

	if (vout->vrfb_dma_tx.tx_status == 0) {
		pr_err("%s: Timeout while waiting for DMA\n", __func__);
		dmaengine_terminate_sync(chan);
		return -EINVAL;
	} else if (status != DMA_COMPLETE) {
		pr_err("%s: DMA completion %s status\n", __func__,
		       status == DMA_ERROR ? "error" : "busy");
		dmaengine_terminate_sync(chan);
		return -EINVAL;
	}

	/* Store buffers physical address into an array. Addresses
	 * from this array will be used to configure DSS */
//	rotation = calc_rotation(vout);
// 	vout->queued_buf_addr[vb->i] = (u8 *)
// 		vout->vrfb_context[vb->i].paddr[rotation];
	return 0;
}

#if 0
static unsigned long omap_vout_alloc_buffer(struct omap_vout_device *vout,
					    int i)
{
	u32 buf_size = vout->buf_size[i];
	u32 *phys_addr = (u32 *) &vout->buf_phy_addr[i];
	u32 order, size;
	unsigned long virt_addr, addr;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);
	virt_addr = __get_free_pages(GFP_KERNEL, order);
	addr = virt_addr;

	if (virt_addr) {
		while (size > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}
	*phys_addr = (u32) virt_to_phys((void *) virt_addr);
	return virt_addr;
}

static void omap_vout_free_buffer(struct omap_vout_device *vout,
				  int i)
{
	unsigned long virtaddr = vout->buf_virt_addr[i];
	u32 buf_size = vout->buf_size[i];
	u32 order, size;
	unsigned long addr = virtaddr;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);

	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages((unsigned long) virtaddr, order);
}

#else

static unsigned long omap_vout_alloc_buffer(struct omap_vout_device *vout,
						     int i)
{
	u32 *phys_addr = (u32 *) &vout->buf_phy_addr[i];
	unsigned long virt_addr;

	virt_addr = (unsigned long) dma_alloc_coherent(NULL, vout->buf_size[i],
							 phys_addr, GFP_KERNEL);

	return virt_addr;
}

static void omap_vout_free_buffer(struct omap_vout_device *vout,
					   int i)
{
	dma_free_coherent(NULL, vout->buf_size[i], (void *)vout->buf_virt_addr[i],
			  vout->buf_phy_addr[i]);
}
#endif

static int ovv_setup_buffers(struct omap_vout_device *vout)
{
	int i, ret;

	for (i = 0; i < NR_OF_BUFFERS; i++) {
		vout->buf_virt_addr[i] = omap_vout_alloc_buffer(vout, i);
		if (!vout->buf_virt_addr[i]) {
			pr_err("%s: failed to allocate %d\n", __func__, i);
			goto out;
		}
		ovv_init_pattern(vout, i);
		ret = ovv_validate_pattern(vout, i);
		if (ret) {
			pr_err("%s: pattern mismatch in %d places\n", __func__, ret);
			ret = 0;
		} else {
			pr_err("%s: pattern is good\n", __func__);
		}
		
	}

	return 0;
out:
	for (; i >=0; --i)
		omap_vout_free_buffer(vout, i);

	return -ENOMEM;
}

static int ovv_free_buffers(struct omap_vout_device *vout)
{
	int i;

	for (i = 0; i < NR_OF_BUFFERS; i++)
		omap_vout_free_buffer(vout, i);
	return 0;
}

static int ovv_get_legacy_dma(struct omap_vout_device *vout)
{
	int ret;

	vout->vrfb_dma_tx.dev_id = OMAP_DMA_NO_DEVICE;
	vout->vrfb_dma_tx.dma_ch = -1;
	vout->vrfb_dma_tx.req_status = DMA_CHAN_ALLOTED;
	ret = omap_request_dma(vout->vrfb_dma_tx.dev_id, "VRFB DMA TX",
			omap_vout_vrfb_dma_tx_callback,
			(void *) &vout->vrfb_dma_tx, &vout->vrfb_dma_tx.dma_ch);
	if (ret < 0) {
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		pr_err("%s: failed to allocate DMA Channel\n", __func__);
	} else {
		pr_err("%s: got channel: %d\n", __func__,
		       vout->vrfb_dma_tx.dma_ch);
	}
	init_waitqueue_head(&vout->vrfb_dma_tx.wait);

	return ret;
}

static int ovv_get_dmaengine_dma(struct omap_vout_device *vout)
{
	int ret = 0;
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_INTERLEAVE, mask);
	vout->vrfb_dma_tx.chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(vout->vrfb_dma_tx.chan)) {
		pr_err("%s: failed to allocate DMA Channel\n", __func__);
		ret = PTR_ERR(vout->vrfb_dma_tx.chan);
	} else {
		pr_err("%s: got channel\n", __func__);
	}

	vout->vrfb_dma_tx.xt = kzalloc(sizeof(struct dma_interleaved_template) +
				       sizeof(struct data_chunk), GFP_KERNEL);
	if (!vout->vrfb_dma_tx.xt) {
		dma_release_channel(vout->vrfb_dma_tx.chan);
		return -ENOMEM;
	}

	return ret;
}

static int __init ovv_init(void)
{
	struct videobuf_buffer vb;
	int ret;

	pr_err("%s: ENTER\n", __func__);

	vout.rotation = dss_rotation_90_degree;
	vout.bpp = IMAGE_BPP;
	vout.vrfb_bpp = 1;
	vout.pix.width = IMAGE_RES_X;
	vout.pix.height = IMAGE_RES_Y;
	vb.i = 0;

	vout.buf_size[0] = IMAGE_RES_X * IMAGE_RES_Y * IMAGE_BPP;
	vout.buf_size[1] = MAX_PIXELS_PER_LINE * IMAGE_RES_Y * IMAGE_BPP * vout.vrfb_bpp;
	vout.buf_size[2] = MAX_PIXELS_PER_LINE * IMAGE_RES_Y * IMAGE_BPP * vout.vrfb_bpp;
	ret = ovv_setup_buffers(&vout);
	if (ret) {
		pr_err("%s: buffer allocation failed\n", __func__);
		return ret;
	}

	ret = ovv_get_legacy_dma(&vout);
	if (ret)
		goto err_ldma;

	ret = ovv_get_dmaengine_dma(&vout);
	if (ret)
		goto err_ddma;

//	ovv_dump_50_item_from(&vout, 0);
//	ovv_dump_50_item_from(&vout, IMAGE_RES_X - 25);

	/* copy with legacy to target1 */
	vout.vrfb_context[vb.i].paddr[0] = vout.buf_phy_addr[1];
	ret = omap_vout_prepare_vrfb(&vout, &vb);
	if (ret) {
		pr_err("%s: legacy copy failed\n", __func__);
	}

	/* copy with legacy to target2 */
	vout.vrfb_context[vb.i].paddr[0] = vout.buf_phy_addr[2];
	ret = omap_vout_prepare_vrfb(&vout, &vb);
	if (ret) {
		pr_err("%s: legacy copy failed\n", __func__);
	}

	ret = ovv_comppare_destinations(&vout);
	if (ret) {
		pr_err("%s: Legacy vs Legacy: they differ in %d places\n",
		       __func__, ret);
		ret = 0;
	} else {
		pr_err("%s: Legacy vs Legacy: they are identical\n", __func__);
	}

	ovv_init_pattern(&vout, 2);
	ret = ovv_validate_pattern(&vout, 2);
	if (ret) {
		pr_err("%s: pattern mismatch in %d places\n", __func__, ret);
		ret = 0;
	} else {
		pr_err("%s: pattern is good\n", __func__);
	}

	/* copy with dmaengine to target2 */
	vout.vrfb_context[vb.i].paddr[0] = vout.buf_phy_addr[2];
	ret = omap_vout_prepare_vrfb2(&vout, &vb);
	if (ret) {
		pr_err("%s: dmaengine copy failed\n", __func__);
	}

//	ovv_dump_50_item_from(&vout, 0);
//	ovv_dump_50_item_from(&vout, IMAGE_RES_X - 25);

	ret = ovv_comppare_destinations(&vout);
	if (ret) {
		pr_err("%s: Legacy vs dmaengine: they differ in %d places\n",
		       __func__, ret);
		ret = 0;
	} else {
		pr_err("%s: Legacy vs dmaengine: they are identical\n", __func__);
	}
	pr_err("%s: GOOD\n", __func__);
	return ret;

err_ddma:
	omap_free_dma(vout.vrfb_dma_tx.dma_ch);
err_ldma:
	ovv_free_buffers(&vout);

	return ret;
}

static void __exit ovv_exit(void)
{
	pr_err("%s: ENTER\n", __func__);

	ovv_free_buffers(&vout);
	omap_free_dma(vout.vrfb_dma_tx.dma_ch);
	dma_release_channel(vout.vrfb_dma_tx.chan);
}

module_init(ovv_init);
module_exit(ovv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Ujfalusi");
MODULE_DESCRIPTION("dmaengine interleaved test");
