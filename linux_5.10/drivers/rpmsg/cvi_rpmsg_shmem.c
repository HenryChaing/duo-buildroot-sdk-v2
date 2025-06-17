/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * derived from the omap-rpmsg implementation.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 *
 * Copyright (C) CHIANG,KUAN-TING <n96121210@gs.ncku.edu.tw>
 *
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <asm/io.h>
#include <asm-generic/io.h>
#include <linux/miscdevice.h>
#include "rtos_cmdqu.h"

struct cvi_virdev {
	struct virtio_device vdev;
	unsigned int vring[2];
	struct virtqueue *vq[2];
	int base_vq_id;
	int num_of_vqs;
};

struct cvi_rpmsg_vproc {
	char *rproc_name;
	struct mutex lock;
	int vdev_nums;
#define MAX_VDEV_NUMS	7
	struct cvi_virdev ivdev[MAX_VDEV_NUMS];
};

static struct delayed_work rpmsg_work;
struct virtio_device *vdev_isr = NULL;

static __u64  reg_base;
volatile struct mailbox_set_register *mbox_reg_rpmsg;
volatile unsigned long *mailbox_context_rpmsg; // mailbox buffer context is 64 Bytess
#define MAILBOX_CONTEXT_OFFSET  0x0400
/*
 * For now, allocate 256 buffers of 512 bytes for each side. each buffer
 * will then have 16B for the msg header and 496B for the payload.
 * This will require a total space of 256KB for the buffers themselves, and
 * 3 pages for every vring (the size of the vring depends on the number of
 * buffers it supports).
 */
#define RPMSG_NUM_BUFS		(256)
#define RPMSG_BUF_SIZE		(512)
#define MAX_NUM 10	/* enlarge it if overflow happen */
/*
 * The alignment between the consumer and producer parts of the vring.
 * Note: this is part of the "wire" protocol. If you change this, you need
 * to update your BIOS image as well
 */
#define RPMSG_VRING_ALIGN	(4096)

/* With 256 buffers, our vring will occupy 3 pages */
#define RPMSG_RING_SIZE	((DIV_ROUND_UP(vring_size(RPMSG_NUM_BUFS, \
				RPMSG_VRING_ALIGN), PAGE_SIZE)) * PAGE_SIZE)

#define to_cvi_virdev(vd) container_of(vd, struct cvi_virdev, vdev)
#define to_cvi_rpdev(vd, id) container_of(vd, struct cvi_rpmsg_vproc, ivdev[id])


struct cvi_rpmsg_vq_info {
	__u16 num;	/* number of entries in the virtio_ring */
	__u16 vq_id;	/* a globaly unique index of this virtqueue */
	__u16 simple_id;
	void *addr;	/* address where we mapped the virtio ring */
	struct cvi_rpmsg_vproc *rpdev;
};

static u64 cvi_rpmsg_get_features(struct virtio_device *vdev)
{
	/* VIRTIO_RPMSG_F_NS has been made private */
	return 1 << 0;
}

static int cvi_rpmsg_finalize_features(struct virtio_device *vdev)
{
	/* Give virtio_ring a chance to accept features */
	vring_transport_features(vdev);
	return 0;
}

/* kick the remote processor, and let it know which virtqueue to poke at */
static bool cvi_rpmsg_notify(struct virtqueue *vq)
{
	unsigned int mu_rpmsg = 0;
	struct cvi_rpmsg_vq_info *rpvq = vq->priv;

	int queue_id = rpvq->simple_id;
	mu_rpmsg = rpvq->vq_id << 16;
	mutex_lock(&rpvq->rpdev->lock);

	/* send the index of the triggered virtqueue as the mu payload */
	struct cmdqu_t cmd = {0};
    cmd.param_ptr = mu_rpmsg;
	cmd.cmd_id = queue_id;
	rtos_cmdqu_send(&cmd);

	mutex_unlock(&rpvq->rpdev->lock);

	return true;
}

static struct virtqueue *rp_find_vq(struct virtio_device *vdev,
				    unsigned int index,
				    void (*callback)(struct virtqueue *vq),
				    const char *name)
{
	struct cvi_virdev *virdev = to_cvi_virdev(vdev);
	struct cvi_rpmsg_vproc *rpdev = to_cvi_rpdev(virdev,
						     virdev->base_vq_id / 2);
	struct cvi_rpmsg_vq_info *rpvq;
	struct virtqueue *vq;
	int err;

	rpvq = kmalloc(sizeof(*rpvq), GFP_KERNEL);
	if (!rpvq) {
		pr_err("no more memory %d\n", -ENOMEM);
		return ERR_PTR(-ENOMEM);
	}
		
	rpvq->addr = (__force void *) ioremap(virdev->vring[index],
	 						RPMSG_RING_SIZE);

	if (!rpvq->addr) {
		err = -ENOMEM;
		pr_err("no more memory %d\n", err);
		goto free_rpvq;
	}

	memset(rpvq->addr, 0, RPMSG_RING_SIZE);

	vq = vring_new_virtqueue(index, RPMSG_NUM_BUFS/*2*/, RPMSG_VRING_ALIGN,
			vdev, true, true, rpvq->addr, cvi_rpmsg_notify, callback,
			name);
	if (!vq) {
		pr_err("vring_new_virtqueue failed\n");
		err = -ENOMEM;
		goto unmap_vring;
	}

	virdev->vq[index] = vq;
	vq->priv = rpvq;
	/* system-wide unique id for this virtqueue */
	rpvq->vq_id = virdev->base_vq_id + index;
	rpvq->rpdev = rpdev;
	rpvq->simple_id = index;
	mutex_init(&rpdev->lock);

	return vq;

unmap_vring:
	/* iounmap normal memory, so make sparse happy */
	iounmap((__force void __iomem *) rpvq->addr);
free_rpvq:
	kfree(rpvq);
	return ERR_PTR(err);
}

static void cvi_rpmsg_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;
	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		struct cvi_rpmsg_vq_info *rpvq = vq->priv;

		iounmap(rpvq->addr);
		vring_del_virtqueue(vq);
		kfree(rpvq);
	}
}

static int cvi_rpmsg_find_vqs (struct virtio_device *vdev, unsigned nvqs,
			struct virtqueue *vqs[], vq_callback_t *callbacks[],
			const char * const names[], const bool *ctx,
			struct irq_affinity *desc)
{
	struct cvi_virdev *virdev = to_cvi_virdev(vdev);
	int i, err;

	/* we maintain two virtqueues per remote processor (for RX and TX) */
	if (nvqs != 2)
		return -EINVAL;

	for (i = 0; i < nvqs; ++i) {
		vqs[i] = rp_find_vq(vdev, i, callbacks[i], names[i]);
		if (IS_ERR(vqs[i])) {
			err = PTR_ERR(vqs[i]);
			goto error;
		}
	}
	virdev->num_of_vqs = nvqs;
	return 0;

error:
	cvi_rpmsg_del_vqs(vdev);
	return err;
}

static void cvi_rpmsg_reset(struct virtio_device *vdev)
{
	dev_dbg(&vdev->dev, "reset !\n");
}

static u8 cvi_rpmsg_get_status(struct virtio_device *vdev)
{
	return 0;
}

static void cvi_rpmsg_set_status(struct virtio_device *vdev, u8 status)
{
	dev_dbg(&vdev->dev, "%s new status: %d\n", __func__, status);
}

static void cvi_rpmsg_vproc_release(struct device *dev)
{
	/* this handler is provided so driver core doesn't yell at us */
}

static struct virtio_config_ops cvi_rpmsg_config_ops = {
	.get_features	= cvi_rpmsg_get_features,
	.finalize_features = cvi_rpmsg_finalize_features,
	.find_vqs	= cvi_rpmsg_find_vqs,
	.del_vqs	= cvi_rpmsg_del_vqs,
	.reset		= cvi_rpmsg_reset,
	.set_status	= cvi_rpmsg_set_status,
	.get_status	= cvi_rpmsg_get_status,
};

static struct cvi_rpmsg_vproc cvi_rpmsg_vprocs[] = {
	{
		.rproc_name	= "c906l",
	},
};

static const struct of_device_id cvi_rpmsg_dt_ids[] = {
	{ .compatible = "cvitek,rpmsg",},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cvi_rpmsg_dt_ids);

static int set_vring_phy_buf(struct platform_device *pdev,
		       struct cvi_rpmsg_vproc *rpdev, int vdev_nums)
{
	struct resource *res;
	resource_size_t size;
	unsigned int start, end;
	int i, ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		size = resource_size(res);
		start = res->start;
		end = res->start + size;
		for (i = 0; i < vdev_nums; i++) {
			rpdev->ivdev[i].vring[0] = start;
			rpdev->ivdev[i].vring[1] = start +
						   0x8000;
			start += 0x10000;
			if (start > end) {
				pr_err("Too small memory size %x!\n",
						(u32)size);
				ret = -EINVAL;
				break;
			}
		}
	} else {
		return -ENOMEM;
	}

	return ret;
}

static volatile int rtos_cmd_id;

static void rpmsg_work_handler(struct work_struct *work)
{
	// virtqueue callback
	struct cvi_virdev *cvi_virdev_inst = container_of(vdev_isr, struct cvi_virdev, vdev);
	cvi_virdev_inst->vq[rtos_cmd_id]->callback(cvi_virdev_inst->vq[rtos_cmd_id]);
}

static void rtos_cmdqu_handler(unsigned char cmd_id, unsigned int param_ptr, void* device_id)
{
	rtos_cmd_id = cmd_id;
	schedule_delayed_work(&rpmsg_work, 0);
}

static int cvi_rpmsg_probe(struct platform_device *pdev)
{
	int i, j, ret = 0;
	struct device_node *np = pdev->dev.of_node;

	mbox_reg_rpmsg = (struct mailbox_set_register *) reg_base;
	mailbox_context_rpmsg = (unsigned long *) (reg_base + MAILBOX_CONTEXT_OFFSET);
	INIT_DELAYED_WORK(&rpmsg_work, rpmsg_work_handler);

	for (i = 0; i < ARRAY_SIZE(cvi_rpmsg_vprocs); i++) {
		struct cvi_rpmsg_vproc *rpdev = &cvi_rpmsg_vprocs[i];

		ret = of_property_read_u32_index(np, "vdev-nums", i,
			&rpdev->vdev_nums);
		if (ret)
			rpdev->vdev_nums = 1;
		if (rpdev->vdev_nums > MAX_VDEV_NUMS) {
			pr_err("vdev-nums exceed the max %d\n", MAX_VDEV_NUMS);
			return -EINVAL;
		}

		if (!strcmp(rpdev->rproc_name, "c906l")) {
			ret = set_vring_phy_buf(pdev, rpdev,
						rpdev->vdev_nums);
			if (ret) {
				pr_err("No vring buffer.\n");
				return -ENOMEM;
			}
		} else {
			pr_err("No remote c906l processor.\n");
			return -ENODEV;
		}

		for (j = 0; j < rpdev->vdev_nums; j++) {
			pr_info("%s rpdev%d vdev%d: vring0 0x%x, vring1 0x%x\n",
				 __func__, i, rpdev->vdev_nums,
				 rpdev->ivdev[j].vring[0],
				 rpdev->ivdev[j].vring[1]);
			rpdev->ivdev[j].vdev.id.device = VIRTIO_ID_RPMSG;
			rpdev->ivdev[j].vdev.config = &cvi_rpmsg_config_ops;
			rpdev->ivdev[j].vdev.dev.parent = &pdev->dev;
			rpdev->ivdev[j].vdev.dev.release = cvi_rpmsg_vproc_release;
			rpdev->ivdev[j].base_vq_id = j * 2;

			ret = register_virtio_device(&rpdev->ivdev[j].vdev);
			if (ret) {
				pr_err("%s failed to register rpdev: %d\n",
						__func__, ret);
				return ret;
			}
			vdev_isr = &rpdev->ivdev[j].vdev;
		}
	}

	int err = request_rtos_irq(0, rtos_cmdqu_handler, pdev->name, &pdev->id);
	if (err) {
		pr_err("fail to register interrupt handler: %d \n", err);
		return -1;
	}

	return ret;
}

static struct platform_driver cvi_rpmsg_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "cvi-rpmsg",
		   .of_match_table = cvi_rpmsg_dt_ids,
		   },
	.probe = cvi_rpmsg_probe,
};

static int __init cvi_rpmsg_init(void)
{
	int ret;
	ret = platform_driver_register(&cvi_rpmsg_driver);
	if (ret)
		pr_err("Unable to initialize rpmsg driver\n");

	return ret;
}

MODULE_AUTHOR("CHIANG,KUAN-TING <n96121210@gs.ncku.edu.tw>");
MODULE_DESCRIPTION("cvi remote processor messaging virtio device");
MODULE_LICENSE("GPL v2");
late_initcall(cvi_rpmsg_init);
