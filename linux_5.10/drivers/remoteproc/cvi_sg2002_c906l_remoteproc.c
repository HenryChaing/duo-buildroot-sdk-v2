// SPDX-License-Identifier: GPL-2.0-only
/*
 * ST's Remote Processor Control Driver
 *
 * Copyright (C) 2015 STMicroelectronics - All Rights Reserved
 *
 * Author: Ludovic Barre <ludovic.barre@st.com>
 *
 * Copyright (C) 2025 CHIANG,KUAN-TING - All Rights Reserved
 *
 * Author: CHIANG,KUAN-TING <n96121210@gs.ncku.edu.tw>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include "remoteproc_internal.h"

struct cvi_rproc {
	struct reset_control	*reset;
};

static void *reg_base;

static int cvi_rproc_start(struct rproc *rproc)
{
	struct cvi_rproc *ddata = rproc->priv;
	int err;

	err = reset_control_deassert(ddata->reset);
	if (err) {
		dev_err(&rproc->dev, "Failed to deassert Power Reset\n");
		return err;
	}

	dev_info(&rproc->dev, "Started from 0x%llx\n", rproc->bootaddr);

	return 0;
}

static int cvi_rproc_stop(struct rproc *rproc)
{
	struct cvi_rproc *ddata = rproc->priv;
	int sw_err = 0, pwr_err = 0;

	pwr_err = reset_control_assert(ddata->reset);
	if (pwr_err)
		dev_err(&rproc->dev, "Failed to assert Power Reset\n");

	return sw_err ?: pwr_err;
}

static void *cvi_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len)
{
	return reg_base;
}

static const struct rproc_ops cvi_rproc_ops = {
	.start			= cvi_rproc_start,
	.stop			= cvi_rproc_stop,
	.da_to_va		= cvi_rproc_da_to_va,
};

static const struct of_device_id cvi_rproc_match[] = {
	{ .compatible = "cvi,sg2002-rproc", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, cvi_rproc_match);

static int cvi_rproc_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct cvi_rproc *ddata = rproc->priv;

	ddata->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(ddata->reset)) {
		dev_err(dev, "Failed to get Power Reset\n");
		return PTR_ERR(ddata->reset);
	}

	return 0;
}

static int cvi_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cvi_rproc *ddata;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(dev, np->name, &cvi_rproc_ops, NULL, sizeof(*ddata));
	if (!rproc) {
		return -ENOMEM;
	}
		
	rproc->has_iommu = false;
	rproc->ops->parse_fw = NULL;
	ddata = rproc->priv;

	platform_set_drvdata(pdev, rproc);

	ret = cvi_rproc_parse_dt(pdev);
	if (ret) {
		goto free_rproc;
	}
	
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg_base = devm_ioremap(&pdev->dev, res->start,
		res->end - res->start);
	rproc->auto_boot = false;

	ret = rproc_add(rproc);
	if (ret) {
		goto free_rproc;
	}
		
	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int cvi_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);   
	rproc_free(rproc);

	return 0;
}

static struct platform_driver cvi_rproc_driver = {
	.probe = cvi_rproc_probe,
	.remove = cvi_rproc_remove,
	.driver = {
		.name = "cvi-rproc",
		.of_match_table = of_match_ptr(cvi_rproc_match),
	},
};
module_platform_driver(cvi_rproc_driver);

MODULE_DESCRIPTION("CVI Remote Processor Control Driver");
MODULE_AUTHOR("CHIANG,KUAN-TING <n96121210@gs.ncku.edu.tw>");
MODULE_LICENSE("GPL v2");
