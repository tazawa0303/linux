/**
 * omap-usb-host.c - The USBHS core driver for OMAP EHCI & OHCI
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Author: Keshava Munegowda <keshava_mgowda@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <plat/cpu.h>
#include <plat/usb.h>
#include <linux/pm_runtime.h>

#define USBHS_DRIVER_NAME	"usbhs_omap"
#define OMAP_EHCI_DEVICE	"ehci-omap"
#define OMAP_OHCI_DEVICE	"ohci-omap3"

/* OMAP USBHOST Register addresses  */

/* UHH Register Set */
#define	OMAP_UHH_REVISION				(0x00)
#define	OMAP_UHH_SYSCONFIG				(0x10)
#define	OMAP_UHH_SYSCONFIG_MIDLEMODE			(1 << 12)
#define	OMAP_UHH_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_UHH_SYSCONFIG_SIDLEMODE			(1 << 3)
#define	OMAP_UHH_SYSCONFIG_ENAWAKEUP			(1 << 2)
#define	OMAP_UHH_SYSCONFIG_SOFTRESET			(1 << 1)
#define	OMAP_UHH_SYSCONFIG_AUTOIDLE			(1 << 0)

#define	OMAP_UHH_SYSSTATUS				(0x14)
#define	OMAP_UHH_HOSTCONFIG				(0x40)
#define	OMAP_UHH_HOSTCONFIG_ULPI_BYPASS			(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS		(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS		(1 << 11)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS		(1 << 12)
#define OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN		(1 << 2)
#define OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN		(1 << 3)
#define OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN		(1 << 4)
#define OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN		(1 << 5)
#define OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS		(1 << 8)
#define OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS		(1 << 9)
#define OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS		(1 << 10)
#define OMAP4_UHH_HOSTCONFIG_APP_START_CLK		(1 << 31)

/* OMAP4-specific defines */
#define OMAP4_UHH_SYSCONFIG_IDLEMODE_CLEAR		(3 << 2)
#define OMAP4_UHH_SYSCONFIG_NOIDLE			(1 << 2)
#define OMAP4_UHH_SYSCONFIG_STDBYMODE_CLEAR		(3 << 4)
#define OMAP4_UHH_SYSCONFIG_NOSTDBY			(1 << 4)
#define OMAP4_UHH_SYSCONFIG_SOFTRESET			(1 << 0)

#define OMAP4_P1_MODE_MASK				(3 << 16)
#define OMAP4_P1_MODE_TLL				(1 << 16)
#define OMAP4_P1_MODE_HSIC				(3 << 16)

#define	OMAP_UHH_DEBUG_CSR				(0x44)

#define MAX_HS_USB_PORTS	3	/* Increase this if any chip has more */

/* Values of UHH_REVISION - Note: these are not given in the TRM */
#define OMAP_USBHS_REV1		0x00000010	/* OMAP3 */
#define OMAP_USBHS_REV2		0x50700100	/* OMAP4 */

#define is_omap_usbhs_rev1(x)	(x->usbhs_rev == OMAP_USBHS_REV1)
#define is_omap_usbhs_rev2(x)	(x->usbhs_rev == OMAP_USBHS_REV2)

#define is_ehci_phy_mode(x)	(x == OMAP_EHCI_PORT_MODE_PHY)
#define is_ehci_tll_mode(x)	(x == OMAP_EHCI_PORT_MODE_TLL)
#define is_ehci_hsic_mode(x)	(x == OMAP_EHCI_PORT_MODE_HSIC)

struct usbhs_port {
	struct clk	*utmi_clk;
	struct clk	*hsic60m_clk;
	struct clk	*hsic480m_clk;
};

struct usbhs_hcd_omap {
	int				nports;
	struct usbhs_port		port[MAX_HS_USB_PORTS];

	struct clk			*xclk60mhsp1_ck;
	struct clk			*xclk60mhsp2_ck;
	struct clk			*init_60m_fclk;
	struct clk			*ehci_logic_fck;

	void __iomem			*uhh_base;

	struct usbhs_omap_platform_data	*pdata;

	u32				usbhs_rev;
};
/*-------------------------------------------------------------------------*/

const char usbhs_driver_name[] = USBHS_DRIVER_NAME;
static u64 usbhs_dmamask = DMA_BIT_MASK(32);

/*-------------------------------------------------------------------------*/

static inline void usbhs_write(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 usbhs_read(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

static inline void usbhs_writeb(void __iomem *base, u8 reg, u8 val)
{
	__raw_writeb(val, base + reg);
}

static inline u8 usbhs_readb(void __iomem *base, u8 reg)
{
	return __raw_readb(base + reg);
}

/*-------------------------------------------------------------------------*/

static struct platform_device *omap_usbhs_alloc_child(const char *name,
			struct resource	*res, int num_resources, void *pdata,
			size_t pdata_size, struct device *dev)
{
	struct platform_device	*child;
	int			ret;

	child = platform_device_alloc(name, 0);

	if (!child) {
		dev_err(dev, "platform_device_alloc %s failed\n", name);
		goto err_end;
	}

	ret = platform_device_add_resources(child, res, num_resources);
	if (ret) {
		dev_err(dev, "platform_device_add_resources failed\n");
		goto err_alloc;
	}

	ret = platform_device_add_data(child, pdata, pdata_size);
	if (ret) {
		dev_err(dev, "platform_device_add_data failed\n");
		goto err_alloc;
	}

	child->dev.dma_mask		= &usbhs_dmamask;
	dma_set_coherent_mask(&child->dev, DMA_BIT_MASK(32));
	child->dev.parent		= dev;

	ret = platform_device_add(child);
	if (ret) {
		dev_err(dev, "platform_device_add failed\n");
		goto err_alloc;
	}

	return child;

err_alloc:
	platform_device_put(child);

err_end:
	return NULL;
}

static int omap_usbhs_alloc_children(struct platform_device *pdev)
{
	struct device				*dev = &pdev->dev;
	struct usbhs_hcd_omap			*omap;
	struct ehci_hcd_omap_platform_data	*ehci_data;
	struct ohci_hcd_omap_platform_data	*ohci_data;
	struct platform_device			*ehci;
	struct platform_device			*ohci;
	struct resource				*res;
	struct resource				resources[2];
	int					ret;

	omap = platform_get_drvdata(pdev);
	ehci_data = omap->pdata->ehci_data;
	ohci_data = omap->pdata->ohci_data;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ehci");
	if (!res) {
		dev_err(dev, "EHCI get resource IORESOURCE_MEM failed\n");
		ret = -ENODEV;
		goto err_end;
	}
	resources[0] = *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "ehci-irq");
	if (!res) {
		dev_err(dev, " EHCI get resource IORESOURCE_IRQ failed\n");
		ret = -ENODEV;
		goto err_end;
	}
	resources[1] = *res;

	ehci = omap_usbhs_alloc_child(OMAP_EHCI_DEVICE, resources, 2, ehci_data,
		sizeof(*ehci_data), dev);

	if (!ehci) {
		dev_err(dev, "omap_usbhs_alloc_child failed\n");
		ret = -ENOMEM;
		goto err_end;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ohci");
	if (!res) {
		dev_err(dev, "OHCI get resource IORESOURCE_MEM failed\n");
		ret = -ENODEV;
		goto err_ehci;
	}
	resources[0] = *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "ohci-irq");
	if (!res) {
		dev_err(dev, "OHCI get resource IORESOURCE_IRQ failed\n");
		ret = -ENODEV;
		goto err_ehci;
	}
	resources[1] = *res;

	ohci = omap_usbhs_alloc_child(OMAP_OHCI_DEVICE, resources, 2, ohci_data,
		sizeof(*ohci_data), dev);
	if (!ohci) {
		dev_err(dev, "omap_usbhs_alloc_child failed\n");
		ret = -ENOMEM;
		goto err_ehci;
	}

	return 0;

err_ehci:
	platform_device_unregister(ehci);

err_end:
	return ret;
}

static bool is_ohci_port(enum usbhs_omap_port_mode pmode)
{
	switch (pmode) {
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_PHY_3PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_3PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM:
		return true;

	default:
		return false;
	}
}

static int usbhs_runtime_resume(struct device *dev)
{
	struct usbhs_hcd_omap		*omap = dev_get_drvdata(dev);
	struct usbhs_omap_platform_data *pdata = omap->pdata;
	int i, r;

	dev_dbg(dev, "usbhs_runtime_resume\n");

	omap_tll_enable();

	if (omap->ehci_logic_fck && !IS_ERR(omap->ehci_logic_fck))
		clk_enable(omap->ehci_logic_fck);

	for (i = 0; i < omap->nports; i++) {
		if (is_ehci_tll_mode(pdata->port_mode[i]) ||
				is_ehci_hsic_mode(pdata->port_mode[i])) {
			if (omap->port[i].utmi_clk) {
				r = clk_enable(omap->port[i].utmi_clk);
				if (r) {
					dev_err(dev,
					 "%s: Can't enable port %d clk : %d\n",
					 __func__, i, r);
				}
			}
		}

		/* Enable HSIC clocks if required */
		if (is_ehci_hsic_mode(pdata->port_mode[i])) {
			if (omap->port[i].hsic60m_clk) {
				r = clk_enable(omap->port[i].hsic60m_clk);
				if (r) {
					dev_err(dev,
					 "%s: Can't enable port %d hsic60m clk : %d\n",
					 __func__, i, r);
				}
			}
			if (omap->port[i].hsic480m_clk) {
				r = clk_enable(omap->port[i].hsic480m_clk);
				if (r) {
					dev_err(dev,
					 "%s: Can't enable port %d hsic480m clk : %d\n",
					 __func__, i, r);
				}
			}
		}
	}

	return 0;
}

static int usbhs_runtime_suspend(struct device *dev)
{
	struct usbhs_hcd_omap		*omap = dev_get_drvdata(dev);
	struct usbhs_omap_platform_data *pdata = omap->pdata;
	int i;

	dev_dbg(dev, "usbhs_runtime_suspend\n");

	for (i = 0; i < omap->nports; i++) {
		if (is_ehci_tll_mode(pdata->port_mode[i]) ||
				is_ehci_hsic_mode(pdata->port_mode[i])) {
			if (omap->port[i].utmi_clk)
				clk_disable(omap->port[i].utmi_clk);
		}

		if (is_ehci_hsic_mode(pdata->port_mode[i])) {
			if (omap->port[i].hsic60m_clk)
				clk_disable(omap->port[i].hsic60m_clk);

			if (omap->port[i].hsic480m_clk)
				clk_disable(omap->port[i].hsic480m_clk);
		}
	}

	if (omap->ehci_logic_fck && !IS_ERR(omap->ehci_logic_fck))
		clk_disable(omap->ehci_logic_fck);

	omap_tll_disable();

	return 0;
}

static void omap_usbhs_init(struct device *dev)
{
	struct usbhs_hcd_omap		*omap = dev_get_drvdata(dev);
	struct usbhs_omap_platform_data	*pdata = omap->pdata;
	unsigned			reg;
	int i;

	dev_dbg(dev, "starting TI HSUSB Controller\n");

	if (pdata->ehci_data->phy_reset) {
		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[0]))
			gpio_request_one(pdata->ehci_data->reset_gpio_port[0],
					 GPIOF_OUT_INIT_LOW, "USB1 PHY reset");

		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[1]))
			gpio_request_one(pdata->ehci_data->reset_gpio_port[1],
					 GPIOF_OUT_INIT_LOW, "USB2 PHY reset");

		/* Hold the PHY in RESET for enough time till DIR is high */
		udelay(10);
	}

	pm_runtime_get_sync(dev);

	reg = usbhs_read(omap->uhh_base, OMAP_UHH_HOSTCONFIG);
	/* setup ULPI bypass and burst configurations */
	reg |= (OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN);
	reg |= OMAP4_UHH_HOSTCONFIG_APP_START_CLK;
	reg &= ~OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN;

	if (is_omap_usbhs_rev1(omap)) {
		if (pdata->port_mode[0] == OMAP_USBHS_PORT_MODE_UNUSED)
			reg &= ~OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS;
		if (pdata->port_mode[1] == OMAP_USBHS_PORT_MODE_UNUSED)
			reg &= ~OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS;
		if (pdata->port_mode[2] == OMAP_USBHS_PORT_MODE_UNUSED)
			reg &= ~OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS;

		/* Bypass the TLL module for PHY mode operation */
		if (cpu_is_omap3430() && (omap_rev() <= OMAP3430_REV_ES2_1)) {
			dev_dbg(dev, "OMAP3 ES version <= ES2.1\n");
			if (is_ehci_phy_mode(pdata->port_mode[0]) ||
				is_ehci_phy_mode(pdata->port_mode[1]) ||
					is_ehci_phy_mode(pdata->port_mode[2]))
				reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
			else
				reg |= OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
		} else {
			dev_dbg(dev, "OMAP3 ES version > ES2.1\n");
			if (is_ehci_phy_mode(pdata->port_mode[0]))
				reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;
			else
				reg |= OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;
			if (is_ehci_phy_mode(pdata->port_mode[1]))
				reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;
			else
				reg |= OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;
			if (is_ehci_phy_mode(pdata->port_mode[2]))
				reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;
			else
				reg |= OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;
		}
	} else if (is_omap_usbhs_rev2(omap)) {
		for (i = 0; i < omap->nports; i++) {
			/* Clear port mode fields for PHY mode*/
			reg &= ~(OMAP4_P1_MODE_MASK << 2*i);

			if (is_ehci_tll_mode(pdata->port_mode[i]) ||
				(is_ohci_port(pdata->port_mode[i])))
				reg |= OMAP4_P1_MODE_TLL << 2*i;
			else if (is_ehci_hsic_mode(pdata->port_mode[i]))
				reg |= OMAP4_P1_MODE_HSIC << 2*i;
		}
	}

	usbhs_write(omap->uhh_base, OMAP_UHH_HOSTCONFIG, reg);
	dev_dbg(dev, "UHH setup done, uhh_hostconfig=%x\n", reg);

	pm_runtime_put_sync(dev);
	if (pdata->ehci_data->phy_reset) {
		/* Hold the PHY in RESET for enough time till
		 * PHY is settled and ready
		 */
		udelay(10);

		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[0]))
			gpio_set_value_cansleep
				(pdata->ehci_data->reset_gpio_port[0], 1);

		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[1]))
			gpio_set_value_cansleep
				(pdata->ehci_data->reset_gpio_port[1], 1);
	}
}

static void omap_usbhs_deinit(struct device *dev)
{
	struct usbhs_hcd_omap		*omap = dev_get_drvdata(dev);
	struct usbhs_omap_platform_data	*pdata = omap->pdata;

	if (pdata->ehci_data->phy_reset) {
		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[0]))
			gpio_free(pdata->ehci_data->reset_gpio_port[0]);

		if (gpio_is_valid(pdata->ehci_data->reset_gpio_port[1]))
			gpio_free(pdata->ehci_data->reset_gpio_port[1]);
	}
}


/**
 * usbhs_omap_probe - initialize TI-based HCDs
 *
 * Allocates basic resources for this USB host controller.
 */
static int usbhs_omap_probe(struct platform_device *pdev)
{
	struct device			*dev =  &pdev->dev;
	struct usbhs_omap_platform_data	*pdata = dev->platform_data;
	struct usbhs_hcd_omap		*omap;
	struct resource			*res;
	int				ret = 0;
	int				i;
	bool				need_logic_fck;

	if (!pdata) {
		dev_err(dev, "Missing platform data\n");
		return -ENODEV;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uhh");
	if (!res) {
		dev_err(dev, "UHH EHCI get resource failed\n");
		return -ENODEV;
	}

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	omap->uhh_base = ioremap(res->start, resource_size(res));
	if (!omap->uhh_base) {
		dev_err(dev, "UHH ioremap failed\n");
		ret = -ENOMEM;
		goto err_remap;
	}

	omap->pdata = pdata;
	platform_set_drvdata(pdev, omap);
	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	omap->usbhs_rev = usbhs_read(omap->uhh_base, OMAP_UHH_REVISION);

	/* we need to call runtime suspend before we update omap->nports
	 * to prevent unbalanced clk_disable()
	 */
	pm_runtime_put_sync(dev);

	/*
	 * If platform data contains nports then use that
	 * else make out number of ports from USBHS revision
	 */
	if (pdata->nports) {
		if (omap->nports > MAX_HS_USB_PORTS) {
			dev_err(dev,
			 "Platform data says %d ports but MAX_HS_USB_PORTS is %d\n",
			 omap->nports, MAX_HS_USB_PORTS);
		} else {
			omap->nports = pdata->nports;
		}
	} else {
		switch (omap->usbhs_rev) {
		case OMAP_USBHS_REV1:
			omap->nports = 3;
			break;
		case OMAP_USBHS_REV2:
		/* Both OMAP4 and 5 show the same revision but they have
		 * different number of ports i.e. 2 and 3 respectively.
		 * OMAP5 platforms must supply nports via platform data.
		 */
			omap->nports = 2;
			break;
		default:
			omap->nports = MAX_HS_USB_PORTS;
			dev_info(dev,
			"USB HOST Rev:0x%d not recognized, assuming %d ports\n",
				omap->usbhs_rev, omap->nports);
			break;
		}
	}

	need_logic_fck = false;
	for (i = 0; i < omap->nports; i++) {
		if (is_ehci_phy_mode(i) || is_ehci_tll_mode(i) ||
			is_ehci_hsic_mode(i))
				need_logic_fck |= true;
	}

	if (need_logic_fck) {
		omap->ehci_logic_fck = clk_get(dev, "ehci_logic_fck");
		if (IS_ERR(omap->ehci_logic_fck)) {
			ret = PTR_ERR(omap->ehci_logic_fck);
			dev_warn(dev, "ehci_logic_fck failed:%d\n", ret);
		}
	}

	omap->xclk60mhsp1_ck = clk_get(dev, "xclk60mhsp1_ck");
	if (IS_ERR(omap->xclk60mhsp1_ck)) {
		ret = PTR_ERR(omap->xclk60mhsp1_ck);
		dev_err(dev, "xclk60mhsp1_ck failed error:%d\n", ret);
		goto err_xclk60mhsp1;
	}

	omap->xclk60mhsp2_ck = clk_get(dev, "xclk60mhsp2_ck");
	if (IS_ERR(omap->xclk60mhsp2_ck)) {
		ret = PTR_ERR(omap->xclk60mhsp2_ck);
		dev_err(dev, "xclk60mhsp2_ck failed error:%d\n", ret);
		goto err_xclk60mhsp2;
	}

	omap->init_60m_fclk = clk_get(dev, "init_60m_fclk");
	if (IS_ERR(omap->init_60m_fclk)) {
		ret = PTR_ERR(omap->init_60m_fclk);
		dev_err(dev, "init_60m_fclk failed error:%d\n", ret);
		goto err_init60m;
	}

	for (i = 0; i < omap->nports; i++) {
		struct clk *pclk;
		char utmi_clk[] = "usb_host_hs_utmi_px_clk";
		char hsic_clk[] = "usb_host_hs_hsic480m_px_clk";

		/* clock names are indexed from 1*/
		sprintf(utmi_clk, "usb_host_hs_utmi_p%d_clk", i + 1);

		/* If a clock is not found we won't bail out as not all
		 * platforms have all clocks and we can function without
		 * them
		 */
		pclk = clk_get(dev, utmi_clk);
		if (IS_ERR(pclk))
			dev_err(dev, "Failed to get clock : %s : %ld\n",
				utmi_clk, PTR_ERR(pclk));
		else
			omap->port[i].utmi_clk = pclk;

		sprintf(hsic_clk, "usb_host_hs_hsic480m_p%d_clk", i + 1);
		pclk = clk_get(dev, hsic_clk);
		if (IS_ERR(pclk))
			dev_err(dev, "Failed to get clock : %s : %ld\n",
				hsic_clk, PTR_ERR(pclk));
		else
			omap->port[i].hsic480m_clk = pclk;

		sprintf(hsic_clk, "usb_host_hs_hsic60m_p%d_clk", i + 1);
		pclk = clk_get(dev, hsic_clk);
		if (IS_ERR(pclk))
			dev_err(dev, "Failed to get clock : %s : %ld\n",
				hsic_clk, PTR_ERR(pclk));
		else
			omap->port[i].hsic60m_clk = pclk;
	}

	if (is_ehci_phy_mode(pdata->port_mode[0])) {
		/* for OMAP3 , the clk set paretn fails */
		ret = clk_set_parent(omap->port[0].utmi_clk,
					omap->xclk60mhsp1_ck);
		if (ret != 0)
			dev_err(dev, "xclk60mhsp1_ck set parent"
				"failed error:%d\n", ret);
	} else if (is_ehci_tll_mode(pdata->port_mode[0])) {
		ret = clk_set_parent(omap->port[0].utmi_clk,
					omap->init_60m_fclk);
		if (ret != 0)
			dev_err(dev, "init_60m_fclk set parent"
				"failed error:%d\n", ret);
	}

	if (is_ehci_phy_mode(pdata->port_mode[1])) {
		ret = clk_set_parent(omap->port[1].utmi_clk,
					omap->xclk60mhsp2_ck);
		if (ret != 0)
			dev_err(dev, "xclk60mhsp2_ck set parent"
					"failed error:%d\n", ret);
	} else if (is_ehci_tll_mode(pdata->port_mode[1])) {
		ret = clk_set_parent(omap->port[1].utmi_clk,
						omap->init_60m_fclk);
		if (ret != 0)
			dev_err(dev, "init_60m_fclk set parent"
				"failed error:%d\n", ret);
	}


	omap_usbhs_init(dev);
	ret = omap_usbhs_alloc_children(pdev);
	if (ret) {
		dev_err(dev, "omap_usbhs_alloc_children failed\n");
		goto err_alloc;
	}

	pr_info("OMAP USB HOST : revision 0x%x, ports %d\n",
			omap->usbhs_rev, omap->nports);
	return 0;

err_alloc:
	omap_usbhs_deinit(&pdev->dev);
	iounmap(omap->uhh_base);

	for (i = 0; i < omap->nports; i++) {
		clk_put(omap->port[i].utmi_clk);
		clk_put(omap->port[i].hsic60m_clk);
		clk_put(omap->port[i].hsic480m_clk);
	}

	clk_put(omap->init_60m_fclk);

err_init60m:
	clk_put(omap->xclk60mhsp2_ck);

err_xclk60mhsp2:
	clk_put(omap->xclk60mhsp1_ck);

err_xclk60mhsp1:
	clk_put(omap->ehci_logic_fck);

	pm_runtime_disable(dev);

err_remap:
	kfree(omap);
	return ret;
}

/**
 * usbhs_omap_remove - shutdown processing for UHH & TLL HCDs
 * @pdev: USB Host Controller being removed
 *
 * Reverses the effect of usbhs_omap_probe().
 */
static int usbhs_omap_remove(struct platform_device *pdev)
{
	struct usbhs_hcd_omap *omap = platform_get_drvdata(pdev);
	int i;

	omap_usbhs_deinit(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	iounmap(omap->uhh_base);

	for (i = 0; i < omap->nports; i++) {
		clk_put(omap->port[i].utmi_clk);
		clk_put(omap->port[i].hsic60m_clk);
		clk_put(omap->port[i].hsic480m_clk);
	}

	clk_put(omap->init_60m_fclk);
	clk_put(omap->xclk60mhsp2_ck);
	clk_put(omap->xclk60mhsp1_ck);
	clk_put(omap->ehci_logic_fck);

	kfree(omap);

	return 0;
}

static const struct dev_pm_ops usbhsomap_dev_pm_ops = {
	.runtime_suspend	= usbhs_runtime_suspend,
	.runtime_resume		= usbhs_runtime_resume,
};

static struct platform_driver usbhs_omap_driver = {
	.driver = {
		.name		= (char *)usbhs_driver_name,
		.owner		= THIS_MODULE,
		.pm		= &usbhsomap_dev_pm_ops,
	},
	.remove		= __exit_p(usbhs_omap_remove),
};

MODULE_AUTHOR("Keshava Munegowda <keshava_mgowda@ti.com>");
MODULE_ALIAS("platform:" USBHS_DRIVER_NAME);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("usb host common core driver for omap EHCI and OHCI");

static int __init omap_usbhs_drvinit(void)
{
	return platform_driver_probe(&usbhs_omap_driver, usbhs_omap_probe);
}

/*
 * init before ehci and ohci drivers;
 * The usbhs core driver should be initialized much before
 * the omap ehci and ohci probe functions are called.
 * This usbhs core driver should be initialized after
 * usb tll driver
 */
fs_initcall_sync(omap_usbhs_drvinit);

static void __exit omap_usbhs_drvexit(void)
{
	platform_driver_unregister(&usbhs_omap_driver);
}
module_exit(omap_usbhs_drvexit);
