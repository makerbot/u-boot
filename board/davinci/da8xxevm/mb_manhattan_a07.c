/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Based on da830evm.c. Original Copyrights follow:
 *
 * Copyright (C) 2009 Nick Thompson, GE Fanuc, Ltd. <nick.thompson@gefanuc.com>
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>
#include <i2c.h>
#include <net.h>
#include <netdev.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/arch/hardware.h>
#include <asm/arch/emif_defs.h>
#include <asm/arch/emac_defs.h>
#include <asm/io.h>
#include <asm/errno.h>
#include "../common/misc.h"
#include "common.h"
#ifdef CONFIG_DAVINCI_MMC
#include <mmc.h>
#include <asm/arch/sdmmc_defs.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define pinmux(x)	(&davinci_syscfg_regs->pinmux[x])

/* SPI0 pin muxer settings */
static const struct pinmux_config spi1_pins[] = {
	{ pinmux(5), 1, 1 },
	{ pinmux(5), 1, 2 },
	{ pinmux(5), 1, 4 },
	{ pinmux(5), 1, 5 }
};

#ifdef CONFIG_DAVINCI_MMC
/* MMC0 pin muxer settings */
const struct pinmux_config mmc0_pins[] = {
	/* GP0[11] is required for SD to work on Rev 3 EVMs */
	{ pinmux(0),  8, 4 },	/* GP0[11] */
	{ pinmux(10), 2, 0 },	/* MMCSD0_CLK */
	{ pinmux(10), 2, 1 },	/* MMCSD0_CMD */
	{ pinmux(10), 2, 2 },	/* MMCSD0_DAT_0 */
	{ pinmux(10), 2, 3 },	/* MMCSD0_DAT_1 */
	{ pinmux(10), 2, 4 },	/* MMCSD0_DAT_2 */
	{ pinmux(10), 2, 5 },	/* MMCSD0_DAT_3 */
	/* DA850 supports only 4-bit mode, remaining pins are not configured */
};
#endif

/* UART pin muxer settings */
static const struct pinmux_config uart_pins[] = {
	{ pinmux(0), 4, 4 },
	{ pinmux(0), 4, 5 },
	{ pinmux(4), 2, 6 },
	{ pinmux(4), 2, 7 }
};

#ifdef CONFIG_DRIVER_TI_EMAC
static const struct pinmux_config emac_pins[] = {
	{ pinmux(2), 8, 1 },
	{ pinmux(2), 8, 2 },
	{ pinmux(2), 8, 3 },
	{ pinmux(2), 8, 4 },
	{ pinmux(2), 8, 5 },
	{ pinmux(2), 8, 6 },
	{ pinmux(2), 8, 7 },
	{ pinmux(3), 8, 0 },
	{ pinmux(3), 8, 1 },
	{ pinmux(3), 8, 2 },
	{ pinmux(3), 8, 3 },
	{ pinmux(3), 8, 4 },
	{ pinmux(3), 8, 5 },
	{ pinmux(3), 8, 6 },
	{ pinmux(3), 8, 7 },
	{ pinmux(4), 8, 0 },
	{ pinmux(4), 8, 1 }
};
#endif /* CONFIG_DRIVER_TI_EMAC */

/* I2C pin muxer settings */
static const struct pinmux_config i2c_pins[] = {
	{ pinmux(4), 2, 2 },
	{ pinmux(4), 2, 3 }
};

#ifdef CONFIG_NAND_DAVINCI
const struct pinmux_config nand_pins[] = {
  { pinmux(7), 1, 1 }, // CS[3]
//	{ pinmux(7), 1, 2 },
	{ pinmux(7), 1, 4 }, // WE
	{ pinmux(7), 1, 5 }, // OE
  { pinmux(7), 1, 7 }, // RB
	{ pinmux(9), 1, 0 }, // D[7]
	{ pinmux(9), 1, 1 }, // D[6]
	{ pinmux(9), 1, 2 }, // D[5]
	{ pinmux(9), 1, 3 }, // D[4]
	{ pinmux(9), 1, 4 }, // D[3]
	{ pinmux(9), 1, 5 }, // D[2]
	{ pinmux(9), 1, 6 }, // D[1]
	{ pinmux(9), 1, 7 }, // D[2]
	{ pinmux(12), 1, 5 }, // A[2] CLE
	{ pinmux(12), 1, 6 } // A[1] ALE
};
#elif defined (CONFIG_SYS_USE_NOR)
const struct pinmux_config nor_pins[] = {
	/* GP0[11] is required for SD to work on Rev 3 EVMs */
	{ pinmux(0), 8, 4 },	/* GP0[11] */
	{ pinmux(5), 1, 6 },
	{ pinmux(6), 1, 6 },
	{ pinmux(7), 1, 0 },
	{ pinmux(7), 1, 4 },
	{ pinmux(7), 1, 5 },
	{ pinmux(8), 1, 0 },
	{ pinmux(8), 1, 1 },
	{ pinmux(8), 1, 2 },
	{ pinmux(8), 1, 3 },
	{ pinmux(8), 1, 4 },
	{ pinmux(8), 1, 5 },
	{ pinmux(8), 1, 6 },
	{ pinmux(8), 1, 7 },
	{ pinmux(9), 1, 0 },
	{ pinmux(9), 1, 1 },
	{ pinmux(9), 1, 2 },
	{ pinmux(9), 1, 3 },
	{ pinmux(9), 1, 4 },
	{ pinmux(9), 1, 5 },
	{ pinmux(9), 1, 6 },
	{ pinmux(9), 1, 7 },
	{ pinmux(10), 1, 0 },
	{ pinmux(10), 1, 1 },
	{ pinmux(10), 1, 2 },
	{ pinmux(10), 1, 3 },
	{ pinmux(10), 1, 4 },
	{ pinmux(10), 1, 5 },
	{ pinmux(10), 1, 6 },
	{ pinmux(10), 1, 7 },
	{ pinmux(11), 1, 0 },
	{ pinmux(11), 1, 1 },
	{ pinmux(11), 1, 2 },
	{ pinmux(11), 1, 3 },
	{ pinmux(11), 1, 4 },
	{ pinmux(11), 1, 5 },
	{ pinmux(11), 1, 6 },
	{ pinmux(11), 1, 7 },
	{ pinmux(12), 1, 0 },
	{ pinmux(12), 1, 1 },
	{ pinmux(12), 1, 2 },
	{ pinmux(12), 1, 3 },
	{ pinmux(12), 1, 4 },
	{ pinmux(12), 1, 5 },
	{ pinmux(12), 1, 6 },
	{ pinmux(12), 1, 7 }
};
#endif

#ifdef CONFIG_DRIVER_TI_EMAC_USE_RMII
#define HAS_RMII 1
#else
#define HAS_RMII 0
#endif

static const struct pinmux_resource pinmuxes[] = {
#ifdef CONFIG_SPI_FLASH
	PINMUX_ITEM(spi1_pins),
#endif
	PINMUX_ITEM(uart_pins),
	PINMUX_ITEM(i2c_pins),
#ifdef CONFIG_NAND_DAVINCI
	PINMUX_ITEM(nand_pins),
#elif defined(CONFIG_USE_NOR)
	PINMUX_ITEM(nor_pins),
#endif
};

static const struct lpsc_resource lpsc[] = {
	{ DAVINCI_LPSC_AEMIF },	/* NAND, NOR */
	{ DAVINCI_LPSC_SPI1 },	/* Serial Flash */
	{ DAVINCI_LPSC_EMAC },	/* image download */
	{ DAVINCI_LPSC_UART1 },	/* console */
	{ DAVINCI_LPSC_GPIO },
#ifdef CONFIG_DAVINCI_MMC
	{ DAVINCI_LPSC_MMC_SD },
#endif		
};

#ifndef CONFIG_DA850_EVM_MAX_CPU_CLK
#define CONFIG_DA850_EVM_MAX_CPU_CLK	300000000
#endif

u32 get_board_type(void)
{
	if (i2c_probe(0x50) == 0)
		return 1;
	else
		return 0;
}

/*
 * get_board_rev() - setup to pass kernel board revision information
 * Returns:
 * bit[0-3]	Maximum cpu clock rate supported by onboard SoC
 *		0000b - 300 MHz
 *		0001b - 372 MHz
 *		0010b - 408 MHz
 *		0011b - 456 MHz
 */
u32 get_board_rev(void)
{
	char *s;
	u32 maxcpuclk = CONFIG_DA850_EVM_MAX_CPU_CLK;
	u32 rev = 0;
	u32 btype;

	s = getenv("maxcpuclk");
	if (s)
		maxcpuclk = simple_strtoul(s, NULL, 10);

	if (maxcpuclk >= 456000000)
		rev = 3;
	else if (maxcpuclk >= 408000000)
		rev = 2;
	else if (maxcpuclk >= 372000000)
		rev = 1;

	btype = get_board_type();

	if (btype == 1)
		rev |= 0x100;

	return rev;
}

int board_init(void)
{
	//unsigned int temp;
#ifndef CONFIG_USE_IRQ
	irq_init();
#endif


#ifdef CONFIG_NAND_DAVINCI
	/*
	 * NAND CS setup - cycle counts based on da850evm NAND timings in the
	 * Linux kernel @ 25MHz EMIFA
	 */
	writel((DAVINCI_ABCR_WSETUP(2) |
		DAVINCI_ABCR_WSTROBE(2) |
		DAVINCI_ABCR_WHOLD(1) |
		DAVINCI_ABCR_RSETUP(1) |
		DAVINCI_ABCR_RSTROBE(4) |
		DAVINCI_ABCR_RHOLD(0) |
		DAVINCI_ABCR_TA(1) |
		DAVINCI_ABCR_ASIZE_8BIT),
	       &davinci_emif_regs->ab2cr); /* CS3 */
#endif

	/* arch number of the board */
	gd->bd->bi_arch_number = MACH_TYPE_DAVINCI_MANHATTAN_A07;

	/* address of boot parameters */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	/*
	 * Power on required peripherals
	 * ARM does not have access by default to PSC0 and PSC1
	 * assuming here that the DSP bootloader has set the IOPU
	 * such that PSC access is available to ARM
	 */
	if (da8xx_configure_lpsc_items(lpsc, ARRAY_SIZE(lpsc)))
		return 1;

	/* setup the SUSPSRC for ARM to control emulation suspend */
	writel(readl(&davinci_syscfg_regs->suspsrc) &
	       ~(DAVINCI_SYSCFG_SUSPSRC_EMAC | DAVINCI_SYSCFG_SUSPSRC_I2C |
		 DAVINCI_SYSCFG_SUSPSRC_SPI1 | DAVINCI_SYSCFG_SUSPSRC_TIMER0 |
		 DAVINCI_SYSCFG_SUSPSRC_UART1),
	       &davinci_syscfg_regs->suspsrc);

	/* configure pinmux settings */
	if (davinci_configure_pin_mux_items(pinmuxes, ARRAY_SIZE(pinmuxes)))
		return 1;

#ifdef CONFIG_SYS_USE_NOR	
	/* Set the GPIO direction as output */
	temp = REG(GPIO_BANK0_REG_DIR_ADDR);
	temp &= ~(0x01 << 11);
	REG(GPIO_BANK0_REG_DIR_ADDR) = temp;

	/* Set the output as low */
	temp = REG(GPIO_BANK0_REG_SET_ADDR);
	temp |= (0x01 << 11);
	REG(GPIO_BANK0_REG_CLR_ADDR) = temp;
#endif	

#ifdef CONFIG_DAVINCI_MMC
	if (davinci_configure_pin_mux(mmc0_pins, ARRAY_SIZE(mmc0_pins)) != 0)
		return 1;

	/* Set the GPIO direction as output */
	temp = REG(GPIO_BANK0_REG_DIR_ADDR);
	temp &= ~(0x01 << 11);
	REG(GPIO_BANK0_REG_DIR_ADDR) = temp;

	/* Set the output as high */
	temp = REG(GPIO_BANK0_REG_SET_ADDR);
	temp |= (0x01 << 11);
	REG(GPIO_BANK0_REG_SET_ADDR) = temp;
#endif

#ifdef CONFIG_DRIVER_TI_EMAC
	if (davinci_configure_pin_mux(emac_pins, ARRAY_SIZE(emac_pins)) != 0)
		return 1;
	da850_emac_mii_mode_sel(HAS_RMII);
#endif /* CONFIG_DRIVER_TI_EMAC */

	/* enable the console UART */
	writel((DAVINCI_UART_PWREMU_MGMT_FREE | DAVINCI_UART_PWREMU_MGMT_URRST |
		DAVINCI_UART_PWREMU_MGMT_UTRST),
	       &davinci_uart1_ctrl_regs->pwremu_mgmt);

	return 0;
}

#ifdef CONFIG_DRIVER_TI_EMAC

/*
 * Initializes on-board ethernet controllers.
 */
int board_eth_init(bd_t *bis)
{
	if (!davinci_emac_initialize()) {
		printf("Error: Ethernet init failed!\n");
		return -1;
	}

	return 0;
}

#endif /* CONFIG_DRIVER_TI_EMAC */

#define CFG_MAC_ADDR_SPI_BUS	0
#define CFG_MAC_ADDR_SPI_CS	0
#define CFG_MAC_ADDR_SPI_MAX_HZ	CONFIG_SF_DEFAULT_SPEED
#define CFG_MAC_ADDR_SPI_MODE	SPI_MODE_3

#define CFG_MAC_ADDR_OFFSET	(flash->size - SZ_64K)

static int  get_mac_addr(u8 *addr)
{

#ifdef SPI_FLASH
	int ret;
	struct spi_flash *flash;

	flash = spi_flash_probe(CFG_MAC_ADDR_SPI_BUS, CFG_MAC_ADDR_SPI_CS,
			CFG_MAC_ADDR_SPI_MAX_HZ, CFG_MAC_ADDR_SPI_MODE);
	if (!flash) {
		printf(" Error - unable to probe SPI flash.\n");
		goto err_probe;
	}

	ret = spi_flash_read(flash, CFG_MAC_ADDR_OFFSET, 6, addr);
	if (ret) {
		printf("Error - unable to read MAC address from SPI flash.\n");
		goto err_read;
	}

err_read:
	/* cannot call free currently since the free function calls free() for
	 * spi_flash structure though it is not directly allocated through
	 * malloc()
	 */
	/* spi_flash_free(flash); */
err_probe:
	return ret;
#else
  /*we need to put in a function to get the MAC address from the nand environment? */
  return 0;
#endif

}

void dsp_lpsc_on(unsigned domain, unsigned int id)
{
	dv_reg_p mdstat, mdctl, ptstat, ptcmd;
	struct davinci_psc_regs *psc_regs;

	psc_regs = davinci_psc0_regs;
	mdstat = &psc_regs->psc0.mdstat[id];
	mdctl = &psc_regs->psc0.mdctl[id];
	ptstat = &psc_regs->ptstat;
	ptcmd = &psc_regs->ptcmd;

	while (*ptstat & (0x1 << domain))
		;

	if ((*mdstat & 0x1f) == 0x03)
		return;                 /* Already on and enabled */

	*mdctl |= 0x03;

	*ptcmd = 0x1 << domain;

	while (*ptstat & (0x1 << domain))
		;
	while ((*mdstat & 0x1f) != 0x03)
		;		/* Probably an overkill... */
}

static void dspwake(void)
{
	unsigned *resetvect = (unsigned *)DAVINCI_L3CBARAM_BASE;

	/* if the device is ARM only, return */
	if ((REG(CHIP_REV_ID_REG) & 0x3f) == 0x10)
		return;

	if (!strcmp(getenv("dspwake"), "no"))
		return;

	*resetvect++ = 0x1E000; /* DSP Idle */
	/* clear out the next 10 words as NOP */
	memset(resetvect, 0, sizeof(unsigned) * 10);

	/* setup the DSP reset vector */
	REG(HOST1CFG) = DAVINCI_L3CBARAM_BASE;

	dsp_lpsc_on(1, DAVINCI_LPSC_GEM);
	REG(PSC0_MDCTL + (15 * 4)) |= 0x100;
}

#ifdef CONFIG_DRIVER_TI_EMAC_USE_RMII
/**
 * rmii_hw_init
 *
 * DA850/OMAP-L138 EVM can interface to a daughter card for
 * additional features. This card has an I2C GPIO Expander TCA6416
 * to select the required functions like camera, RMII Ethernet,
 * character LCD, video.
 *
 * Initialization of the expander involves configuring the
 * polarity and direction of the ports. P07-P05 are used here.
 * These ports are connected to a Mux chip which enables only one
 * functionality at a time.
 *
 * For RMII phy to respond, the MII MDIO clock has to be  disabled
 * since both the PHY devices have address as zero. The MII MDIO
 * clock is controlled via GPIO2[6].
 *
 * This code is valid for Beta version of the hardware
 */
int rmii_hw_init(void)
{
	const struct pinmux_config gpio_pins[] = {
		{ pinmux(6), 8, 1 }
	};
	u_int8_t buf[2];
	unsigned int temp;
	int ret;

	/* PinMux for GPIO */
	if (davinci_configure_pin_mux(gpio_pins, ARRAY_SIZE(gpio_pins)) != 0)
		return 1;

	/* I2C Exapnder configuration */
	/* Set polarity to non-inverted */
	buf[0] = 0x0;
	buf[1] = 0x0;
	ret = i2c_write(CONFIG_SYS_I2C_EXPANDER_ADDR, 4, 1, buf, 2);
	if (ret) {
		printf("\nExpander @ 0x%02x write FAILED!!!\n",
				CONFIG_SYS_I2C_EXPANDER_ADDR);
		return ret;
	}

	/* Configure P07-P05 as outputs */
	buf[0] = 0x1f;
	buf[1] = 0xff;
	ret = i2c_write(CONFIG_SYS_I2C_EXPANDER_ADDR, 6, 1, buf, 2);
	if (ret) {
		printf("\nExpander @ 0x%02x write FAILED!!!\n",
				CONFIG_SYS_I2C_EXPANDER_ADDR);
	}

	/* For Ethernet RMII selection
	 * P07(SelA)=0
	 * P06(SelB)=1
	 * P05(SelC)=1
	 */
	if (i2c_read(CONFIG_SYS_I2C_EXPANDER_ADDR, 2, 1, buf, 1)) {
		printf("\nExpander @ 0x%02x read FAILED!!!\n",
				CONFIG_SYS_I2C_EXPANDER_ADDR);
	}

	buf[0] &= 0x1f;
	buf[0] |= (0 << 7) | (1 << 6) | (1 << 5);
	if (i2c_write(CONFIG_SYS_I2C_EXPANDER_ADDR, 2, 1, buf, 1)) {
		printf("\nExpander @ 0x%02x write FAILED!!!\n",
				CONFIG_SYS_I2C_EXPANDER_ADDR);
	}

	/* Set the output as high */
	temp = REG(GPIO_BANK2_REG_SET_ADDR);
	temp |= (0x01 << 6);
	REG(GPIO_BANK2_REG_SET_ADDR) = temp;

	/* Set the GPIO direction as output */
	temp = REG(GPIO_BANK2_REG_DIR_ADDR);
	temp &= ~(0x01 << 6);
	REG(GPIO_BANK2_REG_DIR_ADDR) = temp;

	return 0;
}
#endif /* CONFIG_DRIVER_TI_EMAC_USE_RMII */

int misc_init_r(void)
{
	uint8_t tmp[20], addr[10];

	printf("ARM Clock : %d Hz\n", clk_get(DAVINCI_ARM_CLKID));
	printf("DDR Clock : %d Hz\n", clk_get(DAVINCI_DDR_CLKID)/2);

	if (getenv("ethaddr") == NULL) {
		/* Read Ethernet MAC address from EEPROM */
		if (dvevm_read_mac_address(addr)) {
			/* Set Ethernet MAC address from EEPROM */
			davinci_sync_env_enetaddr(addr);
		} else {
			get_mac_addr(addr);
		}

		if (is_multicast_ether_addr(addr) || is_zero_ether_addr(addr)) {
			printf("Invalid MAC address read.\n");
			return -EINVAL;
		}
		sprintf((char *)tmp, "%02x:%02x:%02x:%02x:%02x:%02x", addr[0],
				addr[1], addr[2], addr[3], addr[4], addr[5]);

		setenv("ethaddr", (char *)tmp);
	}
#ifdef CONFIG_DRIVER_TI_EMAC_USE_RMII
	/* Select RMII fucntion through the expander */
	if (rmii_hw_init())
		printf("RMII hardware init failed!!!\n");
#endif

	dspwake();

	return 0;
}

#ifdef CONFIG_DAVINCI_MMC
static struct davinci_mmc mmc_sd0 = {
	.reg_base = (struct davinci_mmc_regs *)DAVINCI_MMC_SD0_BASE,
	.host_caps = MMC_MODE_4BIT,     /* DA850 supports only 4-bit SD/MMC */
	.voltages = MMC_VDD_32_33 | MMC_VDD_33_34,
	.version = MMC_CTLR_VERSION_2,
};

int board_mmc_init(bd_t *bis)
{
	mmc_sd0.input_clk = clk_get(DAVINCI_MMCSD_CLKID);

	/* Add slot-0 to mmc subsystem */
	return davinci_mmc_init(bis, &mmc_sd0);
}
#endif

#ifdef CONFIG_USB_DAVINCI
//int usb_phy_on(void)
//{
//	u32 timeout;
//	u32 cfgchip2;
//
//	cfgchip2 = readl(&davinci_syscfg_regs->cfgchip2);
//
//	cfgchip2 &= ~(CFGCHIP2_RESET | CFGCHIP2_PHYPWRDN | CFGCHIP2_OTGPWRDN |
//		      CFGCHIP2_OTGMODE | CFGCHIP2_REFFREQ |
//		      CFGCHIP2_USB1PHYCLKMUX);
//	cfgchip2 |= CFGCHIP2_SESENDEN | CFGCHIP2_VBDTCTEN | CFGCHIP2_PHY_PLLON |
//		    CFGCHIP2_REFFREQ_24MHZ | CFGCHIP2_USB2PHYCLKMUX |
//		    CFGCHIP2_USB1SUSPENDM;
//
//	writel(cfgchip2, &davinci_syscfg_regs->cfgchip2);
//
//	/* wait until the usb phy pll locks */
//	timeout = DA8XX_USB_OTG_TIMEOUT;
//	while (timeout--)
//		if (readl(&davinci_syscfg_regs->cfgchip2) & CFGCHIP2_PHYCLKGD)
//			return 1;
//
//	/* USB phy was not turned on */
//	return 0;
//}


//void usb_phy_off(void)
//{
//	u32 cfgchip2;
//
//	/*
//	 * Power down the on-chip PHY.
//	 */
//	cfgchip2 = readl(&davinci_syscfg_regs->cfgchip2);
//	cfgchip2 &= ~(CFGCHIP2_PHY_PLLON | CFGCHIP2_USB1SUSPENDM);
//	cfgchip2 |= CFGCHIP2_PHYPWRDN | CFGCHIP2_OTGPWRDN | CFGCHIP2_RESET;
//	writel(cfgchip2, &davinci_syscfg_regs->cfgchip2);
//}
#warning code for CONFIG_USB_DAVINCIis elsewhere
#else /*CONFIG_USB_DAVINCI*/
#error MUST Enable CONFIG_USB_DAVINCI
#endif /*CONFIG_USB_DAVINCI*/
