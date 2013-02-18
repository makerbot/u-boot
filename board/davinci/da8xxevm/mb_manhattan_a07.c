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
#include <net.h>
#include <netdev.h>
#include <asm/arch/hardware.h>
#include <asm/arch/emif_defs.h>
#include <asm/arch/emac_defs.h>
#include <asm/arch/pinmux_defs.h>
#include <asm/io.h>
#include <asm/arch/davinci_misc.h>
#include <asm/errno.h>
#include <hwconfig.h>

DECLARE_GLOBAL_DATA_PTR;


static int get_mac_addr(u8 *addr)
{

/* we need to put in a function to get the MAC address from the nand environment
   * temporarily, we hardcode the MAC Address*/

  addr[0] = 26;
  addr[1] = 15;
  addr[2] = 99;
  addr[3] = 105;
  addr[4] = 52;
  addr[5] = 230;
  return 0;

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
	u32 val;

	/* if the device is ARM only, return */
	if ((readl(CHIP_REV_ID_REG) & 0x3f) == 0x10)
		return;

	if (hwconfig_subarg_cmp_f("dsp", "wake", "no", NULL))
		return;

	*resetvect++ = 0x1E000; /* DSP Idle */
	/* clear out the next 10 words as NOP */
	memset(resetvect, 0, sizeof(unsigned) *10);

	/* setup the DSP reset vector */
	writel(DAVINCI_L3CBARAM_BASE, HOST1CFG);

	dsp_lpsc_on(1, DAVINCI_LPSC_GEM);
	val = readl(PSC0_MDCTL + (15 * 4));
	val |= 0x100;
	writel(val, (PSC0_MDCTL + (15 * 4)));
}

int misc_init_r(void)
{

  uint8_t tmp[20], addr[10];

  if (getenv("ethaddr") == NULL) {
    get_mac_addr(addr);
  }

  if (is_multicast_ether_addr(addr) || is_zero_ether_addr(addr)) {
      printf("Invalid MAC address read.\n");
      return -EINVAL;
  }
  sprintf((char *)tmp, "%02x:%02x:%02x:%02x:%02x:%02x", addr[0],
        addr[1], addr[2], addr[3], addr[4], addr[5]);

  eth_setenv_enetaddr("ethaddr", (char *)tmp);
	dspwake();

	return 0;
}


const struct pinmux_resource pinmuxes[] = {
	PINMUX_ITEM(emac_pins_mdio),
	PINMUX_ITEM(emac_pins_mii),
	PINMUX_ITEM(uart1_pins_txrx),
	PINMUX_ITEM(uart1_pins_rtscts),
	PINMUX_ITEM(emifa_pins_cs3),
	PINMUX_ITEM(emifa_pins_nand),
};

const int pinmuxes_size = ARRAY_SIZE(pinmuxes);

const struct lpsc_resource lpsc[] = {
	{ DAVINCI_LPSC_AEMIF },	/* NAND, NOR */
	{ DAVINCI_LPSC_EMAC },	/* image download */
	{ DAVINCI_LPSC_UART1 },	/* console */
	{ DAVINCI_LPSC_GPIO },
};

const int lpsc_size = ARRAY_SIZE(lpsc);

#ifndef CONFIG_DA850_EVM_MAX_CPU_CLK
#define CONFIG_DA850_EVM_MAX_CPU_CLK	300000000
#endif

#define REV_AM18X_EVM		0x100

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

	s = getenv("maxcpuclk");
	if (s)
		maxcpuclk = simple_strtoul(s, NULL, 10);

	if (maxcpuclk >= 456000000)
		rev = 3;
	else if (maxcpuclk >= 408000000)
		rev = 2;
	else if (maxcpuclk >= 372000000)
		rev = 1;
#ifdef CONFIG_DA850_AM18X_EVM
	rev |= REV_AM18X_EVM;
#endif
	return rev;
}

int board_early_init_f(void)
{
	/*
	 * Power on required peripherals
	 * ARM does not have access by default to PSC0 and PSC1
	 * assuming here that the DSP bootloader has set the IOPU
	 * such that PSC access is available to ARM
	 */
	if (da8xx_configure_lpsc_items(lpsc, ARRAY_SIZE(lpsc)))
		return 1;

	return 0;
}

int board_init(void)
{

#ifndef CONFIG_USE_IRQ
	irq_init();
#endif

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

	/* arch number of the board */
	gd->bd->bi_arch_number = MACH_TYPE_DAVINCI_MANHATTAN_A07;

	/* address of boot parameters */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	/* setup the SUSPSRC for ARM to control emulation suspend */
	writel(readl(&davinci_syscfg_regs->suspsrc) &
	       ~(DAVINCI_SYSCFG_SUSPSRC_EMAC | DAVINCI_SYSCFG_SUSPSRC_I2C |
		 DAVINCI_SYSCFG_SUSPSRC_SPI1 | DAVINCI_SYSCFG_SUSPSRC_TIMER0 |
		 DAVINCI_SYSCFG_SUSPSRC_UART2),
	       &davinci_syscfg_regs->suspsrc);

	/* configure pinmux settings */
	if (davinci_configure_pin_mux_items(pinmuxes, ARRAY_SIZE(pinmuxes)))
		return 1;


#ifdef CONFIG_DRIVER_TI_EMAC
	davinci_emac_mii_mode_sel(0);  // set mode MII
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
