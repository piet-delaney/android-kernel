/* arch/arm/mach-s5pv210/include/mach/idle2.h
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - IDLE2 functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <mach/regs-gpio.h>
#include <mach/cpuidle.h>
#include <mach/power-domain.h>
#include <mach/gpio.h>
#include <mach/gpio-herring.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <mach/pm-core.h>

#define MAX_CHK_DEV	5

/* IDLE2 control flags */
static unsigned int idle2_flags;
#define DISABLED_BY_SUSPEND	(1 << 0)
#define EXTERNAL_ACTIVE		(1 << 1)
#define WORK_INITIALISED	(1 << 2)
#define INACTIVE_PENDING	(1 << 3)
#define EARLYSUSPEND_ACTIVE	(1 << 4)
#define NEEDS_TOPON		(1 << 5)
#define TOPON_CANCEL_PENDING	(1 << 6)
#define TOPON_CONC_REQ		(1 << 7)
#define BLUETOOTH_TIMEOUT	(1 << 8)
#define BLUETOOTH_REQUEST	(1 << 9)
#define UART_TIMEOUT		(1 << 10)
#define UART_REQUEST		(1 << 11)

static struct workqueue_struct *idle2_wq;
struct work_struct idle2_external_active_work;
struct delayed_work idle2_external_inactive_work;
struct work_struct idle2_enable_topon_work;
struct delayed_work idle2_cancel_topon_work;
bool top_enabled __read_mostly;
/* For stat collection */
static u32 bail_vic, bail_mmc, bail_gating, bail_i2s, bail_rtc, bail_nand, bail_C3_gating, bail_DMA;

/*
 * For saving & restoring state
 */
static unsigned long vic_regs[4];
static unsigned long tmp, val;
static unsigned long save_eint_mask;

#define GPIO_OFFSET		0x20
#define GPIO_CON_PDN_OFFSET	0x10
#define GPIO_PUD_PDN_OFFSET	0x14
#define GPIO_PUD_OFFSET		0x08

/* Specific device list for checking before entering
 * idle2 mode
 **/
struct check_device_op {
	void __iomem		*base;
	struct platform_device	*pdev;
};

/* Array of checking devices list */
struct check_device_op chk_dev_op[] = {
#if defined(CONFIG_S3C_DEV_HSMMC)
	{.base = 0, .pdev = &s3c_device_hsmmc0},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC1)
	{.base = 0, .pdev = &s3c_device_hsmmc1},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC2)
	{.base = 0, .pdev = &s3c_device_hsmmc2},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC3)
	{.base = 0, .pdev = &s3c_device_hsmmc3},
#endif
	{.base = 0, .pdev = &s5p_device_onenand},
	{.base = 0, .pdev = NULL},
};

#define S3C_HSMMC_PRNSTS	(0x24)
#define S3C_HSMMC_CLKCON	(0x2c)
#define S3C_HSMMC_CMD_INHIBIT	0x00000001
#define S3C_HSMMC_DATA_INHIBIT	0x00000002
#define S3C_HSMMC_CLOCK_CARD_EN	0x0004

extern void i2sdma_getpos(dma_addr_t *src);
extern unsigned int get_rtc_cnt(void);

inline static bool idle2_pre_entry_check(void)
{
	unsigned int reg1, reg2;
	unsigned char iter;
	dma_addr_t src;
	unsigned int current_cnt;
	void __iomem *base_addr;

	/*
	 * Check for RTC interrupt
	 */
	current_cnt = get_rtc_cnt();
	if (unlikely(current_cnt < 0x40)) {
		bail_rtc++;
		return true;
	}

	/*
	 * Check for HSMMC activity
	 */
	for (iter = 0; iter < 2; iter++) {
		if (unlikely(iter > 1)) {
		pr_err("Invalid ch[%d] for SD/MMC \n", iter);
		return false;
		}
		base_addr = chk_dev_op[iter].base;
		/* Check CMDINHDAT[1] and CMDINHCMD [0] */
		reg1 = readl(base_addr + S3C_HSMMC_PRNSTS);
		/* Check CLKCON [2]: ENSDCLK */
		reg2 = readl(base_addr + S3C_HSMMC_CLKCON);

		if (unlikely((reg1 & (S3C_HSMMC_CMD_INHIBIT | S3C_HSMMC_DATA_INHIBIT))
				|| (reg2 & (S3C_HSMMC_CLOCK_CARD_EN)))) {
			/* Increment bail counter for stats */
			bail_mmc++;
			return true;
		}
	}

	/*
	 * Check for pending VIC interrupt
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2])) {
		bail_vic++;
		return true;
	}

	/*
	 * Check for OneNAND activity
	 */
	base_addr = chk_dev_op[3].base;

	val = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(val & 0x1)) {
		bail_nand++;
		return true;
	}

	/*
	 * Check I2S DMA activity
	 */
	i2sdma_getpos(&src);
	src = src & 0x3FFF;
	src = 0x4000 - src;
	if (src < 0x2000)
		printk("I2S DMA src: %x\n", src);
	if (unlikely(src < 0x150)) {
		bail_i2s++;
		return true;
	}

	/* check clock gating */
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_PDMA1))) {
		bail_DMA++;
		return true;
	}
	val = __raw_readl(S5P_CLKGATE_IP3);
		if (unlikely(val & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
						| S5P_CLKGATE_IP3_I2C2))) {
		bail_gating++;
		return true;
	}
	val = __raw_readl(S5P_CLKGATE_IP1);
	if (val & S5P_CLKGATE_IP1_USBHOST) {
		bail_gating++;
		return true;
	}

	/* If nothing is true, return false so we can continue */
	return false;
}

inline static bool check_C3_clock_gating(void)
{
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & S5P_CLKGATE_IP0_G3D)) {
		bail_C3_gating++;
		return true;
	}
	return false;
}

inline static int s5p_enter_idle2(bool top_enabled)
{
	if (unlikely(pm_cpu_sleep == NULL)) {
		pr_err("%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	/*
	 * Store the resume address in the INFORM0 register
	 */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	/*
	 * Check VIC Status again before entering IDLE2 mode.
	 * Return EBUSY if there is an interrupt pending
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2]))
		return -EBUSY;

	/* GPIO Power Down Control */
	if (likely(!top_enabled)) {
		void __iomem *gpio_base = S5PV210_GPA0_BASE;
		do {
			/* Keep the previous state in idle2 mode */
			__raw_writel(0xffff, gpio_base + GPIO_CON_PDN_OFFSET);

			/* Pull up-down state in idle2 is same as normal */
			tmp = __raw_readl(gpio_base + GPIO_PUD_OFFSET);
			__raw_writel(tmp, gpio_base + GPIO_PUD_PDN_OFFSET);

			gpio_base += GPIO_OFFSET;
		} while (gpio_base <= S5PV210_MP28_BASE);
	}

	/*
	 * Configure IDLE_CFG register
	 * Set ARM_L2_CACHE field to 0b111111
	 *
	 * For TOP block on:
	 * TOP_LOGIC = ON
	 * TOP_MEMORY = ON
	 * ARM_L2_CACHE = Retention
	 * CFG_DIDLE = DEEP
	 *
	 * For TOP block off
	 * TOP_LOGIC = OFF
	 * TOP_MEMORY = OFF
	 * ARM_L2_CACHE = Retention
	 * CFG_DIDLE = DEEP
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(0x3fU << 26);
	if (unlikely(top_enabled))
		tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	else
		tmp |= (S5P_IDLE_CFG_TL_RET | S5P_IDLE_CFG_TM_RET
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	__raw_writel(tmp, S5P_IDLE_CFG);

	/*
	 * Set configuration for idle entry
	 */

	/*
	 * Wakeup Sources: Enable all wakeup sources by unsetting
	 * all bits in S5P_WAKEUP_MASK.
	 * RTC_ALARM, RTC_TICK, TS0 & 1, KEY, MMC0-3, I2S, ST, CEC
	 */
	tmp = __raw_readl(S5P_WAKEUP_MASK);
	tmp &= ~0xFFFFFFFFU;
	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/*
	 * External Interrupts: Save current EINT MASK register
	 * Set all bits to avoid spurious wakeups
	 * Enable EINT 22 & 29 as wakeup sources
	 */
	save_eint_mask = __raw_readl(S5P_EINT_WAKEUP_MASK);
	tmp = save_eint_mask;
	tmp |= 0xFFFFFFFFU;
	tmp &= ~((1 << 22) | (1 << 29));
	__raw_writel(tmp, S5P_EINT_WAKEUP_MASK);

	/*
	 * Set PWR_CFG register to enter IDLE mode
	 * when WFI is called
	 */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_IDLE;
	__raw_writel(tmp, S5P_PWR_CFG);

	/*
	 * Disable interrupts through SYSCON
	 */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	/*
	 * Clear wakeup status register
	 */
	__raw_writel(__raw_readl(S5P_WAKEUP_STAT), S5P_WAKEUP_STAT);

	/*
	 * Enter idle2 mode using the platform suspend code
	 */
	s3c_idle2_cpu_save(0, PLAT_PHYS_OFFSET - PAGE_OFFSET);

	/*
	 * We have resumed from IDLE and returned.
	 * Use platform CPU init code to continue.
	 */
	cpu_init();

	if (likely(!top_enabled)) {
		/*
		 * Release retention of GPIO/MMC/UART IO pads
		 */
		tmp = __raw_readl(S5P_OTHERS);
		tmp |= (S5P_OTHERS_RET_IO | S5P_OTHERS_RET_CF
			| S5P_OTHERS_RET_MMC | S5P_OTHERS_RET_UART);
		__raw_writel(tmp, S5P_OTHERS);
	}

	/*
	 * Restore the saved EINT Wakeup mask
	 */
	__raw_writel(save_eint_mask, S5P_EINT_WAKEUP_MASK);

	/*
	 * Restore S5P_IDLE_CFG & S5P_PWR_CFG registers
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(S5P_IDLE_CFG_TL_MASK | S5P_IDLE_CFG_TM_MASK
		| S5P_IDLE_CFG_L2_MASK | S5P_IDLE_CFG_DIDLE);
	tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON);
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	return 0;
}

int s5p_init_remap(void)
{
	int i = 0;
	struct platform_device *pdev;
	struct resource *res;
	/* Allocate memory region to access IP's directly */
	for (i = 0 ; i < MAX_CHK_DEV ; i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("%s: failed to get io memory region\n", __func__);
			return -EINVAL;
		}
		/* ioremap for register block */
		if(pdev == &s5p_device_onenand)
			chk_dev_op[i].base = ioremap(res->start+0x00600000, 4096);
		else
			chk_dev_op[i].base = ioremap(res->start, 4096);

		if (!chk_dev_op[i].base) {
			pr_err("%s: failed to remap io region\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}
