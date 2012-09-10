/*
 * arch/arm/mach-s5pv210/idle2.c
 *
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 * Portions of code (c) Samsung
 *
 * IDLE2 driver for S5PV210 - Provides additional idle states
 * and associated functions.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <linux/cpu_pm.h>
#include <linux/clockchips.h>

#include <mach/regs-irq.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/idle2.h>

#include <plat/pm.h>
#include <plat/devs.h>

#include <asm/cacheflush.h>

/*
 * WARNING: Do not change IDLE2_FREQ because it it also SLEEP_FREQ as we no
 * longer set that explicity in cpufreq.c. SLEEP_FREQ is defined as 800 MHz.
 * Unfdefined behaviour may result, you have been warned!
 */
#define IDLE2_FREQ		(800 * 1000) /* 800MHz Screen off / Suspend */
#define DISABLE_FURTHER_CPUFREQ	0x10
#define ENABLE_FURTHER_CPUFREQ	0x20
#define STATE_C2		2
#define STATE_C3		3
#define MAX_CHK_DEV		5

/* IDLE2 control flags */
static u16 idle2_flags;
#define IDLE2_DISABLED		(1 << 0)
#define EXTERNAL_ACTIVE		(1 << 1)
#define WORK_INITIALISED	(1 << 2)
#define INACTIVE_PENDING	(1 << 3)
#define EARLYSUSPEND_ACTIVE	(1 << 4)
#define NEEDS_TOPON		(1 << 5)
#define BLUETOOTH_REQUEST	(1 << 6)
#define UART_REQUEST		(1 << 7)
#define TOP_BLOCK_ON		(1 << 8)
#define IDLE2_EXT_KILL		(1 << 9)


#ifdef CONFIG_S5P_IDLE2_STATS
/* For statistics */
u32 done_C2, done_C3, fallthrough;
u8 idle_state;
u64 time_in_state[3];
#endif
/* Work function declarations */
struct work_struct idle2_external_active_work;
struct delayed_work idle2_external_inactive_work;
struct delayed_work idle2_bluetooth_irq_timeout_work;
struct delayed_work idle2_lock_cpufreq_work;
struct delayed_work idle2_unlock_cpufreq_work;
struct delayed_work idle2_timeout_kill_work;

static bool idle2_disabled __read_mostly;

/*
 * For saving & restoring state
 */
static unsigned long vic_regs[4];

#define GPIO_OFFSET		0x20
#define GPIO_CON_PDN_OFFSET	0x10
#define GPIO_PUD_PDN_OFFSET	0x14
#define GPIO_PUD_OFFSET		0x08

/*
 * Specific device list for checking before entering
 * idle2 mode
 */
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

inline static bool idle2_pre_entry_check(void)
{
	unsigned int reg1, reg2;
	unsigned long tmp;
	unsigned char iter;
	void __iomem *base_addr;

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
			return true;
		}
	}

	/*
	 * Check for pending VIC interrupt
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2])) {
		return true;
	}

	/*
	 * Check for OneNAND activity
	 */
	base_addr = chk_dev_op[3].base;

	tmp = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(tmp & 0x1)) {
		return true;
	}

	/* check clock gating */
	tmp = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(tmp & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_PDMA1))) {
		return true;
	}
	tmp = __raw_readl(S5P_CLKGATE_IP3);
		if (unlikely(tmp & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
						| S5P_CLKGATE_IP3_I2C2))) {
		return true;
	}
	tmp = __raw_readl(S5P_CLKGATE_IP1);
	if (unlikely(tmp & S5P_CLKGATE_IP1_USBHOST)) {
		return true;
	}

	/* If nothing is true, return false so we can continue */
	return false;
}

inline static bool check_C3_clock_gating(void)
{
	unsigned long tmp;
	tmp = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(tmp & S5P_CLKGATE_IP0_G3D)) {
		return true;
	}
	return false;
}

/*
 * Function which actually enters into DEEP-IDLE states
 */
extern void s5p_enter_idle(void);
inline static int s5p_enter_idle2(void)
{
	unsigned long tmp;

	if (unlikely(pm_cpu_sleep == NULL)) {
		pr_err("%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	/*
	 * Check VIC Status again before entering IDLE2 mode.
	 * Return EBUSY if there is an interrupt pending
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2]))
		return -EBUSY;

	/*
	 * Store the resume address in the INFORM0 register
	 */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	/*
	 * Notify timer infrastructure that we are entering a
	 * low power state.
	 */
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, 0);

	/*
	 * Notify CPU PM notifier that we are entering a low
	 * power state.
	 * This currently saves the VFP state.
	 */
	cpu_pm_enter();

	/* GPIO Power Down Control */
	if (likely(!(idle2_flags & TOP_BLOCK_ON))) {
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
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	/*
	 * Clear bits 26 to 31
	 * This unsets the TL, TM & L2 bits
	 */
	tmp &= ~(0x3f << 26);
	if (likely(!(idle2_flags & TOP_BLOCK_ON)))
		/*
		 * TOP block OFF configuration
		 * TOP logic retention, TOP memory retention
		 * L2 cache OFF (bit 26 is already unset)
		 */
		tmp |= (S5P_IDLE_CFG_TL_RET | S5P_IDLE_CFG_TM_RET
			| S5P_IDLE_CFG_DIDLE);
	else
		/*
		 * TOP block ON configuration
		 * TOP logic ON, TOP memory retention
		 * L2 cache OFF (bit 26 is already unset)
		 */
		tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_RET
			| S5P_IDLE_CFG_DIDLE);
	__raw_writel(tmp, S5P_IDLE_CFG);

	/*
	 * Set configuration for idle2 entry
	 */

	/*
	 * Wakeup Source configuration:
	 * Reset all unreserved bits to enable
	 * all sources.
	 */
	tmp = __raw_readl(S5P_WAKEUP_MASK);
	tmp &= ~0xffff;
	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/*
	 * External Interrupt wakeup sources:
	 * Enable all sources, then mask sources
	 * which cause spurious wakeups.
	 *
	 * EINT 1,2 & 30 are permanently pending
	 * causing continual wakeups.
	 */
	tmp &= ~0xffffffff;
	tmp |= ((1 << 1) | (1 << 2) | (1 << 30));
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
	 * Clear wakeup status register by writing the current
	 * value back to it.
	 */
	__raw_writel(__raw_readl(S5P_WAKEUP_STAT), S5P_WAKEUP_STAT);

	/*
	 * Entering idle2 state using platform PM code.
	 */

	/*
	 * Save CPU state and enter idle2 state
	 */
	s3c_cpu_save(0, PLAT_PHYS_OFFSET - PAGE_OFFSET);

	/*
	 * We have resumed from IDLE and returned.
	 * Use platform CPU init code to continue and
	 * flush caches to be sure.
	 */
	cpu_init();
	flush_cache_all();

	if (likely(!(idle2_flags & TOP_BLOCK_ON))) {
		/*
		 * Release retention of GPIO/MMC/UART IO pads
		 */
		tmp = __raw_readl(S5P_OTHERS);
		tmp |= (S5P_OTHERS_RET_IO | S5P_OTHERS_RET_CF
			| S5P_OTHERS_RET_MMC | S5P_OTHERS_RET_UART);
		__raw_writel(tmp, S5P_OTHERS);
	}

	/*
	 * Reset IDLE_CFG & PWR_CFG register
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(S5P_IDLE_CFG_TL_MASK | S5P_IDLE_CFG_TM_MASK
		| S5P_IDLE_CFG_L2_MASK | S5P_IDLE_CFG_DIDLE);
	tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON);
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	/*
	 * Reset EINT wakeup mask for NORMAL mode
	 */
	tmp &= ~0xffffffff;
	__raw_writel(tmp, S5P_EINT_WAKEUP_MASK);

	/*
	 * Notify that we have left the low power state
	 */
	cpu_pm_exit();
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, 0);

	return 0;
}

/*
 * Function which controls invocation of DEEP-IDLE states
 * and invokes all the necessary checks and does all the
 * necessary preparations to enter.
 */
inline int s5p_enter_idle_deep(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	s8 err;
	u8 *state_name = (state->name);
	u32 idle_time;

	/*
	 * Convert second ascii character of state_name,
	 * (which is the state number) to char.
	 * Looks cludgy, but it works and is cheap.
	 */
	unsigned char requested_state = (state_name[1] - 48);

	do_gettimeofday(&before);

	local_irq_disable();
	local_fiq_disable();

	/*
	 * Save current VIC registers
	 */
	vic_regs[0] = __raw_readl(VA_VIC0 + VIC_INT_ENABLE);
	vic_regs[1] = __raw_readl(VA_VIC1 + VIC_INT_ENABLE);
	vic_regs[2] = __raw_readl(VA_VIC2 + VIC_INT_ENABLE);
	vic_regs[3] = __raw_readl(VA_VIC3 + VIC_INT_ENABLE);

	/*
	 * Clear VIC registers to disable interrupts
	 */
	__raw_writel(0xffffffff, (VA_VIC0 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC1 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC2 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC3 + VIC_INT_ENABLE_CLEAR));

	switch (requested_state){
	case STATE_C2:
		if (unlikely(idle2_pre_entry_check())) {
			device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
			fallthrough++;
			idle_state = 0;
#endif
			break;
		} else {
			idle2_flags |= TOP_BLOCK_ON;
			err = s5p_enter_idle2();
			if (unlikely(err)) {
				device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
				fallthrough++;
				idle_state = 0;
#endif
				break;
			} else {
#ifdef CONFIG_S5P_IDLE2_STATS
				done_C2++;
				idle2_update_wakeup_stats();
				idle_state = 1;
#endif
				break;
			}
		}
	case STATE_C3:
		if (unlikely(idle2_pre_entry_check())) {
			device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
			fallthrough++;
			idle_state = 0;
#endif
			break;
		} else if (unlikely(check_C3_clock_gating())) {
			/*
			 * Tell cpuidle governor that we went into
			 * C2 instead of C3
			 */
			device->last_state = &device->states[1];
			idle2_flags |= TOP_BLOCK_ON;
			err = s5p_enter_idle2();
			if (unlikely(err)) {
				device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
				fallthrough++;
				idle_state = 0;
#endif
				break;
			} else {
#ifdef CONFIG_S5P_IDLE2_STATS
				done_C2++;
				idle2_update_wakeup_stats();
				idle_state = 1;
#endif
				break;
			}
		} else {
			idle2_flags &= ~TOP_BLOCK_ON;
			err = s5p_enter_idle2();
			if (unlikely(err)) {
				device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
				fallthrough++;
				idle_state = 0;
#endif
				break;
			} else {
#ifdef CONFIG_S5P_IDLE2_STATS
				done_C3++;
				idle2_update_wakeup_stats();
				idle_state = 2;
#endif
				break;
			}
		}
	default:
		WARN_ON(0);
		device->last_state = &device->states[0];
#ifdef CONFIG_S5P_IDLE2_STATS
		fallthrough++;
		idle_state = 0;
#endif
		break;
	}

	/*
	 * Restore saved VIC registers and enable interrupts
	 */
	__raw_writel(vic_regs[0], VA_VIC0 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[1], VA_VIC1 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[2], VA_VIC2 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[3], VA_VIC3 + VIC_INT_ENABLE);

	do_gettimeofday(&after);

	local_fiq_enable();
	local_irq_enable();

	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
#ifdef CONFIG_S5P_IDLE2_STATS
	if (likely(idle_state))
		time_in_state[idle_state] += (u64)idle_time;
	idle_state = 0;
#endif
	return idle_time;
}

/*
 * Workqueue related functions which control IDLE2 invocation.
 */

void idle2_kill(bool kill, u16 timeout)
{
	if (!(idle2_flags & WORK_INITIALISED))
		return;
	if (kill && timeout) {
		if (delayed_work_pending(&idle2_timeout_kill_work))
			cancel_delayed_work(&idle2_timeout_kill_work);

		idle2_flags |= IDLE2_EXT_KILL;
		pr_info("idle2: |= IDLE2_EXT_KILL - Timeout: %u\n", timeout);
		schedule_delayed_work(&idle2_timeout_kill_work, timeout);
	} else if (kill) {
		if (delayed_work_pending(&idle2_timeout_kill_work))
			cancel_delayed_work(&idle2_timeout_kill_work);
		if (!(idle2_flags & IDLE2_EXT_KILL)) {
			idle2_flags |= IDLE2_EXT_KILL;
			pr_info("idle2: |= IDLE2_EXT_KILL\n");
		}
	} else
		schedule_delayed_work(&idle2_timeout_kill_work, timeout);
}

static void idle2_timeout_kill_work_fn(struct work_struct *work)
{
	idle2_flags &= ~IDLE2_EXT_KILL;
	pr_info("idle2: &= ~IDLE2_EXT_KILL\n");
}

static void idle2_bluetooth_irq_timeout_work_fn(struct work_struct *work)
{
	idle2_flags &= ~BLUETOOTH_REQUEST;
	pr_info("idle2: &= ~BLUETOOTH_REQUEST\n");
}

static void idle2_lock_cpufreq_work_fn(struct work_struct *work)
{
	cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
			DISABLE_FURTHER_CPUFREQ);
	pr_info("%s: CPUfreq locked to %dKHz\n", __func__, IDLE2_FREQ);
}

static void idle2_unlock_cpufreq_work_fn(struct work_struct *work)
{
	cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
			ENABLE_FURTHER_CPUFREQ);
	pr_info("%s: CPUfreq unlocked from %dKHz\n", __func__, IDLE2_FREQ);
}

void earlysuspend_active_fn(bool flag)
{
	if (!(idle2_flags & WORK_INITIALISED))
		return;
	if (flag) {
		idle2_flags |= EARLYSUSPEND_ACTIVE;
		schedule_delayed_work(&idle2_lock_cpufreq_work, 0);
		pr_info("idle2: |= EARLYSUSPEND_ACTIVE\n");
	} else {
		idle2_flags &= ~EARLYSUSPEND_ACTIVE;
		schedule_delayed_work(&idle2_unlock_cpufreq_work, 0);
		pr_info("idle2: &= ~EARLYSUSPEND_ACTIVE\n");
	}
}

static void idle2_external_active_work_fn(struct work_struct *work)
{
	cancel_delayed_work(&idle2_external_inactive_work);
	idle2_flags |= EXTERNAL_ACTIVE;
	pr_info("idle2: |= EXTERNAL_ACTIVE\n");
}

static void idle2_external_inactive_work_fn(struct work_struct *work)
{
	idle2_flags &= ~EXTERNAL_ACTIVE;
	pr_info("idle2: &= ~EXTERNAL_ACTIVE\n");
}

/*
 * external_active / inactive functions are called for
 * USB vbus activity.
 */
void idle2_external_active(void)
{
	if ((idle2_flags & WORK_INITIALISED)
		&& (!(idle2_flags & EXTERNAL_ACTIVE)
		|| (idle2_flags & INACTIVE_PENDING))) {
		schedule_work(&idle2_external_active_work);
		idle2_flags &= ~INACTIVE_PENDING;
	}
}

void idle2_external_inactive(unsigned long delay)
{
	if ((idle2_flags & WORK_INITIALISED)
		&& (idle2_flags & EXTERNAL_ACTIVE)
		&& (!(idle2_flags & INACTIVE_PENDING))) {
		idle2_flags |= INACTIVE_PENDING;
		schedule_delayed_work(&idle2_external_inactive_work, delay);
	}
}

/*
 * Bluetooth functions are called for bluetooth activity.
 * They also ignore the large number of active calls you can
 * get whilst bluetooth is active.
 */

void idle2_bluetooth_irq_active(bool kill, u16 timeout)
{
	if ((idle2_flags & WORK_INITIALISED) && kill && timeout) {
		if (delayed_work_pending(&idle2_bluetooth_irq_timeout_work))
			cancel_delayed_work(&idle2_bluetooth_irq_timeout_work);

		idle2_flags |= BLUETOOTH_REQUEST;
		pr_info("idle2: |= BLUETOOTH_REQUEST - Timeout: %u\n", timeout);
		schedule_delayed_work(&idle2_bluetooth_irq_timeout_work, timeout);
	}
}

/*
 * uart functions are called when the GPS is active
 */
void idle2_uart_active(bool flag)
{
	if (flag) {
		idle2_flags |= UART_REQUEST;
		pr_info("idle2: |= UART_REQUEST\n");
	} else {
		idle2_flags &= ~UART_REQUEST;
		pr_info("idle2: &= ~UART_REQUEST\n");
	}
}

/*
 * Functions which are called from cpuidle.c.
 */

inline int s5p_idle_prepare(struct cpuidle_device *device)
{
	if (unlikely((idle2_flags & IDLE2_DISABLED)
		|| (!(idle2_flags & EARLYSUSPEND_ACTIVE))
		|| (idle2_flags & IDLE2_EXT_KILL))) {
		/*
		 * Ignore DEEP-IDLE states
		 */
		device->states[1].flags |= CPUIDLE_FLAG_IGNORE;
		device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
	} else if (unlikely((idle2_flags & EXTERNAL_ACTIVE)
		|| (idle2_flags & UART_REQUEST)
		|| (idle2_flags & BLUETOOTH_REQUEST))) {
		/*
		 * Ignore DEEP-IDLE TOP block OFF state
		 */
		device->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
		device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
	} else {
		/*
		 * Enable both DEEP-IDLE states and allow
		 * cpuidle governor to choose the best one
		 */
		device->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
		device->states[2].flags &= ~CPUIDLE_FLAG_IGNORE;
	}
	return 0;
}

void s5p_init_idle2_work(void)
{
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_bluetooth_irq_timeout_work, idle2_bluetooth_irq_timeout_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_external_inactive_work, idle2_external_inactive_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_lock_cpufreq_work, idle2_lock_cpufreq_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_unlock_cpufreq_work, idle2_unlock_cpufreq_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_timeout_kill_work, idle2_timeout_kill_work_fn);
	INIT_WORK(&idle2_external_active_work, idle2_external_active_work_fn);
	idle2_flags |= WORK_INITIALISED;
}

int s5p_idle2_post_init(void)
{
	int i = 0;
	struct platform_device *pdev;
	struct resource *res;

	printk(KERN_INFO "cpuidle: IDLE2 support enabled - version 0.%d by <willtisdale@gmail.com>\n", IDLE2_VERSION);

	/*
	 * Allocate memory region to access HSMMC & OneNAND registers
	 */
	for (i = 0 ; i < MAX_CHK_DEV ; i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("%s: failed to get io memory region\n", __func__);
			return -EINVAL;
		}
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

static int idle2_disabled_set(const char *arg, const struct kernel_param *kp)
{

	param_set_bool(arg, kp);

	if (idle2_disabled) {
		idle2_flags |= IDLE2_DISABLED;
		pr_info("%s: |= IDLE2_DISABLED\n", __func__);
	} else {
		idle2_flags &= ~IDLE2_DISABLED;
		pr_info("%s: &= ~IDLE2_DISABLED\n", __func__);
	}
	return 0;
}

static struct kernel_param_ops idle2_disabled_ops = {
	.set = idle2_disabled_set,
	.get = param_get_bool,
};
module_param_cb(disable, &idle2_disabled_ops, &idle2_disabled, 0644);
MODULE_PARM_DESC(disable, "Disable C2 & C3 states");
