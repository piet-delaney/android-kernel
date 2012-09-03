/*
 * arch/arm/mach-s5pv210/cpuidle.c
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 *
 * CPU idle driver for S5PV210
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpuidle.h>
#include <asm/proc-fns.h>
#include <mach/regs-clock.h>


#ifdef CONFIG_S5P_IDLE2
extern void s5p_init_idle2_work(void);
extern int s5p_enter_idle_deep(struct cpuidle_device *device,
				struct cpuidle_state *state);
extern int s5p_idle_prepare(struct cpuidle_device *device);
extern void s5p_idle2_post_init(void);
#endif

inline void s5p_enter_idle(void)
{
	unsigned long tmp;

	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~((3<<30)|(3<<28)|(1<<0));
	tmp |= ((2<<30)|(2<<28));
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	cpu_do_idle();
}

/* Actual code that puts the SoC in different idle states */
inline static int s5p_enter_idle_normal(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	s5p_enter_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

static DEFINE_PER_CPU(struct cpuidle_device, s5p_cpuidle_device);

static struct cpuidle_driver s5p_idle_driver = {
	.name =         "s5p_idle",
	.owner =        THIS_MODULE,
};

static void idle_enable_hlt(struct work_struct *work)
{
        enable_hlt();
}
static DECLARE_DELAYED_WORK(idle_enable_hlt_work, idle_enable_hlt);

/* Initialize CPU idle by registering the idle states */
static int s5p_init_cpuidle(void)
{
	struct cpuidle_device *device;

	disable_hlt();
        schedule_delayed_work(&idle_enable_hlt_work, 30 * HZ);
#ifdef CONFIG_S5P_IDLE2
	s5p_init_idle2_work();
#endif
	cpuidle_register_driver(&s5p_idle_driver);

	device = &per_cpu(s5p_cpuidle_device, smp_processor_id());
	device->state_count = 0;

	/* Wait for interrupt state */
	device->states[0].enter = s5p_enter_idle_normal;
	device->states[0].exit_latency = 20;	/* uS */
	device->states[0].target_residency = 20;
	device->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[0].name, "C1");
	strcpy(device->states[0].desc, "ARM clock gating - WFI");
	device->state_count++;
#ifdef CONFIG_S5P_IDLE2
		/* Deep-Idle top ON Wait for interrupt state */
	device->states[1].enter = s5p_enter_idle_deep;
	device->states[1].exit_latency = 600;
	device->states[1].target_residency = 6000;
	device->states[1].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[1].name, "C2");
	strcpy(device->states[1].desc, "ARM Power gating - WFI");
	device->state_count++;

	/* Deep-Idle top OFF Wait for interrupt state */
	device->states[2].enter = s5p_enter_idle_deep;
	device->states[2].exit_latency = 1000;
	device->states[2].target_residency = 6000;
	device->states[2].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[2].name, "C3");
	strcpy(device->states[2].desc, "ARM/TOP/SUB Power gating - WFI");
	device->state_count++;
	/*
	 * Device prepare isn't required when we are building with
	 * CONFIG_S5P_IDLE2 disabled as there is only one active state
	 * so there is nothing to prepare.
	 */
	device->prepare = s5p_idle_prepare;
#endif

	if (cpuidle_register_device(device)) {
		pr_err("s5p_init_cpuidle: Failed registering\n");
		BUG();
		return -EIO;
	}
#ifdef CONFIG_S5P_IDLE2
	s5p_idle2_post_init();
#endif /* CONFIG_S5P_IDLE2 */
	return 0;
}

device_initcall(s5p_init_cpuidle);
