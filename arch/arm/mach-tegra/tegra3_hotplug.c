/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * Enable debug output to dump the average
 * calculations and ring buffer array values
 * WARNING: Enabling this causes a ton of overhead
 *
 * FIXME: Turn it into debugfs stats (somehow)
 * because currently it is a sack of shit.
 */
#define DEBUG 0

#define CPUS_AVAILABLE 		num_possible_cpus()
#define MIN_ONLINE_CPUS 	1
#define SAMPLING_PERIODS 	10
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)
#define MIN_SAMPLING_RATE 	msecs_to_jiffies(10) /* 10 ms min sampling rate */
#define ENABLE_LOAD_THRESHOLD 	(150 * CPUS_AVAILABLE) /* When load spikes high, enable all CPUs */
#define DISABLE_LOAD_THRESHOLD 	60 /* When CPU load is 0.6 disable additional CPUs */
#define ONE_SECOND 		msecs_to_jiffies(1000) /* 1 second */
#define ONE_HUNDRED_MS		msecs_to_jiffies(100) /* 100 milliseconds */

struct delayed_work tegra3_hotplug_work;
struct delayed_work hotplug_online_work;
struct delayed_work hotplug_online_all_work;
struct delayed_work hotplug_offline_work;
struct delayed_work hotplug_offline_all_work;
struct workqueue_struct *tegra3_hotplug_wq;

static unsigned char min_online_cpus __read_mostly = MIN_ONLINE_CPUS;
static unsigned int history[SAMPLING_PERIODS];
static unsigned char index;
static unsigned int avg_running;
static DEFINE_SPINLOCK(hotplug_lock);

static void tegra3_hotplug(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, enable_load;
	unsigned char online_cpus, i, j;
#if DEBUG
	unsigned char k;
#endif

	spin_lock(&hotplug_lock);

	online_cpus = num_online_cpus();

	disable_load = DISABLE_LOAD_THRESHOLD * online_cpus;
	enable_load = ENABLE_LOAD_THRESHOLD / 2 * online_cpus;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use float division to get the average, as it is a hell
	 * of a lot more expensive.
	 */
	running = nr_running() * 100;

	history[index] = running;

#if DEBUG
	pr_info("online_cpus is: %d\n", online_cpus);
	pr_info("enable_load is: %d\n", enable_load);
	pr_info("disable_load is: %d\n", disable_load);
	pr_info("index is: %d\n", index);
	pr_info("running is: %d\n", running);
#endif

	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < SAMPLING_PERIODS; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

#if DEBUG
	pr_info("array contents: ");
	for (k = 0; k < SAMPLING_PERIODS; k++) {
		 pr_info("%d\t", history[k]);
	}
	pr_info("\n");
	pr_info("avg_running before division: %d\n", avg_running);
#endif

	avg_running = avg_running / SAMPLING_PERIODS;

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;

#if DEBUG
	pr_info("average_running is: %d\n", avg_running);
#endif
	spin_unlock(&hotplug_lock);

	if(unlikely(((avg_running >= ENABLE_LOAD_THRESHOLD) && (online_cpus < CPUS_AVAILABLE))
			|| (online_cpus < min_online_cpus))) {
		pr_info("tegra3_hotplug: Onlining CPUs, avg running: %d\n", avg_running);
		/*
		 * Flush any delayed offlining work from the workqueue.
		 * No point in having expensive unnecessary hotplug transitions.
		 * We still online after flushing, because load is high enough to
		 * warrant it.
		 */
		cancel_delayed_work_sync(&hotplug_offline_work);
		queue_delayed_work(tegra3_hotplug_wq, &hotplug_online_all_work, 0);
		return;
	}
	if(unlikely((avg_running < disable_load) && (online_cpus > min_online_cpus))) {
		pr_info("tegra3_hotplug: Offlining CPU, avg running: %d\n", avg_running);
		queue_delayed_work(tegra3_hotplug_wq, &hotplug_offline_work, ONE_SECOND * 2);
		return;
	}

	/*
	 * If we don't queue a cpu_up or cpu_down then fall through to here
	 * reduce the sampling rate dynamically based on online cpus.
	 */
	sampling_rate = MIN_SAMPLING_RATE * (online_cpus * online_cpus);
#if DEBUG
	pr_info("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
	queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, sampling_rate);
}

static int min_online_state_set(const char *val, const struct kernel_param *kp)
{
	param_set_uint(val, kp);
	pr_info("tegra3_hotplug min_online_cpus: %d\n", min_online_cpus);
	/*
	 * Make sure that a sane value has been passed to
	 * the min_online_cpus parameter.
	 */
	if (unlikely(min_online_cpus > CPUS_AVAILABLE))
		min_online_cpus = CPUS_AVAILABLE;
	else if (unlikely(min_online_cpus < 1))
		min_online_cpus = 1;
	return 0;
}

static int min_online_state_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static struct kernel_param_ops tegra_min_online_ops = {
	.set = min_online_state_set,
	.get = min_online_state_get,
};
module_param_cb(min_online_cpus, &tegra_min_online_ops, &min_online_cpus, 0644);

static void hotplug_online_all(struct work_struct *work)
{
	unsigned char cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu))) {
			cpu_up(cpu);
			pr_info("tegra3_hotplug: CPU%d coming online.\n", cpu);
		}
	}
	/*
	 * Pause for 2 seconds before even considering offlining a CPU
	 */
	queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_SECOND * 2);
}

static void hotplug_offline_all(struct work_struct *work)
{
	unsigned char cpu;
	for_each_possible_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu != 0))) {
			cpu_down(cpu);
			pr_info("tegra3_hotplug: CPU%d going offline.\n", cpu);
		}
	}
}

static void hotplug_offline(struct work_struct *work)
{
	/*
	 * The CPU number that we want to offline will be one less
	 * than num_online_cpus().
	 * If this is not the case, something has been fiddling
	 * so try to rectify the situation by onlining all CPUs
	 * then continue and hope that whatever is fiddling stops.
	 * WARNING: Fiddling will result in lots of messages in dmesg. :)
	 */
	unsigned char cpu;
	unsigned char cpus_online = num_online_cpus();
	if (unlikely(cpus_online == 1)) {
		queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_HUNDRED_MS);
		WARN(1, KERN_WARNING
			"tegra3_hotplug: %s called but no secondary cores online?\n", __func__);
		return;
	}
	cpu = (cpus_online - 1);
	if (likely(cpu_online(cpu))) {
		if (unlikely(cpu == 0)) {
			queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_HUNDRED_MS);
			return;
		}
		cpu_down(cpu);
		pr_info("tegra3_hotplug: CPU%d going offline.\n", cpu);
	} else {
		/*
		 * This shouldn't happen, but if it does, online all cores and allow
		 * the situation to resolve itself.
		 */
		WARN(1, KERN_WARNING
			"tegra3_hotplug: CPU %d already offline! Onlining all CPUs to correct this.\n", cpu);
		queue_delayed_work(tegra3_hotplug_wq, &hotplug_online_all_work, 0);
		return;
	}
	queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_HUNDRED_MS);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tegra3_hotplug_early_suspend(struct early_suspend *handler)
{
	pr_info("tegra3_hotplug: early suspend handler\n");

	cancel_delayed_work_sync(&tegra3_hotplug_work);
	if (num_online_cpus() > 1) {
		pr_info("tegra3_hotplug: Offlining CPUs for early suspend\n");
		queue_delayed_work(tegra3_hotplug_wq, &hotplug_offline_all_work, ONE_SECOND);
	}
}

static void tegra3_hotplug_late_resume(struct early_suspend *handler)
{
	pr_info("tegra3_hotplug: late resume handler\n");

	queue_delayed_work(tegra3_hotplug_wq, &hotplug_online_all_work, 0);
	queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_SECOND);
}

static struct early_suspend tegra3_hotplug_suspend = {
	.suspend = tegra3_hotplug_early_suspend,
	.resume = tegra3_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int __init tegra3_hotplug_init(void)
{
	pr_info("tegra3_hotplug: v0.100 by _thalamus init()\n");
	pr_info("tegra3_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);
	tegra3_hotplug_wq = create_singlethread_workqueue("tegra3_hotplug");
	BUG_ON(!tegra3_hotplug_wq);
	INIT_DELAYED_WORK_DEFERRABLE(&tegra3_hotplug_work, tegra3_hotplug);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_online_all_work, hotplug_online_all);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_all_work, hotplug_offline_all);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_work, hotplug_offline);
	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	queue_delayed_work(tegra3_hotplug_wq, &tegra3_hotplug_work, ONE_SECOND * 30);
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&tegra3_hotplug_suspend);
#endif
	return 0;
}
late_initcall(tegra3_hotplug_init);
