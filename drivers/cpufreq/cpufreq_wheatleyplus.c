/*
 *  drivers/cpufreq/cpufreq_wheatleyplus.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2012 Ezekeel <notezekeel@googlemail.com>
 *            (C)  2014 JOVY23@XDA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/cpuidle.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_UP_THRESHOLD		(76)
#define DEF_SAMPLING_DOWN_FACTOR		(3)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define DEF_SAMPLING_DOWN_MAX_MOMENTUM		(10)
#define DEF_SAMPLING_DOWN_MOMENTUM_SENSITIVITY	(100)
#define MAX_SAMPLING_DOWN_MOMENTUM_SENSITIVITY	(1000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(80)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define DEF_TARGET_RESIDENCY			(10000)
#define DEF_ALLOWED_MISSES			(2)
#define DEF_SMOOTH_UI				(1)
#define DEFAULT_WHFREQ_BOOST_TIME		(500000)
#define DEFAULT_BOOST_FREQ			(652800)
#define MAX_WHFREQ_BOOST_TIME			(5000000)

/* Phase configurables */
#define MAX_IDLE_COUNTER			160
#define PHASE_2_PERCENT				65
#define PHASE_3_PERCENT				85
#define PHASE_2_FREQ				883200
#define PHASE_3_FREQ				1036800
#define SEMI_BUSY_THRESHOLD			16
#define SEMI_BUSY_CLR_THRESHOLD			8
#define BUSY_THRESHOLD				120
#define BUSY_CLR_THRESHOLD			80
#define DECREASE_IDLE_COUNTER			12

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
bool gpu_busy_state_wp;
#define GPU_MAX_IDLE_COUNTER			800
#define GPU_COUNTER_INCREASE			20
#define GPU_SEMI_BUSY_THRESHOLD			200
#define GPU_SEMI_BUSY_CLR_THRESHOLD		120
#define GPU_BUSY_THRESHOLD			500
#define GPU_BUSY_CLR_THRESHOLD			350
#define DECREASE_GPU_IDLE_COUNTER		4
#endif

u64 whfreq_boosted_time;

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;
static unsigned int num_misses;
static unsigned int orig_sampling_rate;
static unsigned int orig_sampling_down_factor;
static unsigned int orig_sampling_down_max_mom;

static unsigned int touch_state_val;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)



static void do_dbs_timer(struct work_struct *work);
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_WHEATLEYPLUS
static
#endif
struct cpufreq_governor cpufreq_gov_wheatleyplus = {
    .name                   = "wheatleyplus",
    .governor               = cpufreq_governor_dbs,
    .max_transition_latency = TRANSITION_LATENCY_LIMIT,
    .owner                  = THIS_MODULE,
};

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
    cputime64_t prev_cpu_idle;
    cputime64_t prev_cpu_iowait;
    cputime64_t prev_cpu_wall;
    cputime64_t prev_cpu_nice;
    struct cpufreq_policy *cur_policy;
    struct delayed_work work;
    struct cpufreq_frequency_table *freq_table;
    unsigned int freq_lo;
    unsigned int freq_lo_jiffies;
    unsigned int freq_hi_jiffies;
    unsigned int rate_mult;
    unsigned int momentum_adder;
    int cpu;
    unsigned int sample_type:1;
    unsigned long long prev_idletime;
    unsigned long long prev_idleusage;

    /*
     * percpu mutex that serializes governor limit change with
     * do_dbs_timer invocation. We do not want do_dbs_timer to run
     * when user is changing the governor or limits.
     */
    struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

DECLARE_PER_CPU(struct cpuidle_device *, cpuidle_devices);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct	*kwheatley_wq;

static struct dbs_tuners {
    unsigned int sampling_rate;
    unsigned int up_threshold;
    unsigned int down_diff;
    unsigned int ignore_nice;
    unsigned int sampling_down_factor;
    unsigned int sampling_down_momentum;
    unsigned int sampling_down_max_mom;
    unsigned int sampling_down_mom_sens;
    unsigned int powersave_bias;
    unsigned int smooth_ui;
    unsigned int target_residency;
    unsigned int allowed_misses;
    unsigned int boosted;
    unsigned int whfreq_boost_time;
    unsigned int boostfreq;
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
    unsigned int two_phase_freq;
    unsigned int semi_busy_threshold;
    unsigned int semi_busy_clr_threshold;
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
    unsigned int three_phase_freq;
    unsigned int busy_threshold;
    unsigned int busy_clr_threshold;
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
    unsigned int gpu_semi_busy_threshold;
    unsigned int gpu_semi_busy_clr_threshold;
    unsigned int gpu_busy_threshold;
    unsigned int gpu_busy_clr_threshold;
#endif

} dbs_tuners_ins = {
    .up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
    .sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
    .sampling_down_momentum = 0,
    .sampling_down_max_mom = DEF_SAMPLING_DOWN_MAX_MOMENTUM,
    .sampling_down_mom_sens =
		DEF_SAMPLING_DOWN_MOMENTUM_SENSITIVITY,
    .ignore_nice = 0,
    .powersave_bias = 0,
    .smooth_ui = DEF_SMOOTH_UI,
    .target_residency = DEF_TARGET_RESIDENCY,
    .allowed_misses = DEF_ALLOWED_MISSES,
    .whfreq_boost_time = DEFAULT_WHFREQ_BOOST_TIME,
    .boostfreq = DEFAULT_BOOST_FREQ,
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
    .two_phase_freq = PHASE_2_FREQ,
    .semi_busy_threshold = SEMI_BUSY_THRESHOLD,
    .semi_busy_clr_threshold = SEMI_BUSY_CLR_THRESHOLD,
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
    .three_phase_freq = PHASE_3_FREQ,
    .busy_threshold = BUSY_THRESHOLD,
    .busy_clr_threshold = BUSY_CLR_THRESHOLD,
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
    .gpu_semi_busy_threshold = GPU_SEMI_BUSY_THRESHOLD,
    .gpu_semi_busy_clr_threshold = GPU_SEMI_BUSY_CLR_THRESHOLD,
    .gpu_busy_threshold = GPU_BUSY_THRESHOLD,
    .gpu_busy_clr_threshold = GPU_BUSY_CLR_THRESHOLD,
#endif
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
						  cputime64_t *wall)
{
    cputime64_t idle_time;
    cputime64_t cur_wall_time;
    cputime64_t busy_time;

    cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

        busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
        busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
        busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
        busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
        busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
        busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

    return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
    u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

    if (idle_time == -1ULL)
	return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

    return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
    u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

    if (iowait_time == -1ULL)
	return 0;

    return iowait_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
    unsigned int freq_req, freq_reduc, freq_avg;
    unsigned int freq_hi, freq_lo;
    unsigned int index = 0;
    unsigned int jiffies_total, jiffies_hi, jiffies_lo;
    struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
					       policy->cpu);

    if (!dbs_info->freq_table) {
	dbs_info->freq_lo = 0;
	dbs_info->freq_lo_jiffies = 0;
	return freq_next;
    }

    cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
				   relation, &index);
    freq_req = dbs_info->freq_table[index].frequency;
    freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
    freq_avg = freq_req - freq_reduc;

    /* Find freq bounds for freq_avg in freq_table */
    index = 0;
    cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
				   CPUFREQ_RELATION_H, &index);
    freq_lo = dbs_info->freq_table[index].frequency;
    index = 0;
    cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
				   CPUFREQ_RELATION_L, &index);
    freq_hi = dbs_info->freq_table[index].frequency;

    /* Find out how long we have to be in hi and lo freqs */
    if (freq_hi == freq_lo) {
	dbs_info->freq_lo = 0;
	dbs_info->freq_lo_jiffies = 0;
	return freq_lo;
    }
    jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
    jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
    jiffies_hi += ((freq_hi - freq_lo) / 2);
    jiffies_hi /= (freq_hi - freq_lo);
    jiffies_lo = jiffies_total - jiffies_hi;
    dbs_info->freq_lo = freq_lo;
    dbs_info->freq_lo_jiffies = jiffies_lo;
    dbs_info->freq_hi_jiffies = jiffies_hi;
    return freq_hi;
}

static void wheatleyplus_powersave_bias_init_cpu(int cpu)
{
    struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
    dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
    dbs_info->freq_lo = 0;
}

static ssize_t show_sampling_rate_max(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
    printk_once(KERN_INFO "CPUFREQ: wheatleyplus sampling_rate_max "
		"sysfs file is deprecated - used by: %s\n", current->comm);
    return sprintf(buf, "%u\n", -1U);
}

static void wheatleyplus_powersave_bias_init(void)
{
    int i;
    for_each_online_cpu(i) {
	wheatleyplus_powersave_bias_init_cpu(i);
    }
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_max);
define_one_global_ro(sampling_rate_min);

/* cpufreq_wheatleyplus Governor Tunables */
#define show_one(file_name, object)				\
    static ssize_t show_##file_name				\
    (struct kobject *kobj, struct attribute *attr, char *buf)	\
    {								\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);	\
    }
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(sampling_down_factor, sampling_down_factor);
show_one(sampling_down_max_momentum, sampling_down_max_mom);
show_one(sampling_down_momentum_sensitivity, sampling_down_mom_sens);
show_one(ignore_nice_load, ignore_nice);
show_one(powersave_bias, powersave_bias);
show_one(smooth_ui, smooth_ui);
show_one(target_residency, target_residency);
show_one(allowed_misses, allowed_misses);
show_one(boostpulse, boosted);
show_one(boostfreq, boostfreq);

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
show_one(two_phase_freq, two_phase_freq);
show_one(semi_busy_threshold, semi_busy_threshold);
show_one(semi_busy_clr_threshold, semi_busy_clr_threshold);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
show_one(three_phase_freq, three_phase_freq);
show_one(busy_threshold, busy_threshold);
show_one(busy_clr_threshold, busy_clr_threshold);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
show_one(gpu_semi_busy_threshold, gpu_semi_busy_threshold);
show_one(gpu_semi_busy_clr_threshold, gpu_semi_busy_clr_threshold);
show_one(gpu_busy_threshold, gpu_busy_threshold);
show_one(gpu_busy_clr_threshold, gpu_busy_clr_threshold);
#endif

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	orig_sampling_rate = dbs_tuners_ins.sampling_rate;
	return count;
}

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
static ssize_t store_two_phase_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.two_phase_freq = input;

	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
static ssize_t store_three_phase_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.three_phase_freq = input;

	return count;
}
#endif


static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	dbs_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;
	orig_sampling_down_factor = dbs_tuners_ins.sampling_down_factor;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_sampling_down_max_momentum(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR -
	    dbs_tuners_ins.sampling_down_factor || input < 0)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_max_mom = input;
	orig_sampling_down_max_mom =
			dbs_tuners_ins.sampling_down_max_mom;

	/* Reset momentum_adder*/
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->momentum_adder = 0;
	}

	return count;
}

static ssize_t store_sampling_down_momentum_sensitivity(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_MOMENTUM_SENSITIVITY
	    || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_mom_sens = input;

	/* Reset momentum_adder*/
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->momentum_adder = 0;
	}

	 return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
    unsigned int input;
    int ret;

    unsigned int j;

    ret = sscanf(buf, "%u", &input);
    if (ret != 1)
	return -EINVAL;

    if (input > 1)
	input = 1;

    if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
	return count;
    }
    dbs_tuners_ins.ignore_nice = input;

    /* we need to re-evaluate prev_cpu_idle */
    for_each_online_cpu(j) {
	struct cpu_dbs_info_s *dbs_info;
	dbs_info = &per_cpu(od_cpu_dbs_info, j);
	dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						    &dbs_info->prev_cpu_wall);
	if (dbs_tuners_ins.ignore_nice)
	    dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];

    }
    return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1)
	return -EINVAL;

    if (input > 1000)
	input = 1000;

    dbs_tuners_ins.powersave_bias = input;
    wheatleyplus_powersave_bias_init();
    return count;
}

static ssize_t store_smooth_ui(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.smooth_ui = !!input;
	return count;
}

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned int input;

	ret = sscanf(buf, "%u", &input);
	if (ret < 0)
		return ret;

	if (input > 1 && input <= MAX_WHFREQ_BOOST_TIME)
		dbs_tuners_ins.whfreq_boost_time = input;
	else
		dbs_tuners_ins.whfreq_boost_time = DEFAULT_WHFREQ_BOOST_TIME;

	dbs_tuners_ins.boosted = 1;
	whfreq_boosted_time = ktime_to_us(ktime_get());
	return count;
}

static ssize_t store_boostfreq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.boostfreq = input;
	return count;
}


static ssize_t store_target_residency(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1)
	return -EINVAL;

    dbs_tuners_ins.target_residency = input;
    return count;
}

static ssize_t store_allowed_misses(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
    unsigned int input;
    int ret;
    ret = sscanf(buf, "%u", &input);

    if (ret != 1)
	return -EINVAL;

    dbs_tuners_ins.allowed_misses = input;
    return count;
}

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
static ssize_t store_semi_busy_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.busy_threshold ||
			input <= 0 || input > dbs_tuners_ins.busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.semi_busy_threshold = input;
	return count;
}
static ssize_t store_semi_busy_clr_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.busy_clr_threshold ||
			input < 0 || input > dbs_tuners_ins.semi_busy_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.semi_busy_clr_threshold = input;
	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
static ssize_t store_busy_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_IDLE_COUNTER ||
			input <= 0 || input < dbs_tuners_ins.semi_busy_threshold ||
			input < dbs_tuners_ins.busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.busy_threshold = input;
	return count;
}
static ssize_t store_busy_clr_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.busy_threshold ||
			input <= 0 || input < dbs_tuners_ins.semi_busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.busy_clr_threshold = input;
	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
static ssize_t store_gpu_semi_busy_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.gpu_busy_threshold ||
			input <= 0 || input > dbs_tuners_ins.gpu_busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.gpu_semi_busy_threshold = input;
	return count;
}
static ssize_t store_gpu_semi_busy_clr_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.gpu_busy_clr_threshold ||
			input < 0 || input > dbs_tuners_ins.gpu_semi_busy_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.gpu_semi_busy_clr_threshold = input;
	return count;
}

static ssize_t store_gpu_busy_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > GPU_MAX_IDLE_COUNTER ||
			input <= 0 || input < dbs_tuners_ins.gpu_semi_busy_threshold ||
			input < dbs_tuners_ins.gpu_busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.gpu_busy_threshold = input;
	return count;
}
static ssize_t store_gpu_busy_clr_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > dbs_tuners_ins.gpu_busy_threshold ||
			input <= 0 || input < dbs_tuners_ins.gpu_semi_busy_clr_threshold) {
		return -EINVAL;
	}
	dbs_tuners_ins.gpu_busy_clr_threshold = input;
	return count;
}
#endif

define_one_global_rw(sampling_rate);
define_one_global_rw(up_threshold);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(sampling_down_max_momentum);
define_one_global_rw(sampling_down_momentum_sensitivity);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
define_one_global_rw(smooth_ui);
define_one_global_rw(target_residency);
define_one_global_rw(allowed_misses);
define_one_global_rw(boostpulse);
define_one_global_rw(boostfreq);
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
define_one_global_rw(two_phase_freq);
define_one_global_rw(semi_busy_threshold);
define_one_global_rw(semi_busy_clr_threshold);
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
define_one_global_rw(three_phase_freq);
define_one_global_rw(busy_threshold);
define_one_global_rw(busy_clr_threshold);
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
define_one_global_rw(gpu_semi_busy_threshold);
define_one_global_rw(gpu_semi_busy_clr_threshold);
define_one_global_rw(gpu_busy_threshold);
define_one_global_rw(gpu_busy_clr_threshold);
#endif

static struct attribute *dbs_attributes[] = {
    &sampling_rate_max.attr,
    &sampling_rate_min.attr,
    &sampling_rate.attr,
    &up_threshold.attr,
    &sampling_down_factor.attr,
    &sampling_down_max_momentum.attr,
    &sampling_down_momentum_sensitivity.attr,
    &ignore_nice_load.attr,
    &powersave_bias.attr,
    &smooth_ui.attr,
    &target_residency.attr,
    &allowed_misses.attr,
    &boostpulse.attr,
    &boostfreq.attr,
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
    &two_phase_freq.attr,
    &semi_busy_threshold.attr,
    &semi_busy_clr_threshold.attr,
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
    &three_phase_freq.attr,
    &busy_threshold.attr,
    &busy_clr_threshold.attr,
#endif
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
    &gpu_semi_busy_threshold.attr,
    &gpu_semi_busy_clr_threshold.attr,
    &gpu_busy_threshold.attr,
    &gpu_busy_clr_threshold.attr,
#endif
    NULL
};

static struct attribute_group dbs_attr_group = {
    .attrs = dbs_attributes,
    .name = "wheatleyplus",
};

/************************** sysfs end ************************/

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
    if (dbs_tuners_ins.powersave_bias)
	freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
    else if (p->cur == p->max)
	return;

    __cpufreq_driver_target(p, freq, dbs_tuners_ins.powersave_bias ?
			    CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
    unsigned int max_load;

    struct cpufreq_policy *policy;
    unsigned int j;
    unsigned int boostfreq;
    unsigned int up_threshold = dbs_tuners_ins.up_threshold;
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
	static unsigned int phase = 0;
	static unsigned int counter = 0;
	unsigned int new_phase_max = 0;
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
	static unsigned int gpu_busy_counter = 0;
	static unsigned int gpu_busy_phase = 0;
#endif
#endif

    unsigned long total_idletime, total_usage;

    this_dbs_info->freq_lo = 0;
    policy = this_dbs_info->cur_policy;

/* Only core0 controls the boost */
	if (dbs_tuners_ins.boosted && policy->cpu == 0) {
		if (ktime_to_us(ktime_get()) - whfreq_boosted_time >=
					dbs_tuners_ins.whfreq_boost_time) {
			dbs_tuners_ins.boosted = 0;
		}
	}
	if (dbs_tuners_ins.boostfreq != 0)
		boostfreq = dbs_tuners_ins.boostfreq;
	else
		boostfreq = policy->max;


    /*
     * Every sampling_rate, we calculate the relative load (percentage of 
     * time spend outside of idle) and the usage and average residency of 
     * the highest C-state during the last sampling interval.
     *
     * If the highest C-state has been used and the average residency is
     * greater or equal the user-defined target_residency or the relative 
     * load is above up_threshold percent, we increase the frequency to 
     * maximum (or stay there if we already are at maximum).
     *
     * If the highest C-state has not been used or the average residency
     * too low, we note that and if it happens more than allowed_misses
     * times in a row, we look for a the lowest frequency which can sustain
     * the current load with a relative load value below (up_threshold - 
     * down_differential) percent. If such a frequency exists, we decrease
     * to this frequency.
     */

    /* 
     * Get load (in terms of the current frequency)
     * and usage and average residency of the highest C-state 
     */

    max_load = 0;
    total_idletime = 0;
    total_usage = 0;

    for_each_cpu(j, policy->cpus) {
	struct cpu_dbs_info_s *j_dbs_info;
	cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
	unsigned int idle_time, wall_time, iowait_time;
	unsigned int load;
	struct cpuidle_device * j_cpuidle_dev = NULL;
//	struct cpuidle_state * deepidle_state = NULL;
//	unsigned long long deepidle_time, deepidle_usage;

	j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

	cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
	cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

	wall_time = (unsigned int) (cur_wall_time - j_dbs_info->prev_cpu_wall);
	j_dbs_info->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int) (cur_idle_time - j_dbs_info->prev_cpu_idle);
	j_dbs_info->prev_cpu_idle = cur_idle_time;

	iowait_time = (unsigned int) (cur_iowait_time - j_dbs_info->prev_cpu_iowait);
	j_dbs_info->prev_cpu_iowait = cur_iowait_time;

	if (dbs_tuners_ins.ignore_nice) {
	    cputime64_t cur_nice;
	    unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
	    /*
	     * Assumption: nice time between sampling periods will
	     * be less than 2^32 jiffies for 32 bit sys
	     */
	    cur_nice_jiffies = (unsigned long)
		cputime64_to_jiffies64(cur_nice);

	    j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	    idle_time += jiffies_to_usecs(cur_nice_jiffies);
	}

	/*
	 * For the purpose of wheatleyplus, waiting for disk IO is an
	 * indication that you're performance critical, and not that
	 * the system is actually idle. So subtract the iowait time
	 * from the cpu idle time.
	 */

	if (idle_time >= iowait_time)
	    idle_time -= iowait_time;

	if (unlikely(!wall_time || wall_time < idle_time))
	    continue;

	load = 100 * (wall_time - idle_time) / wall_time;

	if (load > max_load)
	    max_load = load;

	j_cpuidle_dev = per_cpu(cpuidle_devices, j);

/*
	if (j_cpuidle_dev)
	    deepidle_state = &j_cpuidle_dev->states[j_cpuidle_dev->state_count - 1];

	if (deepidle_state) {
	    deepidle_time = deepidle_state->time;
	    deepidle_usage = deepidle_state->usage;

	    total_idletime += (unsigned long)(deepidle_time - j_dbs_info->prev_idletime);
	    total_usage += (unsigned long)(deepidle_usage - j_dbs_info->prev_idleusage);

	    j_dbs_info->prev_idletime = deepidle_time;
	    j_dbs_info->prev_idleusage = deepidle_usage;
	}
*/
    }

    if (total_usage > 0 && total_idletime / total_usage >= dbs_tuners_ins.target_residency) { 
	if (num_misses > 0)
	    num_misses--;
    } else {
	if (num_misses <= dbs_tuners_ins.allowed_misses)
	    num_misses++;
    }

	/* Check for frequency increase */
	if ((dbs_tuners_ins.smooth_ui && touch_state_val) ||
	     max_load > up_threshold*policy->cur || num_misses <= dbs_tuners_ins.allowed_misses) {
#ifndef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
		if (counter < 0)
			counter = 0;

		if (counter < MAX_IDLE_COUNTER) {
			if ((counter < dbs_tuners_ins.semi_busy_threshold) && (phase == 0))
				counter += 12;
			else
				counter += 4;
			if ((counter > dbs_tuners_ins.semi_busy_threshold) && (phase < 1)) {
				/* change to semi-busy phase (3) */
				phase = 1;
			}
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
			if ((counter > dbs_tuners_ins.busy_threshold) && (phase < 2)) {
				/* change to busy phase (full) */
				phase = 2;
			}
#endif
		}

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
		if (gpu_busy_counter < 0)
			gpu_busy_counter = 0;

		if ((gpu_busy_counter < GPU_MAX_IDLE_COUNTER) &&
		    (gpu_busy_state_wp == true)) {
			gpu_busy_counter += GPU_COUNTER_INCREASE;
			if ((gpu_busy_counter > dbs_tuners_ins.gpu_semi_busy_threshold) && (gpu_busy_phase < 1)) {
				/* change to semi-busy phase (3) */
				gpu_busy_phase = 1;
			}
			if ((gpu_busy_counter > dbs_tuners_ins.gpu_busy_threshold) && (gpu_busy_phase < 2)) {
				/* change to busy phase (full) */
				gpu_busy_phase = 2;
			}
		} else if (gpu_busy_state_wp == false) {
			if (gpu_busy_counter > 0) {
				if ((gpu_busy_phase >= 1) && (gpu_busy_counter >= (DECREASE_GPU_IDLE_COUNTER/2)))
					gpu_busy_counter -= (DECREASE_GPU_IDLE_COUNTER/2);
				if ((gpu_busy_phase > 1) && (gpu_busy_counter >= DECREASE_GPU_IDLE_COUNTER))
					gpu_busy_counter -= DECREASE_GPU_IDLE_COUNTER;
			}
		}
/*
 * Debug output for gpu control. Still needed for finetuning.
 *
 *		printk(KERN_INFO "wheatleyplus: gpu_busy_phase: '%i' |			\
 *			 gpu_busy_counter: '%i' | busy? '%s'", gpu_busy_phase,		\
 *			 gpu_busy_counter, (gpu_busy_state_wp)?"true":"false");
 */
#endif

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
		if ((dbs_tuners_ins.two_phase_freq != 0 && ((phase == 0) || (gpu_busy_phase == 0)))) {
#else
		if ((dbs_tuners_ins.two_phase_freq != 0) && (phase == 0)) {
#endif
			/* idle phase */
			if (dbs_tuners_ins.two_phase_freq > (policy->max*PHASE_2_PERCENT/100)) {
				new_phase_max = (policy->max*PHASE_2_PERCENT/100);
			} else {
				new_phase_max = dbs_tuners_ins.two_phase_freq;
			}
			dbs_freq_increase(policy, new_phase_max);
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
		} else if (dbs_tuners_ins.three_phase_freq != 0 && ((phase == 1) || (gpu_busy_phase == 1))) {
#else
		} else if ((dbs_tuners_ins.three_phase_freq != 0) && (phase == 1)) {
#endif
			/* semi-busy phase */
			if (dbs_tuners_ins.three_phase_freq > (policy->max*PHASE_3_PERCENT/100)) {
				new_phase_max = (policy->max*PHASE_3_PERCENT/100);
			} else {
				new_phase_max = dbs_tuners_ins.three_phase_freq;
			}
			dbs_freq_increase(policy, new_phase_max);
#endif
		} else {
			/* busy phase */
			if (policy->cur < policy->max)
				this_dbs_info->rate_mult =
					dbs_tuners_ins.sampling_down_factor;
			dbs_freq_increase(policy, policy->max);
		}
#endif
		return;

		if (this_dbs_info->momentum_adder <
		    dbs_tuners_ins.sampling_down_mom_sens) {
			if (dbs_tuners_ins.smooth_ui && touch_state_val)
				this_dbs_info->momentum_adder =
				dbs_tuners_ins.sampling_down_mom_sens;
			else
				this_dbs_info->momentum_adder++;
			dbs_tuners_ins.sampling_down_momentum =
				(this_dbs_info->momentum_adder *
				  dbs_tuners_ins.sampling_down_max_mom) /
				  dbs_tuners_ins.sampling_down_mom_sens;
			dbs_tuners_ins.sampling_down_factor =
				orig_sampling_down_factor +
				dbs_tuners_ins.sampling_down_momentum;
		}
		return;
	}

		unsigned int freq_next;

		/* check for frequency boost */
		if (dbs_tuners_ins.boosted && policy->cur < boostfreq) {
			dbs_freq_increase(policy, boostfreq);
			return;
		}

		/* Calculate momentum and update sampling down factor */

		if (this_dbs_info->momentum_adder > 1) {
			this_dbs_info->momentum_adder -= 2;
			dbs_tuners_ins.sampling_down_momentum =
				(this_dbs_info->momentum_adder *
			 	dbs_tuners_ins.sampling_down_max_mom) /
			 	dbs_tuners_ins.sampling_down_mom_sens;
			dbs_tuners_ins.sampling_down_factor =
				orig_sampling_down_factor +
			 	dbs_tuners_ins.sampling_down_momentum;
		}

	/* Calculate the next frequency proportional to load */
		freq_next = max_load * policy->cpuinfo.max_freq / 100;

		if (dbs_tuners_ins.boosted &&
				freq_next < boostfreq) {
			freq_next = boostfreq;
		}

		/* No longer fully busy, reset rate_mult */
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!dbs_tuners_ins.powersave_bias) {
			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_2_PHASE
		if (counter > 0) {
			if (counter >= DECREASE_IDLE_COUNTER)
				counter -= DECREASE_IDLE_COUNTER;
			if ((counter > 0) && (counter < DECREASE_IDLE_COUNTER))
				counter--;

#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_3_PHASE
			if ((counter < dbs_tuners_ins.busy_clr_threshold) && (phase > 1)) {
				/* change to semi busy phase */
				phase = 1;
			}
#endif
			if ((counter < dbs_tuners_ins.semi_busy_clr_threshold) && (phase > 0)) {
				/* change to idle phase */
				phase = 0;
			}
		}
#ifdef CONFIG_CPU_FREQ_GOV_WHEATLEYPLUS_GPU_CONTROL
		if (gpu_busy_counter > 0) {
			if (gpu_busy_counter > (GPU_MAX_IDLE_COUNTER - (GPU_MAX_IDLE_COUNTER*10/100)))
				gpu_busy_counter -= DECREASE_GPU_IDLE_COUNTER*25;
			else if (gpu_busy_counter > DECREASE_GPU_IDLE_COUNTER)
				gpu_busy_counter -= DECREASE_GPU_IDLE_COUNTER;
			else if (gpu_busy_counter > 0)
				gpu_busy_counter--;

			if ((gpu_busy_counter < dbs_tuners_ins.gpu_busy_clr_threshold) && (gpu_busy_phase > 1)) {
				/* change to semi-busy phase */
				gpu_busy_phase = 1;
			}
			if ((gpu_busy_counter < dbs_tuners_ins.gpu_semi_busy_clr_threshold) && (gpu_busy_phase > 0)) {
				/* change to idle phase */
				gpu_busy_phase = 0;
			}
		}
	}
#endif
#endif

static void do_dbs_timer(struct work_struct *work)
{
    struct cpu_dbs_info_s *dbs_info =
	container_of(work, struct cpu_dbs_info_s, work.work);
    unsigned int cpu = dbs_info->cpu;
    int sample_type = dbs_info->sample_type;

    int delay;

    mutex_lock(&dbs_info->timer_mutex);

 	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			/* We want all CPUs to do sampling nearly on
			 * same jiffy
			 */
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
		delay = dbs_info->freq_lo_jiffies;
	}
    queue_delayed_work_on(cpu, kwheatley_wq, &dbs_info->work, delay);
    mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
    /* We want all CPUs to do sampling nearly on same jiffy */
    int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
    delay -= jiffies % delay;

    dbs_info->sample_type = DBS_NORMAL_SAMPLE;
    INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
    queue_delayed_work_on(dbs_info->cpu, kwheatley_wq, &dbs_info->work,
			  delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
    cancel_delayed_work_sync(&dbs_info->work);
}

/*
 * Not all CPUs want IO time to be accounted as busy; this dependson how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (androidlcom) calis this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void powersave_early_suspend(struct early_suspend *handler)
{
	dbs_tuners_ins.smooth_ui = 0;
	dbs_tuners_ins.sampling_down_max_mom = 0;
	dbs_tuners_ins.sampling_rate *= 4;
}

static void powersave_late_resume(struct early_suspend *handler)
{
	dbs_tuners_ins.smooth_ui = 1;
	dbs_tuners_ins.sampling_down_max_mom =
		orig_sampling_down_max_mom;
	dbs_tuners_ins.sampling_rate = orig_sampling_rate;
}

static struct early_suspend _powersave_early_suspend = {
	.suspend = powersave_early_suspend,
	.resume = powersave_late_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};
#endif

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event)
{
    unsigned int cpu = policy->cpu;
    struct cpu_dbs_info_s *this_dbs_info;
    unsigned int j;
    int rc;

    this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

    switch (event) {
    case CPUFREQ_GOV_START:
	if ((!cpu_online(cpu)) || (!policy->cur))
	    return -EINVAL;

	mutex_lock(&dbs_mutex);

	dbs_enable++;
	for_each_cpu(j, policy->cpus) {
	    struct cpu_dbs_info_s *j_dbs_info;
	    j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
	    j_dbs_info->cur_policy = policy;

	    j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
							  &j_dbs_info->prev_cpu_wall);
	    if (dbs_tuners_ins.ignore_nice) {
		j_dbs_info->prev_cpu_nice =
		    kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	    }
	}
	this_dbs_info->cpu = cpu;
	this_dbs_info->rate_mult = 1;
	this_dbs_info->momentum_adder = 0;
	wheatleyplus_powersave_bias_init_cpu(cpu);
	num_misses = 0;
	/*
	 * Start the timerschedule work, when this governor
	 * is used for first time
	 */
	if (dbs_enable == 1) {
	    unsigned int latency;

	    rc = sysfs_create_group(cpufreq_global_kobject,
				    &dbs_attr_group);
	    if (rc) {
		mutex_unlock(&dbs_mutex);
		return rc;
	    }

	    /* policy latency is in nS. Convert it to uS first */
	    latency = policy->cpuinfo.transition_latency / 1000;
	    if (latency == 0)
		latency = 1;
	    /* Bring kernel and HW constraints together */
	    min_sampling_rate = max(min_sampling_rate,
				    MIN_LATENCY_MULTIPLIER * latency);
	    dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);
			orig_sampling_rate = dbs_tuners_ins.sampling_rate;
			orig_sampling_down_factor =
				dbs_tuners_ins.sampling_down_factor;
			orig_sampling_down_max_mom =
				dbs_tuners_ins.sampling_down_max_mom;
	}
	mutex_unlock(&dbs_mutex);

	mutex_init(&this_dbs_info->timer_mutex);
	dbs_timer_init(this_dbs_info);
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&_powersave_early_suspend);
#endif
	break;

    case CPUFREQ_GOV_STOP:
	dbs_timer_exit(this_dbs_info);

	mutex_lock(&dbs_mutex);
	mutex_destroy(&this_dbs_info->timer_mutex);
	dbs_enable--;
	mutex_unlock(&dbs_mutex);
	if (!dbs_enable)
	    sysfs_remove_group(cpufreq_global_kobject,
			       &dbs_attr_group);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&_powersave_early_suspend);
#endif
	break;

    case CPUFREQ_GOV_LIMITS:
	mutex_lock(&this_dbs_info->timer_mutex);
	if (policy->max < this_dbs_info->cur_policy->cur)
	    __cpufreq_driver_target(this_dbs_info->cur_policy,
				    policy->max, CPUFREQ_RELATION_H);
	else if (policy->min > this_dbs_info->cur_policy->cur)
	    __cpufreq_driver_target(this_dbs_info->cur_policy,
				    policy->min, CPUFREQ_RELATION_L);
	mutex_unlock(&this_dbs_info->timer_mutex);
	break;
    }
    return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
    cputime64_t wall;
    u64 idle_time;
    int cpu = get_cpu();
    int err;

    idle_time = get_cpu_idle_time_us(cpu, &wall);
    put_cpu();
    if (idle_time != -1ULL) {
	/* Idle micro accounting is supported. Use finer thresholds */
	dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
	/*
	 * In no_hz/micro accounting case we set the minimum frequency
	 * not depending on HZ, but fixed (very low). The deferred
	 * timer might skip some samples if idle/sleeping as needed.
	 */
	min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
    } else {
	/* For correct statistics, we need 10 ticks for each measure */
	min_sampling_rate =
	    MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
    }

    kwheatley_wq = create_workqueue("kwheatley");
    if (!kwheatley_wq) {
	printk(KERN_ERR "Creation of kwheatley failed\n");
	return -EFAULT;
    }
    err = cpufreq_register_governor(&cpufreq_gov_wheatleyplus);
    if (err)
	destroy_workqueue(kwheatley_wq);


    return cpufreq_register_governor(&cpufreq_gov_wheatleyplus);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
    cpufreq_unregister_governor(&cpufreq_gov_wheatleyplus);
    destroy_workqueue(kwheatley_wq);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_AUTHOR("Ezekeel <notezekeel@googlemail.com>");
MODULE_AUTHOR("Jovy23@XDA");
MODULE_DESCRIPTION("'cpufreq_wheatleyplus' - A dynamic cpufreq governor for "
		   "Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_WHEATLEYPLUS
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
