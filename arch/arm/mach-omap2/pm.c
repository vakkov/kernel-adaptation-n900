/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/opp.h>

#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include <plat/voltage.h>

#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include "pm.h"
#include "cm.h"
#include "cm-regbits-34xx.h"
#include "prm.h"

static struct omap_device_pm_latency *pm_lats;

static struct device *mpu_dev;
static struct device *iva_dev;
static struct device *l3_dev;
static struct device *dsp_dev;

static struct clk *dpll1_clk, *dpll2_clk, *dpll3_clk;

struct device *omap2_get_mpuss_device(void)
{
	WARN_ON_ONCE(!mpu_dev);
	return mpu_dev;
}

struct device *omap2_get_iva_device(void)
{
	WARN_ON_ONCE(!iva_dev);
	return iva_dev;
}

struct device *omap2_get_l3_device(void)
{
	WARN_ON_ONCE(!l3_dev);
	return l3_dev;
}

struct device *omap4_get_dsp_device(void)
{
	WARN_ON_ONCE(!dsp_dev);
	return dsp_dev;
}
EXPORT_SYMBOL(omap4_get_dsp_device);

/* static int _init_omap_device(struct omap_hwmod *oh, void *user) */
static int _init_omap_device(char *name, struct device **new_dev)
{
	struct omap_hwmod *oh;
	struct omap_device *od;

	oh = omap_hwmod_lookup(name);
	if (WARN(!oh, "%s: could not find omap_hwmod for %s\n",
		 __func__, name))
		return -ENODEV;

	od = omap_device_build(oh->name, 0, oh, NULL, 0, pm_lats, 0, false);
	if (WARN(IS_ERR(od), "%s: could not build omap_device for %s\n",
		 __func__, name))
		return -ENODEV;

	*new_dev = &od->pdev.dev;

	return 0;
}

static unsigned long omap3_mpu_get_rate(struct device *dev)
{
	return dpll1_clk->rate;
}

static int omap3_mpu_set_rate(struct device *dev, unsigned long rate)
{
	int ret;

	ret = clk_set_rate(dpll1_clk, rate);
	if (ret) {
		dev_warn(dev, "%s: Unable to set rate to %ld\n",
			__func__, rate);
		return ret;
	}

	return 0;
}

static int omap3_iva_set_rate(struct device *dev, unsigned long rate)
{
	return clk_set_rate(dpll2_clk, rate);
}

static unsigned long omap3_iva_get_rate(struct device *dev)
{
	return dpll2_clk->rate;
}

static int omap3_l3_set_rate(struct device *dev, unsigned long rate)
{
	int l3_div;

	l3_div = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;

	return clk_set_rate(dpll3_clk, rate * l3_div);
}

static unsigned long omap3_l3_get_rate(struct device *dev)
{
	int l3_div;

	l3_div = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;
	return dpll3_clk->rate / l3_div;
}


/*
 * Build omap_devices for processors and bus.
 */
static void omap2_init_processor_devices(void)
{
	_init_omap_device("mpu", &mpu_dev);
	_init_omap_device("iva", &iva_dev);
	if (cpu_is_omap44xx()) {
		_init_omap_device("l3_main_1", &l3_dev);
		_init_omap_device("dsp", &dsp_dev);
	} else {
		_init_omap_device("l3_main", &l3_dev);
	}

	if (cpu_is_omap34xx()) {
		dpll1_clk = clk_get(NULL, "dpll1_ck");
		dpll2_clk = clk_get(NULL, "dpll2_ck");
		dpll3_clk = clk_get(NULL, "dpll3_m2_ck");

		if (mpu_dev)
			omap_device_register_dvfs_callbacks(mpu_dev,
				omap3_mpu_set_rate, omap3_mpu_get_rate);
		if (iva_dev)
			omap_device_register_dvfs_callbacks(iva_dev,
				omap3_iva_set_rate, omap3_iva_get_rate);
		if (l3_dev)
			omap_device_register_dvfs_callbacks(l3_dev,
				omap3_l3_set_rate, omap3_l3_get_rate);

	}
}

/*
 * This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled.
 */
int omap_set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}

/*
 * This API is to be called during init to put the various voltage
 * domains to the voltage as per the opp table. Typically we boot up
 * at the nominal voltage. So this function finds out the rate of
 * the clock associated with the voltage domain, finds out the correct
 * opp entry and puts the voltage domain to the voltage specifies
 * in the opp entry
 */
static int __init omap2_set_init_voltage(char *vdd_name, char *clk_name,
						struct device *dev)
{
	struct voltagedomain *voltdm;
	struct clk *clk;
	struct opp *opp;
	unsigned long freq, bootup_volt;

	if (!vdd_name || !clk_name || !dev) {
		printk(KERN_ERR "%s: Invalid parameters!\n", __func__);
		goto exit;
	}

	voltdm = omap_voltage_domain_lookup(vdd_name);
	if (IS_ERR(voltdm)) {
		printk(KERN_ERR "%s: Unable to get vdd pointer for vdd_%s\n",
			__func__, vdd_name);
		goto exit;
	}

	clk =  clk_get(NULL, clk_name);
	if (IS_ERR(clk)) {
		printk(KERN_ERR "%s: unable to get clk %s\n",
			__func__, clk_name);
		goto exit;
	}

	freq = clk->rate;
	clk_put(clk);

	opp = opp_find_freq_ceil(dev, &freq);
	if (IS_ERR(opp)) {
		printk(KERN_ERR "%s: unable to find boot up OPP for vdd_%s\n",
			__func__, vdd_name);
		goto exit;
	}

	bootup_volt = opp_get_voltage(opp);
	if (!bootup_volt) {
		printk(KERN_ERR "%s: unable to find voltage corresponding"
			"to the bootup OPP for vdd_%s\n", __func__, vdd_name);
		goto exit;
	}

	omap_voltage_scale_vdd(voltdm, bootup_volt);
	return 0;

exit:
	printk(KERN_ERR "%s: Unable to put vdd_%s to its init voltage\n\n",
		__func__, vdd_name);
	return -EINVAL;
}

static void __init omap3_init_voltages(void)
{
	if (!cpu_is_omap34xx())
		return;

	omap2_set_init_voltage("mpu", "dpll1_ck", mpu_dev);
	omap2_set_init_voltage("core", "l3_ick", l3_dev);
}

static int __init omap2_common_pm_init(void)
{
	omap2_init_processor_devices();
	omap_pm_if_init();

	return 0;
}
device_initcall(omap2_common_pm_init);

static int __init omap2_common_pm_late_init(void)
{
	/* Init the OMAP TWL parameters */
	omap3_twl_init();

	/* Init the voltage layer */
	omap_voltage_late_init();

	/* Initialize the voltages */
	omap3_init_voltages();

	/* Smartreflex device init */
	omap_devinit_smartreflex();

	return 0;
}
late_initcall(omap2_common_pm_late_init);
