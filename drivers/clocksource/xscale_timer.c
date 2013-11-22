/*
 * PXA/SA1100 clocksource, clockevents, and OST interrupt handlers.
 * Copyright (c) 2007 by Bill Gatliff <bgat@billgatliff.com>.
 *
 * Derived from Nicolas Pitre's PXA timer handler Copyright (c) 2001
 * by MontaVista Software, Inc.  (Nico, your code rocks!)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/sched_clock.h>

/*
 * This is PXA/SA11x0's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */

#define OSMR0  		0x00  /* OS timer Match Reg. 0 */
#define OSMR1  		0x04  /* OS timer Match Reg. 1 */
#define OSMR2  		0x08  /* OS timer Match Reg. 2 */
#define OSMR3  		0x0c  /* OS timer Match Reg. 3 */
#define OSMR4  		0x80  /* OS timer Match Reg. 4 */
#define OSCR		0x10  /* OS Timer Counter Register */
#define OSCR4		0x40  /* OS Timer Counter Register */
#define OMCR4		0xC0  /* */
#define OSSR		0x14  /* OS Timer Status Register */
#define OWER		0x18  /* OS Timer Watchdog Enable Register */
#define OIER		0x1C  /* OS Timer Interrupt Enable Register */

#define OSSR_M3		(1 << 3)	/* Match status channel 3 */
#define OSSR_M2		(1 << 2)	/* Match status channel 2 */
#define OSSR_M1		(1 << 1)	/* Match status channel 1 */
#define OSSR_M0		(1 << 0)	/* Match status channel 0 */

#define OWER_WME	(1 << 0)	/* Watchdog Match Enable */

#define OIER_E3		(1 << 3)	/* Interrupt enable channel 3 */
#define OIER_E2		(1 << 2)	/* Interrupt enable channel 2 */
#define OIER_E1		(1 << 1)	/* Interrupt enable channel 1 */
#define OIER_E0		(1 << 0)	/* Interrupt enable channel 0 */


static void __iomem *reg_base;

static u32 notrace xscale_read_sched_clock(void)
{
	return readl_relaxed(reg_base + OSCR);
}

#define MIN_OSCR_DELTA 16

static irqreturn_t
xscale_ost0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* Disarm the compare/match, signal the event. */
	writel_relaxed(readl_relaxed(reg_base + OIER) & ~OIER_E0,
			reg_base + OIER);
	writel_relaxed(OSSR_M0, reg_base + OSSR);
	c->event_handler(c);

	return IRQ_HANDLED;
}

static int
xscale_osmr0_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long next, oscr;

	writel_relaxed(readl_relaxed(reg_base + OIER) | OIER_E0,
			reg_base + OIER);
	next = readl_relaxed(reg_base + OSCR) + delta;
	writel_relaxed(next, reg_base + OSMR0);
	oscr = readl_relaxed(reg_base + OSCR);

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
xscale_osmr0_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		writel_relaxed(readl_relaxed(reg_base + OIER) & ~OIER_E0,
				reg_base + OIER);
		writel_relaxed(OSSR_M0, reg_base + OSSR);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

#ifdef CONFIG_PM
static unsigned long osmr[4], oier, oscr;

static void xscale_timer_suspend(struct clock_event_device *cedev)
{
	osmr[0] = readl_relaxed(reg_base + OSMR0);
	osmr[1] = readl_relaxed(reg_base + OSMR1);
	osmr[2] = readl_relaxed(reg_base + OSMR2);
	osmr[3] = readl_relaxed(reg_base + OSMR3);
	oier = readl_relaxed(reg_base + OIER);
	oscr = readl_relaxed(reg_base + OSCR);
}

static void xscale_timer_resume(struct clock_event_device *cedev)
{
	/*
	 * Ensure that we have at least MIN_OSCR_DELTA between match
	 * register 0 and the OSCR, to guarantee that we will receive
	 * the one-shot timer interrupt.  We adjust OSMR0 in preference
	 * to OSCR to guarantee that OSCR is monotonically incrementing.
	 */
	if (osmr[0] - oscr < MIN_OSCR_DELTA)
		osmr[0] = oscr + MIN_OSCR_DELTA;

	writel_relaxed(OSSR_M0 | OSSR_M1 | OSSR_M2 | OSSR_M3, reg_base + OSSR);
	writel_relaxed(osmr[0], reg_base + OSMR0);
	writel_relaxed(osmr[1], reg_base + OSMR1);
	writel_relaxed(osmr[2], reg_base + OSMR2);
	writel_relaxed(osmr[3], reg_base + OSMR3);
	writel_relaxed(oier, reg_base + OIER);
	writel_relaxed(oscr, reg_base + OSCR);
}
#else
#define xscale_timer_suspend NULL
#define xscale_timer_resume NULL
#endif

static struct clock_event_device ckevt_xscale_osmr0 = {
	.name		= "osmr0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= xscale_osmr0_set_next_event,
	.set_mode	= xscale_osmr0_set_mode,
	.suspend	= xscale_timer_suspend,
	.resume		= xscale_timer_resume,
};

static struct irqaction xscale_timer_irq = {
	.name		= "ost0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= xscale_ost0_interrupt,
	.dev_id		= &ckevt_xscale_osmr0,
};

void __init xscale_timer_init(void __iomem *base,
		int irq,
		unsigned long clock_tick_rate)
{
	reg_base = base;

	writel_relaxed(0, reg_base + OIER);
	writel_relaxed(OSSR_M0 | OSSR_M1 | OSSR_M2 | OSSR_M3, reg_base + OSSR);

	setup_sched_clock(xscale_read_sched_clock, 32, clock_tick_rate);

	ckevt_xscale_osmr0.cpumask = cpumask_of(0);

	setup_irq(irq, &xscale_timer_irq);

	clocksource_mmio_init(reg_base + OSCR, "oscr0", clock_tick_rate, 200, 32,
		clocksource_mmio_readl_up);
	clockevents_config_and_register(&ckevt_xscale_osmr0, clock_tick_rate,
		MIN_OSCR_DELTA * 2, 0x7fffffff);
}
