/*
 * linux/arch/arm/mach-sa1100/irq.c
 *
 * Copyright (C) 1999-2001 Nicolas Pitre
 *
 * Generic IRQ handling for the SA11x0, GPIO 11-27 IRQ demultiplexing.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>

#include <asm/exception.h>

#include "generic.h"

#define ICIP	0x00  /* IC IRQ Pending reg.             */
#define ICMR	0x04  /* IC Mask Reg.                    */
#define ICLR	0x08  /* IC Level Reg.                   */
#define ICCR	0x0C  /* IC Control Reg.                 */
#define ICFP	0x10  /* IC FIQ Pending reg.             */
#define ICPR	0x20  /* IC Pending Reg.                 */

#define IC_GPIO(Nb)	        	/* GPIO [0..10]                    */ \
			(0x00000001 << (Nb))
#define IC_GPIO0	IC_GPIO (0)	/* GPIO  [0]                       */
#define IC_GPIO1	IC_GPIO (1)	/* GPIO  [1]                       */
#define IC_GPIO2	IC_GPIO (2)	/* GPIO  [2]                       */
#define IC_GPIO3	IC_GPIO (3)	/* GPIO  [3]                       */
#define IC_GPIO4	IC_GPIO (4)	/* GPIO  [4]                       */
#define IC_GPIO5	IC_GPIO (5)	/* GPIO  [5]                       */
#define IC_GPIO6	IC_GPIO (6)	/* GPIO  [6]                       */
#define IC_GPIO7	IC_GPIO (7)	/* GPIO  [7]                       */
#define IC_GPIO8	IC_GPIO (8)	/* GPIO  [8]                       */
#define IC_GPIO9	IC_GPIO (9)	/* GPIO  [9]                       */
#define IC_GPIO10	IC_GPIO (10)	/* GPIO [10]                       */
#define IC_GPIO11_27	0x00000800	/* GPIO [11:27] (ORed)             */

static void __iomem *sc_irq_base;

/*
 * We don't need to ACK IRQs on the SA1100 unless they're GPIOs
 * this is for internal IRQs i.e. from 11 to 31.
 */
static void sa1100_mask_irq(struct irq_data *d)
{
	uint32_t icmr = readl_relaxed(sc_irq_base + ICMR);

	icmr &= ~(1 << d->irq);
	writel_relaxed(icmr, sc_irq_base + ICMR);
}

static void sa1100_unmask_irq(struct irq_data *d)
{
	uint32_t icmr = readl_relaxed(sc_irq_base + ICMR);

	icmr |= 1 << d->irq;
	writel_relaxed(icmr, sc_irq_base + ICMR);
}

/*
 * Apart form GPIOs, only the RTC alarm can be a wakeup event.
 */
static int sa1100_set_wake(struct irq_data *d, unsigned int on)
{
	return sa11x0_sc_set_wake(d->irq, on);
}

static struct irq_chip sa1100_normal_chip = {
	.name		= "SC",
	.irq_ack	= sa1100_mask_irq,
	.irq_mask	= sa1100_mask_irq,
	.irq_unmask	= sa1100_unmask_irq,
	.irq_set_wake	= sa1100_set_wake,
};

static struct resource irq_resource =
	DEFINE_RES_MEM_NAMED(0x90050000, SZ_64K, "irqs");

static struct sa1100irq_state {
	unsigned int	saved;
	unsigned int	icmr;
	unsigned int	iclr;
	unsigned int	iccr;
} sa1100irq_state;

static int sa1100irq_suspend(void)
{
	struct sa1100irq_state *st = &sa1100irq_state;

	st->saved = 1;
	st->icmr = readl_relaxed(sc_irq_base + ICMR);
	st->iclr = readl_relaxed(sc_irq_base + ICLR);
	st->iccr = readl_relaxed(sc_irq_base + ICCR);

	/*
	 * Disable all GPIO-based interrupts.
	 */
	writel_relaxed(st->icmr &
		~(IC_GPIO11_27|IC_GPIO10|IC_GPIO9|IC_GPIO8|IC_GPIO7|
		  IC_GPIO6|IC_GPIO5|IC_GPIO4|IC_GPIO3|IC_GPIO2|
		  IC_GPIO1|IC_GPIO0), sc_irq_base + ICMR);


	return 0;
}

static void sa1100irq_resume(void)
{
	struct sa1100irq_state *st = &sa1100irq_state;

	if (st->saved) {
		writel_relaxed(st->iccr, sc_irq_base + ICCR);
		writel_relaxed(st->iclr, sc_irq_base + ICLR);
		writel_relaxed(st->icmr, sc_irq_base + ICMR);
	}
}

static struct syscore_ops sa1100irq_syscore_ops = {
	.suspend	= sa1100irq_suspend,
	.resume		= sa1100irq_resume,
};

static int __init sa1100irq_init_devicefs(void)
{
	register_syscore_ops(&sa1100irq_syscore_ops);
	return 0;
}

device_initcall(sa1100irq_init_devicefs);

void __init sa1100_init_irq(void)
{
	unsigned int irq;

	request_resource(&iomem_resource, &irq_resource);

	sc_irq_base = ioremap(irq_resource.start, resource_size(&irq_resource));

	/* disable all IRQs */
	writel_relaxed(0, sc_irq_base + ICMR);

	/* all IRQs are IRQ, not FIQ */
	writel_relaxed(0, sc_irq_base + ICLR);

	/*
	 * Whatever the doc says, this has to be set for the wait-on-irq
	 * instruction to work... on a SA1100 rev 9 at least.
	 */
	writel_relaxed(1, sc_irq_base + ICCR);

	for (irq = 0; irq <= 31; irq++) {
		irq_set_chip_and_handler(irq, &sa1100_normal_chip,
					 handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}
}

asmlinkage void __exception_irq_entry sa1100_handle_irq(struct pt_regs *regs)
{
	uint32_t icip, icmr, mask;

	do {
		icip = readl_relaxed(sc_irq_base + ICIP);
		icmr = readl_relaxed(sc_irq_base + ICMR);
		mask = icip & icmr;

		if (mask == 0)
			break;

		handle_IRQ(fls(mask) - 1, regs);
	} while (1);
}
