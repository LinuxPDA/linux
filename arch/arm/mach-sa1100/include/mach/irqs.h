/*
 * arch/arm/mach-sa1100/include/mach/irqs.h
 *
 * Copyright (C) 1996 Russell King
 * Copyright (C) 1998 Deborah Wallach (updates for SA1100/Brutus).
 * Copyright (C) 1999 Nicolas Pitre (full GPIO irq isolation)
 *
 * 2001/11/14	RMK	Cleaned up and standardised a lot of the IRQs.
 */

#define	IRQ_GPIO0_		0
#define	IRQ_GPIO1_		1
#define	IRQ_GPIO2_		2
#define	IRQ_GPIO3_		3
#define	IRQ_GPIO4_		4
#define	IRQ_GPIO5_		5
#define	IRQ_GPIO6_		6
#define	IRQ_GPIO7_		7
#define	IRQ_GPIO8_		8
#define	IRQ_GPIO9_		9
#define	IRQ_GPIO10_		10
#define	IRQ_GPIO11_27		11
#define	IRQ_LCD  		12	/* LCD controller           */
#define	IRQ_Ser0UDC		13	/* Ser. port 0 UDC          */
#define	IRQ_Ser1SDLC		14	/* Ser. port 1 SDLC         */
#define	IRQ_Ser1UART		15	/* Ser. port 1 UART         */
#define	IRQ_Ser2ICP		16	/* Ser. port 2 ICP          */
#define	IRQ_Ser3UART		17	/* Ser. port 3 UART         */
#define	IRQ_Ser4MCP		18	/* Ser. port 4 MCP          */
#define	IRQ_Ser4SSP		19	/* Ser. port 4 SSP          */
#define	IRQ_DMA0 		20	/* DMA controller channel 0 */
#define	IRQ_DMA1 		21	/* DMA controller channel 1 */
#define	IRQ_DMA2 		22	/* DMA controller channel 2 */
#define	IRQ_DMA3 		23	/* DMA controller channel 3 */
#define	IRQ_DMA4 		24	/* DMA controller channel 4 */
#define	IRQ_DMA5 		25	/* DMA controller channel 5 */
#define	IRQ_OST0 		26	/* OS Timer match 0         */
#define	IRQ_OST1 		27	/* OS Timer match 1         */
#define	IRQ_OST2 		28	/* OS Timer match 2         */
#define	IRQ_OST3 		29	/* OS Timer match 3         */
#define	IRQ_RTC1Hz		30	/* RTC 1 Hz clock           */
#define	IRQ_RTCAlrm		31	/* RTC Alarm                */

#define	IRQ_GPIO0		32
#define	IRQ_GPIO1		33
#define	IRQ_GPIO2		34
#define	IRQ_GPIO3		35
#define	IRQ_GPIO4		36
#define	IRQ_GPIO5		37
#define	IRQ_GPIO6		38
#define	IRQ_GPIO7		39
#define	IRQ_GPIO8		40
#define	IRQ_GPIO9		41
#define	IRQ_GPIO10		42
#define	IRQ_GPIO11		43
#define	IRQ_GPIO12		44
#define	IRQ_GPIO13		45
#define	IRQ_GPIO14		46
#define	IRQ_GPIO15		47
#define	IRQ_GPIO16		48
#define	IRQ_GPIO17		49
#define	IRQ_GPIO18		50
#define	IRQ_GPIO19		51
#define	IRQ_GPIO20		52
#define	IRQ_GPIO21		53
#define	IRQ_GPIO22		54
#define	IRQ_GPIO23		55
#define	IRQ_GPIO24		56
#define	IRQ_GPIO25		57
#define	IRQ_GPIO26		58
#define	IRQ_GPIO27		59

/*
 * The next 16 interrupts are for board specific purposes.  Since
 * the kernel can only run on one machine at a time, we can re-use
 * these.  If you need more, increase IRQ_BOARD_END, but keep it
 * within sensible limits.  IRQs 60 to 75 are available.
 */
#define IRQ_BOARD_START		60
#define IRQ_BOARD_END		76

/*
 * Figure out the MAX IRQ number.
 *
 * Neponset, SA1111 and UCB1x00 are sparse IRQ aware, so can dynamically
 * allocate their IRQs above NR_IRQS.
 *
 * LoCoMo has 4 additional IRQs, but is not sparse IRQ aware, and so has
 * to be included in the NR_IRQS calculation.
 */
#ifdef CONFIG_SHARP_LOCOMO
#define NR_IRQS_LOCOMO		4
#else
#define NR_IRQS_LOCOMO		0
#endif

#ifndef NR_IRQS
#define NR_IRQS (IRQ_BOARD_START + NR_IRQS_LOCOMO)
#endif
#define SA1100_NR_IRQS (IRQ_BOARD_START + NR_IRQS_LOCOMO)
