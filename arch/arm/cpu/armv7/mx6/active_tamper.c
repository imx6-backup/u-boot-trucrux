#include <common.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/snvs.h>


static void configure_dryice_ext_tamper(unsigned int tx, unsigned int rx)
{
	int val;
	int reg = DRY_TPCTRL0_OFFSET + rx * 4;

	val = (0x0<<28) |
          (0x1<<27) |
          (0x0<<21) |
          (0x1<<20) | // use extended sample period
          ((tx - 4)<<17) | // active tamper N
          (0x0<<16) |
          (0x1<<15) | // pull always asserts the tamper pin
          (0x1<<14) |
          (0x1<<13) |
#ifdef MX6UL_TO_1_1
          (0x1<<11) | // Let the tamper pin work as tamper pin, for mx6ul chip >= TO1.1
#endif
          (0x0<<10) |
          (0x1<<7) |
          (0x1<<6) |
          (0x1<<4) |  // tamper pin sample frequency is once every 32 cycles
          (0x3<<2) |  // tamper pin sampling 4 cycles
          (0x0<<0);

	writel(val, reg);
}

static void stop_dryice_ext_tamper(unsigned int tx, unsigned int rx)
{
	int val;
	int reg = DRY_TPCTRL0_OFFSET + rx * 4;

	val = (0x0<<28) |
		  (0x1<<27) |
          (0x0<<21) |
          (0x1<<20) | // use extended sample period
          ((tx - 4)<<17) | // active tamper N
          (0x0<<16) |
          (0x1<<15) | // pull always asserts the tamper pin
          (0x1<<14) |
          (0x1<<13) |
          (0x0<<10) |
          (0x1<<7) |
          (0x1<<6) |
          (0x1<<4) |  // tamper pin sample frequency is once every 32 cycles
          (0x3<<2) |  // tamper pin sampling 4 cycles
          (0x0<<0),
	writel(val, reg);

	val = (0x0<<28) |
          (0x0<<27) | // use 32kHz as IGWCS
          (0x0<<21) |
          (0x1<<20) | // use extended sample period
          ((tx - 4)<<17) | // active tamper N
          (0x0<<16) |
          (0x1<<15) | // pull always asserts the tamper pin
          (0x0<<14) | // disable tamper pin
          (0x1<<13) |
          (0x0<<10) |
          (0x1<<7) |
          (0x0<<6) |  // use 32kHz as TSCS
          (0x1<<4) |  // tamper pin sample frequency is once every 32 cycles
          (0x0<<2) |  // disabled tamper pin sampling
          (0x0<<0),
	writel(val, reg);

}

void active_tamper_test(unsigned int tx, unsigned int rx)
{
	int val, tamper_pin;
	printf("start active tamper test on %d -> %d\n", tx, rx);

	/*********************************************
	 *   Configuring shadow registers to enable tamper pin  *
	 *********************************************/
	val = readl(OCOTP_LOCK) & 0x3;
	tamper_pin = (readl(OCOTP_CFG2) & 0x300000) >> 20;
	if((tamper_pin != 0) && val) {
		printf("tamper fuse is programmed, tamper shadow register is locked, can't verify tamper function\n");
		return;
	}
	if((tamper_pin != 0) && (val == 0)){
		printf("set OCOTP_CFG2 shadow register to enable tamper detect mode\n");
		val = readl(OCOTP_CFG2);
		val &= ~0x300000;
		writel(val, OCOTP_CFG2);
	}

	/****************************
	 *   Configuring CAAM and SNVS  *
	 ****************************/

	/* Initialize power glitch detector register */
	val = 0x41736166;
	writel(val, SNVS_LPPGDR);

	/* W1C PGD */
	val = readl(SNVS_LPSR) & 0x00000008;
	writel(val, SNVS_LPSR);

	/* Programming ZMK via SW */
	writel(0x11110000, SNVS_LPZMKR0);
	writel(0x22220000, SNVS_LPZMKR1);
	writel(0x33330000, SNVS_LPZMKR2);
	writel(0x44440000, SNVS_LPZMKR3);
	writel(0x55550000, SNVS_LPZMKR4);
	writel(0x66660000, SNVS_LPZMKR5);
	writel(0x77770000, SNVS_LPZMKR6);
	writel(0x88880000, SNVS_LPZMKR7);

	val = readl(SNVS_LPMKCR) | 0xa;
	writel(val, SNVS_LPMKCR);
	val = readl(SNVS_HPCOMR) | 0x1000;
	writel(val, SNVS_HPCOMR);

	val = readl(SNVS_LPMKCR) | 0x10;
	writel(val, SNVS_LPMKCR);

	val = readl(SNVS_HPSVSR);

	/* LP Security Violation is a non-fatal Violation */
	val = 0x40000000;
	writel(val, SNVS_HPSVCR);

	/* Enable SRTC invalidation in case of security violation */
	val = readl(SNVS_LPCR);
	val |= 0x11;
	writel(val, SNVS_LPCR);


	/*********************************
	 *   Configuring active tamper tx output  *
	 *********************************/

	/* Configure LFSR polynomial and seed for active tamper tx */
	val = AT5_POLYSEED;
	writel(val, SNVS_LPAT1CR + (tx - 5) * 4);

	/* Enable active tamper tx external pad */
	val = readl(SNVS_LPATCTLR) | (1 << (tx - 5 + 16));
	writel(val, SNVS_LPATCTLR);

	/* Enable active tamper tx clk 16hz */
	val = readl(SNVS_LPATCLKR);
	val &= ~(3 << (tx - 5) * 4);
	writel(val, SNVS_LPATCLKR);

	/* Enable active tamper tx LFSR */
	val = readl(SNVS_LPATCTLR) | (1 << (tx - 5));
	writel(val, SNVS_LPATCTLR);

	/* Enable glitch filter for external tamper rx */
	if (rx < 2) {
		val = readl(SNVS_LPTGFCR);
		if (rx == 0)
			val |= 0x800000;
		else if (rx == 1)
			val |= 0x80000000;
		writel(val, SNVS_LPTGFCR);
	} else if (rx < 6){
		val = readl(SNVS_LPTGF1CR);
		val |= 1 << ((rx - 1) * 8 - 1);
		writel(val, SNVS_LPTGF1CR);
	} else {
		val = readl(SNVS_LPTGF2CR);
		val |= 1 << ((rx - 5) * 8 - 1);
		writel(val, SNVS_LPTGF2CR);
	}

	/* Route active tamper tx to external tamper rx */
	if (rx < 8) {
		val = readl(SNVS_LPATRC1R);
		val &= ~(0xf << (rx * 4));
		val |= ((tx - 4) << (rx * 4));
		writel(val, SNVS_LPATRC1R);
	} else {
		val = readl(SNVS_LPATRC2R);
		val &= ~(0xf << ((rx - 8) * 4));
		val |= ((tx - 4) << ((rx - 8) * 4));
		writel(val, SNVS_LPATRC2R);
	}


	/* Enable external tamper rx */
	if (rx < 2) {
		val = readl(SNVS_LPTDCR);
		if (rx == 0)
			val |= 0x200;
		else if (rx == 1)
			val |= 0x400;
		writel(val, SNVS_LPTDCR);
	} else {
		val = readl(SNVS_LPTDC2R);
		val |= 1 << (rx - 2);
		writel(val, SNVS_LPTDC2R);
	}

	configure_dryice_ext_tamper(tx, rx);
}
