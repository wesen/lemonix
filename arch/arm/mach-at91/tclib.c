#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "tclib.h"

static struct atmel_tcblock *blocks;
static int nblocks;

/*
 * Called from the processor-specific init to register the TC Blocks.
 */
void __init atmel_tc_init(struct atmel_tcblock *tcblocks, int n)
{
	blocks = tcblocks;
	nblocks = n;
}
