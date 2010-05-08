
#define TC_PER_TCB	3

struct atmel_tcblock {
	u32 		physaddr;
	void __iomem	*ioaddr;
	struct clk	*clk[TC_PER_TCB];
	int		irq[TC_PER_TCB];
};

extern void __init atmel_tc_init(struct atmel_tcblock *tcblocks, int n);
