#ifndef __ASM_PARAVIRT_H
#define __ASM_PARAVIRT_H
/* Various instructions on x86 need to be replaced for
 * para-virtualization: those hooks are defined here. */
#include <linux/linkage.h>
#include <linux/stringify.h>
#include <asm/page.h>

#ifdef CONFIG_PARAVIRT

#ifndef __ASSEMBLY__

struct paravirt_ops
{
	int paravirt_enabled;
};

#define paravirt_enabled() (paravirt_ops.paravirt_enabled)

extern struct paravirt_ops paravirt_ops;

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PARAVIRT */
#endif	/* __ASM_PARAVIRT_H */
