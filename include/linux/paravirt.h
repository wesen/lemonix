#ifndef __LINUX_PARAVIRT_H
#define __LINUX_PARAVIRT_H

#ifdef CONFIG_PARAVIRT
# include <asm/paravirt.h>
#endif

/*
 * Paravirtualization support
 */

#ifndef CONFIG_PARAVIRT
# define paravirt_enabled()	0
#endif

#endif
