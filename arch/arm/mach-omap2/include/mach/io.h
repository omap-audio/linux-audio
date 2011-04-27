/*
 * arch/arm/mach-omap2/include/mach/io.h
 */

#include <plat/io.h>


/* Select ARM view IO behavior */
#ifdef CONFIG_INTERCONNECT_IO_POSTING
/*
 * ARM writes to devices are postable.  Further software
 * sychronization neeed ex: DSB or register read back
 */
#define IO_MAP_TYPE    MT_DEVICE
#else
/* ARM writes to devices are sychronized */
#define IO_MAP_TYPE    MT_MEMORY_SO
#endif



