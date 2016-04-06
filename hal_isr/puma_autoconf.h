#ifndef _PUMA_AUTOCONF_H
#define _PUMA_AUTOCONF_H

#ifndef __KERNEL__
#define CONFIG_MACH_PUMA6
#endif

#define SYSTEM_PUMA5_SOC_ID    (50)
#define SYSTEM_PUMA6_SOC_ID    (60)

#ifndef __KERNEL__
/* If SoC ID is Puma5 */
#define PUMA5_SOC_TYPE  (CONFIG_SYSTEM_PUMA_SOC_ID == SYSTEM_PUMA5_SOC_ID)

/* If SoC ID is Puma6 */
#define PUMA6_SOC_TYPE  (CONFIG_SYSTEM_PUMA_SOC_ID == SYSTEM_PUMA6_SOC_ID)

/* If SoC ID is Puma6 or newer (ex. Puma6MG, Puma7...) */
#define PUMA6_OR_NEWER_SOC_TYPE  (CONFIG_SYSTEM_PUMA_SOC_ID >= SYSTEM_PUMA6_SOC_ID)
#endif

#ifdef __KERNEL__
/* If SoC ID is Puma5 */
#define PUMA5_SOC_TYPE  (defined (CONFIG_MACH_PUMA5))

/* If SoC ID is Puma6 */
#define PUMA6_SOC_TYPE  (defined (CONFIG_MACH_PUMA6))

/* If SoC ID is Puma6 or newer (ex. Puma6, Puma7...) */
#define PUMA6_OR_NEWER_SOC_TYPE  (defined (CONFIG_MACH_PUMA6))
#endif

#endif

