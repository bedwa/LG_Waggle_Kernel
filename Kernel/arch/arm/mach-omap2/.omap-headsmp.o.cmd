cmd_arch/arm/mach-omap2/omap-headsmp.o := /home/user/Kernel/tcc/bin/arm-none-eabi-gcc -Wp,-MD,arch/arm/mach-omap2/.omap-headsmp.o.d  -nostdinc -isystem /home/user/Kernel/tcc/bin/../lib/gcc/arm-none-eabi/4.4.1/include -I/home/user/Kernel/LG_Waggle_Kernel/Kernel/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2       -c -o arch/arm/mach-omap2/omap-headsmp.o arch/arm/mach-omap2/omap-headsmp.S

deps_arch/arm/mach-omap2/omap-headsmp.o := \
  arch/arm/mach-omap2/omap-headsmp.S \
  /home/user/Kernel/LG_Waggle_Kernel/Kernel/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/user/Kernel/LG_Waggle_Kernel/Kernel/arch/arm/include/asm/linkage.h \
  include/linux/init.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/hotplug.h) \

arch/arm/mach-omap2/omap-headsmp.o: $(deps_arch/arm/mach-omap2/omap-headsmp.o)

$(deps_arch/arm/mach-omap2/omap-headsmp.o):
