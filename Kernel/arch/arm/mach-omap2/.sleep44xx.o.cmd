cmd_arch/arm/mach-omap2/sleep44xx.o := /home/user/Kernel/tcc/bin/arm-none-eabi-gcc -Wp,-MD,arch/arm/mach-omap2/.sleep44xx.o.d  -nostdinc -isystem /home/user/Kernel/tcc/bin/../lib/gcc/arm-none-eabi/4.4.1/include -I/home/user/Kernel/LG_Waggle_Kernel/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2       -c -o arch/arm/mach-omap2/sleep44xx.o arch/arm/mach-omap2/sleep44xx.S

deps_arch/arm/mach-omap2/sleep44xx.o := \
  arch/arm/mach-omap2/sleep44xx.S \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/cache/l2x0.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/linkage.h \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/arch/has/barriers.h) \
    $(wildcard include/config/arm/dma/mem/bufferable.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/hardware/cache-l2x0.h \
  arch/arm/mach-omap2/include/mach/omap4-common.h \
    $(wildcard include/config/pm.h) \
  arch/arm/plat-omap/include/plat/omap44xx.h \
    $(wildcard include/config/base.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/memory.h \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/highmem.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
  include/linux/const.h \
  arch/arm/mach-omap2/include/mach/memory.h \
  arch/arm/plat-omap/include/plat/memory.h \
    $(wildcard include/config/arch/omap1.h) \
    $(wildcard include/config/arch/omap15xx.h) \
    $(wildcard include/config/fb/omap/consistent/dma/size.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
    $(wildcard include/config/sparsemem.h) \

arch/arm/mach-omap2/sleep44xx.o: $(deps_arch/arm/mach-omap2/sleep44xx.o)

$(deps_arch/arm/mach-omap2/sleep44xx.o):
