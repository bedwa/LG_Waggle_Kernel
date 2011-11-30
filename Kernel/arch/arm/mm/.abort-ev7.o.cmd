cmd_arch/arm/mm/abort-ev7.o := /home/user/Kernel/tcc/bin/arm-none-eabi-gcc -Wp,-MD,arch/arm/mm/.abort-ev7.o.d  -nostdinc -isystem /home/user/Kernel/tcc/bin/../lib/gcc/arm-none-eabi/4.4.1/include -I/home/user/Kernel/LG_Waggle_Kernel/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork  -D__LINUX_ARM_ARCH__=7 -march=armv7-a  -include asm/unified.h -msoft-float -gdwarf-2     -Wa,-march=armv7-a  -c -o arch/arm/mm/abort-ev7.o arch/arm/mm/abort-ev7.S

deps_arch/arm/mm/abort-ev7.o := \
  arch/arm/mm/abort-ev7.S \
    $(wildcard include/config/verify/permission/fault.h) \
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
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/smp.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/hwcap.h \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
    $(wildcard include/config/cpu/use/domains.h) \

arch/arm/mm/abort-ev7.o: $(deps_arch/arm/mm/abort-ev7.o)

$(deps_arch/arm/mm/abort-ev7.o):
