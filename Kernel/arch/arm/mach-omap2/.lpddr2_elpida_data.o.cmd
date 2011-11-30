cmd_arch/arm/mach-omap2/lpddr2_elpida_data.o := /home/user/Kernel/tcc/bin/arm-none-eabi-gcc -Wp,-MD,arch/arm/mach-omap2/.lpddr2_elpida_data.o.d  -nostdinc -isystem /home/user/Kernel/tcc/bin/../lib/gcc/arm-none-eabi/4.4.1/include -I/home/user/Kernel/LG_Waggle_Kernel/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -DTARGET_CARRIER_ -DTARGET_COUNTRY_   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(lpddr2_elpida_data)"  -D"KBUILD_MODNAME=KBUILD_STR(lpddr2_elpida_data)"  -c -o arch/arm/mach-omap2/lpddr2_elpida_data.o arch/arm/mach-omap2/lpddr2_elpida_data.c

deps_arch/arm/mach-omap2/lpddr2_elpida_data.o := \
  arch/arm/mach-omap2/lpddr2_elpida_data.c \
  arch/arm/mach-omap2/include/mach/emif.h \
    $(wildcard include/config/devct/1.h) \
    $(wildcard include/config/devwdt/32.h) \
  arch/arm/mach-omap2/include/mach/emif-44xx.h \
    $(wildcard include/config/2.h) \
  arch/arm/mach-omap2/include/mach/lpddr2-jedec.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /home/user/Kernel/LG_Waggle_Kernel/arch/arm/include/asm/posix_types.h \

arch/arm/mach-omap2/lpddr2_elpida_data.o: $(deps_arch/arm/mach-omap2/lpddr2_elpida_data.o)

$(deps_arch/arm/mach-omap2/lpddr2_elpida_data.o):
