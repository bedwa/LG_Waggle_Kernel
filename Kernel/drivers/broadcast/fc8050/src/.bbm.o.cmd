cmd_drivers/broadcast/fc8050/src/bbm.o := /home/user/Kernel/tcc/bin/arm-none-eabi-gcc -Wp,-MD,drivers/broadcast/fc8050/src/.bbm.o.d  -nostdinc -isystem /home/user/Kernel/tcc/bin/../lib/gcc/arm-none-eabi/4.4.1/include -I/home/user/Kernel/LG_Waggle_Kernel/Kernel/arch/arm/include -Iinclude  -include include/generated/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-omap2/include -Iarch/arm/plat-omap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=7 -march=armv7-a -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -DTARGET_CARRIER_ -DTARGET_COUNTRY_   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(bbm)"  -D"KBUILD_MODNAME=KBUILD_STR(bbm)"  -c -o drivers/broadcast/fc8050/src/bbm.o drivers/broadcast/fc8050/src/bbm.c

deps_drivers/broadcast/fc8050/src/bbm.o := \
  drivers/broadcast/fc8050/src/bbm.c \
  drivers/broadcast/fc8050/src/../inc/fci_types.h \
  drivers/broadcast/fc8050/src/../inc/fci_tun.h \
  drivers/broadcast/fc8050/src/../inc/fci_types.h \
  drivers/broadcast/fc8050/src/../inc/fc8050_regs.h \
  drivers/broadcast/fc8050/src/../inc/fc8050_bb.h \
  drivers/broadcast/fc8050/src/../inc/fci_hal.h \
  drivers/broadcast/fc8050/src/../inc/fc8050_isr.h \

drivers/broadcast/fc8050/src/bbm.o: $(deps_drivers/broadcast/fc8050/src/bbm.o)

$(deps_drivers/broadcast/fc8050/src/bbm.o):
