# SPDX-License-Identifier: GPL-2.0
#
# Build the esp32_wor module against the running kernel or a cross-compiled
# kernel source tree (set KDIR for cross-compilation).
#
# Usage (native on RPi):
#   make
#   sudo insmod esp32_wor.ko
#   sudo rmmod  esp32_wor
#
# Usage (cross-compile from macOS/Linux host for RPi):
#   make KDIR=/path/to/linux ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
#

obj-m := esp32_wor.o

KDIR  ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
