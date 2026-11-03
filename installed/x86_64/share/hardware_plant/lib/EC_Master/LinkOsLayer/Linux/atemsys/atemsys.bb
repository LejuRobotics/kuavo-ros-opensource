SUMMARY = "acontis atemsys user mode driver"
LICENSE = ""

inherit module

SRC_URI = "file://Makefile \
           file://atemsys.c \
           file://atemsys.h \
          "
S = "${WORKDIR}"

export KERNELDIR="${KERNEL_SRC}"

# inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
