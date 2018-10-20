imx-mu
======

i.MX Messaging Unit Linux Kernel Module

Description
===========

This loadable Linux kernel module can be used with multicore i.MX SoCs
containing a Messaging Unit peripheral.

The module creates four character devices `/dev/mu0` till `/dev/mu3`,
which are used to read data from and write data into the Messaging Unit.

Building and Running
====================

The example has been tested on __UDOO Neo__ board with the Linux distribution
__UDOObuntu 2.2.0__ running on the A9 core. The M4 core was running
the __Zephyr RTOS__ sample application `ipm_imx` located here:

[https://github.com/zephyrproject-rtos/zephyr/tree/master/samples/subsys/ipc/ipm\_imx](https://github.com/zephyrproject-rtos/zephyr/tree/master/samples/subsys/ipc/ipm_imx)

The Zephyr application simply echoes the data written to the Messaging
Unit back.

Since this module replaces the kernel's own handling of the Messaging Unit,
the kernel has to be patched.

Clone the GIT repository with the UDOObuntu version of the Linux kernel:

    $ git clone https://github.com/UDOOboard/linux_kernel.git

Navigate into the linux\_kernel folder and remember its location:

    $ cd linux_kernel
    $ export KERNEL_PATH=`pwd`

Checkout at SHA 5111ea94bbd0818035351fce39c83f1dbf81457b:

    $ git checkout 5111ea94bbd0818035351fce39c83f1dbf81457b

Copy the file 0001-arm-imx-disable-mcc-and-mu-driver-support.patch into
the linux\_kernel folder and apply the patch to disable kernel's own handling
of Messaging Unit:

    $ git apply 0001-arm-imx-disable-mcc-and-mu-driver-support.patch

Define the kernel architecture:

    $ export ARCH=arm

Provide path to the compiler toolchain:

    $ export CROSS_COMPILE=~/bin/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-

Create .config file for the UDOObuntu Linux kernel:

    $ make udoo_neo_defconfig

Build the Linux kernel:

    $ make

Create a folder for Linux kernel modules and build them:

    $ mkdir /tmp/modpath/
    $ export INSTALL_MOD_PATH=/tmp/modpath/
    $ make modules_install

Boot the UDOO Neo board.

Replace the file /boot/zImage in the UDOObuntu filesystem with the file
arch/arm/boot/zImage from the build filesystem.

Replace the .dtb files in the UDOObuntu filesystem with the .dtb files
from the arch/arm/boot/dts/ folder in the build filesystem.

Copy the contents of the `${INSTALL_MOD_PATH}/lib` from the build filesystem
into the `/lib` folder in the UDOObuntu filesystem.

Reboot the UDOO Neo board to apply the modified kernel.

Go back to this repository.

Make sure the `LINUX_KERNEL` and `CROSS_COMPILE` variables have been set
in the previous steps and build the kernel module:

    $ make

Copy the Linux kernel module file `mu.ko` into the UDOObuntu filesystem.

Use serial console or SSH to log in the UDOObuntu system.

Load the Zephyr sample code to run on the M4 core.

    # mqx_upload_on_m4SoloX zephyr.bin

Building of the Zephyr firmware is described here:

[https://docs.zephyrproject.org/latest/getting\_started/getting\_started.html](https://docs.zephyrproject.org/latest/getting\_started/getting\_started.html)

Load the kernel module:

    # insmod mu.ko

It creates four character devices:

    # ls -la /dev/mu*
    crw------- 1 root root 247, 0 Jan  1 05:33 /dev/mu0
    crw------- 1 root root 247, 1 Jan  1 05:33 /dev/mu1
    crw------- 1 root root 247, 2 Jan  1 05:33 /dev/mu2
    crw------- 1 root root 247, 3 Jan  1 05:33 /dev/mu3

Each of the devices uses one pair of the Messaging Unit's receive and transmit
registers to communicate with the Zephyr application. One byte of the register
is used for payload length, other three for the payload. The Zephyr application
does not interpret this and just echoes the data back.

You can open any of the devices from the console for writing the payload:

     # cat > /dev/mu0
     abcdefgh
     ^D

Open the device from another console for reading and observe the data echoed
back using the Zephyr application:

     # cat /dev/mu0
     abcdefgh
     ^C

Close all open devices and remove the kernel module:

    # rmmod mu

You can check the debug output of the kernel module:

    # dmesg
    [ 1154.944467] mu_probe
    [ 1162.583453] open /dev/mu0, fd=0xa87a3e40
    [ 1167.700685] open /dev/mu0, fd=0xa85e6000
    [ 1167.700821] read /dev/mu0, fd=0xa85e6000, length=65536
    [ 1172.331693] write /dev/mu0, fd=0xa87a3e40, length=9
    [ 1172.331802] write /dev/mu0, fd=0xa87a3e40, written=9
    [ 1172.331832] mu_isr /dev/mu0 tx 3 bytes: 0x03636261
    [ 1172.331858] mu_isr /dev/mu0 tx 3 bytes: 0x03666564
    [ 1172.335270] mu_isr /dev/mu0 rx 3 bytes: 0x03636261
    [ 1172.335334] mu_isr /dev/mu0 tx 3 bytes: 0x030a6867
    [ 1172.335413] read /dev/mu0, fd=0xa85e6000, read 3 bytes
    [ 1172.335519] read /dev/mu0, fd=0xa85e6000, length=65536
    [ 1172.338824] mu_isr /dev/mu0 rx 3 bytes: 0x03666564
    [ 1172.338895] read /dev/mu0, fd=0xa85e6000, read 3 bytes
    [ 1172.338964] read /dev/mu0, fd=0xa85e6000, length=65536
    [ 1172.342390] mu_isr /dev/mu0 rx 3 bytes: 0x030a6867
    [ 1172.342804] read /dev/mu0, fd=0xa85e6000, read 3 bytes
    [ 1172.342893] read /dev/mu0, fd=0xa85e6000, length=65536
    [ 1173.526658] close /dev/mu0, fd=0xa87a3e40
    [ 1176.765493] read /dev/mu0, fd=0xa85e6000, read 0 bytes
    [ 1176.765893] close /dev/mu0, fd=0xa85e6000
    [ 1186.156877] mu_remove
