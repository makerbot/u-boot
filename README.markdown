# U-Boot

These are based on TI's instructions for building u-boot. Make sure that the Angstrom toolchain is installed and set up in your environment before continuing.

Before you can follow these instructions, you should:

* [Set up a development environment](http://ops.makerbot.com/research:gen-5-development-environment)
* [Get a copy of the bootloader utility](http://gondor.makerbot.com/software/), or [build one yourself](http://ops.makerbot.com/research:gen-5-development-environment)


## Building
Use the [gen5-setup script](https://github.com/makerbot/gen5-setup-scripts) to set up your development environment; this will automatically clone the u-boot repository to the u-boot folder.

Build:
    cd u-boot
    make distclean CROSS_COMPILE=arm-angstrom-linux-gnueabi-
    make mb_rev_c_config CROSS_COMPILE=arm-angstrom-linux-gnueabi-
    make all CROSS_COMPILE=arm-angstrom-linux-gnueabi- -j8

The end result will be a file called u-boot.bin.

## Installation
Turn off the target board. Ensure that you have hooked up the serial cable to the board.
cd to the directory with the sfh_OMAP-L138 flashing tool. This should be 'bootutils/RevC_FlashAndBootUtils_2_29/OMAP-L138/GNU/'.
Run the loader:
For Linux:

    mono ./sfh_OMAP-L138.exe -flash -targetType AM1808 -flashType SPI_MEM -p /dev/ttyUSB0  ./ubl/ubl_AM1808_SPI_MEM.bin ~/makerbot/u-boot/u-boot.bin

In Ubuntu Linux 11.10, the parameter '—runtime=v4.0' may be required for mono.

For Windows (first, build the system in Linux, and copy u-boot.bin to the flash utilities directory):

    sfh_OMAP-L138.exe -flash -targetType AM1808 -flashType SPI_MEM -p COM20 ubl_AM1808_SPI_MEM.bin u-boot.bin

For OS/X, you need to get the mono framework (http://www.go-mono.com/mono-downloads/download.html), then you can use the same instructions as Linux.

Turn on the power for the target board, or hit the reset button if it's powered on.
Wait for the uploader to complete.

_Note: We've automated the bootloader on the production line, using a [python script](https://github.com/makerbot/pyReplicator/blob/master/burn_bootloader.py)_

## Erasing the flash chip
Erasing the flash chip will ensure that the u-boot environment is reset. This is required to reset the MAC Address.
To completely reset the flash memory, use the -erase switch:

    mono ./sfh_OMAP-L138.exe -erase -targetType AM1808 -flashType SPI_MEM -p /dev/tty.usbserial-A400fYVz


## Configuring U-Boot

To get to the U-Boot configuration prompt:

1. Connect an FTDI cable to the USB2TTL connector on the board
2. Run a console program (we recommend screen on Linux and OS/X, and putty on Windows), and connect to the FTDI serial port at 115200/n/1
3. Reset the board.
4. Press any key within 3 seconds, to interrupt the boot sequence.

U-boot is suprisingly flexible. Type in 'help' to see a list of possible commands.

_Note: We've automated U-Boot configuration on the production line, using a [python script](https://github.com/makerbot/pyReplicator/blob/master/UBootProgrammer.py)_

Here are some sample environment configurations:

### Boot from an SD card
By default, U-Boot should be configured to boot from an SD card. This is how to set the configuration:

    setenv bootfile 'boot/uImage'
    setenv bootcmd 'mmcinfo\; ext2load mmc 0 0xc0700000 ${bootfile}\; setenv bootargs console=ttyS1,115200n8 noinitrd root=/dev/mmcblk0p1 rootfstype=ext3 rw rootwait mem=64M ethaddr=${ethaddr}\; bootm'
    saveenv
    reset


### Network boot using DHCP
If you configure your host computer network port to serve DHCP, you can use this to set the configuration. Bootserver is the IP address of the TFTP/NFS server. This assumes that you have a DHCP server running on your network, and a single computer that is providing the kernel image via TFTP, and a root filesystem via NFS.

    setenv rootpath /home/YOUR_USERNAME/makerbot/workdir/filesys
    setenv tftpblocksize 1400
    setenv bootserver 192.168.2.6
    setenv bootcmd 'dhcp ${bootserver}:uImage; setenv bootargs console=ttyS1,115200n8 noinitrd rw ip=${ipaddr} root=/dev/nfs nfsroot=${bootserver}:${rootpath},nolock mem=64M ethaddr=${ethaddr};bootm'
    saveenv
    reset

*Note: The board needs to have an ethernet MAC address configured in order to boot over the network. Note that the address can only be configured once without erasing the flash:*
    setenv ethaddr 12:34:56:78:90:ab
    saveenv

### Network boot using a static IP address
If a DHCP server is not available, then the board can be configured to use static addresses:

    setenv nfshost <ip address on nfs host> 
    setenv rootpath <directory to mount> 
    setenv ipaddr <fixed ip of board>
    setenv bootargs console=ttyS2,115200n8 noinitrd rw ip=${ipaddr} root=/dev/nfs nfsroot=${nfshost}:${rootpath},nolock mem=64M ethaddr=${ethaddr}
    setenv bootfile uImage 
    setenv bootcmd 'tftp;bootm' 
    setenv tftpblocksize 1400

Here is a command sequence, that was used to boot from 10.0.0.11 on a Rev A board:

    setenv nfshost 10.0.0.11
    setenv rootpath /home/makerbot/workdir/filesys
    setenv ipaddr 10.0.0.13
    setenv bootargs console=ttyS1,115200n8 noinitrd rw ip=${ipaddr} root=/dev/nfs nfsroot=${nfshost}:${rootpath},nolock mem=64M 
    setenv serverip ${nfshost}
    setenv bootfile uImage 
    setenv bootcmd 'tftp;bootm' 
    setenv tftpblocksize 1400
    boot

The 'tftpblocksize' parameter reduces the size of the TFTP packets; the default may sometimes be larger than the MTU your network can handle. We've set it to a more conservative value above.

