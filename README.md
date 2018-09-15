# oak8x
A fully open source Asterisk FXO/S module which supports up to 8 channels and based on Raspberry Pi 

# OAK8X Manual
### Compile the new Kernel dtbs files to support tdm bus
Follow this link to download Raspberry Pi Kernel https://www.raspberrypi.org/documentation/linux/kernel/building.md
```shell
cd ~/linux/arch/arm/boot/dts
vim bcm283x.dtsi
```
Go to the section for DMA and follow below to remove interrupts 25, 26 and DMA 9, 10.

```shell
                dma: dma@7e007000 {
                        compatible = "brcm,bcm2835-dma";
                        reg = <0x7e007000 0xf00>;
                        interrupts = <1 16>,
                                     <1 17>,
                                     <1 18>,
                                     <1 19>,
                                     <1 20>,
                                     <1 21>,
                                     <1 22>,
                                     <1 23>,
                                     <1 24>,
                                     <1 25>, //remove
                                     <1 26>, //remove
                                     /* dma channel 11-14 share one irq */
                      
			  interrupt-names = "dma0",
                                          "dma1",
                                          "dma2",
                                          "dma3",
                                          "dma4",
                                          "dma5",
                                          "dma6",
                                          "dma7",
                                          "dma8",
                                          "dma9”,  //remove
                                          "dma10”, //remove
                                          "dma11",
                                          "dma12",
                                          "dma13",
```

Then add below tdm bus definition after the i2s section:

```shell
                i2s: i2s@7e203000 {
                        compatible = "brcm,bcm2835-i2s";
                        reg = <0x7e203000 0x24>;
                        clocks = <&clocks BCM2835_CLOCK_PCM>;

                        dmas = <&dma 2>,
                               <&dma 3>;
                        dma-names = "tx", "rx";
                        status = "disabled";
                };

                // new section to add for oak8x:
                tdm: tdm@7e203000 {
                        compatible = "brcm,pi-tdm";
                        reg = <0x7e203000 0x20>,
                              <0x7e101098 0x02>;

                        dmas = <&dma 2>,
                               <&dma 3>;
                        dma-names = "tx", "rx";
                        interrupts = <1 25>, <1 26>;
                        interrupt-names = "dma9", "dma10";
                        status = "okay";
                };

```
```shell
cd ~/linux
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
```
when done
```shell
mkdir pidtbs
cp arch/arm/boot/dts/*.dtb pidtbs/
```
Then copy the new dtb files to the RaspberryPi /boot folder, make sure you have a backup before you overwrite them.

### Disable the spi and i2s bus in /boot/config.txt file like below shows
```shell
dtparam=i2c_arm=on
#dtparam=i2s=on
#dtparam=spi=on
```

### Compile the dahdi
Get the source code from https://github.com/lixinswitchpi/oak.git
```shell
cd /usr/src/dahdi-linux
make
make install
```

### Compile the dahdi-tools

### Recompile the Asterisk to add dahdi supports
install required packages
```shell
apt install libncurses5-dev uuid-dev libjansson-dev libxml2-dev libsqlite3-dev openssl libssl-dev build-essential
```
Download the asterisk-13.20.0 and recompile it, remember to check up the chan-dahdi

### The detailed manual
Please check out the detailed manual in https://switchpi.com/2018/08/29/manual-of-install-oak8x-module/

