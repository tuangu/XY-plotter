# XY Plotter
This is a LPC1549-based firmware to control Makeblock XY-plotter with laser engraver update.

[See the plotter in action](https://imgur.com/AzgXh5j)

## Requirements
* Hardware
    * [Makeblock XY-plotter](http://store.makeblock.com/xy-plotter-robot-kit)
    * [LPC1549 LPCXpresso Board](http://www.embeddedartists.com/products/lpcxpresso/lpc1549_xpr.php)
* Software
    * [mDraw](https://github.com/Makeblock-official/mDrawBot) to control the plotter.
    * [MCUXpresso IDE](https://www.nxp.com/support/developer-resources/run-time-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide-v10.0.2:MCUXpresso-IDE) to build the source code.

## How to run it
* Clone this repository and import all projects into MCUXpresso IDE.
* Build the `plotter` project.
* Flash the binary to a LPC1549 LPCXpresso board which is already connected to a Makeblock XY-plotter.
* Connect the LPCXpresso board to a computer through a USB-serial port.
* Use mDraw to control the plotter.

## Dependencies
* LPCOpen Libraries for LPC1549
* FreeRTOS v8.0

## Contributors
* [perseus0832](https://github.com/perseus0832)
* [minhhungft9](https://github.com/minhhungft9)
* [tuangu](https://github.com/tuangu)





