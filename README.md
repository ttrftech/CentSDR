CentSDR - Tiny Standalone Software Defined Receiver
==========================================================

<div align="center">
<img src="/doc/centsdr.jpg" width="480px">
</div>

# About

CentSDR is tiny handheld standalone software defined receiver with LCD display,
that is simple, low budget, but has reasonable perfomance.
This project is aimed at contributing to studies, experiments and educations around
RF technology. 

# Block Diagram

<div align="center">
<img src="/doc/centsdr-blockdiagram.png" width="480px">
</div>

# Build Firmware

## Prepare ARM Cross Tools

Install cross tools and firmware updating tool.
 
    $ brew cask install gcc-arm-embedded
	
## Fetch Source

Clone source code from github.

    $ git clone https://github.com/ttrftech/CentSDR centsdr

Then fetch ChibiOS submodule into tree.

    $ cd centsdr
    $ git submodule update --init --recursive

Just make in the top directory.

    $ make

## Flash firmware

### Using OpenOCD and gdb

Prepare openocd.

    $ brew install openocd

Connect ST-Link2 to target board, then launch openocd as follows.

    $ openocd -f board/stm32f3discovery.cfg

Flash firmware using gdb.

    $ arm-none-eabi-gdb 
    > target extended-remote :4242
    > exec build/ch.elf 
    > load
    > quit

Or use gdb script to flash.

    $ arm-none-eabi-gdb -x flash-openocd.gdb --silent

### Using st-util and gdb

Prepare stlink utilities.

    $ brew install stlink

Connect target board via SWD with ST-Link2, In other terminal, launch st-util

    $ st-util

Then, flash the firmware.

    $ arm-none-eabi-gdb -x flash-stutil.gdb --silent

## Flash firmware using Nucleo st-link v2.1

Or you can flash the firmware using Nucleo. First, mount as usb mass storage device, then copy 'build/ch.bin' file into mounted volume.

## Attention

This repository contains only source of CentSDR firmware, but NO hardware design resources.

## Reference

* Kit available from http://ttrftech.tumblr.com/kit/centsdr
* Credit: @edy555

[EOF]
