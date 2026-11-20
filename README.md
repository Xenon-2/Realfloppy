# Floppystation & Realfloppy (Raspberry Pi Pico 1 / Pico 2)

A Floppystation and Realfloppy interface based on the Raspberry Pi Pico 1 or Pico 2.

## Features
- **Floppystation:** Capable of reading and writing floppy disk images directly to/from an SD card. Supports up to 2 floppy drives (both 3.5" and 5.25").
- **Greaseweazle:** Fully compatible with official Greaseweazle host tools.
- **Realfloppy:** Compatible with my custom client and cycle-exact emulator ([IBM5150-emulator](https://github.com/Xenon-2/IBM5150-emulator)).

## Required Hardware
- **Greaseweazle-only version:**
  - Just a Raspberry Pi Pico 1 or Pico 2 (no extra components required).
- **Full version (Realfloppy + Floppystation + Greaseweazle):**
  - Raspberry Pi Pico 1 or Pico 2
  - 1x FT232RL USB-to-Serial adapter
  - 1x Arduino with a touchscreen display

## Project Background & Story
This project goes way back to my desire to adapt PC emulators to use real, physical floppy drives. My benchmark test has always been booting from a 5.25" 360KB floppy drive (DD, not HD) and running *BIGTOP* through an Arduino Mega 2560-based controller using simple MFM track extraction and rewriting. 

About 4 years ago, I decided to build a "Floppystation" that, besides acting as a real floppy interface for my emulators, could also work as a standalone tool to backup and write floppy images using an SD card.

Over time, I modified several PC emulators to bring their performance closer to a real 4.77 MHz Intel 8088 PC interfaced with my "Realfloppy" hardware, though I had never reached 100% perfection. A few months ago, after seeing Reenigne's work (*Area 5150*), I wanted to understand how it worked under the hood. My previous emulators were accurate enough to play *BIGTOP*, but the `MIPS.COM` benchmark was never quite exact, always off by a few percentage points. 

So, I took the challenging path of keeping Reenigne's microcode-based core and building a complete emulator around it capable of running `MIPS.COM` both cycle-exact and time-exact. While there are wonderful projects on GitHub made by developers far more skilled than me, none of them hit that exact mark in terms of timing and performance. Eventually, I succeeded and published my first public result on GitHub: [IBM5150-emulator](https://github.com/Xenon-2/IBM5150-emulator).

Once that was done, I wanted a new challenge: building a low-cost Greaseweazle interface with zero extra components—just a Pico 1 or Pico 2. No clone issues, no complicated custom PCBs to solder. Sure, you can buy a ready-made board for 30 euros, but my goal is to build—and let others build—without frustration and at the lowest possible cost (just the price of a Pico).

That's how I created my own lightweight Greaseweazle implementation—without even using PIO state machines or DMA—that runs smoothly on both Pico 1 and Pico 2, while also integrating "Realfloppy" and standalone Floppystation capabilities.

## Available Versions
Currently, there are two ready-to-use setups in this repository but both share the same .elf :
1. **Greaseweazle Device:** Works with the official Greaseweazle tools. Requires only a Raspberry Pi Pico 1 or Pico 2.
2. **Floppystation:** Implements the Greaseweazle protocol, Realfloppy protocol, and a standalone Floppystation mode controlled via a touchscreen display.

Both systems work seamlessly with both 3.5" and 5.25" floppy drives.
