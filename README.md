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
  - 1x SPI MicroSD Card module
  - 1x Arduino with a touchscreen display

## Project Background & Story
This project goes way back to my desire to adapt PC emulators to use real, physical floppy drives. My benchmark test has always been booting from a 5.25" 360KB floppy drive (DD, not HD) and running *BIGTOP* through an Arduino Mega 2560-based controller using simple MFM track extraction and rewriting. 

About 4 years ago, I decided to build a "Floppystation" that, besides acting as a real floppy interface for my emulators, could also work as a standalone tool to backup and write floppy images using an SD card.

Over time, I modified several PC emulators to bring their performance closer to a real 4.77 MHz Intel 8088 PC interfaced with my "Realfloppy" hardware, though I had never reached 100% perfection. A few months ago, after seeing Reenigne's work (*Area 5150*), I wanted to understand how it worked under the hood. My previous emulators were accurate enough to play *BIGTOP*, but the `MIPS.COM` benchmark was never quite exact, always off by a few percentage points. 

So, I took the challenging path of keeping Reenigne's microcode-based core and building a complete emulator around it capable of running `MIPS.COM` both cycle-exact and time-exact. While there are wonderful projects on GitHub made by developers far more skilled than me, none of them hit that exact mark in terms of timing and performance. Eventually, I succeeded and published my first public result on GitHub: [IBM5150-emulator](https://github.com/Xenon-2/IBM5150-emulator).

Once that was done, I wanted a new challenge: building a low-cost Greaseweazle interface with zero extra components—just a Pico 1 or Pico 2. No clone issues, no complicated custom PCBs to solder. Sure, you can buy a ready-made board for 30 euros, but my goal is to build—and let others build—without frustration and at the lowest possible cost (just the price of a Pico).

That's how I created my own lightweight Greaseweazle implementation—without even using PIO state machines or DMA—that runs smoothly on both Pico 1 and Pico 2, while also integrating "Realfloppy" and standalone Floppystation capabilities.


## Wiring & Pinout

### 1. Floppy Drive Interface (34-Pin IDC Connector)
This connection is **required for both versions** (Greaseweazle-only and Full Floppystation). All floppy control signals are active-low.

| Raspberry Pi Pico | Floppy IDC Pin | Signal Name | Direction | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **GP1** | Pin 2 | `/DENSITY` | Output | Density Select (LOW = DD) |
| **GP2** | Pin 8 | `/INDEX` | Input | Index Pulse |
| **GP22** | Pin 10 | `/MOTOR_A` | Output | Motor Enable Drive A (5.25" 360K) |
| **GP3** | Pin 12 | `/SELECT_B` | Output | Drive Select B (3.5") - *Default* |
| **GP28** | Pin 14 | `/SELECT_A` | Output | Drive Select A (5.25" 360K) |
| **GP4** | Pin 16 | `/MOTOR_B` | Output | Motor Enable Drive B (3.5") |
| **GP5** | Pin 18 | `/DIR` | Output | Step Direction |
| **GP6** | Pin 20 | `/STEP` | Output | Step Pulse |
| **GP7** | Pin 22 | `/WRDATA` | Output | Write Data |
| **GP8** | Pin 24 | `/WRGATE` | Output | Write Gate |
| **GP9** | Pin 26 | `/TRK0` | Input | Track 00 Sensor |
| **GP10** | Pin 28 | `/PROT` | Input | Write Protect Sensor |
| **GP11** | Pin 30 | `/READ` | Input | Read Data |
| **GP12** | Pin 32 | `/SIDE` | Output | Side / Head Select |
| **GP13** | Pin 34 | `/READY` | Input | Disk Change / Ready |
| **GND** | Odd Pins (1–33) | `GND` | Common | Connect multiple odd pins to Pico GND |

> **Hardware Note:** 
> - If you only use a **single drive** in the Greaseweazle-only version, connect it to **Drive B** (`GP3` / `GP4`), as it is the fail-safe default at boot.
> - Floppy drives require an external **+5V** (and **+12V** for 5.25" drives) power supply. Share a common **GND** between the floppy PSU and the Pico.

---

### 2. Full Version Peripherals (Floppystation & Realfloppy)
If you are building the full version, connect the following modules to the Raspberry Pi Pico:

#### SD Card Module (SPI0)

| Raspberry Pi Pico | SD Card Module | Function |
| :--- | :--- | :--- |
| **GP16** | `MISO` | SPI0 RX |
| **GP17** | `CS` | Chip Select |
| **GP18** | `SCK` | SPI0 Clock |
| **GP19** | `MOSI` | SPI0 TX |
| **3V3 (OUT)** | `VCC` | 3.3V Power |
| **GND** | `GND` | Ground |

#### FT232RL USB-to-Serial Adapter (UART1 @ 2,000,000 baud)
Used for high-speed communication with the **Realfloppy** client and the **IBM5150-emulator**.

| Raspberry Pi Pico | FT232RL Pin | Function |
| :--- | :--- | :--- |
| **GP20** | `RXD` | Pico TX -> Adapter RX |
| **GP21** | `TXD` | Pico RX <- Adapter TX |
| **GP26** | `CTS#` | Hardware Flow Control (Active Low) |
| **GND** | `GND` | Ground |

> **Important:** Set the FT232RL voltage jumper to **3.3V** to safely interface with the Pico GPIO pins.

#### Arduino Mega + Touchscreen Display (I2C1 Slave @ `0x42`)
Used as the UI controller to switch modes (`Realfloppy`, `Greaseweazle`, `Floppystation`), select drives (`A:` / `B:`), and browse SD images.

| Raspberry Pi Pico | Arduino Mega | Function |
| :--- | :--- | :--- |
| **GP14** | `SDA` (Pin 20) | I2C1 Data |
| **GP15** | `SCL` (Pin 21) | I2C1 Clock |
| **GND** | `GND` | Common Ground |

> **Voltage Level Warning:** The Arduino Mega operates at **5V** logic while the Raspberry Pi Pico operates at **3.3V**. Use a bidirectional **I2C logic level shifter (3.3V <-> 5V)** on `SDA` and `SCL` to protect the Pico pins.
>


## Available Versions
Currently, there are two ready-to-use hardware setups in this repository, both sharing the same `.uf2` firmware (depending on whether you use a Pico 1 or Pico 2):
1. **Greaseweazle Device (Boot Default):** Works out-of-the-box with official Greaseweazle host tools. Requires only a Raspberry Pi Pico 1 or Pico 2.
2. **Floppystation (Full Setup):** Starts in Greaseweazle mode by default and allows switching via the touchscreen display to Realfloppy mode or standalone Floppystation mode (SD card read/write).

Both systems work seamlessly with both 3.5" and 5.25" floppy drives.


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
