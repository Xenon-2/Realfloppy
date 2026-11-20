#pragma once
#include "hardware/spi.h"
#include <cstdint>
// --- Clock and Conversion Ticks Greaseweazle (100 MHz) ---
#define SYS_CLOCK_KHZ 150000
#define GW_SAMPLE_FREQ_HZ 100000000

#define CY_TO_TICKS(cy) ((uint32_t)(((uint64_t)(cy) * 100000ULL) / SYS_CLOCK_KHZ))
#define TICKS_TO_CY(tk) ((uint32_t)(((uint64_t)(tk) * SYS_CLOCK_KHZ) / 100000ULL))
#define US_TO_CY(us) ((uint32_t)(((uint64_t)(us) * SYS_CLOCK_KHZ) / 1000ULL))

// pin driver
/*
pin 14 select a   gpio 28
pin 10 motor a    gpio 22
pin 12 select b   gpio 03
pin 16 motor b    gpio 04
*/

// --- Pinout Hardware Floppy ---
#define DENSITY_PIN 1 // GP1  - IDC 2  (output)
#define INDEX_PIN 2   // GP2  - IDC 8  (input)

#define SELECT_A_PIN 28 // GP28 - IDC 14 (output) drive A (5.25" 360K)
#define MOTOR_A_PIN  22 // GP22 - IDC 10 (output) drive A (5.25" 360K)
#define SELECT_B_PIN 3  // GP3  - IDC 12 (output) drive B (3.5")
#define MOTOR_B_PIN  4  // GP4  - IDC 16 (output) drive B (3.5")

#define DIR_PIN 5     // GP5  - IDC 18 (output)
#define STEP_PIN 6    // GP6  - IDC 20 (output)
#define WRDATA_PIN 7  // GP7  - IDC 22 (output)
#define WRGATE_PIN 8  // GP8  - IDC 24 (output)
#define TRK0_PIN 9    // GP9  - IDC 26 (input)
#define PROT_PIN 10   // GP10 - IDC 28 (input)
#define READ_PIN 11   // GP11 - IDC 30 (input)
#define SIDE_PIN 12   // GP12 - IDC 32 (output)
#define READY_PIN 13  // GP13 - IDC 34 (input)

// Fail-Safe Default: Drive B 3.5" + RealFloppy
extern volatile uint8_t g_active_select_pin;
extern volatile uint8_t g_active_motor_pin;
extern volatile uint32_t g_active_step_us;
extern volatile bool g_rf_mode_active;
extern volatile bool g_mode_changed;

// --- Buffers ---
#define CMD_BUF_LEN 32
#define DELAYS_LEN 8
#define IDX_PKT_LEN 6
#define TIMES_LEN 16
#define RESP_BUF_LEN 64
#define XFER_BUF_LEN 512
#define FLUX_BUF_LEN 95000
#define TX_CHUNK_LEN 1024
#define IDX_ARRAY_LEN 8

// SPI connection

#define SD_SPI_PORT spi0
#define SD_PIN_MISO 16
#define SD_PIN_CS   17
#define SD_PIN_SCK  18
#define SD_PIN_MOSI 19

// I2C connection with arduino

#define I2C_PORT i2c1
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15

// FT232RL 

#define TX_PIN 20      // GP20 -> FT232RL RXD
#define RX_PIN 21      // GP21 <- FT232RL TXD
#define CTS_OUT_PIN 26 // GP26 -> FT232RL CTS# (attivo basso)

// Common flux buffer
extern uint16_t flux_buf[FLUX_BUF_LEN];

