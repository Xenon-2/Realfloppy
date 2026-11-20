#pragma once
#include <cstdint>


// Global Flag set from IRQ I2C Arduino sending CMD:ENTER_FS
extern volatile bool g_fs_mode_requested;

// Circular queur eith arduino mega 
void fs_i2c_queue_msg(const char *msg);
uint8_t fs_i2c_pop_tx_byte(void);

// call from IRQ I2C 
void fs_i2c_on_rx_line(const char *line);

// FloppyStation loop (exit only on CMD:EXIT_FS)
void floppystation_run(void);
