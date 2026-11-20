#include "rf_protocol.h"
#include "bridge.h"

#include <cstdio>
#include <cstring>
#include <cctype>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "ff.h"
#include "floppystation.h"

// --- Pinout UART1 (FT232RL) ---
#define UART_ID uart1
#define UART_BAUD 2000000


#define LED_ON() gpio_put(PICO_DEFAULT_LED_PIN, 1)
#define LED_OFF() gpio_put(PICO_DEFAULT_LED_PIN, 0)
#define LED_TOGGLE() gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN))

#define LOCK_PC_HARDWARE() gpio_put(CTS_OUT_PIN, 1)  // CTS HIGH -> PC busy
#define UNLOCK_PC_HARDWARE() gpio_put(CTS_OUT_PIN, 0) // CTS LOW  -> PC free

// --- cycles counter 
#define RF_US_TO_CY(us) ((uint32_t)(((uint64_t)(us) * SYS_CLOCK_KHZ) / 1000ULL))


#if PICO_RP2040
#include "hardware/structs/systick.h"
#define RF_GET_CYCCNT() (systick_hw->cvr)
#define RF_CY_ELAPSED(start, now) (((start) - (now)) & 0x00FFFFFFu)
#define RF_CY_ADD(base, delta) (((base) - (delta)) & 0x00FFFFFFu)
#define RF_CY_REACHED(now, target) ((((int32_t)(((target) - (now)) << 8)) >> 8) >= 0)



static void rf_enable_cycle_counter(void)
{
    systick_hw->rvr = 0x00FFFFFFu;
    systick_hw->cvr = 0;
    systick_hw->csr = 0x5u;
}
#else
#define RF_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define RF_DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define RF_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define RF_GET_CYCCNT() (RF_DWT_CYCCNT)
#define RF_CY_ELAPSED(start, now) ((uint32_t)((now) - (start)))
#define RF_CY_ADD(base, delta) ((uint32_t)((base) + (delta)))
#define RF_CY_REACHED(now, target) ((int32_t)((now) - (target)) >= 0)

static void rf_enable_cycle_counter(void)
{
    RF_DEMCR |= (1u << 24);
    RF_DWT_CYCCNT = 0;
    RF_DWT_CTRL |= 1u;
}
#endif
// prototypes
static bool mfm_write_sectors_dd(uint8_t track, uint8_t side, uint8_t start_sector, uint8_t num_sectors, const uint8_t *src_pkts);
static bool mfm_decode_track_dd(uint8_t expected_track, uint8_t expected_head, uint8_t num_sectors, uint8_t *dest);
static FATFS sd_fs;
static bool sd_mounted = false;

extern void rf_i2c_queue_msg(const char *msg);
extern uint8_t g_mode_drive; // 1 = 5.25" - 2 = 3.5" 

static volatile bool i2c_has_pending_cmd = false;
static char i2c_pending_cmd[96];
static char i2c_selected_file[64] = "BIGTOP.IMG";
static char i2c_selected_proto[24] = "PC DOS 720K";





static bool rf_ensure_sd_mounted(void)
{
    if (sd_mounted) {
        return true;
    }
    gpio_init(SD_PIN_MISO);
    gpio_pull_up(SD_PIN_MISO);
    sleep_ms(20);

    for (int t = 0; t < 3; t++) {
        if (f_mount(&sd_fs, "", 1) == FR_OK) {
            sd_mounted = true;
            return true;
        }
        sleep_ms(50);
    }
    return false;
}



// --- Hardware states Realfloppy ---
static bool rf_drive_selected = false;
static bool rf_motor_running = false;
static int16_t rf_current_cyl = -1;
static uint32_t rf_motor_timeout_ms = 0;

static const char RF_VERSION[10] = {'R', 'F', 'l', '.', ' ', '1', '.', '0', '.', '1'};

#define TEMPBUFFER_SIZE 128
#define MAX_SECTORS 9
#define SECTOR_PKT_LEN 515

static char tempbuffer[TEMPBUFFER_SIZE];
static int rf_rx_len = 0;
static bool rf_rx_ready = false;

static uint8_t databuffer2[MAX_SECTORS * 516];

static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

static void rf_setup_in_pin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    gpio_set_input_enabled(pin, true);
    gpio_set_input_hysteresis_enabled(pin, true);
}

static void rf_setup_out_pin(uint pin, bool initial_high)
{
    gpio_init(pin);
    gpio_put(pin, initial_high ? 1 : 0);
    gpio_set_dir(pin, GPIO_OUT);
}

static void rf_hw_select(bool on)
{
    rf_drive_selected = on;
    gpio_put(g_active_select_pin, on ? 0 : 1);
}

static void rf_hw_motor(bool on)
{
    if (on && !rf_motor_running)
    {
        rf_motor_running = true;
        gpio_put(g_active_motor_pin, 0);
        sleep_ms(600);
    }
    else if (!on)
    {
        rf_motor_running = false;
        gpio_put(g_active_motor_pin, 1);
    }
}

static void rf_hw_step_pulse(bool inward)
{
    gpio_put(DIR_PIN, inward ? 0 : 1);
    sleep_us(5);
    gpio_put(STEP_PIN, 0);
    sleep_us(10);
    gpio_put(STEP_PIN, 1);
    sleep_us(g_active_step_us);
}

static bool rf_hw_seek(uint8_t target_cyl)
{
    if (target_cyl == 0 || rf_current_cyl < 0)
    {
        for (int i = 0; i < 90; i++)
        {
            if (gpio_get(TRK0_PIN) == 0)
            {
                rf_current_cyl = 0;
                break;
            }
            rf_hw_step_pulse(false);
        }
        if (gpio_get(TRK0_PIN) != 0)
            return false;
        rf_current_cyl = 0;
    }
    while (rf_current_cyl < target_cyl)
    {
        rf_hw_step_pulse(true);
        rf_current_cyl++;
    }
    while (rf_current_cyl > target_cyl)
    {
        rf_hw_step_pulse(false);
        rf_current_cyl--;
    }
    sleep_ms(15);
    return true;
}

static void rf_hw_set_side(uint8_t side)
{
    gpio_put(SIDE_PIN, side == 0 ? 1 : 0);
}

static bool rf_hw_is_write_protected(void)
{
    return gpio_get(PROT_PIN) == 0;
}

static uint32_t __not_in_flash_func(rf_capture_flux_cycles)(uint32_t duration_us, uint32_t max_samples)
{
    if (max_samples > FLUX_BUF_LEN)
        max_samples = FLUX_BUF_LEN;

    const uint32_t read_mask_bit = (1u << READ_PIN);
    uint32_t count = 0;
    uint32_t irq_state = save_and_disable_interrupts();

    uint32_t start_us = time_us_32();
    uint32_t last_edge_cy = 0;
    bool has_last_edge = false;
    uint32_t prev_pins = sio_hw->gpio_in;

    while (count < max_samples)
    {
        uint32_t pins = sio_hw->gpio_in;
        uint32_t now_cy = RF_GET_CYCCNT();

        if ((time_us_32() - start_us) >= duration_us)
            break;

        uint32_t falling = prev_pins & ~pins;
        prev_pins = pins;

        if (falling & read_mask_bit)
        {
            if (has_last_edge)
            {
                uint32_t delta = RF_CY_ELAPSED(last_edge_cy, now_cy);
                if (delta > 65000)
                    delta = 65000;
                flux_buf[count++] = (uint16_t)delta;
            }
            last_edge_cy = now_cy;
            has_last_edge = true;
        }
    }

    restore_interrupts(irq_state);
    return count;
}

static void __not_in_flash_func(rf_play_flux_cycles)(const uint16_t *cy_intervals, uint32_t count, bool cue_at_index, bool terminate_at_index)
{
    if (count == 0)
        return;

    const uint32_t idx_mask_bit = (1u << INDEX_PIN);
    const uint32_t wrdata_bit = (1u << WRDATA_PIN);
    const uint32_t wrgate_bit = (1u << WRGATE_PIN);
    const uint32_t pulse_low_cy = (600u * SYS_CLOCK_KHZ) / 1000000u; // 600 ns

    uint32_t irq_state = save_and_disable_interrupts();

    if (cue_at_index)
    {
        uint32_t t_guard_us = time_us_32();
        while ((sio_hw->gpio_in & idx_mask_bit) == 0)
        {
            if ((time_us_32() - t_guard_us) > 500000u)
                break;
        }
        t_guard_us = time_us_32();
        while ((sio_hw->gpio_in & idx_mask_bit) != 0)
        {
            if ((time_us_32() - t_guard_us) > 1000000u)
                break;
        }
    }

    sio_hw->gpio_clr = wrgate_bit;

    uint32_t write_start_us = time_us_32();
    uint32_t target_cy = RF_GET_CYCCNT();
    uint32_t prev_pins = sio_hw->gpio_in;

    for (uint32_t i = 0; i < count; i++)
    {
        target_cy = RF_CY_ADD(target_cy, cy_intervals[i]);

        while (!RF_CY_REACHED(RF_GET_CYCCNT(), target_cy))
        {
            if (terminate_at_index)
            {
                uint32_t pins = sio_hw->gpio_in;
                uint32_t falling = prev_pins & ~pins;
                prev_pins = pins;

                if ((falling & idx_mask_bit) && (time_us_32() - write_start_us) >= 150000u)
                {
                    goto end_play;
                }
            }
        }

        sio_hw->gpio_clr = wrdata_bit;
        uint32_t p_end = RF_CY_ADD(RF_GET_CYCCNT(), pulse_low_cy);
        while (!RF_CY_REACHED(RF_GET_CYCCNT(), p_end))
        {
        }
        sio_hw->gpio_set = wrdata_bit;
    }
    {
        uint32_t tail_end = RF_CY_ADD(RF_GET_CYCCNT(), RF_US_TO_CY(50));
        while (!RF_CY_REACHED(RF_GET_CYCCNT(), tail_end))
        {
        }
    }
end_play:
    sio_hw->gpio_set = wrdata_bit | wrgate_bit;
    restore_interrupts(irq_state);
}

// --- CRC e UART ---
static uint16_t mfm_calc_crc(const uint8_t *buf, int n)
{
    uint16_t crc = 0xCDB4;
    while (n-- > 0)
    {
        crc = crc16_table[((crc >> 8) ^ *buf++) & 0xFF] ^ (crc << 8);
    }
    return crc;
}
static inline void rf_write_byte(uint8_t b)
{
    uart_putc_raw(UART_ID, (char)b);
}
static void rf_write_bytes(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        uart_putc_raw(UART_ID, (char)buf[i]);
    }
    uart_tx_wait_blocking(UART_ID);
}
static void rf_flush_rx(void)
{
    while (uart_is_readable(UART_ID))
    {
        (void)uart_getc(UART_ID);
    }
}

static void rf_list_sd_files(void)
{
    if (!rf_ensure_sd_mounted()) {
        const char *err = "ERROR: Unable to mount SD.\r\n";
        rf_write_bytes((const uint8_t *)err, strlen(err));
        return;
    }

    DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, "/") != FR_OK) {
        sd_mounted = false; // 
        const char *err = "ERROR: Unable to open dir.\r\n";
        rf_write_bytes((const uint8_t *)err, strlen(err));
        return;
    }

    const char *hdr = "--- FILES ON SD CARD ---\r\n";
    rf_write_bytes((const uint8_t *)hdr, strlen(hdr));

    int count = 0;
    char line[96];
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
        if (fno.fattrib & AM_HID) continue;
        if (fno.fattrib & AM_DIR) {
            snprintf(line, sizeof(line), "[DIR] %s\r\n", fno.fname);
        } else {
            snprintf(line, sizeof(line), "%-20s (%lu byte)\r\n", fno.fname, (unsigned long)fno.fsize);
        }
        rf_write_bytes((const uint8_t *)line, strlen(line));
        count++;
    }

    if (count == 0) {
        const char *empty = "(No files found)\r\n";
        rf_write_bytes((const uint8_t *)empty, strlen(empty));
    }

    const char *ftr = "-----------------------\r\n";
    rf_write_bytes((const uint8_t *)ftr, strlen(ftr));

    f_closedir(&dir);
}

static bool rf_restore_image_from_sd(const char *filename)
{
    char dbg[96];
    snprintf(dbg, sizeof(dbg), "Requested file: [%s]\r\n", filename);
    rf_write_bytes((const uint8_t *)dbg, strlen(dbg));

    if (rf_hw_is_write_protected()) {
        const char *wp = "ERROR: Write protected floppy!\r\n";
        rf_write_bytes((const uint8_t *)wp, strlen(wp));
        return false;
    }

    if (!rf_ensure_sd_mounted()) {
        const char *err = "ERROR: SD not mounted.\r\n";
        rf_write_bytes((const uint8_t *)err, strlen(err));
        return false;
    }

    FIL fil;
    FRESULT fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        if (fr == FR_DISK_ERR || fr == FR_NOT_READY) sd_mounted = false;
        snprintf(dbg, sizeof(dbg), "ERROR f_open [%s] (code %d)\r\n", filename, (int)fr);
        rf_write_bytes((const uint8_t *)dbg, strlen(dbg));
        return false;
    }

    FSIZE_t fsize = f_size(&fil);
    uint8_t max_tracks = 0;
    uint8_t max_heads = 0;

    if (fsize == 184320) {          // 180 KB
        max_tracks = 40;
        max_heads = 1;
    } else if (fsize == 368640) {   // 360 KB
        max_tracks = 40;
        max_heads = 2;
    } else if (fsize == 737280) {   // 720 KB
        max_tracks = 80;
        max_heads = 2;
    } else {
        const char *sz = "ERROR: Format not supportted (valid 180KB, 360KB, 720KB)!\r\n";
        rf_write_bytes((const uint8_t *)sz, strlen(sz));
        f_close(&fil);
        return false;
    }

    snprintf(dbg, sizeof(dbg), "Geometry: %d traks, %d sides, 9 sectors (%lu KB)\r\n",
             max_tracks, max_heads, (unsigned long)(fsize / 1024));
    rf_write_bytes((const uint8_t *)dbg, strlen(dbg));

    bool all_ok = true;
    static uint8_t sector_raw[9 * 512];

    for (uint8_t track = 0; track < max_tracks && all_ok; track++) {
        char prog;
        if (track % 10 == 0) {
            prog = '0' + (track / 10);
        } else if (track % 5 == 0) {
            prog = 'v';
        } else {
            prog = '.';
        }
        rf_write_bytes((const uint8_t *)&prog, 1);

        for (uint8_t head = 0; head < max_heads && all_ok; head++) {
            LED_TOGGLE();

            UINT br = 0;
            if (f_read(&fil, sector_raw, sizeof(sector_raw), &br) != FR_OK || br != sizeof(sector_raw)) {
                all_ok = false;
                break;
            }

            for (int s = 0; s < 9; s++) {
                uint8_t *pkt = databuffer2 + (s * SECTOR_PKT_LEN);
                pkt[0] = 0xFB;
                memcpy(pkt + 1, sector_raw + (s * 512), 512);
            }

            if (!mfm_write_sectors_dd(track, head, 1, 9, databuffer2)) {
                all_ok = false;
                break;
            }
        }
    }

    f_close(&fil);
    return all_ok;
}

static bool rf_backup_720k_to_sd(const char *filename)
{
    if (!rf_ensure_sd_mounted()) {
        return false;
    }

    f_unlink(filename);

    FIL fil;
    FRESULT fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        if (fr == FR_DISK_ERR || fr == FR_NOT_READY) sd_mounted = false;
        return false;
    }

    bool all_ok = true;
    static uint8_t sector_raw[9 * 512];

    for (uint8_t track = 0; track < 80 && all_ok; track++) {
        if (!rf_hw_seek(track)) {
            all_ok = false;
            break;
        }

        char prog;
        if (track % 10 == 0) {
            prog = '0' + (track / 10);
        } else if (track % 5 == 0) {
            prog = 'v';
        } else {
            prog = '.';
        }
        rf_write_bytes((const uint8_t *)&prog, 1);

        for (uint8_t head = 0; head < 2 && all_ok; head++) {
            rf_hw_set_side(head);
            LED_TOGGLE();

            bool trk_ok = false;
            for (int retry = 0; retry < 3 && !trk_ok; retry++) {
                trk_ok = mfm_decode_track_dd(track, head, 9, databuffer2);
            }

            if (!trk_ok) {
                all_ok = false;
                break;
            }

            for (int s = 0; s < 9; s++) {
                memcpy(sector_raw + (s * 512), databuffer2 + (s * SECTOR_PKT_LEN) + 1, 512);
            }

            UINT bw = 0;
            if (f_write(&fil, sector_raw, sizeof(sector_raw), &bw) != FR_OK || bw != sizeof(sector_raw)) {
                all_ok = false;
                break;
            }
        }
    }

    f_close(&fil);
    return all_ok;
}

static char *rf_wait_handshake_line(char *buf, int buflen)
{
    int l = 0;
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    UNLOCK_PC_HARDWARE();
    while (to_ms_since_boot(get_absolute_time()) - t0 < 2000)
    {
        if (uart_is_readable(UART_ID))
        {
            uint8_t i = (uint8_t)uart_getc(UART_ID);
            if (i == 13 || i == 10)
            {
                LOCK_PC_HARDWARE();
                break;
            }
            else if (isprint(i) && l < buflen - 1)
            {
                buf[l++] = (char)i;
            }
        }
    }
    buf[l] = 0;
    return buf;
}

static bool rf_poll_uart_line(char *buf, int buflen)
{
    UNLOCK_PC_HARDWARE();
    while (uart_is_readable(UART_ID))
    {
        LED_TOGGLE();
        uint8_t i = (uint8_t)uart_getc(UART_ID);
        if (i == 13 || i == 10)
        {
            LOCK_PC_HARDWARE();
            while (rf_rx_len > 0 && isspace((unsigned char)buf[rf_rx_len - 1]))
                rf_rx_len--;
            buf[rf_rx_len] = 0;
            bool has_cmd = (rf_rx_len > 0);
            rf_rx_len = 0;
            return has_cmd;
        }
        else if (i == 27)
        {
            rf_rx_len = 0;
        }
        else if (isprint(i) && rf_rx_len < buflen - 1)
        {
            buf[rf_rx_len++] = (char)i;
        }
    }

    if (rf_motor_timeout_ms > 0 && to_ms_since_boot(get_absolute_time()) > rf_motor_timeout_ms)
    {
        rf_hw_motor(false);
        rf_hw_select(false);
        rf_motor_timeout_ms = 0;
    }
    return false;
}



// --- Encode and decode MFM DD track ---
static bool mfm_decode_track_dd(uint8_t expected_track, uint8_t expected_head, uint8_t num_sectors, uint8_t *dest)
{
    // old values
//    const uint32_t T_SHORT_MAX = RF_US_TO_CY(5) + (RF_US_TO_CY(1) / 4); // 5.25 us
//    const uint32_t T_MED_MAX = RF_US_TO_CY(7) + (RF_US_TO_CY(1) / 4);   // 7.25 us

    // standard values
    const uint32_t T_SHORT_MAX = RF_US_TO_CY(5); // 5.00 us (metà tra 4 e 6 us)
    const uint32_t T_MED_MAX   = RF_US_TO_CY(7); // 7.00 us (metà tra 6 e 8 us)

    uint32_t n_flux = rf_capture_flux_cycles(240000, FLUX_BUF_LEN);
    if (n_flux < 1000)
        return false;
    bool sector_found[MAX_SECTORS + 1] = {false};
    uint8_t found_count = 0;
    uint64_t shift_reg = 0;
    int sync_cnt = 0;
    int bits_since_sync = 0;
    bool in_frame = false;
    uint8_t bit_acc = 0;
    uint8_t bit_cnt = 0;
    bool is_odd = true;
    uint8_t frame_buf[520];
    uint32_t frame_len = 0;
    uint32_t frame_target = 0;
    int last_id_cyl = -1;
    int last_id_hd = -1;
    int last_id_sec = -1;
    bool has_valid_id = false;
    auto push_mfm_bit = [&](uint8_t mfm_bit)
    {
        if (!in_frame)
        {
            shift_reg = (shift_reg << 1) | (mfm_bit & 1);
            if ((shift_reg & 0xFFFF) == 0x4489)
            {
                sync_cnt++;
                bits_since_sync = 0;
                shift_reg = 0;
                if (sync_cnt == 3)
                {
                    in_frame = true;
                    is_odd = true;
                    bit_acc = 0;
                    bit_cnt = 0;
                    frame_len = 0;
                    frame_target = 1;
                }
            }
            else if (sync_cnt > 0)
            {
                bits_since_sync++;
                if (bits_since_sync > 16)
                {
                    sync_cnt = 0;
                    bits_since_sync = 0;
                }
            }
            return;
        }
        if (is_odd)
        {
            is_odd = false;
        }
        else
        {
            is_odd = true;
            bit_acc = (bit_acc << 1) | (mfm_bit & 1);
            bit_cnt++;
            if (bit_cnt == 8)
            {
                bit_cnt = 0;
                if (frame_len < sizeof(frame_buf))
                {
                    frame_buf[frame_len++] = bit_acc;
                }
                if (frame_len == 1)
                {
                    if (frame_buf[0] == 0xFE)
                    {
                        frame_target = 7;
                    }
                    else if (frame_buf[0] == 0xFB)
                    {
                        frame_target = 515;
                    }
                    else
                    {
                        in_frame = false;
                        sync_cnt = 0;
                        bits_since_sync = 0;
                    }
                }
                else if (frame_len >= frame_target)
                {
                    if (frame_buf[0] == 0xFE)
                    {
                        uint16_t crc = mfm_calc_crc(frame_buf, 5);
                        uint16_t rx_crc = ((uint16_t)frame_buf[5] << 8) | frame_buf[6];
                        if (crc == rx_crc)
                        {
                            last_id_cyl = frame_buf[1];
                            last_id_hd = frame_buf[2];
                            last_id_sec = frame_buf[3];
                            has_valid_id = true;
                        }
                        else
                        {
                            has_valid_id = false;
                        }
                    }
                    else if (frame_buf[0] == 0xFB && has_valid_id)
                    {
                        uint16_t crc = mfm_calc_crc(frame_buf, 513);
                        uint16_t rx_crc = ((uint16_t)frame_buf[513] << 8) | frame_buf[514];
                        if (crc == rx_crc &&
                            last_id_cyl == expected_track &&
                            last_id_hd == expected_head &&
                            last_id_sec >= 1 && last_id_sec <= num_sectors)
                        {
                            if (!sector_found[last_id_sec])
                            {
                                sector_found[last_id_sec] = true;
                                found_count++;
                                memcpy(dest + (last_id_sec - 1) * SECTOR_PKT_LEN, frame_buf, SECTOR_PKT_LEN);
                            }
                        }
                        has_valid_id = false;
                    }
                    in_frame = false;
                    sync_cnt = 0;
                    bits_since_sync = 0;
                }
            }
        }
    };
    for (uint32_t i = 0; i < n_flux && found_count < num_sectors; i++)
    {
        uint32_t dt = flux_buf[i];
        int zeros = (dt < T_SHORT_MAX) ? 1 : (dt < T_MED_MAX ? 2 : 3);
        for (int z = 0; z < zeros; z++)
            push_mfm_bit(0);
        push_mfm_bit(1);
    }
    return (found_count == num_sectors);
}
struct MfmStreamBuilder
{
    uint32_t count;
    uint8_t prev_data_bit;
    uint32_t pending_cells;
    uint32_t prev_cells;
    uint32_t precomp_cy;

    void init(uint8_t track)
    {
        count = 0;
        prev_data_bit = 0;
        pending_cells = 0;
        prev_cells = 0;
        precomp_cy = 0;
        // Write precomp Drive A (5.25") -> track 20 | Drive B (3.5") -> track 40
        // 150 nanosecond for 3.5" - 250 nanosecond for 5.25"
        if (g_mode_drive == 1){
            if(track >=20)precomp_cy = ((250u * SYS_CLOCK_KHZ) / 1000000u);
        }                              
        else{
            if(track >=40)precomp_cy = ((150u * SYS_CLOCK_KHZ) / 1000000u);
        }  
//        uint8_t precomp_start = (g_active_select_pin == SELECT_A_PIN) ? 20 : 40;
//        precomp_cy = (track >= precomp_start) ? ((150u * SYS_CLOCK_KHZ) / 1000000u) : 0;
    }

    void emit_interval(uint32_t cur_cells)
    {
        if (prev_cells > 0 && count < FLUX_BUF_LEN)
        {
            int32_t cy = (int32_t)(prev_cells * RF_US_TO_CY(2));

            // Se l'intervallo precedente era corto (2 celle = 4us) e il successivo è lungo (>2 celle),
            // l'impulso centrale va RITARDATO: allunghiamo l'intervallo precedente.
            if (prev_cells == 2 && cur_cells > 2)
            {
                cy += (int32_t)precomp_cy;
            }
            // Se l'intervallo precedente era lungo (>2 celle) e il successivo è corto (2 celle = 4us),
            // l'impulso centrale va ANTICIPATO: accorciamo l'intervallo precedente.
            else if (prev_cells > 2 && cur_cells == 2)
            {
                cy -= (int32_t)precomp_cy;
            }

            if (cy > 65000) cy = 65000;
            flux_buf[count++] = (uint16_t)cy;
        }
        prev_cells = cur_cells;
    }

    void push_mfm_bit(uint8_t b)
    {
        if (b == 0)
        {
            pending_cells++;
        }
        else
        {
            if (pending_cells > 0)
            {
                emit_interval(pending_cells);
            }
            pending_cells = 1;
        }
    }

    void finish()
    {
        // Scarica l'ultimo intervallo rimasto in sospeso
        if (prev_cells > 0 && count < FLUX_BUF_LEN)
        {
            uint32_t cy = prev_cells * RF_US_TO_CY(2);
            flux_buf[count++] = (cy > 65000) ? 65000 : (uint16_t)cy;
            prev_cells = 0;
        }
    }

    void push_byte(uint8_t data)
    {
        for (int bit = 7; bit >= 0; bit--)
        {
            uint8_t d = (data >> bit) & 1;
            uint8_t c = (prev_data_bit == 0 && d == 0) ? 1 : 0;
            push_mfm_bit(c);
            push_mfm_bit(d);
            prev_data_bit = d;
        }
    }

    void push_sync_a1()
    {
        uint16_t raw = 0x4489;
        for (int bit = 15; bit >= 0; bit--)
            push_mfm_bit((raw >> bit) & 1);
        prev_data_bit = 1;
    }

    void push_sync_c2()
    {
        uint16_t raw = 0x5224;
        for (int bit = 15; bit >= 0; bit--)
            push_mfm_bit((raw >> bit) & 1);
        prev_data_bit = 0;
    }
};

static bool mfm_format_track_dd(uint8_t track, uint8_t side, uint8_t interleave)
{
    if (rf_hw_is_write_protected())
        return false;
    if (!rf_drive_selected)
        rf_hw_select(true);
    if (!rf_motor_running)
        rf_hw_motor(true);
    if (!rf_hw_seek(track))
        return false;
    rf_hw_set_side(side);
    const uint8_t numsec = 9;
    uint8_t sec_order[9] = {0};
    uint8_t idx = 0, n = 0;
    while (n < numsec)
    {
        if (sec_order[idx] != 0)
            idx++;
        else
        {
            sec_order[idx] = n + 1;
            idx += interleave;
            n++;
        }
        if (idx >= numsec)
            idx -= numsec;
    }
    MfmStreamBuilder mb;
    mb.init(track);
    for (int i = 0; i < 56; i++)
        mb.push_byte(0x4E);
    for (int i = 0; i < 12; i++)
        mb.push_byte(0x00);
    for (int i = 0; i < 3; i++)
        mb.push_sync_c2();
    mb.push_byte(0xFC);
    for (int i = 0; i < 50; i++)
        mb.push_byte(0x4E);
    for (uint8_t s = 0; s < numsec; s++)
    {
        uint8_t sec_num = sec_order[s];
        for (int i = 0; i < 12; i++)
            mb.push_byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.push_sync_a1();
        uint8_t id_rec[5] = {0xFE, track, side, sec_num, 2};
        uint16_t id_crc = mfm_calc_crc(id_rec, 5);
        for (int i = 0; i < 5; i++)
            mb.push_byte(id_rec[i]);
        mb.push_byte(id_crc >> 8);
        mb.push_byte(id_crc & 0xFF);
        for (int i = 0; i < 22; i++)
            mb.push_byte(0x4E);
        for (int i = 0; i < 12; i++)
            mb.push_byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.push_sync_a1();
        uint8_t dam_buf[513];
        dam_buf[0] = 0xFB;
        memset(dam_buf + 1, 0xF6, 512);
        uint16_t data_crc = mfm_calc_crc(dam_buf, 513);
        for (int i = 0; i < 513; i++)
            mb.push_byte(dam_buf[i]);
        mb.push_byte(data_crc >> 8);
        mb.push_byte(data_crc & 0xFF);
        if (s < numsec - 1)
        {
            for (int i = 0; i < 80; i++)
                mb.push_byte(0x4E);
        }
    }
    for (int i = 0; i < 250; i++)
        mb.push_byte(0x4E);
    mb.finish();
    rf_play_flux_cycles(flux_buf, mb.count, true, true);
    return true;
}
static bool mfm_write_sectors_dd(uint8_t track, uint8_t side, uint8_t start_sector, uint8_t num_sectors, const uint8_t *src_pkts)
{
    if (rf_hw_is_write_protected())
        return false;
    if (!rf_drive_selected)
        rf_hw_select(true);
    if (!rf_motor_running)
        rf_hw_motor(true);
    if (!rf_hw_seek(track))
        return false;
    rf_hw_set_side(side);
    static uint8_t full_track_buf[9 * SECTOR_PKT_LEN];
    if (num_sectors < 9)
    {
        bool ok = false;
        for (int retry = 0; retry < 3 && !ok; retry++)
        {
            ok = mfm_decode_track_dd(track, side, 9, full_track_buf);
        }
        if (!ok)
            return false;
        memcpy(full_track_buf + (start_sector - 1) * SECTOR_PKT_LEN, src_pkts, SECTOR_PKT_LEN);
    }
    else
    {
        memcpy(full_track_buf, src_pkts, 9 * SECTOR_PKT_LEN);
    }
    MfmStreamBuilder mb;
    mb.init(track);
    for (int i = 0; i < 56; i++)
        mb.push_byte(0x4E);
    for (int i = 0; i < 12; i++)
        mb.push_byte(0x00);
    for (int i = 0; i < 3; i++)
        mb.push_sync_c2();
    mb.push_byte(0xFC);
    for (int i = 0; i < 50; i++)
        mb.push_byte(0x4E);
    for (uint8_t s = 1; s <= 9; s++)
    {
        uint8_t *pkt = full_track_buf + (s - 1) * SECTOR_PKT_LEN;
        pkt[0] = 0xFB;
        uint16_t data_crc = mfm_calc_crc(pkt, 513);
        for (int i = 0; i < 12; i++)
            mb.push_byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.push_sync_a1();
        uint8_t id_rec[5] = {0xFE, track, side, s, 2};
        uint16_t id_crc = mfm_calc_crc(id_rec, 5);
        for (int i = 0; i < 5; i++)
            mb.push_byte(id_rec[i]);
        mb.push_byte(id_crc >> 8);
        mb.push_byte(id_crc & 0xFF);
        for (int i = 0; i < 22; i++)
            mb.push_byte(0x4E);
        for (int i = 0; i < 12; i++)
            mb.push_byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.push_sync_a1();
        for (int i = 0; i < 513; i++)
            mb.push_byte(pkt[i]);
        mb.push_byte(data_crc >> 8);
        mb.push_byte(data_crc & 0xFF);
        if (s < 9)
        {
            for (int i = 0; i < 80; i++)
                mb.push_byte(0x4E);
        }
    }
    for (int i = 0; i < 200; i++)
        mb.push_byte(0x4E);
    mb.finish();
    rf_play_flux_cycles(flux_buf, mb.count, true, true);
    return true;
}

// --- Protocol Polling Init ---
void rf_protocol_init(void)
{
    rf_enable_cycle_counter();
    rf_setup_out_pin(PICO_DEFAULT_LED_PIN, false);
    rf_setup_out_pin(DENSITY_PIN, false); // DD = LOW
    rf_setup_out_pin(SELECT_A_PIN, true);
    rf_setup_out_pin(MOTOR_A_PIN, true);
    rf_setup_out_pin(SELECT_B_PIN, true);
    rf_setup_out_pin(MOTOR_B_PIN, true);
    rf_setup_out_pin(DIR_PIN, true);
    rf_setup_out_pin(STEP_PIN, true);
    rf_setup_out_pin(WRDATA_PIN, true);
    rf_setup_out_pin(WRGATE_PIN, true);
    rf_setup_out_pin(SIDE_PIN, true);
    rf_setup_in_pin(INDEX_PIN);
    rf_setup_in_pin(TRK0_PIN);
    rf_setup_in_pin(PROT_PIN);
    rf_setup_in_pin(READ_PIN);
    rf_setup_in_pin(READY_PIN);
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(RX_PIN);
    uart_set_hw_flow(UART_ID, false, false);
    gpio_init(CTS_OUT_PIN);
    gpio_set_dir(CTS_OUT_PIN, GPIO_OUT);
    UNLOCK_PC_HARDWARE();
    rf_flush_rx();
    rf_rx_len = 0;
    for (int i = 0; i < 2; i++)
    {
        LED_ON();
        sleep_ms(150);
        LED_OFF();
        sleep_ms(150);
    }
}
void rf_protocol_poll(void)
{
    if (!rf_poll_uart_line(tempbuffer, TEMPBUFFER_SIZE))
        return;

    char cmd = 0;
    int a1 = 0, a2 = 0, a3 = 0;
    int n = sscanf(tempbuffer, "%c%i,%i,%i", &cmd, &a1, &a2, &a3);
    if (n <= 0 || isspace((unsigned char)cmd))
        return;
    if (cmd >96 && cmd <= 122 ) cmd-=32;
    LED_ON();
    if (cmd != 'V' && cmd != 'D' && cmd != 'H' && cmd != '?')
    {
        if (!rf_drive_selected)
            rf_hw_select(true);
        if (!rf_motor_running)
            rf_hw_motor(true);
        rf_motor_timeout_ms = 0;
    }

    if (cmd == 'V')
    {
        rf_flush_rx();
        rf_write_bytes((const uint8_t *)RF_VERSION, 10);
    }
    else if (cmd == 'R' && n >= 3)
    {
        int track = a1, sectors = a2, head = (n == 3) ? 0 : a3;
        if (head >= 0 && head < 2 && track >= 0 && track < 80 && sectors >= 1 && sectors <= 9)
        {
            if (rf_hw_seek((uint8_t)track))
            {
                rf_hw_set_side((uint8_t)head);
                bool ok = false;
                for (int retry = 0; retry < 3 && !ok; retry++)
                {
                    ok = mfm_decode_track_dd((uint8_t)track, (uint8_t)head, (uint8_t)sectors, databuffer2);
                }
                if (ok)
                {
                    rf_write_byte('1');
                    char go = 0;
                    sscanf(rf_wait_handshake_line(tempbuffer + 1, 8), "%c", &go);
                    rf_write_bytes(databuffer2, (size_t)sectors * SECTOR_PKT_LEN);
                }
                else
                {
                    rf_write_byte('0');
                }
            }
            else
            {
                rf_write_byte('0');
            }
        }
    }
    else if (cmd == 'W' && n >= 3)
    {
        int track = a1, sectors = a2, head = (n == 3) ? 0 : a3;
        size_t totalBytesExpected = (size_t)sectors * SECTOR_PKT_LEN;
        UNLOCK_PC_HARDWARE();
        sleep_ms(5);
        rf_write_byte('1');
        size_t received = 0;
        uint32_t rx_timeout = to_ms_since_boot(get_absolute_time());
        bool timeout = false;
        while (received < totalBytesExpected)
        {
            if (uart_is_readable(UART_ID))
            {
                databuffer2[received++] = (uint8_t)uart_getc(UART_ID);
                rx_timeout = to_ms_since_boot(get_absolute_time());
            }
            else if (to_ms_since_boot(get_absolute_time()) - rx_timeout > 4000)
            {
                timeout = true;
                break;
            }
        }
        LOCK_PC_HARDWARE();
        if (timeout || received < totalBytesExpected)
        {
            rf_write_byte('0');
        }
        else
        {
            bool ok = mfm_write_sectors_dd((uint8_t)track, (uint8_t)head, 1, (uint8_t)sectors, databuffer2);
            rf_flush_rx();
            rf_write_byte(ok ? 'W' : '0');
        }
        rf_flush_rx();
    }

     else if (cmd == 'D')
    {
        rf_list_sd_files();
        rf_flush_rx();
    }
    else if (cmd == 'B')
    {
        char fname[32] = "BACKUP.IMG";
        char *comma = strchr(tempbuffer, ',');
        if (comma != NULL) {
            char *p = comma + 1;
            while (*p == ' ') p++;
            if (*p != '\0') {
                strncpy(fname, p, sizeof(fname) - 1);
                fname[sizeof(fname) - 1] = '\0';
            }
        }

        const char *start_msg = "Start backup 720KB on SD...\r\n";
        rf_write_bytes((const uint8_t *)start_msg, strlen(start_msg));

        bool ok = rf_backup_720k_to_sd(fname);

        const char *end_msg = ok ? "\r\nOK! Backup done.\r\n" : "\r\nERROR during backup!\r\n";
        rf_write_bytes((const uint8_t *)end_msg, strlen(end_msg));
        rf_flush_rx();
    }
    else if (cmd == 'L')
    {
        char *comma = strchr(tempbuffer, ',');
        char *p = (comma != NULL) ? (comma + 1) : NULL;
        while (p != NULL && *p == ' ') p++;

        if (p == NULL || *p == '\0') {
            const char *usage = "Usage: L,name.img\r\n";
            rf_write_bytes((const uint8_t *)usage, strlen(usage));
        } else {
            char fname[32];
            strncpy(fname, p, sizeof(fname) - 1);
            fname[sizeof(fname) - 1] = '\0';

            const char *start_msg = "Wrinting image on floppy...\r\n";
            rf_write_bytes((const uint8_t *)start_msg, strlen(start_msg));

            bool ok = rf_restore_image_from_sd(fname);

            const char *end_msg = ok ? "\r\nOK! Floppy written.\r\n" : "\r\nERROR during write!\r\n";
            rf_write_bytes((const uint8_t *)end_msg, strlen(end_msg));
        }
        rf_flush_rx();
    }
    else if (cmd == 'H' || cmd == '?')
    {
        const char *help_msg =
            "\r\n=== REALFLOPPY Commands ===\r\n"
            "V       : Firmware version\r\n"
            "D       : File list on SD\r\n"
            "B       : Backup floppy 720KB -> BACKUP720.IMG\r\n"
            "B,name  : Backup floppy 720KB -> name\r\n"
            "L,name  : Write  .IMG (180K/360K/720K) from SD to floppy\r\n"
            "H       : Mostra questo elenco\r\n"
            "==============================\r\n";
        rf_write_bytes((const uint8_t *)help_msg, strlen(help_msg));
        rf_flush_rx();
    }

    else if (cmd == 'F' && n >= 2)
    {
        int track = a1, head = (n >= 3) ? a2 : 0;
        bool ok = mfm_format_track_dd((uint8_t)track, (uint8_t)head, 1);
        rf_flush_rx();
        rf_write_byte(ok ? 'W' : '0');
        rf_flush_rx();
    }
    LED_OFF();
    rf_motor_timeout_ms = to_ms_since_boot(get_absolute_time()) + 5000;
}