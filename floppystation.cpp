#include "floppystation.h"
#include "bridge.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "ff.h"

volatile uint8_t g_active_select_pin = SELECT_B_PIN; // Default: Drive B (3.5")
volatile uint8_t g_active_motor_pin  = MOTOR_B_PIN;
volatile uint32_t g_active_step_us   = 3000;         // 3 ms for 3.5" (8 ms for 5.25")
volatile bool g_rf_mode_active       = true;         // Default: RealFloppy UART1
volatile bool g_mode_changed         = false;

// change according to your disks configuration
uint8_t g_mode_drive        = 2;           // 1 = 5.25" - 2 = 3.5" 



#define FS_LED_ON() gpio_put(PICO_DEFAULT_LED_PIN, 1)
#define FS_LED_OFF() gpio_put(PICO_DEFAULT_LED_PIN, 0)
#define FS_LED_TOGGLE() gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN))

#define FS_US_TO_CY(us) ((uint32_t)(((uint64_t)(us) * SYS_CLOCK_KHZ) / 1000ULL))

#if PICO_RP2040
#include "hardware/structs/systick.h"
#define FS_GET_CYCCNT() (systick_hw->cvr)
#define FS_CY_ELAPSED(start, now) (((start) - (now)) & 0x00FFFFFFu)
#define FS_CY_ADD(base, delta) (((base) - (delta)) & 0x00FFFFFFu)
#define FS_CY_REACHED(now, target) ((((int32_t)(((target) - (now)) << 8)) >> 8) >= 0)
#else
#define FS_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define FS_DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define FS_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define FS_GET_CYCCNT() (FS_DWT_CYCCNT)
#define FS_CY_ELAPSED(start, now) ((uint32_t)((now) - (start)))
#define FS_CY_ADD(base, delta) ((uint32_t)((base) + (delta)))
#define FS_CY_REACHED(now, target) ((int32_t)((now) - (target)) >= 0)
#endif

// --- IRQ <-> Loop ---
volatile bool g_fs_mode_requested = false;
static volatile bool fs_exit_requested = false;
static volatile bool fs_cmd_pending = false;
static char fs_cmd_buf[96];

// Circlular TX queue with Arduino Mega (buffer Wire Arduino = 32 byte)
static char fs_tx_ring[2048];
static volatile uint16_t fs_tx_head = 0;
static volatile uint16_t fs_tx_tail = 0;
static char fs_selected_file[64] = "(None)";
static char fs_selected_proto[24] = "DOS 720K";

void fs_i2c_queue_msg(const char *msg)
{
    uint32_t irq = save_and_disable_interrupts();
    for (size_t i = 0; msg[i] != '\0'; i++)
    {
        uint16_t next = (fs_tx_head + 1) % sizeof(fs_tx_ring);
        if (next != fs_tx_tail)
        {
            fs_tx_ring[fs_tx_head] = msg[i];
            fs_tx_head = next;
        }
    }
    restore_interrupts(irq);
}

uint8_t fs_i2c_pop_tx_byte(void)
{
    if (fs_tx_tail == fs_tx_head)
    {
        return 0x00;
    }
    uint8_t b = (uint8_t)fs_tx_ring[fs_tx_tail];
    fs_tx_tail = (fs_tx_tail + 1) % sizeof(fs_tx_ring);
    return b;
}

void fs_i2c_on_rx_line(const char *line)
{
    if (strcmp(line, "CMD:ENTER_FS") == 0)
    {
        g_fs_mode_requested = true;
        return;
    }
    if (strcmp(line, "CMD:EXIT_FS") == 0)
    {
        fs_exit_requested = true;
        return;
    }
    if (strcmp(line, "CMD:SET_DRIVE:A") == 0)
    {
        gpio_put(SELECT_A_PIN, 1); gpio_put(MOTOR_A_PIN, 1);
        gpio_put(SELECT_B_PIN, 1); gpio_put(MOTOR_B_PIN, 1);
        g_active_select_pin = SELECT_A_PIN;
        g_active_motor_pin  = MOTOR_A_PIN;
        g_active_step_us    = 8000; // 8 ms for  5.25" 360K
        g_mode_changed      = true; // Force recalibrate
        g_mode_drive        = 2;    // 1 = 5.25" - 2 = 3.5" 
        return;
    }
    if (strcmp(line, "CMD:SET_DRIVE:B") == 0)
    {
        gpio_put(SELECT_A_PIN, 1); gpio_put(MOTOR_A_PIN, 1);
        gpio_put(SELECT_B_PIN, 1); gpio_put(MOTOR_B_PIN, 1);
        g_active_select_pin = SELECT_B_PIN;
        g_active_motor_pin  = MOTOR_B_PIN;
        g_active_step_us    = 3000; // 3 ms for 3.5"
        g_mode_changed      = true; // Force recalibrate
        g_mode_drive        = 1;    // 1 = 5.25" - 2 = 3.5" 
        return;
    }
    if (strcmp(line, "CMD:SET_MODE:RF") == 0)
    {
        if (!g_rf_mode_active) {
            g_rf_mode_active = true;
            g_mode_changed   = true;
        }
        return;
    }
    if (strcmp(line, "CMD:SET_MODE:GW") == 0)
    {
        if (g_rf_mode_active) {
            g_rf_mode_active = false;
            g_mode_changed   = true;
        }
        return;
    }

    if (strncmp(line, "CMD:SET_PROTO:", 14) == 0)
    {
        strncpy(fs_selected_proto, line + 14, sizeof(fs_selected_proto) - 1);
        fs_selected_proto[sizeof(fs_selected_proto) - 1] = '\0';
        char resp[48];
        snprintf(resp, sizeof(resp), "PROTO:%s\n", fs_selected_proto);
        fs_i2c_queue_msg(resp);
        return;
    }    

    if (!fs_cmd_pending)
    {
        strncpy(fs_cmd_buf, line, sizeof(fs_cmd_buf) - 1);
        fs_cmd_buf[sizeof(fs_cmd_buf) - 1] = '\0';
        fs_cmd_pending = true;
    }
}

// --- FloppyStation status ---
static FATFS fs_sd;
static bool fs_sd_mounted = false;
static bool fs_drive_sel = false;
static bool fs_motor_on = false;
static int16_t fs_cur_cyl = -1;
static uint32_t fs_motor_off_ms = 0;



#define FS_MAX_SECTORS 9
#define FS_PKT_LEN 515
static uint8_t fs_track_buf[FS_MAX_SECTORS * 516];

static const uint16_t fs_crc16_table[256] = {
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

static uint16_t fs_crc16(const uint8_t *buf, int n)
{
    uint16_t crc = 0xCDB4;
    while (n-- > 0)
    {
        crc = fs_crc16_table[((crc >> 8) ^ *buf++) & 0xFF] ^ (crc << 8);
    }
    return crc;
}

static bool fs_ensure_sd(void)
{
    if (fs_sd_mounted) return true;

    gpio_init(SD_PIN_MISO);
    gpio_pull_up(SD_PIN_MISO);
    gpio_init(SD_PIN_CS);
    gpio_set_dir(SD_PIN_CS, GPIO_OUT);
    gpio_put(SD_PIN_CS, 1);
    sleep_ms(50);

    for (int t = 0; t < 5; t++) {
        f_mount(NULL, "", 0); // Reset 
        sleep_ms(30);
        if (f_mount(&fs_sd, "", 1) == FR_OK) {
            fs_sd_mounted = true;
            return true;
        }
        sleep_ms(100);
    }
    return false;
}

static void fs_hw_init(void)
{
    auto cfg_out = [](uint pin, bool hi)
    {
        gpio_init(pin);
        gpio_put(pin, hi ? 1 : 0);
        gpio_set_dir(pin, GPIO_OUT);
    };
    auto cfg_in = [](uint pin)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
        gpio_set_input_enabled(pin, true);
        gpio_set_input_hysteresis_enabled(pin, true);
    };

    cfg_out(PICO_DEFAULT_LED_PIN, false);
    cfg_out(DENSITY_PIN, false); // DD
    cfg_out(SELECT_A_PIN, true);
    cfg_out(MOTOR_A_PIN, true);
    cfg_out(SELECT_B_PIN, true);
    cfg_out(MOTOR_B_PIN, true);
    cfg_out(DIR_PIN, true);
    cfg_out(STEP_PIN, true);
    cfg_out(WRDATA_PIN, true);
    cfg_out(WRGATE_PIN, true);
    cfg_out(SIDE_PIN, true);

    cfg_in(INDEX_PIN);
    cfg_in(TRK0_PIN);
    cfg_in(PROT_PIN);
    cfg_in(READ_PIN);
    cfg_in(READY_PIN);

    fs_drive_sel = false;
    fs_motor_on = false;
    fs_cur_cyl = -1;
}

static void fs_select(bool on)
{
    fs_drive_sel = on;
    gpio_put(g_active_select_pin, on ? 0 : 1);
}

static void fs_motor(bool on)
{
    if (on && !fs_motor_on)
    {
        fs_motor_on = true;
        gpio_put(g_active_motor_pin, 0);
        sleep_ms(600);
    }
    else if (!on)
    {
        fs_motor_on = false;
        gpio_put(g_active_motor_pin, 1);
    }
}

static void fs_step(bool inward)
{
    gpio_put(DIR_PIN, inward ? 0 : 1);
    sleep_us(5);
    gpio_put(STEP_PIN, 0);
    sleep_us(10);
    gpio_put(STEP_PIN, 1);
    sleep_us(g_active_step_us);
}

static bool fs_seek(uint8_t cyl)
{
    if (cyl == 0 || fs_cur_cyl < 0)
    {
        for (int i = 0; i < 90; i++)
        {
            if (gpio_get(TRK0_PIN) == 0)
            {
                fs_cur_cyl = 0;
                break;
            }
            fs_step(false);
        }
        if (gpio_get(TRK0_PIN) != 0)
            return false;
        fs_cur_cyl = 0;
    }
    while (fs_cur_cyl < cyl)
    {
        fs_step(true);
        fs_cur_cyl++;
    }
    while (fs_cur_cyl > cyl)
    {
        fs_step(false);
        fs_cur_cyl--;
    }
    sleep_ms(20);  // was 15
    return true;
}

static void fs_set_side(uint8_t side)
{
    gpio_put(SIDE_PIN, side == 0 ? 1 : 0);
}

static uint32_t __not_in_flash_func(fs_capture_flux)(uint32_t duration_us, uint32_t max_samples)
{
    if (max_samples > FLUX_BUF_LEN)
        max_samples = FLUX_BUF_LEN;
    const uint32_t read_bit = (1u << READ_PIN);
    uint32_t count = 0;
    uint32_t irq = save_and_disable_interrupts();

    uint32_t start_us = time_us_32();
    uint32_t last_cy = 0;
    bool has_edge = false;
    uint32_t prev = sio_hw->gpio_in;

    while (count < max_samples)
    {
        uint32_t pins = sio_hw->gpio_in;
        uint32_t now_cy = FS_GET_CYCCNT();
        if ((time_us_32() - start_us) >= duration_us)
            break;
        uint32_t falling = prev & ~pins;
        prev = pins;
        if (falling & read_bit)
        {
            if (has_edge)
            {
                uint32_t dt = FS_CY_ELAPSED(last_cy, now_cy);
                if (dt > 65000)
                    dt = 65000;
                flux_buf[count++] = (uint16_t)dt;
            }
            last_cy = now_cy;
            has_edge = true;
        }
    }
    restore_interrupts(irq);
    return count;
}

static void __not_in_flash_func(fs_play_flux)(const uint16_t *cy_intervals, uint32_t count)
{
    if (count == 0)
        return;
    const uint32_t idx_bit = (1u << INDEX_PIN);
    const uint32_t wrdata_bit = (1u << WRDATA_PIN);
    const uint32_t wrgate_bit = (1u << WRGATE_PIN);
    const uint32_t pulse_cy = (600u * SYS_CLOCK_KHZ) / 1000000u;

    uint32_t irq = save_and_disable_interrupts();
    uint32_t t0 = time_us_32();
    while ((sio_hw->gpio_in & idx_bit) == 0 && (time_us_32() - t0) < 500000u)
    {
    }
    t0 = time_us_32();
    while ((sio_hw->gpio_in & idx_bit) != 0 && (time_us_32() - t0) < 1000000u)
    {
    }

    sio_hw->gpio_clr = wrgate_bit;
    uint32_t w_start = time_us_32();
    uint32_t target_cy = FS_GET_CYCCNT();
    uint32_t prev = sio_hw->gpio_in;

    for (uint32_t i = 0; i < count; i++)
    {
        target_cy = FS_CY_ADD(target_cy, cy_intervals[i]);
        while (!FS_CY_REACHED(FS_GET_CYCCNT(), target_cy))
        {
            uint32_t pins = sio_hw->gpio_in;
            uint32_t falling = prev & ~pins;
            prev = pins;
            if ((falling & idx_bit) && (time_us_32() - w_start) >= 150000u)
            {
                goto end_play;
            }
        }
        sio_hw->gpio_clr = wrdata_bit;
        uint32_t p_end = FS_CY_ADD(FS_GET_CYCCNT(), pulse_cy);
        while (!FS_CY_REACHED(FS_GET_CYCCNT(), p_end))
        {
        }
        sio_hw->gpio_set = wrdata_bit;
    }
    {
        uint32_t tail = FS_CY_ADD(FS_GET_CYCCNT(), FS_US_TO_CY(50));
        while (!FS_CY_REACHED(FS_GET_CYCCNT(), tail))
        {
        }
    }
end_play:
    sio_hw->gpio_set = wrdata_bit | wrgate_bit;
    restore_interrupts(irq);
}
static bool fs_decode_track(uint8_t exp_trk, uint8_t exp_hd, uint8_t num_sec, uint8_t *dest)
{
    //const uint32_t T_SHORT = FS_US_TO_CY(5) + (FS_US_TO_CY(1) / 4);
    //const uint32_t T_MED = FS_US_TO_CY(7) + (FS_US_TO_CY(1) / 4);
    const uint32_t T_SHORT = FS_US_TO_CY(5); // 5.00 us
    const uint32_t T_MED   = FS_US_TO_CY(7); // 7.00 us    
    uint32_t n_flux = fs_capture_flux(240000, FLUX_BUF_LEN);
    if (n_flux < 1000)
        return false;
    bool found[FS_MAX_SECTORS + 1] = {false};
    uint8_t found_cnt = 0;
    uint64_t sr = 0;
    int sync_cnt = 0, bits_sync = 0;
    bool in_frame = false, is_odd = true, valid_id = false;
    uint8_t bit_acc = 0, bit_cnt = 0;
    uint8_t fbuf[520];
    uint32_t flen = 0, ftarget = 0;
    int id_c = -1, id_h = -1, id_s = -1;
    auto push_bit = [&](uint8_t b)
    {
        if (!in_frame)
        {
            sr = (sr << 1) | (b & 1);
            if ((sr & 0xFFFF) == 0x4489)
            {
                sync_cnt++;
                bits_sync = 0;
                sr = 0;
                if (sync_cnt == 3)
                {
                    in_frame = true;
                    is_odd = true;
                    bit_acc = 0;
                    bit_cnt = 0;
                    flen = 0;
                    ftarget = 1;
                }
            }
            else if (sync_cnt > 0 && ++bits_sync > 16)
            {
                sync_cnt = 0;
                bits_sync = 0;
            }
            return;
        }
        if (is_odd)
        {
            is_odd = false;
            return;
        }
        is_odd = true;
        bit_acc = (bit_acc << 1) | (b & 1);
        if (++bit_cnt == 8)
        {
            bit_cnt = 0;
            if (flen < sizeof(fbuf))
                fbuf[flen++] = bit_acc;
            if (flen == 1)
            {
                if (fbuf[0] == 0xFE)
                    ftarget = 7;
                else if (fbuf[0] == 0xFB)
                    ftarget = 515;
                else
                {
                    in_frame = false;
                    sync_cnt = 0;
                    bits_sync = 0;
                }
            }
            else if (flen >= ftarget)
            {
                if (fbuf[0] == 0xFE)
                {
                    uint16_t rx = ((uint16_t)fbuf[5] << 8) | fbuf[6];
                    if (fs_crc16(fbuf, 5) == rx)
                    {
                        id_c = fbuf[1];
                        id_h = fbuf[2];
                        id_s = fbuf[3];
                        valid_id = true;
                    }
                    else
                        valid_id = false;
                }
                else if (fbuf[0] == 0xFB && valid_id)
                {
                    uint16_t rx = ((uint16_t)fbuf[513] << 8) | fbuf[514];
                    if (fs_crc16(fbuf, 513) == rx && id_c == exp_trk && id_h == exp_hd && id_s >= 1 && id_s <= num_sec)
                    {
                        if (!found[id_s])
                        {
                            found[id_s] = true;
                            found_cnt++;
                            memcpy(dest + (id_s - 1) * FS_PKT_LEN, fbuf, FS_PKT_LEN);
                        }
                    }
                    valid_id = false;
                }
                in_frame = false;
                sync_cnt = 0;
                bits_sync = 0;
            }
        }
    };
    for (uint32_t i = 0; i < n_flux && found_cnt < num_sec; i++)
    {
        uint32_t dt = flux_buf[i];
        int z = (dt < T_SHORT) ? 1 : (dt < T_MED ? 2 : 3);
        for (int k = 0; k < z; k++)
            push_bit(0);
        push_bit(1);
    }
    return (found_cnt == num_sec);
}

struct FsMfmBuilder
{
    uint32_t count;
    uint8_t prev_d;
    uint32_t pending;
    uint32_t prev_cells;
    uint32_t precomp_cy;

    void init(uint8_t track)
    {
        count = 0;
        prev_d = 0;
        pending = 0;
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
            int32_t cy = (int32_t)(prev_cells * FS_US_TO_CY(2));
            if (prev_cells == 2 && cur_cells > 2)
                cy += (int32_t)precomp_cy;
            else if (prev_cells > 2 && cur_cells == 2)
                cy -= (int32_t)precomp_cy;

            if (cy > 65000) cy = 65000;
            flux_buf[count++] = (uint16_t)cy;
        }
        prev_cells = cur_cells;
    }

    void bit(uint8_t b)
    {
        if (b == 0)
            pending++;
        else
        {
            if (pending > 0)
                emit_interval(pending);
            pending = 1;
        }
    }

    void finish()
    {
        if (prev_cells > 0 && count < FLUX_BUF_LEN)
        {
            uint32_t cy = prev_cells * FS_US_TO_CY(2);
            flux_buf[count++] = (cy > 65000) ? 65000 : (uint16_t)cy;
            prev_cells = 0;
        }
    }
    void byte(uint8_t d_byte)
    {
        for (int i = 7; i >= 0; i--)
        {
            uint8_t d = (d_byte >> i) & 1;
            bit((prev_d == 0 && d == 0) ? 1 : 0);
            bit(d);
            prev_d = d;
        }
    }
    void a1()
    {
        for (int i = 15; i >= 0; i--)
            bit((0x4489 >> i) & 1);
        prev_d = 1;
    }
    void c2()
    {
        for (int i = 15; i >= 0; i--)
            bit((0x5224 >> i) & 1);
        prev_d = 0;
    }
};
static bool fs_write_track(uint8_t track, uint8_t side, const uint8_t *src_pkts)
{
    if (gpio_get(PROT_PIN) == 0)
        return false;
    if (!fs_drive_sel)
        fs_select(true);
    if (!fs_motor_on)
        fs_motor(true);
    if (!fs_seek(track))
        return false;
    fs_set_side(side);
    FsMfmBuilder mb;
    mb.init(track);
    for (int i = 0; i < 56; i++)
        mb.byte(0x4E);
    for (int i = 0; i < 12; i++)
        mb.byte(0x00);
    for (int i = 0; i < 3; i++)
        mb.c2();
    mb.byte(0xFC);
    for (int i = 0; i < 50; i++)
        mb.byte(0x4E);
    for (uint8_t s = 1; s <= 9; s++)
    {
        const uint8_t *pkt = src_pkts + (s - 1) * FS_PKT_LEN;
        uint16_t dcrc = fs_crc16(pkt, 513);
        for (int i = 0; i < 12; i++)
            mb.byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.a1();
        uint8_t id[5] = {0xFE, track, side, s, 2};
        uint16_t icrc = fs_crc16(id, 5);
        for (int i = 0; i < 5; i++)
            mb.byte(id[i]);
        mb.byte(icrc >> 8);
        mb.byte(icrc & 0xFF);
        for (int i = 0; i < 22; i++)
            mb.byte(0x4E);
        for (int i = 0; i < 12; i++)
            mb.byte(0x00);
        for (int i = 0; i < 3; i++)
            mb.a1();
        for (int i = 0; i < 513; i++)
            mb.byte(pkt[i]);
        mb.byte(dcrc >> 8);
        mb.byte(dcrc & 0xFF);
        if (s < 9)
        {
            for (int i = 0; i < 80; i++)
                mb.byte(0x4E);
        }
    }
    for (int i = 0; i < 200; i++)
        mb.byte(0x4E);
    mb.finish();
    fs_play_flux(flux_buf, mb.count);
    return true;
}
// --- Exxecutes Arduino Mega  UI commands---
static void fs_cmd_get_files(int offset)
{
    if (!fs_ensure_sd())
    {
        fs_i2c_queue_msg("STAT:ERROR SD\n");
        return;
    }
    DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, "/") != FR_OK)
    {
        fs_sd_mounted = false;
        fs_i2c_queue_msg("STAT:ERROR DIR\n");
        return;
    }
    char list_str[256] = {0};
    int total = 0, added = 0;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0)
    {
        if (fno.fattrib & AM_HID)
            continue;
         if (total >= offset && added < 6)
        {
            if (added > 0)
                strncat(list_str, "|", sizeof(list_str) - strlen(list_str) - 1);
            
            if (fno.fattrib & AM_DIR) {
                char item[80];
                snprintf(item, sizeof(item), "%s/", fno.fname);
                strncat(list_str, item, sizeof(list_str) - strlen(list_str) - 1);
            } else {
                char item[96];
                unsigned long kb = (unsigned long)(fno.fsize / 1024);
                snprintf(item, sizeof(item), "%s (%luK)", fno.fname, kb);
                strncat(list_str, item, sizeof(list_str) - strlen(list_str) - 1);
            }
            added++;
        }        
        total++;
    }
    f_closedir(&dir);
    char out[300];
    snprintf(out, sizeof(out), "PAGE:%d:%s\n", total, list_str);
    fs_i2c_queue_msg(out);
}

static void fs_cmd_write(void)
{
    if (gpio_get(PROT_PIN) == 0)
    {
        fs_i2c_queue_msg("STAT:PROTECT!\n");
        return;
    }
    if (!fs_ensure_sd())
    {
        fs_i2c_queue_msg("STAT:ERROR SD\n");
        return;
    }
    FIL fil;
    if (f_open(&fil, fs_selected_file, FA_READ) != FR_OK)
    {
        fs_i2c_queue_msg("STAT:NO FILE\n");
        return;
    }

    FSIZE_t sz = f_size(&fil);
    uint8_t max_trk = 80, max_hd = 2;
    if (sz == 184320)
    {
        max_trk = 40;
        max_hd = 1;
    }
    else if (sz == 368640)
    {
        max_trk = 40;
        max_hd = 2;
    }
    else if (sz == 737280)
    {
        max_trk = 80;
        max_hd = 2;
    }
    else
    {
        f_close(&fil);
        fs_i2c_queue_msg("STAT:SIZE ERR\n");
        return;
    }

    if (!fs_drive_sel)
        fs_select(true);
    if (!fs_motor_on)
        fs_motor(true);
    fs_i2c_queue_msg("STAT:SCRIVO...\n");

    static uint8_t raw[9 * 512];
    static uint8_t verify_buf[FS_MAX_SECTORS * 516];
    char msg[32];
    bool ok = true;

    for (uint8_t t = 0; t < max_trk && ok && !fs_exit_requested; t++)
    {
        for (uint8_t h = 0; h < max_hd && ok && !fs_exit_requested; h++)
        {
            FS_LED_TOGGLE();

            UINT br = 0;
            if (f_read(&fil, raw, sizeof(raw), &br) != FR_OK || br != sizeof(raw))
            {
                ok = false;
                snprintf(msg, sizeof(msg), "SEC:%d:%d:2\n", t, h);
                fs_i2c_queue_msg(msg);
                sleep_ms(25);
                break;
            }
            for (int s = 0; s < 9; s++)
            {
                uint8_t *pkt = fs_track_buf + (s * FS_PKT_LEN);
                pkt[0] = 0xFB;
                memcpy(pkt + 1, raw + (s * 512), 512);
            }

            // Fino a 3 tentativi di scrittura + verifica CRC
            bool trk_written = false;
            for (int retry = 0; retry < 3 && !trk_written && !fs_exit_requested; retry++)
            {
                if (retry > 0)
                {
                    snprintf(msg, sizeof(msg), "SEC:%d:%d:1\n", t, h); // Giallo = Retry
                    fs_i2c_queue_msg(msg);
                    sleep_ms(25);
                    if (retry == 2)
                        fs_cur_cyl = -1; // Ricalibra da TRK0 all'ultimo tentativo
                }
                if (fs_write_track(t, h, fs_track_buf))
                {
                    if (fs_decode_track(t, h, 9, verify_buf))
                    {
                        trk_written = true;
                    }
                }
            }

            if (trk_written)
            {
                snprintf(msg, sizeof(msg), "SEC:%d:%d:0\n", t, h); // Verde = OK
                fs_i2c_queue_msg(msg);
                sleep_ms(25); 
            }
            else
            {
                ok = false;
                snprintf(msg, sizeof(msg), "SEC:%d:%d:2\n", t, h); // Rosso = Errore
                fs_i2c_queue_msg(msg);
            }
        }
    }

    f_close(&fil);

    if (!ok || fs_exit_requested)
    {
        fs_motor(false);
        fs_select(false);
        fs_motor_off_ms = 0;
    }
    else
    {
        fs_motor_off_ms = to_ms_since_boot(get_absolute_time()) + 3000;
    }

    fs_i2c_queue_msg(ok ? "STAT:WRITE OK\n" : "STAT:WRITE ERR.\n");
}

static void fs_cmd_dump(void)
{
    if (!fs_ensure_sd()) {
        fs_i2c_queue_msg("STAT:ERROR SD\n");
        return;
    }

    // Geometry calc
    uint8_t max_trk = 80, max_hd = 2;
    const char *prefix = "BK720";
    if (strstr(fs_selected_proto, "360")) {
        max_trk = 40; max_hd = 2; prefix = "BK360";
    } else if (strstr(fs_selected_proto, "180")) {
        max_trk = 40; max_hd = 1; prefix = "BK180";
    }

    // find first free file number
    char auto_name[16] = {0};
    FIL test_fil;
    bool found_free = false;
    for (int idx = 1; idx <= 99; idx++) {
        snprintf(auto_name, sizeof(auto_name), "%s_%02d.IMG", prefix, idx);
        FRESULT fr_chk = f_open(&test_fil, auto_name, FA_READ);
        if (fr_chk == FR_NO_FILE) {
            found_free = true;
            break;
        } else if (fr_chk == FR_OK) {
            f_close(&test_fil);
        }
    }

    if (!found_free) {
        fs_i2c_queue_msg("STAT:SD FULL (99)\n");
        return;
    }

    FIL fil;
    if (f_open(&fil, auto_name, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
        fs_i2c_queue_msg("STAT:ERR CREATE\n");
        return;
    }

    // send to arduino the filename
    char info_msg[32];
    snprintf(info_msg, sizeof(info_msg), "FILE:%s\n", auto_name);
    fs_i2c_queue_msg(info_msg);
    snprintf(info_msg, sizeof(info_msg), "STAT:%s\n", auto_name);
    fs_i2c_queue_msg(info_msg);

    if (!fs_drive_sel) fs_select(true);
    if (!fs_motor_on)  fs_motor(true);

    fs_i2c_queue_msg("STAT:READING..\n");
    static uint8_t raw[9 * 512];
    char msg[32];
    bool ok = true;
    for (uint8_t t = 0; t < max_trk && ok && !fs_exit_requested; t++)
    {
        if (!fs_seek(t))
        {
            ok = false;
            break;
        }
        for (uint8_t h = 0; h < max_hd && ok && !fs_exit_requested; h++)
        {
            fs_set_side(h);
            FS_LED_TOGGLE();
            snprintf(msg, sizeof(msg), "TRK:%d:%d\n", t, h);
            fs_i2c_queue_msg(msg);
            bool trk_ok = false;
            for (int r = 0; r < 3 && !trk_ok; r++)
            {
                trk_ok = fs_decode_track(t, h, 9, fs_track_buf);
            }
            if (!trk_ok)
            {
                ok = false;
                snprintf(msg, sizeof(msg), "SEC:%d:%d:2\n", t, h);
                fs_i2c_queue_msg(msg);
                sleep_ms(25);
                break;
            }
            for (int s = 0; s < 9; s++)
            {
                memcpy(raw + (s * 512), fs_track_buf + (s * FS_PKT_LEN) + 1, 512);
            }
            UINT bw = 0;
            if (f_write(&fil, raw, sizeof(raw), &bw) == FR_OK && bw == sizeof(raw))
            {
                snprintf(msg, sizeof(msg), "SEC:%d:%d:0\n", t, h);
                fs_i2c_queue_msg(msg);
                sleep_ms(25);
            }
            else
            {
                ok = false;
                snprintf(msg, sizeof(msg), "SEC:%d:%d:2\n", t, h);
                fs_i2c_queue_msg(msg);
                sleep_ms(25);
            }
        }
    }
    f_close(&fil);

    if (!ok || fs_exit_requested) {
        fs_motor(false);
        fs_select(false);
        fs_motor_off_ms = 0;
        f_unlink(auto_name);    // delete not completed file     
    } else {
        fs_motor_off_ms = to_ms_since_boot(get_absolute_time()) + 3000;
    }

    fs_i2c_queue_msg(ok ? "STAT:DUMP OK\n" : "STAT:ERR DUMP\n");
}
static void fs_clean_filename(const char *raw, char *out, size_t out_sz)
{
    strncpy(out, raw, out_sz - 1);
    out[out_sz - 1] = '\0';
    char *tag = strstr(out, " (");
    if (tag != NULL) {
        *tag = '\0'; // Tronca subito prima di " ("
    }
}

void floppystation_run(void)
{
    fs_exit_requested = false;
    fs_cmd_pending = false;
    fs_hw_init();
    fs_i2c_queue_msg("STAT:FS ACTIVE\n");
    while (!fs_exit_requested)
    {
        if (fs_cmd_pending)
        {
            char cmd[96];
            strncpy(cmd, fs_cmd_buf, sizeof(cmd));
            fs_cmd_pending = false;
            if (strncmp(cmd, "CMD:GET_FILES:", 14) == 0)
            {
                fs_cmd_get_files(atoi(cmd + 14));
            }
            else if (strncmp(cmd, "CMD:SELECT:", 11) == 0)
            {
                fs_clean_filename(cmd + 11, fs_selected_file, sizeof(fs_selected_file));
                
                char resp[80]; 
                snprintf(resp, sizeof(resp), "FILE:%s\n", fs_selected_file);
                fs_i2c_queue_msg(resp);

                FILINFO fno;
                if (fs_ensure_sd() && f_stat(fs_selected_file, &fno) == FR_OK) {
                    snprintf(resp, sizeof(resp), "SIZE:%lu\n", (unsigned long)fno.fsize);
                    fs_i2c_queue_msg(resp);
                }
            }
            else if (strncmp(cmd, "CMD:DEL:", 8) == 0) 
            {
                char clean_target[64]; 
                fs_clean_filename(cmd + 8, clean_target, sizeof(clean_target));
                if (fs_ensure_sd() && f_unlink(clean_target) == FR_OK) {
                    fs_i2c_queue_msg("STAT:DELETED\n");
                } else {
                    fs_i2c_queue_msg("STAT:ERR DEL\n");
                }
            }

            else if (strncmp(cmd, "CMD:SET_PROTO:", 14) == 0)
            {
                strncpy(fs_selected_proto, cmd + 14, sizeof(fs_selected_proto) - 1);
                fs_selected_proto[sizeof(fs_selected_proto) - 1] = '\0';
                char resp[48];
                snprintf(resp, sizeof(resp), "PROTO:%s\n", fs_selected_proto);
                fs_i2c_queue_msg(resp);
            }
            else if (strcmp(cmd, "CMD:WRITE") == 0)
            {
                fs_cmd_write();
            }
            else if (strcmp(cmd, "CMD:DUMP") == 0)
            {
                fs_cmd_dump();
            }
        }
        if (fs_motor_off_ms > 0 && to_ms_since_boot(get_absolute_time()) > fs_motor_off_ms)
        {
            fs_motor(false);
            fs_select(false);
            fs_motor_off_ms = 0;
        }
        sleep_ms(5);
    }
    fs_motor(false);
    fs_select(false);
    g_fs_mode_requested = false;
    fs_i2c_queue_msg("STAT:EXIT FS\n");
}