#include <cstring>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "rf_protocol.h"
#include "bridge.h"
#include "tusb.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "ff.h"

#include "floppystation.h"

// --- I2C Slave (address 0x42) ---
static char i2c_rx_buf[96];
static uint8_t i2c_rx_idx = 0;

// --- hardware cycles (RP2040 SysTick 24-bit vs RP2350 DWT 32-bit) ---
#if PICO_RP2040
#include "hardware/structs/systick.h"
#define GET_CYCCNT() (systick_hw->cvr)
#define CY_ELAPSED(start, now) (((start) - (now)) & 0x00FFFFFFu)
#define CY_ADD(base, delta) (((base) - (delta)) & 0x00FFFFFFu)
#define CY_REACHED(now, target) ((((int32_t)(((target) - (now)) << 8)) >> 8) >= 0)


//----------------------------------------
//  static void enable_cycle_counter(void)
//----------------------------------------

static void enable_cycle_counter(void)
{
  systick_hw->rvr = 0x00FFFFFFu; // 24-bit max (~111.8 ms a 150 MHz)
  systick_hw->cvr = 0;
  systick_hw->csr = 0x5u; // Clock CPU + Enable
}
#else
#define DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define GET_CYCCNT() (DWT_CYCCNT)
#define CY_ELAPSED(start, now) ((uint32_t)((now) - (start)))
#define CY_ADD(base, delta) ((uint32_t)((base) + (delta)))
#define CY_REACHED(now, target) ((int32_t)((now) - (target)) >= 0)

static void enable_cycle_counter(void)
{
  DEMCR |= (1u << 24);
  DWT_CYCCNT = 0;
  DWT_CTRL |= 1u;
}
#endif

static uint8_t rx_block[XFER_BUF_LEN];
static uint32_t rx_pos = 0;
static uint32_t rx_avail = 0;

uint16_t flux_buf[FLUX_BUF_LEN];


enum
{
  CMD_GET_INFO = 0,
  CMD_SEEK = 2,
  CMD_HEAD = 3,
  CMD_SET_PARAMS = 4,
  CMD_GET_PARAMS = 5,
  CMD_MOTOR = 6,
  CMD_READ_FLUX = 7,
  CMD_WRITE_FLUX = 8,
  CMD_GET_FLUX_STATUS = 9,
  CMD_GET_INDEX_TIMES = 10,
  CMD_SELECT = 12,
  CMD_DESELECT = 13,
  CMD_SET_BUS_TYPE = 14,
  CMD_SET_PIN = 15,
  CMD_RESET = 16,
  CMD_SOURCE_BYTES = 18,
  CMD_SINK_BYTES = 19,
  CMD_GET_PIN = 20,
  CMD_NO_CLICK_STEP = 22,
};

enum
{
  ACK_OK = 0,
  ACK_BAD_CMD = 1,
  ACK_NO_INDEX = 2,
  ACK_NO_TRK0 = 3,
  ACK_WRPROT = 6,
  ACK_BAD_CYL = 11,
};

enum
{
  FLUXOP_INDEX = 1,
  FLUXOP_SPACE = 2,
};

static uint8_t g_cmd_buf[CMD_BUF_LEN];
static uint8_t g_cmd_len = 0;
static uint16_t gw_delays[DELAYS_LEN] = {0, 3000, 15, 600, 10000, 2000, 2000, 5000};
static uint32_t bw_min_bytes, bw_min_us, bw_max_bytes, bw_max_us;

static bool drive_selected = false;
static bool motor_running = false;
static int16_t current_cyl = -1;
static uint8_t last_flux_status = ACK_OK;


static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t b = i2c_read_byte_raw(i2c);
            if (b == '\n') {
                i2c_rx_buf[i2c_rx_idx] = '\0';
                i2c_rx_idx = 0;
                fs_i2c_on_rx_line(i2c_rx_buf);
            } else if (b != '\r' && i2c_rx_idx < sizeof(i2c_rx_buf) - 1) {
                i2c_rx_buf[i2c_rx_idx++] = (char)b;
            }
            break;
        }
        case I2C_SLAVE_REQUEST: {
            i2c_write_byte_raw(i2c, fs_i2c_pop_tx_byte());
            break;
        }
        case I2C_SLAVE_FINISH:
            break;
    }
}



static inline uint8_t usb_read_byte_fast(void)
{
  while (rx_pos >= rx_avail)
  {
    tud_task();
    uint32_t n = tud_cdc_available();
    if (n > 0)
    {
      if (n > sizeof(rx_block))
        n = sizeof(rx_block);
      rx_avail = tud_cdc_read(rx_block, n);
      rx_pos = 0;
    }
  }
  return rx_block[rx_pos++];
}

static uint32_t usb_read_n28(void)
{
  uint32_t val = 0;
  val |= (uint32_t)(usb_read_byte_fast() >> 1) << 0;
  val |= (uint32_t)(usb_read_byte_fast() >> 1) << 7;
  val |= (uint32_t)(usb_read_byte_fast() >> 1) << 14;
  val |= (uint32_t)(usb_read_byte_fast() >> 1) << 21;
  return val;
}

// --- Encoder Greaseweazle protocol ---
static uint8_t *write_n28(uint8_t *p, uint32_t val)
{
  *p++ = (uint8_t)(((val >> 0) & 0x7F) << 1) | 1;
  *p++ = (uint8_t)(((val >> 7) & 0x7F) << 1) | 1;
  *p++ = (uint8_t)(((val >> 14) & 0x7F) << 1) | 1;
  *p++ = (uint8_t)(((val >> 21) & 0x7F) << 1) | 1;
  return p;
}

static uint8_t *greasepack_val(uint8_t *p, uint32_t val)
{
  if (val == 0)
    val = 1;
  if (val <= 249)
  {
    *p++ = (uint8_t)val;
  }
  else if (val <= 1524)
  {
    uint32_t v = val - 250;
    *p++ = (uint8_t)(250 + (v / 255));
    *p++ = (uint8_t)((v % 255) + 1);
  }
  else
  {
    *p++ = 0xFF;
    *p++ = FLUXOP_SPACE;
    p = write_n28(p, val - 1);
    *p++ = 0x01;
  }
  return p;
}

// --- Hardware Floppy ---
static void setup_in_pin(uint pin)
{
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
  gpio_set_input_enabled(pin, true);
  gpio_set_input_hysteresis_enabled(pin, true);
}

static void setup_out_pin(uint pin, bool initial_high)
{
  gpio_init(pin);
  gpio_put(pin, initial_high ? 1 : 0);
  gpio_set_dir(pin, GPIO_OUT);
}

static void floppy_hw_init(void)
{
  setup_out_pin(PICO_DEFAULT_LED_PIN, false);
  setup_out_pin(DENSITY_PIN, true);

  setup_out_pin(SELECT_A_PIN, true);
  setup_out_pin(MOTOR_A_PIN, true);
  setup_out_pin(SELECT_B_PIN, true);
  setup_out_pin(MOTOR_B_PIN, true);

  setup_out_pin(DIR_PIN, true);
  setup_out_pin(STEP_PIN, true);
  setup_out_pin(WRDATA_PIN, true);
  setup_out_pin(WRGATE_PIN, true);
  setup_out_pin(SIDE_PIN, true);

  setup_in_pin(INDEX_PIN);
  setup_in_pin(TRK0_PIN);
  setup_in_pin(PROT_PIN);
  setup_in_pin(READ_PIN);
  setup_in_pin(READY_PIN);
  current_cyl = -1;
  drive_selected = false; 
  motor_running = false;
}

static void hw_select(bool on)
{
  drive_selected = on;
  gpio_put(g_active_select_pin, on ? 0 : 1);
  if (on && gw_delays[0] > 0)
    sleep_us(gw_delays[0]);
}

static void hw_motor(bool on)
{
  if (on && !motor_running)
  {
    motor_running = true;
    gpio_put(g_active_motor_pin, 0);
    sleep_ms(gw_delays[3] ? gw_delays[3] : 500);
  }
  else if (!on)
  {
    motor_running = false;
    gpio_put(g_active_motor_pin, 1);
  }
}

static void hw_step_pulse(bool inward)
{
  gpio_put(DIR_PIN, inward ? 0 : 1);
  sleep_us(5);
  gpio_put(STEP_PIN, 0);
  sleep_us(10);
  gpio_put(STEP_PIN, 1);
  uint32_t step_us = gw_delays[1] ? gw_delays[1] : g_active_step_us;
  sleep_us(step_us);
}

static bool hw_seek(uint8_t target_cyl)
{
  if (target_cyl == 0 || current_cyl < 0)
  {
    for (int i = 0; i < 90; i++)
    {
      if (gpio_get(TRK0_PIN) == 0)
      {
        current_cyl = 0;
        break;
      }
      hw_step_pulse(false);
    }
    if (gpio_get(TRK0_PIN) != 0)
      return false;
    current_cyl = 0;
  }
  while (current_cyl < target_cyl)
  {
    hw_step_pulse(true);
    current_cyl++;
  }
  while (current_cyl > target_cyl)
  {
    hw_step_pulse(false);
    current_cyl--;
  }
  sleep_ms(gw_delays[2] ? gw_delays[2] : 15);
  return true;
}

// --- USB ---
static void gw_send_response(const uint8_t *data, uint32_t len)
{
  uint32_t w = 0;
  while (w < len)
  {
    tud_task();
    uint32_t avail = tud_cdc_write_available();
    if (avail == 0)
    {
      tud_cdc_write_flush();
      continue;
    }
    uint32_t n = len - w;
    if (n > avail)
      n = avail;
    tud_cdc_write(data + w, n);
    w += n;
  }
  tud_cdc_write_flush();
}

static void gw_send_ack(uint8_t cmd, uint8_t code)
{
  uint8_t ack_buf[2] = {cmd, code};
  gw_send_response(ack_buf, 2);
}

// --- Waint Index Hole with LED blink ---
static uint32_t wait_index_pulse(uint32_t timeout_us)
{
  uint32_t t_start = time_us_32();

  while (gpio_get(INDEX_PIN) == 0)
  {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    tud_task();
    if (time_us_32() - t_start > timeout_us)
      return 0;
  }
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(2);

  while (gpio_get(INDEX_PIN) == 1)
  {
    tud_task();
    if (time_us_32() - t_start > timeout_us)
      return 0;
  }
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  return time_us_32();
}

// --- Read flux in SRAM on RP2040/RP2350 ---
static void __not_in_flash_func(cmd_read_flux)(void)
{
  uint32_t ticks_limit;
  uint16_t revs;
  memcpy(&ticks_limit, g_cmd_buf + 2, 4);
  memcpy(&revs, g_cmd_buf + 6, 2);

  gw_send_ack(CMD_READ_FLUX, ACK_OK);

  if (!drive_selected)
    hw_select(true);
  if (!motor_running)
    hw_motor(true);

  uint16_t target_revs = (revs > 1) ? (revs - 1) : 1;
  if (target_revs > 5)
    target_revs = 5;

  uint32_t target_us = 0;
  if (ticks_limit > 0)
  {
    target_us = (uint32_t)(((uint64_t)ticks_limit * 101ULL) / 10000ULL);
  }
  uint32_t max_loop_us = (uint32_t)(target_revs + 2) * 300000u;

  const uint32_t idx_mask_bit = (1u << INDEX_PIN);
  const uint32_t read_mask_bit = (1u << READ_PIN);
  uint32_t index_pos[IDX_ARRAY_LEN];
  uint16_t index_cnt = 0;
  uint32_t count = 0;
  uint32_t irq_state = save_and_disable_interrupts();
  if (revs > 0)
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
  uint32_t capture_start_us = time_us_32();
  uint32_t rev_start_us = capture_start_us;
  uint32_t last_edge_cy = 0;
  bool has_last_edge = false;
  index_pos[index_cnt++] = 0;
  uint32_t prev_pins = sio_hw->gpio_in;
  while (count < FLUX_BUF_LEN)
  {
    uint32_t pins = sio_hw->gpio_in;
    uint32_t now_cy = GET_CYCCNT();
    uint32_t now_us = time_us_32();
    uint32_t elapsed_total = now_us - capture_start_us;
    if (elapsed_total > max_loop_us)
      break;
    if (revs > 0)
    {
      if (index_cnt > target_revs)
        break;
      if (target_us > 0 && elapsed_total >= target_us)
        break;
    }
    else
    {
      if (target_us > 0 && elapsed_total >= target_us)
        break;
    }
    uint32_t falling = prev_pins & ~pins;
    prev_pins = pins;
    if (falling & read_mask_bit)
    {
      if (has_last_edge)
      {
        uint32_t delta_ticks = CY_TO_TICKS(CY_ELAPSED(last_edge_cy, now_cy));
        if (delta_ticks > 65000)
          delta_ticks = 65000;
        flux_buf[count++] = (uint16_t)delta_ticks;
      }
      last_edge_cy = now_cy;
      has_last_edge = true;
    }
    if ((falling & idx_mask_bit) && (now_us - rev_start_us) >= 150000u)
    {
      rev_start_us = now_us;
      if (index_cnt < IDX_ARRAY_LEN)
        index_pos[index_cnt++] = count;
    }
  }
  restore_interrupts(irq_state);
  uint8_t idx_pkt[IDX_PKT_LEN] = {0xFF, FLUXOP_INDEX, 1, 1, 1, 1};
  gw_send_response(idx_pkt, IDX_PKT_LEN);
  uint8_t ob[TX_CHUNK_LEN];
  uint8_t *op = ob;
  uint16_t next_idx = 1;
  for (uint32_t i = 0; i < count; i++)
  {
    op = greasepack_val(op, flux_buf[i]);
    if ((op - ob) >= 900)
    {
      gw_send_response(ob, (uint32_t)(op - ob));
      op = ob;
    }
    if (next_idx < index_cnt && (i + 1) == index_pos[next_idx])
    {
      if (op > ob)
      {
        gw_send_response(ob, (uint32_t)(op - ob));
        op = ob;
      }
      gw_send_response(idx_pkt, IDX_PKT_LEN);
      next_idx++;
    }
  }
  if (op > ob)
  {
    gw_send_response(ob, (uint32_t)(op - ob));
  }
  uint8_t end_byte = 0;
  gw_send_response(&end_byte, 1);
}

static void __not_in_flash_func(cmd_write_flux)(void)
{
  uint8_t cue_at_index = g_cmd_buf[2];
  uint8_t terminate_at_index = g_cmd_buf[3];
  if (!drive_selected)
    hw_select(true);
  if (!motor_running)
    hw_motor(true);
  if (gpio_get(PROT_PIN) == 0)
  {
    last_flux_status = ACK_WRPROT;
    gw_send_ack(CMD_WRITE_FLUX, ACK_WRPROT);
    return;
  }
  rx_pos = 0;
  rx_avail = 0;
  gw_send_ack(CMD_WRITE_FLUX, ACK_OK);
  uint32_t count = 0;
  uint32_t pending_space = 0;
  while (true)
  {
    uint8_t b = usb_read_byte_fast();
    if (b == 0)
      break;
    uint32_t ticks = 0;
    if (b <= 249)
    {
      ticks = b;
    }
    else if (b <= 254)
    {
      uint8_t b2 = usb_read_byte_fast();
      ticks = 250 + ((uint32_t)(b - 250) * 255) + (b2 - 1);
    }
    else
    {
      uint8_t op = usb_read_byte_fast();
      if (op == FLUXOP_INDEX || op == FLUXOP_SPACE)
      {
        uint32_t val = usb_read_n28();
        if (op == FLUXOP_SPACE)
          pending_space += val;
        continue;
      }
      else
      {
        usb_read_n28();
        continue;
      }
    }
    uint32_t total = pending_space + ticks;
    pending_space = 0;
    if (count < FLUX_BUF_LEN)
    {
      flux_buf[count++] = (total > 65535) ? 65535 : (uint16_t)total;
    }
  }
  if (count == 0)
  {
    last_flux_status = ACK_OK;
    uint8_t done = 0;
    gw_send_response(&done, 1);
    return;
  }
  const uint32_t idx_mask_bit = (1u << INDEX_PIN);
  const uint32_t wrdata_bit = (1u << WRDATA_PIN);
  const uint32_t wrgate_bit = (1u << WRGATE_PIN);
  const uint32_t pulse_low_cy = (600u * SYS_CLOCK_KHZ) / 1000000u; // 600 ns esatti
  const uint32_t post_idx_cy = US_TO_CY(1500);                     // 1,5 ms post-index
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
  uint32_t target_cy = GET_CYCCNT();
  uint32_t prev_pins = sio_hw->gpio_in;
  bool index_seen = false;
  uint32_t index_end_cy = 0;
  for (uint32_t i = 0; i < count; i++)
  {
    uint32_t interval_cy = TICKS_TO_CY(flux_buf[i]);
    target_cy = CY_ADD(target_cy, interval_cy);
    while (!CY_REACHED(GET_CYCCNT(), target_cy))
    {
      if (terminate_at_index)
      {
        uint32_t pins = sio_hw->gpio_in;
        uint32_t falling = prev_pins & ~pins;
        prev_pins = pins;
        if (!index_seen)
        {
          if ((falling & idx_mask_bit) && (time_us_32() - write_start_us) >= 150000u)
          {
            index_seen = true;
            index_end_cy = CY_ADD(GET_CYCCNT(), post_idx_cy);
          }
        }
        else if (CY_REACHED(GET_CYCCNT(), index_end_cy))
        {
          goto end_write;
        }
      }
    }
    sio_hw->gpio_clr = wrdata_bit;
    uint32_t p_end = CY_ADD(GET_CYCCNT(), pulse_low_cy);
    while (!CY_REACHED(GET_CYCCNT(), p_end))
    {
    }
    sio_hw->gpio_set = wrdata_bit;
  }
  {
    uint32_t tail_end = CY_ADD(GET_CYCCNT(), US_TO_CY(50)); 
    while (!CY_REACHED(GET_CYCCNT(), tail_end))
    {
    }
  }
end_write:
  sio_hw->gpio_set = wrdata_bit | wrgate_bit;
  restore_interrupts(irq_state);
  last_flux_status = ACK_OK;
  uint8_t done = 0;
  gw_send_response(&done, 1);
}
static void cmd_get_index_times(void)
{
  uint16_t revs;
  memcpy(&revs, g_cmd_buf + 2, 2);
  if (revs == 0 || revs > 10)
    revs = 1;
  if (!drive_selected)
    hw_select(true);
  if (!motor_running)
    hw_motor(true);
  uint32_t times[TIMES_LEN];
  uint32_t t0 = wait_index_pulse(1000000);
  if (t0 == 0)
  {
    gw_send_ack(CMD_GET_INDEX_TIMES, ACK_NO_INDEX);
    return;
  }
  times[0] = 0;
  for (uint16_t r = 1; r <= revs; r++)
  {
    uint32_t t_now = wait_index_pulse(1000000);
    if (t_now == 0)
    {
      gw_send_ack(CMD_GET_INDEX_TIMES, ACK_NO_INDEX);
      return;
    }
    times[r] = (uint32_t)((uint64_t)(t_now - t0) * (GW_SAMPLE_FREQ_HZ / 1000000));
  }
  uint8_t resp[RESP_BUF_LEN];
  resp[0] = CMD_GET_INDEX_TIMES;
  resp[1] = ACK_OK;
  memcpy(resp + 2, times, (revs + 1) * 4);
  gw_send_response(resp, 2 + (revs + 1) * 4);
}
static void cmd_get_info(void)
{
  uint8_t subcmd = g_cmd_buf[2];
  uint8_t resp[RESP_BUF_LEN];
  resp[0] = CMD_GET_INFO;
  resp[1] = ACK_OK;
  memset(resp + 2, 0, 32);
  switch (subcmd)
  {
  case 0:
  {
    resp[2] = 3;  // fw_major = 1
    resp[3] = 0;  // fw_minor = 6
    resp[4] = 1;  // is_main_firmware = 1
    resp[5] = 22; // max_cmd = 22
    uint32_t sf = GW_SAMPLE_FREQ_HZ;
    memcpy(resp + 6, &sf, 4);
    resp[10] = 4; // hw_model: 4 = Greaseweazle V4
    resp[11] = 0; // hw_submodel
    uint16_t mhz = (uint16_t)(clock_get_hz(clk_sys) / 1000000);
    memcpy(resp + 14, &mhz, 2);
#if PICO_RP2040
    uint16_t sram = 264;
#else
    uint16_t sram = 512;
#endif
    memcpy(resp + 16, &sram, 2);
    uint16_t usbbuf = 4;
    memcpy(resp + 18, &usbbuf, 2);
    break;
  }
  case 1:
    memcpy(resp + 2, &bw_min_bytes, 4);
    memcpy(resp + 6, &bw_min_us, 4);
    memcpy(resp + 10, &bw_max_bytes, 4);
    memcpy(resp + 14, &bw_max_us, 4);
    break;
  case 7:
  {
    uint32_t flags = motor_running ? 2 : 0;
    uint32_t cyl = (current_cyl >= 0) ? (uint32_t)current_cyl : 0;
    memcpy(resp + 2, &flags, 4);
    memcpy(resp + 6, &cyl, 4);
    break;
  }
  default:
    resp[1] = ACK_BAD_CMD;
    break;
  }
  gw_send_response(resp, 34);
}
static void cmd_source_bytes(void)
{
  uint32_t nr, seed;
  memcpy(&nr, g_cmd_buf + 2, 4);
  memcpy(&seed, g_cmd_buf + 6, 4);
  gw_send_ack(CMD_SOURCE_BYTES, ACK_OK);
  uint32_t t0 = time_us_32();
  uint32_t nr_total = nr;
  uint32_t randnum = seed;
  uint8_t buf[XFER_BUF_LEN];
  while (nr > 0)
  {
    uint32_t chunk = nr > sizeof(buf) ? sizeof(buf) : nr;
    for (uint32_t i = 0; i < chunk; i++)
    {
      buf[i] = (uint8_t)randnum;
      randnum = (randnum & 0x01) ? ((randnum >> 1) ^ 0x80000062) : (randnum >> 1);
    }
    gw_send_response(buf, chunk);
    nr -= chunk;
  }
  uint32_t elapsed = time_us_32() - t0;
  if (elapsed == 0)
    elapsed = 1;
  bw_min_bytes = bw_max_bytes = nr_total;
  bw_min_us = bw_max_us = elapsed;
}
static void cmd_sink_bytes(void)
{
  uint32_t nr;
  memcpy(&nr, g_cmd_buf + 2, 4);
  gw_send_ack(CMD_SINK_BYTES, ACK_OK);
  uint32_t t0 = time_us_32();
  uint32_t nr_total = nr;
  uint8_t buf[XFER_BUF_LEN];
  while (nr > 0)
  {
    tud_task();
    uint32_t avail = tud_cdc_available();
    if (avail == 0)
      continue;
    uint32_t n = avail > sizeof(buf) ? sizeof(buf) : avail;
    if (n > nr)
      n = nr;
    tud_cdc_read(buf, n);
    nr -= n;
  }
  uint32_t elapsed = time_us_32() - t0;
  if (elapsed == 0)
    elapsed = 1;
  bw_min_bytes = bw_max_bytes = nr_total;
  bw_min_us = bw_max_us = elapsed;
  uint8_t ack = ACK_OK;
  gw_send_response(&ack, 1);
}
static void gw_dispatch(void)
{
  uint8_t cmd = g_cmd_buf[0];
  switch (cmd)
  {
  case CMD_GET_INFO:
    cmd_get_info();
    break;
  case CMD_SET_BUS_TYPE:
    gw_send_ack(CMD_SET_BUS_TYPE, ACK_OK);
    break;
  case CMD_RESET:
    hw_motor(false);
    hw_select(false);
    current_cyl = -1;
    gw_send_ack(CMD_RESET, ACK_OK);
    break;
  case CMD_SELECT:
    hw_select(true);
    gw_send_ack(CMD_SELECT, ACK_OK);
    break;
  case CMD_DESELECT:
    hw_select(false);
    gw_send_ack(CMD_DESELECT, ACK_OK);
    break;
  case CMD_MOTOR:
    hw_motor(g_cmd_buf[3] != 0);
    gw_send_ack(CMD_MOTOR, ACK_OK);
    break;
  case CMD_SEEK:
  {
    int16_t cyl = (g_cmd_len == 3)
                      ? (int8_t)g_cmd_buf[2]
                      : (int16_t)(g_cmd_buf[2] | (g_cmd_buf[3] << 8));
    if (cyl < 0 || cyl > 83)
    {
      gw_send_ack(CMD_SEEK, ACK_BAD_CYL);
      break;
    }
    if (!drive_selected)
      hw_select(true);
    if (!motor_running)
      hw_motor(true);
    gw_send_ack(CMD_SEEK, hw_seek((uint8_t)cyl) ? ACK_OK : ACK_NO_TRK0);
    break;
  }
  case CMD_HEAD:
    gpio_put(SIDE_PIN, g_cmd_buf[2] == 0 ? 1 : 0);
    gw_send_ack(CMD_HEAD, ACK_OK);
    break;
  case CMD_READ_FLUX:
    cmd_read_flux();
    break;
  case CMD_WRITE_FLUX:
    cmd_write_flux();
    break;
  case CMD_GET_FLUX_STATUS:
    gw_send_ack(CMD_GET_FLUX_STATUS, last_flux_status);
    break;
  case CMD_GET_PIN:
  {
    uint8_t pin = g_cmd_buf[2];
    bool lvl = true;
    if (pin == 26)
      lvl = gpio_get(TRK0_PIN);
    else if (pin == 28)
      lvl = gpio_get(PROT_PIN);
    else if (pin == 2)
      lvl = gpio_get(READY_PIN);
    else if (pin == 0)
      lvl = gpio_get(INDEX_PIN);
    uint8_t resp[3] = {CMD_GET_PIN, ACK_OK, (uint8_t)(lvl ? 1 : 0)};
    gw_send_response(resp, 3);
    break;
  }
  case CMD_SET_PIN:
  {
    uint8_t pin = g_cmd_buf[2];
    uint8_t lvl = g_cmd_buf[3];
    if (pin == 2)
      gpio_put(DENSITY_PIN, lvl ? 1 : 0);
    if (pin == 24)
      hw_motor(lvl == 0);
    if (pin == 23)
      hw_select(lvl == 0);
    if (pin == 4)
      gpio_put(SIDE_PIN, lvl ? 1 : 0);
    gw_send_ack(CMD_SET_PIN, ACK_OK);
    break;
  }
  case CMD_GET_INDEX_TIMES:
    cmd_get_index_times();
    break;
  case CMD_GET_PARAMS:
  {
    uint8_t nr = g_cmd_buf[3];
    if (nr > sizeof(gw_delays))
      nr = sizeof(gw_delays);
    uint8_t resp[RESP_BUF_LEN];
    resp[0] = CMD_GET_PARAMS;
    resp[1] = ACK_OK;
    memcpy(resp + 2, gw_delays, nr);
    gw_send_response(resp, 2 + nr);
    break;
  }
  case CMD_SET_PARAMS:
  {
    uint8_t datalen = g_cmd_len - 3;
    if (datalen > sizeof(gw_delays))
      datalen = sizeof(gw_delays);
    memcpy(gw_delays, g_cmd_buf + 3, datalen);
    gw_send_ack(CMD_SET_PARAMS, ACK_OK);
    break;
  }
  case CMD_NO_CLICK_STEP:
    gpio_put(DIR_PIN, 1);
    sleep_us(1);
    gpio_put(STEP_PIN, 0);
    sleep_us(10);
    gpio_put(STEP_PIN, 1);
    sleep_us(1);
    gpio_put(DIR_PIN, 0);
    gw_send_ack(CMD_NO_CLICK_STEP, ACK_OK);
    break;
  case CMD_SOURCE_BYTES:
    cmd_source_bytes();
    break;
  case CMD_SINK_BYTES:
    cmd_sink_bytes();
    break;
  default:
    gw_send_ack(cmd, ACK_BAD_CMD);
    break;
  }
}


int main(void)
{
  set_sys_clock_khz(SYS_CLOCK_KHZ, true);
  enable_cycle_counter();

  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
  i2c_init(I2C_PORT, 100000);
  i2c_slave_init(I2C_PORT, 0x42, &i2c_slave_handler);

  tusb_init();

  while (true)
  {
    g_mode_changed = false;

    // Enter FloppyStation if requested from display
    if (g_fs_mode_requested) {
        floppystation_run();
        g_mode_changed = true; 
        continue;
    }

    // 2. REALFLOPPY (Default)
    if (g_rf_mode_active) {
        rf_protocol_init();
        while (g_rf_mode_active && !g_fs_mode_requested && !g_mode_changed) {
            rf_protocol_poll();
            tud_task(); 
        }
    }
    // 3. GREASEWEAZLE
    else {
        gpio_put(CTS_OUT_PIN, 1); // disable FT232RL traffic
        floppy_hw_init();
        while (!g_rf_mode_active && !g_fs_mode_requested && !g_mode_changed) {
            tud_task();
            gpio_put(PICO_DEFAULT_LED_PIN, gpio_get(INDEX_PIN) == 0 ? 1 : 0);
            if (tud_cdc_available() >= 1)
            {
                uint8_t first = (uint8_t)tud_cdc_read_char();
                while (tud_cdc_available() < 1) { tud_task(); }
                g_cmd_buf[0] = first;
                g_cmd_buf[1] = (uint8_t)tud_cdc_read_char();
                g_cmd_len = g_cmd_buf[1];
                if (g_cmd_len < 2 || g_cmd_len > sizeof(g_cmd_buf))
                {
                    gw_send_ack(g_cmd_buf[0], ACK_BAD_CMD);
                    continue;
                }
                while (tud_cdc_available() < (int)(g_cmd_len - 2)) { tud_task(); }
                for (uint8_t i = 2; i < g_cmd_len; i++)
                {
                    g_cmd_buf[i] = (uint8_t)tud_cdc_read_char();
                }
                gw_dispatch();
            }
        }
    }
  }
}
