/* SPDX-License-Identifier: MIT
 * ELE v9.0.0 — M5Stack Cardputer-Adv (ESP32-S3) Real Hardware Lab Firmware
 * Executes onboard physical experiments over USB CDC Serial (115200 baud):
 *   - PING         : Chip rev, CPU clock, anonymized SHA-256 device ID
 *   - EXP_CSI      : Real 2.4 GHz live OFDM Wi-Fi CSI (complex I/Q subcarriers)
 *   - EXP_THERMAL  : Real silicon die heating/cooling + rail droop step response
 *   - EXP_GPIO_RC  : Real 12-bit factory-calibrated ADC noise, GPIO1 pull-up step & I2C scan
 */
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_chip_info.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t rotr32(uint32_t x, uint32_t n) { return (x >> n) | (x << (32U - n)); }

static void compute_mac_sha8(const uint8_t mac[6], char out[9]) {
  static const uint32_t K[64] = {
    0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
    0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
    0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
    0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
    0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
    0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
    0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
    0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
  };
  uint32_t H[8] = {
    0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
    0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U
  };
  uint8_t blk[64];
  memset(blk, 0, sizeof(blk));
  memcpy(blk, mac, 6);
  blk[6] = 0x80U;
  blk[63] = 48U;
  uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = ((uint32_t)blk[4*i] << 24) | ((uint32_t)blk[4*i+1] << 16) |
           ((uint32_t)blk[4*i+2] << 8) | (uint32_t)blk[4*i+3];
  }
  for (int i = 16; i < 64; ++i) {
    uint32_t s0 = rotr32(w[i-15], 7) ^ rotr32(w[i-15], 18) ^ (w[i-15] >> 3);
    uint32_t s1 = rotr32(w[i-2], 17) ^ rotr32(w[i-2], 19) ^ (w[i-2] >> 10);
    w[i] = w[i-16] + s0 + w[i-7] + s1;
  }
  uint32_t a=H[0], b=H[1], c=H[2], d=H[3], e=H[4], f=H[5], g=H[6], h=H[7];
  for (int i = 0; i < 64; ++i) {
    uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
    uint32_t ch = (e & f) ^ ((~e) & g);
    uint32_t t1 = h + S1 + ch + K[i] + w[i];
    uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
    uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    uint32_t t2 = S0 + maj;
    h = g; g = f; f = e; e = d + t1;
    d = c; c = b; b = a; a = t1 + t2;
  }
  uint32_t h0 = H[0] + a;
  snprintf(out, 9, "%08x", (unsigned)h0);
}

#define CSI_RING_CAP 16
#define CSI_MAX_BYTES 128

struct CsiSlot {
  uint32_t t_us;
  int8_t rssi;
  int8_t noise_floor;
  uint8_t ch;
  uint16_t len;
  int8_t buf[CSI_MAX_BYTES];
};

static portMUX_TYPE g_csi_mux = portMUX_INITIALIZER_UNLOCKED;
static CsiSlot g_csi_ring[CSI_RING_CAP];
static volatile uint32_t g_csi_head = 0;
static volatile uint32_t g_csi_tail = 0;
static volatile bool g_csi_armed = false;

static void IRAM_ATTR on_csi_rx(void *ctx, wifi_csi_info_t *info) {
  (void)ctx;
  if (!g_csi_armed || info == NULL || info->buf == NULL || info->len <= 0) return;
  uint16_t n = (uint16_t)((info->len > CSI_MAX_BYTES) ? CSI_MAX_BYTES : info->len);
  portENTER_CRITICAL_ISR(&g_csi_mux);
  uint32_t next = (g_csi_head + 1U) % CSI_RING_CAP;
  if (next != g_csi_tail) {
    CsiSlot &s = g_csi_ring[g_csi_head];
    s.t_us = (uint32_t)esp_timer_get_time();
    s.rssi = info->rx_ctrl.rssi;
    s.noise_floor = info->rx_ctrl.noise_floor;
    s.ch = info->rx_ctrl.channel;
    s.len = n;
    memcpy(s.buf, info->buf, n);
    g_csi_head = next;
  }
  portEXIT_CRITICAL_ISR(&g_csi_mux);
}

static bool pop_csi_slot(void *out_ptr) {
  CsiSlot *out = (CsiSlot *)out_ptr;
  bool ok = false;
  portENTER_CRITICAL(&g_csi_mux);
  if (g_csi_tail != g_csi_head) {
    *out = g_csi_ring[g_csi_tail];
    g_csi_tail = (g_csi_tail + 1U) % CSI_RING_CAP;
    ok = true;
  }
  portEXIT_CRITICAL(&g_csi_mux);
  return ok;
}

static uint32_t read_rail_mv(void) {
  /* Cardputer-Adv battery divider is on GPIO10 (2:1 divider -> x2) */
  uint32_t sum = 0;
  for (int i = 0; i < 4; ++i) {
    sum += (uint32_t)analogReadMilliVolts(10);
  }
  return (sum * 2U) / 4U;
}

static void handle_ping(void) {
  esp_chip_info_t ci;
  esp_chip_info(&ci);
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char sha8[9] = {0};
  compute_mac_sha8(mac, sha8);
  Serial.printf("PONG,ESP32-S3,rev=%d,cpu_mhz=%lu,mac_sha8=%s\n",
                (int)ci.revision, (unsigned long)getCpuFrequencyMhz(), sha8);
}

static void handle_exp_csi(void) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  delay(80);

  int n_ap = WiFi.scanNetworks(false, false, false, 120);
  /* Aggregate unique active channels sorted by strongest RSSI (never log raw SSID/BSSID) */
  int cand_ch[14];
  int cand_rssi[14];
  int n_cand = 0;
  for (int i = 0; i < n_ap; ++i) {
    int ch = WiFi.channel(i);
    int rssi = WiFi.RSSI(i);
    if (ch < 1 || ch > 13) continue;
    int freq = 2407 + 5 * ch;
    Serial.printf("SCAN,%d,%d,%d,%d\n", i, ch, freq, rssi);
    int found = -1;
    for (int k = 0; k < n_cand; ++k) {
      if (cand_ch[k] == ch) { found = k; break; }
    }
    if (found < 0 && n_cand < 14) {
      cand_ch[n_cand] = ch;
      cand_rssi[n_cand] = rssi;
      ++n_cand;
    } else if (found >= 0 && rssi > cand_rssi[found]) {
      cand_rssi[found] = rssi;
    }
  }
  WiFi.scanDelete();
  for (int i = 0; i < n_cand; ++i) {
    for (int j = i + 1; j < n_cand; ++j) {
      if (cand_rssi[j] > cand_rssi[i]) {
        int tc = cand_ch[i]; cand_ch[i] = cand_ch[j]; cand_ch[j] = tc;
        int tr = cand_rssi[i]; cand_rssi[i] = cand_rssi[j]; cand_rssi[j] = tr;
      }
    }
  }
  if (n_cand == 0) {
    cand_ch[0] = 1; cand_ch[1] = 6; cand_ch[2] = 11;
    n_cand = 3;
  }

  wifi_csi_config_t cfg = {};
  cfg.lltf_en = true;
  cfg.htltf_en = true;
  cfg.stbc_htltf2_en = true;
  cfg.ltf_merge_en = true;
  cfg.channel_filter_en = false;
  cfg.manu_scale = false;
  cfg.shift = 0;

  portENTER_CRITICAL(&g_csi_mux);
  g_csi_head = 0;
  g_csi_tail = 0;
  g_csi_armed = true;
  portEXIT_CRITICAL(&g_csi_mux);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_csi_rx_cb(&on_csi_rx, NULL);
  esp_wifi_set_csi_config(&cfg);
  esp_wifi_set_csi(true);

  int csi_count = 0;
  int ch_idx = 0;
  esp_wifi_set_channel((uint8_t)cand_ch[0], WIFI_SECOND_CHAN_NONE);
  uint32_t t_start = millis();
  uint32_t t_hop = t_start;
  CsiSlot slot;

  while (csi_count < 40 && (millis() - t_start) < 6000UL) {
    while (pop_csi_slot(&slot) && csi_count < 40) {
      Serial.printf("CSI,%d,%lu,%u,%d,%d,%u",
                    csi_count, (unsigned long)slot.t_us, (unsigned)slot.ch,
                    (int)slot.rssi, (int)slot.noise_floor, (unsigned)slot.len);
      for (uint16_t k = 0; k < slot.len; ++k) {
        Serial.printf(",%d", (int)slot.buf[k]);
      }
      Serial.print('\n');
      ++csi_count;
    }
    if ((millis() - t_hop) > 2500UL && csi_count < 15 && ch_idx + 1 < n_cand) {
      ++ch_idx;
      esp_wifi_set_channel((uint8_t)cand_ch[ch_idx], WIFI_SECOND_CHAN_NONE);
      t_hop = millis();
    }
    delay(5);
  }

  g_csi_armed = false;
  esp_wifi_set_csi(false);
  esp_wifi_set_promiscuous(false);
  WiFi.mode(WIFI_OFF);
  Serial.println("END_CSI");
}

static void burn_load_ms(uint32_t ms) {
  uint32_t t0 = millis();
  volatile float acc = 1.0001f;
  while ((millis() - t0) < ms) {
    for (int i = 0; i < 4000; ++i) {
      acc = acc * 1.00001f + sinf((float)i * 0.01f);
      if (acc > 1000.0f) acc = 1.0f;
    }
  }
}

static void handle_exp_thermal(void) {
  WiFi.mode(WIFI_OFF);
  delay(100);
  uint32_t t0_ms = millis();
  const int n_idle = 40;   /* 4 s */
  const int n_load = 100;  /* 10 s */
  const int n_cool = 80;   /* 8 s */
  int total = n_idle + n_load + n_cool;

  for (int k = 0; k < total; ++k) {
    uint32_t target_ms = (uint32_t)k * 100UL;
    int load_on = (k >= n_idle && k < n_idle + n_load) ? 1 : 0;

    if (k == n_idle) {
      WiFi.mode(WIFI_STA);
      esp_wifi_set_ps(WIFI_PS_NONE);
    } else if (k == n_idle + n_load) {
      WiFi.scanDelete();
      WiFi.mode(WIFI_OFF);
    }

    if (load_on) {
      if ((k % 10) == 0) {
        WiFi.scanNetworks(true, false, false, 60);
      }
      burn_load_ms(75);
    }
    while ((millis() - t0_ms) < target_ms) {
      delay(2);
    }
    float t_die = temperatureRead();
    uint32_t v_mv = read_rail_mv();
    float t_s = 0.001f * (float)(millis() - t0_ms);
    Serial.printf("THERM,%d,%.3f,%lu,%.3f,%d\n",
                  k, (double)t_s, (unsigned long)v_mv, (double)t_die, load_on);
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  Serial.println("END_THERMAL");
}

static void handle_exp_gpio_rc(void) {
  /* 1. 200-sample calibrated 12-bit ADC noise floor on GPIO10 */
  for (int i = 0; i < 200; ++i) {
    uint32_t t_us = (uint32_t)micros();
    uint32_t mv = (uint32_t)analogReadMilliVolts(10) * 2U;
    Serial.printf("ADC,%d,%lu,%lu\n", i, (unsigned long)t_us, (unsigned long)mv);
    delayMicroseconds(50);
  }

  /* 2. Real Grove GPIO1 pull-up RC charge step observation */
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  delay(2);
  pinMode(1, INPUT_PULLUP);
  for (int i = 0; i < 40; ++i) {
    uint32_t t_us = (uint32_t)micros();
    uint32_t mv = (uint32_t)analogReadMilliVolts(1);
    Serial.printf("RC,%d,%lu,%lu\n", i, (unsigned long)t_us, (unsigned long)mv);
  }

  /* 3. Probe optional Grove external I2C bus (GPIO2=SDA, GPIO1=SCL) without touching internal SPI/kbd */
  Wire1.begin(2, 1, 100000);
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.printf("I2C,grove,0x%02x\n", (unsigned)addr);
    }
  }
  Wire1.end();
  Serial.println("END_GPIO_RC");
}

static void handle_exp_power_modes(void) {
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(80);
  delay(80);

  uint32_t t0_ms = millis();
  const int total = 240; /* 12 s at 50 ms cadence */
  for (int k = 0; k < total; ++k) {
    uint32_t target_ms = (uint32_t)k * 50UL;
    const char *mode_name = "80M_IDLE";
    if (k == 40) {
      setCpuFrequencyMhz(240);
    } else if (k == 140) {
      WiFi.mode(WIFI_STA);
      esp_wifi_set_ps(WIFI_PS_NONE);
    } else if (k == 200) {
      WiFi.scanDelete();
      WiFi.mode(WIFI_OFF);
      setCpuFrequencyMhz(80);
    }

    if (k < 40) {
      mode_name = "80M_IDLE";
    } else if (k < 80) {
      mode_name = "240M_IDLE";
    } else if (k < 140) {
      mode_name = "240M_FPU";
      burn_load_ms(38);
    } else if (k < 200) {
      mode_name = "240M_WIFI";
      if ((k % 12) == 8) {
        WiFi.scanNetworks(true, false, false, 60);
      }
      burn_load_ms(35);
    } else {
      mode_name = "80M_RECOV";
    }

    while ((millis() - t0_ms) < target_ms) {
      delay(2);
    }
    uint32_t v_mv = (read_rail_mv() + read_rail_mv()) / 2U;
    float t_die = temperatureRead();
    uint32_t t_now = (uint32_t)(millis() - t0_ms);
    uint32_t cur_mhz = (uint32_t)getCpuFrequencyMhz();
    Serial.printf("PWR,%d,%lu,%s,%lu,%lu,%.3f\n",
                  k, (unsigned long)t_now, mode_name,
                  (unsigned long)cur_mhz, (unsigned long)v_mv, (double)t_die);
  }
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(240);
  delay(20);
  Serial.println("END_POWER_MODES");
}

void setup(void) {
  Serial.begin(115200);
  delay(200);
  analogReadResolution(12);
  WiFi.mode(WIFI_OFF);
  Serial.println("READY,CARDPUTER_ELE_LAB");
}

static char g_cmd_buf[64];
static size_t g_cmd_len = 0;

void loop(void) {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r' || ch == '\n') {
      if (g_cmd_len > 0) {
        g_cmd_buf[g_cmd_len] = '\0';
        if (strcmp(g_cmd_buf, "PING") == 0) handle_ping();
        else if (strcmp(g_cmd_buf, "EXP_CSI") == 0) handle_exp_csi();
        else if (strcmp(g_cmd_buf, "EXP_THERMAL") == 0) handle_exp_thermal();
        else if (strcmp(g_cmd_buf, "EXP_GPIO_RC") == 0) handle_exp_gpio_rc();
        else if (strcmp(g_cmd_buf, "EXP_POWER_MODES") == 0) handle_exp_power_modes();
        else Serial.printf("ERR,UNKNOWN,%s\n", g_cmd_buf);
        g_cmd_len = 0;
      }
    } else if (g_cmd_len + 1 < sizeof(g_cmd_buf)) {
      g_cmd_buf[g_cmd_len++] = (char)ch;
    }
  }
  delay(10);
}
