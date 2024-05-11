#include "boards/pico_w.h"
#include "wifi_credentials.h"
#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_ll.h"
#include "dht.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/rtc.h"
#include "hardware/sync.h"
#include "lwip/arch.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/tcpbase.h"
#include "pico/cyw43_arch.h"
#include "pico/error.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/types.h"
#include "stdarg.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void blink_error_code(int error_code) {
  do {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    if (error_code & 1) {
      sleep_ms(1000);
    } else {
      sleep_ms(500);
    }

    error_code >>= 1;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    if (error_code > 0) {
      sleep_ms(300);
    }
  } while (error_code > 0);
}

void signal_error(int error_code, char *format, ...) {
  va_list l;
  va_start(l, format);
  for (int i = 0; i < 3; i++) {
    va_list args;
    va_copy(args, l);
    vprintf(format, args);
    va_end(args);
    blink_error_code(error_code);
    sleep_ms(1000);
  }
  va_end(l);
}

void signal_permanent_error(int error_code, char *format, ...) {
  va_list l;
  va_start(l, format);
  for (;;) {
    va_list args;
    va_copy(args, l);
    vprintf(format, args);
    va_end(args);
    blink_error_code(error_code);
    sleep_ms(1000);
  }
  va_end(l);
}

void wifi() {
  for (int i = 0; i < 3; i++) {
    int err = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 12000);
    if (err != PICO_OK) {
      signal_error(12, "WiFi Verbindungsfehler: %d\n", err);
      sleep_ms(1000);
      continue;
    }

    break;
  }
}

#define RTC_INIT_BUFFER_LEN 6

#define TRANSMISSION_COUNT 0
#define TRANSMISSION_PRE_FLASH 1
#define TRANSMISSION_FLASH_PAGE 2

struct tcp_server_state {
  struct tcp_pcb *listen_pcb;
  struct tcp_pcb *client_pcb;

  uint8_t rtc_init_buffer[RTC_INIT_BUFFER_LEN];
  int rtc_init_buffer_read_count;

  int unacknowledged_bytes_left;
  int last_transmission;
};

err_t disconnect_client(struct tcp_server_state *state) {
  err_t return_err = ERR_OK;

  tcp_arg(state->client_pcb, NULL);
  tcp_err(state->client_pcb, NULL);
  tcp_recv(state->client_pcb, NULL);
  tcp_sent(state->client_pcb, NULL);

  if (tcp_close(state->client_pcb)) {
    tcp_abort(state->client_pcb);
    return_err = ERR_ABRT;
  }

  state->client_pcb = NULL;

  return return_err;
}

char *delegated_error_msg = NULL;
int delegated_error_code = 0;

void delegate_signal_error(int error_code, char *format, ...) {
  va_list args, args2;

  va_start(args, format);

  va_copy(args2, args);

  size_t needed = vsnprintf(NULL, 0, format, args2) + 1;
  va_end(args2);

  if (delegated_error_msg) {
    free(delegated_error_msg);
  }

  delegated_error_msg = malloc(needed);

  if (!delegated_error_msg) {
    signal_permanent_error(10, "Speicher für delegierte Fehlermeldung konnte "
                               "nicht reserviert werden\n");
  }

  vsnprintf(delegated_error_msg, needed, format, args);
  va_end(args);
  delegated_error_code = error_code;
}

err_t write_client(struct tcp_server_state *state, void *data, int count,
                   int transmission_type) {
  err_t err = tcp_write(state->client_pcb, data, count, TCP_WRITE_FLAG_COPY);

  if (err) {
    delegate_signal_error(11, "TCP Schreibfehler: %d\n", err);

    return disconnect_client(state);
  }

  state->unacknowledged_bytes_left += count;
  state->last_transmission = transmission_type;

  return ERR_OK;
}

uint64_t compute_packed_measurement(int temp, int humidity, uint64_t *result) {
  datetime_t current;
  if (!rtc_get_datetime(&current)) {
    signal_permanent_error(15, "Keine Echtzeit trotz Initialisierung\n");
  }

  return ((uint64_t)current.sec)                 // 6 bit
         | (((uint64_t)current.min) << 6)        // 6 bit
         | ((uint64_t)current.hour) << 12        // 5 bit
         | (((uint64_t)current.day - 1) << 17)   // 5 bit
         | (((uint64_t)current.month - 1) << 22) // 4 bit
         | (((uint64_t)current.year) << 26)      // 16 bit
         | (((uint64_t)temp) << 42)              // 9 bit
         | (((uint64_t)humidity) << 51)          // 10 bit
                                                 // 61 / 64 bits used
      ;
}

// 64 bit and 8 bit per byte; total amount of measurements that can be stored in
// a flash page
#define MEASUREMENTS_PER_FLASH_PAGE (FLASH_PAGE_SIZE / (64 / 8))
#define FLASH_SECTOR_COUNT (PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE)
#define FLASH_PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)

uint64_t pre_flash_measurements[MEASUREMENTS_PER_FLASH_PAGE];
unsigned pre_flash_measurement_count = 0;

unsigned ring_page_count; // how many flash pages in the ring?
unsigned first_page_in_ring;
unsigned read_page_index;
unsigned write_page_index; // the next flash page to be written to

extern char __flash_binary_end;

void setup_flash_ring(void) {
  int first_sector_in_ring =
      (((uint32_t)&__flash_binary_end) - XIP_BASE + FLASH_SECTOR_SIZE - 1) /
          FLASH_SECTOR_SIZE +
      1;

  unsigned ring_sector_count = FLASH_SECTOR_COUNT - first_sector_in_ring;
  ring_page_count = ring_sector_count * FLASH_PAGES_PER_SECTOR;

  read_page_index = write_page_index =
      (get_rand_32() % ring_sector_count) * FLASH_PAGES_PER_SECTOR;
}

void flush_pre_flash_measurements_to_flash(void) {
  uint32_t ints = save_and_disable_interrupts();
  if (write_page_index % FLASH_PAGES_PER_SECTOR == 0) {
    flash_range_erase((first_page_in_ring + write_page_index) * FLASH_PAGE_SIZE,
                      FLASH_SECTOR_SIZE);
  }

  flash_range_program((first_page_in_ring + write_page_index) * FLASH_PAGE_SIZE,
                      (const uint8_t *)&pre_flash_measurements,
                      sizeof pre_flash_measurements);

  pre_flash_measurement_count = 0;
  write_page_index = (write_page_index + 1) % ring_page_count;

  restore_interrupts(ints);
}

void measure(dht_t *dht) {
  float temp, humidity;

  for (int fail_count = 1;; fail_count++) {
    dht_start_measurement(dht);

    dht_result_t result =
        dht_finish_measurement_blocking(dht, &humidity, &temp);

    if (result == DHT_RESULT_OK) {
      break;
    }

    if (fail_count == 3) {
      signal_error(14, "Lesen des Sensors nach drei Fehlversuchen fehlgeschlagen: %d\n", result);
      return;
    }

    sleep_ms(1000);
  }

  uint64_t packed = compute_packed_measurement(
      (int)(temp * 10.0f), (int)(humidity * 10.0f), &packed);

  if (pre_flash_measurement_count == MEASUREMENTS_PER_FLASH_PAGE) {
    flush_pre_flash_measurements_to_flash();
  }

  pre_flash_measurements[pre_flash_measurement_count] = packed;
  pre_flash_measurement_count++;
}

err_t on_tcp_sent(void *arg, struct tcp_pcb *client, u16_t send_count) {
  struct tcp_server_state *state = arg;

  state->unacknowledged_bytes_left -= send_count;

  if (state->unacknowledged_bytes_left) {
    return ERR_OK;
  }

  switch (state->last_transmission) {
  case TRANSMISSION_FLASH_PAGE:
    read_page_index = (read_page_index + 1) % ring_page_count;
  case TRANSMISSION_COUNT:
    if (pre_flash_measurement_count > 0) {
      return write_client(state, (void *)&pre_flash_measurements,
                          pre_flash_measurement_count * 8,
                          TRANSMISSION_PRE_FLASH);
    }
  case TRANSMISSION_PRE_FLASH:
    pre_flash_measurement_count = 0;

    if (read_page_index == write_page_index) {
      return disconnect_client(state);
    }

    return write_client(
        state,
        (void *)(XIP_BASE +
                 (first_page_in_ring + read_page_index) * FLASH_PAGE_SIZE),
        FLASH_PAGE_SIZE, TRANSMISSION_FLASH_PAGE);
  }
}

bool rtc_initialized = false;

err_t on_tcp_recv(void *arg, struct tcp_pcb *client, struct pbuf *data,
                  err_t err) {
  struct tcp_server_state *state = arg;

  if (err) {
    delegate_signal_error(9, "TCP Empfangsfehler: %d\n", err);

    return disconnect_client(state);
  }

  if (!data) {
    state->client_pcb = NULL;
    return ERR_OK;
  }

  if (data->tot_len > 0) {
    const uint16_t buffer_left =
        RTC_INIT_BUFFER_LEN - state->rtc_init_buffer_read_count;
    state->rtc_init_buffer_read_count += pbuf_copy_partial(
        data, state->rtc_init_buffer + state->rtc_init_buffer_read_count,
        data->tot_len > buffer_left ? buffer_left : data->tot_len, 0);
    tcp_recved(client, data->tot_len);
  }
  pbuf_free(data);

  if (state->rtc_init_buffer_read_count < RTC_INIT_BUFFER_LEN) {
    return ERR_OK;
  }

  datetime_t time;
  time.sec = state->rtc_init_buffer[0] & 0b111111; // 6 bit from first byte
  time.min = state->rtc_init_buffer[0] >> 6 |      // 2 bit from first byte
             (state->rtc_init_buffer[1] & 0b1111)
                 << 2; // and 4 bit from second byte
  time.hour =
      (state->rtc_init_buffer[1] >> 4) & 0b1111 | // 4 bit from second byte
      (state->rtc_init_buffer[2] & 0b1) << 4;     // 1 bit from third
  time.dotw = (state->rtc_init_buffer[2] >> 1) & 0b111;
  time.day = (state->rtc_init_buffer[2] >> 4 | (state->rtc_init_buffer[3] & 0b1)
                                                   << 4) +
             1;
  time.month = (state->rtc_init_buffer[3] >> 1 & 0b1111) + 1;
  time.year = ((uint16_t)state->rtc_init_buffer[3]) >> 5 |
              ((uint16_t)state->rtc_init_buffer[4]) << 3 |
              ((uint16_t)state->rtc_init_buffer[5] & 0b11111) << 11;

  if (!rtc_set_datetime(&time)) {
    signal_permanent_error(10, "Ungültiges Datum erhalten\n");
  }

  sleep_us(64); // wait for the rtc write to complete

  rtc_initialized = true;

  uint32_t measurement_count =
      pre_flash_measurement_count +
      ((ring_page_count + write_page_index - read_page_index) % ring_page_count) * MEASUREMENTS_PER_FLASH_PAGE;

  return write_client(state, &measurement_count, 4, TRANSMISSION_COUNT);
}

void on_tcp_err(void *arg, err_t err) {
  // we need to ignore abort errors; no idea why...
  if (err == ERR_ABRT) {
    return;
  }

  struct tcp_server_state *state = arg;

  delegate_signal_error(8, "TCP Client Fehler: %d\n", err);

  disconnect_client(state);
}

err_t on_tcp_accept(void *arg, struct tcp_pcb *client, err_t err) {
  if (err) {
    delegate_signal_error(5, "TCP accept fehlgeschlagen: %d\n", err);
    return err;
  }

  if (!client) {
    delegate_signal_error(6, "TCP accept fehlgeschlagen; client war NULL\n");
    return ERR_OK;
  }

  struct tcp_server_state *state = arg;

  if (state->client_pcb) {
    delegate_signal_error(7, "TCP accept; client bereits verbunden\n");

    if (tcp_close(client)) {
      tcp_abort(client);
      return ERR_ABRT;
    }

    return ERR_OK;
  }

  state->client_pcb = client;
  state->rtc_init_buffer_read_count = 0;

  tcp_arg(state->client_pcb, state);
  tcp_err(state->client_pcb, on_tcp_err);
  tcp_recv(state->client_pcb, on_tcp_recv);
  tcp_sent(state->client_pcb, on_tcp_sent);

  return ERR_OK;
}

#define TCP_PORT 60438

void setup_tcp_server(void) {
  struct tcp_server_state *server_data =
      calloc(1, sizeof(struct tcp_server_state));

  if (!server_data) {
    signal_permanent_error(1,
                           "TCP Serverdaten konnten nicht allokiert werden\n");
  }

  server_data->listen_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);

  if (!server_data->listen_pcb) {
    signal_permanent_error(2, "TCP PCB konnte nicht allokiert werden\n");
  }

  err_t err = tcp_bind(server_data->listen_pcb, NULL, TCP_PORT);

  if (err) {
    signal_permanent_error(3, "TCP Portbindung fehlgeschlagen: %d\n", err);
  }

  server_data->listen_pcb =
      tcp_listen_with_backlog_and_err(server_data->listen_pcb, 1, &err);

  if (!server_data->listen_pcb) {
    signal_permanent_error(4, "TCP Listen fehlgeschlagen: %d\n", err);
  }

  tcp_arg(server_data->listen_pcb, server_data);
  tcp_accept(server_data->listen_pcb, on_tcp_accept);
}

int main(void) {
  stdio_init_all();

  int err = cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY);
  if (err) {
    signal_permanent_error(0, "WiFi Initialisierung fehlgeschlagen: %d\n", err);
  }

  cyw43_arch_enable_sta_mode();

  rtc_init();
  setup_tcp_server();
  setup_flash_ring();

  while (!rtc_initialized) {
    wifi();
    sleep_ms(1000);
    printf("Warten auf RTC Initialisierung\n");
    blink_error_code(13);
    sleep_ms(1000);
  }

  dht_t dht;
  dht_init(&dht, DHT11, pio0, 18, true);

  absolute_time_t next_measurement = get_absolute_time();

  for (;;) {
    next_measurement = delayed_by_ms(next_measurement, 60 * 1000);

    measure(&dht);

    wifi();

    while (get_absolute_time() < next_measurement) {
      if (delegated_error_msg) {
        char *msg = delegated_error_msg;
        int error_code = delegated_error_code;
        delegated_error_msg = NULL;

        signal_error(error_code, "%s", delegated_error_msg);

        free(delegated_error_msg);
      }

      sleep_until(
          MIN(delayed_by_ms(get_absolute_time(), 1000), next_measurement));
    }
  }
}
