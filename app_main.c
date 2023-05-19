/* ZDGG (bire GPC) Rknzcyr

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "FreeRTOSConfig.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "nvs.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "mqtt_client.h"
#include "portmacro.h"
#include "projdefs.h"

#include <ds18x20.h>

#define TIMER_WAKEUP_TIME_US 900000000

// My prototypes
void send_temperature();
void mqtt_connect();
void mqtt_disconnect();
void sleep_mode();

esp_mqtt_client_handle_t client;

static const gpio_num_t SENSOR_GPIO = CONFIG_EXAMPLE_ONEWIRE_GPIO;

// Use address of your own sensor here!
// You can find out the address of your sensor by running ds18x20_multi example
static const ds18x20_addr_t SENSOR_ADDR = 0xdf3c43e38143f628;

static const char *TAG3 = "ds18x20_test";

// Global Variables
char influx_line[100]; // array to hold influx line protocol
uint8_t int_temp_upper;
uint8_t int_temp_lower;

/* ======================================================================= */
/* This is the station getting started from examples just copied over */
/* ======================================================================= */

/* The examples use WiFi configuration that you can set via project
   configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG2 = "wifi station";

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG2, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG2, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG2, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta = {.ssid = EXAMPLE_ESP_WIFI_SSID, .password = EXAMPLE_ESP_WIFI_PASS},
  };

  /* Setting a password implies station will connect to all security modes
   * including WEP/WPA. However these modes are deprecated and not advisable to
   * be used. Incase your Access point doesn't support WPA2, these mode can be
   * enabled by commenting below line */

  if (strlen((char *)wifi_config.sta.password)) {
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG2, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG2, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG2, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else {
    ESP_LOGE(TAG2, "UNEXPECTED EVENT");
  }

  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &event_handler));
  vEventGroupDelete(s_wifi_event_group);
}

/* END OF WIFI STATION EXAMPLE  */

/* ======================================================================= */
/* This is the mqtt example                                                */
/* ======================================================================= */

static const char *TAG = "MQTT_EXAMPLE";

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  // your_context_t *context = event->context;
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED: // Client successfully established a connection to
                             // a broker
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    break;
  case MQTT_EVENT_DISCONNECTED: // Client has aborted the connection due to
                                // being unable to read or write data becuase
                                // the server is unavailable
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_PUBLISHED:
    msg_id = esp_mqtt_client_publish(client, "/topic/temp_home", influx_line, 0,
                                     0, 0);
    break;
    /* This handles if wifi disconnects, the MQTT will stay connected and keep
     * trying to send but fail. If this hapeens we need to re-initailize the
     * wifi*/
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    printf("NEED TO RESTART WIFI");
    wifi_init_sta();
    mqtt_connect();
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base,
           event_id);
  mqtt_event_handler_cb(event_data);
}

// Function that uses light sleep
void sleep_mode() {

  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_TIME_US));
  ESP_ERROR_CHECK(esp_light_sleep_start());
  printf("WAKEUP!\n");
}

/* This is the app that will puslish the mqtt data to broker  */

void mqtt_connect(void) {

  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 client);

  ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

void mqtt_disconnect(void) {

  ESP_ERROR_CHECK(esp_mqtt_client_disconnect(client));
  ESP_ERROR_CHECK(esp_mqtt_client_stop(client));
}
static void mqtt_app_start(void) {

  // Make sure that the internal pull-up resistor is enabled on the GPIO pin
  // so that one can connect up a sensor without needing an external pull-up.
  // (Note: The internal (~47k) pull-ups of the ESP do appear to work, at
  // least for simple setups (one or two sensors connected with short leads),
  // but do not technically meet the pull-up requirements from the ds18x20
  // datasheet and may not always be reliable. For a real application, a
  // proper 4.7k external pull-up resistor is recommended instead!)
  gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);

  float temperature;
  float faren;
  esp_err_t res;
  res = ds18x20_measure_and_read(SENSOR_GPIO, SENSOR_ADDR, &temperature);
  faren = ((temperature * 1.8) + 32);
  // There are no printfs for floats to save space
  int_temp_upper = (int)faren;
  int_temp_lower = (faren * 100) - (((int)faren) * 100);

  if (res != ESP_OK)
    ESP_LOGE(TAG3,
             "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
             (uint32_t)(SENSOR_ADDR >> 32), (uint32_t)SENSOR_ADDR, res,
             esp_err_to_name(res));
  else
    sprintf(influx_line, "Temperature,location=mudroom temperature=%i.%i",
            int_temp_upper, int_temp_lower);

  vTaskDelay(pdMS_TO_TICKS(1000));

  esp_mqtt_client_publish(client, "/topic/home_temp", influx_line, 0, 0, 0);

  vTaskDelay(pdMS_TO_TICKS(5) / portTICK_PERIOD_MS);
}

void app_main(void) {
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());

  /* This helper function configures Wi-Fi or Ethernet, as selected in
   * menuconfig. Read "Establishing Wi-Fi or Ethernet Connection" section in
   * examples/protocols/README.md for more information about this function.
   */
  ESP_LOGI(TAG2, "ESP_WIFI_MODE_STA");
  wifi_init_sta();

  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = CONFIG_BROKER_URL,
  };

  client = esp_mqtt_client_init(&mqtt_cfg);

  mqtt_connect();

  while (1) {
    mqtt_app_start();
    mqtt_disconnect();
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());

    vTaskDelay(pdMS_TO_TICKS(100));

    sleep_mode();
    esp_restart();

    // Unable to get light sleep to work with restarting wifi. Keep insace ever
    // do.
    /* printf("START wifi\n"); */
    /* ESP_ERROR_CHECK(esp_event_loop_delete_default()); */
    /* wifi_init_sta(); */
    /* mqtt_connect(); */
  }
}
