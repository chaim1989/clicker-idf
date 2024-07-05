
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// #include <esp_wifi.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#define LCD_H_RES 128
#define LCD_V_RES 64
#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_HOST_ID 0
#define I2C_DEV_ADDR 0x3C
#define DISP_BUF_SIZE (128 * 64 / 8)
#define BUZZER_PIN 47
#define VIBRATE_PIN 21
#define BAT_MON_PIN 1
#define BAT_CHG 17
#define BAT_STDBY 18
#define FIRMWARE_VERSION "1.22"
#define MIN_BATTERY 3.2
// #include "esp_lcd_panel_io_interface.h"
#include "pitches.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "led_strip_encoder.h"
#include "driver/rmt_tx.h"
#include "mdns.h"
// #include "mdns_private.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_RING_PIN 48
#define LED_NUMBERS 8
#define CHASE_SPEED_MS 100
static uint8_t led_strip_pixels[LED_NUMBERS * 3];

#include "driver/i2c.h"

// #include "esp_hw_support.h"
#include "esp_mac.h"
typedef enum
{
    LED_OFF = 0,
    LED_ANIM = 1,
    LED_COLOR = 2,
    LED_BLINK = 3

} led_modes;
#include "esp_websocket_client.h"
#include "esp_event.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

// #include "Arduino.h"
// #include "libraries/arduinoWebSockets/src/SocketIOclient.h"
// #include "lwip/err.h"
// #include "lwip/sys.h"
#include "Arduino.h"

#include <ArduinoJson.h>
// #include "SocketIOclient.h"
#include <esp32-hal-log.h>
#include "cJSON.h"
#include <esp_http_client.h>
#include "iot_button.h"
#include "Wire.h"
#define SSD1306_128_64

#include "Adafruit_SSD1306.h"
#include "Fonts/FreeSerifItalic24pt7b.h"
#include "funclick_logo.h"
#include <esp_ota_ops.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
// #include "../managed_components/espressif__mdns/private_include/mdns_private.h"

#define WIFI_SSID "FUNCLICK"
#define WIFI_PASS "funclick123!"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static const char *MDNS_TAG = "mdns";
static const char *LOOP_TAG = "loop function";
// SocketIOclient socketIO;
static char SocketServerIp[16];

static int s_retry_num = 0;

#define NO_DATA_TIMEOUT_SEC 5
#define POWER_PIN 37
#define BUTTON_1 15
#define BUTTON_2 5
#define BUTTON_3 13
#define BUTTON_4 4
#define BUTTON_5 6
#define BUTTON_6 11
#define BUTTON_7 16
#define BUTTON_1_LED 2
#define BUTTON_2_LED 10
#define BUTTON_3_LED 14
#define BUTTON_4_LED 3
#define BUTTON_5_LED 7
#define BUTTON_6_LED 12
#define BUTTONS_LED_R 34
#define BUTTONS_LED_G 35
#define BUTTONS_LED_B 36
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCL_PIN 9
#define SDA_PIN 8
Adafruit_SSD1306 display;
SemaphoreHandle_t xMutex;
rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
rmt_transmit_config_t tx_config = {
    .loop_count = 0,
};
bool ChargingFlag = false;
int ledStatus = LED_ANIM;
int ledValueR = 0;
int ledValueG = 0;
int ledValueB = 0;
int ledValueBlinkRate = 0;
int ledValueBlinkCounter = 0;
int ledValueBrightness = 0;
int player_number;
int score;
int rank;
int rssi;
int battery_voltage;
char *bottomText;
void printCenter(const char *buf, int y);
void printStatusBottom(const char *buf);
void sendMessage(void *pvParameters);
void sendStatus(void *pvParameters);
void beepTask(void *pvParameter);
void webSocketTask(void *pvParameters);
void ota_task(void *pvParameter);
void find_mdns_service(const char *service_name, const char *proto);
void led_strip_hsv2rgb(uint32_t h, uint32_t saturation, uint32_t value, uint32_t *r, uint32_t *g, uint32_t *b);
void safeFree(void **ptr)
{
    if (ptr != NULL && *ptr != NULL)
    {
        free(*ptr);
        *ptr = NULL; // Set the pointer to NULL after freeing it
    }
}
void *safeMalloc(size_t size)
{
    void *ptr = pvPortMalloc(size);
    if (ptr == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed!");
    }
    return ptr;
}
void beep(int note, int duration, int delayms)
{
    if (delayms > 0)
    {
        vTaskDelay(delayms / portTICK_PERIOD_MS);
    }
    // pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, note, duration);
    // ledcWriteTone(BUZZER_PIN, note);
}
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT)
    {
        switch (event_id)
        {
        case ESP_HTTPS_OTA_START:
            ESP_LOGI(TAG, "OTA started");
            printStatusBottom("UPDATE STARTED");
            break;
        case ESP_HTTPS_OTA_CONNECTED:
            ESP_LOGI(TAG, "Connected to server");
            break;
        case ESP_HTTPS_OTA_GET_IMG_DESC:
            ESP_LOGI(TAG, "Reading Image Description");
            break;
        case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
            ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
            break;
        case ESP_HTTPS_OTA_DECRYPT_CB:
            ESP_LOGI(TAG, "Callback to decrypt function");
            break;
        case ESP_HTTPS_OTA_WRITE_FLASH:
            ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
            break;
        case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
            ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
            break;
        case ESP_HTTPS_OTA_FINISH:
            ESP_LOGI(TAG, "OTA finish");
            break;
        case ESP_HTTPS_OTA_ABORT:
            ESP_LOGI(TAG, "OTA abort");
            printStatusBottom("UPDATE ABORT");
            break;
        }
    }
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        if (s_retry_num < 100)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            printStatusBottom("NTWRK FAIL SHTDWN");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            ESP_LOGW(TAG, "SHUTDOWN");
            beep(NOTE_A4, 80, 0);
            beep(NOTE_A, 80, 100);
            digitalWrite(POWER_PIN, LOW);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    esp_netif_t *m_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(m_netif, "FUNCLICK_CLICKER");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold{
                .authmode = WIFI_AUTH_WPA2_PSK,
            }},
    };
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    printStatusBottom("CONNECTING");

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);

        printStatusBottom("NTWRK CONNECTED");
    }
    else if (bits & WIFI_FAIL_BIT)
    {

        printStatusBottom("W CONNECT FAIL");
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
extern "C"
{
    void app_main();
}
// static TimerHandle_t shutdown_signal_timer;
// static SemaphoreHandle_t shutdown_sema;
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// static void shutdown_signaler(TimerHandle_t xTimer)
// {
//     ESP_LOGI(TAG, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
//     xSemaphoreGive(shutdown_sema);
// }
void turnOnBtnLeds()
{
    digitalWrite(BUTTON_1_LED, HIGH);
    digitalWrite(BUTTON_2_LED, HIGH);
    digitalWrite(BUTTON_3_LED, HIGH);
    digitalWrite(BUTTON_4_LED, HIGH);
    digitalWrite(BUTTON_5_LED, HIGH);
    digitalWrite(BUTTON_6_LED, HIGH);
}
void turnOffBtnLeds()
{
    digitalWrite(BUTTON_1_LED, LOW);
    digitalWrite(BUTTON_2_LED, LOW);
    digitalWrite(BUTTON_3_LED, LOW);
    digitalWrite(BUTTON_4_LED, LOW);
    digitalWrite(BUTTON_5_LED, LOW);
    digitalWrite(BUTTON_6_LED, LOW);
}
int stopVibrateMills;
int checkBatteryMillis;
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    // heap_caps_print_heap_info("Before websocket_event_handler");

    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
    {

        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        display.fillRect(0, 52, 128, 12, SSD1306_BLACK);
        display.fillRect(0, 0, 10, 10, SSD1306_BLACK);
        display.drawBitmap(0, 0, icon_connected, 7, 8, SSD1306_WHITE);
        display.display();

        xTaskCreate(sendStatus, "sendStatus", 4096, NULL, 4, NULL);
    }
    break;
    case WEBSOCKET_EVENT_DISCONNECTED:
    {
        printStatusBottom("SRVR DSCNCT");
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        log_error_if_nonzero("HTTP status code", data->error_handle.esp_ws_handshake_status_code);
        if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", data->error_handle.esp_transport_sock_errno);
        }
        display.fillRect(0, 0, 7, 8, SSD1306_WHITE);
        display.display();
    }
    break;
    case WEBSOCKET_EVENT_DATA:
    {
        // ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
        // ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
        if (data->op_code == 0x08 && data->data_len == 2)
        {
            ESP_LOGW(TAG, "Received closed message with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
        }
        else
        {
            // ESP_LOGW(TAG, "Received=%.*s\n\n", data->data_len, (char *)data->data_ptr);
        }

        // If received data contains json structure it succeed to parse
        cJSON *root = cJSON_Parse(data->data_ptr);
        if (root)
        {
            // for (int i = 0; i < cJSON_GetArraySize(root); i++)
            // {
            cJSON *event_name_object = cJSON_GetArrayItem(root, 0);
            char *event_name_p = cJSON_Print(event_name_object);
            if (event_name_p != nullptr)
            {
                std::string event_name_str(event_name_p);

                event_name_str = event_name_str.substr(1, event_name_str.length() - 2);
                const char *event_name = event_name_str.c_str();

                ESP_LOGW(TAG, "Recevied Event %s", event_name);

                if (std::string(event_name) == "led")
                {
                    ESP_LOGW(TAG, "processing led event");
                    // cJSON_free(event_name);
                    if (cJSON_GetArraySize(root) > 1)
                    {
                        cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                        int value = cJSON_GetObjectItem(event_data_object, "value")->valueint;
                        ESP_LOGW(TAG, "Recevide Event led value %i", value);

                        if (value)
                        {
                            int value_r = cJSON_GetObjectItem(event_data_object, "value_R")->valueint;
                            int value_g = cJSON_GetObjectItem(event_data_object, "value_G")->valueint;
                            int value_b = cJSON_GetObjectItem(event_data_object, "value_B")->valueint;
                            ledValueR = value_r;
                            ledValueG = value_g;
                            ledValueB = value_b;

                            int value_blink = cJSON_GetObjectItem(event_data_object, "value_blink")->valueint;
                            if (value_blink == 1)
                            {
                                ledStatus = LED_BLINK;
                            }
                            else
                            {
                                ledStatus = LED_COLOR;
                            }
                            // ledStatus = LED_COLOR;
                            int value_blink_rate = cJSON_GetObjectItem(event_data_object, "value_blink_rate")->valueint;
                            int value_blink_counter = cJSON_GetObjectItem(event_data_object, "value_blink_counter")->valueint;
                            int value_brightness = cJSON_GetObjectItem(event_data_object, "value_brightness")->valueint;
                            ledValueBlinkRate = value_blink_rate;
                            ledValueBlinkCounter = value_blink_counter;
                            ledValueBrightness = value_brightness;

                            analogWrite(BUTTONS_LED_R, 255 - value_r);
                            analogWrite(BUTTONS_LED_G, 255 - value_g);
                            analogWrite(BUTTONS_LED_B, 255 - value_b);

                            turnOnBtnLeds();
                        }
                        else
                        {
                            ledStatus = LED_OFF;
                            turnOffBtnLeds();
                        }
                        if (((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095) < MIN_BATTERY)
                        {
                            analogWrite(BUTTONS_LED_R, 255 - 255);
                            analogWrite(BUTTONS_LED_G, 255 - 0);
                            analogWrite(BUTTONS_LED_B, 255 - 0);
                            turnOnBtnLeds();
                        }
                        // cJSON_free(event_data);
                        // cJSON_free(event_data_object);
                    }
                }
                else if (std::string(event_name) == "player_info")
                {
                    if (!(checkBatteryMillis > 0))
                    {
                        ESP_LOGW(TAG, "processing player_info event");
                        cJSON *event_data_object = cJSON_GetArrayItem(root, 1);

                        cJSON *player_number_obj = cJSON_GetObjectItem(event_data_object, "number");

                        cJSON *rank_obj = cJSON_GetObjectItem(event_data_object, "rank");
                        cJSON *score_obj = cJSON_GetObjectItem(event_data_object, "score");
                        int player_number = 0;
                        if (player_number_obj != NULL)
                        {
                            player_number = player_number_obj->valueint;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "player_number_obj is null");
                        }
                        int rank = 0; //= cJSON_GetObjectItem(event_data_object, "rank")->valueint;
                        if (rank_obj != NULL)
                        {
                            rank = rank_obj->valueint;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "rank_obj is null");
                        }
                        int score = 0; //= cJSON_GetObjectItem(event_data_object, "rank")->valueint;
                        if (score_obj != NULL)
                        {
                            score = score_obj->valueint;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "score_obj is null");
                        }
                        // ESP_LOGI(TAG, "PLAYER NUMBER %i", player_number);
                        // ESP_LOGI(TAG, "PLAYER RANK %i", rank);
                        // ESP_LOGI(TAG, "PLAYER SCORE %i", score);

                        if (player_number > 0)
                        {
                            display.fillRect(24, 0, 80, 50, SSD1306_BLACK);
                            display.display();
                            // vTaskDelay(100 / portTICK_PERIOD_MS);
                            // beep(NOTE_A4, 100, 0);
                            display.setFont(&FreeSerifItalic24pt7b);
                            display.setTextSize(1);
                            ESP_LOGW(TAG, "player_number %d", player_number);
                            char player_number_str[11];
                            snprintf(player_number_str, sizeof(player_number_str), "%d", player_number);
                            ESP_LOGW(TAG, "player_number_str %s", player_number_str);
                            printCenter(player_number_str, 45);
                            display.display();
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                        }

                        display.fillRect(0, 50, 128, 14, SSD1306_BLACK);
                        display.drawLine(0, 50, 128, 50, WHITE);
                        display.drawLine(63, 50, 63, 64, WHITE);
                        display.setFont();
                        if (rank > 0)
                        {
                            display.setCursor(2, 55);
                            display.print("RANK:" + String(rank));
                        }
                        if (score > 0)
                        {
                            display.setCursor(66, 55);
                            display.print("SCORE:" + String(score));
                        }
                        if (score > 0 || rank > 0)
                        {
                            display.display();
                        }
                    }
                }
                else if (String(event_name) == "shutdown")
                {
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                    int shutdown_in = cJSON_GetObjectItem(event_data_object, "value")->valueint;
                    display.fillRect(0, 0, 128, 64, SSD1306_BLACK);
                    display.display();
                    printCenter("SHUTDOWN", 20);
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    ESP_LOGW(TAG, "SHUTDOWN");
                    beep(NOTE_A4, 80, 0);
                    beep(NOTE_A, 80, 100);
                    digitalWrite(POWER_PIN, LOW);

                    // xTimerReset(shutdown_signal_timer, portMAX_DELAY);
                }
                else if (std::string(event_name) == "vibrate")
                {
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                    int value = cJSON_GetObjectItem(event_data_object, "value")->valueint;
                    digitalWrite(VIBRATE_PIN, HIGH);
                    stopVibrateMills = millis() + value;
                }
                else if (std::string(event_name) == "check_battery")
                {
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                    int value = cJSON_GetObjectItem(event_data_object, "value")->valueint;

                    checkBatteryMillis = millis() + (value * 1000);
                    // ESP_LOGI(TAG, "checkBatteryMillis %i", checkBatteryMillis);
                }
                else if (std::string(event_name) == "tone")
                {
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);

                    int duration = cJSON_GetObjectItem(event_data_object, "duration")->valueint;
                    double value = cJSON_GetObjectItem(event_data_object, "value")->valuedouble;
                    beep(value, duration, 0);
                }
                // cJSON *action = cJSON_GetObjectItem(elem, "action");
                // cJSON *value = cJSON_GetObjectItem(elem, "value");
                // ESP_LOGW(TAG, "Json={'action': '%s', 'value': '%s'}", action->valuestring, value->valuestring);
                // }
                cJSON_free(event_name_p);
            }
            cJSON_Delete(root);
        }

        // ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

        // xTimerReset(shutdown_signal_timer, portMAX_DELAY);
    }
    break;

    case WEBSOCKET_EVENT_ERROR:
    {
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
        log_error_if_nonzero("HTTP status code", data->error_handle.esp_ws_handshake_status_code);
        if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", data->error_handle.esp_transport_sock_errno);
        }
    }
    break;
    }
    // heap_caps_print_heap_info("After Task Allocation");
}
void macToString(const uint8_t *mac, char *str)
{
    snprintf(str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
String uniqueID;
esp_websocket_client_handle_t client;
static void websocket_app_start(void)
{
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    char macStr[18]; // 6 bytes * 2 characters per byte + 5 colons + 1 null terminator

    macToString(mac, macStr);
    uniqueID = String(macStr);
    esp_websocket_client_config_t websocket_cfg = {
        // .ping_interval_sec = 2,
        // .ping_interval_sec = 3,
        .task_prio = 6,
        .keep_alive_enable = true,
        .keep_alive_interval = 2,
        .reconnect_timeout_ms = 1000,
        .network_timeout_ms = 3000};

    // websocket_cfg.headers = ("hw_id:" + uniqueID + "\r\nversion:1.0.1\r\n").c_str();
    // ESP_LOGI(TAG, "hw_id set to %s", websocket_cfg.headers);
    // ESP_LOGI(TAG, "websocket_cfg created");
    // shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS,
    //                                      pdFALSE, NULL, shutdown_signaler);
    // shutdown_sema = xSemaphoreCreateBinary();

    websocket_cfg.transport = WEBSOCKET_TRANSPORT_OVER_TCP;
    //   websocket_cfg.uri = "ws://192.168.0.2";
    ESP_LOGI(TAG, "websocket_cfg set transport");
    websocket_cfg.host = SocketServerIp;
    // websocket_cfg.host = "192.168.1.193";
    ESP_LOGI(TAG, "websocket_cfg set host %s", websocket_cfg.host);

    websocket_cfg.port = 3001;
    ESP_LOGI(TAG, "websocket_cfg set port");

    // #if CONFIG_WS_OVER_TLS_SKIP_COMMON_NAME_CHECK
    websocket_cfg.skip_cert_common_name_check = true;
    ESP_LOGI(TAG, "websocket_cfg set skip_cert_common_name_check");
    // #endif

    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.host);

    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_client_append_header(client, "hw_id", uniqueID.c_str());
    esp_websocket_client_set_ping_interval_sec(client, 2);

    // esp_websocket_client_set_headers(client, "hw_id: %s\r\nversion:1.0.1\r\n", uniqueID.c_str());
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);
    // xTimerStart(shutdown_signal_timer, portMAX_DELAY);
    // char data[32];
    // int i = 0;
    // while (i < 5)
    // {
    //     if (esp_websocket_client_is_connected(client))
    //     {
    //         int len = sprintf(data, "hello %04d", i++);
    //         ESP_LOGI(TAG, "Sending %s", data);
    //         esp_websocket_client_send_text(client, data, len, portMAX_DELAY);
    //     }
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    ESP_LOGI(TAG, "Sending fragmented message");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // memset(data, 'a', sizeof(data));
    // esp_websocket_client_send_text_partial(client, data, sizeof(data), portMAX_DELAY);
    // memset(data, 'b', sizeof(data));
    // esp_websocket_client_send_cont_msg(client, data, sizeof(data), portMAX_DELAY);
    // esp_websocket_client_send_fin(client, portMAX_DELAY);

    // xSemaphoreTake(shutdown_sema, portMAX_DELAY);
    // esp_websocket_client_close(client, portMAX_DELAY);
    // ESP_LOGI(TAG, "Websocket Stopped");
    // esp_websocket_client_destroy(client);
}
void sendMessage(void *pvParameters)
{
    char *output = (char *)pvParameters;
    if (esp_websocket_client_is_connected(client))
    {

        ESP_LOGI(TAG, "SEND Message %s", output);
        // esp_websocket_client_send_text(client, output.c_s, strlen(output), portMAX_DELAY);
        esp_websocket_client_send_text(client, output, strlen(output), portMAX_DELAY);
    }
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
        safeFree((void **)&output);
        xSemaphoreGive(xMutex);
    }
    vTaskDelete(NULL);
}
void sendStatus(void *pvParameters)
{

    if (esp_websocket_client_is_connected(client))
    {
        ESP_LOGI(TAG, "Sending Status sendStatus");
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();
        array.add("status");
        JsonObject param1 = array.createNestedObject();
        param1["version"] = FIRMWARE_VERSION;
        param1["free_heap"] = esp_get_free_heap_size();
        param1["battery"] = ((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095);
        int rssi;
        esp_wifi_sta_get_rssi(&rssi);
        param1["rssi"] = rssi;
        String output;
        serializeJson(doc, output);
        doc.clear();
        char *taskData = (char *)safeMalloc(strlen(output.c_str()) + 1);
        if (taskData != nullptr)
        {
            strcpy(taskData, output.c_str());
            safeFree((void **)&output);
            if (xSemaphoreTake(xMutex, portMAX_DELAY))
            {
                xTaskCreate(sendMessage, "sendMessage", 4096, taskData, 4, NULL);

                xSemaphoreGive(xMutex);
            }
        }
    }
    vTaskDelete(NULL);
}
static void button_single_click_cb(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK %i", (int)usr_data);
    if (esp_websocket_client_is_connected(client))
    {

        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();
        array.add("click");
        JsonObject param1 = array.createNestedObject();
        param1["pressed"] = (int)usr_data;

        // param1["voltage"] = ((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095);
        String output;
        serializeJson(doc, output);
        doc.clear();
        // socketIO.sendEVENT(output);
        // char *taskData = (char *)malloc(strlen(output.c_str()) + 1);
        char *taskData = (char *)safeMalloc(strlen(output.c_str()) + 1);
        xTaskCreate(beepTask, "beepTask", 4096, NULL, 4, NULL);
        if (taskData != nullptr)
        {
            strcpy(taskData, output.c_str());
            safeFree((void **)&output);
            // char data[50];
            // sprintf(data, "{'event':'click','data':'click','button':'%i'}", (int)usr_data);
            if (xSemaphoreTake(xMutex, portMAX_DELAY))
            {
                xTaskCreate(sendMessage, "sendMessage", 4096, taskData, 5, NULL);
                xSemaphoreGive(xMutex);
            }
        }
        //
        // free(taskData);
        // safeFree((void**)&taskData);
    }
}
void registerButton(int pin, int button)
{

    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 3000,
        .short_press_time = 50,
        .gpio_button_config = {
            .gpio_num = pin,
            .active_level = 0,
        },
    };
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn)
    {
        ESP_LOGE(TAG, "Button create failed");
    }
    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, (int *)button);
}

void printStatusBottom(const char *buf)
{
    display.fillRect(0, 55, 128, 9, SSD1306_BLACK);
    display.setFont();

    printCenter(buf, 55);
}
void printCenter(const char *buf, int y)
{
    int x = 128;
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextColor(WHITE, BLACK);
    display.getTextBounds(buf, x, y, &x1, &y1, &w, &h); // calc width of new string
    display.setCursor(x / 2 - w / 2, y);

    display.print(buf);
    display.display();
}
void webSocketTask(void *pvParameters)
{
    websocket_app_start();
    vTaskDelete(NULL);
}
int checkRstButtonsCounter = 0;
int rssi_counter = 0;
int status_counter = 0;
int charging_status_counter = 0;

void screenLoop(void *pvParameters)
{
}
void ledLoop(void *pvParameters)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;
    bool blinkToggle = false;
    int lastBlinkToggleMillis =0;
    int blinkCounter = 0;
    while (1)
    {
        ESP_LOGI(TAG, "ledStatus %i", ledStatus);
        if (ledStatus == LED_OFF)
        {
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        else if (ledStatus == LED_ANIM)
        {

            for (int j = 0; j < LED_NUMBERS; j += 1)
            {
                hue = j * 360 / LED_NUMBERS / 2 + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[(j * 3 + 0)] = green;
                led_strip_pixels[(j * 3 + 1)] = blue;
                led_strip_pixels[(j * 3 + 2)] = red;
            }
            start_rgb += (360 / LED_NUMBERS / 2);
        }
        else if (ledStatus == LED_COLOR)
        {

            for (int j = 0; j < LED_NUMBERS; j += 1)
            {
                led_strip_pixels[(j * 3 + 0)] = ledValueG;
                led_strip_pixels[(j * 3 + 1)] = ledValueR;
                led_strip_pixels[(j * 3 + 2)] = ledValueB;
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                vTaskDelay(CHASE_SPEED_MS / 5 / portTICK_PERIOD_MS);
            }
        }
        else if (ledStatus == LED_BLINK)
        {

            if ( lastBlinkToggleMillis < millis())
            {
                lastBlinkToggleMillis = millis() + ledValueBlinkRate;
                blinkToggle = !blinkToggle;
                if (ledValueBlinkCounter > 0 && blinkToggle)
                {
                    blinkCounter++;
                }
            }

            if (!blinkToggle)
            {
                memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            }
            else
            {

                for (int j = 0; j < LED_NUMBERS; j += 1)
                {
                    led_strip_pixels[(j * 3 + 0)] = ledValueG;
                    led_strip_pixels[(j * 3 + 1)] = ledValueR;
                    led_strip_pixels[(j * 3 + 2)] = ledValueB;
                }
            }
            ESP_LOGI(TAG, "blinkToggle %s", blinkToggle ? "true" : "false");
            if (ledValueBlinkCounter>0 && blinkCounter > ledValueBlinkCounter)
            {
                ledStatus = LED_COLOR;
                blinkCounter = 0;
            }
        }

        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay((ledStatus == LED_BLINK && ledValueBlinkRate < CHASE_SPEED_MS ? ledValueBlinkRate : CHASE_SPEED_MS) / portTICK_PERIOD_MS);

        // memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
        // ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        // ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        // vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS * 10));
    }
}
void chargingLoop(void *pvParameters)
{
    int i = 0;
    while (1)
    {
        i++;
        int bat_chg_value = analogRead(BAT_CHG);
        int bat_stby_value = analogRead(BAT_STDBY);
        ESP_LOGI(TAG, "BAT_CHG value %i", bat_chg_value);
        ESP_LOGI(TAG, "BAT_STDBY value %i", bat_stby_value);
        if (digitalRead(BUTTON_1) == LOW && digitalRead(BUTTON_3) == LOW && digitalRead(BUTTON_7) == LOW)
        {
            // Serial.println("checkRstButtonsCounter " + checkRstButtonsCounter);
            // checkRstButtonsCounter++;

            // if (checkRstButtonsCounter > 1)
            // {
            ESP.restart();
            // }
        }
        else
        {
            // checkRstButtonsCounter = 0;
        }

        if (!(bat_chg_value < 70 || bat_stby_value < 70))
        {
            ESP_LOGI(TAG, "CHARGING STOPED");
            beep(NOTE_A6, 200, 50);
            beep(NOTE_G6, 200, 50);
            beep(NOTE_F6, 200, 0);
            ChargingFlag = false;
            printStatusBottom("CHRG STOPPED");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            battery_voltage = ((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095);
            display.setCursor(0, 55);
            display.setFont();
            display.setTextSize(1);
            display.print(String((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095));
            display.display();
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            digitalWrite(POWER_PIN, LOW);
        }
        else
        {

            if (bat_stby_value > 70 && bat_chg_value < 70)
            {
                printStatusBottom("BAT FULL");
                if (i % 2)
                {
                    for (int j = 0; j < LED_NUMBERS; j += 1)
                    {
                        led_strip_pixels[(j * 3 + 0)] = 255;
                        led_strip_pixels[(j * 3 + 1)] = 0;
                        led_strip_pixels[(j * 3 + 2)] = 0;
                    }
                }
                else
                {
                    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
                }

                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                printStatusBottom("CHARGING");
            }
        }
        battery_voltage = ((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095);
        display.setCursor(0, 55);
        display.setFont();
        display.setTextSize(1);
        display.print(String((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095));
        display.display();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (i > 100)
        {
            i = 0;
        }
    }
}
void loop(void *pvParameters)
{
    while (1)
    {

        rssi_counter++;

        status_counter++;

        charging_status_counter++;
        if (checkBatteryMillis > 0 && checkBatteryMillis > millis())
        {
            double bv = (analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095;

            // battery_voltage = ();
            display.fillRect(0, 0, 128, 64, SSD1306_BLACK);
            display.setCursor(40, 8);
            display.setFont();
            display.setTextSize(2);
            display.print(String(bv));
            display.display();
            ESP_LOGI(TAG, "Battery Voltage %f", bv);
            if (bv > 4.1)
            {
                ledStatus = LED_COLOR;
                ledValueG = 255;
                ledValueB = 0;
                ledValueR = 0;
            }
            else if (bv > 3.9)
            {
                ledStatus = LED_COLOR;
                ledValueG = 150;
                ledValueB = 50;
                ledValueR = 0;
            }
            else if (bv > 3.4)
            {
                ledStatus = LED_COLOR;
                ledValueG = 10;
                ledValueB = 10;
                ledValueR = 100;
            }
            else
            {
                ledStatus = LED_COLOR;
                ledValueG = 0;
                ledValueB = 0;
                ledValueR = 255;
            }
            analogWrite(BUTTONS_LED_R, 255 - ledValueR);
            analogWrite(BUTTONS_LED_G, 255 - ledValueG);
            analogWrite(BUTTONS_LED_B, 255 - ledValueB);
            turnOnBtnLeds();
        }
        else
        {
            if (checkBatteryMillis > 0)
            {
                // CHANGE status to led off if battery check time is over
                ledStatus = LED_OFF;
                display.fillRect(0, 0, 128, 64, SSD1306_BLACK);
                display.setTextSize(1);
                display.display();
                turnOffBtnLeds();
            }

            checkBatteryMillis = 0;
        }
        if (checkBatteryMillis > 0 || (stopVibrateMills > 0 && stopVibrateMills < millis()))
        {
            digitalWrite(VIBRATE_PIN, LOW);
            stopVibrateMills = 0;
        }
        if (digitalRead(BUTTON_1) == LOW && digitalRead(BUTTON_3) == LOW && digitalRead(BUTTON_7) == LOW)
        {
            // Serial.println("checkRstButtonsCounter " + checkRstButtonsCounter);
            checkRstButtonsCounter++;
            if (checkRstButtonsCounter > 20)
            {
                digitalWrite(VIBRATE_PIN, HIGH);
            }
            if (checkRstButtonsCounter > 25)
            {
                ESP.restart();
            }
        }
        else
        {
            checkRstButtonsCounter = 0;
            if (checkBatteryMillis > 0 || stopVibrateMills == 0)
            {
                digitalWrite(VIBRATE_PIN, LOW);
            }
        }

        if (!(checkBatteryMillis > 0) && rssi_counter > 25)
        {
            ESP_LOGI(LOOP_TAG, "rssi counter");
            ESP_LOGI(LOOP_TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
            // heap_caps_print_heap_info(MALLOC_CAP_8BIT);
            int rssi;
            esp_wifi_sta_get_rssi(&rssi);
            // printf("%d\n", ap.rssi);
            display.setCursor(110, 0);
            display.setFont();
            display.print(String(135 + rssi));
            display.drawLine(127, 0, 127, 45, BLACK);
            display.drawLine(127, max(0, static_cast<int>(floor(((135 + rssi) / 100) * 45))), 127, 45, WHITE);
            display.display();

            rssi_counter = 0;
        }

        if (!(checkBatteryMillis > 0) && status_counter > 50)
        {
            ESP_LOGI(LOOP_TAG, "status_counter");
            // TODO: send status
            if (((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095) < MIN_BATTERY)
            {
                beep(NOTE_G5, 200, 0);
                beep(NOTE_B4, 400, 0);
                beep(NOTE_G5, 200, 0);
                beep(NOTE_B4, 400, 0);
                analogWrite(BUTTONS_LED_R, 255 - 255);
                analogWrite(BUTTONS_LED_G, 255 - 0);
                analogWrite(BUTTONS_LED_B, 255 - 0);
                turnOnBtnLeds();
                printStatusBottom("LOW BATTERY");
                status_counter = 20;
            }
            else
            {
                xTaskCreate(sendStatus, "sendStatus", 4096, NULL, 4, NULL);
                status_counter = 0;
            }
        }

        if (ChargingFlag && charging_status_counter > 20)
        {
            int bat_chg_value = analogRead(BAT_CHG);
            int bat_stby_value = analogRead(BAT_STDBY);

            // DynamicJsonDocument doc(1024);
            // JsonArray array = doc.to<JsonArray>();
            // array.add("status");
            // JsonObject param1 = array.createNestedObject();
            // param1["version"] = FIRMWARE_VERSION;
            // param1["battery"] = ((analogRead(BAT_MON_PIN) * 2 * 3.7) / 4095);
            // param1["bat_chg_value"] = bat_chg_value;
            // param1["bat_stby_value"] = bat_stby_value;
            // String output;
            // serializeJson(doc, output);
            //   doc.clear();
            // char *taskData = (char *)malloc(strlen(output.c_str()) + 1);
            // strcpy(taskData, output.c_str());
            //   safeFree((void **)&output);
            // xTaskCreate(sendMessage, "sendMessage", 4096, taskData, 4, NULL);

            // safeFree((void**)&taskData);
            // xTaskCreate(sendStatus, "sendStatus", 4096, taskData, 4, NULL);
            if (!(bat_chg_value < 70 || bat_stby_value < 70))
            {
                ESP_LOGI(TAG, "CHARGING STOPED");
                beep(NOTE_A6, 200, 50);
                beep(NOTE_G6, 200, 50);
                beep(NOTE_F6, 200, 0);
                ChargingFlag = false;
                printStatusBottom("CHARGING STOPED");
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                digitalWrite(POWER_PIN, LOW);
            }

            charging_status_counter = 0;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void beepTask(void *pvParameter)
{
    // beep(NOTE_A4, 100, 0);
    beep(NOTE_E6, 150, 0);
    vTaskDelete(NULL);
}
void ota_task(void *pvParameter)
{
    char url[100]; // Ensure this is large enough to hold the final string
    sprintf(url, "http://192.168.0.2:3000/update?version=%s", FIRMWARE_VERSION);
    esp_http_client_config_t config = {
        .url = url,
        .skip_cert_common_name_check = true,

    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");

        printStatusBottom("UPDATE SUCCESS");
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // gpio_num_t gpio_num = GPIO_NUM_37;
        ESP_ERROR_CHECK(gpio_hold_en(GPIO_NUM_37));
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
    // while (1)
    // {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    vTaskDelete(NULL);
}
void led_strip_hsv2rgb(uint32_t h, uint32_t saturation, uint32_t value, uint32_t *r, uint32_t *g, uint32_t *b)
{
    uint32_t step = 60;
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = value * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - saturation) / 100.0f;

    uint32_t i = h / step;
    uint32_t diff = h % step;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / step;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
// static const char *if_str[] = {"STA", "AP", "ETH", "MAX"};
static const char *ip_protocol_str[] = {"V4", "V6", "MAX"};
void mdns_print_results(mdns_result_t *results)
{
    mdns_result_t *r = results;
    mdns_ip_addr_t *a = NULL;
    int i = 1, t;
    while (r)
    {
        printf("%d: Type: %s\n", i++, ip_protocol_str[r->ip_protocol]);
        if (r->instance_name)
        {
            ESP_LOGW(MDNS_TAG, "  PTR : %s\n", r->instance_name);
        }
        if (r->hostname)
        {
            ESP_LOGW(MDNS_TAG, "  SRV : %s.local:%u\n", r->hostname, r->port);
        }
        if (r->txt_count)
        {
            ESP_LOGW(MDNS_TAG, "  TXT : [%u] ", r->txt_count);
            for (t = 0; t < r->txt_count; t++)
            {
                ESP_LOGW(MDNS_TAG, "%s=%s; ", r->txt[t].key, r->txt[t].value);
            }
            // printf("\n");
        }
        a = r->addr;
        while (a)
        {
            if (a->addr.type == MDNS_IP_PROTOCOL_V6)
            {
                ESP_LOGW(MDNS_TAG, "  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
            }
            else
            {
                ESP_LOGW(MDNS_TAG, "  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));

                if (r->hostname && strcmp(r->hostname, "funclick") == 0)
                {
                    char buf[16];
                    sprintf(buf, "%d.%d.%d.%d", IP2STR(&(a->addr.u_addr.ip4)));
                    // ESP_LOGI(TAG, "buf=[%s]", buf);
                    // ESP_LOGI(TAG, "*buf=[%c]", *buf);
                    // SocketServerIp = *&buf;
                    // ESP_LOGI(TAG, "ServerIp Address=[%s]", SocketServerIp);
                    // SocketServerIp = buf;
                    strcpy(SocketServerIp, buf);

                    ESP_LOGI(TAG, "ServerIp Value=[%s]", SocketServerIp);
                    ESP_LOGW(MDNS_TAG, "Server Found at " IPSTR "", IP2STR(&(a->addr.u_addr.ip4)));
                    // safeFree(buf);
                }
            }
            a = a->next;
        }
        r = r->next;
    }
}
mdns_result_t *results = NULL;
int mdns_counter = 0;
void find_mdns_service(const char *service_name, const char *proto)
{
    printStatusBottom("PLEASE WAIT");
    mdns_counter++;
    if (mdns_counter > 200)
    {
        mdns_counter = 0;
        printStatusBottom("SRVR NOT FOUND");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // ESP_LOGW(TAG, "SHUTDOWN");
        beep(NOTE_A4, 80, 0);
        // beep(NOTE_A, 80, 100);
        digitalWrite(POWER_PIN, LOW);
        return;
    }
    else if (mdns_counter > 100)
    {
        // printStatusBottom("SRCH SRVR (" + mdns_counter + ")");
        mdns_free();
    }
    else if (mdns_counter > 10)
    {
        ESP_LOGI(MDNS_TAG, "MDNS Counter %i", mdns_counter);
        char statusMessage[50]; // Adjust the size as needed
        snprintf(statusMessage, sizeof(statusMessage), "SRCH SRVR (%d)", mdns_counter);
        printStatusBottom(statusMessage);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // find_mdns_service(service_name, proto);
        return;
    }

    ESP_LOGI(MDNS_TAG, "Query PTR: %s.%s.local", service_name, proto);

    // esp_err_t err = mdns_query_srv(service_name, "funclick", proto, 3000, &results);
    // esp_err_t err = mdns_query_srv(service_name, "_http", proto, 3000, &results);
    // esp_err_t err = mdns_query( service_name, "_http", proto, MDNS_TYPE_SRV, 3000, 20, &results);
    esp_err_t err = mdns_query(service_name, "_http", proto, MDNS_TYPE_A, 3000, 20, &results);
    if (err)
    {
        ESP_LOGE(MDNS_TAG, "Query Failed %i", err);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        find_mdns_service(service_name, proto);

        return;
    }
    if (!results)
    {
        ESP_LOGW(MDNS_TAG, "No results found!");

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        find_mdns_service(service_name, proto);
        return;
    }
    ESP_LOGI(MDNS_TAG, "Some Service found");

    mdns_print_results(results);

    // ESP_LOGI(MDNS_TAG, "Service found :%s.local", results->addr);
    mdns_query_results_free(results);
    if (strlen(SocketServerIp) > 0)
    {
        mdns_free();
    }
    else
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        find_mdns_service(service_name, proto);
    }
}
void app_main()
{
    ESP_LOGI(TAG, "app_main %s", FIRMWARE_VERSION);
    // heap_caps_print_heap_info("Initial");

    xMutex = xSemaphoreCreateMutex();

    initArduino();
    ESP_ERROR_CHECK(gpio_hold_dis(GPIO_NUM_37));

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    pinMode(BAT_CHG, INPUT);
    pinMode(BAT_STDBY, INPUT);
    // pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BAT_MON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);
    pinMode(BUTTON_4, INPUT_PULLUP);
    pinMode(BUTTON_5, INPUT_PULLUP);
    pinMode(BUTTON_6, INPUT_PULLUP);
    pinMode(BUTTON_7, INPUT_PULLUP);
    int bat_chg_value = analogRead(BAT_CHG);
    int bat_stby_value = analogRead(BAT_STDBY);
    ESP_LOGI(TAG, "BAT_CHG value %i", bat_chg_value);
    ESP_LOGI(TAG, "BAT_STDBY value %i", bat_stby_value);
    if ((digitalRead(BUTTON_1) == LOW))
    {
        ESP_LOGI(TAG, "BUTTON_1 is pressed");
    }
    if ((bat_chg_value < 70 || bat_stby_value < 70) && !(digitalRead(BUTTON_1) == LOW))
    {
        ChargingFlag = true;
        beep(NOTE_F6, 200, 0);
        beep(NOTE_G6, 100, 50);
        beep(NOTE_A6, 100, 50);
        beep(NOTE_G6, 100, 50);
        ESP_LOGI(TAG, "STAUTS CABLE");
        if (bat_chg_value > 70)
        {
            ESP_LOGI(TAG, "CHARGING");
        }
        if (bat_stby_value > 70)
        {
            ESP_LOGI(TAG, "BATTERY FULL");
        }
    }
    else
    {
        ChargingFlag = false;
        beep(NOTE_C6, 50, 0);
        beep(NOTE_C7, 50, 50);
    }
    pinMode(VIBRATE_PIN, OUTPUT);
    digitalWrite(VIBRATE_PIN, HIGH);
    delay(1 * 1000);
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    digitalWrite(VIBRATE_PIN, LOW);

    display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.display();
    display.drawBitmap(24, 0, funclick_logo, 80, 50, 1);
    char ver[100]; // Ensure this is large enough to hold the final string
    if (ChargingFlag)
    {
        printStatusBottom("CHARGING");
    }
    else
    {
        sprintf(ver, "VER: %s", FIRMWARE_VERSION);
        printStatusBottom(ver);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRATE_PIN, OUTPUT);
    pinMode(BUTTONS_LED_R, OUTPUT);
    pinMode(BUTTONS_LED_G, OUTPUT);
    pinMode(BUTTONS_LED_B, OUTPUT);
    analogWrite(255 - BUTTONS_LED_R, 0);
    analogWrite(255 - BUTTONS_LED_G, 0);
    analogWrite(255 - BUTTONS_LED_B, 0);

    pinMode(BUTTON_1_LED, OUTPUT);
    pinMode(BUTTON_2_LED, OUTPUT);
    pinMode(BUTTON_3_LED, OUTPUT);
    pinMode(BUTTON_4_LED, OUTPUT);
    pinMode(BUTTON_5_LED, OUTPUT);
    pinMode(BUTTON_6_LED, OUTPUT);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("websocket_client", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_ws", ESP_LOG_VERBOSE);
    esp_log_level_set("trans_tcp", ESP_LOG_VERBOSE);
    if (!ChargingFlag)
    {
        ESP_LOGI(TAG, "CONNECTING TO WIFI..");
        xTaskCreate(loop, "loop", 4096, NULL, 4, NULL);
        wifi_init_sta();

        esp_err_t err = mdns_init();
        if (err)
        {
            ESP_LOGW(MDNS_TAG, "MDNS Init failed: %d\n", err);
            // return;
        }
        else
        {
            find_mdns_service("funclick", "_tcp");
        }
        if (strlen(SocketServerIp) > 0)
        {
            xTaskCreate(&ota_task, "ota_task", 8192, NULL, 1, NULL);
            // ESP_LOGW(MDNS_TAG, "Starting WebSocket Task %c", SocketServerIp);
            // ESP_LOGW(MDNS_TAG, "Starting WebSocket Task * %c", &SocketServerIp);
            ESP_LOGW(MDNS_TAG, "Starting WebSocket Task & %s", SocketServerIp);
            xTaskCreate(webSocketTask, "webSocketTask", 4096, NULL, 4, NULL);
        }
        analogWrite(BUTTONS_LED_R, 255);
        analogWrite(BUTTONS_LED_G, 0);
        analogWrite(BUTTONS_LED_B, 255);
        digitalWrite(BUTTON_1_LED, HIGH);
        digitalWrite(BUTTON_2_LED, HIGH);
        digitalWrite(BUTTON_3_LED, HIGH);
        digitalWrite(BUTTON_4_LED, HIGH);
        digitalWrite(BUTTON_5_LED, HIGH);
        digitalWrite(BUTTON_6_LED, HIGH);
    }

    if (!ChargingFlag)
    {
        // xTaskCreate(&ota_task, "ota_task", 8192, NULL, 1, NULL);
    }
    ESP_LOGI(TAG, "webSocketTask start");
    if (!ChargingFlag)
    {
        // xTaskCreate(webSocketTask, "webSocketTask", 4096, NULL, 4, NULL);
    }
    // websocket_app_start();
    registerButton(BUTTON_1, 1);
    registerButton(BUTTON_2, 2);
    registerButton(BUTTON_3, 3);
    registerButton(BUTTON_4, 4);
    registerButton(BUTTON_5, 5);
    registerButton(BUTTON_6, 6);
    registerButton(BUTTON_7, 7);

    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = GPIO_NUM_48,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering

        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    if (ChargingFlag)
    {
        // esp_pm_config_t pm_config = {
        // .max_freq_mhz = 40,
        // .min_freq_mhz = 40};
        // ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        xTaskCreate(chargingLoop, "chargingLoop", 4096, NULL, 4, NULL);
    }
    else
    {

        xTaskCreate(ledLoop, "ledLoop", 4096, NULL, 4, NULL);
    }
}
