
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// #include <esp_wifi.h>
#include "esp_system.h"
#include "esp_wifi.h"
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
#define FIRMWARE_VERSION "1.00"
// #include "esp_lcd_panel_io_interface.h"
#include "pitches.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "driver/i2c.h"

// #include "esp_hw_support.h"
#include "esp_mac.h"

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

#define WIFI_SSID "FUNCLICK"
#define WIFI_PASS "funclick123!"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static const char *LOOP_TAG = "loop function";
// SocketIOclient socketIO;
// String ServerIp = "192.168.0.2";
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

bool ChargingFlag = false;
void printCenter(const char *buf, int y);
void printStatusBottom(const char *buf);
void sendMessage(void *pvParameters);
void sendStatus(void *pvParameters);
void beepTask(void *pvParameter);
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
    pinMode(BUZZER_PIN, OUTPUT);
    tone(BUZZER_PIN, note, duration);
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

    esp_netif_create_default_wifi_sta();

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

        display.fillRect(0, 0, 7, 8, SSD1306_BLACK);
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
                            int value_blink = cJSON_GetObjectItem(event_data_object, "value_blink")->valueint;
                            int value_blink_counter = cJSON_GetObjectItem(event_data_object, "value_blink_counter")->valueint;
                            int value_brightness = cJSON_GetObjectItem(event_data_object, "value_brightness")->valueint;
                            analogWrite(BUTTONS_LED_R, 255 - value_r);
                            analogWrite(BUTTONS_LED_G, 255 - value_g);
                            analogWrite(BUTTONS_LED_B, 255 - value_b);

                            turnOnBtnLeds();
                        }
                        else
                        {
                            turnOffBtnLeds();
                        }
                        // cJSON_free(event_data);
                        // cJSON_free(event_data_object);
                    }
                }
                else if (std::string(event_name) == "player_info")
                {
                    ESP_LOGW(TAG, "processing player_info event");
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                    int player_number = cJSON_GetObjectItem(event_data_object, "number")->valueint;
                    int rank = cJSON_GetObjectItem(event_data_object, "rank")->valueint;
                    int score = cJSON_GetObjectItem(event_data_object, "score")->valueint;
                    // ESP_LOGI(TAG, "PLAYER NUMBER %i", player_number);
                    // ESP_LOGI(TAG, "PLAYER RANK %i", rank);
                    // ESP_LOGI(TAG, "PLAYER SCORE %i", score);
                    display.fillRect(24, 0, 80, 50, SSD1306_BLACK);
                    display.display();
                    // vTaskDelay(100 / portTICK_PERIOD_MS);
                    // beep(NOTE_A4, 100, 0);
                    display.setFont(&FreeSerifItalic24pt7b);
                    display.setTextSize(1);
                    char player_number_str[10];
                    snprintf(player_number_str, sizeof(player_number_str), "%d", player_number);
                    printCenter(player_number_str, 45);

                    display.display();
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    display.fillRect(0, 50, 128, 14, SSD1306_BLACK);
                    display.drawLine(0, 50, 128, 50, WHITE);
                    display.drawLine(63, 50, 63, 64, WHITE);
                    display.setFont();
                    display.setCursor(2, 55);
                    display.print("RANK:" + String(rank));
                    display.setCursor(66, 55);
                    display.print("SCORE:" + String(score));
                    display.display();
                }
                else if (String(event_name) == "shutdown")
                {
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
    websocket_cfg.host = "192.168.0.2";
    // websocket_cfg.host = "192.168.1.193";
    ESP_LOGI(TAG, "websocket_cfg set host");

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
    if (esp_websocket_client_is_connected(client))
    {

        char *output = (char *)pvParameters;

        ESP_LOGI(TAG, "SEND Message %s", output);
        // esp_websocket_client_send_text(client, output.c_s, strlen(output), portMAX_DELAY);
        esp_websocket_client_send_text(client, output, strlen(output), portMAX_DELAY);
        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            safeFree((void **)&output);
            xSemaphoreGive(xMutex);
        }
    }
    vTaskDelete(NULL);
}
void sendStatus(void *pvParameters)
{
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
void loop(void *pvParameters)
{
    while (1)
    {

        rssi_counter++;

        status_counter++;

        charging_status_counter++;
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
            digitalWrite(VIBRATE_PIN, LOW);
        }

        if (rssi_counter > 25)
        {

            // ESP_LOGI(LOOP_TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
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

        if (status_counter > 50)
        {
            // TODO: send status
            xTaskCreate(sendStatus, "sendStatus", 4096, NULL, 4, NULL);
            status_counter = 0;
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
        beep(NOTE_G6, 200, 50);
        beep(NOTE_A6, 200, 50);
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

    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

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

    sprintf(ver, "VER: %s", FIRMWARE_VERSION);
    printStatusBottom(ver);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    pinMode(BUZZER_PIN, OUTPUT);
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
    digitalWrite(BUTTON_1_LED, HIGH);
    digitalWrite(BUTTON_2_LED, HIGH);
    digitalWrite(BUTTON_3_LED, HIGH);
    digitalWrite(BUTTON_4_LED, HIGH);
    digitalWrite(BUTTON_5_LED, HIGH);
    digitalWrite(BUTTON_6_LED, HIGH);

    wifi_init_sta();

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("websocket_client", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_ws", ESP_LOG_VERBOSE);
    esp_log_level_set("trans_tcp", ESP_LOG_VERBOSE);
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 1, NULL);

    ESP_LOGI(TAG, "webSocketTask start");
    xTaskCreate(webSocketTask, "webSocketTask", 4096, NULL, 4, NULL);
    // websocket_app_start();
    registerButton(BUTTON_1, 1);
    registerButton(BUTTON_2, 2);
    registerButton(BUTTON_3, 3);
    registerButton(BUTTON_4, 4);
    registerButton(BUTTON_5, 5);
    registerButton(BUTTON_6, 6);
    registerButton(BUTTON_7, 7);
    xTaskCreate(loop, "loop", 4096, NULL, 4, NULL);
    // while (1)
    // {
    //     ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }
}
