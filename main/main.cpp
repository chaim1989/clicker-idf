/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// #define CONFIG_BRIDGE_SOFTAP_PASSWORD "funclick123!"
// #define CONFIG_BRIDGE_SOFTAP_SSID "FUNCLICK_MESH"
#define CONFIG_MESH_LITE_MAX_ROUTER_NUMBER 10
#define CONFIG_BRIDGE_SOFTAP_MAX_CONNECT_NUMBER 10
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "funclick_logo.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <string.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/socket.h>
#include "esp_websocket_client.h"
#include "esp_mac.h"
#include "Arduino.h"
#include "pitches.h"
#include <ArduinoJson.h>
#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include <esp32-hal-log.h>
#include "cJSON.h"
#include <esp_http_client.h>
#include "iot_button.h"
#include "Wire.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define PAYLOAD_LEN (1456) /**< Max payload size(in bytes) */
#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_HOST_ID 0
#define I2C_DEV_ADDR 0x3C
#define DISP_BUF_SIZE (128 * 64 / 8)
#define POWER_PIN 37
#define BUZZER_PIN 47
#define VIBRATE_PIN 21
#define BAT_MON_PIN 1
#define BAT_CHG 17
#define BAT_STDBY 18
#include "Adafruit_SSD1306.h"
#include "Fonts/FreeSerifItalic24pt7b.h"
Adafruit_SSD1306 display;
static int g_sockfd = -1;
static const char *TAG = "local_control";
static void websocket_app_start(void);
extern "C"
{
    void app_main();
}
/**
 * @brief Create a tcp client
 */
// static int socket_tcp_client_create(const char *ip, uint16_t port)
// {
//     ESP_LOGD(TAG, "Create a tcp client, ip: %s, port: %d", ip, port);

//     esp_err_t ret = ESP_OK;
//     int sockfd = -1;
//     struct ifreq iface;
//     memset(&iface, 0x0, sizeof(iface));
//     struct sockaddr_in server_addr = {
//         .sin_family = AF_INET,
//         .sin_port = htons(port),
//         .sin_addr = {.s_addr = inet_addr(ip)},
//     };

//     sockfd = socket(AF_INET, SOCK_STREAM, 0);
//     if (sockfd < 0)
//     {
//         ESP_LOGE(TAG, "socket create, sockfd: %d", sockfd);
//         goto ERR_EXIT;
//     }

//     esp_netif_get_netif_impl_name(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), iface.ifr_name);
//     if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(struct ifreq)) != 0)
//     {
//         ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sockfd, iface.ifr_name);
//     }

//     ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
//     if (ret < 0)
//     {
//         ESP_LOGD(TAG, "socket connect, ret: %d, ip: %s, port: %d",
//                  ret, ip, port);
//         goto ERR_EXIT;
//     }
//     return sockfd;

// ERR_EXIT:

//     if (sockfd != -1)
//     {
//         close(sockfd);
//     }

//     return -1;
// }
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
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
        // display.drawBitmap(0, 0, icon_connected, 7, 8, SSD1306_WHITE);
        display.display();

        // xTaskCreate(sendStatus, "sendStatus", 4096, NULL, 4, NULL);
    }
    break;
    case WEBSOCKET_EVENT_DISCONNECTED:
    {
        // printStatusBottom("SRVR DSCNCT");
        // ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
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
                            // ledValueR = value_r;
                            // ledValueG = value_g;
                            // ledValueB = value_b;
                            // ledStatus = 1;
                            int value_blink = cJSON_GetObjectItem(event_data_object, "value_blink")->valueint;
                            int value_blink_counter = cJSON_GetObjectItem(event_data_object, "value_blink_counter")->valueint;
                            int value_brightness = cJSON_GetObjectItem(event_data_object, "value_brightness")->valueint;
                            // analogWrite(BUTTONS_LED_R, 255 - value_r);
                            // analogWrite(BUTTONS_LED_G, 255 - value_g);
                            // analogWrite(BUTTONS_LED_B, 255 - value_b);

                            // turnOnBtnLeds();
                        }
                        else
                        {
                            // turnOffBtnLeds();
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
                    // printCenter(player_number_str, 45);

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
                    // printCenter("SHUTDOWN", 20);
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    ESP_LOGW(TAG, "SHUTDOWN");
                    beep(NOTE_A4, 80, 0);
                    beep(NOTE_A, 80, 100);
                    // digitalWrite(POWER_PIN, LOW);

                    // xTimerReset(shutdown_signal_timer, portMAX_DELAY);
                }
                else if (std::string(event_name) == "vibrate")
                {
                    cJSON *event_data_object = cJSON_GetArrayItem(root, 1);
                    int value = cJSON_GetObjectItem(event_data_object, "value")->valueint;
                    digitalWrite(VIBRATE_PIN, HIGH);
                    // stopVibrateMills = millis() + value;
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
void webSocketTask(void *pvParameters)
{
    websocket_app_start();
    vTaskDelete(NULL);
}
esp_websocket_client_handle_t client;
// String uniqueID;
static void websocket_app_start(void)
{
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    char macStr[18]; // 6 bytes * 2 characters per byte + 5 colons + 1 null terminator

    ;
    //  uniqueID = String(macStr);
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
    esp_websocket_client_append_header(client, "hw_id", macStr);
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

// void tcp_client_write_task(void *arg)
// {
//     size_t size = 0;
//     int count = 0;
//     char *data = NULL;
//     esp_err_t ret = ESP_OK;
//     uint8_t sta_mac[6] = {0};

//     esp_wifi_get_mac(WIFI_IF_STA, sta_mac);

//     ESP_LOGI(TAG, "TCP client write task is running");

//     while (1)
//     {
//         if (g_sockfd == -1)
//         {
//             vTaskDelay(500 / portTICK_PERIOD_MS);
//             g_sockfd = socket_tcp_client_create("192.168.0.2", 3001);
//             continue;
//         }

//         vTaskDelay(3000 / portTICK_PERIOD_MS);

//         size = asprintf(&data, "{\"src_addr\": \"" MACSTR "\",\"data\": \"Hello TCP Server!\",\"level\": %d,\"count\": %d}\r\n",
//                         MAC2STR(sta_mac), esp_mesh_lite_get_level(), count++);

//         ESP_LOGD(TAG, "TCP write, size: %d, data: %s", size, data);
//         ret = write(g_sockfd, data, size);
//         free(data);

//         if (ret <= 0)
//         {
//             ESP_LOGE(TAG, "<%s> TCP write", strerror(errno));
//             close(g_sockfd);
//             g_sockfd = -1;
//             continue;
//         }
//     }

//     ESP_LOGI(TAG, "TCP client write task is exit");

//     close(g_sockfd);
//     g_sockfd = -1;
//     if (data)
//     {
//         free(data);
//     }
//     vTaskDelete(NULL);
// }

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    //     uint8_t primary = 0;
    //     uint8_t sta_mac[6] = {0};
    //     wifi_ap_record_t ap_info = {0};
    //     wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    //     wifi_sta_list_t wifi_sta_list = {0x0};

    //     esp_wifi_sta_get_ap_info(&ap_info);
    //     esp_wifi_get_mac(WIFI_IF_STA, sta_mac);
    //     esp_wifi_ap_get_sta_list(&wifi_sta_list);
    //     esp_wifi_get_channel(&primary, &second);

    //     ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR ", parent rssi: %d, free heap: %" PRIu32 "", primary,
    //              esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
    //              (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());
    // #if CONFIG_MESH_LITE_MAXIMUM_NODE_NUMBER
    //     ESP_LOGI(TAG, "child node number: %d", esp_mesh_lite_get_child_node_number());
    // #endif /* MESH_LITE_NODE_INFO_REPORT */
    //     for (int i = 0; i < wifi_sta_list.num; i++)
    //     {
    //         ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    //     }
}

static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    static bool tcp_task = false;

    if (!tcp_task)
    {
        // xTaskCreate(webSocketTask, "webSocketTask", 4 * 1024, NULL, 5, NULL);
        tcp_task = true;
    }
}

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static void wifi_init(void)
{
    // Station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "FUNCLICK",
            .password = "funclick123!",
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
    strlcpy((char *)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[32];
    uint8_t softap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));

#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
    snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
    snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
    esp_mesh_lite_set_softap_ssid_to_nvs(softap_ssid);
    esp_mesh_lite_set_softap_psw_to_nvs(CONFIG_BRIDGE_SOFTAP_PASSWORD);
    esp_mesh_lite_set_softap_info(softap_ssid, CONFIG_BRIDGE_SOFTAP_PASSWORD);
}

void app_main()
{
    /**
     * @brief Set the log level for serial port printing.
     */
    initArduino();
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();

    esp_mesh_lite_start();

    display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setRotation(2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.display();
    display.drawBitmap(24, 0, funclick_logo, 80, 50, 1);
    /**
     * @breif Create handler
     */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}