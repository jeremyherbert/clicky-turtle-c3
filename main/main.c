#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <esp_mac.h>
#include <esp_random.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"

#include "esp_bt_main.h"
#include "esp_hid_common.h"
#include "esp_gap_ble_api.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "esp_hidd.h"

#define SIZEOF_ARRAY(a) (sizeof(a)/sizeof(*a))

static const char *TAG = "MAIN";

/* GPIO configuration */
#define GPIO_MOSI 7
#define GPIO_MISO 6
#define GPIO_SCLK 5
#define GPIO_CS 4

#define GPIO_SPI_IDLE 3

#define GPIO_CONN_STATE_0 0
#define GPIO_CONN_STATE_1 1

/* events to indicate the status of the BLE connection */
#define BLE_EVENT_ADVERTISING_STARTED 0
#define BLE_EVENT_PASSKEY_REQUEST 1
#define BLE_EVENT_CONNECTED 2
EventGroupHandle_t ble_events;
StaticEventGroup_t ble_events_mem;

typedef enum {
    CONN_STATE_OFF = 0,
    CONN_STATE_DISCONNECTED = 1,
    CONN_STATE_PASSKEY_REQUEST = 2,
    CONN_STATE_CONNECTED = 3,
} ble_conn_state_t;

/* lock for downlink buffer */
SemaphoreHandle_t latest_downlink_lock = NULL;
StaticSemaphore_t latest_downlink_lock_buffer;

/* SPI commands */
#define SPI_COMMAND_PASSKEY 0xAB
#define SPI_COMMAND_SCANCODE 0xCD
#define SPI_COMMAND_SIDEBAND_UPLINK 0xEF
#define SPI_COMMAND_SIDEBAND_DOWNLINK 0xA1
#define SPI_COMMAND_UNPAIR 0xB2
#define SPI_COMMAND_SYNC 0xC3

/* target address for passkey response */
static esp_bd_addr_t auth_target;

/* buffer to hold latest downlink */
static uint8_t latest_downlink[32];

/* ESP_HID handle */
esp_hidd_dev_t *hid_dev;

void set_conn_state(ble_conn_state_t conn_state) {
    gpio_set_level(GPIO_CONN_STATE_0, conn_state & 0x01);
    gpio_set_level(GPIO_CONN_STATE_1, (conn_state & 0x02) >> 1);
}

void spi_set_idle() {
    gpio_set_level(GPIO_SPI_IDLE, true);
}

void spi_clear_idle() {
    gpio_set_level(GPIO_SPI_IDLE, false);
}

void spi_and_gpio_init() {
    esp_err_t ret;

    spi_bus_config_t spi_config = {
            .mosi_io_num = GPIO_MOSI,
            .miso_io_num = GPIO_MISO,
            .sclk_io_num = GPIO_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slave_config = {
            .mode = 0,
            .spics_io_num = GPIO_CS,
            .queue_size = 3,
            .flags = 0,
            .post_setup_cb=spi_clear_idle,
            .post_trans_cb=spi_set_idle,
    };

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    ret = spi_slave_initialize(SPI2_HOST, &spi_config, &slave_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    gpio_config_t cfg = {
            .pin_bit_mask = (1 << GPIO_SPI_IDLE) | (1 << GPIO_CONN_STATE_0) | (1 << GPIO_CONN_STATE_1),
            .mode = GPIO_MODE_OUTPUT,
    };
    ret = gpio_config(&cfg);
    ESP_ERROR_CHECK(ret);

    set_conn_state(CONN_STATE_OFF);
}

void spi_task(void *pvParameters) {
    WORD_ALIGNED_ATTR uint8_t tx_buf[34] = {0};
    WORD_ALIGNED_ATTR uint8_t rx_buf[34] = {0};

    spi_slave_transaction_t t = {0};
    t.length = 33 * 8;
    t.trans_len = 0;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    esp_err_t ret;

    while (1) {
        if (xSemaphoreTake(latest_downlink_lock, 0) == pdTRUE) {
            memcpy(tx_buf, latest_downlink, 32);
            xSemaphoreGive(latest_downlink_lock);
        } else {
            ESP_LOGW(TAG, "Failed to queue downlink for TX");
        }

        memset(rx_buf, 0, sizeof(rx_buf));

        spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        ESP_LOGI(TAG, "Received SPI size: %i\r\n", t.trans_len);

        if (t.trans_len == 7 * 8 && rx_buf[0] == SPI_COMMAND_PASSKEY) {
            uint32_t passkey = 0;
            for (int i = 0; i < 6; i++) {
                uint8_t digit = rx_buf[i + 1];
                if (digit > 9) {
                    ESP_LOGE(TAG, "Passkey digit %i is out of range", i);
                    continue;
                }
                passkey *= 10;
                passkey += digit;
            }

            ESP_LOGI(TAG, "Sending passkey %lu to %02X:%02X:%02X:%02X:%02X:%02X", passkey, auth_target[0],
                     auth_target[1],
                     auth_target[2], auth_target[3], auth_target[4], auth_target[5]);
            esp_ble_passkey_reply(auth_target, true, passkey);
        } else if (t.trans_len == 9 * 8 && rx_buf[0] == SPI_COMMAND_SCANCODE) {
            if (rx_buf[2] != 0x00) {
                ESP_LOGE(TAG, "HID report reserved byte is not 0x00");
            } else {
                ESP_LOGI(TAG, "Sending scan code report");
                esp_hidd_dev_input_set(hid_dev, 0, 1, rx_buf + 1, t.trans_len / 8 - 1);
            }
        } else if (t.trans_len == 33 * 8 && rx_buf[0] == SPI_COMMAND_SIDEBAND_UPLINK) {
            ESP_LOGI(TAG, "Sending sideband uplink");
            esp_hidd_dev_input_set(hid_dev, 1, 3, rx_buf + 1, t.trans_len / 8 - 1);
        } else if (t.trans_len == 32 * 8 && rx_buf[0] == SPI_COMMAND_SIDEBAND_DOWNLINK) {
            ESP_LOGI(TAG, "Downlink requested");
            // do nothing as this will be queued next time automatically
        } else if (t.trans_len == 8 && rx_buf[0] == SPI_COMMAND_SYNC) {
            ESP_LOGI(TAG, "Sync requested");
        } else if (t.trans_len == 8 * 8 && rx_buf[0] == SPI_COMMAND_UNPAIR) {
            nvs_handle_t nvs_handle;
            ret = nvs_open("data", NVS_READWRITE, &nvs_handle);
            ESP_ERROR_CHECK(ret);

            nvs_set_u8(nvs_handle, "unpair", 1);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            esp_restart();
        }
    }
}

/* BLE GAP strings */
static const char *ble_gap_evt_names[] = {"ADV_DATA_SET_COMPLETE", "SCAN_RSP_DATA_SET_COMPLETE",
                                          "SCAN_PARAM_SET_COMPLETE", "SCAN_RESULT", "ADV_DATA_RAW_SET_COMPLETE",
                                          "SCAN_RSP_DATA_RAW_SET_COMPLETE", "ADV_START_COMPLETE", "SCAN_START_COMPLETE",
                                          "AUTH_CMPL", "KEY", "SEC_REQ", "PASSKEY_NOTIF", "PASSKEY_REQ", "OOB_REQ",
                                          "LOCAL_IR", "LOCAL_ER", "NC_REQ", "ADV_STOP_COMPLETE", "SCAN_STOP_COMPLETE",
                                          "SET_STATIC_RAND_ADDR", "UPDATE_CONN_PARAMS", "SET_PKT_LENGTH_COMPLETE",
                                          "SET_LOCAL_PRIVACY_COMPLETE", "REMOVE_BOND_DEV_COMPLETE",
                                          "CLEAR_BOND_DEV_COMPLETE", "GET_BOND_DEV_COMPLETE", "READ_RSSI_COMPLETE",
                                          "UPDATE_WHITELIST_COMPLETE"};
static const char *bt_gap_evt_names[] = {"DISC_RES", "DISC_STATE_CHANGED", "RMT_SRVCS", "RMT_SRVC_REC", "AUTH_CMPL",
                                         "PIN_REQ", "CFM_REQ", "KEY_NOTIF", "KEY_REQ", "READ_RSSI_DELTA"};
static const char *ble_addr_type_names[] = {"PUBLIC", "RANDOM", "RPA_PUBLIC", "RPA_RANDOM"};

const char *esp_ble_key_type_str(esp_ble_key_type_t key_type) {
    const char *key_str = NULL;
    switch (key_type) {
        case ESP_LE_KEY_NONE:
            key_str = "ESP_LE_KEY_NONE";
            break;
        case ESP_LE_KEY_PENC:
            key_str = "ESP_LE_KEY_PENC";
            break;
        case ESP_LE_KEY_PID:
            key_str = "ESP_LE_KEY_PID";
            break;
        case ESP_LE_KEY_PCSRK:
            key_str = "ESP_LE_KEY_PCSRK";
            break;
        case ESP_LE_KEY_PLK:
            key_str = "ESP_LE_KEY_PLK";
            break;
        case ESP_LE_KEY_LLK:
            key_str = "ESP_LE_KEY_LLK";
            break;
        case ESP_LE_KEY_LENC:
            key_str = "ESP_LE_KEY_LENC";
            break;
        case ESP_LE_KEY_LID:
            key_str = "ESP_LE_KEY_LID";
            break;
        case ESP_LE_KEY_LCSRK:
            key_str = "ESP_LE_KEY_LCSRK";
            break;
        default:
            key_str = "INVALID BLE KEY TYPE";
            break;

    }
    return key_str;
}

const char *ble_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(ble_gap_evt_names)) {
        return "UNKNOWN";
    }
    return ble_gap_evt_names[event];
}


esp_err_t esp_hid_ble_gap_adv_start(void) {
    static esp_ble_adv_params_t adv_params = {
            .adv_int_min        = 0x20,
            .adv_int_max        = 0x30,
            .adv_type           = ADV_TYPE_IND,
            .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
            .channel_map        = ADV_CHNL_ALL,
            .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    ESP_LOGI(TAG, "Advertising started");
    set_conn_state(CONN_STATE_DISCONNECTED);
    return esp_ble_gap_start_advertising(&adv_params);
}

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE GAP ADV_DATA_SET_COMPLETE");
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE GAP ADV_START_COMPLETE");
            xEventGroupSetBits(ble_events, BLE_EVENT_ADVERTISING_STARTED);
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (!param->ble_security.auth_cmpl.success) {
                ESP_LOGE(TAG, "BLE GAP AUTH ERROR: 0x%x", param->ble_security.auth_cmpl.fail_reason);
                xEventGroupClearBits(ble_events, BLE_EVENT_CONNECTED);
                set_conn_state(CONN_STATE_DISCONNECTED);
                esp_restart();
                //esp_hid_ble_gap_adv_start();
            } else {
                ESP_LOGI(TAG, "BLE GAP AUTH SUCCESS");
                xEventGroupSetBits(ble_events, BLE_EVENT_CONNECTED);
                set_conn_state(CONN_STATE_CONNECTED);
            }
            break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(TAG, "BLE GAP SEC_REQ");
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            ESP_LOGI(TAG, "Host is requesting passkey");
            memcpy(auth_target, param->ble_security.ble_req.bd_addr, sizeof(auth_target));
            xEventGroupSetBits(ble_events, BLE_EVENT_PASSKEY_REQUEST);
            set_conn_state(CONN_STATE_PASSKEY_REQUEST);
            break;

        default:
            ESP_LOGI(TAG, "BLE GAP EVENT %s", ble_gap_evt_str(event));
            break;
    }
}

esp_err_t ble_hid_gap_init() {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // release BT classic memory to heap
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %d", ret);
        return ret;
    }

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(ble_gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", ret);
        return ret;
    }

    return ESP_OK;
}

const unsigned char hidapiReportMap[] = { //8 bytes input, 8 bytes feature
        0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
        0x09, 0x01,        // Usage (0x01)
        0xA1, 0x01,        // Collection (Application)
        0x85, 0x03,         //   Report ID (3)
        0x09, 0x02,        //   Usage (0x02)
        0x15, 0x00,        //   Logical Minimum (0)
        0x25, 0xFF,        //   Logical Maximum (-1)
        0x75, 0x08,        //   Report Size (8)
        0x95, 0x20,        //   Report Count (32)
        0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        0x09, 0x03,        //   Usage (0x03)
        0x15, 0x00,        //   Logical Minimum (0)
        0x25, 0xFF,        //   Logical Maximum (-1)
        0x75, 0x08,        //   Report Size (8)
        0x95, 0x20,        //   Report Count (32)
        0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
        0xC0,              // End Collection
};

const unsigned char keyboardHidReportMap[] = {
        0x05, 0x01,  // Usage Page (Keyboard)
        0x09, 0x06,  // Usage (Keyboard)
        0xA1, 0x01,  // Collection (Application)
        0x85, 0x01,  //   Report ID (1)
        0x05, 0x07,  //     Usage Page (Key Codes)
        0x19, 0xe0, //     Usage Minimum (224)
        0x29, 0xe7, //     Usage Maximum (231)
        0x15, 0x00, //     Logical Minimum (0)
        0x25, 0x01, //     Logical Maximum (1)
        0x75, 0x01, //     Report Size (1)
        0x95, 0x08, //     Report Count (8)
        0x81, 0x02, //     Input (Data, Variable, Absolute)

        0x95, 0x01, //     Report Count (1)
        0x75, 0x08, //     Report Size (8)
        0x81, 0x01, //     Input (Constant) reserved byte(1)

        0x95, 0x05, //     Report Count (5)
        0x75, 0x01, //     Report Size (1)
        0x05, 0x08, //     Usage Page (Page# for LEDs)
        0x19, 0x01, //     Usage Minimum (1)
        0x29, 0x05, //     Usage Maximum (5)
        0x91, 0x02, //     Output (Data, Variable, Absolute), Led report

        0x95, 0x01, //     Report Count (1)
        0x75, 0x03, //     Report Size (3)
        0x91, 0x01, //     Output (Data, Variable, Absolute), Led report padding

        0x95, 0x06, //     Report Count (6)
        0x75, 0x08, //     Report Size (8)
        0x15, 0x00, //     Logical Minimum (0)
        0x25, 0x65, //     Logical Maximum (101)
        0x05, 0x07, //     Usage Page (Key codes)
        0x19, 0x00, //     Usage Minimum (0)
        0x29, 0x65, //     Usage Maximum (101)
        0x81, 0x00, //     Input (Data, Array) Key array(6 bytes)

        0x09, 0x05, //     Usage (Vendor Defined)
        0x15, 0x00, //     Logical Minimum (0)
        0x26, 0xFF, 0x00, //     Logical Maximum (255)
        0x75, 0x08, //     Report Count (2)
        0x95, 0x02, //     Report Size (8 bit)
        0xB1, 0x02, //     Feature (Data, Variable, Absolute)

        0xC0, // End Collection (Application)
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
        {
                .data = keyboardHidReportMap,
                .len = sizeof(keyboardHidReportMap),
        },
        {
                .data = hidapiReportMap,
                .len = sizeof(hidapiReportMap)
        },
};

static esp_hid_device_config_t ble_hid_config = {
        .vendor_id          = 0xCAFE,
        .product_id         = 0xF00D,
        .version            = 0x0100,
        .device_name        = "clicky-turtle-c3",
        .manufacturer_name  = "Herbert Engineering",
        .serial_number      = "1234567890",
        .report_maps        = ble_report_maps,
        .report_maps_len    = 2
};

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t) id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *) event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event) {
        case ESP_HIDD_START_EVENT: {
            ESP_LOGI(TAG, "START");
            esp_hid_ble_gap_adv_start();
            break;
        }
        case ESP_HIDD_CONNECT_EVENT: {
            ESP_LOGI(TAG, "CONNECT");
            break;
        }
        case ESP_HIDD_PROTOCOL_MODE_EVENT: {
            ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index,
                     param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
            break;
        }
        case ESP_HIDD_CONTROL_EVENT: {
            ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
            break;
        }
        case ESP_HIDD_OUTPUT_EVENT: {
            ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index,
                     esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
            ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);

            if (param->output.map_index == 1 && param->output.report_id == 3) {
                if ((param->output.length == 33 && param->output.data[0] == 0x03) || (param->output.length == 32)) {
                    if (xSemaphoreTake(latest_downlink_lock, 1) == pdTRUE) {
                        ESP_LOGI(TAG, "Got downlink");
                        if (param->output.length == 32) {
                            // windows specific
                            memcpy(latest_downlink, param->output.data, 32);
                        } else {
                            // mac/linux specific
                            memcpy(latest_downlink, param->output.data+1, 32);
                        }
                        xSemaphoreGive(latest_downlink_lock);
                    } else {
                        ESP_LOGE(TAG, "Failed to update latest downlink");
                    }

                } else {
                    ESP_LOGE(TAG, "Ignoring report as size is not 33 (was %i)", param->output.length);
                }
            }
            break;
        }
        case ESP_HIDD_FEATURE_EVENT: {
            ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index,
                     esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
        }
        case ESP_HIDD_DISCONNECT_EVENT: {
            ESP_LOGI(TAG, "DISCONNECT: %s",
                     esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev),
                                                   param->disconnect.reason));
            esp_hid_ble_gap_adv_start();
            break;
        }
        case ESP_HIDD_STOP_EVENT: {
            ESP_LOGI(TAG, "STOP");
            break;
        }
        default:
            ESP_LOGW(TAG, "Event: %i", event);
            break;
    }
}

static char ble_name[50];
static char ble_serial[50];

esp_err_t ble_hid_gap_adv_init(uint16_t appearance, const char *device_name) {
    esp_err_t ret;

    const uint8_t hidd_service_uuid128[] = {
            0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
    };

    esp_ble_adv_data_t ble_adv_data = {
            .set_scan_rsp = true,
            .include_name = true,
            .include_txpower = false,
            .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
            .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
            .appearance = appearance,
            .manufacturer_len = 0,
            .p_manufacturer_data =  NULL,
            .service_data_len = 0,
            .p_service_data = NULL,
            .service_uuid_len = sizeof(hidd_service_uuid128),
            .p_service_uuid = (uint8_t *) hidd_service_uuid128,
            .flag = 0x6,
    };

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t key_size = 16; //the key size should be 7~16 bytes

    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param AUTHEN_REQ_MODE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param IOCAP_MODE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_INIT_KEY failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_RSP_KEY failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param MAX_KEY_SIZE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_device_name(device_name)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_device_name failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_config_adv_data(&ble_adv_data)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP config_adv_data failed: %d", ret);
        return ret;
    }

    return ret;
}

void ble_hid_init() {
    esp_err_t ret;

    ble_events = xEventGroupCreateStatic(&ble_events_mem);
    configASSERT(ble_events != NULL);

    ret = ble_hid_gap_init();
    ESP_ERROR_CHECK(ret);

    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gatts_register_callback failed: %d", ret);
        return;
    }

    ret = ble_hid_gap_adv_init(ESP_HID_APPEARANCE_KEYBOARD, ble_hid_config.device_name);
    ESP_ERROR_CHECK(ret);

    ret = esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &hid_dev);


    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ble_hid_init success");
}

void app_main(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t nvs_handle;
    ret = nvs_open("data", NVS_READWRITE, &nvs_handle);
    ESP_ERROR_CHECK(ret);

    uint8_t unpair;
    ret = nvs_get_u8(nvs_handle, "unpair", &unpair);
    if (ret != ESP_OK) {
        unpair = 0;
    }
    if (unpair != 0) {
        nvs_close(nvs_handle);
        nvs_flash_erase();
        esp_restart();
    }

    uint8_t random_mac[8];
    uint8_t static_mac[8];
    size_t random_mac_len = 8;
    size_t static_mac_len = 8;

    ret = nvs_get_blob(nvs_handle, "rand", random_mac, &random_mac_len);
    ESP_LOGI(TAG, "ret: %i", ret);

    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Creating new base MAC");
        esp_read_mac(static_mac, ESP_MAC_BT);
        ret = nvs_set_blob(nvs_handle, "static", static_mac, sizeof(static_mac));
        ESP_ERROR_CHECK(ret);

        esp_fill_random(random_mac, sizeof(random_mac));
        random_mac[0] &= ~1;
        ret = nvs_set_blob(nvs_handle, "rand", random_mac, sizeof(random_mac));
        ESP_ERROR_CHECK(ret);

        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Saved new MAC");
        esp_restart();
    }
    ret = nvs_get_blob(nvs_handle, "static", static_mac, &static_mac_len);
    ESP_ERROR_CHECK(ret);
    nvs_close(nvs_handle);

    esp_base_mac_addr_set(random_mac);

    ESP_LOGI(TAG, "Static MAC: %02X %02X %02X %02X %02X %02X %02X %02X", static_mac[0], static_mac[1], static_mac[2], static_mac[3], static_mac[4], static_mac[5], static_mac[6], static_mac[7]);
    sprintf(ble_name, "c-%x%x%x%x%x%x%x%x", static_mac[0], static_mac[1], static_mac[2], static_mac[3], static_mac[4], static_mac[5], static_mac[6], static_mac[7]);
    sprintf(ble_serial, "%x%x%x%x%x%x", random_mac[0], random_mac[1], random_mac[2], random_mac[3], random_mac[4],
            random_mac[5]);
    ble_hid_config.device_name = ble_name;
    ble_hid_config.serial_number = ble_serial;

    memset(latest_downlink, 0, sizeof(latest_downlink));
    latest_downlink_lock = xSemaphoreCreateMutexStatic(&latest_downlink_lock_buffer);
    configASSERT(latest_downlink_lock);

    spi_and_gpio_init();
    ble_hid_init();

    xTaskCreate(spi_task, "spi_task", 8 * 1024, NULL, configMAX_PRIORITIES - 3,
                NULL);
}
