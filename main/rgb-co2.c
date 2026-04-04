/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Based on:
 * Zigbee HA_color_dimmable_light Example
 * and:
 * https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.c
 */

#include "rgb-co2.h"
#include "util/light_driver.h"
#include "util/zb-core.h"
#include "util/esp-util.h"

#include <string.h>

#include "esp_log.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "esp_pm.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_BUF_SIZE 256
#define RECV_BUF_SIZE 32
#define PIN_TX 19
#define PIN_RX 20

#define LED_CO2_LEVEL 30

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig
#endif

/** By how much the ppm value (zb converted) has to change to be reported */
static const float_t MIN_PPM_CHANGE_REPORTED = 10.0f / 1000000.0f;

// Request Status + CO2-Value
static char ALLreq[] = {0xFE, 0x04, 0x00, 0x00, 0x00, 0x04, 0xE5, 0xC6};
// Request ABC-Interval in [h] 1f in Hex -> 31 dezimal
static char ABCreq[] = {0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0xC3};
// Set ABC interval to 180 hours
static char ABCset[] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0xB4, 0xAC, 0x74};
// Set ABC interval to 0 hours (disable)
//static char ABCdisable[] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03};
static char FWreq[] = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};    // Readout FirmWareversion
static uint8_t recv_buffer[RECV_BUF_SIZE];
// For suspending and resuming the blink task
static TaskHandle_t taskBlink = NULL;

/** True if zigbee stack has been fully initialized */
static bool zigbee_ok = false;
/** Set to > 0 to enable countdown for writing config to flash */
static uint32_t write_persistent_countdown = 0;

#define schedule_persistent_write() do { write_persistent_countdown = 2; } while(0)

#define DATA_V1_SIZE 20
struct persistent_data
{
    uint16_t struct_size;
    uint16_t co2_low, co2_med, co2_high;
    uint16_t led_x, led_y;
    uint8_t led_level;
    uint8_t co2_led_level;
    uint16_t padding;
    uint32_t write_count;
};
_Static_assert(DATA_V1_SIZE == sizeof(struct persistent_data),
    "Latest persistent_data struct size messup");

static struct {
    uint16_t x, y;
    uint8_t red, green, blue;
    uint8_t level;
    uint8_t co2_level;
    bool on;
} led = {.co2_level = LED_CO2_LEVEL};

// Sanity check reported sensor value. I've seen crazy high values twice in about a year now...
#define CO2_IGNORE_BELOW 380
#define CO2_IGNORE_ABOVE 3000
// Delay zigbee init, to make sure we have stable CO2 values before we start reporting anything
#define ZB_INIT_DELAY_MS 20000
// Number of probes to pick minimum value from
#define CO2_HIST_LEN 6
// Delay between CO2 sensor polls. The sensor updates its value every 4 secs, so this should >= 4
#define CO2_POLL_INTERVAL_MS 4500
static struct {
    float_t zb_value;
    uint16_t hist[CO2_HIST_LEN];
    uint16_t abc_hours;
    uint16_t status;
    uint16_t co2_low;
    uint16_t co2_med;
    uint16_t co2_high;
    uint8_t histpos;
} co2 = {
    .co2_low = 450,
    .co2_med = 900,
    .co2_high = 1500,
};

// If you violate this you might get unexpected results
#if CO2_HIST_LEN > 255
#error CO2_HIST_LEN must be < 256
#endif

static void write_persistent_data(void);


esp_err_t zbcb_configure_reporting(void)
{
    const char *TAG = "REPORTING";
    esp_err_t ret;
    ESP_LOGI(TAG, "Configuring reporting");
    /* CO2: Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_CUSTOM_CO2_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .u.send_info.min_interval = 15,
        .u.send_info.max_interval = 600,
        .u.send_info.def_min_interval = 15,
        .u.send_info.def_max_interval = 600,
        .u.send_info.delta.f32 = MIN_PPM_CHANGE_REPORTED,
        .attr_id = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGIW(TAG, ret, "Reporting config co2");

    /* STATUS: Config the reporting info  */
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_CUSTOM_CO2_ENDPOINT,
        .cluster_id = CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 0,
        .attr_id = ATTR_STATUS_FLAGS,
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGIW(TAG, ret, "Reporting config STATUS");

    /* ABC HOURS: Config the reporting info  */
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_CUSTOM_CO2_ENDPOINT,
        .cluster_id = CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 0,
        .attr_id = ATTR_ABC_HOURS,
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGIW(TAG, ret, "Reporting config ABC HOURS");

    return ESP_OK;
}

void zbcb_initialized(esp_err_t err_status, bool factory_reset)
{
    if ( err_status != ESP_OK )
        return;
    zigbee_ok = true;
}

static void sanitize_co2_thresholds()
{
    co2.co2_low = util_clamp(co2.co2_low, 400, 10000);
    co2.co2_med = util_clamp(co2.co2_med, 400, 10000);
    co2.co2_high = util_clamp(co2.co2_high, 400, 10000);

    if (co2.co2_low >= co2.co2_med) {
        co2.co2_med = co2.co2_low + 1;
    }
    if (co2.co2_med >= co2.co2_high) {
        co2.co2_high = co2.co2_med + 1;
    }

    if (co2.co2_high > 10000) {
        co2.co2_high = 10000;
        if (co2.co2_med >= co2.co2_high) {
            co2.co2_med = co2.co2_high - 1;
        }
        if (co2.co2_low >= co2.co2_med) {
            co2.co2_low = co2.co2_med - 1;
        }
    }
}

/**
 * Handle light endpoint.
 */
void zb_handle_light(uint16_t cluster, const esp_zb_zcl_attribute_t *attr)
{
    const char *TAG = "SETATTR_LIGHT";

    switch ( cluster ) {
    case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
        if ( attr->id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID ) {
            if ( PTR_TO_BOOL( led.on, attr->data ) ) {
                ESP_LOGI( TAG, "Light set to %s", led.on ? "On" : "Off" );
                if ( led.on ) {
                    vTaskResume( taskBlink );
                }
                schedule_persistent_write();
            }
        } else {
            ESP_LOGW( TAG, "Unknown On/Off cluster data: attribute(0x%x), type(0x%x)", attr->id,
                attr->data.type );
        }
        break;
    case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
        if ( attr->id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID ) {
            if ( PTR_TO_UINT16( led.x, attr->data ) ) {
                ESP_LOGI( TAG, "Light color x changes to 0x%x", led.x );
                schedule_persistent_write();
            }
        } else if ( attr->id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID ) {
            if ( PTR_TO_UINT16( led.y, attr->data ) ) {
                ESP_LOGI( TAG, "Light color y changes to 0x%x", led.y );
                schedule_persistent_write();
            }
        } else {
            ESP_LOGW( TAG, "Unknown Color Control cluster data: attribute(0x%x), type(0x%x)", attr->id,
                attr->data.type );
        }
        break;
    case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
        if ( attr->id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID ) {
            if ( PTR_TO_UINT8( led.level, attr->data ) ) {
                ESP_LOGI( TAG, "Light level changes to %d", led.level );
                schedule_persistent_write();
            }
        } else {
            ESP_LOGW( TAG, "Unknown Level Control cluster data: attribute(0x%x), type(0x%x)", attr->id,
                attr->data.type );
        }
        break;
    default:
        ESP_LOGW( TAG, "Unknown cluster(0x%x), attribute(0x%x)  ", cluster, attr->id );
    }
}

void zb_handle_co2(uint16_t cluster, const esp_zb_zcl_attribute_t *attr)
{
    const char *TAG = "SETATTR_CO2";

    if ( cluster != CUSTOM_CLUSTER_ID ) {
        ESP_LOGW( TAG, "Unknown cluster(0x%x), attribute(0x%x)  ", cluster, attr->id );
        return;
    }
    if ( attr->id == ATTR_CO2_BRIGHTNESS ) {
        if ( PTR_TO_UINT8( led.co2_level, attr->data ) ) {
            ESP_LOGI( TAG, "CO2 LED brightness changes to %d", led.co2_level );
            schedule_persistent_write();
        }
    } else if ( attr->id == ATTR_CO2_LOW ) {
        if ( PTR_TO_UINT16( co2.co2_low, attr->data ) ) {
            sanitize_co2_thresholds();
            ESP_LOGI( TAG, "CO2 low threshold changes to %d (med: %d, high: %d)", co2.co2_low, co2.co2_med,
                co2.co2_high );
            schedule_persistent_write();
        }
    } else if ( attr->id == ATTR_CO2_MED ) {
        if ( PTR_TO_UINT16( co2.co2_med, attr->data ) ) {
            sanitize_co2_thresholds();
            ESP_LOGI( TAG, "CO2 med threshold changes to %d (low: %d, high: %d)", co2.co2_med, co2.co2_low,
                co2.co2_high );
            schedule_persistent_write();
        }
    } else if ( attr->id == ATTR_CO2_HIGH ) {
        if ( PTR_TO_UINT16( co2.co2_high, attr->data ) ) {
            sanitize_co2_thresholds();
            ESP_LOGI( TAG, "CO2 high threshold changes to %d (low: %d, med: %d)", co2.co2_high, co2.co2_low,
                co2.co2_med );
            schedule_persistent_write();
        }
    } else {
        ESP_LOGW( TAG, "Unknown CO2 cluster data: attribute(0x%x), type(0x%x)", attr->id,
                attr->data.type );
    }
}

esp_err_t zbcb_attribute_set_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    const char *TAG = "ATTRIBHNDL";
    esp_err_t ret = ESP_OK;

    if (message->info.dst_endpoint == HA_COLOR_DIMMABLE_LIGHT_ENDPOINT) {
        zb_handle_light( message->info.cluster, &message->attribute );

    } else if (message->info.dst_endpoint == HA_CUSTOM_CO2_ENDPOINT) {
        zb_handle_co2( message->info.cluster, &message->attribute );
    } else {
        ESP_LOGW(TAG, "Unknown endpoint(0x%x) message data: cluster(0x%x), attribute(0x%x)  ", message->info.dst_endpoint, message->info.cluster, message->attribute.id);
    }
    return ret;
}

static void setLedCo2(int ppm)
{
    if (ppm != 0) {
        if (ppm <= co2.co2_low) {
            led.red = 0;
            led.green = 255;
        } else if (ppm >= co2.co2_high) {
            led.red = 255;
            led.green = 0;
        } else if (ppm <= co2.co2_med) {
            led.red = (uint8_t)(255 * (ppm - co2.co2_low) / (co2.co2_med - co2.co2_low));
            led.green = 255;
        } else {
            led.red = 255;
            led.green = (uint8_t)(255 * (co2.co2_high - ppm) / (co2.co2_high - co2.co2_med));
        }
        led.blue = 0;
    }
    light_driver_set_color_RGB(led.red, led.green, led.blue);
    light_driver_set_level(led.co2_level);
}

/**
 * Task that makes the LED alternate between the configured color/brightness of
 * the LED bulb zigbee config, and the CO2 level display.
 * Task is only running if lamp is on.
 */
static void blink_task(void *arg)
{
    const char *TAG = "BLINK_TASK";
    while (1) {
        while (!led.on) {
            ESP_LOGW(TAG, "Going to sleep!");
            vTaskSuspend(NULL);
            ESP_LOGW(TAG, "Woke up for blinking!");
        }
        light_driver_set_color_xy(led.x, led.y);
        light_driver_set_level(led.level);
        ESP_LOGW(TAG, "Pre-delay 200");
        SLEEP_MS( 200 );
        ESP_LOGW(TAG, "Post-delay 200");
        setLedCo2( 0 );
        if (led.on) {
            ESP_LOGW(TAG, "Pre-delay 800");
            SLEEP_MS( 800 );
            ESP_LOGW(TAG, "Post-delay 800");
        }
    }
}

esp_err_t zbcb_register_device(void)
{
    const char *TAG = "ZB_REGDEV";
    esp_err_t ret;

        // Light endpoint
    esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    esp_zb_ep_list_t *endpoint_list = esp_zb_color_dimmable_light_ep_create(
            HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, &light_cfg);

    zbc_add_manufacturer_model(endpoint_list, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
            MANUFACTURER_NAME, MODEL_IDENTIFIER);

    // CO2 sensor endpoint
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_config = {
        .measured_value = co2.zb_value,
        .min_measured_value = CO2_IGNORE_BELOW / 1000000.0f,
        .max_measured_value = CO2_IGNORE_ABOVE / 1000000.0f,
    };
    esp_zb_attribute_list_t *esp_zb_carbon_dioxide_meas_cluster
        = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_config);

    // create cluster lists for this endpoint
    esp_zb_cluster_list_t *co2_ep_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(co2_ep_cluster_list,
            esp_zb_carbon_dioxide_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    // Status
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_STATUS_FLAGS, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &co2.status);
    ESP_LOGIW(TAG, ret, "Adding status flags to custom");
    // ABC hours
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_ABC_HOURS, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &co2.abc_hours);
    ESP_LOGIW(TAG, ret, "Adding abc hours to custom");
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_CO2_BRIGHTNESS, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U8,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &led.co2_level);
    ESP_LOGIW(TAG, ret, "Adding co2 brightness to custom");
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_CO2_LOW, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &co2.co2_low);
    ESP_LOGIW(TAG, ret, "Adding co2 low to custom");
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_CO2_MED, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &co2.co2_med);
    ESP_LOGIW(TAG, ret, "Adding co2 med to custom");
    ret = esp_zb_cluster_add_manufacturer_attr(custom_cluster,
        CUSTOM_CLUSTER_ID, ATTR_CO2_HIGH, CUSTOM_MANUFACTURER_CODE,
            ESP_ZB_ZCL_ATTR_TYPE_U16,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &co2.co2_high);
    ESP_LOGIW(TAG, ret, "Adding co2 high to custom");
    // Assign our manufacturer ID to custom cluster
    zbc_set_cluster_manufacturer(co2_ep_cluster_list, CUSTOM_CLUSTER_ID, CUSTOM_MANUFACTURER_CODE);
    // Add custom cluster to endpoint
    ret = esp_zb_cluster_list_add_custom_cluster(co2_ep_cluster_list, custom_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGIW(TAG, ret, "Adding custom cluster");

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_CUSTOM_CO2_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(endpoint_list, co2_ep_cluster_list, endpoint_config);
    // Add entire EP list, light + co2
    esp_zb_device_register(endpoint_list);

    co2.status = 0xff;
    co2.abc_hours = 0xffff;

    return ESP_OK;
}

static void esp_zb_task(void *pvParameters)
{
    const char *TAG = "ZB_TASK";

    ESP_LOGI(TAG, "Waiting %dms before initializing zigbee stack", ZB_INIT_DELAY_MS);
    // Wait here to make sure we have valid values for co2ppm, abc_hours
    // and status_flags right from the start.
    vTaskDelay(ZB_INIT_DELAY_MS / portTICK_PERIOD_MS);
    // initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();

    zbc_main(&zb_nwk_cfg, ESP_ZB_PRIMARY_CHANNEL_MASK);
}

static uint16_t modbus_crc(const unsigned char * buf, int len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];   // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {         // Loop over each bit
      if ((crc & 0x0001) != 0) {           // If the LSB is set
        crc >>= 1;                         // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

static int co2_send(const char* data, int bytes)
{
    const char *TAG = "UART_SEND";
    uart_flush(UART_NUM_1); // In case anything unsolicited/left over is present
    const int txBytes = uart_write_bytes(UART_NUM_1, data, bytes);
    if (txBytes != bytes) {
        ESP_LOGW(TAG, "send failed");
        return 0;
    }
    SLEEP_MS( 100 ); // Wait 100 ms for reply
    return true;
}

static int co2_recv(int bytes)
{
    const char *TAG = "UART_RECV";
    int rxBytes = uart_read_bytes(UART_NUM_1, recv_buffer, RECV_BUF_SIZE, 250 / portTICK_PERIOD_MS);
    if (rxBytes < bytes + 2) {
        ESP_LOGW(TAG, "Received only %d bytes, wanted %d", rxBytes, bytes + 2);
        for (int i = 0; i < rxBytes; ++i) {
            ESP_LOGI(TAG, "Byte: %02X", (int)recv_buffer[i]);
        }
        return 0;
    }
    uint16_t ReadCRC = (uint16_t)recv_buffer[bytes + 1] * 256 + (uint16_t)recv_buffer[bytes];
    uint16_t DataCRC = modbus_crc(recv_buffer, bytes);
    if (DataCRC == ReadCRC) {
        return 1;
    }
    ESP_LOGE(TAG, "CRC-Error: %d != %d", ReadCRC, DataCRC);
    for (int i = 0; i < rxBytes; ++i) {
        ESP_LOGI(TAG, "Byte: %02X", (int)recv_buffer[i]);
    }
    return 0;
}

static void co2_task(void *arg)
{
    const char *TAG = "CO2_TASK";
    float_t ppm;
    uint16_t status;
    uint8_t loopCounter = 0;
    bool doAbc = true;
    bool doUpdate = false;

    // Give sensor time to power up
    SLEEP_MS( 1000 );
    while (!co2_send(FWreq, 8) || !co2_recv(5)) {
        ESP_LOGI(TAG, "Could not query firmware version, sensor not up? Retrying...");
        SLEEP_MS( 1000 );
    }
    ESP_LOGI(TAG, "SenseAir S8 firmware version: %d.%d", recv_buffer[3], recv_buffer[4]);
    ESP_LOGI(TAG, "Sending initial ABC status request");
    SLEEP_MS( 250 );
    if (!co2_send(ABCreq, 8) || !co2_recv(5)) {
        ESP_LOGW(TAG, "REQUEST FAILED");
    } else {
        co2.abc_hours = (recv_buffer[3] * 256) + recv_buffer[4];
        ESP_LOGI(TAG, "Hours: %d", co2.abc_hours);
        if (co2.abc_hours == 0) {
            // No interval set, let's reset to enable
            ESP_LOGW(TAG, "ABC seems disabled, setting to 180h");
            do {
                SLEEP_MS( 2500 );
            } while (!co2_send(ABCset, 8) || !co2_recv(6));
            co2.abc_hours = (recv_buffer[4] * 256) + recv_buffer[5]; // !! 4+5 in this reply
        }
    }
    SLEEP_MS( 250 );
    light_driver_set_color_RGB(0, 0, 255);
    light_driver_set_power(true);
    // Mainloop start
    while (1) {
        loopCounter++;
        ESP_LOGI(TAG, "Sending CO2 request");
        if (co2_send(ALLreq, 8) && co2_recv(11)) {
            uint16_t rval = (recv_buffer[9] * 256) + recv_buffer[10];
            if ( rval >= CO2_IGNORE_BELOW && rval <= CO2_IGNORE_ABOVE ) {
                // Value just read is in plausible range
                co2.hist[++co2.histpos % CO2_HIST_LEN] = rval;
            }
            rval = 65535;
            // Use the lowest value from history to avoid sudden fluctuations from someone
            // breathing at the sensor. This can easily cause very extreme spikes.
            // I used average first, but this seems better, as the natural increase
            // over time with closed windows is rather slow, so a maximum delay of
            // HISTLEN*POLLINTERVAL seems acceptable.
            for (int i = 0; i < CO2_HIST_LEN; ++i) {
                if (co2.hist[i] == 0)
                    continue;
                if (co2.hist[i] < rval) {
                    rval = co2.hist[i];
                }
            }
            if ( rval >= CO2_IGNORE_BELOW && rval <= CO2_IGNORE_ABOVE ) {
                ppm = ((float_t) rval) / 1000000.0f;
                status = (recv_buffer[3] * 256) + recv_buffer[4];
                if ( status != co2.status ) {
                    co2.status = status;
                    doUpdate = true;
                }
                if ( ppm - co2.zb_value > MIN_PPM_CHANGE_REPORTED || co2.zb_value - ppm > MIN_PPM_CHANGE_REPORTED ) {
                    co2.zb_value = ppm;
                    doUpdate = true;
                }
                setLedCo2(rval);
                ESP_LOGI(TAG, "CO2: %dppm, Status flags: %d", rval, co2.status);
            }
        }
        if ( loopCounter == 1 ) {
            // Re-read abc hours every now and then (TODO: Make settable)
            SLEEP_MS( 250 );
            ESP_LOGI( TAG, "Sending ABC status request" );
            if ( co2_send( ABCreq, 8 ) && co2_recv( 5 ) ) {
                co2.abc_hours = (recv_buffer[3] * 256) + recv_buffer[4];
                uint16_t ddays = co2.abc_hours / 24;
                uint16_t dhours = co2.abc_hours % 24;
                ESP_LOGI( TAG, "ABC interval: %dd%dh", ddays, dhours );
                doAbc = true;
            }
        }
        if ( zigbee_ok && ( doUpdate || doAbc ) ) {
            esp_zb_zcl_status_t ret;

            esp_zb_lock_acquire( portMAX_DELAY );
            if ( doUpdate ) {
                doUpdate = false;
                ret = esp_zb_zcl_set_attribute_val( HA_CUSTOM_CO2_ENDPOINT,
                    ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &co2.zb_value,
                    false );
                zbc_handle_state_err_locked( TAG, "ppm", ret );
                ret = esp_zb_zcl_set_manufacturer_attribute_val( HA_CUSTOM_CO2_ENDPOINT,
                    CUSTOM_CLUSTER_ID,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    CUSTOM_MANUFACTURER_CODE,
                    ATTR_STATUS_FLAGS, &co2.status,
                    false );
                zbc_handle_state_err_locked( TAG, "status flags", ret );
            }
            if ( doAbc ) {
                doAbc = false;
                ret = esp_zb_zcl_set_manufacturer_attribute_val( HA_CUSTOM_CO2_ENDPOINT,
                    CUSTOM_CLUSTER_ID,
                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    CUSTOM_MANUFACTURER_CODE,
                    ATTR_ABC_HOURS, &co2.abc_hours,
                    false );
                zbc_handle_state_err_locked( TAG, "ABC hours", ret );
            }
            esp_zb_lock_release();
        }
        vTaskDelay(CO2_POLL_INTERVAL_MS / portTICK_PERIOD_MS);
        // Maybe move to own task? Currently the timing is a bit opaque
        if (write_persistent_countdown != 0 && --write_persistent_countdown == 0) {
            write_persistent_data();
        }
    }
}

static nvs_handle_t nvs_storage;

/** Keep track of how many times we wrote persistent data */
static uint32_t persistent_writes_count = 0;

static void write_persistent_data(void)
{
    const char *TAG = "WRITE_PERSISTENT";
    static struct persistent_data last_data = {};
    struct persistent_data new_data = {
        .struct_size = sizeof(struct persistent_data),
        .led_x = led.x,
        .led_y = led.y,
        .led_level = led.level,
        .co2_high = co2.co2_high,
        .co2_med = co2.co2_med,
        .co2_low = co2.co2_low,
        .co2_led_level = led.co2_level,
    };

    if (memcmp(&last_data, &new_data, sizeof(struct persistent_data)) == 0)
        return; // Nothing to do
    memcpy(&last_data, &new_data, sizeof(struct persistent_data));
    // Do this after copying, otherwise the struct will always have changed
    new_data.write_count = ++persistent_writes_count;
    ESP_ERROR_CHECK(nvs_set_blob(nvs_storage, "config", &new_data, sizeof(new_data)));
    ESP_ERROR_CHECK(nvs_commit(nvs_storage));
    ESP_LOGI(TAG, "Wrote persistent data, count %" PRIu32, persistent_writes_count);
}

static void load_persistent_data(void)
{
    const char *TAG = "LOAD_PERSISTENT";
    struct persistent_data data;

    size_t size = sizeof(data);
    esp_err_t err = nvs_get_blob(nvs_storage, "config", &data, &size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load persistent data, using defaults (%zu/%zu)", size, sizeof(data));
        write_persistent_countdown = 1;
        return;
    }
    if (size < DATA_V1_SIZE) {
        ESP_LOGE(TAG, "Invalid persistent data size %zu, expecting at least %u, using defaults", size, DATA_V1_SIZE);
        write_persistent_countdown = 1;
        return;
    }
    if (data.struct_size < DATA_V1_SIZE) {
        ESP_LOGW(TAG, "Invalid persistent data version %u, expecting at least %u, using defaults", data.struct_size, DATA_V1_SIZE);
        write_persistent_countdown = 1;
        return;
    }
    led.x = data.led_x;
    led.y = data.led_y;
    led.level = data.led_level;
    co2.co2_high = data.co2_high;
    co2.co2_med = data.co2_med;
    co2.co2_low = data.co2_low;
    led.co2_level = data.co2_led_level;
    persistent_writes_count = data.write_count;
    ESP_LOGI(TAG, "Loaded existing config, write count at %" PRIu32, persistent_writes_count);
}

/**
 * Main
 */
void app_main(void)
{
    util_power_save_init();
    // Disable blue LED
    util_pull_low( PIN_BLUE_LED );
    // Open storage
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition("my_storage"));
    ESP_ERROR_CHECK(nvs_open_from_partition(
            "my_storage", "root",
            NVS_READWRITE, &nvs_storage));
    load_persistent_data();
    zbc_maybe_wipe_zigbee_config(nvs_storage);
    // Setup UART for SenseAir S8
    ESP_ERROR_CHECK(util_default_uart(UART_NUM_1, PIN_RX, PIN_TX, UART_BUF_SIZE));
    // Zigbee
    esp_zb_platform_config_t config = {
        .radio_config.radio_mode =  ZB_RADIO_MODE_NATIVE,
        .host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    light_driver_init(LIGHT_DEFAULT_OFF);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(co2_task, "CO2_main", 4096, NULL, 4, NULL);
    xTaskCreate(blink_task, "blink_main", 2048, NULL, 3, &taskBlink);
}
