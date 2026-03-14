#ifndef RGB_CO2_H_
#define RGB_CO2_H_

#include "esp_zigbee_core.h"


/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   1000    /* 1s */
#define HA_COLOR_DIMMABLE_LIGHT_ENDPOINT 10     /* esp light bulb device endpoint, used to process light controlling commands */
#define HA_CUSTOM_CO2_ENDPOINT           12     /* esp co2 device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define CUSTOM_CLUSTER_ID 0xffd0
#define CUSTOM_MANUFACTURER_CODE 0x1307
#define ATTR_STATUS_FLAGS 1
#define ATTR_ABC_HOURS 2
#define ATTR_CO2_BRIGHTNESS 3 /* for setting the LED brightness when displaying the CO2 level */
#define ATTR_CO2_LOW 4 /* configure the "low" co2 level, showing as RGB(0, 255, 0) */
#define ATTR_CO2_MED 5 /* configure the "medium" co2 level, showing as RGB(255, 255, 0) */
#define ATTR_CO2_HIGH 6 /* configure the "high" co2 level, showing as RGB(255, 0, 0) */

/* Basic manufacturer information */
#define MANUFACTURER_NAME "srett"        /* Customized manufacturer name */
#define MODEL_IDENTIFIER "KLOJA-CO2-RGB" /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#endif