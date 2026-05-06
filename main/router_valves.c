/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "light_driver.h"
#include "alarm_timer.h"
#include "valve_driver.h"

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "router_valves.h"

static const char *TAG = "ROUTER_VALVES";

/* Keep Zigbee endpoint config and valve driver count in lockstep. */
_Static_assert(ESP_ZIGBEE_HA_NB_EP == VALVE_COUNT, "ESP_ZIGBEE_HA_NB_EP must match VALVE_COUNT");

esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;

    if (is_inited) {
        return ESP_OK;
    }

    /* Initialize light driver (left in place) and valve driver */
    light_driver_init(ESP_ZIGBEE_HA_NB_EP);
    esp_err_t err = valve_driver_init(false); /* default closed */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "valve_driver_init failed: %s", esp_err_to_name(err));
        return err;
    }

    is_inited = true;

    return ESP_OK;
}

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "Device first startup");
        /* no break, continue to reboot handling */
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            esp_err_t err = deferred_driver_init();
            ESP_LOGI(TAG, "Deferred driver initialization %s", (err == ESP_OK) ? "successful" : "failed");
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Deferred driver init error: %s", esp_err_to_name(err));
            }
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ezb_extpanid_t extended_pan_id;
                ezb_nwk_get_extended_panid(&extended_pan_id);
                ESP_LOGI(TAG, "Already joined network: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                         ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), retry again", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
        } else {
            ESP_LOGW(TAG, "Failed to join network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration) {
            ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), duration);
        } else {
            ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
        }
    } break;
    default:
        ESP_LOGI(TAG, "Unhandled Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static void zcl_core_set_attr_value_handler(ezb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, , TAG, "message is empty");
    ESP_LOGD(TAG, "ZCL SetAttributeValue endpoint(%d) cluster(0x%04x) %s status(0x%02x)", message->info.dst_ep,
             message->info.cluster_id, message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client",
             message->info.status);

    /* We only act on server-side writes. */
    if (message->info.cluster_role != EZB_ZCL_CLUSTER_SERVER) {
        return;
    }

    switch (message->info.cluster_id) {
    case EZB_ZCL_CLUSTER_ID_ON_OFF: {
        uint8_t ep = message->info.dst_ep;
        if (ep < ESP_ZIGBEE_HA_FIRST_EP_ID || ep > (ESP_ZIGBEE_HA_FIRST_EP_ID + ESP_ZIGBEE_HA_NB_EP - 1)) {
            return;
        }
        if (message->in.attribute.id != EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            ESP_LOGW(TAG, "Unsupported attribute ID(0x%04x) for On/Off cluster", message->in.attribute.id);
            return;
        }
        if (message->in.attribute.data.value == NULL) {
            ESP_LOGW(TAG, "On/Off attribute value is NULL (ep=%u)", ep);
            return;
        }

        uint8_t on = *(uint8_t *)message->in.attribute.data.value;
        uint8_t valve_index = ep - ESP_ZIGBEE_HA_FIRST_EP_ID; /* Valve 1 -> index 0 */

        esp_err_t err = valve_driver_set_power(valve_index, on != 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set valve %u state: %s", (unsigned)(valve_index + 1), esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "Set Valve %u On/Off: %u", (unsigned)(valve_index + 1), (unsigned)on);
        return;
    } break;
    default:
        ESP_LOGW(TAG, "Unsupported cluster ID(0x%04x)", message->info.cluster_id);
    }
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        zcl_core_set_attr_value_handler(message);
        break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        ESP_LOGD(TAG, "Received ZCL Default Response status(0x%02x)", default_rsp->in.status_code);
    } break;
    default:
        ESP_LOGW(TAG, "ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

esp_err_t esp_zigbee_create_valve_devices(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    ezb_af_node_power_desc_t power_desc = {
        .current_power_mode = 0,         /* always in full power mode */
        .available_power_sources = EZB_AF_NODE_POWER_SOURCE_CONSTANT_POWER, /* mains power available */
        .current_power_source = EZB_AF_NODE_POWER_SOURCE_CONSTANT_POWER,    /* currently using mains power */
        .current_power_source_level = 0, /* n/a for mains */
    };
    ESP_ERROR_CHECK(ezb_af_set_node_power_desc(&power_desc));

    ezb_zha_mains_power_outlet_config_t outlet_cfg = EZB_CUSTOM_POWER_OUTLET_CONFIG();

    for (uint8_t ep = ESP_ZIGBEE_HA_FIRST_EP_ID; ep <= ESP_ZIGBEE_HA_FIRST_EP_ID + ESP_ZIGBEE_HA_NB_EP - 1; ++ep) {
        ESP_LOGI(TAG, "Creating valve device for endpoint %d...", ep);
        ezb_af_ep_desc_t ep_desc = ezb_zha_create_mains_power_outlet(ep, &outlet_cfg);
        if (ep_desc == NULL) {
            ESP_LOGE(TAG, "Failed to create endpoint %d", ep);
            return ESP_FAIL;
        }

        if (ep == ESP_ZIGBEE_HA_FIRST_EP_ID) {
            ezb_zcl_cluster_desc_t basic_desc = ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
            if (basic_desc) {
                ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
                ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);
            } else {
                ESP_LOGW(TAG, "Endpoint %u missing Basic cluster; manufacturer/model not set for primary endpoint", (unsigned)ep);
            }
        }

        ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    }

    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    ESP_LOGI(TAG, "Registering ZCL core action handler...");
    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_valve_devices());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}



void valve_changed_callback(uint8_t valve_index, valve_state_t new_state)
{
    color_light_color_t color;
    switch (new_state) {
    case VALVE_STATE_CLOSED:
        ESP_LOGI(TAG, "Reporting valve %d state: Closed", valve_index + 1);
        color = COLOR_LIGHT_RED();
        light_driver_set_color(valve_index, color);
        break;
    case VALVE_STATE_OPENING:
        ESP_LOGI(TAG, "Reporting valve %d state: Opening", valve_index + 1);
        color = COLOR_LIGHT_YELLOW();
        light_driver_set_color(valve_index, color);
        break;
    case VALVE_STATE_OPEN:
        ESP_LOGI(TAG, "Reporting valve %d state: Open", valve_index + 1);
        color = COLOR_LIGHT_GREEN();
        light_driver_set_color(valve_index, color);
        break;
    case VALVE_STATE_PENDING:
        ESP_LOGI(TAG, "Reporting valve %d state: Pending (queued to open)", valve_index + 1);
        color = COLOR_LIGHT_BLUE();
        light_driver_set_color(valve_index, color);
        break;
    default:
        ESP_LOGW(TAG, "Reporting valve %d state: Unknown(%d)", valve_index + 1, new_state);
        break;
    }
}

static esp_err_t init_nvs_with_recovery(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init failed (%s), erasing and retrying", esp_err_to_name(err));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase failed");
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs_with_recovery());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
    ESP_LOGI(TAG, "Start ESP Zigbee Stack");
    BaseType_t ok = xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Zigbee task");
    }
}
