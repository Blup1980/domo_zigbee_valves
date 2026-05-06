/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK   (0x07FFF800U)
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK (0x07FFF800U)

#define ESP_ZIGBEE_HA_FIRST_EP_ID (1)
#define ESP_ZIGBEE_HA_NB_EP (11)

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"

#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER "\x05""VALVE"

#define ESP_ZIGBEE_ROUTER_CONFIG()                       \
    {                                                    \
        .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,       \
        .install_code_policy = false,                    \
        .zczr_config = {                                 \
            .max_children = 10,                          \
        },                                               \
    }

#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
        .radio_config = {                                            \
            .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,              \
        },                                                           \
    }

#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ROUTER_CONFIG(),     \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    };

#define EZB_CUSTOM_POWER_OUTLET_CONFIG()                                              \
    {                                                                              \
        .basic_cfg =                                                               \
            {                                                                      \
                .zcl_version  = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,           \
                .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,              \
            },                                                                     \
        .identify_cfg =                                                            \
            {                                                                      \
                .identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,     \
            },                                                                     \
        .groups_cfg =                                                              \
            {                                                                      \
                .name_support = EZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE,         \
            },                                                                     \
        .scenes_cfg =                                                              \
            {                                                                      \
                .scene_count      = EZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,      \
                .current_scene    = EZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,    \
                .current_group    = EZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,    \
                .scene_valid      = EZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,      \
                .name_support     = EZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE,     \
                .scene_table_size = EZB_ZCL_SCENES_SCENE_TABLE_SIZE_DEFAULT_VALUE, \
            },                                                                     \
        .on_off_cfg = {                                                            \
            .on_off = EZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,                         \
        },                                                                         \
    }
