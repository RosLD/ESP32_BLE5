#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include <driver/gpio.h>
 
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define EXT_ADV_HANDLE      0
#define NUM_EXT_ADV         1

#define LOG_TAG "BLE50"

uint8_t addr_2m[6] = {0xc0, 0xde, 0x52, 0x00, 0x00, 0x02};

#define ble_ena GPIO_NUM_21
#define GPIO_OUTPUT_PIN_SEL (1ULL<<ble_ena)
#define ESP_INTR_FLAG_DEFAULT 0

void init_gpiopin(){

	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	
}

  //.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_DIRECTED,
    //.type = ESP_BLE_ADV_REPORT_EXT_ADV_IND,
    //.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE,

esp_ble_gap_ext_adv_params_t ext_adv_params = {
  
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = 0x0140,//0xA0,
    .interval_max = 0x0140,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_CODED,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_CODED,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    // instance, duration, period
    [0] = {EXT_ADV_HANDLE, 0, 0},
};

uint16_t contador = 0;

static uint8_t raw_ext_adv_data[] = {
        0x02, 0x01, 0x01,
        0x0b, 0x09, 'E', 'S', 'P', '3', '2', '_', 'L', 'R',0x0,0x0
};

/*
static esp_ble_gap_periodic_adv_params_t periodic_adv_params = {
    .interval_min = 0x40, // 80 ms interval
    .interval_max = 0x40,
    .properties = 0, // Do not include TX power
};

static uint8_t periodic_adv_raw_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb,
        0x03, 0x03, 0xab, 0xcd,
        0x11, 0x09, 'E', 'S', 'P', '_', 'P', 'E', 'R', 'I', 'O', 'D', 'I',
        'C', '_', 'A', 'D', 'V'
};*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT, status %d", param->ext_adv_set_rand_addr.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT, status %d", param->ext_adv_set_params.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT, status %d", param->ext_adv_data_set.status);
        break;
    case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT, status %d", param->scan_rsp_set.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        gpio_set_level(ble_ena,1);
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status %d", param->ext_adv_start.status);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        esp_ble_gap_ext_adv_stop(NUM_EXT_ADV, &ext_adv[0]);
        break;
    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        gpio_set_level(ble_ena,0);
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT, status %d", param->ext_adv_stop.status);
        contador++;
        raw_ext_adv_data[13] = (uint8_t)(contador >> 8);
        raw_ext_adv_data[14] = (uint8_t)contador;
        vTaskDelay(500/portTICK_PERIOD_MS);
        esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE,sizeof(raw_ext_adv_data),&raw_ext_adv_data[0]);
        esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv[0]);

        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT, status %d", param->peroid_adv_set_params.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT, status %d", param->period_adv_data_set.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT, status %d", param->period_adv_start.status);
        break;
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_ble_gap_set_prefered_default_phy(ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING,ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    init_gpiopin();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(LOG_TAG, "gap register error, error code = %x", ret);
        return;
    }

    //uint8_t pwt = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV); it is at P3 dbm (3dBm console logs 10)
    //printf("Potencia actual %d \n",pwt);
//
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_N0); //<-- his power level
    vTaskDelay(1);
    uint8_t pwt = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV); //it is at P3 dbm (3dBm console logs 10)
    printf("Potencia actual %d deberia dar 9\n",pwt);

    esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params);
    
    esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, addr_2m);
    esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_ext_adv_data), &raw_ext_adv_data[0]);
    //esp_ble_gap_config_ext_scan_rsp_data_raw(EXT_ADV_HANDLE,sizeof(raw_ext_adv_data),&raw_ext_adv_data[0]);
    esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv[0]);


    //esp_ble_gap_periodic_adv_set_params(EXT_ADV_HANDLE, &periodic_adv_params);
    //esp_ble_gap_config_periodic_adv_data_raw(EXT_ADV_HANDLE, sizeof(periodic_adv_raw_data), &periodic_adv_raw_data[0]);
    //esp_ble_gap_periodic_adv_start(EXT_ADV_HANDLE);
}
