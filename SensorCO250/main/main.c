#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
 
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "scd41.h"

#define EXT_ADV_HANDLE      0
#define NUM_EXT_ADV         1

#define LOG_TAG "BLE50"

#define SENSIRION 0x62

uint8_t addr_2m[6] = {0xc0, 0xde, 0x52, 0x00, 0x00, 0x02};


esp_ble_gap_ext_adv_params_t ext_adv_params = {
    //.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_DIRECTED,
    //.type = ESP_BLE_ADV_REPORT_EXT_ADV_IND,
    //.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE,
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
    .interval_min = 0xA0,
    .interval_max = 0xA0,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_CODED,//ESP_BLE_GAP_PHY_CODED,//ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,//ESP_BLE_GAP_PHY_1M,//poner combinaciones (a 1M ppk2 observar consumo)
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    // instance, duration, period
    [0] = {EXT_ADV_HANDLE, 0, 0},
};

uint16_t contador = 0;

//======================================PARAMETROS====================================
uint8_t id = 1;
uint8_t mes = 5;
bool smode = false; //Single-shot mode True -> activated 
//Variables
RTC_DATA_ATTR uint8_t nseq = 0;
 uint16_t co2 = 0;
 float temp,hum = 0;
RTC_DATA_ATTR uint8_t cmed = 0;

RTC_DATA_ATTR uint16_t co2_sum = 0;
RTC_DATA_ATTR float temp_sum,hum_sum = 0;

RTC_DATA_ATTR uint16_t co2_old,temp_old,hum_old = 0;

//====================================================================================
static uint8_t raw_ext_adv_data[] = {
        0x02, 0x01, 0x01,
        0x03, 0x03, 0xE2, 0xFF,
        0x10, 0xff, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};


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
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status %d", param->ext_adv_start.status);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        esp_ble_gap_ext_adv_stop(NUM_EXT_ADV, &ext_adv[0]);
        break;
    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        
        ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT, status %d", param->ext_adv_stop.status);

        if(nseq == 255){
            nseq = 1;
        }else
            nseq++;
        
        go_sleep(60-(mes*5));

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

void bluetooth_on(void)
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

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_N0); //<-- his power level

    uint8_t pwt = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV); //it is at P3 dbm by default(3dBm console logs 10)
    printf("Potencia actual (deberia ser 9) %d \n",pwt);

    esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params);
    
    esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, addr_2m);
    esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_ext_adv_data), &raw_ext_adv_data[0]);
    
    esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv[0]);


  
}

void app_main(void)
{
    printf("ESP32 - SCD41 - UPCT - BLE 5.0\n");

    init_i2c();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if(cause == ESP_SLEEP_WAKEUP_TIMER){
        
        while(!get_ready_status(SENSIRION)){
            vTaskDelay(20);
        }

        do_sensor(SENSIRION,&co2,&temp,&hum);

        printf("======================================\n");
        printf("CO2: %d\n",co2);
        printf("temp: %f\n",temp);
        printf("hum: %f\n",hum);
        printf("======================================\n");
        co2_sum += co2;
        temp_sum += temp;
        hum_sum += hum;

        cmed++;
        printf("Got measurement number: %d\n",cmed);

        if(cmed == mes){

            co2 = (uint16_t)co2_sum/cmed;
            temp = temp_sum/cmed;
            hum = hum_sum/cmed;

            printf("CO2 medio = %d ppm",co2);

            co2_sum = 0;
            temp_sum = 0;
            hum_sum = 0;
            cmed = 0;

            raw_ext_adv_data[9] = id;
            raw_ext_adv_data[10] = nseq;

            //First rotate
            raw_ext_adv_data[17] = (uint8_t)(co2_old>>8);
            raw_ext_adv_data[18] = (uint8_t)co2_old;
            raw_ext_adv_data[19] = (uint8_t)(temp_old>>8);
            raw_ext_adv_data[20] = (uint8_t)temp_old;
            raw_ext_adv_data[21] = (uint8_t)(hum_old>>8);
            raw_ext_adv_data[22] = (uint8_t)hum_old;

            //then
            raw_ext_adv_data[11] = (uint8_t)(co2>>8);
            raw_ext_adv_data[12] = (uint8_t)co2;
            raw_ext_adv_data[13] = (uint8_t)(int)temp;
            raw_ext_adv_data[14] = (uint8_t)(temp-raw_ext_adv_data[13])*256;
            raw_ext_adv_data[15] = (uint8_t)(int)hum;
            raw_ext_adv_data[16] = (uint8_t)(hum-raw_ext_adv_data[15])*256;

            co2_old = co2;
            temp_old = (uint16_t)raw_ext_adv_data[13]<<8|raw_ext_adv_data[14];
            hum_old = (uint16_t)raw_ext_adv_data[15]<<8|raw_ext_adv_data[16];

            bluetooth_on(); //Activate bluetooth and do stuff

        }else{

            if(smode)
                measure_oneshot(SENSIRION);
            vTaskDelay(2);
            go_sleep(5);

        }
    }else{

        if(smode){ //If  single-shot stop periodic measurement

            stop_sensor(SENSIRION);
            measure_oneshot(SENSIRION);

        }else{

            start_sensor(SENSIRION);

        }

        vTaskDelay(2);
        go_sleep(5);

    }

}
