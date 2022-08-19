#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include <driver/uart.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"


#define LOG_TAG "SCANNER50"

#define tx_pin 17
#define rx_pin 18

//UART Buffer
#define BUF_SIZE (1024)
#define UART_PORT_NUM UART_NUM_2

uint32_t scan_time = 60;     //Insert time in minutes

uint8_t *adve;

uint8_t *msg;

void init_uart() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
	
    uart_param_config(UART_PORT_NUM, &uart_config);
    //uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    //uart_set_pin(UART_PORT_NUM,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
	uart_set_pin(UART_PORT_NUM,tx_pin,rx_pin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

	
}

static esp_ble_ext_scan_params_t ble_scan_params = {
		
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
        .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
        .coded_cfg = {
            .scan_type              = BLE_SCAN_TYPE_ACTIVE,
            .scan_interval          = 0xA0,
	        .scan_window            = 0xA0
        },
        .uncoded_cfg = {BLE_SCAN_TYPE_ACTIVE, 0xA0, 0xA0},
		
};



static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

        case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
            
            ESP_LOGI(LOG_TAG, "SCAN PARAMS ESTABLISHED, status %d", param->set_ext_scan_params.status);

            esp_ble_gap_start_ext_scan(scan_time*6000,0);    //Time here is multiplied by 10 ms
            break;

        case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
            printf("Scanning started!\n");
            break;
        

        case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:

            //if(param->period_adv_data_set.status){
            //    printf("YES!\n");
            //}
            //printf("Adv type: %d\n",param->ext_adv_report.params.event_type);
            
            
            adve = (uint8_t*)param->ext_adv_report.params.adv_data; 
            //coun++;
            if(adve[5]==0xE2 && adve[6]==0xFF){
                //printf("SCAN RESULT =======================\n");   

                memcpy(msg,&adve[9],param->ext_adv_report.params.adv_data_len-9);   
                for(int i = 0;i< param->ext_adv_report.params.adv_data_len-9;i++){
                    printf("%02X",msg[i]);
                }
                printf("\n");
                uart_write_bytes(UART_PORT_NUM, (const char *) msg, param->ext_adv_report.params.adv_data_len-9);

            }
            
            
            
            
            break;

        case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:

            printf("\nSCAN TIMEOUT\n");
            esp_ble_gap_start_ext_scan(scan_time*6000,0);
            
            break;

        default:
            printf("EVENT: %d\n",event);
            break;
    }
}


void app_main(void)
{
    printf("Hello worlds\n");

    init_uart();

    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_ble_gap_set_prefered_default_phy(ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING,ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING);

    msg = malloc(15);
    adve = malloc(255);

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
        ESP_LOGE(LOG_TAG, "gap register error, error code = %x \n", ret);
        return;
    }

    ret = esp_ble_gap_set_ext_scan_params(&ble_scan_params);
    if (ret){
        ESP_LOGE(LOG_TAG, "Param setting error, error code = %x\n", ret);
        return;
    }

}
