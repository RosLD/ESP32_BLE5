#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include <driver/uart.h>

#define LOG_TAG "SCANNER50"

int coun = 0;

uint32_t scan_time = 1;     //Insert time in minutes

uint8_t addr_2m[6] = {0xc0, 0xde, 0x52, 0x00, 0x00, 0x02};
//uint8_t msg[7] = {0xFF,0xE4,'E','N','D',0x0D,0x0A};

#define BUF_SIZE (1024) 
#define UART_PORT_NUM UART_NUM_0
#define rxpin 18
#define txpin 17

static esp_ble_ext_scan_params_t ble_scan_params = {
		
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
        .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
        .coded_cfg = {
            .scan_type              = BLE_SCAN_TYPE_PASSIVE,
            .scan_interval          = 0x0140,
	        .scan_window            = 0x0140
        },
        .uncoded_cfg = {BLE_SCAN_TYPE_PASSIVE, 0xA0, 0xA0},
		
};


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
    uart_set_pin(UART_PORT_NUM,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
	//uart_set_pin(UART_PORT_NUM,txpin,rxpin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

}

uint16_t cuenta = 0;
uint16_t curse = 0;
uint8_t *msg;
int a = 0;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

        case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
            
            //ESP_LOGI(LOG_TAG, "SCAN PARAMS ESTABLISHED, status %d", param->set_ext_scan_params.status);
            a=0;
            //uint8_t a = 3;
            //while (a > 0){
            //    //printf("Sincronizando reloj en: %d\n",a);
            //    a--;
            //    vTaskDelay(1000/portTICK_PERIOD_MS);
            //    
            //}

            esp_ble_gap_start_ext_scan(scan_time*6000,0);    //Time here is multiplied by 10 ms
            break;

        case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
            //printf("Scanning started!\n");
            break;
        

        case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:

            //if(param->period_adv_data_set.status){
            //    printf("YES!\n");
            //}
            //printf("Adv type: %d\n",param->ext_adv_report.params.event_type);    
            a = 0;
            uint8_t *addr = (uint8_t*)param->ext_adv_report.params.addr;
            
            if(memcmp(addr,addr_2m,6)==0){
                //printf("SCAN RESULT =======================\n");
                //adve = (uint8_t*)param->ext_adv_report.params.adv_data; 
                msg[0] = param->ext_adv_report.params.adv_data[13];
                msg[1] = param->ext_adv_report.params.adv_data[14];
                msg[2] = -param->ext_adv_report.params.rssi;
                msg[3] = 0x0D;
                msg[4] = 0x0A;
                //cuenta = ((param->ext_adv_report.params.adv_data[13]<<8) | param->ext_adv_report.params.adv_data[14]);
                
                uart_write_bytes(UART_PORT_NUM, (const char *) &msg[0], 5);

                //if(cuenta == curse){
                //    coun++;
                //}else{
                //    //printf("Para %d se han encontrado %d, tasa: %f\n",curse,coun,(float)coun/10);
                //    printf("%d;%d;%f\n",curse,coun,(float)coun/10);
                //    coun = 1;
                //    curse = cuenta;
                //    
                //}


                //printf("Cuenta actual: %d\n",cuenta);
                //for(int i = 0;i< param->ext_adv_report.params.adv_data_len;i++){
                //    printf("%02X",adve[i]);
                //}
                
                /*free(addr);
                free(adve);*/
            }

            
            
            
            
            break;

        case ESP_GAP_BLE_SCAN_TIMEOUT_EVT:

            //printf("\nSCAN TIMEOUT\n");
            a=0;
            esp_ble_gap_start_ext_scan(scan_time*6000,0);   //Do it again.
            //uart_write_bytes(UART_PORT_NUM, msg, 7);
            break;

        default:
            printf("EVENT: %d\n",event);
            break;
    }
}


void app_main(void)
{
    printf("Hello worlds\n");

    msg = malloc(5);

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
    
    init_uart();

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
