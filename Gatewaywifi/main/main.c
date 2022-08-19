#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include <driver/uart.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define tx_pin 17
#define rx_pin 16


static const char *TAG = "uart_events";
static QueueHandle_t uart2_queue;
TaskHandle_t Task1;


#define espip 130
#define EXAMPLE_ESP_WIFI_SSID      "IoTUT"
#define EXAMPLE_ESP_WIFI_PASS      "vp:tppsd44"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define MQTT_SERVER "mqtt://212.128.44.50:1883"

esp_mqtt_client_handle_t client;
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

char topic[] = "CRAIUPCT_co2";
char id[] = "esp_sen_1";


static void uart2_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        
        if(xQueueReceive(uart2_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_2);

            switch(event.type) {
                //Event of UART receving data
                
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    
                    uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
                    esp_mqtt_client_publish(client,topic,(const char *) &dtmp[0],event.size,0,0);
					//for(int i = 0;i<event.size;i++){
                    //    printf("%02X",dtmp[i]);
                    //}
                    //printf("\n");
                   
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    //ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
        
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    //ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart2_queue);
                    break;
            
                //Others
                default:
                    //ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
        
    }   
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);

}



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
	

	uart_param_config(UART_NUM_2, &uart_config);
							//TX:16	RX:17
    uart_set_pin(UART_NUM_2,tx_pin,rx_pin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_2, BUF_SIZE * 2,  BUF_SIZE * 2, 20, &uart2_queue, 0);
    xTaskCreate(uart2_event_task, "uart2_event_task", 2048, NULL, 12, NULL);
	printf("Initiated UART 2\n");

    
	
}

//WIFI FUNCTIONS --------------------------------------------
//mqtt callback
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    //esp_mqtt_client_handle_t client = event->client;
    
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("Conectado al server mqtt\n");
            init_uart();
		   
        	break;
        
        case MQTT_EVENT_ERROR:
            
            break;
		
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        	break;
    }
    return ESP_OK;
}

//Esta funcion inicializa MQTT
static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_SERVER,
        .event_handle = mqtt_event_handler,
        .client_id = id,
        .keepalive = 5
        /*.username="esp32co2_1",
        .password="esp"*/

    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

static void event_handler(void* arg, esp_event_base_t event_base,		//manejador de eventos
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        mqtt_app_start();
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

	esp_netif_dhcpc_stop(sta_netif);
    assert(sta_netif);


	esp_netif_ip_info_t ip_info;
    esp_netif_dns_info_t dns_info;

    IP4_ADDR(&ip_info.ip, 192, 168, 102, espip);
   	IP4_ADDR(&ip_info.gw, 192, 168, 102, 254);
   	IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    IP_ADDR4(&dns_info.ip, 212,128,20,252);

	esp_netif_set_ip_info(sta_netif, &ip_info);
    esp_netif_set_dns_info(sta_netif,ESP_NETIF_DNS_MAIN,&dns_info);




    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}



void app_main(void)
{

    nvs_flash_init();
	
    wifi_init_sta();

    
}
