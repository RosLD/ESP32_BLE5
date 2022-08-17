#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include <driver/i2c.h>
#include <esp_sleep.h>
#include "esp_sntp.h"
#include <math.h>

#define rdy GPIO_NUM_25
#define GPIO_INPUT_PIN_SEL ((1ULL<<rdy))

#define SDA_IO_NUM 19
#define SCL_IO_NUM 21
#define WRITE_BIT I2C_MASTER_WRITE            
#define READ_BIT I2C_MASTER_READ
//ACKs
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

uint16_t co2_p = 0;
float temp_p,hum_p = 0;

static i2c_port_t i2c_port = I2C_NUM_0;


//I2C Basic functions
void send_i2c_msg(uint8_t *data_wr, uint8_t direccion,int len){ //Send command //requires command array and direction
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (direccion << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	
}

uint8_t* read_i2c_msg(uint8_t direccion, size_t size){ //Read from i2c, requires direction returns reading

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (direccion << 1) | READ_BIT, ACK_CHECK_EN);
    uint8_t *data_rd = malloc(size);
    
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);	//MUST send ACK to sensor
    
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return data_rd;
}


//Deepsleep controller
void go_sleep(uint8_t tiemposleep){
		
    esp_err_t comp = esp_sleep_enable_timer_wakeup(tiemposleep*1000000);

	if(comp == ESP_OK){
		
		//esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_ON);
    	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    	esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF); 
		esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);		
		esp_deep_sleep_start();
	}else{
		printf("Insomnio");
	}

    
}

uint8_t calcCrc2b(uint16_t seed)
{
  uint8_t bit;                  // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  
  // calculates 8-Bit checksum with given polynomial
 
    crc ^= (seed >> 8) & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ 0x131;
      else           crc = (crc << 1);
    }
 
    crc ^= seed & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ 0x131;
      else           crc = (crc << 1);
    }
    
  return crc;
}

//I2C function starter
void init_i2c(){
	//Configuracion basica i2c
	int i2c_master_port = i2c_port;
	i2c_config_t conf = {
    	.mode = I2C_MODE_MASTER,
    	.sda_io_num = SDA_IO_NUM,         // select GPIO specific to your project
    	.sda_pullup_en = GPIO_PULLUP_ENABLE,
    	.scl_io_num = SCL_IO_NUM,         // select GPIO specific to your project
    	.scl_pullup_en = GPIO_PULLUP_ENABLE,
    	.master.clk_speed = 9600,  // select frequency specific to your project
    	
	};

	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0);
}



bool do_sensor(uint8_t direction, int * co22, float * temp2, float * hum2){

	unsigned char msg2[] = {0xec,0x05}; //Read command
	send_i2c_msg(msg2,direction,2);

    vTaskDelay(3);

    uint8_t *mensg = read_i2c_msg(direction,9);

    *co22 = (mensg[0]<<8|mensg[1]);

    *temp2 = -45+175*((mensg[3]<<8|mensg[4])/((pow(2,16))-1));

    *hum2 = 100*((mensg[6]<<8|mensg[7])/(pow(2,16)-1));

    if (co22 > 0)
        return true;
    
    return false;
}

void start_sensor(uint8_t direction){
	
	//Command to start sensor
    //SCD41 Start periodice measurement
	unsigned char msg1[] = {0x21,0xb1};
	send_i2c_msg(msg1,direction,2);
    vTaskDelay(2);
	
}

void stop_sensor(uint8_t direction){//use it to calibrate and to set it to low power

    unsigned char msg1[] = {0x3f,0x86};
	send_i2c_msg(msg1,direction,2);
    vTaskDelay(2);

}

bool get_ready_status(uint8_t direction){

    unsigned char msg1[] = {0xe4,0xb8};
    send_i2c_msg(msg1,direction,2);

    vTaskDelay(3);
    uint8_t *stdata = read_i2c_msg(direction,3);

    uint16_t abc = 0x8000;

    abc = (stdata[0]<<8|stdata[1]);

    if(abc != 0x8000)
        return true;

    return false;
    

}

void measure_oneshot(uint8_t direction){

    
    unsigned char msgsi[] = {0x21, 0x9d};

    send_i2c_msg(msgsi,direction,2);

}


void continuous_measure(uint8_t direction){

    while(true){

        while(!get_ready_status(direction))
            vTaskDelay(20);
        
        int co23;
        float temp3,hum3;
        do_sensor(direction,&co23,&temp3,&hum3);

        printf("CO2: %d ppm\n Temp: %f ºC\n Hum: %f %%\n",co23,temp3,hum3);

        vTaskDelay(5000/portTICK_PERIOD_MS);


    }

}

void calibrate(uint16_t offset,uint8_t direction){

    printf("Calibration has been issued are you sure about this?\n");
    
    printf("Unplug/reprogram ESP32 if no, else wait 3 minutes for this process to start\n");

    printf("Starting Sensor\n");
    start_sensor(direction);
    uint16_t cont = 36;

    while(cont>0){

        while(!get_ready_status(direction))
            vTaskDelay(20);
        
        int co23;
        float temp3,hum3;
        do_sensor(direction,&co23,&temp3,&hum3);

        printf("CO2: %d ppm\n Temp: %f ºC\n Hum: %f %%\n",co23,temp3,hum3);

        vTaskDelay(5000/portTICK_PERIOD_MS);

        printf("%d seconds left!\n",cont*5);

        cont--;
    }

    printf("Time up! Stopping sensor to start calibration\n");

    stop_sensor(direction);

    vTaskDelay(500/portTICK_PERIOD_MS);

    unsigned char msg1[] = {0x36,0x2f,(uint8_t)offset>>8,(uint8_t)offset,calcCrc2b(offset)};
	send_i2c_msg(msg1,direction,5);
    
    vTaskDelay(400/portTICK_PERIOD_MS);

    uint8_t *ackdata = read_i2c_msg(direction,3);

    if(ackdata[0]==0xff && ackdata[1]==0xff){
        printf("Calibration error!\n");
    }

    printf("Calibration succesfull!\n");


}


///////////////////////////////