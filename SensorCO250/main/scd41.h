#include <stdio.h>
#include <stdlib.h>

#ifndef scd30
#define scd30
#include <esp_sleep.h>

//Getters//Setters
void go_sleep(uint16_t tiemposleep);

void init_i2c();

//void config_sensor(uint8_t direction,uint8_t sleep_time);

void start_sensor(uint8_t direction);

void stop_sensor(uint8_t direction);

bool do_sensor(uint8_t direction, int * co2,float * temp,float * hum);

bool get_ready_status(uint8_t direction);

void calibrate(uint16_t offset,uint8_t direction);

void continuous_measure(uint8_t direction);

void measure_oneshot(uint8_t direction);



#endif
