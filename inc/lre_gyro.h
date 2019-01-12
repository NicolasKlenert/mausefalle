/*
 * mm_gyro.h
 *
 *  Created on: 12.01.2017
 *      Author: marc
 */

#ifndef LRE_GYRO_H_
#define LRE_GYRO_H_

#include "stm32f0xx.h"
#include "stm32f072b_discovery_l3gd20.h"

#define _lre_gyro_calibration_samples 200

#define _lre_gyro_scale_error_x 1.0
#define _lre_gyro_scale_error_y 1.0
#define _lre_gyro_scale_error_z 1.0  // scale error measured by measuring the raw data response of the sensor on a 360 degree turn (it only showed ~344 degree)

#define lre_gyro_status_standby 0
#define lre_gyro_status_reading 1
#define lre_gyro_status_newData 4

// rate value struct
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lre_gyro_vector_typeDef;

// heading value struct
typedef struct {
    float x;
    float y;
    float z;
} lre_gyro_vector_float_typeDef;

// gyro struct definition

// todo: add units, clean up for gods sake!
// struct for saving attitude data globally
typedef struct {
    uint8_t status; 				// standby / reading / new data
    lre_gyro_vector_float_typeDef rate; 	// rate vector
    lre_gyro_vector_typeDef rateI16;
    lre_gyro_vector_typeDef offset;
    uint8_t temperature;				// temperatur
    uint8_t temporary;				// temporary buffer
    uint8_t buffer[6];
    uint8_t bCounter;
} lre_gyro_typeDef;

// global gyro/attitude data struct
lre_gyro_typeDef lre_gyro;

void lre_gyro_init(void);
void lre_gyro_enableInt2(void);
float lre_gyro_getZRate(void);
void lre_gyro_requestRate(void);
void lre_gyro_resetStatus(void);
void lre_gyro_calibrate(void);
void lre_gyro_extiCmd(FunctionalState NewState);
void lre_gyro_getTemperature(void);

#endif /* LRE_GYRO_H_ */
