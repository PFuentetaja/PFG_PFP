#ifndef SENS_AUX_H
#define SENS_AUX_H

#include "bmi160.h"
#include "bmm150.h"
#include "i2c.h"
#include <math.h>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define INT1_pin  23

struct dev_angle_rad{
	double x;
	double y;
	double z;
};

struct dev_angle_gyro{
	double cabeceo;
	double alabeo;
	double guinada;
};

extern double alpha;
extern double beta;
extern double sigma;

extern struct bmi160_dev bmi160dev;

double get_sensitivity(struct bmi160_dev bmi160dev, uint8_t gyro_accel);

void get_angle_rad(struct bmi160_dev bmi160dev, struct bmi160_sensor_data bmi160_accel, struct dev_angle_rad *accel_angle);
void get_angle_gyro(struct bmi160_dev bmi160dev, struct bmi160_sensor_data bmi160_gyro, struct dev_angle_gyro *gyro_angle);

void init_bmi160(struct bmi160_dev *bmi160dev);

void set_foc_conf(struct bmi160_foc_conf *foc_conf);

void compass_angle(double x/*ponerle un menos delante*/, double y, double *angle);

void comp_iron(struct bmm150_mag_data mg_comp, double *mg_comp_iron);

void bmm150_calibration(uint8_t mag_data[8], struct bmi160_dev *bmi160dev, struct bmm150_mag_data *mg_comp, struct bmm150_dev *bmm150dev, int timeout_us );

void init_bmi160_sensor_driver_interface(struct bmi160_dev *bmi160dev);

void init_bmm150_sensor_driver_interface(struct bmm150_dev *bmm150dev);

int8_t bmm150_aux_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

int8_t bmm150_aux_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

void delay_ms(uint32_t period);

void delay_us(uint32_t period, void *intf_ptr);

int8_t set_data_ready_int(uint8_t feature_enable);

#endif
