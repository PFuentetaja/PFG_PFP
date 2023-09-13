#include "sensors.h"

double alpha = 0;
double beta = 0;
double sigma = 1;

double get_sensitivity(struct bmi160_dev bmi160dev, uint8_t gyro_accel){
	double sens;
	if(gyro_accel == BMI160_ACCEL_SEL){
		switch(bmi160dev.accel_cfg.range){
		case BMI160_ACCEL_RANGE_2G:
			sens=16384;
			break;
		case BMI160_ACCEL_RANGE_4G:
			sens=8192;
			break;
		case BMI160_ACCEL_RANGE_8G:
			sens=4096;
			break;
		case BMI160_ACCEL_RANGE_16G:
			sens=2048;
			break;
		default:
			sens = -1;
		}
	}
	else if(gyro_accel == BMI160_GYRO_SEL){
		switch(bmi160dev.gyro_cfg.range){
		case BMI160_GYRO_RANGE_2000_DPS:
			sens=16.4;
			break;
		case BMI160_GYRO_RANGE_1000_DPS:
			sens=32.8;
			break;
		case BMI160_GYRO_RANGE_500_DPS:
			sens=65.6;
			break;
		case BMI160_GYRO_RANGE_250_DPS:
			sens=131.2;
			break;
		case BMI160_GYRO_RANGE_125_DPS:
			sens=262.4;
			break;
		default:
			sens = -1;
		}
	}
	else{
		sens = -1;
	}
	return sens;
}


void get_angle_rad(struct bmi160_dev bmi160dev, struct bmi160_sensor_data bmi160_accel, struct dev_angle_rad *accel_angle){

	double Ax, Ay, Az;
	double sens = get_sensitivity(bmi160dev, BMI160_ACCEL_SEL);;

	Ax = bmi160_accel.x/sens;
	Ay = bmi160_accel.y/sens;
	Az = bmi160_accel.z/sens;
	accel_angle->x = atan(Ax/sqrt(pow(Ay, 2)+pow(Az, 2)));
	accel_angle->y = atan(Ay/sqrt(pow(Ax, 2)+pow(Az, 2)));
	accel_angle->z = atan(sqrt(pow(Ay, 2)+pow(Ax, 2))/Az);
}

void get_angle_gyro(struct bmi160_dev bmi160dev, struct bmi160_sensor_data bmi160_gyro, struct dev_angle_gyro *gyro_angle){
	static uint32_t sensortime_aux = 0;
	double Gx, Gy, Gz;
	double sens = get_sensitivity(bmi160dev, BMI160_GYRO_SEL);;
	Gx = bmi160_gyro.x/sens;
	Gy = bmi160_gyro.y/sens;
	Gz = bmi160_gyro.z/sens;
	sensortime_aux = bmi160_gyro.sensortime - sensortime_aux;

	gyro_angle->alabeo = gyro_angle->alabeo + Gx*0.02;
	gyro_angle->cabeceo = gyro_angle->cabeceo + Gy*0.02;
	gyro_angle->guinada = gyro_angle->guinada + Gz*0.02;
	printf("Alabeo: %f, cabeceo: %f,  guñada: %f, sensor time: %u\n", gyro_angle->alabeo, gyro_angle->cabeceo,gyro_angle->guinada, bmi160_gyro.sensortime);
}

void init_bmm150_sensor_driver_interface(struct bmm150_dev *bmm150dev){
	bmm150dev->intf_ptr = &fd_i2c;
	bmm150dev->intf = BMM150_I2C_INTF;
	bmm150dev->read = bmm150_aux_read;
	bmm150dev->write = bmm150_aux_write;
	bmm150dev->delay_us = delay_us;
}

void init_bmi160(struct bmi160_dev *bmi160dev)
{
    int8_t rslt;

    rslt = bmi160_init(bmi160dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev->chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
        return;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev->accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
    bmi160dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev->gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
    bmi160dev->gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    bmi160dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	bmi160dev->aux_cfg.aux_sensor_enable = BMI160_ENABLE;
	bmi160dev->aux_cfg.manual_enable = BMI160_ENABLE;
	bmi160dev->aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; //8 bytes. Lo pone en el datasheet MAG_IF[1]<1:0>
	//bmi160dev.aux_cfg. aux_odr =
	bmi160dev->aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(bmi160dev);
}


int8_t set_data_ready_int(uint8_t feature_enable)
{
    int8_t rslt = BMI160_OK;
    struct bmi160_int_settg int_config;

    if (feature_enable > 0)
    {
        /* Select the Interrupt channel/pin */
        int_config.int_channel = BMI160_INT_CHANNEL_1; /* Interrupt channel/pin 1 */

        /* Select the interrupt channel/pin settings */
        int_config.int_pin_settg.output_en = BMI160_ENABLE; /* Enabling interrupt pins to act as output pin */
        int_config.int_pin_settg.output_mode = BMI160_DISABLE; /* Choosing push-pull mode for interrupt pin */
        int_config.int_pin_settg.output_type = BMI160_ENABLE; /* Choosing active low output */
        int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */
        int_config.int_pin_settg.input_en = BMI160_DISABLE; /* Disabling interrupt pin to act as input */
        int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; /* non-latched output */

        /* Select the Interrupt type */
        int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT; /* Choosing type of interrupt */

//        /* Select the Any-motion interrupt parameters */
//        int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_ENABLE; /* 1- Enable tap, 0- disable tap */
//        int_config.int_type_cfg.acc_tap_int.tap_thr = 2; /* Set tap threshold */
//        int_config.int_type_cfg.acc_tap_int.tap_dur = 2; /* Set tap duration */
//        int_config.int_type_cfg.acc_tap_int.tap_shock = 0; /* Set tap shock value */
//        int_config.int_type_cfg.acc_tap_int.tap_quiet = 0; /* Set tap quiet duration */
//        int_config.int_type_cfg.acc_tap_int.tap_data_src = 1; /* data source 0 : filter or 1 : pre-filter */


        /* Set the Any-motion interrupt */
        rslt = bmi160_set_int_config(&int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev  */
        printf("bmi160_set_int_config(tap enable) status:%d\n", rslt);
    }
    else
    {
        /* Select the Interrupt channel/pin */
        int_config.int_channel = BMI160_INT_CHANNEL_1;
        int_config.int_pin_settg.output_en = BMI160_DISABLE; /* Disabling interrupt pins to act as output pin */
        int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */

        /* Select the Interrupt type */
        int_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT; /* Choosing Tap interrupt */
        //int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_DISABLE; /* 1- Enable tap, 0- disable tap */

        /* Set the Data ready interrupt */
        rslt = bmi160_set_int_config(&int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev */
        printf("bmi160_set_int_config(tap disable) status:%d\n", rslt);
    }

    return rslt;
}
void set_foc_conf(struct bmi160_foc_conf *foc_conf){

	foc_conf->foc_gyr_en=BMI160_ENABLE;

	foc_conf->foc_acc_x=BMI160_FOC_ACCEL_0G;
	foc_conf->foc_acc_y=BMI160_FOC_ACCEL_0G;
	foc_conf->foc_acc_z=BMI160_FOC_ACCEL_POSITIVE_G;

	foc_conf->acc_off_en=BMI160_ENABLE;
	foc_conf->gyro_off_en=BMI160_ENABLE;
}

void comp_iron(struct bmm150_mag_data mg_comp, double *mg_comp_iron){
	mg_comp_iron[0] = (sigma<1)? (mg_comp.x - alpha)*sigma : (mg_comp.x - alpha)/sigma;
	mg_comp_iron[1] = mg_comp.y - beta;
}

void compass_angle(double x, double y, double *angle){

	*angle = atan2(y,x);
	if(*angle<0)
		*angle += 2*M_PI;
}


void bmm150_calibration(uint8_t mag_data[8], struct bmi160_dev *bmi160dev, struct bmm150_mag_data *mg_comp, struct bmm150_dev *bmm150dev, int timeout_us ){

	uint32_t t_fin = gpioTick() + timeout_us;
#ifdef BMM150_USE_FLOATING_POINT
	float x_min, x_max, y_min, y_max;
#else
	int x_min, x_max, y_min, y_max;
#endif
	//uint8_t vueltas = 0;
	//double angle = double compass_angle(struct bmm150_mag_data mag_data);

	bmi160_read_aux_data_auto_mode(mag_data, bmi160dev);
	bmm150_aux_mag_data(mag_data, mg_comp, bmm150dev);

	x_min = mg_comp->x;
	x_max = mg_comp->x;
	y_min = mg_comp->y;
	y_max = mg_comp->y;

	//fprintf(stdout, "Comienza el bucle de calibracion\n");
	//for(int i=0; i<9999; i++){
	while (gpioTick()<t_fin){
		bmi160_read_aux_data_auto_mode(mag_data, bmi160dev);
		bmm150_aux_mag_data(mag_data, mg_comp, bmm150dev);
		if(mg_comp->x<x_min)
			x_min = mg_comp->x;
		else if(mg_comp->x>x_max)
			x_max = mg_comp->x;
		//if(mg_comp.y <50){
		if(mg_comp->y<y_min)
			y_min = mg_comp->y;
		else if(mg_comp->y>y_max)
			y_max = mg_comp->y;
		//}

		//printf("x_max = %f, x_min = %f, y_max = %f, y_min = %f\n", (float)x_max, (float)x_min, (float)y_max, (float)y_min);
	}
	fprintf(stdout, "Fin del bucle de calibracion\n");

	alpha = ((double)(x_min + x_max))/2;
	beta = ((double)(y_min + y_max))/2;
	//Estos valores se le restan a los valores x e y.

	sigma = ((double)(x_max - x_min))/((double)(y_max - y_min));
	//Este valor se multiplica al valor de x si es menor que 1, si no se multiplica
	//fprintf(stdout, "x_max = %f, x_min = %f, y_max = %f, y_min = %f, alpha = %f, beta = %f, sigma = %f\n", (double)x_max, (double)x_min, (double)y_max, (double)y_min, alpha, beta, sigma);
}

void init_bmi160_sensor_driver_interface(struct bmi160_dev *bmi160dev)
{
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev->write = i2c_write;
    bmi160dev->read = i2c_read;
    bmi160dev->delay_ms = delay_ms;

    /* set correct i2c address */
    bmi160dev->id = BMI160_I2C_ADDR;
    bmi160dev->intf = BMI160_I2C_INTF;
}


void delay_ms(uint32_t period){
	gpioDelay(period*1000);
}

void delay_us(uint32_t period, void *intf_ptr){
	gpioDelay(period);
}

int8_t bmm150_aux_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    //(void) id; /* id is unused here */

    return bmi160_aux_read(reg_addr, reg_data, len, &bmi160dev);
}


int8_t bmm150_aux_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    //(void) id; /* id is unused here */

    return bmi160_aux_write(reg_addr, reg_data, len, &bmi160dev);
}
