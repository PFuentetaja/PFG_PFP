#include <pigpio.h>
#include <signal.h>
#include "lcd.h"
#include "sensors.h"
#include "coords.h"

#define GRAD     1
#define RAD      2
#define G        3
#define LSB	     4
#define mG	     5

#define OFFSETS			1
#define NUEVA_MEDIDA	2

#define MAX_LINEAS_REG 100000

uint8_t u_acc_ang = GRAD;
uint8_t u_acc = G;
uint8_t u_acc_off = mG;
uint8_t u_gyr = GRAD;
uint8_t u_gyr_off = GRAD;

struct bmi160_dev bmi160dev;
struct bmm150_dev bmm150dev;
struct bmm150_settings bmm_settings;

struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;
struct bmi160_foc_conf foc_conf;
struct bmi160_offsets offsets;

struct dev_angle_rad accel_angle;
struct dev_angle_gyro gyro_angle;

struct bmm150_mag_data mg_comp;
double mg_comp_iron[2]={0};
uint8_t mag_data[8]={0};
double angulo = 0;

struct interrupciones lcd_int = {0};

int fd_i2c = 0;

uint8_t flanco_ts = 0;
uint8_t falling_edge =0;
int clk_freq = CLKIN_INI_FREQ;

uint8_t data_rd = 0;
uint8_t data_wr = 0;

FILE *fptr;
FILE *accptr;
FILE *gyrptr;
FILE *magptr;


uint8_t ena_mag= 0;
uint8_t ena_gyr= 0;
uint8_t ena_acc= 0;
uint8_t ena_acc_ang = 0;

typedef enum{PPAL, ACC, ACC_CONF, ACC_OFF, GYR, GYR_CONF, GYR_OFF, MAG} t_estado;

uint8_t menu;

//Señales
//uint8_t s_nuevo_odr=0;
//uint8_t s_nuevo_rango=0;
uint8_t s_volver=0;
uint8_t s_config=0;
uint8_t s_config_start = 0;
uint8_t s_calib = 0;
uint8_t s_calib_start = 0;
uint8_t s_acc = 0;
uint8_t s_gyr = 0;
uint8_t s_mag = 0;
uint8_t s_ppal = 0;
uint8_t s_mas = 0;
uint8_t s_menos = 0;

void my_function_kll(int sig){ // can be called asynchronously
	fprintf(stderr,"\nCerrando la aplicación\n");
	fclose(accptr);
	fclose(gyrptr);
	fclose(magptr);
	gpioHardwarePWM(CLKIN_pin, 0, 500000);
	kill(getpid(), /*SIGINT*/ SIGKILL); //9 o -9 si es al grupo entero
}

void copy_to_buffer(FILE *ptr);

void *sens_func(void);
void *edges_func(void);
void *lcd_ctrl_func(void);
void *monitor_func(void);
void *lcd_img_func(void);

void actualizar_medidas_acc(void);
void actualizar_medidas_gyro(void);
void actualizar_odr(struct coord_pixel coord, int odr, int sens);
void actualizar_rango(struct coord_pixel coord, int rango, int sens);
void actualizar_conf(int sens);
void vaciar_medidas(int medida);
void actualizar_off_lcd(int sens);
void actualizar_brujula(struct coord_pixel coord_mag_ang, double angulo /*radianes*/);
void actualizar_medidas_mag(void);

void print_menu_conf(void);
void print_menu_ppal(void);
void print_menu_gyr(void);
void print_menu_acc(void);
void print_menu_mag(void);

void accion_menu_off(int16_t pos);
void accion_menu_acc(int16_t pos);
void accion_menu_gyr(int16_t pos);
void accion_menu_mag(int16_t pos);
void accion_menu_ppal(int16_t pos);
void accion_menu_conf(int16_t pos);


int iniciar_ficheros(void);
void actualizar_fichero(uint8_t sensor, uint8_t tipo);


int main(int argc, char *argv[]) {

	signal(SIGINT, my_function_kll);
//	signal(SIGTSTP, my_function_kll);
//	//signal(SIGSEGV, my_function_kll);

	gpioCfgSetInternals(1<<10);

	int rslt = 0;

	pthread_t *lcd_img;
//	pthread_t *edges;
	pthread_t *sens;
	pthread_t *lcd_ctrl;
	pthread_t *monitor;

//	gpioCfgClock(0, 2, 1);
//	gpioCfgClock(2, 1, 0);

	if (gpioInitialise() == PI_INIT_FAILED) {
		fprintf(stderr, "failed to initialize GPIO\n");
		exit(EXIT_FAILURE);
	}
	else{
		fprintf(stderr, "Iniciado correctamente\n");
	}

	//Inicialización BMI160
	init_bmi160_sensor_driver_interface(&bmi160dev);
	fd_i2c = open_i2c_bus(BMI160_I2C_ADDR);
	init_bmi160(&bmi160dev);
	delay_ms(200); //Espera de 200ms tras iniciar ,según datasheet

	//Inicialización magnetometro
	init_bmm150_sensor_driver_interface(&bmm150dev);
	bmi160_aux_init(&bmi160dev);
	bmm150_init(&bmm150dev);

	//Configuracion como sensor auxiliar
	bmm_settings.preset_mode = BMM150_PRESETMODE_REGULAR;
	bmm150_set_presetmode(&bmm_settings, &bmm150dev);

	bmm_settings.pwr_mode = BMM150_POWERMODE_FORCED;
	bmm150_set_op_mode(&bmm_settings, &bmm150dev);

    uint8_t bmm150_data_start = BMM150_REG_DATA_X_LSB;
    bmi160dev.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;

    bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi160dev);

    delay_ms(200);//Espera de 200ms tras iniciar ,según datasheet

    //Habilitación offsets
	set_foc_conf(&foc_conf);

	//Inicialización LCD
	lcd_init();
	fprintf(stdout, "Configurando LCD...");
	//24 bits
	rslt = 0;
	do{
		lcd_spi_read(R03,&data_rd);
		data_wr=(data_rd&0xF0)|0x0D;
		lcd_spi_write(R03,&data_wr);
		lcd_spi_read(R03,&data_rd);
		rslt++;
	}
	while((data_rd!=0xCD)&&(rslt<3));
	if(rslt==3)
		fprintf(stdout, "Error al configurar el display en DE_24. 3 intentos\n");
	else
		fprintf(stdout,"modo DE_24\n");

	fprintf(stderr, "Secuencia de autotest\n");
	lcd_spi_read(R00,&data_rd);
	data_wr=(data_rd&0x0F)|0x20;//Black
	lcd_spi_write(R00,&data_wr);
	gpioDelay(1000000);
	data_wr=(data_rd&0x0F)|0xB0;//Color bar
	lcd_spi_write(R00,&data_wr);
	gpioDelay(1000000);
	data_wr=data_rd&0x0F;
	lcd_spi_write(R00,&data_wr);
	fprintf(stdout, "Fin Secuencia de autotest\n");

	fptr = fopen("./media/ppal.bmp", "r");

	if(fptr==NULL)
		fprintf(stderr, "la imagen no se abre correctamente\n");

	copy_to_buffer(fptr);


	//Inicializacion tactil
	init_ts();

	//bmi160_start_foc(&foc_conf, &offsets, &bmi160dev);



	sens = gpioStartThread(sens_func, "thread 2");
	lcd_ctrl = gpioStartThread(lcd_ctrl_func, "thread 3");
	monitor = gpioStartThread(monitor_func, "thread 4");
	lcd_img = gpioStartThread(lcd_img_func, "thread 1");
//	edges = gpioStartThread(edges_func, "thread 5");

	//Configuación señal de interrupcion
	if(set_data_ready_int(BMI160_ENABLE)!=BMI160_OK)
		fprintf(stderr, "Interrupción mal configurada\n");

	//Creacion ficheros para guardar datos
	rslt = iniciar_ficheros();
	if(rslt!=0)
		fprintf(stderr, "Error al abrir el fichero de almacenamiento de medidas %i \n", rslt);



	t_estado estado = PPAL;
	menu = DSPY_PPAL;
	lcd_int.actualizar_freq =1;

	uint8_t rango = bmi160dev.accel_cfg.range;
	uint8_t odr = bmi160dev.accel_cfg.odr;

	while(1){

		switch(estado){
		case PPAL :
			if(s_acc){
				s_acc = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc_ang = 1;
				ena_acc = 1;
				menu = DSPY_ACC;
				lcd_int.cambiar_pantalla = DSPY_ACC;
				estado = ACC;
			}
			else if(s_mag){
				s_mag = 0;
				ena_gyr = 0;
				ena_mag = 1;
				ena_acc_ang = 0;
				ena_acc = 0;
				menu = DSPY_MAG;
				lcd_int.cambiar_pantalla= DSPY_MAG;
				estado = MAG;
			}
			/*else if(s_mas){
				s_mas = 0;
				double_prueba = double_prueba + 100;
				clk_freq = clk_freq + 50000;
				gpioHardwarePWM(CLKIN_pin, clk_freq, 500000);
				lcd_int.actualizar_freq = 1;
			}
			else if(s_menos){
				s_menos = 0;
				double_prueba = double_prueba - 100;
				clk_freq = clk_freq - 50000;
				gpioHardwarePWM(CLKIN_pin, clk_freq, 500000);
				lcd_int.actualizar_freq = 1;
			}*/
			break;
		case MAG:
			if(s_calib_start){
				ena_mag = 0;
				fprintf(stdout, "Comienza el bucle de calibracion\n");
				lcd_int.exito = CALIB_MAG;
				bmm150_calibration(mag_data, &bmi160dev, &mg_comp, &bmm150dev, 20000000);
				escribir_rectangulo(aviso_mag, 17, 230, 0x00); //para borrar la notificacion
				fprintf(stdout,"Fin del bucle de calibracion\n");
				actualizar_fichero(BMM150_MAG_SEL, OFFSETS);
				printf("alpha = %f, beta = %f, sigma = %f\n", alpha, beta, sigma);
				ena_mag = 1;
				s_calib_start = 0;//Lo último, para que si se pulsa "calibrar" mientras se está calibrando, no se vuelva activar la señal, ya que quedaría sin poner a 0 de nuevo
			}
			else if(s_gyr){
				s_gyr = 0;
				ena_gyr = 1;
				ena_mag = 0;
				ena_acc_ang = 0;
				ena_acc = 0;
				menu = DSPY_GYR;
				lcd_int.cambiar_pantalla = DSPY_GYR;
				estado = GYR;
			}
			else if(s_ppal){
				s_ppal = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc_ang = 0;
				ena_acc = 0;
				menu = DSPY_PPAL;
				lcd_int.cambiar_pantalla = DSPY_PPAL;
				lcd_int.actualizar_freq = 1;
				estado = PPAL;
			}
			break;
		case ACC :
			if(s_config){
				s_config = 0;
				ena_acc = 0;
				ena_acc_ang = 0;
				menu = DSPY_ACC_CONF;
				lcd_int.cambiar_pantalla = DSPY_ACC_CONF;
				estado = ACC_CONF;
			}
			else if(s_calib){
				s_calib = 0;
				ena_acc = 1;
				ena_acc_ang = 0;
				menu = DSPY_ACC_OFF;
				lcd_int.cambiar_pantalla = DSPY_ACC_OFF;
				estado = ACC_OFF;
			}
			else if(s_ppal){
				s_ppal = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc = 0;
				ena_acc_ang = 0;
				menu = DSPY_PPAL;
				lcd_int.cambiar_pantalla = DSPY_PPAL;
				lcd_int.actualizar_freq = 1;
				estado = PPAL;
			}
			else if(s_gyr){
				s_gyr = 0;
				ena_gyr = 1;
				ena_mag = 0;
				ena_acc = 0;
				ena_acc_ang = 0;
				menu = DSPY_GYR;
				lcd_int.cambiar_pantalla = DSPY_GYR;
				estado = GYR;
			}
			break;

		case ACC_CONF:
			if(s_config_start){
				odr = s_config_start >> 4;
				rango = s_config_start & 0x0F;
				s_config_start = 0;
				switch(rango){
				case 4:
					bmi160dev.accel_cfg.range=BMI160_ACCEL_RANGE_16G;
					break;
				case 3:
					bmi160dev.accel_cfg.range=BMI160_ACCEL_RANGE_8G;
					break;
				case 2:
					bmi160dev.accel_cfg.range=BMI160_ACCEL_RANGE_4G;
					break;
				default:
					bmi160dev.accel_cfg.range=BMI160_ACCEL_RANGE_2G;
					break;
				}
				bmi160dev.accel_cfg.odr=odr;
				printf("odr = %d Hz,  en bmi = %d Hz....rango = %d g, en bmi = %d g\n", odr, bmi160dev.accel_cfg.odr, rango, bmi160dev.accel_cfg.range);
				if(bmi160_set_sens_conf(&bmi160dev)==BMI160_OK)
					lcd_int.exito = EXITO_CONF;
				else
					lcd_int.exito = ERROR_CONF;
			}
//			else if(nuevo_odr){ // valores de 0(nada seleccionado) a 8
//				lcd_int.seleccionar_odr=nuevo_odr;
//				odr = nuevo_odr;
//				nuevo_odr = 0;
//			}
//			else if(nuevo_rango){
//				lcd_int.seleccionar_rango=nuevo_rango;
//				rango = nuevo_rango;
//				nuevo_rango = 0;
//			}
			else if(s_volver){
				s_volver = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc = 1;
				ena_acc_ang = 1;
				menu = DSPY_ACC;
				lcd_int.cambiar_pantalla = DSPY_ACC;
				estado = ACC;
			}
			break;
		case ACC_OFF : //ACC_OFF
			if(s_calib_start){
				s_calib_start = 0;
				if(bmi160_start_foc(&foc_conf, &offsets, &bmi160dev)==BMI160_OK){
					actualizar_fichero(BMI160_ACCEL_SEL, OFFSETS);
					lcd_int.exito = EXITO_OFF;
					lcd_int.actualizar_off = 1;
				}
				else
					lcd_int.exito = ERROR_OFF;

			}
			else if(s_volver){
				s_volver = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc = 1;
				ena_acc_ang = 1;
				menu = DSPY_ACC;
				lcd_int.cambiar_pantalla = DSPY_ACC;
				estado = ACC;
			}
			break;
		case GYR :
			if(s_config){
				s_config = 0;
				ena_gyr = 0;
				menu = DSPY_GYR_CONF;
				lcd_int.cambiar_pantalla = DSPY_GYR_CONF;
				estado = GYR_CONF;
			}
			else if(s_calib){
				s_calib = 0;
				menu = DSPY_GYR_OFF;
				lcd_int.cambiar_pantalla = DSPY_GYR_OFF;
				estado = GYR_OFF;
			}
			else if(s_acc){
				s_acc = 0;
				ena_gyr = 0;
				ena_mag = 0;
				ena_acc_ang = 1;
				ena_acc = 1;
				menu = DSPY_ACC;
				lcd_int.cambiar_pantalla = DSPY_ACC;
				estado = ACC;
			}
			else if(s_mag){
				s_mag = 0;
				ena_gyr = 0;
				ena_mag = 1;
				ena_acc_ang = 0;
				ena_acc = 0;
				menu = DSPY_MAG;
				lcd_int.cambiar_pantalla= DSPY_MAG;
				estado = MAG;
			}
			break;
		case GYR_CONF:
			if(s_config_start){
				odr = s_config_start >> 4;
				rango = s_config_start & 0x0F;
				s_config_start = 0;
				bmi160dev.gyro_cfg.range=5-rango;
				bmi160dev.gyro_cfg.odr=odr+1;
					//printf("odr = %d,  en bmi = %d....rango = %d, en bmi = %d\n", odr, bmi160dev.accel_cfg.odr, rango, bmi160dev.accel_cfg.range);
				if(bmi160_set_sens_conf(&bmi160dev)==BMI160_OK)
					lcd_int.exito = EXITO_CONF;
				else
					lcd_int.exito = ERROR_CONF;
			}
//			if(nuevo_odr){ // valores de 0(nada seleccionado) a 8
//				lcd_int.seleccionar_odr=nuevo_odr;
//				odr = nuevo_odr;
//				nuevo_odr = 0;
//			}
//			else if(nuevo_rango){
//				lcd_int.seleccionar_rango=nuevo_rango;
//				rango = nuevo_rango;
//				nuevo_rango = 0;
//			}
			else if(s_volver){
			s_volver = 0;
			ena_gyr = 1;
			ena_mag = 0;
			ena_acc = 0;
			ena_acc_ang = 0;
			menu = DSPY_GYR;
			lcd_int.cambiar_pantalla = DSPY_GYR;
			estado = GYR;
		}
			break;
		case GYR_OFF : //ACC_OFF
			if(s_calib_start){
				s_calib_start = 0;
				if(bmi160_start_foc(&foc_conf, &offsets, &bmi160dev)==BMI160_OK){
					actualizar_fichero(BMI160_GYRO_SEL, OFFSETS);
					lcd_int.exito = EXITO_OFF;
					lcd_int.actualizar_off = 1;
				}
				else
					lcd_int.exito = ERROR_OFF;
			}
			else if(s_volver){
				s_volver = 0;
				ena_gyr = 1;
				ena_mag = 0;
				ena_acc = 0;
				ena_acc_ang = 0;
				menu = DSPY_GYR;
				lcd_int.cambiar_pantalla = DSPY_GYR;
				estado = GYR;
			}
			break;
		default:
			break;

		}

		gpioSleep(PI_TIME_RELATIVE, 0, 10000);
	}

	fclose(fptr);

	gpioTerminate();

	fprintf(stderr, "Fin del programa\n");

	return 0;
}

void *edges_func(void){
	uint8_t valor_ini = gpioRead(EDGE_detect_pin);
	uint8_t valor_sec = valor_ini;

    while (1){
	   valor_sec = gpioRead(EDGE_detect_pin);
	   falling_edge = valor_ini&(!valor_sec);
	   valor_ini = valor_sec;
   }
}

void *sens_func(void){

	printf("Sensibilidad acelerometro = %i\n", (int)get_sensitivity(bmi160dev, BMI160_ACCEL_SEL));
	printf("Sensibilidad giroscopio = %f\n", get_sensitivity(bmi160dev, BMI160_GYRO_SEL));
	uint8_t buffer_sens=0;
    while (1){
    	if(gpioRead(INT1_pin)&(ena_acc|ena_gyr|ena_mag)){

    		//Se lee STATUS para identificar el sensor con medidas nuevas
    		i2c_read(BMI160_I2C_ADDRESS, 0x1B, &buffer_sens, 1);

    		if(ena_mag&(buffer_sens>>5)){
    			bmi160_read_aux_data_auto_mode(mag_data, &bmi160dev);
    			bmm150_aux_mag_data(mag_data, &mg_comp, &bmm150dev);
    			comp_iron(mg_comp, mg_comp_iron);
    			actualizar_fichero(BMM150_MAG_SEL, NUEVA_MEDIDA);
    			lcd_int.nueva_medida=BMM150_MAG_SEL;
    		}
    		else if(ena_gyr&(buffer_sens>>6)){
    			bmi160_get_sensor_data(BMI160_GYRO_SEL|BMI160_TIME_SEL, &bmi160_accel, &bmi160_gyro, &bmi160dev);
    			actualizar_fichero(BMI160_GYRO_SEL, NUEVA_MEDIDA);
    			lcd_int.nueva_medida=BMI160_GYRO_SEL;
    		}
    		else if(ena_acc&(buffer_sens>>7)){
    			bmi160_get_sensor_data(BMI160_ACCEL_SEL|BMI160_TIME_SEL, &bmi160_accel, &bmi160_gyro, &bmi160dev);
    			get_angle_rad(bmi160dev, bmi160_accel, &accel_angle);
    			actualizar_fichero(BMI160_ACCEL_SEL, NUEVA_MEDIDA);
    			lcd_int.nueva_medida=BMI160_ACCEL_SEL;
    		}
    	}
    	gpioSleep(PI_TIME_RELATIVE, 0, 100); // el ODR máximo es de 3200 Hz ---> 312,5 us
    }
}

void *lcd_ctrl_func(void){

	uint8_t actual_odr = bmi160dev.accel_cfg.odr;
	uint8_t actual_rango = (bmi160dev.accel_cfg.range>>2)+1;

    while (1){

    	if(lcd_int.nueva_medida!=0){
    		switch (lcd_int.nueva_medida){
    		case BMI160_ACCEL_SEL:
    			actualizar_medidas_acc();
    			break;
    		case BMI160_GYRO_SEL:
    			actualizar_medidas_gyro();
    			break;
    		case BMM150_MAG_SEL:
    			actualizar_medidas_mag();
    			break;
    		default:
    			break;
    		}
    		lcd_int.nueva_medida=0;
    	}


    	if (lcd_int.cambio_unidades_acc){
    		lcd_int.cambio_unidades_acc=0;
//    		vaciar_medidas(G);
    		invertir_pixels(selec_acc, 22, 60);
    	}
    	if (lcd_int.cambio_unidades_gyr){
    	 	lcd_int.cambio_unidades_gyr=0;
//    	 	vaciar_medidas(RAD);
    	 	invertir_pixels(selec_gyr, 22, 60);
    	}
    	else if (lcd_int.cambio_unidades_ang){
    	    lcd_int.cambio_unidades_ang=0;
    	    invertir_pixels(selec_ang, 22, 60);
    	}
    	else if (lcd_int.cambio_unidades_off){
    	    lcd_int.cambio_unidades_off=0;
//    	    vaciar_medidas(mG);
    	    invertir_pixels(selec_off, 22, 60);
    	    actualizar_off_lcd((ena_acc == 1)?BMI160_ACCEL_SEL:BMI160_GYRO_SEL);
    	}
    	else if(lcd_int.seleccionar_odr){
    		cambiar_seleccion(lcd_int.seleccionar_odr, actual_odr);
    		fprintf(stdout, "ODR-> nuevo: %d, actual: %d\n", lcd_int.seleccionar_odr, actual_odr);
    		actual_odr = lcd_int.seleccionar_odr;
    		lcd_int.seleccionar_odr = 0;
    	}
    	else if(lcd_int.seleccionar_rango){
    	    cambiar_seleccion(lcd_int.seleccionar_rango, actual_rango);
    	    fprintf(stdout, "Rango-> nuevo: %d, actual: %d\n", lcd_int.seleccionar_rango, actual_rango);
    	    actual_rango = lcd_int.seleccionar_rango;
    	    lcd_int.seleccionar_rango = 0;
    	}
    	else if(lcd_int.exito){
    		notificar(lcd_int.exito);
    		lcd_int.exito = 0;
    	}/*
    	else if(lcd_int.actualizar_freq){
    		lcd_int.actualizar_freq = 0;
    		escribir_entero(clk_freq, 8, 0, coord_freq_lcd);
    	}*/
    	else if(lcd_int.cambiar_pantalla){
    		fclose(fptr);
    		switch(lcd_int.cambiar_pantalla){
    		case DSPY_ACC:
    			fptr = fopen("media/acc.bmp", "r");
    			copy_to_buffer(fptr);
    			actualizar_conf(BMI160_ACCEL_SEL);
    			if(u_acc==LSB)
    				invertir_pixels(selec_acc, 22, 60);
    			if(u_acc_ang == RAD)
    				invertir_pixels(selec_ang, 22, 60);
    			break;
    		case DSPY_ACC_CONF:
    			fptr = fopen("media/acc_conf.bmp", "r");
    			if(fptr==NULL)
    				fprintf(stdout, "acc_conf no se abre correctamente\n");
    			else
    				fprintf(stdout, "acc_conf se abre correctamente\n");
    			copy_to_buffer(fptr);
    			actual_odr = bmi160dev.accel_cfg.odr;
    			actual_rango = (bmi160dev.accel_cfg.range>>2)+1;
    			cambiar_seleccion(actual_odr, actual_odr);
    			cambiar_seleccion(actual_rango, actual_rango);
    			break;
    		case DSPY_ACC_OFF:
    	   		fptr = fopen("media/acc_off.bmp", "r");
    	   		copy_to_buffer(fptr);
    	   		actualizar_off_lcd(BMI160_ACCEL_SEL);
    	   		if(u_acc==LSB)
    	   		    invertir_pixels(selec_acc, 22, 60);
    	   		if(u_acc_off == LSB)
    	   		    invertir_pixels(selec_off, 22, 60);
    	   		break;
    		case DSPY_GYR:
    		  	fptr = fopen("media/gyr.bmp", "r");
    		  	copy_to_buffer(fptr);
    		  	actualizar_conf(BMI160_GYRO_SEL);
    		  	if(u_gyr==LSB)
    		  		invertir_pixels(selec_gyr, 22, 60);
    		    break;
    		case DSPY_GYR_CONF:
    		  	fptr = fopen("media/gyr_conf.bmp", "r");
    		  	if(fptr==NULL)
    		  		fprintf(stdout, "gyro_conf no se abre correctamente\n");
    		  	else
    		  		fprintf(stdout, "gyro_conf se abre correctamente\n");
    		  	copy_to_buffer(fptr);
    		  	actual_odr = bmi160dev.gyro_cfg.odr-1;
    		  	actual_rango = 5-bmi160dev.gyro_cfg.range;
    		  	cambiar_seleccion(actual_odr, actual_odr);
    		  	cambiar_seleccion(actual_rango, actual_rango);
    		  	break;
    		case DSPY_GYR_OFF:
    			fptr = fopen("media/gyr_off.bmp", "r");
    			copy_to_buffer(fptr);
    			actualizar_off_lcd(BMI160_GYRO_SEL);
    			if(u_gyr==LSB)
    			    invertir_pixels(selec_gyr, 22, 60);
    			if(u_gyr_off == LSB)
    			    invertir_pixels(selec_off, 22, 60);
    			break;
    		case DSPY_MAG:
    		   	fptr = fopen("media/mag.bmp", "r");
    		   	copy_to_buffer(fptr);
    		    break;
    		case DSPY_PPAL:
    		    fptr = fopen("media/ppal.bmp", "r");
    		    copy_to_buffer(fptr);
//    		   	actualizar_conf(BMI160_GYRO_SEL);
//    		    if(u_gyr==LSB)
//    		    	invertir_pixels(selec_acc, 22, 60);
    		    break;
    		default:
    			fprintf(stdout, "valor de cambiar_pantalla inválido\n");
    			break;
    		}
    		lcd_int.cambiar_pantalla = 0;
    	}
    	else if(lcd_int.actualizar_off){
    		lcd_int.actualizar_off = 0;
    		actualizar_off_lcd((ena_acc == 1)?BMI160_ACCEL_SEL:BMI160_GYRO_SEL);
    	}
    	gpioSleep(PI_TIME_RELATIVE, 0, 20000); //20 ms
    }
}

void *monitor_func(void){
	//int tecla = 5;
	uint8_t fin_toque = 1;
	uint8_t buffer_ts[4]={0};
	uint8_t mode = 1;
	int16_t pos_x = 0;
	int16_t pos_y = 0;
	int16_t accion = 0;
	print_menu_ppal();
	i2c_write(TS_I2C_ADDR, 0xA4, &mode, 1); //modo  periodico
	gpioSetISRFunc(nINT_pin, EITHER_EDGE, 18, cambio_nINT); //18 ms


	while (1){ // Mira primero si hay desplazamiento y en caso negativo, algún punto
		if((flanco_ts==BAJADA)&!((buffer_ts[0]==0x14)|(buffer_ts[0]==0x1C))){
			flanco_ts = 0;
			fin_toque = 0;
			i2c_read(TS_I2C_ADDR, 0x01, buffer_ts, 1);
		}
		else if((flanco_ts == TIMEOUT)&(!fin_toque)){
			flanco_ts = 0;
			fin_toque = 1;
			if(buffer_ts[0] == 0){
				i2c_read(TS_I2C_ADDR, 0x03, buffer_ts, 4);
				//printf("buffer = [0][1][2][3] = 0x%2X, 0x%2X, 0x%2X, 0x%2X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
				pos_x = (((uint16_t)(buffer_ts[0]&0x0F))<<8)|buffer_ts[1];
				pos_y = (((uint16_t)(buffer_ts[2]&0x0F))<<8)|buffer_ts[3];
				printf("coord x = %d  coord y = %d\n", pos_x, pos_y);
				//printf("coord x = %04x  coord y = %04x\n", pos_x, pos_y);
				accion = pos_x;
			}
			else{
				printf("desplazamiento: 0x%2X\n", buffer_ts[0]);
				if(buffer_ts[0]==0x14)
					accion = DER;
				else if(buffer_ts[0]==0x1C)
					accion = IZQ;
			}

			if(menu == DSPY_ACC)
				accion_menu_acc(accion);
			else if(menu == DSPY_GYR)
				accion_menu_gyr(accion);
			else if((menu == DSPY_ACC_CONF)||(menu == DSPY_GYR_CONF))
				accion_menu_conf(accion);
			else if((menu == DSPY_ACC_OFF)||(menu == DSPY_GYR_OFF))
				accion_menu_off(accion);
			else if(menu == DSPY_PPAL)
				accion_menu_ppal(accion); //pos_x
			else if(menu == DSPY_MAG)
				accion_menu_mag(accion); //pos_x

			accion = 0;
			for (int i = 0; i<4; i++)
				buffer_ts[i] = 0; //realmente no necesario. Sí para que no lea el punto anterior si se satura y no lee correctamente. Así no falla en los desplazamientos

		}
//		gpioDelay(100000);
		gpioSleep(PI_TIME_RELATIVE, 0, 1000); //1 ms
	}
}

void *lcd_img_func(void){
	while(1){
		send_img(DE_24_mode);
		gpioDelay(200);
//		nanosleep(2);
	}
}

void actualizar_medidas_acc(void){

	double medida;
	double sens;

	if(u_acc==G){
		sens = get_sensitivity(bmi160dev, BMI160_ACCEL_SEL);
		escribir_double(bmi160_accel.x/sens, 1, ejeX_acc);
		escribir_double(bmi160_accel.y/sens, 1, ejeY_acc);
		escribir_double(bmi160_accel.z/sens, 1, ejeZ_acc);
	}
	else{//LSB
		escribir_entero(bmi160_accel.x, 5, 1, ejeX_acc);
		escribir_entero(bmi160_accel.y, 5, 1, ejeY_acc);
		escribir_entero(bmi160_accel.z, 5, 1, ejeZ_acc);

	}
	if(ena_acc_ang){
		medida = (u_acc_ang == RAD)?accel_angle.x:(accel_angle.x)*(180.0/M_PI);
		escribir_double(medida, 1, ejeX_ang);
		medida = (u_acc_ang == RAD)?accel_angle.y:(accel_angle.y)*(180.0/M_PI);
		escribir_double(medida, 1, ejeY_ang);
		medida = (u_acc_ang == RAD)?accel_angle.z:(accel_angle.z)*(180.0/M_PI);
		escribir_double(medida, 1, ejeZ_ang);
	}
}

void actualizar_medidas_gyro(void){

	double sens;

	if(u_gyr==GRAD){
		sens = get_sensitivity(bmi160dev, BMI160_GYRO_SEL);
		escribir_double(bmi160_gyro.x/sens, 1, ejeX_gyr);
		escribir_double(bmi160_gyro.y/sens, 1, ejeY_gyr);
		escribir_double(bmi160_gyro.z/sens, 1, ejeZ_gyr);
	}
	else{ //LSB
		escribir_entero(bmi160_gyro.x, 5, 1, ejeX_gyr);
		escribir_entero(bmi160_gyro.y, 5, 1, ejeY_gyr);
		escribir_entero(bmi160_gyro.z, 5, 1, ejeZ_gyr);
	}
}

void actualizar_odr(struct coord_pixel coord, int odr, int sens){
	int pos_x = coord.x*3;
	int pos_y = coord.y;
	int odr_aux;
	//int off = 7-sens;

	if((odr == BMI160_ACCEL_ODR_12_5HZ)&&(sens == BMI160_ACCEL_SEL)){
		escribe_num(&pos_x, pos_y, 1);
		escribe_num(&pos_x, pos_y, 2);
		escribe_num(&pos_x, pos_y, COMA);
		escribe_num(&pos_x, pos_y, 5);
	}
	else{
		odr_aux = 25*pow(2,(odr-6));
		escribir_entero(odr_aux, 4, 0, coord);
	}
}

void actualizar_rango(struct coord_pixel coord, int rango, int sens){

	int pos_x = coord.x*3;
	int pos_y = coord.y;
	int rango_aux;

	if(sens == BMI160_GYRO_SEL){
		rango_aux = 125*pow(2,(4-rango));
		escribir_entero(rango_aux, 4, 0, coord);
	}
	else if (sens == BMI160_ACCEL_SEL){
		switch(rango){
		case BMI160_ACCEL_RANGE_2G:
			escribe_num(&pos_x, pos_y, 2);
			break;
		case BMI160_ACCEL_RANGE_4G:
			escribe_num(&pos_x, pos_y, 4);
			break;
		case BMI160_ACCEL_RANGE_8G:
			escribe_num(&pos_x, pos_y, 8);
			break;
		default:
			escribe_num(&pos_x, pos_y, 1);
			escribe_num(&pos_x, pos_y, 6);
			break;
		}
	}
}

void actualizar_conf(int sens){
	if(sens == BMI160_ACCEL_SEL){
		actualizar_odr(coord_odr, bmi160dev.accel_cfg.odr, BMI160_ACCEL_SEL);
		actualizar_rango(coord_rango, bmi160dev.accel_cfg.range, BMI160_ACCEL_SEL);
	}
	else if(sens == BMI160_GYRO_SEL){
		actualizar_odr(coord_odr, bmi160dev.gyro_cfg.odr, BMI160_GYRO_SEL);
		actualizar_rango(coord_rango, bmi160dev.gyro_cfg.range, BMI160_GYRO_SEL);
	}
}

void vaciar_medidas(int medida){
	int pos_x, pos_y;
	if(medida == G){
	//X
		pos_x = (ejeX_acc.x+45)*3;
		pos_y = ejeX_acc.y;
		escribe_num(&pos_x, pos_y, NADA);
	//Y
		pos_x = (ejeY_acc.x+45)*3;
		pos_y = ejeY_acc.y;
		escribe_num(&pos_x, pos_y, NADA);
	//Z
		pos_x = (ejeZ_acc.x+45)*3;
		pos_y = ejeZ_acc.y;
		escribe_num(&pos_x, pos_y, NADA);
	}
	else if (medida == RAD){
		pos_x = (ejeX_gyr.x+45)*3;
		pos_y = ejeX_gyr.y;
		escribe_num(&pos_x, pos_y, NADA);
		pos_x = (ejeY_gyr.x+45)*3;
		pos_y = ejeY_gyr.y;
		escribe_num(&pos_x, pos_y, NADA);
		pos_x = (ejeZ_gyr.x+45)*3;
		pos_y = ejeZ_gyr.y;
		escribe_num(&pos_x, pos_y, NADA);
	}
	else if (medida == mG){
		pos_x = (ejeX_off.x+45)*3;
		pos_y = ejeX_off.y;
		escribe_num(&pos_x, pos_y, NADA);
		pos_x = (ejeY_off.x+45)*3;
		pos_y = ejeY_off.y;
		escribe_num(&pos_x, pos_y, NADA);
		pos_x = (ejeZ_off.x+45)*3;
		pos_y = ejeZ_off.y;
		escribe_num(&pos_x, pos_y, NADA);
	}
}

void actualizar_off_lcd(int sens){
	double medida;
	int8_t off_x;
	int8_t off_y;
	int8_t off_z;

	if(sens == BMI160_ACCEL_SEL){
		off_x = offsets.off_acc_x;
		off_y = offsets.off_acc_y;
		off_z = offsets.off_acc_z;

		if(u_acc_off==mG){
			medida = ((double) off_x)*3.9;
			escribir_double(medida, 1, ejeX_off);
			medida = ((double) off_y)*3.9;
			escribir_double(medida, 1, ejeY_off);
			medida = ((double) off_z)*3.9;
			escribir_double(medida, 1, ejeZ_off);
		}
		else{
			escribir_entero(off_x, 5, 1, ejeX_off);
			escribir_entero(off_y, 5, 1, ejeY_off);
			escribir_entero(off_z, 5, 1, ejeZ_off);
		}
	}
	else if(sens == BMI160_GYRO_SEL){
			off_x = offsets.off_gyro_x;
			off_y = offsets.off_gyro_y;
			off_z = offsets.off_gyro_z;

			if(u_gyr_off==GRAD){
				medida = ((double) off_x)*0.061;
				escribir_double(medida, 1, ejeX_off);
				medida = ((double) off_y)*0.061;
				escribir_double(medida, 1, ejeY_off);
				medida = ((double) off_z)*0.061;
				escribir_double(medida, 1, ejeZ_off);
			}
			else{
				escribir_entero(off_x, 5, 1, ejeX_off);
				escribir_entero(off_y, 5, 1, ejeY_off);
				escribir_entero(off_z, 5, 1, ejeZ_off);
			}
		}
}

void actualizar_brujula(struct coord_pixel coord_mag_ang, double angulo /*radianes*/){
	double angulo_aux;
	static struct coord_pixel pos_aguja = {200, 58};
	uint8_t radio = 57;
	//fprintf(stdout, "angulo rad = %f  ", angulo);

	//primero borrar la posicion original
	for (int i = 0; i<10; i++){
		for (int j = 0; j<10; j++){
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j]=0xFF;
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j+1]=0xFF;
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j+2]=0xFF;
		}
	}

	// ahora escribir la nueva
	pos_aguja.x = centro.x + round(radio*sin(angulo)) - 5; //mitad pixeles de lo que ocupa el recuadro de la aguja
	pos_aguja.y = centro.y - round(radio*cos(angulo)) - 5;
	//fprintf(stdout, "pos_aguja.x = %d, pos_aguja.y = %d", pos_aguja.x, pos_aguja.y);
	for (int i = 0; i<10; i++){
		for (int j = 0; j<10; j++){
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j]=/*0x00*/aguja[10*i+j];
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j+1]=/*0x00*/aguja[10*i+j];
			buffer[pos_aguja.y + i][3*pos_aguja.x +3*j+2]=/*0x00*/aguja[10*i+j];
		}
	}
	angulo_aux = round(angulo*180.0/M_PI); //grados
	//fprintf(stdout, "angulo = %f  \n", angulo_aux);
	// escribir el angulo
	escribir_entero(angulo_aux, 3, 0, coord_mag_ang);
}

void actualizar_medidas_mag(void){

	compass_angle(-1*mg_comp_iron[0],-1*mg_comp_iron[1], &angulo);
	actualizar_brujula(coord_mag_ang, angulo /*radianes*/);

	//fprintf(stdout, "angulo = %d\n", abs(angulo_ant));
#ifdef	BMM150_USE_FLOATING_POINT
	escribir_double(mg_comp_iron[0], 1, coord_magX);
	escribir_double(mg_comp_iron[1], 1, coord_magY);
	escribir_double(mg_comp.z, 1, coord_magZ);
#else
	escribir_entero(round(mg_comp_iron[0]), 5, 1, coord_magX);
	escribir_entero(round(mg_comp_iron[1]), 5, 1, coord_magY);
	escribir_entero(mg_comp.z, 5, 1, coord_magZ);
#endif
}

void copy_to_buffer(FILE *ptr){ //viene en formato BGR BGR BGR asi que hay que reorganizarlo
	fseek(ptr, 0x36, SEEK_SET); // IMPORTANTE poner la cabecera en hexadecimal (valor observado en HexEd). Siguiente, coger datos interesantes de cabecera y usarlos por si el offset es distinto
	for(int i =0; i<FILAS; i++){
		for(int j=0; j<COLUMNAS*3; j=j+3){
			buffer[239-i][j+2]=getc(ptr);//B
			buffer[239-i][j+1]=getc(ptr);//G
			buffer[239-i][j]=getc(ptr);//R
		}
	}
}




/****************************************************************************************************************/
/*										Monitorización															*/
/****************************************************************************************************************/

void print_menu_off(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*              MENU CALIB                  *\n");
	fprintf(stdout, "*    1. Cambiar unidades medidas           *\n");
	fprintf(stdout, "*    2. Cambiar unidades offsets           *\n");
	fprintf(stdout, "*    3. Iniciar calibración                *\n");
	fprintf(stdout, "*    4. Volver                             *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_off(int16_t pos){
	if ((pos >= 275)&&(pos <= 313) /*intervalo del botón cambio unidades sens*/){
		fprintf(stdout, "Cambio de unidades : %d --> ",(menu == DSPY_ACC_OFF)?u_acc:u_gyr);
		if(menu == DSPY_ACC_OFF){
			u_acc = (u_acc == G)?LSB:G;
			lcd_int.cambio_unidades_acc=1;
		}
		else{
			u_gyr = (u_gyr == GRAD)?LSB:GRAD;
			lcd_int.cambio_unidades_gyr=1;
		}
		fprintf(stdout, "%d\n",(menu == DSPY_ACC_OFF)?u_acc:u_gyr);
		print_menu_off();
	}
	else if ((pos >= 211)&&(pos <= 249) /*intervalo del botón cambio unidades off*/){
		fprintf(stdout, "Cambio de unidades offset : %d --> ",(menu == DSPY_ACC_OFF)?u_acc_off:u_gyr_off);
		if(menu == DSPY_ACC_OFF)
			u_acc_off = (u_acc_off == LSB)?mG:LSB;
		else
			u_gyr_off = (u_gyr_off == LSB)?GRAD:LSB;
		lcd_int.cambio_unidades_off = 1;
		fprintf(stdout, "%d\n",(menu == DSPY_ACC_OFF)?u_acc_off:u_gyr_off);
		print_menu_off();
	}
	else if ((pos >= 80)&&(pos <= 180)){
		fprintf(stdout, "Calibrar\n");
		s_calib_start = 1;
		print_menu_off();
	}
	else if (pos < 55){
		(menu == DSPY_ACC_OFF)?print_menu_acc():print_menu_gyr();
		s_volver = 1;
		fprintf(stdout, "Volver\n");
	}
	else{
		fprintf(stdout, "Opción inválida");
	}
}

void print_menu_acc(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*                 MENU ACC                 *\n");
	fprintf(stdout, "*    IZQ. GYR                              *\n");
	fprintf(stdout, "*    DER. PPAL                             *\n");
	fprintf(stdout, "*    3. Cambiar unidades medidas           *\n");
	fprintf(stdout, "*    4. Cambiar unidades º/rad             *\n");
	fprintf(stdout, "*    5. Calibrar                           *\n");
	fprintf(stdout, "*    6. Modificar                          *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_acc(int16_t pos){
	if (pos == DER){
		fprintf(stdout, "Desplazamiento DER: ppal\n");
		s_ppal = 1;
		print_menu_ppal();
	}
	else if (pos == IZQ){
		fprintf(stdout, "Desplazamiento IZQ: gyr\n");
		s_gyr = 1;
		print_menu_gyr();
	}
	else if ((pos >= 273)&&(pos <= 311) /*intervalo del botón cambio unidades acc*/){
		fprintf(stdout, "Cambio de unidades : %d --> ",u_acc);
		u_acc = (u_acc == G)?LSB:G;
		fprintf(stdout, "%d\n",(menu == DSPY_ACC)?u_acc:u_gyr);
		lcd_int.cambio_unidades_acc=1;
		print_menu_acc();
	}
	else if ((pos >= 222)&&(pos <= 260) /*intervalo del botón cambio unidades ang*/){
		fprintf(stdout, "Cambio de unidades : %d --> ",u_acc_ang);
		u_acc_ang = (u_acc_ang == GRAD)?RAD:GRAD;
		fprintf(stdout, "%d\n",u_acc_ang);
		lcd_int.cambio_unidades_ang=1;
		print_menu_acc();
	}
	else if ((pos >= 120)&&(pos <= 200) /*intervalo del botón calibrar*/){
		fprintf(stdout, "Calibrar\n");
		s_calib = 1;
		print_menu_off();
	}
	else if ((pos >= 15)&&(pos <= 95) /*intervalo del botón modificar*/){
		fprintf(stdout, "Modificar\n");
		s_config = 1;
		print_menu_conf();
	}
	else{
		fprintf(stdout, "Opción inválida");
	}
}

void print_menu_gyr(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*                 MENU GYR                 *\n");
	fprintf(stdout, "*    IZQ. MAG                              *\n");
	fprintf(stdout, "*    DER. ACC                              *\n");
	fprintf(stdout, "*    3. Cambiar unidades medidas           *\n");
	fprintf(stdout, "*    4. Calibrar                           *\n");
	fprintf(stdout, "*    5. Modificar                          *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_gyr(int16_t pos){
	if (pos == DER){
		fprintf(stdout, "Desplazamiento DER: acc\n");
		s_acc = 1;
		print_menu_acc();
	}
	else if (pos == IZQ){
		fprintf(stdout, "Desplazamiento IZQ: mag\n");
		s_mag = 1;
		print_menu_mag();
	}
	else if ((pos >= 257)&&(pos <= (257+30)) /*intervalo del botón cambio unidades*/){
		fprintf(stdout, "Cambio de unidades : %d --> ",u_gyr);
		u_gyr = (u_gyr == GRAD)?LSB:GRAD;
		fprintf(stdout, "%d\n",u_gyr);
		lcd_int.cambio_unidades_gyr=1;
		print_menu_gyr();
	}
	else if ((pos >= 139)&&(pos <= (139+80)) /*intervalo del botón calibrar*/){
		fprintf(stdout, "Calibrar\n");
		s_calib = 1;
		print_menu_off();
	}
	else if ((pos >= 17)&&(pos <= (17+80)) /*intervalo del botón modificar*/){
		fprintf(stdout, "Modificar\n");
		s_config = 1;
		print_menu_conf();
	}
	else{
		fprintf(stdout, "Opción inválida : pos = %d", pos);
	}
}

void print_menu_conf(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*                MENU CONF                 *\n");
	fprintf(stdout, "*    1. Elegir ODR                         *\n");
	fprintf(stdout, "*    2. Elegir rango                       *\n");
	fprintf(stdout, "*    3. Actualizar configuración           *\n");
	fprintf(stdout, "*    4. Volver                             *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_conf(int16_t pos){
	static uint8_t rango, odr;
	static uint8_t ctrl = 1;
	//sólo si es la primera vez que se llama a esta funcion tras entrar en la pantalla de config, se guardan los valores de odr y rango en ese momento
	if (ctrl&&(menu == DSPY_ACC_CONF)){
		odr = bmi160dev.accel_cfg.odr;
		rango = (bmi160dev.accel_cfg.range>>2)+1;
		ctrl = 0;
		fprintf(stdout, "ACC: ODR : %d     Rango : %d\n", odr, rango);
	}
	else if (ctrl&&(menu == DSPY_GYR_CONF)){
		rango = 5-bmi160dev.gyro_cfg.range;
		odr = bmi160dev.gyro_cfg.odr -1;
		ctrl = 0;
		fprintf(stdout, "GYR: ODR : %d     Rango : %d\n", odr, rango);
	}

	if ((pos >= 292)&&(pos <= 320)){
		fprintf(stdout, "Volver\n");
		(menu == DSPY_ACC_CONF)?print_menu_acc():print_menu_gyr();
		ctrl = 1;
		s_volver = 1;
	}
	else if ((pos >= 44)&&(pos <= (44+31))){ //arriba odr
		if(odr<12)
//			s_nuevo_odr = ++odr;
			lcd_int.seleccionar_odr = ++odr;
		fprintf(stdout, "Subir odr : %d\n", odr);
		//print_menu_conf();
	}
	else if ((pos >= 84)&&(pos <= (84+31))){ //abajo odr
		if(odr>5)
			lcd_int.seleccionar_odr = --odr;
//			snuevo_odr = --odr;
		fprintf(stdout, "Bajar odr : %d\n", odr);
		//print_menu_conf();
	}
	else if ((pos >= 204)&&(pos <= (204+31))){ //arriba rango
		if(rango<4)
			lcd_int.seleccionar_rango = ++rango;
//			nuevo_rango = ++rango;
		fprintf(stdout, "Subir rango : %d\n", rango);
		print_menu_conf();
	}
	else if ((pos >= 244)&&(pos <= (244+31))){ //abajo rango
		if(rango>1)
			lcd_int.seleccionar_rango = --rango;
//			s_nuevo_rango = --rango;
		fprintf(stdout, "Bajar rango : %d\n", rango);
		print_menu_conf();
	}
	else if ((pos >= 147)&&(pos <= (147+30))){
		fprintf(stdout, "Actualizar configuración: odr = %d, rango = %d\n", odr, rango);
		s_config_start = (odr<<4)|(rango);
		fprintf(stdout, "s_config_start = %02X\n", s_config_start);
		print_menu_conf();
	}
	else{
		fprintf(stdout, "Opción inválida\n");
	}
}

void print_menu_mag(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*                 MENU MAG                 *\n");
	fprintf(stdout, "*    IZQ. GYR                              *\n");
	fprintf(stdout, "*    DER. PPAL                             *\n");
	fprintf(stdout, "*    3. Calibrar                           *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_mag(int16_t pos){
	if ((pos >= 5)&&(pos <= (82))){
		fprintf(stdout, "Calibrar\n");
		s_calib_start = 1;
		print_menu_mag();
	}
	else if (pos == DER){
		fprintf(stdout, "Desplazamiento DER: gyr\n");
		s_gyr = 1;
		print_menu_gyr();
	}
	else if (pos == IZQ){
		fprintf(stdout, "Desplazamiento IZQ: ppal\n");
		s_ppal = 1;
		print_menu_ppal();
	}
	else{
		fprintf(stdout, "Opción inválida\n");
	}
}

void print_menu_ppal(void){
	fprintf(stdout, "********************************************\n");
	fprintf(stdout, "*                 MENU PPAL                *\n");
	fprintf(stdout, "*    DER. acelerómetro                     *\n");
	fprintf(stdout, "*    IZQ. magnetómetro                     *\n");
	fprintf(stdout, "********************************************\n");
}

void accion_menu_ppal(int16_t pos){
	if (pos == IZQ/*(pos >= 218)&&(pos <= (218+95))*/){
		fprintf(stdout, "Desplazamiento IZQ : acc\n");
		s_acc = 1;
		print_menu_acc();
	}
	else if (pos == DER/*(pos >= 218)&&(pos <= (218+95))*/){
		fprintf(stdout, "Desplazamiento DER: mag\n");
		s_mag = 1;
		print_menu_mag();
	}/*
	else if (pos < 60){
		fprintf(stdout, "Disminuir frecuencia pantalla : %i\n", clk_freq);
		s_menos = 1;
	}
	else if (pos > 250){
		fprintf(stdout, "Aumentar frecuencia pantalla : %i\n", clk_freq);
		s_mas = 1;
	}*/
	else{
		fprintf(stdout, "Opción inválida\n");
	}
}



/****************************************************************************************************************/
/*										Gestión ficheros de datos												*/
/****************************************************************************************************************/

void actualizar_fichero(uint8_t sensor, uint8_t tipo){
	static int linea_acc = 1;
	static int linea_gyr = 1;
	static int linea_mag = 1;

	switch (sensor){
	case BMI160_ACCEL_SEL:
		if(linea_acc < MAX_LINEAS_REG){
			if(tipo == NUEVA_MEDIDA)
				fprintf(accptr, "%i\t%u\t%i\t%i\t%i\t%i\n", linea_acc, bmi160_accel.sensortime, bmi160_accel.x, bmi160_accel.y, bmi160_accel.z, abs(get_sensitivity(bmi160dev, BMI160_ACCEL_SEL)));
			else if(tipo == OFFSETS)
				fprintf(accptr, "%i\t%u\t%i\t%i\t%i\t%i\tDispositivo calibrado --> offsets(x, y, z): %i\t%i\t%i\n", linea_acc, bmi160_accel.sensortime, bmi160_accel.x, bmi160_accel.y, bmi160_accel.z, abs(get_sensitivity(bmi160dev, BMI160_ACCEL_SEL)),offsets.off_acc_x, offsets.off_acc_y, offsets.off_acc_z);
			linea_acc++;
		}
		else{
			fclose(accptr);
			accptr = fopen("./registros/registro_acc.txt", "w");
			fprintf(accptr, "Índice\tTiempo\tEje X\tEje Y\tEje Z\tSensibilidad\tComentarios\n");
			linea_acc = 1;
		}
		break;
	case BMI160_GYRO_SEL:
		if(linea_gyr < MAX_LINEAS_REG){
			if(tipo == NUEVA_MEDIDA)
				fprintf(gyrptr, "%i\t%u\t%i\t%i\t%i\t%i\n", linea_gyr, bmi160_gyro.sensortime, bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z, abs(get_sensitivity(bmi160dev, BMI160_GYRO_SEL)));
			else if(tipo == OFFSETS)
				fprintf(gyrptr, "%i\t%u\t%i\t%i\t%i\t%i\tDispositivo calibrado --> offsets(x, y, z): %i\t%i\t%i\n", linea_gyr, bmi160_gyro.sensortime, bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z, abs(get_sensitivity(bmi160dev, BMI160_GYRO_SEL)),offsets.off_gyro_x, offsets.off_gyro_y, offsets.off_gyro_z);
			linea_gyr++;
		}
		else{
			fclose(gyrptr);
			gyrptr = fopen("./registros/registro_gyr.txt", "w");
			fprintf(gyrptr, "Índice\tTiempo\tEje X\tEje Y\tEje Z\tSensibilidad\tComentarios\n");
			linea_gyr = 1;
		}
		break;

	case BMM150_MAG_SEL:
		if(linea_mag < MAX_LINEAS_REG){
			if(tipo == NUEVA_MEDIDA){
				fprintf(magptr, "%i\t%i\t%i\t%i\t", linea_mag, (int) mag_data[0], (int) mag_data[1], (int) mag_data[2]);
				fprintf(magptr, "%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\n", mg_comp.x, mg_comp.y, mg_comp.z, mg_comp_iron[0], mg_comp_iron[1]);
			}
			if(tipo == OFFSETS){
				fprintf(magptr, "%i\t\t%i\t\t%i\t\t%i\t", linea_mag, (int) mag_data[0], (int) mag_data[1], (int) mag_data[2]);
				fprintf(magptr, "%.2f\t%.2f\t%.2f\t%.2f\t\t%.2f\tNueva calibración: alfa = %.4f\tbeta = %.4f\tsigma = %.4f\n", mg_comp.x, mg_comp.y, mg_comp.z, mg_comp_iron[0], mg_comp_iron[1], alpha, beta, sigma);
			}
			linea_mag++;
		}
		else{
			fclose(magptr);
			accptr = fopen("./registros/registro_acc.txt", "w");
			fprintf(magptr, "Índice\tEje X\tEje Y\tEje Z\tX(RH)\tY(RH)\tZ(RH)\tX(IRON)\t\tY(IRON)\n");
			linea_mag = 1;
		}
		break;
	}
}

int iniciar_ficheros(void){
	int rslt = 0;
	accptr = fopen("./registros/registro_acc.txt", "w"); //1
	if(accptr==NULL)
		rslt = 1;
	gyrptr = fopen("./registros/registro_gyr.txt", "w"); //2
	if(gyrptr==NULL)
		rslt = 2;
	magptr = fopen("./registros/registro_mag.txt", "w"); //3
	if(magptr==NULL)
		rslt = 3;
	fprintf(accptr, "Índice\tTiempo\tEje X\tEje Y\tEje Z\tSensibilidad\tComentarios\n");
	fprintf(gyrptr, "Índice\tTiempo\tEje X\tEje Y\tEje Z\tSensibilidad\tComentarios\n");
	fprintf(magptr, "Índice\tEje X\tEje Y\tEje Z\tX(RH)\tY(RH)\tZ(RH)\tX(IRON)\t\tY(IRON)\n");

	return rslt;
}


