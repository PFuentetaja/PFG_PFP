#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pigpio.h"
#include "lcd.h"
#include "nums_font.h"
#include <math.h>
#include <time.h>




uint8_t flanco_bajada = 0;
unsigned char buffer[FILAS][COLUMNAS*3];
int clkin_freq = CLKIN_INI_FREQ;
uint8_t dir_fall;

uint16_t cnt_invalid = 0;
uint16_t cnt_valid = 0;
uint8_t valor_ini = 0;
uint8_t valor_sec = 0;


int lcd_spi_write (uint8_t add, uint8_t *data_write){

	int rslt = 0;
	uint16_t msg;

	msg = ((uint16_t)add<<10)|(1<<9)|((uint16_t)*data_write);
	//fprintf(stderr, "add = 0x%X   data = 0x%X   --->   msg = 0x%X\n", add, *data_write, msg);

	gpioWrite(SPENB_pin, 0); 							//CS -> LOW;

	for (int i = 0; i<16; i++){
		gpioWrite(SPCK_pin, 0);							//CLK LOW
		gpioWrite(SPDA_pin, (msg&(1<<(15-i)))>>(15-i)); //Actualizar SDA (dirección + datos) en flanco de bajada
		gpioDelay(10); 									//Semiciclo negativo 10us
		gpioWrite(SPCK_pin, 1);							//CLK HIGH
		gpioDelay(10); 									//Semiciclo positivo 10us
	}

	gpioWrite(SPENB_pin, 1); 							//CS -> HIGH;
	gpioDelay(10);
	return rslt;
}

void lcd_spi_read(uint8_t add, uint8_t *data_read){
	uint8_t bit;
	uint8_t msg = (add<<2);
	uint8_t data_read_aux=0;

	gpioWrite(SPENB_pin, 0); 							//CS -> LOW;

	for (int i = 0; i<8; i++){
		gpioWrite(SPCK_pin, 0);							//CLK LOW
		gpioWrite(SPDA_pin, (msg&(1<<(7-i)))>>(7-i));	//Acualizar SDA (dirección) en flanco de bajada
		gpioDelay(10);									//Semiciclo negativo 10 us
		gpioWrite(SPCK_pin, 1);							//CLK HIGH
		gpioDelay(10);									//Semiciclo positivo 10 us
	}

	gpioSetMode(SPDA_pin, PI_INPUT);					//Configurar SDA como entrada para lectura

	for (int i = 0; i<8; i++){

			gpioWrite(SPCK_pin, 0);						//CLK LOW
			gpioDelay(10);								//Semiciclo negativo 10 us
			gpioWrite(SPCK_pin, 1);						//CLK HIGH
			bit = gpioRead(SPDA_pin);					//Leer SDA en flanco de subida
			//fprintf(stderr, "bits = %d \t", bit);
			data_read_aux |= (bit<<(7-i));
			gpioDelay(10);								//Semiciclo positivo 10 us
			/*fprintf(stderr, "data_read_aux = ");
			for (int j = 0; j<8; j++){
				fprintf(stderr, "%d",(data_read_aux&(1<<(7-j)))>>(7-j));
			}
			fprintf(stderr, "\n");*/
	}
//	fprintf(stderr, "\n");
	gpioWrite(SPENB_pin, 1); 							//CS -> HIGH;
	gpioSetMode(SPDA_pin, PI_OUTPUT);					//SDA como salida para escritura
	*data_read=data_read_aux;
	gpioDelay(10);// 2 us
}

void lcd_spi_init(void){

	gpioSetMode(SPENB_pin, PI_OUTPUT);
	gpioSetMode(SPCK_pin, PI_OUTPUT);
	gpioSetMode(SPDA_pin, PI_OUTPUT);

	gpioWrite(SPENB_pin, 1);
	gpioWrite(SPCK_pin, 1);
}

void lcd_init(void){

	flanco_bajada = 0;
	gpioSetMode(DIN0_pin, PI_OUTPUT);
	gpioSetMode(DIN1_pin, PI_OUTPUT);
	gpioSetMode(DIN2_pin, PI_OUTPUT);
	gpioSetMode(DIN3_pin, PI_OUTPUT);
	gpioSetMode(DIN4_pin, PI_OUTPUT);
	gpioSetMode(DIN5_pin, PI_OUTPUT);
	gpioSetMode(DIN6_pin, PI_OUTPUT);
	gpioSetMode(DIN7_pin, PI_OUTPUT);

	gpioSetMode(VSYNC_pin, PI_OUTPUT);
	gpioSetMode(HSYNC_pin, PI_OUTPUT);
	gpioWrite(VSYNC_pin, 1);
	gpioWrite(HSYNC_pin, 1);

	gpioSetMode(RSTB_pin, PI_OUTPUT);
	gpioSetMode(DE_pin, PI_OUTPUT);
	gpioSetMode(CLKIN_pin, PI_OUTPUT);
	gpioSetMode(CLKIN_pin, PI_ALT0);  //Modo PWM


	gpioHardwarePWM(CLKIN_pin, CLKIN_INI_FREQ, 500000);
	//gpioHardwareClock(CLKIN_pin, 27000000);
	gpioWrite(RSTB_pin, 0);
	gpioWrite(DE_pin, 0);
	lcd_spi_init();

	gpioDelay(100); // 100 us

	gpioWrite(RSTB_pin, 1);

	gpioSetMode(EDGE_detect_pin, PI_INPUT);

	for(int i = 0; i<10; i++){
		for(int j = 0; j<30; j++){
			buffer[i][j]=0x00;
		}
	}
}



int flanco_bajada_aux(void);

void send_img_line_DE_24(unsigned char line[], uint8_t valid){
	while(!gpioRead(EDGE_detect_pin));
	gpioWrite(DE_pin, valid);

	if(valid){
		gpioWrite(DIN0_pin, (line[0]!=0)?1:0);

		for(int i=1; i<COLUMNAS; i++){
			while(!gpioRead(EDGE_detect_pin));
			gpioWrite(DIN0_pin, (line[3*i]!=0)?1:0);
		}

		while(!gpioRead(EDGE_detect_pin));
		gpioWrite(DE_pin, 0);

		for(int i=1; i<TOTAL_C-COLUMNAS; i++){
			while(!gpioRead(EDGE_detect_pin));
			gpioWrite(DIN0_pin, 0);
		}
	}
	else{
		gpioWrite(DIN0_pin, 0);
		for(int i=1; i<TOTAL_C; i++){
			while(!gpioRead(EDGE_detect_pin));
			gpioWrite(DIN0_pin, 0);
		}
	}
}

void send_img_line_DE(unsigned char line[], uint8_t valid){

	for(int i=0; i<1716; i++){

		//detectar flanco de bajada
		/*while(!gpioRead(CLKIN_pin)){		//esperar hasta que este a nivel alto
		}
		while(gpioRead(CLKIN_pin)) {		//esperar hasta que baje
		}*/

		//opcion 2
		while(!(falling_edge/*flanco*/ )){	//falling edge generado en otro hilo. Espera a que se ponga a 1
			/*valor_sec = gpioRead(4);
			falling_edge = valor_ini&(!valor_sec);
			valor_ini = valor_sec;*/
		}
		falling_edge = 0;

		if((i<960)&valid){

			//Siguente DIN[7:0]
			gpioWrite(DIN7_pin, (line[i]&(1<<7))>>7);
			gpioWrite(DIN6_pin, (line[i]&(1<<6))>>6);
			gpioWrite(DIN5_pin, (line[i]&(1<<5))>>5);
			gpioWrite(DIN4_pin, (line[i]&(1<<4))>>4);
			gpioWrite(DIN3_pin, (line[i]&(1<<3))>>3);
			gpioWrite(DIN2_pin, (line[i]&(1<<2))>>2);
			gpioWrite(DIN1_pin, (line[i]&(1<<1))>>1);
			gpioWrite(DIN0_pin, (line[i]&(1<<0))>>0);
			//cnt_valid++;

			if(i==0)
				gpioWrite(DE_pin, 1);

		}//Fin en 959

		else if((i==960)&valid){	//DE a nivel bajo. Acaba la transmision de la linea

			gpioWrite(DE_pin, 0);
			//cnt_invalid++;
		}

		else{
			//cnt_invalid++;
		}
	}//Fin
	//if(primera_vez==1)
		//printf("ciclos invalid: %d\t ciclos valid: %d\t ciclos totales: %d\n", cnt_invalid, cnt_valid, cnt_valid+cnt_invalid); //Va bien
}

void send_img_line_HV(unsigned char line[], uint8_t valid){

	uint16_t cnt_invalid = 0;
	uint16_t cnt_valid = 0;

	gpioWrite(HSYNC_pin, 0);

	//Bucle
	for(int i=0; i<1716; i++){

		//detectar flanco de bajada
		/*while(!gpioRead(CLKIN_pin)){	//esperar hasta que este a nivel bajo
		}
		while(gpioRead(CLKIN_pin)) {	//esperar hasta que suba
		}*/

		//while(!(falling_edge));
		/*flanco_bajada = 0;
		while(!flanco_bajada);
		flanco_bajada = 0;*/

		if(i<70){
			if(i==3)
				gpioWrite(HSYNC_pin, 1);
			cnt_invalid++;
		}
		else if((i<1030)&valid){

			//Siguente DIN[7:0]
			gpioWrite(DIN7_pin, (line[i]&(1<<7))>>7);
			gpioWrite(DIN6_pin, (line[i]&(1<<6))>>6);
			gpioWrite(DIN5_pin, (line[i]&(1<<5))>>5);
			gpioWrite(DIN4_pin, (line[i]&(1<<4))>>4);
			gpioWrite(DIN3_pin, (line[i]&(1<<3))>>3);
			gpioWrite(DIN2_pin, (line[i]&(1<<2))>>2);
			gpioWrite(DIN1_pin, (line[i]&(1<<1))>>1);
			gpioWrite(DIN0_pin, (line[i]&(1<<0))>>0);
			cnt_valid++;

		}//Fin en 959

		else{
			cnt_invalid++;
		}

	}
	 //que devuelva cnt para comprobar que todo ha ido bien
	gpioWrite(HSYNC_pin, 0);

	//Fin
	//if(primera_vez==1)
		//printf("ciclos invalid: %d\t ciclos valid: %d\t ciclos totales: %d\n", cnt_invalid, cnt_valid, cnt_valid+cnt_invalid); //Va bien
}

void send_img(int mode){

	switch (mode) {

	case DE_24_mode:
		for(int i= 0; i<TOTAL_F; i++){
			if(i<TOTAL_F-FILAS)
				send_img_line_DE_24(0, INVALID);
			else
				send_img_line_DE_24(buffer[i-(TOTAL_F-FILAS)], VALID);
		}
	break;

	case DE_8_mode:
		gpioWrite(DE_pin, 0);
//		primera_vez=1;
		for(int i= 0; i<TOTAL_F; i++){
			if(i<TOTAL_F-FILAS){
				send_img_line_DE(0, INVALID);
//				primera_vez=0;
			}
			else{
//				primera_vez++;
				send_img_line_DE(buffer[i-(TOTAL_F-FILAS)], VALID);
			}
		}
	break;

	case HV_mode:
		gpioWrite(VSYNC_pin, 0);
		for(int i= 0; i<263; i++){
			if(i<13){
				send_img_line_HV(0, INVALID);
//				primera_vez=0;
			}
			else if(i<(253)){
//				primera_vez++;
				send_img_line_HV(buffer[i-13], VALID);
			}
			else{
				send_img_line_HV(0, INVALID);
			}

		}
		gpioWrite(VSYNC_pin, 1);

	break;

	default:
		printf("Modo de entrada no valido\n");
	}

}

int flanco_bajada_aux(void){
	uint8_t *flanco = 0;
	//uint8_t
	uint8_t a = gpioRead(4);
	uint8_t b = a;
	while(!(*flanco)){
		b=gpioRead(4);
		*flanco=a&(!b);
		a = b;
	}
	return *flanco;//0 si flanco de bajada, 1 si flanco de subida
}


void escribe_num(int *pos_x, int pos_y, int num){
	if((num==COMA)|(num==ESPACIO)){
		for(int i=0; i<ALT_FONT16; i++){
			for(int j=0; j<ANCH_COMA_FONT16; j++){
				buffer[pos_y+i][*pos_x+3*j] = (num==COMA)?nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_COMA_FONT16*i +j]:0xFF;
				buffer[pos_y+i][*pos_x+3*j+1] = (num==COMA)?nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_COMA_FONT16*i +j]:0xFF;
				buffer[pos_y+i][*pos_x+3*j+2] = (num==COMA)?nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_COMA_FONT16*i +j]:0xFF;
			}
		}
		*pos_x = *pos_x + ANCH_COMA_FONT16*3;
	}
	else{
		for(int i=0; i<ALT_FONT16; i++){
			for(int j=0; j<ANCH_FONT16; j++){
				buffer[pos_y+i][*pos_x+3*j] = nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_FONT16*i +j];
				buffer[pos_y+i][*pos_x+3*j+1] = nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_FONT16*i +j];
				buffer[pos_y+i][*pos_x+3*j+2] = nums9x16[ALT_FONT16*ANCH_FONT16*num + ANCH_FONT16*i +j];
			}
		}
		*pos_x = *pos_x + ANCH_FONT16*3;
	}
}


void seleccion(struct coord_pixel posicion, uint8_t seleccionado){
	//fclose(fptr);
	unsigned char buff_aux[15][45];
	int pos_x = posicion.x*3;
	int pos_y = posicion.y;
	FILE *icono;
	icono = (seleccionado)?fopen("media/sel.bmp", "r"):fopen("media/no_sel.bmp", "r");
	if(icono==NULL)
		fprintf(stdout, "el icono no se abre correctamente\n");
//	else
//		fprintf(stdout, "el icono se abre correctamente\n");
	fseek(icono, 0x36, SEEK_SET); // IPORTANTE poner la cabecera en hexadecimal
	for(int i=0; i<15; i++){
		for(int j=0; j<45; j=j+3){
			buff_aux[14-i][j+2]=getc(icono);//B
			buff_aux[14-i][j+1]=getc(icono);//G
			buff_aux[14-i][j]=getc(icono);//R
		}
		getc(icono);//Los archivos .bmp tiene una extensión de línea de multiplos de 4 Bytes. Como 15 pixeles * 3bytes/pixel = 45 Bytes, mete 3 más
		getc(icono);//para llegar a los 48, que si es multiplo de 4. Y estos bytes son arbitrarios, por eso no se tienen en cuenta.
		getc(icono);
	}
	//fprintf(stdout, "Desborda aquí?\n");
	for(int i = 0; i<15; i++){
		for(int j = 0; j<45; j ++){
			buffer[i+pos_y][pos_x+j] = buff_aux[i][j];
		}
	}
	fclose(icono);
}


void cambiar_seleccion(uint8_t nuevo, uint8_t actual){
	struct coord_pixel pos;

	//Primero se deselecciona el antiguo
	if(actual<5){
		pos = selec_colum_3;
		pos.y = selec_colum_3.y + 25*(4 - actual);
	}
	else if(actual<9){
		pos = selec_colum_2;
		pos.y = selec_colum_2.y + 25*(8 - actual);
	}
	else if(actual<13){
		pos = selec_colum_1;
		pos.y = selec_colum_1.y + 25*(12 - actual);
	}
	else
		printf("valor deseleccionado no válido\n");

	seleccion(pos,DESELECCIONAR);

	//Ahora se selecciona el nuevo
	if(nuevo<5){
		pos = selec_colum_3;
		pos.y = selec_colum_3.y + 25*(4 - nuevo);
	}
	else if(nuevo<9){
		pos = selec_colum_2;
		pos.y = selec_colum_2.y + 25*(8 - nuevo);
	}
	else if(nuevo<13){
		pos = selec_colum_1;
		pos.y = selec_colum_1.y + 25*(12 - nuevo);
	}
	else
		printf("valor seleccionado no válido\n");

	seleccion(pos,SELECCIONAR);
}

void notificar(uint8_t mensaje){
	int pos_x, pos_y;
	unsigned char buff_aux[20][260*3];
	FILE *mens_ptr;

	if(mensaje == EXITO_CONF){
		mens_ptr = fopen("media/conf_exito.bmp", "r");
	}
	else if(mensaje == ERROR_CONF){
		mens_ptr = fopen("media/conf_error.bmp", "r");
	}
	else if(mensaje == EXITO_OFF){
		mens_ptr = fopen("media/off_exito.bmp", "r");
	}
	else if(mensaje == ERROR_OFF){
		mens_ptr = fopen("media/off_error.bmp", "r");
	}
	else if(mensaje == CALIB_MAG){
			mens_ptr = fopen("media/mag_calib.bmp", "r");
		}

	fseek(mens_ptr, 0x36, SEEK_SET); // IMPORTANTE poner la cabecera en hexadecimal
	for(int i=0; i<20; i++){
		for(int j=0; j<260*3; j=j+3){
			buff_aux[19-i][j+2]=getc(mens_ptr);//B
			buff_aux[19-i][j+1]=getc(mens_ptr);//G
			buff_aux[19-i][j]=getc(mens_ptr);//R
		}
	}
	if(mensaje == CALIB_MAG){
		pos_x = aviso_mag.x*3;
		pos_y = aviso_mag.y;
	}
	else if((mensaje == EXITO_CONF)||(mensaje == ERROR_CONF)){
		pos_x = aviso_conf.x*3;
		pos_y = aviso_conf.y;
	}
	else{
		pos_x = aviso_off.x*3;
		pos_y = aviso_off.y;
	}

	for(int i = 0; i<20; i++){
		for(int j = 0; j<((mensaje == CALIB_MAG)? 208*3 : 260*3); j ++){
			buffer[i+pos_y][pos_x+j] = buff_aux[i][j];
		}
	}

	fclose(mens_ptr);
}


void escribir_double(double num, int signo, struct coord_pixel coord){

	int pos_x = coord.x*3;
	int pos_y = coord.y;
	int cero = 0;
	int long_ent = 3;
	int num_aux = (int) num;
	int parte_entera = abs(num_aux);
	int parte_decimal = abs((int) (num*100))%100;

	if(signo!=0) //signo
		(num<0)?escribe_num(&pos_x, pos_y, MENOS):escribe_num(&pos_x, pos_y, MAS);

	if(parte_entera<1000){
		if(parte_entera == 0)
			escribe_num(&pos_x, pos_y, 0);
		else{
			for(int i = 0; i<long_ent; i++){ //Parte entera
				num_aux = (parte_entera%abs((pow(10,long_ent-i))))/abs((pow(10,long_ent-1-i)));
				if((num_aux==0)&&(cero==i))
					cero++;
				else
					escribe_num(&pos_x, pos_y, num_aux);
			}
		}//Parte decimal
		escribe_num(&pos_x, pos_y, COMA);
		escribe_num(&pos_x, pos_y, parte_decimal/10);
		escribe_num(&pos_x, pos_y, parte_decimal%10);
		while(cero!=0){
			escribe_num(&pos_x, pos_y, NADA);
			cero--;
		}
	}
	else{
		fprintf(stdout,"número mayor que |999,99|\n");
	}
}

void escribir_entero(int num, int longitud, uint8_t signo, struct coord_pixel coord){

	int pos_x = coord.x*3;
	int pos_y = coord.y;
	int num_aux = 0;
	uint8_t cero = 0;

	if(signo!=0)
		(num<0)?escribe_num(&pos_x, pos_y, MENOS):escribe_num(&pos_x, pos_y, MAS);

	num = abs(num);

	if(num == 0)
		escribe_num(&pos_x, pos_y, 0);
	else{
		for(int i = 0; i<longitud; i++){
			num_aux = (num%abs((pow(10,longitud-i))))/abs((pow(10,longitud-1-i)));
			if((num_aux==0)&&(cero==i))
				cero++;
			else
				escribe_num(&pos_x, pos_y, num_aux);
		}
	}
	while(cero!=0){
		escribe_num(&pos_x, pos_y, NADA);
		cero--;
	}
}


void invertir_pixels(struct coord_pixel coord, int ancho, int largo){ //ancho verticalmente, largo horizontalmente
	int max_largo = 320-coord.x;
	int max_ancho = 240-coord.y;

	if(ancho>max_ancho)
		ancho = max_ancho;
	if(largo>max_largo)
		largo = max_largo;

	for(int i = 0; i<ancho; i++){
		for(int j = 0; j<largo*3; j++){
			buffer[i+coord.y][j+3*coord.x] = ~(buffer[i+coord.y][j+3*coord.x])/*==0)?0xFF:0x00*/;
		}
	}
}

void escribir_rectangulo(struct coord_pixel coord, int ancho, int largo, uint8_t color){ //ancho verticalmente, largo horizontalmente
	int max_largo = 320-coord.x;
	int max_ancho = 240-coord.y;

	if(ancho>max_ancho)
		ancho = max_ancho;
	if(largo>max_largo)
		largo = max_largo;

	for(int i = 0; i<ancho; i++){
		for(int j = 0; j<largo*3; j++){
			buffer[i+coord.y][j+3*coord.x] = color;
		}
	}
}

void cambio_nINT(int gpio, int level, uint32_t tick){
	if((gpio==nINT_pin)&&(level==0)){
		//printf("flanco de bajada del pin %d\n", nINT_pin);
		flanco_ts = BAJADA;
	}
	else if((gpio==nINT_pin)&&(level==2)){
	//	printf("flanco de subida del pin %d\n", nINT_pin);
		flanco_ts = TIMEOUT;
	}
	else if((gpio==nINT_pin)&&(level==1)){
	//	printf("flanco de subida del pin %d\n", nINT_pin);
		flanco_ts = SUBIDA;
	}
}

void init_ts(void){
	gpioSetMode(nRST_pin, PI_OUTPUT);//nRST
	gpioSetMode(nINT_pin, PI_INPUT);//nINT
	//gpioSetPullUpDown(nINT_pin, PI_PUD_OFF);

	gpioWrite(nRST_pin, 0); //nRST ya está a 0
	gpioDelay(10000); // minimo 1ms tras la subida de alimentacion
	gpioWrite(nRST_pin,1);
	gpioDelay(200000); //200 ms para empezar a pasar datos con sentido

	//gpioSetAlertFunc(nINT_pin, cambio_nINT);
}




