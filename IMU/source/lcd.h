#ifndef LCD_H
#define LCD_H

#define FILAS     240
#define TOTAL_F   263
#define COLUMNAS  320
#define TOTAL_C     408


#define SPENB_pin 7
#define SPDA_pin  25
#define SPCK_pin  8

#define RSTB_pin  26
#define CLKIN_pin 12
#define DE_pin    19
#define HSYNC_pin 4
#define VSYNC_pin 17
#define DIN0_pin    17
#define DIN1_pin    27
#define DIN2_pin    22
#define DIN3_pin    5
#define DIN4_pin    20
#define DIN5_pin    6
#define DIN6_pin    21
#define DIN7_pin    13
#define EDGE_detect_pin 6

#define CLKIN_INI_FREQ 1700000

//Tactil
#define nINT_pin  20
#define nRST_pin  0

#define BAJADA 1
#define SUBIDA 2
#define TIMEOUT 3

#define IZQ 1000
#define DER 2000

extern uint8_t flanco_ts;
void cambio_nINT(int gpio, int level, uint32_t tick);



#define R00    0x00
#define R01    0x01
#define R02    0x02
#define R03    0x03
#define R04    0x04
#define R05    0x05
#define R07    0x07
#define R08    0x08
#define R09    0x09
#define R0B    0x0B
#define R0C    0x0C
#define R0D    0x0D
#define R0E    0x0E
#define R0F    0x0F
#define R10    0x10
#define R11    0x11
#define R12    0x12
#define R1D    0x1D
#define R1E    0x1E
#define R1F    0x1F

#define DE_8_mode  1
#define HV_mode    2
#define DE_24_mode 3


#define R 0
#define W 1

#define INVALID 0
#define VALID   1

/*
#define ALT_NUM  11
#define ANCH_NUM  8
#define ANCH_COMA  4*/

#define ANCH_FONT16 9
#define ALT_FONT16  16
#define ANCH_COMA_FONT16 4
#define ESPACIO_FONT16 4

#define MAS 10
#define MENOS 11
#define NADA 12
#define COMA 13
#define ESPACIO 20

#define SELECCIONAR   1
#define DESELECCIONAR 0

uint8_t flanco_bajada;

extern unsigned char buffer[FILAS][COLUMNAS*3];
extern int clkin_freq;
extern uint8_t falling_edge;
extern int posicion_lcd;

struct coord_pixel{
	uint16_t x;
	uint16_t y;
};


extern struct coord_pixel selec_colum_1;
extern struct coord_pixel selec_colum_2;
extern struct coord_pixel selec_colum_3;
extern struct coord_pixel aviso_off;
extern struct coord_pixel aviso_conf;
extern struct coord_pixel aviso_mag;

extern const unsigned char aguja[];

#define EXITO_CONF 1
#define ERROR_CONF 2
#define EXITO_OFF  3
#define ERROR_OFF  4
#define CALIB_MAG  5

#define DSPY_PPAL      1
#define DSPY_ACC       2
#define DSPY_ACC_CONF  3
#define DSPY_ACC_OFF   4
#define DSPY_GYR       5
#define DSPY_GYR_OFF   6
#define DSPY_GYR_CONF  7
#define DSPY_MAG  	   8

struct interrupciones{
	uint8_t nueva_medida;
	uint8_t cambio_unidades_acc;
	uint8_t cambio_unidades_gyr;
	uint8_t cambio_unidades_ang;
	uint8_t cambio_unidades_off;
	uint8_t seleccionar_odr;
	uint8_t seleccionar_rango;
	uint8_t exito;
	uint8_t cambiar_pantalla;
	uint8_t actualizar_off;
	uint8_t actualizar_freq;
};

int lcd_spi_write (uint8_t add, uint8_t *data_write);

void lcd_spi_read(uint8_t add, uint8_t *data_read);

void lcd_spi_init(void);

void lcd_init(void);

void init_ts(void);

void send_img(int mode);

void escribe_num(int *pos_x, int pos_y, int num);

void escribir_entero(int num, int longitud, uint8_t signo, struct coord_pixel coord);

void escribir_double(double num, int signo, struct coord_pixel coord);

void seleccion(struct coord_pixel posicion, uint8_t seleccionado);

void cambiar_seleccion(uint8_t nuevo, uint8_t actual);

void notificar(uint8_t mensaje);

void invertir_pixels(struct coord_pixel coord, int ancho, int largo);

void escribir_rectangulo(struct coord_pixel coord, int ancho, int largo, uint8_t color);

#endif
