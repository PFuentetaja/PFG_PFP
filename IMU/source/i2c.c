#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include "i2c.h"

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

int open_i2c_bus(uint8_t dev_addr);

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
	uint8_t buff[MAX_SIZE_BUFFER];

	if (ioctl(fd_i2c, I2C_SLAVE, dev_addr) < 0) {
	  /* ERROR HANDLING; you can check errno to see what went wrong */
		printf("Error al intentar contactar con el dispositivo i2c");
	}

	buff[0]=reg_addr;
	//printf("buff[0..3]: 0x%02X  0x%02X  0x%02X 0x%02X\n", buff[0], buff[1], buff[2], buff[3] );
	for(uint8_t i = 0; i<len; i++){
		buff[i+1]=data[i];
	}

	//printf("buff[0..3]: 0x%02X  0x%02X  0x%02X 0x%02X\n", buff[0], buff[1], buff[2], buff[3] );

	if(write(fd_i2c,buff,len+1) != len+1){ // +1 porque len sólo se refiere al tamaño de lo que se quiere escribir, pero antes hay que escribir también la dirección del registro!!
		printf("Escritura incorrecta\n");
		return -1;
	}
	//printf("Escritura correcta\n");

	return 0;
}


int8_t i2c_read_aux(int fd, int8_t reg, uint8_t size, void *array){ //int fd, int8_t reg, uint8_t size, void *array
	uint8_t buff[MAX_SIZE_BUFFER];

	buff[0]=reg;

    if(write(fd_i2c,buff,1) != 1) {
    	printf("Escritura a registro incorrecta\n");
    }
    if(read(fd_i2c,buff,size) != size) {
        printf("Lectura incorrecta: numero de bytes leidos no coincidentes\n");
        return -1;
    }
    memcpy(array,&buff[0],size);
    return size;
}

int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){

	int rst = 0;

	if (ioctl(fd_i2c, I2C_SLAVE, dev_addr) < 0) {
		  /* ERROR HANDLING; you can check errno to see what went wrong */
			printf("Error al intentar contactar con el dispositivo i2c");
	}
	if(i2c_read_aux(fd_i2c, reg_addr, len, data)!=len){
		printf("Lectura incorrecta\n");
		rst = -12;
	}
	return rst;
}

int open_i2c_bus(uint8_t dev_addr){

	char *filename = "/dev/i2c-1";
	int fd = open(filename, O_RDWR);
	if (fd < 0) {
	  /* ERROR HANDLING; you can check errno to see what went wrong */
	  printf( "Error al intentar abir el fichero i2c");
	}

//	if (ioctl(fd, I2C_SLAVE, dev_addr) < 0) {
//	  /* ERROR HANDLING; you can check errno to see what went wrong */
//		printf("Error al intentar contactar con el dispositivo i2c");
//	}
	return fd;
}


