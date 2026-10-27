
/*
 * Aplicacion de tacometro
 *
 * COMANDOS POSIBLES por SERIAL
 * 0x1A : reporta distancia (alta frecuencia > 100hz )
 * 0x1B : reporta velocidad (baja frecuencia: aprox entre 4hz a 10hz) 
 * 0b0XXXXXXX : establece distancia recorrida por flanco de taco
 *        Este ultimo comando estable cuanta distancia se recorrió por
 *        cada flanco de taco de la rueda. EN mm/s.
 *        Para calibrar: recorrer con el tractor X mts y contar Y flancos.
 *                       X * 1000 / Y = Z mm recorrido por flanco.
 *
 * ATENCION: la autocalibracion por tacos, y stop de calibracion, funciona si 
 +           el sistema inicia luego de un reset en modo reporta velocidad. 
 *           Si es asi, se cuentan tacos hasta CANT_TACOS_AUTO_CALIBRACION 
 *           y se detiene la calibracion.
 *           Si se inicia en modo reporta distancia se continuará la 
 *           calibracion hasta que se ingrese al modo reporta velocidad.
 */

#include <util/delay.h>
#include <avr/interrupt.h>
#include "eeprom.h"
#include "gpio.h"
#include "serial.h"
#include "ultrasound.h"
#include "tacometro.h"
#include "velocidad.h"
#include "timer2.h"

#define CMD_REPORTAR_DISTANCIA 0x1A
#define CMD_REPORTAR_VELOCIDAD 0x1B
#define SET_DISTANCIA_X_TACO_CMD 7		

#define CMD_MODE_MASK 0x80
#define CMD_MODE 0x80

#define CANT_TACOS_AUTO_CALIBRACION 15

/* sample_n son la cantidad de tacos impresos. Definida en velocidad.c */
extern uint16_t sample_n;

void procesar_comando(uint8_t comando);

void main()
{
	uint16_t val;
	char comando = -1;

	int estado_anterior = -1;

	int n = 0;

	serial_init();
	timer2_init();
	sei();

	/* recuperamos el modo de funcionamiento */
	comando = eeprom_read(0);	
	if ((comando == 0x1A) || (comando == 0x1B)) {
		tacometro_set_mode(comando);
	}

	/* recuperamos la calibracion */
	comando = eeprom_read(1);	
	velocidad_set_mm_x_taco(comando);

	while (1) {

		if (serial_rx_data()) {		// obtuvimos un comando
			serial_cli_rx_data();
			comando = serial_get_char();
			procesar_comando(comando);
		}

		/* obtener una distancia del ultrasound, calibramos, detectamos */
 		val = (uint16_t) ultrasound_get(2, 3);	// en tacometro D3: echo, D2: trigger
 		//val = (uint16_t) ultrasound_get(6, 5);	// en giro ruedas

		if (sample_n <= CANT_TACOS_AUTO_CALIBRACION)
			calibrar((uint16_t) val);

		estado = detectar_taco((uint16_t) val, 4);

		if (tacometro_get_mode() == CMD_REPORTAR_VELOCIDAD) {
			if (estado != estado_anterior) {
				velocidad_print();
				estado_anterior = estado;
			}
		} else {
			serial_put_int(val, 4);
			serial_put_str(" \n\r");
		}

		_delay_ms(10);
	}
}


void procesar_comando(uint8_t comando)
{
		serial_put_str("COMANDO \n\r");
		serial_put_int(comando, 4);
		serial_put_str(" \n\r");
		_delay_ms(2000);

		switch (comando) {
		case CMD_REPORTAR_DISTANCIA:
		case CMD_REPORTAR_VELOCIDAD:
			serial_put_str("COMANDO \n\r");
			serial_put_int(comando, 4);
			serial_put_str(" \n\r");
			_delay_ms(2000);
			tacometro_set_mode(comando);
			eeprom_write(0, comando);
			break;
		default:
			if ((comando & CMD_MODE_MASK) == 0) {
				comando = comando & 0x7F;
				velocidad_set_mm_x_taco(comando);
				eeprom_write(1, comando);
			}
			break;
		}
}

