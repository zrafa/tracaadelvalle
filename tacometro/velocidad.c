#include <stdint.h>
#include "serial.h"
#include "timer2.h"

uint16_t mm_x_taco;
uint16_t sample_n = 0;

/*
 * COMANDOS POSIBLES
 * 0x1A : reporta distancia (alta frecuencia > 100hz )
 * 0x1B : reporta velocidad (baja frecuencia: aprox entre 4hz a 10hz) 
 * 0b0XXXXXXX : establece distancia recorrida por flanco de taco
 *        Este ultimo comando estable cuanta distancia se recorrió por
 *        cada flanco de taco de la rueda. EN mm/s.
 *        Para calibrar: recorrer con el tractor X mts y contar Y flancos.
 *                       X * 1000 / Y = Z mm recorrido por flanco.
 */

char mode = -1;

void tacometro_set_mode(char m)
{
	mode = m;
}

char tacometro_get_mode(void)
{
	return mode;
}


void velocidad_set_mm_x_taco(uint16_t n)
{
	mm_x_taco = n;
}

uint16_t velocidad(void)
{
	uint16_t mm_x_seg;
	uint32_t elapsed = timer2_get_ticks();
	timer2_set_ticks(0);

	mm_x_seg = (uint16_t) ((1000.0 * (double) mm_x_taco) / (double) elapsed);
	return mm_x_seg;
}

void velocidad_print(void)
{
	uint16_t vel = velocidad();
	uint16_t vel_m_s = vel / 1000;
	uint16_t vel_cm_s = (uint16_t) ((vel % 1000) / 10);
	sample_n++;
	serial_put_int(sample_n, 4);
	serial_put_char(':');
	serial_put_int(vel_m_s, 2);
	serial_put_char('.');
	serial_put_int(vel_cm_s, 2);
	serial_put_str("\n\r");
}

