// ============================================================================
//  protocolo.h  --  Cosas COMPARTIDAS entre el EKF y los publicadores de prueba
//
//  FORMATO DE LOS MENSAJES (texto, un mensaje zmq = una línea):
//
//      "<topico> <timestamp> <valor(es)...>"
//
//      giro   t delta            delta  : giro de ruedas delanteras [rad], + = izquierda
//      vel    t v                v      : velocidad [m/s]
//      rumbo  t psi              psi    : rumbo del INS, 0 = ESTE, antihorario + [rad]
//                                       (cualquier rango: [0,2pi) o [-pi,pi])
//      pos    t x y              x: este [m], y: norte [m]
//      gt     t x y psi v        ground truth (SOLO para pruebas), psi con la misma convención
//
//  timestamp : instante en que se MIDIÓ el dato (no cuando se publicó), en segundos
//              (epoch, double). Es lo que permite ordenar eventos que llegan tarde.
// ============================================================================
#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <math.h>

static const double PI = 3.14159265358979323846;

// ---- Vehículo ----
static const double L        = 1.5;   // distancia entre ejes [m]
static const double Y_RUEDA  = -0.5;  // posición lateral de la rueda del sensor de velocidad
                                      // respecto del centro del eje trasero [m] (+ izq, - der).
                                      // Poner 0 si la velocidad ya viene referida al centro del eje.
static const double ANTENA_X = 0.5;   // sensor de posición: adelante del eje trasero [m]
static const double ANTENA_Y = 0.2;   // sensor de posición: a la izquierda del centro [m]
                                      // (poner ambos en 0 si la posición ya es la del eje trasero)

// ---- Ruido de los sensores (el EKF lo usa como R; los publicadores para simular) ----
/* 
 *  REPASO de nuestro ruidos de los sensores acá:
 *
 *	Constante	Valor	
 *	SIGMA_DELTA	0.02	giro de ruedas: ±0.02 rad (≈1.1°)
 *	SIGMA_VW	0.10	velocidad: ±0.10 m/s
 *	SIGMA_INS	0.02	rumbo: ±0.02 rad (≈1.1°)
 *	SIGMA_POS	0.50	posición: ±0.5 m por eje
 *	68 % de las lecturas caen dentro de ±σ del valor real, y alrededor del 
 *      95 % dentro de ±2σ.
 *	Rango: cualquier valor mayor que 0, sin tope. Un GPS malo puede tener 
 *      SIGMA_POS = 5.0. No ponga 0, porque el filtro asumiría un sensor perfecto.
 *	Cuanto mayor menos confía el filtro en ese sensor y más se apoya en el
 *      modelo y en los demás sensores. Cuanto menor, más lo sigue.
*/
static const double SIGMA_DELTA = 0.02;  // giro de ruedas [rad]
static const double SIGMA_VW    = 0.10;  // velocidad [m/s]
static const double SIGMA_INS   = 0.02;  // rumbo del INS [rad]
static const double SIGMA_POS   = 0.10;  // posición, por eje [m]

// ---- Red ----
static const char *const HOST = "127.0.0.1";
enum { PUERTO_GIRO = 5561, PUERTO_VEL = 5562, PUERTO_RUMBO = 5563,
       PUERTO_POS  = 5564, PUERTO_GT  = 5565 };

#define TOP_GIRO  "giro"
#define TOP_VEL   "vel"
#define TOP_RUMBO "rumbo"
#define TOP_POS   "pos"
#define TOP_GT    "gt"

// ---- Utilidades de ángulos ----
static inline double wrap_pi(double a)            // a [-pi, pi]
{
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

// Rumbo del INS (0 = este, antihorario +) -> psi interno del filtro. Misma convención: solo normaliza.
static inline double rumbo_a_psi(double rumbo)
{
    return wrap_pi(rumbo);
}

// psi interno -> rumbo en [0, 2pi)   (lo usan los publicadores de prueba)
static inline double psi_a_rumbo(double psi)
{
    double a = fmod(psi, 2.0 * PI);
    if (a < 0) a += 2.0 * PI;
    return a;
}

#endif
